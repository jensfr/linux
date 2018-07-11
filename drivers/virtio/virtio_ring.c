/* Virtio ring implementation.
 *
 *  Copyright 2007 Rusty Russell IBM Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <linux/virtio.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_config.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/dma-mapping.h>
#include <xen/xen.h>

#ifdef DEBUG
/* For development, we want to crash whenever the ring is screwed. */
#define BAD_RING(_vq, fmt, args...)				\
	do {							\
		dev_err(&(_vq)->vq.vdev->dev,			\
			"%s:"fmt, (_vq)->vq.name, ##args);	\
		BUG();						\
	} while (0)
/* Caller is supposed to guarantee no reentry. */
#define START_USE(_vq)						\
	do {							\
		if ((_vq)->in_use)				\
			panic("%s:in_use = %i\n",		\
			      (_vq)->vq.name, (_vq)->in_use);	\
		(_vq)->in_use = __LINE__;			\
	} while (0)
#define END_USE(_vq) \
	do { BUG_ON(!(_vq)->in_use); (_vq)->in_use = 0; } while(0)
#else
#define BAD_RING(_vq, fmt, args...)				\
	do {							\
		dev_err(&_vq->vq.vdev->dev,			\
			"%s:"fmt, (_vq)->vq.name, ##args);	\
		(_vq)->broken = true;				\
	} while (0)
#define START_USE(vq)
#define END_USE(vq)
#endif

#define _VRING_DESC_F_AVAIL(b)	((__u16)(b) << 7)
#define _VRING_DESC_F_USED(b)	((__u16)(b) << 15)

struct vring_desc_state {
	void *data;			/* Data for callback. */
	struct vring_desc *indir_desc;	/* Indirect descriptor, if any. */
};

struct vring_desc_state_packed {
	void *data;			/* Data for callback. */
	struct vring_packed_desc *indir_desc; /* Indirect descriptor, if any. */
	int num;			/* Descriptor list length. */
	dma_addr_t addr;		/* Buffer DMA addr. */
	u32 len;			/* Buffer length. */
	u16 flags;			/* Descriptor flags. */
	int next;			/* The next desc state. */
};

struct vring_virtqueue {
	struct virtqueue vq;

	/* Is this a packed ring? */
	bool packed;

	/* Can we use weak barriers? */
	bool weak_barriers;

	/* Other side has made a mess, don't try any more. */
	bool broken;

	/* Host supports indirect buffers */
	bool indirect;

	/* Host publishes avail event idx */
	bool event;

	/* Head of free buffer list. */
	unsigned int free_head;
	/* Number we've added since last sync. */
	unsigned int num_added;

	/* Last used index we've seen. */
	u16 last_used_idx;

	union {
		/* Available for split ring */
		struct {
			/* Actual memory layout for this queue. */
			struct vring vring;

			/* Last written value to avail->flags */
			u16 avail_flags_shadow;

			/* Last written value to avail->idx in
			 * guest byte order. */
			u16 avail_idx_shadow;
		};

		/* Available for packed ring */
		struct {
			/* Actual memory layout for this queue. */
			struct vring_packed vring_packed;

			/* Driver ring wrap counter. */
			bool avail_wrap_counter;

			/* Device ring wrap counter. */
			bool used_wrap_counter;

			/* Index of the next avail descriptor. */
			u16 next_avail_idx;

			/* Last written value to driver->flags in
			 * guest byte order. */
			u16 event_flags_shadow;
		};
	};

	/* How to notify other side. FIXME: commonalize hcalls! */
	bool (*notify)(struct virtqueue *vq);

	/* DMA, allocation, and size information */
	bool we_own_ring;
	size_t queue_size_in_bytes;
	dma_addr_t queue_dma_addr;

#ifdef DEBUG
	/* They're supposed to lock for us. */
	unsigned int in_use;

	/* Figure out if their kicks are too delayed. */
	bool last_add_time_valid;
	ktime_t last_add_time;
#endif

	/* Per-descriptor state. */
	union {
		struct vring_desc_state desc_state[1];
		struct vring_desc_state_packed desc_state_packed[1];
	};
};

#define to_vvq(_vq) container_of(_vq, struct vring_virtqueue, vq)

static inline bool virtqueue_use_indirect(struct virtqueue *_vq,
					  unsigned int total_sg)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	/* If the host supports indirect descriptor tables, and we have multiple
	 * buffers, then go indirect. FIXME: tune this threshold */
	return (vq->indirect && total_sg > 1 && vq->vq.num_free);
}

/*
 * Modern virtio devices have feature bits to specify whether they need a
 * quirk and bypass the IOMMU. If not there, just use the DMA API.
 *
 * If there, the interaction between virtio and DMA API is messy.
 *
 * On most systems with virtio, physical addresses match bus addresses,
 * and it doesn't particularly matter whether we use the DMA API.
 *
 * On some systems, including Xen and any system with a physical device
 * that speaks virtio behind a physical IOMMU, we must use the DMA API
 * for virtio DMA to work at all.
 *
 * On other systems, including SPARC and PPC64, virtio-pci devices are
 * enumerated as though they are behind an IOMMU, but the virtio host
 * ignores the IOMMU, so we must either pretend that the IOMMU isn't
 * there or somehow map everything as the identity.
 *
 * For the time being, we preserve historic behavior and bypass the DMA
 * API.
 *
 * TODO: install a per-device DMA ops structure that does the right thing
 * taking into account all the above quirks, and use the DMA API
 * unconditionally on data path.
 */

static bool vring_use_dma_api(struct virtio_device *vdev)
{
	if (!virtio_has_iommu_quirk(vdev))
		return true;

	/* Otherwise, we are left to guess. */
	/*
	 * In theory, it's possible to have a buggy QEMU-supposed
	 * emulated Q35 IOMMU and Xen enabled at the same time.  On
	 * such a configuration, virtio has never worked and will
	 * not work without an even larger kludge.  Instead, enable
	 * the DMA API if we're a Xen guest, which at least allows
	 * all of the sensible Xen configurations to work correctly.
	 */
	if (xen_domain())
		return true;

	return false;
}

/*
 * The DMA ops on various arches are rather gnarly right now, and
 * making all of the arch DMA ops work on the vring device itself
 * is a mess.  For now, we use the parent device for DMA ops.
 */
static inline struct device *vring_dma_dev(const struct vring_virtqueue *vq)
{
	return vq->vq.vdev->dev.parent;
}

/* Map one sg entry. */
static dma_addr_t vring_map_one_sg(const struct vring_virtqueue *vq,
				   struct scatterlist *sg,
				   enum dma_data_direction direction)
{
	if (!vring_use_dma_api(vq->vq.vdev))
		return (dma_addr_t)sg_phys(sg);

	/*
	 * We can't use dma_map_sg, because we don't use scatterlists in
	 * the way it expects (we don't guarantee that the scatterlist
	 * will exist for the lifetime of the mapping).
	 */
	return dma_map_page(vring_dma_dev(vq),
			    sg_page(sg), sg->offset, sg->length,
			    direction);
}

static dma_addr_t vring_map_single(const struct vring_virtqueue *vq,
				   void *cpu_addr, size_t size,
				   enum dma_data_direction direction)
{
	if (!vring_use_dma_api(vq->vq.vdev))
		return (dma_addr_t)virt_to_phys(cpu_addr);

	return dma_map_single(vring_dma_dev(vq),
			      cpu_addr, size, direction);
}

static int vring_mapping_error(const struct vring_virtqueue *vq,
			       dma_addr_t addr)
{
	if (!vring_use_dma_api(vq->vq.vdev))
		return 0;

	return dma_mapping_error(vring_dma_dev(vq), addr);
}

static void vring_unmap_one_split(const struct vring_virtqueue *vq,
				  struct vring_desc *desc)
{
	u16 flags;

	if (!vring_use_dma_api(vq->vq.vdev))
		return;

	flags = virtio16_to_cpu(vq->vq.vdev, desc->flags);

	if (flags & VRING_DESC_F_INDIRECT) {
		dma_unmap_single(vring_dma_dev(vq),
				 virtio64_to_cpu(vq->vq.vdev, desc->addr),
				 virtio32_to_cpu(vq->vq.vdev, desc->len),
				 (flags & VRING_DESC_F_WRITE) ?
				 DMA_FROM_DEVICE : DMA_TO_DEVICE);
	} else {
		dma_unmap_page(vring_dma_dev(vq),
			       virtio64_to_cpu(vq->vq.vdev, desc->addr),
			       virtio32_to_cpu(vq->vq.vdev, desc->len),
			       (flags & VRING_DESC_F_WRITE) ?
			       DMA_FROM_DEVICE : DMA_TO_DEVICE);
	}
}

static struct vring_desc *alloc_indirect_split(struct virtqueue *_vq,
					       unsigned int total_sg,
					       gfp_t gfp)
{
	struct vring_desc *desc;
	unsigned int i;

	/*
	 * We require lowmem mappings for the descriptors because
	 * otherwise virt_to_phys will give us bogus addresses in the
	 * virtqueue.
	 */
	gfp &= ~__GFP_HIGHMEM;

	desc = kmalloc_array(total_sg, sizeof(struct vring_desc), gfp);
	if (!desc)
		return NULL;

	for (i = 0; i < total_sg; i++)
		desc[i].next = cpu_to_virtio16(_vq->vdev, i + 1);
	return desc;
}

static inline int virtqueue_add_split(struct virtqueue *_vq,
				      struct scatterlist *sgs[],
				      unsigned int total_sg,
				      unsigned int out_sgs,
				      unsigned int in_sgs,
				      void *data,
				      void *ctx,
				      gfp_t gfp)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	struct scatterlist *sg;
	struct vring_desc *desc;
	unsigned int i, n, avail, descs_used, uninitialized_var(prev), err_idx;
	int head;
	bool indirect;

	START_USE(vq);

	BUG_ON(data == NULL);
	BUG_ON(ctx && vq->indirect);

	if (unlikely(vq->broken)) {
		END_USE(vq);
		return -EIO;
	}

#ifdef DEBUG
	{
		ktime_t now = ktime_get();

		/* No kick or get, with .1 second between?  Warn. */
		if (vq->last_add_time_valid)
			WARN_ON(ktime_to_ms(ktime_sub(now, vq->last_add_time))
					    > 100);
		vq->last_add_time = now;
		vq->last_add_time_valid = true;
	}
#endif

	BUG_ON(total_sg == 0);

	head = vq->free_head;

	if (virtqueue_use_indirect(_vq, total_sg))
		desc = alloc_indirect_split(_vq, total_sg, gfp);
	else {
		desc = NULL;
		WARN_ON_ONCE(total_sg > vq->vring.num && !vq->indirect);
	}

	if (desc) {
		/* Use a single buffer which doesn't continue */
		indirect = true;
		/* Set up rest to use this indirect table. */
		i = 0;
		descs_used = 1;
	} else {
		indirect = false;
		desc = vq->vring.desc;
		i = head;
		descs_used = total_sg;
	}

	if (vq->vq.num_free < descs_used) {
		pr_debug("Can't add buf len %i - avail = %i\n",
			 descs_used, vq->vq.num_free);
		/* FIXME: for historical reasons, we force a notify here if
		 * there are outgoing parts to the buffer.  Presumably the
		 * host should service the ring ASAP. */
		if (out_sgs)
			vq->notify(&vq->vq);
		if (indirect)
			kfree(desc);
		END_USE(vq);
		return -ENOSPC;
	}

	for (n = 0; n < out_sgs; n++) {
		for (sg = sgs[n]; sg; sg = sg_next(sg)) {
			dma_addr_t addr = vring_map_one_sg(vq, sg, DMA_TO_DEVICE);
			if (vring_mapping_error(vq, addr))
				goto unmap_release;

			desc[i].flags = cpu_to_virtio16(_vq->vdev, VRING_DESC_F_NEXT);
			desc[i].addr = cpu_to_virtio64(_vq->vdev, addr);
			desc[i].len = cpu_to_virtio32(_vq->vdev, sg->length);
			prev = i;
			i = virtio16_to_cpu(_vq->vdev, desc[i].next);
		}
	}
	for (; n < (out_sgs + in_sgs); n++) {
		for (sg = sgs[n]; sg; sg = sg_next(sg)) {
			dma_addr_t addr = vring_map_one_sg(vq, sg, DMA_FROM_DEVICE);
			if (vring_mapping_error(vq, addr))
				goto unmap_release;

			desc[i].flags = cpu_to_virtio16(_vq->vdev, VRING_DESC_F_NEXT | VRING_DESC_F_WRITE);
			desc[i].addr = cpu_to_virtio64(_vq->vdev, addr);
			desc[i].len = cpu_to_virtio32(_vq->vdev, sg->length);
			prev = i;
			i = virtio16_to_cpu(_vq->vdev, desc[i].next);
		}
	}
	/* Last one doesn't continue. */
	desc[prev].flags &= cpu_to_virtio16(_vq->vdev, ~VRING_DESC_F_NEXT);

	if (indirect) {
		/* Now that the indirect table is filled in, map it. */
		dma_addr_t addr = vring_map_single(
			vq, desc, total_sg * sizeof(struct vring_desc),
			DMA_TO_DEVICE);
		if (vring_mapping_error(vq, addr))
			goto unmap_release;

		vq->vring.desc[head].flags = cpu_to_virtio16(_vq->vdev, VRING_DESC_F_INDIRECT);
		vq->vring.desc[head].addr = cpu_to_virtio64(_vq->vdev, addr);

		vq->vring.desc[head].len = cpu_to_virtio32(_vq->vdev, total_sg * sizeof(struct vring_desc));
	}

	/* We're using some buffers from the free list. */
	vq->vq.num_free -= descs_used;

	/* Update free pointer */
	if (indirect)
		vq->free_head = virtio16_to_cpu(_vq->vdev, vq->vring.desc[head].next);
	else
		vq->free_head = i;

	/* Store token and indirect buffer state. */
	vq->desc_state[head].data = data;
	if (indirect)
		vq->desc_state[head].indir_desc = desc;
	else
		vq->desc_state[head].indir_desc = ctx;

	/* Put entry in available array (but don't update avail->idx until they
	 * do sync). */
	avail = vq->avail_idx_shadow & (vq->vring.num - 1);
	vq->vring.avail->ring[avail] = cpu_to_virtio16(_vq->vdev, head);

	/* Descriptors and available array need to be set before we expose the
	 * new available array entries. */
	virtio_wmb(vq->weak_barriers);
	vq->avail_idx_shadow++;
	vq->vring.avail->idx = cpu_to_virtio16(_vq->vdev, vq->avail_idx_shadow);
	vq->num_added++;

	pr_debug("Added buffer head %i to %p\n", head, vq);
	END_USE(vq);

	/* This is very unlikely, but theoretically possible.  Kick
	 * just in case. */
	if (unlikely(vq->num_added == (1 << 16) - 1))
		virtqueue_kick(_vq);

	return 0;

unmap_release:
	err_idx = i;
	i = head;

	for (n = 0; n < total_sg; n++) {
		if (i == err_idx)
			break;
		vring_unmap_one_split(vq, &desc[i]);
		i = virtio16_to_cpu(_vq->vdev, vq->vring.desc[i].next);
	}

	if (indirect)
		kfree(desc);

	END_USE(vq);
	return -EIO;
}

static bool virtqueue_kick_prepare_split(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	u16 new, old;
	bool needs_kick;

	START_USE(vq);
	/* We need to expose available array entries before checking avail
	 * event. */
	virtio_mb(vq->weak_barriers);

	old = vq->avail_idx_shadow - vq->num_added;
	new = vq->avail_idx_shadow;
	vq->num_added = 0;

#ifdef DEBUG
	if (vq->last_add_time_valid) {
		WARN_ON(ktime_to_ms(ktime_sub(ktime_get(),
					      vq->last_add_time)) > 100);
	}
	vq->last_add_time_valid = false;
#endif

	if (vq->event) {
		needs_kick = vring_need_event(virtio16_to_cpu(_vq->vdev, vring_avail_event(&vq->vring)),
					      new, old);
	} else {
		needs_kick = !(vq->vring.used->flags & cpu_to_virtio16(_vq->vdev, VRING_USED_F_NO_NOTIFY));
	}
	END_USE(vq);
	return needs_kick;
}

static void detach_buf_split(struct vring_virtqueue *vq, unsigned int head,
			     void **ctx)
{
	unsigned int i, j;
	__virtio16 nextflag = cpu_to_virtio16(vq->vq.vdev, VRING_DESC_F_NEXT);

	/* Clear data ptr. */
	vq->desc_state[head].data = NULL;

	/* Put back on free list: unmap first-level descriptors and find end */
	i = head;

	while (vq->vring.desc[i].flags & nextflag) {
		vring_unmap_one_split(vq, &vq->vring.desc[i]);
		i = virtio16_to_cpu(vq->vq.vdev, vq->vring.desc[i].next);
		vq->vq.num_free++;
	}

	vring_unmap_one_split(vq, &vq->vring.desc[i]);
	vq->vring.desc[i].next = cpu_to_virtio16(vq->vq.vdev, vq->free_head);
	vq->free_head = head;

	/* Plus final descriptor */
	vq->vq.num_free++;

	if (vq->indirect) {
		struct vring_desc *indir_desc = vq->desc_state[head].indir_desc;
		u32 len;

		/* Free the indirect table, if any, now that it's unmapped. */
		if (!indir_desc)
			return;

		len = virtio32_to_cpu(vq->vq.vdev, vq->vring.desc[head].len);

		BUG_ON(!(vq->vring.desc[head].flags &
			 cpu_to_virtio16(vq->vq.vdev, VRING_DESC_F_INDIRECT)));
		BUG_ON(len == 0 || len % sizeof(struct vring_desc));

		for (j = 0; j < len / sizeof(struct vring_desc); j++)
			vring_unmap_one_split(vq, &indir_desc[j]);

		kfree(indir_desc);
		vq->desc_state[head].indir_desc = NULL;
	} else if (ctx) {
		*ctx = vq->desc_state[head].indir_desc;
	}
}

static inline bool more_used_split(const struct vring_virtqueue *vq)
{
	return vq->last_used_idx != virtio16_to_cpu(vq->vq.vdev, vq->vring.used->idx);
}

static void *virtqueue_get_buf_ctx_split(struct virtqueue *_vq,
					 unsigned int *len,
					 void **ctx)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	void *ret;
	unsigned int i;
	u16 last_used;

	START_USE(vq);

	if (unlikely(vq->broken)) {
		END_USE(vq);
		return NULL;
	}

	if (!more_used_split(vq)) {
		pr_debug("No more buffers in queue\n");
		END_USE(vq);
		return NULL;
	}

	/* Only get used array entries after they have been exposed by host. */
	virtio_rmb(vq->weak_barriers);

	last_used = (vq->last_used_idx & (vq->vring.num - 1));
	i = virtio32_to_cpu(_vq->vdev, vq->vring.used->ring[last_used].id);
	*len = virtio32_to_cpu(_vq->vdev, vq->vring.used->ring[last_used].len);

	if (unlikely(i >= vq->vring.num)) {
		BAD_RING(vq, "id %u out of range\n", i);
		return NULL;
	}
	if (unlikely(!vq->desc_state[i].data)) {
		BAD_RING(vq, "id %u is not a head!\n", i);
		return NULL;
	}

	/* detach_buf_split clears data, so grab it now. */
	ret = vq->desc_state[i].data;
	detach_buf_split(vq, i, ctx);
	vq->last_used_idx++;
	/* If we expect an interrupt for the next entry, tell host
	 * by writing event index and flush out the write before
	 * the read in the next get_buf call. */
	if (!(vq->avail_flags_shadow & VRING_AVAIL_F_NO_INTERRUPT))
		virtio_store_mb(vq->weak_barriers,
				&vring_used_event(&vq->vring),
				cpu_to_virtio16(_vq->vdev, vq->last_used_idx));

#ifdef DEBUG
	vq->last_add_time_valid = false;
#endif

	END_USE(vq);
	return ret;
}

static void virtqueue_disable_cb_split(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	if (!(vq->avail_flags_shadow & VRING_AVAIL_F_NO_INTERRUPT)) {
		vq->avail_flags_shadow |= VRING_AVAIL_F_NO_INTERRUPT;
		if (!vq->event)
			vq->vring.avail->flags = cpu_to_virtio16(_vq->vdev, vq->avail_flags_shadow);
	}
}

static unsigned virtqueue_enable_cb_prepare_split(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	u16 last_used_idx;

	START_USE(vq);

	/* We optimistically turn back on interrupts, then check if there was
	 * more to do. */
	/* Depending on the VIRTIO_RING_F_EVENT_IDX feature, we need to
	 * either clear the flags bit or point the event index at the next
	 * entry. Always do both to keep code simple. */
	if (vq->avail_flags_shadow & VRING_AVAIL_F_NO_INTERRUPT) {
		vq->avail_flags_shadow &= ~VRING_AVAIL_F_NO_INTERRUPT;
		if (!vq->event)
			vq->vring.avail->flags = cpu_to_virtio16(_vq->vdev, vq->avail_flags_shadow);
	}
	vring_used_event(&vq->vring) = cpu_to_virtio16(_vq->vdev, last_used_idx = vq->last_used_idx);
	END_USE(vq);
	return last_used_idx;
}

static bool virtqueue_poll_split(struct virtqueue *_vq, unsigned last_used_idx)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	return (u16)last_used_idx != virtio16_to_cpu(_vq->vdev, vq->vring.used->idx);
}

static bool virtqueue_enable_cb_delayed_split(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	u16 bufs;

	START_USE(vq);

	/* We optimistically turn back on interrupts, then check if there was
	 * more to do. */
	/* Depending on the VIRTIO_RING_F_USED_EVENT_IDX feature, we need to
	 * either clear the flags bit or point the event index at the next
	 * entry. Always update the event index to keep code simple. */
	if (vq->avail_flags_shadow & VRING_AVAIL_F_NO_INTERRUPT) {
		vq->avail_flags_shadow &= ~VRING_AVAIL_F_NO_INTERRUPT;
		if (!vq->event)
			vq->vring.avail->flags = cpu_to_virtio16(_vq->vdev, vq->avail_flags_shadow);
	}
	/* TODO: tune this threshold */
	bufs = (u16)(vq->avail_idx_shadow - vq->last_used_idx) * 3 / 4;

	virtio_store_mb(vq->weak_barriers,
			&vring_used_event(&vq->vring),
			cpu_to_virtio16(_vq->vdev, vq->last_used_idx + bufs));

	if (unlikely((u16)(virtio16_to_cpu(_vq->vdev, vq->vring.used->idx) - vq->last_used_idx) > bufs)) {
		END_USE(vq);
		return false;
	}

	END_USE(vq);
	return true;
}

static void *virtqueue_detach_unused_buf_split(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	unsigned int i;
	void *buf;

	START_USE(vq);

	for (i = 0; i < vq->vring.num; i++) {
		if (!vq->desc_state[i].data)
			continue;
		/* detach_buf clears data, so grab it now. */
		buf = vq->desc_state[i].data;
		detach_buf_split(vq, i, NULL);
		vq->avail_idx_shadow--;
		vq->vring.avail->idx = cpu_to_virtio16(_vq->vdev, vq->avail_idx_shadow);
		END_USE(vq);
		return buf;
	}
	/* That should have freed everything. */
	BUG_ON(vq->vq.num_free != vq->vring.num);

	END_USE(vq);
	return NULL;
}

/*
 * The layout for the packed ring is a continuous chunk of memory
 * which looks like this.
 *
 * struct vring_packed {
 *	// The actual descriptors (16 bytes each)
 *	struct vring_packed_desc desc[num];
 *
 *	// Padding to the next align boundary.
 *	char pad[];
 *
 *	// Driver Event Suppression
 *	struct vring_packed_desc_event driver;
 *
 *	// Device Event Suppression
 *	struct vring_packed_desc_event device;
 * };
 */
static inline void vring_init_packed(struct vring_packed *vr, unsigned int num,
				     void *p, unsigned long align)
{
	vr->num = num;
	vr->desc = p;
	vr->driver = (void *)ALIGN(((uintptr_t)p +
		sizeof(struct vring_packed_desc) * num), align);
	vr->device = vr->driver + 1;
}

static inline unsigned vring_size_packed(unsigned int num, unsigned long align)
{
	return ((sizeof(struct vring_packed_desc) * num + align - 1)
		& ~(align - 1)) + sizeof(struct vring_packed_desc_event) * 2;
}

static void vring_unmap_state_packed(const struct vring_virtqueue *vq,
				     struct vring_desc_state_packed *state)
{
	u16 flags;

	if (!vring_use_dma_api(vq->vq.vdev))
		return;

	flags = state->flags;

	if (flags & VRING_DESC_F_INDIRECT) {
		dma_unmap_single(vring_dma_dev(vq),
				 state->addr, state->len,
				 (flags & VRING_DESC_F_WRITE) ?
				 DMA_FROM_DEVICE : DMA_TO_DEVICE);
	} else {
		dma_unmap_page(vring_dma_dev(vq),
			       state->addr, state->len,
			       (flags & VRING_DESC_F_WRITE) ?
			       DMA_FROM_DEVICE : DMA_TO_DEVICE);
	}
}

static void vring_unmap_desc_packed(const struct vring_virtqueue *vq,
				   struct vring_packed_desc *desc)
{
	u16 flags;

	if (!vring_use_dma_api(vq->vq.vdev))
		return;

	flags = virtio16_to_cpu(vq->vq.vdev, desc->flags);

	if (flags & VRING_DESC_F_INDIRECT) {
		dma_unmap_single(vring_dma_dev(vq),
				 virtio64_to_cpu(vq->vq.vdev, desc->addr),
				 virtio32_to_cpu(vq->vq.vdev, desc->len),
				 (flags & VRING_DESC_F_WRITE) ?
				 DMA_FROM_DEVICE : DMA_TO_DEVICE);
	} else {
		dma_unmap_page(vring_dma_dev(vq),
			       virtio64_to_cpu(vq->vq.vdev, desc->addr),
			       virtio32_to_cpu(vq->vq.vdev, desc->len),
			       (flags & VRING_DESC_F_WRITE) ?
			       DMA_FROM_DEVICE : DMA_TO_DEVICE);
	}
}

static struct vring_packed_desc *alloc_indirect_packed(struct virtqueue *_vq,
						       unsigned int total_sg,
						       gfp_t gfp)
{
	struct vring_packed_desc *desc;

	/*
	 * We require lowmem mappings for the descriptors because
	 * otherwise virt_to_phys will give us bogus addresses in the
	 * virtqueue.
	 */
	gfp &= ~__GFP_HIGHMEM;

	desc = kmalloc(total_sg * sizeof(struct vring_packed_desc), gfp);

	return desc;
}

static inline int virtqueue_add_packed(struct virtqueue *_vq,
				       struct scatterlist *sgs[],
				       unsigned int total_sg,
				       unsigned int out_sgs,
				       unsigned int in_sgs,
				       void *data,
				       void *ctx,
				       gfp_t gfp)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	struct vring_packed_desc *desc;
	struct scatterlist *sg;
	unsigned int i, n, descs_used, uninitialized_var(prev), err_idx;
	__virtio16 uninitialized_var(head_flags), flags;
	u16 head, avail_wrap_counter, id, curr;
	bool indirect;

	START_USE(vq);

	BUG_ON(data == NULL);
	BUG_ON(ctx && vq->indirect);

	if (unlikely(vq->broken)) {
		END_USE(vq);
		return -EIO;
	}

#ifdef DEBUG
	{
		ktime_t now = ktime_get();

		/* No kick or get, with .1 second between?  Warn. */
		if (vq->last_add_time_valid)
			WARN_ON(ktime_to_ms(ktime_sub(now, vq->last_add_time))
					    > 100);
		vq->last_add_time = now;
		vq->last_add_time_valid = true;
	}
#endif

	BUG_ON(total_sg == 0);

	head = vq->next_avail_idx;
	avail_wrap_counter = vq->avail_wrap_counter;

	if (virtqueue_use_indirect(_vq, total_sg))
		desc = alloc_indirect_packed(_vq, total_sg, gfp);
	else {
		desc = NULL;
		WARN_ON_ONCE(total_sg > vq->vring_packed.num && !vq->indirect);
	}

	if (desc) {
		/* Use a single buffer which doesn't continue */
		indirect = true;
		/* Set up rest to use this indirect table. */
		i = 0;
		descs_used = 1;
	} else {
		indirect = false;
		desc = vq->vring_packed.desc;
		i = head;
		descs_used = total_sg;
	}

	if (vq->vq.num_free < descs_used) {
		pr_debug("Can't add buf len %i - avail = %i\n",
			 descs_used, vq->vq.num_free);
		/* FIXME: for historical reasons, we force a notify here if
		 * there are outgoing parts to the buffer.  Presumably the
		 * host should service the ring ASAP. */
		if (out_sgs)
			vq->notify(&vq->vq);
		if (indirect)
			kfree(desc);
		END_USE(vq);
		return -ENOSPC;
	}

	id = vq->free_head;
	BUG_ON(id == vq->vring_packed.num);

	curr = id;
	for (n = 0; n < out_sgs + in_sgs; n++) {
		for (sg = sgs[n]; sg; sg = sg_next(sg)) {
			dma_addr_t addr = vring_map_one_sg(vq, sg, n < out_sgs ?
					       DMA_TO_DEVICE : DMA_FROM_DEVICE);
			if (vring_mapping_error(vq, addr))
				goto unmap_release;

			flags = cpu_to_virtio16(_vq->vdev, VRING_DESC_F_NEXT |
				  (n < out_sgs ? 0 : VRING_DESC_F_WRITE) |
				  _VRING_DESC_F_AVAIL(vq->avail_wrap_counter) |
				  _VRING_DESC_F_USED(!vq->avail_wrap_counter));
			if (!indirect && i == head)
				head_flags = flags;
			else
				desc[i].flags = flags;

			desc[i].addr = cpu_to_virtio64(_vq->vdev, addr);
			desc[i].len = cpu_to_virtio32(_vq->vdev, sg->length);
			i++;
			if (!indirect) {
				if (vring_use_dma_api(_vq->vdev)) {
					vq->desc_state_packed[curr].addr = addr;
					vq->desc_state_packed[curr].len =
						sg->length;
					vq->desc_state_packed[curr].flags =
						virtio16_to_cpu(_vq->vdev,
								flags);
				}
				curr = vq->desc_state_packed[curr].next;

				if (i >= vq->vring_packed.num) {
					i = 0;
					vq->avail_wrap_counter ^= 1;
				}
			}
		}
	}

	prev = (i > 0 ? i : vq->vring_packed.num) - 1;
	desc[prev].id = cpu_to_virtio16(_vq->vdev, id);

	/* Last one doesn't continue. */
	if (total_sg == 1)
		head_flags &= cpu_to_virtio16(_vq->vdev, ~VRING_DESC_F_NEXT);
	else
		desc[prev].flags &= cpu_to_virtio16(_vq->vdev,
						~VRING_DESC_F_NEXT);

	if (indirect) {
		/* Now that the indirect table is filled in, map it. */
		dma_addr_t addr = vring_map_single(
			vq, desc, total_sg * sizeof(struct vring_packed_desc),
			DMA_TO_DEVICE);
		if (vring_mapping_error(vq, addr))
			goto unmap_release;

		head_flags = cpu_to_virtio16(_vq->vdev, VRING_DESC_F_INDIRECT |
				      _VRING_DESC_F_AVAIL(avail_wrap_counter) |
				      _VRING_DESC_F_USED(!avail_wrap_counter));
		vq->vring_packed.desc[head].addr = cpu_to_virtio64(_vq->vdev,
								   addr);
		vq->vring_packed.desc[head].len = cpu_to_virtio32(_vq->vdev,
				total_sg * sizeof(struct vring_packed_desc));
		vq->vring_packed.desc[head].id = cpu_to_virtio16(_vq->vdev, id);

		if (vring_use_dma_api(_vq->vdev)) {
			vq->desc_state_packed[id].addr = addr;
			vq->desc_state_packed[id].len = total_sg *
					sizeof(struct vring_packed_desc);
			vq->desc_state_packed[id].flags =
					virtio16_to_cpu(_vq->vdev, head_flags);
		}
	}

	/* We're using some buffers from the free list. */
	vq->vq.num_free -= descs_used;

	/* Update free pointer */
	if (indirect) {
		n = head + 1;
		if (n >= vq->vring_packed.num) {
			n = 0;
			vq->avail_wrap_counter ^= 1;
		}
		vq->next_avail_idx = n;
		vq->free_head = vq->desc_state_packed[id].next;
	} else {
		vq->next_avail_idx = i;
		vq->free_head = curr;
	}

	/* Store token and indirect buffer state. */
	vq->desc_state_packed[id].num = descs_used;
	vq->desc_state_packed[id].data = data;
	if (indirect)
		vq->desc_state_packed[id].indir_desc = desc;
	else
		vq->desc_state_packed[id].indir_desc = ctx;

	/* A driver MUST NOT make the first descriptor in the list
	 * available before all subsequent descriptors comprising
	 * the list are made available. */
	virtio_wmb(vq->weak_barriers);
	vq->vring_packed.desc[head].flags = head_flags;
	vq->num_added += descs_used;

	pr_debug("Added buffer head %i to %p\n", head, vq);
	END_USE(vq);

	return 0;

unmap_release:
	err_idx = i;
	i = head;

	for (n = 0; n < total_sg; n++) {
		if (i == err_idx)
			break;
		vring_unmap_desc_packed(vq, &desc[i]);
		i++;
		if (!indirect && i >= vq->vring_packed.num)
			i = 0;
	}

	vq->avail_wrap_counter = avail_wrap_counter;

	if (indirect)
		kfree(desc);

	END_USE(vq);
	return -EIO;
}

static bool virtqueue_kick_prepare_packed(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	u16 flags;
	bool needs_kick;
	u32 snapshot;

	START_USE(vq);
	/* We need to expose the new flags value before checking notification
	 * suppressions. */
	virtio_mb(vq->weak_barriers);

	snapshot = READ_ONCE(*(u32 *)vq->vring_packed.device);
	flags = virtio16_to_cpu(_vq->vdev, (__virtio16)(snapshot >> 16)) & 0x3;

#ifdef DEBUG
	if (vq->last_add_time_valid) {
		WARN_ON(ktime_to_ms(ktime_sub(ktime_get(),
					      vq->last_add_time)) > 100);
	}
	vq->last_add_time_valid = false;
#endif

	needs_kick = (flags != VRING_EVENT_F_DISABLE);
	END_USE(vq);
	return needs_kick;
}

static void detach_buf_packed(struct vring_virtqueue *vq,
			      unsigned int id, void **ctx)
{
	struct vring_desc_state_packed *state = NULL;
	struct vring_packed_desc *desc;
	unsigned int curr, i;

	/* Clear data ptr. */
	vq->desc_state_packed[id].data = NULL;

	curr = id;
	for (i = 0; i < vq->desc_state_packed[id].num; i++) {
		state = &vq->desc_state_packed[curr];
		vring_unmap_state_packed(vq, state);
		curr = state->next;
	}

	BUG_ON(state == NULL);
	vq->vq.num_free += vq->desc_state_packed[id].num;
	state->next = vq->free_head;
	vq->free_head = id;

	if (vq->indirect) {
		u32 len;

		/* Free the indirect table, if any, now that it's unmapped. */
		desc = vq->desc_state_packed[id].indir_desc;
		if (!desc)
			return;

		if (vring_use_dma_api(vq->vq.vdev)) {
			len = vq->desc_state_packed[id].len;
			for (i = 0; i < len / sizeof(struct vring_packed_desc);
					i++)
				vring_unmap_desc_packed(vq, &desc[i]);
		}
		kfree(desc);
		vq->desc_state_packed[id].indir_desc = NULL;
	} else if (ctx) {
		*ctx = vq->desc_state_packed[id].indir_desc;
	}
}

static inline bool is_used_desc_packed(const struct vring_virtqueue *vq,
				       u16 idx, bool used_wrap_counter)
{
	u16 flags;
	bool avail, used;

	flags = virtio16_to_cpu(vq->vq.vdev,
				vq->vring_packed.desc[idx].flags);
	avail = !!(flags & VRING_DESC_F_AVAIL);
	used = !!(flags & VRING_DESC_F_USED);

	return avail == used && used == used_wrap_counter;
}

static inline bool more_used_packed(const struct vring_virtqueue *vq)
{
	return is_used_desc_packed(vq, vq->last_used_idx,
			vq->used_wrap_counter);
}

static void *virtqueue_get_buf_ctx_packed(struct virtqueue *_vq,
					  unsigned int *len,
					  void **ctx)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	u16 last_used, id;
	void *ret;

	START_USE(vq);

	if (unlikely(vq->broken)) {
		END_USE(vq);
		return NULL;
	}

	if (!more_used_packed(vq)) {
		pr_debug("No more buffers in queue\n");
		END_USE(vq);
		return NULL;
	}

	/* Only get used elements after they have been exposed by host. */
	virtio_rmb(vq->weak_barriers);

	last_used = vq->last_used_idx;
	id = virtio16_to_cpu(_vq->vdev, vq->vring_packed.desc[last_used].id);
	*len = virtio32_to_cpu(_vq->vdev, vq->vring_packed.desc[last_used].len);

	if (unlikely(id >= vq->vring_packed.num)) {
		BAD_RING(vq, "id %u out of range\n", id);
		return NULL;
	}
	if (unlikely(!vq->desc_state_packed[id].data)) {
		BAD_RING(vq, "id %u is not a head!\n", id);
		return NULL;
	}

	vq->last_used_idx += vq->desc_state_packed[id].num;
	if (vq->last_used_idx >= vq->vring_packed.num) {
		vq->last_used_idx -= vq->vring_packed.num;
		vq->used_wrap_counter ^= 1;
	}

	/* detach_buf_packed clears data, so grab it now. */
	ret = vq->desc_state_packed[id].data;
	detach_buf_packed(vq, id, ctx);

#ifdef DEBUG
	vq->last_add_time_valid = false;
#endif

	END_USE(vq);
	return ret;
}

static void virtqueue_disable_cb_packed(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	if (vq->event_flags_shadow != VRING_EVENT_F_DISABLE) {
		vq->event_flags_shadow = VRING_EVENT_F_DISABLE;
		vq->vring_packed.driver->flags = cpu_to_virtio16(_vq->vdev,
							vq->event_flags_shadow);
	}
}

static unsigned virtqueue_enable_cb_prepare_packed(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	START_USE(vq);

	/* We optimistically turn back on interrupts, then check if there was
	 * more to do. */

	if (vq->event_flags_shadow == VRING_EVENT_F_DISABLE) {
		vq->event_flags_shadow = VRING_EVENT_F_ENABLE;
		vq->vring_packed.driver->flags = cpu_to_virtio16(_vq->vdev,
							vq->event_flags_shadow);
	}

	END_USE(vq);
	return vq->last_used_idx | ((u16)vq->used_wrap_counter << 15);
}

static bool virtqueue_poll_packed(struct virtqueue *_vq, unsigned off_wrap)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	bool wrap_counter;
	u16 used_idx;

	wrap_counter = off_wrap >> 15;
	used_idx = off_wrap & ~(1 << 15);

	return is_used_desc_packed(vq, used_idx, wrap_counter);
}

static bool virtqueue_enable_cb_delayed_packed(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	START_USE(vq);

	/* We optimistically turn back on interrupts, then check if there was
	 * more to do. */

	if (vq->event_flags_shadow == VRING_EVENT_F_DISABLE) {
		vq->event_flags_shadow = VRING_EVENT_F_ENABLE;
		vq->vring_packed.driver->flags = cpu_to_virtio16(_vq->vdev,
							vq->event_flags_shadow);
		/* We need to enable interrupts first before re-checking
		 * for more used buffers. */
		virtio_mb(vq->weak_barriers);
	}

	if (more_used_packed(vq)) {
		END_USE(vq);
		return false;
	}

	END_USE(vq);
	return true;
}

static void *virtqueue_detach_unused_buf_packed(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	unsigned int i;
	void *buf;

	START_USE(vq);

	for (i = 0; i < vq->vring_packed.num; i++) {
		if (!vq->desc_state_packed[i].data)
			continue;
		/* detach_buf clears data, so grab it now. */
		buf = vq->desc_state_packed[i].data;
		detach_buf_packed(vq, i, NULL);
		END_USE(vq);
		return buf;
	}
	/* That should have freed everything. */
	BUG_ON(vq->vq.num_free != vq->vring_packed.num);

	END_USE(vq);
	return NULL;
}

static inline int virtqueue_add(struct virtqueue *_vq,
				struct scatterlist *sgs[],
				unsigned int total_sg,
				unsigned int out_sgs,
				unsigned int in_sgs,
				void *data,
				void *ctx,
				gfp_t gfp)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	return vq->packed ? virtqueue_add_packed(_vq, sgs, total_sg, out_sgs,
						 in_sgs, data, ctx, gfp) :
			    virtqueue_add_split(_vq, sgs, total_sg, out_sgs,
						in_sgs, data, ctx, gfp);
}

/**
 * virtqueue_add_sgs - expose buffers to other end
 * @vq: the struct virtqueue we're talking about.
 * @sgs: array of terminated scatterlists.
 * @out_num: the number of scatterlists readable by other side
 * @in_num: the number of scatterlists which are writable (after readable ones)
 * @data: the token identifying the buffer.
 * @gfp: how to do memory allocations (if necessary).
 *
 * Caller must ensure we don't call this with other virtqueue operations
 * at the same time (except where noted).
 *
 * Returns zero or a negative error (ie. ENOSPC, ENOMEM, EIO).
 */
int virtqueue_add_sgs(struct virtqueue *_vq,
		      struct scatterlist *sgs[],
		      unsigned int out_sgs,
		      unsigned int in_sgs,
		      void *data,
		      gfp_t gfp)
{
	unsigned int i, total_sg = 0;

	/* Count them first. */
	for (i = 0; i < out_sgs + in_sgs; i++) {
		struct scatterlist *sg;
		for (sg = sgs[i]; sg; sg = sg_next(sg))
			total_sg++;
	}
	return virtqueue_add(_vq, sgs, total_sg, out_sgs, in_sgs,
			     data, NULL, gfp);
}
EXPORT_SYMBOL_GPL(virtqueue_add_sgs);

/**
 * virtqueue_add_outbuf - expose output buffers to other end
 * @vq: the struct virtqueue we're talking about.
 * @sg: scatterlist (must be well-formed and terminated!)
 * @num: the number of entries in @sg readable by other side
 * @data: the token identifying the buffer.
 * @gfp: how to do memory allocations (if necessary).
 *
 * Caller must ensure we don't call this with other virtqueue operations
 * at the same time (except where noted).
 *
 * Returns zero or a negative error (ie. ENOSPC, ENOMEM, EIO).
 */
int virtqueue_add_outbuf(struct virtqueue *vq,
			 struct scatterlist *sg, unsigned int num,
			 void *data,
			 gfp_t gfp)
{
	return virtqueue_add(vq, &sg, num, 1, 0, data, NULL, gfp);
}
EXPORT_SYMBOL_GPL(virtqueue_add_outbuf);

/**
 * virtqueue_add_inbuf - expose input buffers to other end
 * @vq: the struct virtqueue we're talking about.
 * @sg: scatterlist (must be well-formed and terminated!)
 * @num: the number of entries in @sg writable by other side
 * @data: the token identifying the buffer.
 * @gfp: how to do memory allocations (if necessary).
 *
 * Caller must ensure we don't call this with other virtqueue operations
 * at the same time (except where noted).
 *
 * Returns zero or a negative error (ie. ENOSPC, ENOMEM, EIO).
 */
int virtqueue_add_inbuf(struct virtqueue *vq,
			struct scatterlist *sg, unsigned int num,
			void *data,
			gfp_t gfp)
{
	return virtqueue_add(vq, &sg, num, 0, 1, data, NULL, gfp);
}
EXPORT_SYMBOL_GPL(virtqueue_add_inbuf);

/**
 * virtqueue_add_inbuf_ctx - expose input buffers to other end
 * @vq: the struct virtqueue we're talking about.
 * @sg: scatterlist (must be well-formed and terminated!)
 * @num: the number of entries in @sg writable by other side
 * @data: the token identifying the buffer.
 * @ctx: extra context for the token
 * @gfp: how to do memory allocations (if necessary).
 *
 * Caller must ensure we don't call this with other virtqueue operations
 * at the same time (except where noted).
 *
 * Returns zero or a negative error (ie. ENOSPC, ENOMEM, EIO).
 */
int virtqueue_add_inbuf_ctx(struct virtqueue *vq,
			struct scatterlist *sg, unsigned int num,
			void *data,
			void *ctx,
			gfp_t gfp)
{
	return virtqueue_add(vq, &sg, num, 0, 1, data, ctx, gfp);
}
EXPORT_SYMBOL_GPL(virtqueue_add_inbuf_ctx);

/**
 * virtqueue_kick_prepare - first half of split virtqueue_kick call.
 * @vq: the struct virtqueue
 *
 * Instead of virtqueue_kick(), you can do:
 *	if (virtqueue_kick_prepare(vq))
 *		virtqueue_notify(vq);
 *
 * This is sometimes useful because the virtqueue_kick_prepare() needs
 * to be serialized, but the actual virtqueue_notify() call does not.
 */
bool virtqueue_kick_prepare(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	return vq->packed ? virtqueue_kick_prepare_packed(_vq) :
			    virtqueue_kick_prepare_split(_vq);
}
EXPORT_SYMBOL_GPL(virtqueue_kick_prepare);

/**
 * virtqueue_notify - second half of split virtqueue_kick call.
 * @vq: the struct virtqueue
 *
 * This does not need to be serialized.
 *
 * Returns false if host notify failed or queue is broken, otherwise true.
 */
bool virtqueue_notify(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	if (unlikely(vq->broken))
		return false;

	/* Prod other side to tell it about changes. */
	if (!vq->notify(_vq)) {
		vq->broken = true;
		return false;
	}
	return true;
}
EXPORT_SYMBOL_GPL(virtqueue_notify);

/**
 * virtqueue_kick - update after add_buf
 * @vq: the struct virtqueue
 *
 * After one or more virtqueue_add_* calls, invoke this to kick
 * the other side.
 *
 * Caller must ensure we don't call this with other virtqueue
 * operations at the same time (except where noted).
 *
 * Returns false if kick failed, otherwise true.
 */
bool virtqueue_kick(struct virtqueue *vq)
{
	if (virtqueue_kick_prepare(vq))
		return virtqueue_notify(vq);
	return true;
}
EXPORT_SYMBOL_GPL(virtqueue_kick);

static inline bool more_used(const struct vring_virtqueue *vq)
{
	return vq->packed ? more_used_packed(vq) : more_used_split(vq);
}

/**
 * virtqueue_get_buf - get the next used buffer
 * @vq: the struct virtqueue we're talking about.
 * @len: the length written into the buffer
 *
 * If the device wrote data into the buffer, @len will be set to the
 * amount written.  This means you don't need to clear the buffer
 * beforehand to ensure there's no data leakage in the case of short
 * writes.
 *
 * Caller must ensure we don't call this with other virtqueue
 * operations at the same time (except where noted).
 *
 * Returns NULL if there are no used buffers, or the "data" token
 * handed to virtqueue_add_*().
 */
void *virtqueue_get_buf_ctx(struct virtqueue *_vq, unsigned int *len,
			    void **ctx)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	return vq->packed ? virtqueue_get_buf_ctx_packed(_vq, len, ctx) :
			    virtqueue_get_buf_ctx_split(_vq, len, ctx);
}
EXPORT_SYMBOL_GPL(virtqueue_get_buf_ctx);

void *virtqueue_get_buf(struct virtqueue *_vq, unsigned int *len)
{
	return virtqueue_get_buf_ctx(_vq, len, NULL);
}
EXPORT_SYMBOL_GPL(virtqueue_get_buf);
/**
 * virtqueue_disable_cb - disable callbacks
 * @vq: the struct virtqueue we're talking about.
 *
 * Note that this is not necessarily synchronous, hence unreliable and only
 * useful as an optimization.
 *
 * Unlike other operations, this need not be serialized.
 */
void virtqueue_disable_cb(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	if (vq->packed)
		virtqueue_disable_cb_packed(_vq);
	else
		virtqueue_disable_cb_split(_vq);
}
EXPORT_SYMBOL_GPL(virtqueue_disable_cb);

/**
 * virtqueue_enable_cb_prepare - restart callbacks after disable_cb
 * @vq: the struct virtqueue we're talking about.
 *
 * This re-enables callbacks; it returns current queue state
 * in an opaque unsigned value. This value should be later tested by
 * virtqueue_poll, to detect a possible race between the driver checking for
 * more work, and enabling callbacks.
 *
 * Caller must ensure we don't call this with other virtqueue
 * operations at the same time (except where noted).
 */
unsigned virtqueue_enable_cb_prepare(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	return vq->packed ? virtqueue_enable_cb_prepare_packed(_vq) :
			    virtqueue_enable_cb_prepare_split(_vq);
}
EXPORT_SYMBOL_GPL(virtqueue_enable_cb_prepare);

/**
 * virtqueue_poll - query pending used buffers
 * @vq: the struct virtqueue we're talking about.
 * @last_used_idx: virtqueue state (from call to virtqueue_enable_cb_prepare).
 *
 * Returns "true" if there are pending used buffers in the queue.
 *
 * This does not need to be serialized.
 */
bool virtqueue_poll(struct virtqueue *_vq, unsigned last_used_idx)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	/* We need to enable interrupts first before re-checking
	 * for more used buffers. */
	virtio_mb(vq->weak_barriers);
	return vq->packed ? virtqueue_poll_packed(_vq, last_used_idx) :
			    virtqueue_poll_split(_vq, last_used_idx);
}
EXPORT_SYMBOL_GPL(virtqueue_poll);

/**
 * virtqueue_enable_cb - restart callbacks after disable_cb.
 * @vq: the struct virtqueue we're talking about.
 *
 * This re-enables callbacks; it returns "false" if there are pending
 * buffers in the queue, to detect a possible race between the driver
 * checking for more work, and enabling callbacks.
 *
 * Caller must ensure we don't call this with other virtqueue
 * operations at the same time (except where noted).
 */
bool virtqueue_enable_cb(struct virtqueue *_vq)
{
	unsigned last_used_idx = virtqueue_enable_cb_prepare(_vq);
	return !virtqueue_poll(_vq, last_used_idx);
}
EXPORT_SYMBOL_GPL(virtqueue_enable_cb);

/**
 * virtqueue_enable_cb_delayed - restart callbacks after disable_cb.
 * @vq: the struct virtqueue we're talking about.
 *
 * This re-enables callbacks but hints to the other side to delay
 * interrupts until most of the available buffers have been processed;
 * it returns "false" if there are many pending buffers in the queue,
 * to detect a possible race between the driver checking for more work,
 * and enabling callbacks.
 *
 * Caller must ensure we don't call this with other virtqueue
 * operations at the same time (except where noted).
 */
bool virtqueue_enable_cb_delayed(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	return vq->packed ? virtqueue_enable_cb_delayed_packed(_vq) :
			    virtqueue_enable_cb_delayed_split(_vq);
}
EXPORT_SYMBOL_GPL(virtqueue_enable_cb_delayed);

/**
 * virtqueue_detach_unused_buf - detach first unused buffer
 * @vq: the struct virtqueue we're talking about.
 *
 * Returns NULL or the "data" token handed to virtqueue_add_*().
 * This is not valid on an active queue; it is useful only for device
 * shutdown.
 */
void *virtqueue_detach_unused_buf(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	return vq->packed ? virtqueue_detach_unused_buf_packed(_vq) :
			    virtqueue_detach_unused_buf_split(_vq);
}
EXPORT_SYMBOL_GPL(virtqueue_detach_unused_buf);

irqreturn_t vring_interrupt(int irq, void *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	if (!more_used(vq)) {
		pr_debug("virtqueue interrupt with no work for %p\n", vq);
		return IRQ_NONE;
	}

	if (unlikely(vq->broken))
		return IRQ_HANDLED;

	pr_debug("virtqueue callback for %p (%p)\n", vq, vq->vq.callback);
	if (vq->vq.callback)
		vq->vq.callback(&vq->vq);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(vring_interrupt);

struct virtqueue *__vring_new_virtqueue(unsigned int index,
					union vring_union vring,
					bool packed,
					struct virtio_device *vdev,
					bool weak_barriers,
					bool context,
					bool (*notify)(struct virtqueue *),
					void (*callback)(struct virtqueue *),
					const char *name)
{
	struct vring_virtqueue *vq;
	unsigned int num, i;
	size_t size;

	num = packed ? vring.vring_packed.num : vring.vring_split.num;
	size = packed ? num * sizeof(struct vring_desc_state_packed) :
			num * sizeof(struct vring_desc_state);

	vq = kmalloc(sizeof(*vq) + size, GFP_KERNEL);
	if (!vq)
		return NULL;

	vq->vq.callback = callback;
	vq->vq.vdev = vdev;
	vq->vq.name = name;
	vq->vq.num_free = num;
	vq->vq.index = index;
	vq->we_own_ring = false;
	vq->queue_dma_addr = 0;
	vq->queue_size_in_bytes = 0;
	vq->notify = notify;
	vq->weak_barriers = weak_barriers;
	vq->broken = false;
	vq->last_used_idx = 0;
	vq->num_added = 0;
	vq->packed = packed;
	list_add_tail(&vq->vq.list, &vdev->vqs);
#ifdef DEBUG
	vq->in_use = false;
	vq->last_add_time_valid = false;
#endif

	vq->indirect = virtio_has_feature(vdev, VIRTIO_RING_F_INDIRECT_DESC) &&
		!context;
	vq->event = virtio_has_feature(vdev, VIRTIO_RING_F_EVENT_IDX);

	if (vq->packed) {
		vq->vring_packed = vring.vring_packed;
		vq->next_avail_idx = 0;
		vq->avail_wrap_counter = 1;
		vq->used_wrap_counter = 1;
		vq->event_flags_shadow = 0;

		memset(vq->desc_state_packed, 0,
			num * sizeof(struct vring_desc_state_packed));

		/* Put everything in free lists. */
		vq->free_head = 0;
		for (i = 0; i < num-1; i++)
			vq->desc_state_packed[i].next = i + 1;
	} else {
		vq->vring = vring.vring_split;
		vq->avail_flags_shadow = 0;
		vq->avail_idx_shadow = 0;

		/* Put everything in free lists. */
		vq->free_head = 0;
		for (i = 0; i < num-1; i++)
			vq->vring.desc[i].next = cpu_to_virtio16(vdev, i + 1);

		memset(vq->desc_state, 0,
			num * sizeof(struct vring_desc_state));
	}

	/* No callback?  Tell other side not to bother us. */
	if (!callback) {
		if (packed) {
			vq->event_flags_shadow = VRING_EVENT_F_DISABLE;
			vq->vring_packed.driver->flags = cpu_to_virtio16(vdev,
						vq->event_flags_shadow);
		} else {
			vq->avail_flags_shadow |= VRING_AVAIL_F_NO_INTERRUPT;
			if (!vq->event)
				vq->vring.avail->flags = cpu_to_virtio16(vdev,
						vq->avail_flags_shadow);
		}
	}

	return &vq->vq;
}
EXPORT_SYMBOL_GPL(__vring_new_virtqueue);

static void *vring_alloc_queue(struct virtio_device *vdev, size_t size,
			      dma_addr_t *dma_handle, gfp_t flag)
{
	if (vring_use_dma_api(vdev)) {
		return dma_alloc_coherent(vdev->dev.parent, size,
					  dma_handle, flag);
	} else {
		void *queue = alloc_pages_exact(PAGE_ALIGN(size), flag);
		if (queue) {
			phys_addr_t phys_addr = virt_to_phys(queue);
			*dma_handle = (dma_addr_t)phys_addr;

			/*
			 * Sanity check: make sure we dind't truncate
			 * the address.  The only arches I can find that
			 * have 64-bit phys_addr_t but 32-bit dma_addr_t
			 * are certain non-highmem MIPS and x86
			 * configurations, but these configurations
			 * should never allocate physical pages above 32
			 * bits, so this is fine.  Just in case, throw a
			 * warning and abort if we end up with an
			 * unrepresentable address.
			 */
			if (WARN_ON_ONCE(*dma_handle != phys_addr)) {
				free_pages_exact(queue, PAGE_ALIGN(size));
				return NULL;
			}
		}
		return queue;
	}
}

static void vring_free_queue(struct virtio_device *vdev, size_t size,
			     void *queue, dma_addr_t dma_handle)
{
	if (vring_use_dma_api(vdev)) {
		dma_free_coherent(vdev->dev.parent, size, queue, dma_handle);
	} else {
		free_pages_exact(queue, PAGE_ALIGN(size));
	}
}

static inline int
__vring_size(unsigned int num, unsigned long align, bool packed)
{
	return packed ? vring_size_packed(num, align) : vring_size(num, align);
}

struct virtqueue *vring_create_virtqueue(
	unsigned int index,
	unsigned int num,
	unsigned int vring_align,
	struct virtio_device *vdev,
	bool weak_barriers,
	bool may_reduce_num,
	bool context,
	bool (*notify)(struct virtqueue *),
	void (*callback)(struct virtqueue *),
	const char *name)
{
	struct virtqueue *vq;
	void *queue = NULL;
	dma_addr_t dma_addr;
	size_t queue_size_in_bytes;
	union vring_union vring;
	bool packed;

	/* We assume num is a power of 2. */
	if (num & (num - 1)) {
		dev_warn(&vdev->dev, "Bad virtqueue length %u\n", num);
		return NULL;
	}

	packed = virtio_has_feature(vdev, VIRTIO_F_RING_PACKED);

	/* TODO: allocate each queue chunk individually */
	for (; num && __vring_size(num, vring_align, packed) > PAGE_SIZE;
			num /= 2) {
		queue = vring_alloc_queue(vdev, __vring_size(num, vring_align,
							     packed),
					  &dma_addr,
					  GFP_KERNEL|__GFP_NOWARN|__GFP_ZERO);
		if (queue)
			break;
	}

	if (!num)
		return NULL;

	if (!queue) {
		/* Try to get a single page. You are my only hope! */
		queue = vring_alloc_queue(vdev, __vring_size(num, vring_align,
							     packed),
					  &dma_addr, GFP_KERNEL|__GFP_ZERO);
	}
	if (!queue)
		return NULL;

	queue_size_in_bytes = __vring_size(num, vring_align, packed);
	if (packed)
		vring_init_packed(&vring.vring_packed, num, queue, vring_align);
	else
		vring_init(&vring.vring_split, num, queue, vring_align);

	vq = __vring_new_virtqueue(index, vring, packed, vdev, weak_barriers,
				   context, notify, callback, name);
	if (!vq) {
		vring_free_queue(vdev, queue_size_in_bytes, queue,
				 dma_addr);
		return NULL;
	}

	to_vvq(vq)->queue_dma_addr = dma_addr;
	to_vvq(vq)->queue_size_in_bytes = queue_size_in_bytes;
	to_vvq(vq)->we_own_ring = true;

	return vq;
}
EXPORT_SYMBOL_GPL(vring_create_virtqueue);

struct virtqueue *vring_new_virtqueue(unsigned int index,
				      unsigned int num,
				      unsigned int vring_align,
				      struct virtio_device *vdev,
				      bool weak_barriers,
				      bool context,
				      void *pages,
				      bool (*notify)(struct virtqueue *vq),
				      void (*callback)(struct virtqueue *vq),
				      const char *name)
{
	union vring_union vring;
	bool packed;

	packed = virtio_has_feature(vdev, VIRTIO_F_RING_PACKED);
	if (packed)
		vring_init_packed(&vring.vring_packed, num, pages, vring_align);
	else
		vring_init(&vring.vring_split, num, pages, vring_align);

	return __vring_new_virtqueue(index, vring, packed, vdev, weak_barriers,
				     context, notify, callback, name);
}
EXPORT_SYMBOL_GPL(vring_new_virtqueue);

void vring_del_virtqueue(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	if (vq->we_own_ring) {
		vring_free_queue(vq->vq.vdev, vq->queue_size_in_bytes,
				 vq->packed ? (void *)vq->vring_packed.desc :
					      (void *)vq->vring.desc,
				 vq->queue_dma_addr);
	}
	list_del(&_vq->list);
	kfree(vq);
}
EXPORT_SYMBOL_GPL(vring_del_virtqueue);

/* Manipulates transport-specific feature bits. */
void vring_transport_features(struct virtio_device *vdev)
{
	unsigned int i;

	for (i = VIRTIO_TRANSPORT_F_START; i < VIRTIO_TRANSPORT_F_END; i++) {
		switch (i) {
		case VIRTIO_RING_F_INDIRECT_DESC:
			break;
		case VIRTIO_RING_F_EVENT_IDX:
			break;
		case VIRTIO_F_VERSION_1:
			break;
		case VIRTIO_F_IOMMU_PLATFORM:
			break;
		default:
			/* We don't understand this bit. */
			__virtio_clear_bit(vdev, i);
		}
	}
}
EXPORT_SYMBOL_GPL(vring_transport_features);

/**
 * virtqueue_get_vring_size - return the size of the virtqueue's vring
 * @vq: the struct virtqueue containing the vring of interest.
 *
 * Returns the size of the vring.  This is mainly used for boasting to
 * userspace.  Unlike other operations, this need not be serialized.
 */
unsigned int virtqueue_get_vring_size(struct virtqueue *_vq)
{

	struct vring_virtqueue *vq = to_vvq(_vq);

	return vq->packed ? vq->vring_packed.num : vq->vring.num;
}
EXPORT_SYMBOL_GPL(virtqueue_get_vring_size);

bool virtqueue_is_broken(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	return vq->broken;
}
EXPORT_SYMBOL_GPL(virtqueue_is_broken);

/*
 * This should prevent the device from being used, allowing drivers to
 * recover.  You may need to grab appropriate locks to flush.
 */
void virtio_break_device(struct virtio_device *dev)
{
	struct virtqueue *_vq;

	list_for_each_entry(_vq, &dev->vqs, list) {
		struct vring_virtqueue *vq = to_vvq(_vq);
		vq->broken = true;
	}
}
EXPORT_SYMBOL_GPL(virtio_break_device);

dma_addr_t virtqueue_get_desc_addr(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	BUG_ON(!vq->we_own_ring);

	return vq->queue_dma_addr;
}
EXPORT_SYMBOL_GPL(virtqueue_get_desc_addr);

dma_addr_t virtqueue_get_avail_addr(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	BUG_ON(!vq->we_own_ring);

	if (vq->packed)
		return vq->queue_dma_addr + ((char *)vq->vring_packed.driver -
				(char *)vq->vring_packed.desc);

	return vq->queue_dma_addr +
		((char *)vq->vring.avail - (char *)vq->vring.desc);
}
EXPORT_SYMBOL_GPL(virtqueue_get_avail_addr);

dma_addr_t virtqueue_get_used_addr(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	BUG_ON(!vq->we_own_ring);

	if (vq->packed)
		return vq->queue_dma_addr + ((char *)vq->vring_packed.device -
				(char *)vq->vring_packed.desc);

	return vq->queue_dma_addr +
		((char *)vq->vring.used - (char *)vq->vring.desc);
}
EXPORT_SYMBOL_GPL(virtqueue_get_used_addr);

/* Only available for split ring */
const struct vring *virtqueue_get_vring(struct virtqueue *vq)
{
	return &to_vvq(vq)->vring;
}
EXPORT_SYMBOL_GPL(virtqueue_get_vring);

MODULE_LICENSE("GPL");
