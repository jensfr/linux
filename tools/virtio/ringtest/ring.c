/*
 * Copyright (C) 2016 Red Hat, Inc.
 * Author: Michael S. Tsirkin <mst@redhat.com>
 * This work is licensed under the terms of the GNU GPL, version 2.
 *
 * Simple descriptor-based ring. virtio 0.9 compatible event index is used for
 * signalling, unconditionally.
 */
#define _GNU_SOURCE
#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define DEBUG 0
#define DPRINT(fmt, ...) \
	do { if (DEBUG) printf(fmt, __VA_ARGS__); } while (0)


/* Next - Where next entry will be written.
 * Prev - "Next" value when event triggered previously.
 * Event - Peer requested event after writing this entry.
 */
static inline bool need_event(unsigned short event,
			      unsigned short next,
			      unsigned short prev)
{
	return (unsigned short)(next - event - 1) < (unsigned short)(next - prev);
}

/* Design:
 * Guest adds descriptors with unique index values and DESC_HW in flags.
 * Host overwrites used descriptors with correct len, index, and DESC_HW clear.
 * Flags are always set last.
 */
#define DESC_HW   0x80
#define DESC_WRAP 0x40

struct desc {
	unsigned long long addr;
	unsigned len;
	unsigned short index;
	unsigned short flags;
};

/* how much padding is needed to avoid false cache sharing */
#define HOST_GUEST_PADDING 0x80

/* Mostly read */
struct event {
	unsigned short kick_index;
	unsigned char reserved0[HOST_GUEST_PADDING - 2];
	unsigned short call_index;
	unsigned char reserved1[HOST_GUEST_PADDING - 2];
};

struct data {
	void *buf; /* descriptor is writeable, we can't get buf from there */
	void *data;
} *data;

struct desc *ring;
struct event *event;

struct guest {
	unsigned short wrap;
	unsigned avail_idx;
	unsigned last_used_idx;
	unsigned num_free;
	unsigned kicked_avail_idx;
	unsigned char reserved[HOST_GUEST_PADDING - 12];
} guest;

struct host {
	/* we do not need to track last avail index
	 * unless we have more than one in flight.
	 */
	unsigned short wrap;
	unsigned used_idx;
	unsigned called_used_idx;
	unsigned char reserved[HOST_GUEST_PADDING - 4];
} host;

/* implemented by ring */
void alloc_ring(void)
{
	int ret;
	int i;

	ret = posix_memalign((void **)&ring, 0x1000, ring_size * sizeof *ring);
	if (ret) {
		perror("Unable to allocate ring buffer.\n");
		exit(3);
	}
	event = malloc(sizeof *event);
	if (!event) {
		perror("Unable to allocate event buffer.\n");
		exit(3);
	}
	memset(event, 0, sizeof *event);
	guest.avail_idx = 0;
	guest.kicked_avail_idx = -1;
	guest.last_used_idx = 0;
	guest.wrap = 0x40;
	host.used_idx = 0;
	host.called_used_idx = -1;
	host.wrap = 0x40;
	for (i = 0; i < ring_size; ++i) {
		struct desc desc = {
			.index = i,
		};
		ring[i] = desc;
	}
	guest.num_free = ring_size;
	data = malloc(ring_size * sizeof *data);
	if (!data) {
		perror("Unable to allocate data buffer.\n");
		exit(3);
	}
	memset(data, 0, ring_size * sizeof *data);
}


/* guest side */
int add_inbuf(unsigned len, void *buf, void *datap, unsigned short flags)
{
	unsigned head, index;

	if (!guest.num_free) {
		DPRINT("add_inbuf: ring full, guest.last_used=%d, guest.avail_idx=%d\n", guest.last_used_idx,
				guest.avail_idx);
		return -1;
	}

	guest.num_free--;
	head = (ring_size - 1) & (guest.avail_idx++);
	if (head == 0) {
		if (guest.wrap == 0x40)
			guest.wrap = 0;
		else if (guest.wrap == 0)
			guest.wrap = 0x40;
	}

	DPRINT("wrap is %d\n", guest.wrap);
	DPRINT("add_inbuf: add to buf head = %d, guest_avail_idx now %d\n", head, guest.avail_idx);

	/* Start with a write. On MESI architectures this helps
	 * avoid a shared state with consumer that is polling this descriptor.
	 */
	ring[head].addr = (unsigned long)(void*)buf;
	ring[head].len = len;
	/* read below might bypass write above. That is OK because it's just an
	 * optimization. If this happens, we will get the cache line in a
	 * shared state which is unfortunate, but probably not worth it to
	 * add an explicit full barrier to avoid this.
	 */
	barrier();
	index = ring[head].index;
	data[index].buf = buf;
	data[index].data = datap;
	/* Barrier A (for pairing) */
	smp_release();
	ring[head].flags = DESC_HW | guest.wrap;
	DPRINT("ADD_inbuf: write flags %x, head idx = %d\n", ring[head].flags, head);
	return 0;
}

void *get_buf(unsigned *lenp, void **bufp, unsigned short *flags)
{
	unsigned head = (ring_size - 1) & guest.last_used_idx;
	unsigned index;
	void *datap;

	if (ring[head].flags & DESC_HW) {
		DPRINT("get_buf: belongs to device head idx %d\n", head);
		return NULL;
	}
	DPRINT("get_buf: guest.last_used_idx = %d\n", guest.last_used_idx);
	/* Barrier B (for pairing) */
	smp_acquire();
	*lenp = ring[head].len;
	index = ring[head].index & (ring_size - 1);
	datap = data[index].data;
	*bufp = data[index].buf;
	data[index].buf = NULL;
	data[index].data = NULL;
	guest.num_free++;
	guest.last_used_idx++;
	*flags = ring[head].flags;
	return datap;
}

bool used_empty()
{
	unsigned head = (ring_size - 1) & guest.last_used_idx;

	return (ring[head].flags & DESC_HW) && ((ring[head].flags & DESC_WRAP) == host.wrap);
}

void disable_call()
{
	/* Doing nothing to disable calls might cause
	 * extra interrupts, but reduces the number of cache misses.
	 */
}

bool enable_call()
{
	event->call_index = guest.last_used_idx;
	/* Flush call index write */
	/* Barrier D (for pairing) */
	smp_mb();
	return used_empty();
}

void kick_available(void)
{
	/* Flush in previous flags write */
	/* Barrier C (for pairing) */
	smp_mb();
	if (!need_event(event->kick_index,
			guest.avail_idx,
			guest.kicked_avail_idx))
		return;

	guest.kicked_avail_idx = guest.avail_idx;
	kick();
}

/* host side */
void disable_kick()
{
	/* Doing nothing to disable kicks might cause
	 * extra interrupts, but reduces the number of cache misses.
	 */
}

bool enable_kick()
{
	event->kick_index = host.used_idx;
	/* Barrier C (for pairing) */
	smp_mb();
	return avail_empty();
}

bool avail_empty()
{
	unsigned head = (ring_size - 1) & host.used_idx;

	return !(ring[head].flags & DESC_HW);
}

bool use_buf(unsigned *lenp, void **bufp, unsigned short *flags)
{
	unsigned head = (ring_size - 1) & host.used_idx;

	/* Yes this can be done shorter, but I like clarity */
	if (head == 0) {
		if (host.wrap == DESC_WRAP)
			host.wrap = 0;
		else if (host.wrap == 0)
			host.wrap = DESC_WRAP;
	}
	if (!(ring[head].flags & DESC_HW)) 
		return false;

	if ((ring[head].flags & DESC_WRAP) != host.wrap ) 
		return false;

	DPRINT("use_buf: head idx=%d\n", head);
	/* make sure length read below is not speculated */
	/* Barrier A (for pairing) */
	smp_acquire();

	/* simple in-order completion: we don't need
	 * to touch index at all. This also means we
	 * can just modify the descriptor in-place.
	 */
	ring[head].len--;
	/* Make sure len is valid before flags.
	 * Note: alternative is to write len and flags in one access -
	 * possible on 64 bit architectures but wmb is free on Intel anyway
	 * so I have no way to test whether it's a gain.
	 */
	/* Barrier B (for pairing) */
	*flags = ring[head].flags;
	smp_release();
	if (host.wrap )
		ring[head].flags = DESC_WRAP;
	else
		ring[head].flags = 0;
	host.used_idx++;
	return true;
}

void call_used(void)
{
	/* Flush in previous flags write */
	/* Barrier D (for pairing) */
	smp_mb();
	if (!need_event(event->call_index,
			host.used_idx,
			host.called_used_idx))
		return;

	host.called_used_idx = host.used_idx;
	call();
}
