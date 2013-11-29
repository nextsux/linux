/*
 * Cactus Ridge NHI driver
 *
 * Copyright (c) 2013 Andreas Noever <andreas.noever@gmail.com>
 */

#ifndef DSL3510_H_
#define DSL3510_H_

#include <linux/mutex.h>
#include <linux/workqueue.h>

/**
 * struct tb_nhi - thunderbolt native host interface
 */
struct tb_nhi {
	struct mutex lock; /*
			    * Must be held during ring creation/destruction.
			    * Is acquired by interrupt_task to when dispatching
			    * interrupts to individual rings.
			    **/
	struct pci_dev *pdev;
	void __iomem *iobase;
	struct tb_ring **tx_rings;
	struct tb_ring **rx_rings;
	struct work_struct interrupt_task;
	u32 hop_count; /* Number of rings (end point hops) supported by NHI. */
};

typedef void (*ring_cb)(struct tb_ring*, void*);

/**
 * struct tb_ring - thunderbolt TX or RX ring associated with a NHI
 *
 * Will invoke callback once data is available. You should then call ring_poll
 * repeatedly to retrieve all completed (or canceled) packets.
 */
struct tb_ring {
	struct mutex lock; /* must be acquired after nhi->lock */
	struct tb_nhi *nhi;
	int size;
	int hop;
	int head; /* write next descriptor here */
	int tail; /* complete next descriptor here */
	struct ring_desc *descriptors;
	dma_addr_t descriptors_dma;
	ring_cb callback;
	void *callback_data;
	struct list_head queue;
	struct list_head in_flight;
	struct list_head done;
	struct work_struct work;
	bool is_tx;
	bool in_shutdown;
};

/**
 * struct ring_packet - packet for use with ring_rx/ring_tx
 */
struct ring_packet {
	void *buffer;
	dma_addr_t buffer_phy;
	struct list_head list;
	u32 size:12;
	u16 flags:12;
	u32 eof:4;
	u32 sof:4;
	u8 canceled:1;
};

#define TB_FRAME_SIZE 0x100    /* minimum size for ring_rx */

struct tb_ring *ring_alloc_tx(struct tb_nhi *nhi, int hop, int size,
			      ring_cb callback, void *callback_data);
struct tb_ring *ring_alloc_rx(struct tb_nhi *nhi, int hop, int size,
			      ring_cb callback, void *callback_data);
void ring_drain_and_free(struct tb_ring *ring);

void __ring_enqueue(struct tb_ring *ring, struct ring_packet *pkg);

/**
 * ring_rx() - enqueue a packet on an RX ring
 *
 * If the ring is about to shut down the packet will be marked as canceled and
 * put on the done queue.
 *
 * pkg->buffer and pkg->buffer_phy have to be set. The buffer must contain at
 * least TB_FRAME_SIZE bytes. After pkg has been handled and retrieved through
 * ring_poll pkg->canceled, pkg->size, pkg->eof, pkg->sof and pkg->flags will
 * be set.
 */
static inline void ring_rx(struct tb_ring *ring, struct ring_packet *pkg)
{
	WARN_ON(ring->is_tx);
	__ring_enqueue(ring, pkg);
}

/**
 * ring_tx() - enqueue a packet on an TX ring
 *
 * If the ring is about to shut down the packet will be marked as canceled and
 * put on the done queue.
 *
 * pkg->buffer, pkg->buffer_phy, pkg->size, pkg->eof, pkg->sof have to be set.
 * After pkg has been handled and retrieved through ring_poll pkg->canceled
 * will be set.
 */
static inline void ring_tx(struct tb_ring *ring, struct ring_packet *pkg)
{
	WARN_ON(!ring->is_tx);
	__ring_enqueue(ring, pkg);
}

struct ring_packet *ring_poll(struct tb_ring *ring);

#endif
