/*
 * Cactus Ridge NHI driver
 *
 * Copyright (c) 2013 Andreas Noever <andreas.noever@gmail.com>
 */

#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include "dsl3510.h"
#include "dsl3510_regs.h"

#define RING_TYPE(ring) ((ring)->is_tx ? "TX ring" : "RX ring")

static int get_interrupt_index(struct tb_ring *ring)
{
	int bit = ring->hop;
	if (!ring->is_tx)
		bit += ring->nhi->hop_count;
	return bit;
}

/**
 * interrupt_active() - activate/deactivate interrupts for a single ring
 *
 * ring->nhi->lock must be held.
 */
static void interrupt_active(struct tb_ring *ring, bool active)
{
	int reg = get_interrupt_index(ring) / 32 + REG_RING_INTERRUPT_BASE;
	int bit = get_interrupt_index(ring) & 31;
	u32 value;
	value = ioread32(ring->nhi->iobase + reg);
	if (active) {
		if (value & (1 << bit)) {
			dev_WARN(&ring->nhi->pdev->dev,
				 "interrupt for %s %d is already enabled\n",
				 RING_TYPE(ring),
				 ring->hop);
		}
		value |= 1 << bit;
	} else {
		if (!(value & (1 << bit))) {
			dev_WARN(&ring->nhi->pdev->dev,
				 "interrupt for %s %d is already disabled\n",
				 RING_TYPE(ring),
				 ring->hop);
		}
		value &= ~(1 << bit);
	}
	dev_info(&ring->nhi->pdev->dev,
		 "%s interrupt at register %#x bit %d (new value: %#x)\n",
		 active ? "enabling" : "disabling",
		 reg,
		 bit,
		 value);
	iowrite32(value, ring->nhi->iobase + reg);
}

/**
 * dsl3510_disable_interrupts() - disable interrupts for all rings
 */
static void dsl3510_disable_interrupts(struct tb_nhi *nhi)
{
	int i = 0;
	/* disable interrupts */
	for (i = 0; i < RING_INTERRUPT_REG_COUNT(nhi); i++)
		iowrite32(0, nhi->iobase + REG_RING_INTERRUPT_BASE + 4 * i);

	/* clear interrupt status bits */
	for (i = 0; i < RING_NOTIFY_REG_COUNT(nhi); i++)
		ioread32(nhi->iobase + REG_RING_NOTIFY_BASE + 4 * i);
}

static void __iomem *ring_desc_base(struct tb_ring *ring)
{
	void __iomem *io = ring->nhi->iobase;
	io += ring->is_tx ? REG_TX_RING_BASE : REG_RX_RING_BASE;
	io += ring->hop * 16;
	return io;
}

static void __iomem *ring_options_base(struct tb_ring *ring)
{
	void __iomem *io = ring->nhi->iobase;
	io += ring->is_tx ? REG_TX_OPTIONS_BASE : REG_RX_OPTIONS_BASE;
	io += ring->hop * 32;
	return io;
}

static void ring_iowrite16desc(struct tb_ring *ring, u32 value, u32 offset)
{
	iowrite16(value, ring_desc_base(ring) + offset);
}

static void ring_iowrite32desc(struct tb_ring *ring, u32 value, u32 offset)
{
	iowrite32(value, ring_desc_base(ring) + offset);
}

static void ring_iowrite64desc(struct tb_ring *ring, u64 value, u32 offset)
{
	iowrite32(value, ring_desc_base(ring) + offset);
	iowrite32(value >> 32, ring_desc_base(ring) + offset + 4);
}

static void ring_iowrite32options(struct tb_ring *ring, u32 value, u32 offset)
{
	iowrite32(value, ring_options_base(ring) + offset);
}

static bool ring_full(struct tb_ring *ring)
{
	return ((ring->head + 1) % ring->size) == ring->tail;
}

static bool ring_empty(struct tb_ring *ring)
{
	return ring->head == ring->tail;
}

/* ring->lock is held. */
static void ring_write_descriptors(struct tb_ring *ring)
{
	struct ring_packet *pkg, *n;
	struct ring_desc *descriptor;
	list_for_each_entry_safe(pkg, n, &ring->queue, list) {
		if (ring_full(ring))
			break;
		list_move_tail(&pkg->list, &ring->in_flight);
		descriptor = &ring->descriptors[ring->head];
		descriptor->phys = pkg->buffer_phy;
		descriptor->time = 0;
		descriptor->flags = RING_DESC_POSTED | RING_DESC_INTERRUPT;
		if (ring->is_tx) {
			descriptor->length = pkg->size;
			descriptor->eof = pkg->eof;
			descriptor->sof = pkg->sof;
		}
		ring->head = (ring->head + 1) % ring->size;
		ring_iowrite16desc(ring, ring->head, ring->is_tx ? 10 : 8);
	}
}

static void ring_handle_interrupt(struct work_struct *work)
{
	struct tb_ring *ring = container_of(work, typeof(*ring), work);
	struct ring_packet *pkg, *n;
	bool invoke_callback = false;
	mutex_lock(&ring->lock);
	if (ring->in_shutdown)
		goto out;
	list_for_each_entry_safe(pkg, n, &ring->in_flight, list) {
		if (ring_empty(ring))
			break;
		if (!(ring->descriptors[ring->tail].flags & RING_DESC_COMPLETED))
			break;
		list_move_tail(&pkg->list, &ring->done);
		invoke_callback = true;
		if (!ring->is_tx) {
			pkg->size = ring->descriptors[ring->tail].length;
			pkg->eof = ring->descriptors[ring->tail].eof;
			pkg->sof = ring->descriptors[ring->tail].sof;
			pkg->flags = ring->descriptors[ring->tail].flags;
			if (pkg->sof != 0)
				dev_WARN(&ring->nhi->pdev->dev,
					 "%s %d got unexpected SOF: %#x\n",
					 RING_TYPE(ring),
					 ring->hop,
					 pkg->sof);

			/*
			 * known flags:
			 * raw enabled: 0xa
			 * raw not enabled: 0xb
			 * partial packet (>MAX_FRAME_SIZE): 0xe
			 */
			if (pkg->flags != 0xa && pkg->flags != 0xb)
				dev_WARN(&ring->nhi->pdev->dev,
					 "%s %d got unexpected flags: %#x\n",
					 RING_TYPE(ring),
					 ring->hop,
					 pkg->flags);
		}
		ring->tail = (ring->tail + 1) % ring->size;
	}

	ring_write_descriptors(ring);
out:
	mutex_unlock(&ring->lock); /* allow the callback to queue new work */
	if (invoke_callback)
		ring->callback(ring, ring->callback_data);
}

void __ring_enqueue(struct tb_ring *ring, struct ring_packet *pkg)
{
	mutex_lock(&ring->lock);
	if (ring->in_shutdown) {
		pkg->canceled = true;
		list_add_tail(&pkg->list, &ring->done);
	} else {
		pkg->canceled = false;
		list_add_tail(&pkg->list, &ring->queue);
		ring_write_descriptors(ring);
	}
	mutex_unlock(&ring->lock);
}

/**
 * ring_poll() - return completed packet if any
 *
 * Return: Returns the first completed packet from the done queue. Returns NULL
 * if the queue is empty.
 */
struct ring_packet *ring_poll(struct tb_ring *ring)
{
	struct ring_packet *pkg = NULL;
	mutex_lock(&ring->lock);
	if (!list_empty(&ring->done)) {
		pkg = list_entry(ring->done.next, typeof(*pkg), list);
		list_del(&pkg->list);
	}
	mutex_unlock(&ring->lock);
	return pkg;
}

static struct tb_ring *ring_alloc(struct tb_nhi *nhi, u32 hop, int size,
				  ring_cb callback, void *callback_data,
				  bool transmit)
{
	struct tb_ring *ring = NULL;
	dev_info(&nhi->pdev->dev,
		 "allocating %s ring %d\n",
		 transmit ? "TX" : "RX",
		 hop);
	mutex_lock(&nhi->lock);
	if (hop >= nhi->hop_count) {
		dev_WARN(&nhi->pdev->dev, "invalid hop: %d\n", hop);
		goto err;
	}
	if (transmit && nhi->tx_rings[hop]) {
		dev_WARN(&nhi->pdev->dev, "TX hop %d already allocated\n", hop);
		goto err;
	} else if (!transmit && nhi->rx_rings[hop]) {
		dev_WARN(&nhi->pdev->dev, "RX hop %d already allocated\n", hop);
		goto err;
	}
	ring = kzalloc(sizeof(*ring), GFP_KERNEL);
	if (!ring)
		goto err;

	mutex_init(&ring->lock);
	ring->nhi = nhi;
	ring->size = size;
	ring->hop = hop;
	ring->head = 0;
	ring->tail = 0;
	ring->descriptors = dma_alloc_coherent(&ring->nhi->pdev->dev,
					       size * sizeof(*ring->descriptors),
					       &ring->descriptors_dma,
					       GFP_KERNEL | __GFP_ZERO);
	if (!ring->descriptors)
		goto err;

	ring->callback = callback;
	ring->callback_data = callback_data;
	ring->is_tx = transmit;
	ring->in_shutdown = false;
	INIT_LIST_HEAD(&ring->queue);
	INIT_LIST_HEAD(&ring->in_flight);
	INIT_LIST_HEAD(&ring->done);
	INIT_WORK(&ring->work, ring_handle_interrupt);

	if (transmit)
		nhi->tx_rings[hop] = ring;
	else
		nhi->rx_rings[hop] = ring;

	ring_iowrite64desc(ring, ring->descriptors_dma, 0);
	if (ring->is_tx) {
		ring_iowrite32desc(ring, size, 12);
		ring_iowrite32options(ring, 0, 4); /* time releated ? */
		ring_iowrite32options(ring,
				      RING_FLAG_ENABLE | RING_FLAG_RAW,
				      0);
	} else {
		ring_iowrite32desc(ring, (TB_FRAME_SIZE << 16) | size, 12);
		ring_iowrite32options(ring, 0xffffffff, 4); /* SOF EOF mask */
		ring_iowrite32options(ring,
				      RING_FLAG_ENABLE | RING_FLAG_RAW,
				      0);
	}
	interrupt_active(ring, true);
	mutex_unlock(&nhi->lock);
	return ring;
err:
	mutex_unlock(&nhi->lock);
	if (ring)
		mutex_destroy(&ring->lock);
	kfree(ring);
	return NULL;
}

struct tb_ring *ring_alloc_tx(struct tb_nhi *nhi, int hop, int size,
			      ring_cb callback, void *callback_data)
{
	return ring_alloc(nhi, hop, size, callback, callback_data, true);
}

struct tb_ring *ring_alloc_rx(struct tb_nhi *nhi, int hop, int size,
			      ring_cb callback, void *callback_data)
{
	return ring_alloc(nhi, hop, size, callback, callback_data, false);
}

/**
 * ring_drain_and_free() - shutdown a ring
 *
 * This method will disable the ring, cancel all packets and repeatedly
 * call the callback function until all packets have been drained through
 * ring_poll.
 *
 * When this method returns all invocations of ring->callback will have
 * finished.
 *
 * Must NOT be called from ring->callback!
 */
void ring_drain_and_free(struct tb_ring *ring)
{
	struct ring_packet *pkg;
	dev_info(&ring->nhi->pdev->dev,
		 "stopping %s %d\n",
		 RING_TYPE(ring),
		 ring->hop);

	mutex_lock(&ring->nhi->lock);
	mutex_lock(&ring->lock);
	if (ring->in_shutdown) {
		dev_WARN(&ring->nhi->pdev->dev,
			 "%s %d already in_shutdown!\n",
			 RING_TYPE(ring),
			 ring->is_tx);
		mutex_unlock(&ring->nhi->lock);
		mutex_unlock(&ring->lock);
		return;
	}
	ring->in_shutdown = true;

	interrupt_active(ring, false);
	if (ring->is_tx)
		ring->nhi->tx_rings[ring->hop] = NULL;
	else
		ring->nhi->rx_rings[ring->hop] = NULL;

	ring_iowrite32options(ring, 0, 0);
	ring_iowrite64desc(ring, 0, 0);
	ring_iowrite16desc(ring, 0, ring->is_tx ? 10 : 8);
	ring_iowrite32desc(ring, 0, 12);
	dma_free_coherent(&ring->nhi->pdev->dev,
			  ring->size * sizeof(*ring->descriptors),
			  ring->descriptors,
			  ring->descriptors_dma);
	ring->descriptors = 0;
	ring->descriptors_dma = 0;
	ring->head = 0;
	ring->tail = 0;
	ring->size = 0;

	/* Move all packets to the done queue and mark them as canceled. */
	list_for_each_entry(pkg, &ring->in_flight, list)
		pkg->canceled = true;
	list_for_each_entry(pkg, &ring->queue, list)
		pkg->canceled = true;
	list_splice_tail(&ring->in_flight, &ring->done);
	list_splice_tail(&ring->queue, &ring->done);

	mutex_unlock(&ring->lock);
	mutex_unlock(&ring->nhi->lock);
	/*
	 * From this point on ring->work will not get scheduled again. Any new
	 * packets added to the ring will be directly canceled and moved to the
	 * done queue. The in_flight and queue lists will remain empty.
	 */
	cancel_work_sync(&ring->work);

	mutex_lock(&ring->lock);
	while (!list_empty(&ring->done)) {
		mutex_unlock(&ring->lock);
		ring->callback(ring, ring->callback_data);
		mutex_lock(&ring->lock);
	}
	mutex_unlock(&ring->lock);
	mutex_destroy(&ring->lock);
	kfree(ring);
}

static void dsl3510_handle_interrupt(struct work_struct *work)
{
	struct tb_nhi *nhi = container_of(work, typeof(*nhi), interrupt_task);
	int value = 0; /* Suppress uninitialized usage warning. */
	int bit;
	int hop = -1;
	int type = 0; /* current interrupt type 0: TX, 1: RX, 2: RX overflow */
	struct tb_ring *ring;

	mutex_lock(&nhi->lock);

	/*
	 * Starting at REG_RING_NOTIFY_BASE there are three status bitfields
	 * (TX, RX, RX overflow). We iterate over the bits and read a new
	 * dwords as required. The registers are cleared on read.
	 */
	for (bit = 0; bit < 3 * nhi->hop_count; bit++) {
		if (bit % 32 == 0)
			value = ioread32(nhi->iobase
					 + REG_RING_NOTIFY_BASE
					 + 4 * (bit / 32));
		if (++hop == nhi->hop_count) {
			hop = 0;
			type++;
		}
		if ((value & (1 << (bit % 32))) == 0)
			continue;
		if (type == 2) {
			dev_warn(&nhi->pdev->dev,
				 "RX overflow for ring %d\n",
				 hop);
			continue;
		}
		if (type == 0)
			ring = nhi->tx_rings[hop];
		else
			ring = nhi->rx_rings[hop];
		if (ring == NULL) {
			dev_warn(&nhi->pdev->dev,
				 "got interrupt for inactive %s ring %d\n",
				 type ? "RX" : "TX",
				 hop);
			continue;
		}
		schedule_work(&ring->work);
	}
	mutex_unlock(&nhi->lock);
}

static irqreturn_t dsl3510_msi(int irq, void *data)
{
	struct tb_nhi *nhi = data;
	schedule_work(&nhi->interrupt_task);
	return IRQ_HANDLED;
}

static void dsl3510_shutdown(struct tb_nhi *nhi)
{
	int i;
	dev_info(&nhi->pdev->dev, "shutdown\n");

	for (i = 0; i < nhi->hop_count; i++) {
		if (nhi->tx_rings[i])
			dev_WARN(&nhi->pdev->dev,
				 "TX ring %d is still active\n",
				 i);
		if (nhi->rx_rings[i])
			dev_WARN(&nhi->pdev->dev,
				 "RX ring %d is still active\n",
				 i);
	}
	dsl3510_disable_interrupts(nhi);
	/*
	 * We have to release the irq before calling flush_work. Otherwise an
	 * already executing IRQ handler could call schedule_work again.
	 */
	devm_free_irq(&nhi->pdev->dev, nhi->pdev->irq, nhi);
	flush_work(&nhi->interrupt_task);
	mutex_destroy(&nhi->lock);
}

static int dsl3510_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct tb_nhi *nhi;
	int res;

	res = pcim_enable_device(pdev);
	if (res) {
		dev_err(&pdev->dev, "cannot enable PCI device, aborting\n");
		return res;
	}

	res = pci_enable_msi(pdev);
	if (res) {
		dev_err(&pdev->dev, "cannot enable MSI, aborting\n");
		return res;
	}

	res = pcim_iomap_regions(pdev, 1 << 0, "DSL3510");
	if (res) {
		dev_err(&pdev->dev, "cannot obtain PCI resources, aborting\n");
		return res;
	}

	nhi = devm_kzalloc(&pdev->dev, sizeof(*nhi), GFP_KERNEL);
	if (!nhi)
		return -ENOMEM;

	nhi->pdev = pdev;
	/* cannot fail - table is allocated bin pcim_iomap_regions */
	nhi->iobase = pcim_iomap_table(pdev)[0];
	nhi->hop_count = ioread32(nhi->iobase + REG_HOP_COUNT) & 0x3ff;
	if (nhi->hop_count != 12)
		dev_warn(&pdev->dev,
			 "unexpected hop count: %d\n",
			 nhi->hop_count);
	INIT_WORK(&nhi->interrupt_task, dsl3510_handle_interrupt);

	nhi->tx_rings = devm_kzalloc(&pdev->dev,
				     nhi->hop_count * sizeof(struct tb_ring),
				     GFP_KERNEL);
	nhi->rx_rings = devm_kzalloc(&pdev->dev,
				     nhi->hop_count * sizeof(struct tb_ring),
				     GFP_KERNEL);
	if (!nhi->tx_rings || !nhi->rx_rings)
		return -ENOMEM;

	dsl3510_disable_interrupts(nhi); /* In case someone left them on. */
	res = devm_request_irq(&pdev->dev,
			       pdev->irq,
			       dsl3510_msi,
			       0,
			       "dsl3510",
			       nhi);
	if (res) {
		dev_err(&pdev->dev, "request_irq failed, aborting\n");
		return res;
	}

	mutex_init(&nhi->lock);

	pci_set_master(pdev);

	/* magic value - clock related? */
	iowrite32(3906250 / 10000, nhi->iobase + 0x38c00);

	pci_set_drvdata(pdev, nhi); /* for dsl3510_remove only */

	return 0;
}

static void dsl3510_remove(struct pci_dev *pdev)
{
	struct tb_nhi *nhi = pci_get_drvdata(pdev);
	dsl3510_shutdown(nhi);
}

static DEFINE_PCI_DEVICE_TABLE(dsl3510_ids) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1547)},
	{ 0,}
};

MODULE_DEVICE_TABLE(pci, dsl3510_ids);

static struct pci_driver dsl3510_driver = {
	.name = "dsl3510",
	.id_table = dsl3510_ids,
	.probe = dsl3510_probe,
	.remove = dsl3510_remove,
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andreas Noever <andreas.noever@gmail.com>");

static int __init dsl3510_init(void)
{
	struct pci_dev *dev = pci_get_device(PCI_VENDOR_ID_INTEL, 0x1547, NULL);
	if (dev)
		pci_dev_put(dev);
	else
		printk(KERN_INFO "thunderbolt controller not found, try booting with acpi_osi=Darwin");

	return pci_register_driver(&dsl3510_driver);
}

static void __exit dsl3510_unload(void)
{
	pci_unregister_driver(&dsl3510_driver);
}

module_init(dsl3510_init);
module_exit(dsl3510_unload);
