/*
 * Device independent Thunderbolt bus logic
 *
 * Copyright (c) 2013 Andreas Noever <andreas.noever@gmail.com>
 */

#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include "tb.h"

struct tb_hotplug_event {
	struct work_struct work;
	struct tb *tb;
	u64 route;
	u8 port;
	bool unplug;
};

/**
 * tb_handle_hotplug() - handle hotplug event
 *
 * Executes on the tb->wq.
 */
static void tb_handle_hotplug(struct work_struct *work)
{
	struct tb_hotplug_event *ev = container_of(work, typeof(*ev), work);
	struct tb *tb = ev->tb;
	mutex_lock(&tb->lock);
	if (tb->shutdown)
		goto out;
	/* do nothing for now */
out:
	mutex_unlock(&tb->lock);
	kfree(ev);
}

/**
 * tb_schedule_hotplug_handler() - callback function for the config channel
 *
 * Delegates to tb_handle_hotplug.
 */
static void tb_schedule_hotplug_handler(void *data, u64 route, u8 port,
					bool unplug)
{
	struct tb *tb = data;
	struct tb_hotplug_event *ev = kmalloc(sizeof(*ev), GFP_KERNEL);
	if (!ev)
		return;
	INIT_WORK(&ev->work, tb_handle_hotplug);
	ev->tb = tb;
	ev->route = route;
	ev->port = port;
	ev->unplug = unplug;
	queue_work(tb->wq, &ev->work);
}

/**
 * thunderbolt_shutdown_and_free() - shutdown everything
 *
 * Free the config channel.
 */
void thunderbolt_shutdown_and_free(struct tb *tb)
{
	mutex_lock(&tb->lock);
	tb->shutdown = true; /* signal tb_handle_hotplug to quit */

	if (tb->cfg)
		tb_cfg_free(tb->cfg);
	tb->cfg = NULL;

	/* allow tb_handle_hotplug to acquire the lock */
	mutex_unlock(&tb->lock);
	if (tb->wq) {
		flush_workqueue(tb->wq);
		destroy_workqueue(tb->wq);
		tb->wq = NULL;
	}
	mutex_destroy(&tb->lock);
	kfree(tb);
}

/**
 * thunderbolt_alloc_and_start() - setup the thunderbolt bus
 *
 * Allocates a tb_cfg control channel.
 *
 * Return: Returns NULL on error.
 */
struct tb *thunderbolt_alloc_and_start(struct tb_nhi *nhi)
{
	struct tb *tb;

	tb = kzalloc(sizeof(*tb), GFP_KERNEL);
	if (!tb)
		return NULL;

	tb->nhi = nhi;
	mutex_init(&tb->lock);
	mutex_lock(&tb->lock);

	tb->wq = alloc_ordered_workqueue("thunderbolt", 0);
	if (!tb->wq)
		goto err_locked;

	/*
	 * tb_schedule_hotplug_handler may be called as soon as the config
	 * channel is allocated. Thats why we have to hold the lock here.
	 */
	tb->cfg = tb_cfg_alloc(tb->nhi, tb_schedule_hotplug_handler, tb);
	if (!tb->cfg)
		goto err_locked;

	mutex_unlock(&tb->lock);
	return tb;

err_locked:
	/* In case tb_handle_hotplug is already executing. */
	tb->shutdown = true;
	mutex_unlock(&tb->lock);
	thunderbolt_shutdown_and_free(tb);
	return NULL;
}
