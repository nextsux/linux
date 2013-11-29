/*
 * Device independent Thunderbolt bus logic
 *
 * Copyright (c) 2013 Andreas Noever <andreas.noever@gmail.com>
 */

#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include "dsl3510.h"
#include "tb.h"
#include "tb_regs.h"

/* utility functions */

static void tb_dump_switch(struct tb *tb, struct tb_regs_switch_header *sw)
{
	tb_info(tb,
		" Switch: %x:%x (Revision: %d, TB Version: %d)\n",
		sw->vendor_id,
		sw->device_id,
		sw->revision,
		sw->thunderbolt_version);
	tb_info(tb, "  Max Port Number: %d\n", sw->max_port_number);
	tb_info(tb, "  Config:\n");
	tb_info(tb,
		"   Upstream Port Number: %d Depth: %d Route String: %#llx Enabled: %d, PlugEventsDelay: %dms\n",
		sw->upstream_port_number,
		sw->depth,
		(((u64) sw->route_hi) << 32) | sw->route_lo,
		sw->enabled,
		sw->plug_events_delay);
	tb_info(tb,
		"   unknown1: %#x unknown4: %#x\n",
		sw->__unknown1,
		sw->__unknown4);
}

static char *tb_port_type(struct tb_regs_port_header *port)
{
	switch (port->type >> 16) {
	case 0:
		switch ((u8) port->type) {
		case 0:
			return "Inactive";
		case 1:
			return "Port";
		case 2:
			return "NHI";
		default:
			return "unknown";
		}
	case 0x2:
		return "Ethernet";
	case 0x8:
		return "SATA";
	case 0xe:
		return "DP/HDMI";
	case 0x10:
		return "PCIe";
	case 0x20:
		return "USB";
	default:
		return "unknown";
	}
}

static void tb_dump_port(struct tb *tb, struct tb_regs_port_header *port)
{
	tb_info(tb,
		" Port %d: %x:%x (Revision: %d, TB Version: %d, Type: %s (%#x))\n",
		port->port_number,
		port->vendor_id,
		port->device_id,
		port->revision,
		port->thunderbolt_version,
		tb_port_type(port),
		port->type);
	tb_info(tb,
		"  Max hop id (in/out): %d/%d, NFC Credits: %#x\n",
		port->max_in_hop_id,
		port->max_out_hop_id,
		port->nfc_credits);
}

static int tb_route_length(u64 route)
{
	return (fls64(route) + TB_ROUTE_SHIFT - 1) / TB_ROUTE_SHIFT;
}


/* thunderbolt capability lookup */

struct tb_cap_any {
	union {
		struct tb_cap_basic basic;
		struct tb_cap_extended_short extended_short;
		struct tb_cap_extended_long extended_long;
	};
} __packed;

static bool tb_cap_is_basic(struct tb_cap_any *cap)
{
	/* basic.cap is u8. This checks only the lower 8 bit of cap. */
	return cap->basic.cap != 5;
}

static bool tb_cap_is_long(struct tb_cap_any *cap)
{
	return !tb_cap_is_basic(cap)
	       && cap->extended_short.next == 0
	       && cap->extended_short.length == 0;
}

static enum tb_cap tb_cap(struct tb_cap_any *cap)
{
	if (tb_cap_is_basic(cap))
		return cap->basic.cap;
	else
		/* extended_short/long have cap at the same position. */
		return cap->extended_short.cap;
}

static u32 tb_cap_next(struct tb_cap_any *cap, u32 offset)
{
	int next;
	if (offset == 1) {
		/*
		 * The first pointer is part of the switch header and always
		 * a simple pointer.
		 */
		next = cap->basic.next;
	} else {
		if (tb_cap_is_basic(cap))
			next = cap->basic.next;
		/* "255 byte config areas should be enough for anybody." */
		else if (!tb_cap_is_long(cap))
			next = cap->extended_short.next;
		/*
		 * "Also we should have at least three types of capability
		 *  headers in version 1."
		 */
		else
			next = cap->extended_long.next;
	}
	/*
	 * "Hey, we could terminate some capability lists with a null offset
	 *  and others with a pointer to the last element." - "Great idea!"
	 */
	if (next == offset)
		return 0;
	return next;
}

/**
 * tb_find_cap() - find a capability
 *
 * Return: Returns a positive offset if the capability was found and 0 if not.
 * Returns an error code on failure.
 */
int tb_find_cap(struct tb_port *port, enum tb_cfg_space space, enum tb_cap cap)
{
	u32 offset = 1;
	struct tb_cap_any header;
	int res;
	int retries = 10;
	while (retries--) {
		res = tb_port_read(port, &header, space, offset, 1);
		if (res) {
			/* Intel needs some help with linked lists. */
			if (space == TB_CFG_PORT
			    && offset == 0xa
			    && port->config.type == TB_TYPE_DP_HDMI_OUT) {
				offset = 0x39;
				continue;
			}
			return res;
		}
		if (offset != 1 && tb_cap(&header) == cap)
			return offset;
		offset = tb_cap_next(&header, offset);
		if (!offset)
			return 0;
		continue;
	}
	tb_port_WARN(port,
		     "could not find cap %#x in config space %d, last offset: %#x\n",
		     cap,
		     space,
		     offset);
	return -EIO;
}


/* thunderbolt switch utility functions */

/**
 * tb_plug_events_active() - enable/disable plug events on a switch
 *
 * Also configures a sane plug_events_delay of 255ms.
 *
 * Return: Returns 0 on success or an error code on failure.
 */
static int tb_plug_events_active(struct tb_switch *sw, bool active)
{
	u32 data;
	int res;

	sw->config.plug_events_delay = 0xff;
	res = tb_sw_write(sw, ((u32 *) &sw->config) + 4, TB_CFG_SWITCH, 4, 1);
	if (res)
		return res;

	res = tb_sw_read(sw, &data, TB_CFG_SWITCH, sw->cap_plug_events + 1, 1);
	if (res)
		return res;

	if (active) {
		data = data & 0xFFFFFF83;
		if (sw->config.device_id == 0x1547)
			data |= 4;
	} else {
		data = data | 0x7c;
	}
	return tb_sw_write(sw, &data, TB_CFG_SWITCH,
			   sw->cap_plug_events + 1, 1);
}

/* switch/port allocation & initialization */

/**
 * tb_init_port() - initialize a port
 *
 * This is a helper method for tb_switch_alloc. Does not check or initialize
 * any downstream switches.
 *
 * Return: Returns 0 on success or an error code on failure.
 */
static int tb_init_port(struct tb_switch *sw, u8 port_nr)
{
	int res;
	struct tb_port *port = &sw->ports[port_nr];
	port->sw = sw;
	port->port = port_nr;
	res = tb_port_read(port, &port->config, TB_CFG_PORT, 0, 8);
	if (res)
		return res;

	tb_dump_port(sw->tb, &port->config);

	/* TODO: Read dual link port, DP port and more from EEPROM. */
	return 0;

}

/**
 * tb_switch_free() - free a tb_switch and all downstream switches
 */
static void tb_switch_free(struct tb_switch *sw)
{
	kfree(sw->ports);
	kfree(sw);
}

/**
 * tb_switch_alloc() - allocate and initialize a switch
 *
 * Return: Returns a NULL on failure.
 */
static struct tb_switch *tb_switch_alloc(struct tb *tb, u64 route)
{
	int i;
	int cap;
	struct tb_switch *sw;
	int upstream_port = tb_cfg_get_upstream_port(tb->cfg, route);
	if (upstream_port < 0)
		return NULL;

	sw = kzalloc(sizeof(*sw), GFP_KERNEL);
	if (!sw)
		return NULL;

	sw->tb = tb;
	if (tb_cfg_read(tb->cfg, &sw->config, route, 0, 2, 0, 5))
		goto err;
	tb_info(tb,
		"initializing Switch at %#llx (depth: %d, up port: %d)\n",
		route,
		tb_route_length(route),
		upstream_port);
	tb_info(tb, "old switch config:\n");
	tb_dump_switch(tb, &sw->config);

	/* configure switch */
	sw->config.upstream_port_number = upstream_port;
	sw->config.depth = tb_route_length(route);
	sw->config.route_lo = route;
	sw->config.route_hi = route >> 32;
	sw->config.enabled = 1;
	/* from here on we may use the tb_sw_* functions & macros */

	if (sw->config.vendor_id != 0x8086) {
		tb_sw_WARN(sw,
			   "unsupported switch vendor id %#x, aborting\n",
			   sw->config.vendor_id);
		goto err;
	}
	if (sw->config.device_id != 0x1547 && sw->config.device_id != 0x1549) {

		tb_sw_WARN(sw,
			   "unsupported switch device id %#x, aborting\n",
			   sw->config.device_id);
		goto err;
	}

	/* upload configuration */
	if (tb_sw_write(sw, 1 + (u32 *) &sw->config, TB_CFG_SWITCH, 1, 3))
		goto err;

	/* initialize ports */
	sw->ports = kcalloc(sw->config.max_port_number + 1, sizeof(*sw->ports),
	GFP_KERNEL);
	if (!sw->ports)
		goto err;

	for (i = 0; i <= sw->config.max_port_number; i++) {
		if (tb_init_port(sw, i))
			goto err;
		/* TODO: check if port is disabled (EEPROM) */
	}

	/* TODO: I2C, IECS, EEPROM, link controller */

	cap = tb_find_cap(&sw->ports[0], TB_CFG_SWITCH, TB_CAP_PLUG_EVENTS);
	if (cap < 0) {
		tb_sw_WARN(sw, "cannot find TB_CAP_PLUG_EVENTS aborting\n");
		goto err;
	}
	sw->cap_plug_events = cap;

	if (tb_plug_events_active(sw, true))
		goto err;

	return sw;
err:
	kfree(sw->ports);
	kfree(sw);
	return NULL;
}

/**
 * reset_switch() - send TB_CFG_PKG_RESET and enable switch
 *
 * Return: Returns 0 on success or an error code on failure.
 */
static int tb_switch_reset(struct tb *tb, u64 route)
{
	int res;
	struct tb_regs_switch_header header = {
		header.route_hi = route >> 32,
		header.route_lo = route,
		header.enabled = true,
	};
	tb_info(tb, "resetting switch at %llx\n", route);
	res = tb_cfg_reset(tb->cfg, route);
	if (res)
		return res;
	return tb_cfg_write(tb->cfg, ((u32 *) &header) + 2, route, 0, 2, 2, 2);
}

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
 * Free all switches and the config channel.
 */
void thunderbolt_shutdown_and_free(struct tb *tb)
{
	mutex_lock(&tb->lock);
	tb->shutdown = true; /* signal tb_handle_hotplug to quit */

	if (tb->root_switch)
		tb_switch_free(tb->root_switch);
	tb->root_switch = NULL;

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
 * Allocates a tb_cfg control channel, initializes the root switch and enables
 * plug events.
 *
 * Return: Returns NULL on error.
 */
struct tb *thunderbolt_alloc_and_start(struct tb_nhi *nhi)
{
	struct tb *tb;

	BUILD_BUG_ON(sizeof(struct tb_regs_switch_header) != 5 * 4);
	BUILD_BUG_ON(sizeof(struct tb_regs_port_header) != 8 * 4);
	BUILD_BUG_ON(sizeof(struct tb_regs_hop) != 2 * 4);

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

	if (tb_switch_reset(tb, 0))
		goto err_locked;

	tb->root_switch = tb_switch_alloc(tb, 0);
	if (!tb->root_switch)
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
