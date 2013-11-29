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

static void tb_dump_hop(struct tb_port *port, struct tb_regs_hop *hop)
{
	tb_port_info(port,
		     " Hop through port %d to hop %d (%s)\n",
		     hop->out_port,
		     hop->next_hop,
		     hop->enable ? "enabled" : "disabled");
	tb_port_info(port,
		     "  Weight: %d Priority: %d Credits: %d Drop: %d\n",
		     hop->weight,
		     hop->priority,
		     hop->initial_credits,
		     hop->drop_packages);
	tb_port_info(port,
		     "  Counter enabled: %d Counter index: %d\n",
		     hop->counter_enable,
		     hop->counter);
	tb_port_info(port,
		     "  Flow Control (In/Eg): %d/%d Shared Buffer (In/Eg): %d/%d\n",
		     hop->ingress_fc,
		     hop->egress_fc,
		     hop->ingress_shared_buffer,
		     hop->egress_shared_buffer);
	tb_port_info(port,
		     "  Unknown1: %#x Unknown2: %#x Unknown3: %#x\n",
		     hop->unknown1,
		     hop->unknown2,
		     hop->unknown3);
}

/**
 * tb_downstream_route() - get route to downstream switch
 *
 * Port must not be the upstream port (otherwise a loop is created).
 *
 * Return: Returns a route to the switch behind @port.
 */
static u64 tb_downstream_route(struct tb_port *port)
{
	return tb_route(port->sw)
	       | ((u64) port->port << (port->sw->config.depth * 8));
}

static bool tb_is_upstream_port(struct tb_port *port)
{
	return port == tb_upstream_port(port->sw);
}

static int tb_route_length(u64 route)
{
	return (fls64(route) + TB_ROUTE_SHIFT - 1) / TB_ROUTE_SHIFT;
}

static struct tb_switch *get_switch_at_route(struct tb_switch *sw, u64 route)
{
	u8 next_port = route & TB_PORT_MASK;
	if (route == 0)
		return sw;
	if (next_port > sw->config.max_port_number)
		return 0;
	if (tb_is_upstream_port(&sw->ports[next_port]))
		return 0;
	if (!sw->ports[next_port].remote)
		return 0;
	return get_switch_at_route(sw->ports[next_port].remote->sw,
				   route >> TB_ROUTE_SHIFT);
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

/* thunderbolt port utility functions */

/**
 * tb_port_state() - get connectedness state of a port
 *
 * The port must have a TB_CAP_PHY (i.e. it should be a real port).
 *
 * Return: Returns a tb_port_state on success or an error code on failure.
 */
static enum tb_port_state tb_port_state(struct tb_port *port)
{
	struct tb_cap_phy phy;
	int res;
	if (port->cap_phy == 0) {
		tb_port_WARN(port, "does not have a PHY\n");
		return -EINVAL;
	}
	res = tb_port_read(port, &phy, TB_CFG_PORT, port->cap_phy, 2);
	if (res)
		return res;
	return phy.state;
}

/**
 * tb_wait_for_port() - wait for a port to become ready
 *
 * Check if the port is connected but not up. If so we wait for some
 * time to see whether it comes up.
 *
 * Return: Returns an error code on failure. Returns 0 if the port is not
 * connected or failed to reach state TB_PORT_UP within one second. Returns 1
 * if the port is connected and in state TB_PORT_UP.
 */
static int tb_wait_for_port(struct tb_port *port)
{
	int retries = 10;
	enum tb_port_state state;
	if (!port->cap_phy) {
		tb_port_WARN(port, "does not have PHY\n");
		return -EINVAL;
	}
	if (tb_is_upstream_port(port)) {
		tb_port_WARN(port, "is the upstream port\n");
		return -EINVAL;
	}

	while (retries--) {
		state = tb_port_state(port);
		if (state < 0)
			return state;
		if (state == TB_PORT_DISABLED) {
			tb_port_info(port, "is disabled (state: 0)\n");
			return 0;
		}
		if (state == TB_PORT_UNPLUGGED) {
			tb_port_info(port, "is unplugged (state: 7)\n");
			return 0;
		}
		if (state == TB_PORT_UP) {
			tb_port_info(port,
				     "is connected, link is up (state: 2)\n");
			return 1;
		}

		/*
		 * After plug-in the state is TB_PORT_CONNECTING. Give it some
		 * time.
		 */
		tb_port_info(port,
			     "is connected, link is not up (state: %d), retrying...\n",
			     state);
		msleep(100);
	}
	tb_port_WARN(port,
		     "failed to reach state TB_PORT_UP. Ignoring port...\n");
	return 0;
}

/**
 * tb_port_add_nfc_credits() - add/remove non flow controlled credits to port
 *
 * Change the number of NFC credits allocated to @port by @credits. To remove
 * NFC credits pass a negative amount of credits.
 *
 * Return: Returns 0 on success or an error code on failure.
 */
static int tb_port_add_nfc_credits(struct tb_port *port, int credits)
{
	if (credits == 0)
		return 0;
	tb_port_info(port,
		     "adding %#x NFC credits (%#x -> %#x)",
		     credits,
		     port->config.nfc_credits,
		     port->config.nfc_credits + credits);
	port->config.nfc_credits += credits;
	return tb_port_write(port, &port->config.nfc_credits,
			     TB_CFG_PORT, 4, 1);
}

/**
 * tb_port_clear_counter() - clear a counter in TB_CFG_COUNTER
 *
 * Return: Returns 0 on success or an error code on failure.
 */
static int tb_port_clear_counter(struct tb_port *port, int counter)
{
	u32 zero[3] = { 0, 0, 0 };
	tb_port_info(port, "clearing counter %d\n", counter);
	return tb_port_write(port, zero, TB_CFG_COUNTERS, 3 * counter, 3);
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
	int cap;
	struct tb_port *port = &sw->ports[port_nr];
	port->sw = sw;
	port->port = port_nr;
	port->remote = NULL;
	res = tb_port_read(port, &port->config, TB_CFG_PORT, 0, 8);
	if (res)
		return res;

	/* Port 0 is the switch itself and has no PHY. */
	if (port->config.type == TB_TYPE_PORT && port_nr != 0) {
		cap = tb_find_cap(port, TB_CFG_PORT, TB_CAP_PHY);

		if (cap > 0)
			port->cap_phy = cap;
		else
			tb_port_WARN(port, "non switch port without a PHY\n");
	}

	tb_dump_port(sw->tb, &port->config);

	/* TODO: Read dual link port, DP port and more from EEPROM. */
	return 0;

}

/**
 * tb_switch_free() - free a tb_switch and all downstream switches
 */
static void tb_switch_free(struct tb_switch *sw)
{
	int i;
	/* port 0 is the switch itself and never has a remote */
	for (i = 1; i <= sw->config.max_port_number; i++) {
		if (tb_is_upstream_port(&sw->ports[i]))
			continue;
		if (sw->ports[i].remote)
			tb_switch_free(sw->ports[i].remote->sw);
	}
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

/* thunderbolt path handling */

/**
 * tb_path_alloc() - allocate a thunderbolt path
 *
 * Return: Returns a tb_path on success or an error code on failure.
 */
struct tb_path *tb_path_alloc(struct tb *tb, int num_hops)
{
	struct tb_path *path = kzalloc(sizeof(*path), GFP_KERNEL);
	if (!path)
		return NULL;
	path->hops = kcalloc(num_hops, sizeof(*path->hops), GFP_KERNEL);
	if (!path->hops) {
		kfree(path);
		return NULL;
	}
	path->tb = tb;
	path->path_length = num_hops;
	return path;
}

/**
 * tb_path_free() - free a deactivated path
 */
void tb_path_free(struct tb_path *path)
{
	if (path->activated) {
		tb_WARN(path->tb, "trying to free an activated path\n")
		return;
	}
	kfree(path->hops);
	kfree(path);
}

static void __tb_path_deallocate_nfc(struct tb_path *path, int first_hop)
{
	int i, res;
	for (i = first_hop; i < path->path_length; i++) {
		res = tb_port_add_nfc_credits(path->hops[i].in_port,
					      -path->nfc_credits);
		if (res)
			tb_port_warn(path->hops[i].in_port,
				     "nfc credits deallocation failed for hop %d\n",
				     i);
	}
}

static void __tb_path_deactivate_hops(struct tb_path *path, int first_hop)
{
	int i, res;
	struct tb_regs_hop hop = { };
	for (i = first_hop; i < path->path_length; i++) {
		res = tb_port_write(path->hops[i].in_port,
				    &hop,
				    TB_CFG_HOPS,
				    2 * path->hops[i].in_hop_index,
				    2);
		if (res)
			tb_port_warn(path->hops[i].in_port,
				     "hop deactivation failed for hop %d, index %d\n",
				     i,
				     path->hops[i].in_hop_index);
	}
}

void tb_path_deactivate(struct tb_path *path)
{
	if (!path->activated) {
		tb_WARN(path->tb, "trying to deactivate an inactive path\n");
		return;
	}
	tb_info(path->tb,
		"deactivating path from %llx:%x to %llx:%x\n",
		tb_route(path->hops[0].in_port->sw),
		path->hops[0].in_port->port,
		tb_route(path->hops[path->path_length - 1].out_port->sw),
		path->hops[path->path_length - 1].out_port->port);
	__tb_path_deactivate_hops(path, 0);
	__tb_path_deallocate_nfc(path, 0);
	path->activated = false;
}

/**
 * tb_path_activate() - activate a path
 *
 * Activate a path starting with the last hop and iterating backwards. The
 * caller must fill path->hops before calling path_activate().
 *
 * Return: Returns 0 on success or an error code on failure.
 */
int tb_path_activate(struct tb_path *path)
{
	int i, res;
	enum tb_path_port out_mask, in_mask;
	if (path->activated) {
		tb_WARN(path->tb, "trying to activate already activated path\n");
		return -EINVAL;
	}

	tb_info(path->tb,
		"activating path from %llx:%x to %llx:%x\n",
		tb_route(path->hops[0].in_port->sw),
		path->hops[0].in_port->port,
		tb_route(path->hops[path->path_length - 1].out_port->sw),
		path->hops[path->path_length - 1].out_port->port);

	/* Clear counters. */
	for (i = path->path_length - 1; i >= 0; i--) {
		if (path->hops[i].in_counter_index == -1)
			continue;
		res = tb_port_clear_counter(path->hops[i].in_port,
					    path->hops[i].in_counter_index);
		if (res)
			goto err;
	}

	/* Add non flow controlled credits. */
	for (i = path->path_length - 1; i >= 0; i--) {
		res = tb_port_add_nfc_credits(path->hops[i].in_port,
					      path->nfc_credits);
		if (res) {
			__tb_path_deallocate_nfc(path, i);
			goto err;
		}
	}

	/* Activate hops. */
	for (i = path->path_length - 1; i >= 0; i--) {
		struct tb_regs_hop hop;

		/* dword 0 */
		hop.next_hop = path->hops[i].next_hop_index;
		hop.out_port = path->hops[i].out_port->port;
		/* TODO: figure out why these are good values */
		hop.initial_credits = (i == path->path_length - 1) ? 16 : 7;
		hop.unknown1 = 0;
		hop.enable = 1;

		/* dword 1 */
		out_mask = (i == path->path_length - 1) ?
				TB_PATH_DESTINATION : TB_PATH_INTERNAL;
		in_mask = (i == 0) ? TB_PATH_SOURCE : TB_PATH_INTERNAL;
		hop.weight = path->weight;
		hop.unknown2 = 0;
		hop.priority = path->priority;
		hop.drop_packages = path->drop_packages;
		hop.counter = path->hops[i].in_counter_index;
		hop.counter_enable = path->hops[i].in_counter_index != -1;
		hop.ingress_fc = path->ingress_fc_enable & in_mask;
		hop.egress_fc = path->egress_fc_enable & out_mask;
		hop.ingress_shared_buffer = path->ingress_shared_buffer
					    & in_mask;
		hop.egress_shared_buffer = path->egress_shared_buffer
					   & out_mask;
		hop.unknown3 = 0;

		tb_port_info(path->hops[i].in_port,
			     "Writing hop %d, index %d",
			     i,
			     path->hops[i].in_hop_index);
		tb_dump_hop(path->hops[i].in_port, &hop);
		res = tb_port_write(path->hops[i].in_port,
				    &hop,
				    TB_CFG_HOPS,
				    2 * path->hops[i].in_hop_index,
				    2);
		if (res) {
			__tb_path_deactivate_hops(path, i);
			__tb_path_deallocate_nfc(path, 0);
			goto err;
		}
	}
	path->activated = true;
	tb_info(path->tb, "path activation complete\n");
	return 0;
err:
	tb_WARN(path->tb, "path activation failed\n");
	return res;
}

/**
 * tb_path_is_invalid() - check whether any ports on the path are invalid
 *
 * Return: Returns true if the path is invalid, false otherwise.
 */
bool tb_path_is_invalid(struct tb_path *path)
{
	int i = 0;
	for (i = 0; i < path->path_length; i++) {
		if (path->hops[i].in_port->invalid)
			return true;
		if (path->hops[i].out_port->invalid)
			return true;
	}
	return false;
}

/* startup, enumeration, hot plug handling and shutdown */

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

static void tb_link_ports(struct tb_port *down, struct tb_port *up)
{
	down->remote = up;
	up->remote = down;
}

static void tb_scan_port(struct tb_port *port);

static void tb_scan_ports(struct tb_switch *sw)
{
	int i;
	for (i = 1; i <= sw->config.max_port_number; i++)
		tb_scan_port(&sw->ports[i]);
}

/**
 * tb_scan_port() - check for switches below port
 *
 * Checks whether port is connected. If so then we try to create a downstream
 * switch and recursively scan its ports
 */
static void tb_scan_port(struct tb_port *port)
{
	struct tb_switch *sw;
	if (tb_is_upstream_port(port))
		return;
	if (port->config.type != TB_TYPE_PORT)
		return;
	if (tb_wait_for_port(port) <= 0)
		return;

	sw = tb_switch_alloc(port->sw->tb, tb_downstream_route(port));
	if (!sw)
		return;
	tb_link_ports(port, tb_upstream_port(sw));
	tb_scan_ports(sw);
}

/**
 * tb_invalidate_below() - recursively invalidate all ports below a given port
 */
static void tb_invalidate_below(struct tb_port *port)
{
	struct tb_switch *sw;
	int i;
	if (tb_is_upstream_port(port)) {
		tb_port_WARN(port, "trying to invalidate an upstream port.\n");
		return;
	}
	if (!port->remote)
		return;
	sw = port->remote->sw;
	for (i = 0; i <= sw->config.max_port_number; i++) {
		sw->ports[i].invalid = true;
		if (!tb_is_upstream_port(&sw->ports[i]))
			tb_invalidate_below(&sw->ports[i]);
	}
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
	struct tb_switch *sw;
	struct tb_port *port;
	mutex_lock(&tb->lock);
	if (tb->shutdown)
		goto out;

	sw = get_switch_at_route(tb->root_switch, ev->route);
	if (!sw) {
		tb_WARN(tb,
			"hotplug event for non existent switch %llx:%x (unplug: %d)\n",
			ev->route,
			ev->port,
			ev->unplug)
		goto out;
	}
	if (sw->config.max_port_number < ev->port) {
		tb_WARN(tb,
			"hotplug event for non existent port %llx:%x (unplug: %d)\n",
			ev->route,
			ev->port,
			ev->unplug)
		goto out;
	}
	port = &sw->ports[ev->port];
	if (tb_is_upstream_port(port)) {
		tb_WARN(tb,
			"hotplug event for upstream port %llx:%x (unplug: %d)\n",
			ev->route,
			ev->port,
			ev->unplug)
		goto out;
	}
	if (ev->unplug) {
		if (port->remote) {
			tb_port_info(port, "unplugged\n");
			tb_invalidate_below(port);
			tb_switch_free(port->remote->sw);
			port->remote = NULL;
		} else {
			tb_port_info(port,
				     "got unplug event for disconnected port\n");
		}
	} else {
		tb_port_info(port, "hotplug: scanning\n");
		tb_scan_port(port);
		if (!port->remote)
			tb_port_info(port, "hotplug: no switch found\n");
	}
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

	tb_scan_ports(tb->root_switch);

	mutex_unlock(&tb->lock);
	return tb;

err_locked:
	/* In case tb_handle_hotplug is already executing. */
	tb->shutdown = true;
	mutex_unlock(&tb->lock);
	thunderbolt_shutdown_and_free(tb);
	return NULL;
}
