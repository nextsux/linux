/*
 * Device independent Thunderbolt bus logic
 *
 * Copyright (c) 2013 Andreas Noever <andreas.noever@gmail.com>
 */

#ifndef TB_H_
#define TB_H_

#include <linux/pci.h>

#include "tb_regs.h"
#include "tb_cfg.h"

#define TB_PORT_MASK 0x3f
#define TB_ROUTE_SHIFT 8

/**
 * struct tb_switch - a thunderbolt switch
 */
struct tb_switch {
	struct tb_regs_switch_header config;
	struct tb_port *ports;
	struct tb *tb;
	int cap_plug_events; /* offset, zero if not found */
	bool invalid; /* unplugged, will go away */
};

/**
 * struct tb_port - a thunderbolt port, part of a tb_switch
 */
struct tb_port {
	struct tb_regs_port_header config;
	struct tb_switch *sw;
	struct tb_port *remote; /* remote port, NULL if not connected */
	int cap_phy; /* offset, zero if not found */
	u8 port; /* port number on switch */
	bool invalid; /* unplugged, will go away */
};

/**
 * struct tb - main thunderbolt bus structure
 */
struct tb {
	struct mutex lock;	/*
				 * Big lock. Must be held when accessing cfg or
				 * any struct tb_switch / struct tb_port.
				 */
	struct tb_nhi *nhi;
	struct tb_cfg *cfg;
	struct workqueue_struct *wq; /* ordered workqueue for plug events */
	struct tb_switch *root_switch;
	bool shutdown;	/*
			 * Once this is set tb_handle_hotplug will exit (once it
			 * can aquire lock at least once). Used to drain wq.
			 */
};

/**
 * tb_upstream_port() - return the upstream port of a switch
 *
 * Every switch has an upstream port (for the root switch it is the NHI).
 *
 * During switch alloc/init tb_upstream_port()->remote may be NULL, even for
 * non root switches (on the NHI port remote is always NULL).
 *
 * Return: Returns the upstream port of the switch.
 */
static inline struct tb_port *tb_upstream_port(struct tb_switch *sw)
{
	return &sw->ports[sw->config.upstream_port_number];
}

static inline u64 tb_route(struct tb_switch *sw)
{
	return ((u64) sw->config.route_hi) << 32 | sw->config.route_lo;
}

/* helper functions & macros */

static inline int tb_sw_read(struct tb_switch *sw, void *buffer,
			     enum tb_cfg_space space, uint32_t offset,
			     uint32_t length)
{
	return tb_cfg_read(sw->tb->cfg,
			   buffer,
			   tb_route(sw),
			   0,
			   space,
			   offset,
			   length);
}

static inline int tb_sw_write(struct tb_switch *sw, void *buffer,
			      enum tb_cfg_space space, uint32_t offset,
			      uint32_t length)
{
	return tb_cfg_write(sw->tb->cfg,
			    buffer,
			    tb_route(sw),
			    0,
			    space,
			    offset,
			    length);
}

static inline int tb_port_read(struct tb_port *port, void *buffer,
			       enum tb_cfg_space space, uint32_t offset,
			       uint32_t length)
{
	return tb_cfg_read(port->sw->tb->cfg,
			   buffer,
			   tb_route(port->sw),
			   port->port,
			   space,
			   offset,
			   length);
}

static inline int tb_port_write(struct tb_port *port, void *buffer,
				enum tb_cfg_space space, uint32_t offset,
				uint32_t length)
{
	return tb_cfg_write(port->sw->tb->cfg,
			    buffer,
			    tb_route(port->sw),
			    port->port,
			    space,
			    offset,
			    length);
}

#define tb_WARN(tb, fmt, arg...) dev_WARN(&(tb)->nhi->pdev->dev, fmt, ## arg)
#define tb_warn(tb, fmt, arg...) dev_warn(&(tb)->nhi->pdev->dev, fmt, ## arg)
#define tb_info(tb, fmt, arg...) dev_info(&(tb)->nhi->pdev->dev, fmt, ## arg)


#define __TB_SW_PRINT(level, sw, fmt, arg...)           \
	do {                                            \
		struct tb_switch *__sw = (sw);          \
		level(__sw->tb, "%llx: " fmt,           \
		      tb_route(__sw), ## arg);          \
	} while (0)
#define tb_sw_WARN(sw, fmt, arg...) __TB_SW_PRINT(tb_WARN, sw, fmt, ##arg)
#define tb_sw_warn(sw, fmt, arg...) __TB_SW_PRINT(tb_warn, sw, fmt, ##arg)
#define tb_sw_info(sw, fmt, arg...) __TB_SW_PRINT(tb_info, sw, fmt, ##arg)


#define __TB_PORT_PRINT(level, _port, fmt, arg...)                      \
	do {                                                            \
		struct tb_port *__port = (_port);                       \
		level(__port->sw->tb, "%llx:%x: " fmt,                  \
		      tb_route(__port->sw), __port->port, ## arg);      \
	} while (0)
#define tb_port_WARN(port, fmt, arg...) \
	__TB_PORT_PRINT(tb_WARN, port, fmt, ##arg)
#define tb_port_warn(port, fmt, arg...) \
	__TB_PORT_PRINT(tb_warn, port, fmt, ##arg)
#define tb_port_info(port, fmt, arg...) \
	__TB_PORT_PRINT(tb_info, port, fmt, ##arg)

struct tb *thunderbolt_alloc_and_start(struct tb_nhi *nhi);
void thunderbolt_shutdown_and_free(struct tb *tb);

int tb_find_cap(struct tb_port *port, enum tb_cfg_space space, u32 value);

#endif
