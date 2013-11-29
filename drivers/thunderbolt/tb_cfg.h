/*
 * Thunderbolt configuration channel
 *
 * Copyright (c) 2013 Andreas Noever <andreas.noever@gmail.com>
 */

#ifndef _TB_CFG
#define _TB_CFG

#include <linux/types.h>
#include <linux/kfifo.h>
#include <linux/dmapool.h>

#include "dsl3510.h"

typedef void (*hotplug_cb)(void *data, u64 route, u8 port, bool unplug);

/**
 * struct tb_cfg - thunderbolt configuration channel
 */
struct tb_cfg {
	struct tb_nhi *nhi;
	struct tb_ring *tx;
	struct tb_ring *rx;

	struct dma_pool *packet_pool;

	DECLARE_KFIFO(response_fifo, struct ring_packet*, 16);

	struct completion response_ready;

	hotplug_cb callback;
	void *callback_data;
};

#define tb_cfg_WARN(cfg, format, arg...) \
	dev_WARN(&(cfg)->nhi->pdev->dev, format, ## arg)

#define tb_cfg_err(cfg, format, arg...) \
	dev_err(&(cfg)->nhi->pdev->dev, format, ## arg)

#define tb_cfg_warn(cfg, format, arg...) \
	dev_warn(&(cfg)->nhi->pdev->dev, format, ## arg)

#define tb_cfg_info(cfg, format, arg...) \
	dev_info(&(cfg)->nhi->pdev->dev, format, ## arg)

struct tb_cfg *tb_cfg_alloc(struct tb_nhi *nhi, hotplug_cb cb, void *cb_data);
void tb_cfg_free(struct tb_cfg *cfg);

enum tb_cfg_space {
	TB_CFG_HOPS = 0,
	TB_CFG_PORT = 1,
	TB_CFG_SWITCH = 2,
	TB_CFG_COUNTERS = 3,
};

int tb_cfg_error(struct tb_cfg *cfg, u64 route, u32 port);
int tb_cfg_reset(struct tb_cfg *cfg, u64 route);
int tb_cfg_read(struct tb_cfg *cfg, void *buffer, uint64_t route, uint32_t port,
		enum tb_cfg_space space, uint32_t offset, uint32_t length);
int tb_cfg_write(struct tb_cfg *cfg, void *buffer, uint64_t route,
		 uint32_t port, enum tb_cfg_space space, uint32_t offset,
		 uint32_t length);
int tb_cfg_get_upstream_port(struct tb_cfg *cfg, u64 route);

#endif
