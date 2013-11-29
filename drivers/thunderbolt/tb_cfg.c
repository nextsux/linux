/*
 * Thunderbolt configuration channel
 *
 * Copyright (c) 2013 Andreas Noever <andreas.noever@gmail.com>
 */

#include <linux/crc32.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/dmapool.h>
#include <linux/workqueue.h>

#include "tb_cfg.h"
#include "dsl3510.h"


/* config packet definitions */

enum tb_cfg_pkg_type {
	TB_CFG_PKG_READ = 1,
	TB_CFG_PKG_WRITE = 2,
	TB_CFG_PKG_ERROR = 3,
	TB_CFG_PKG_NOTIFY_ACK = 4,
	TB_CFG_PKG_EVENT = 5,
	TB_CFG_PKG_XDOMAIN_REQ = 6,
	TB_CFG_PKG_XDOMAIN_RESP = 7,
	TB_CFG_PKG_OVERRIDE = 8,
	TB_CFG_PKG_RESET = 9,
	TB_CFG_PKG_PREPARE_TO_SLEEP = 0xd,
};

enum tb_cfg_error {
	TB_CFG_ERROR_PORT_NOT_CONNECTED = 0,
	TB_CFG_ERROR_INVALID_CONFIG_SPACE = 2,
	TB_CFG_ERROR_NO_SUCH_PORT = 4,
	TB_CFG_ERROR_ACK_PLUG_EVENT = 7,
	TB_CFG_ERROR_LOOP = 8,
};

struct tb_cfg_header {
	u32 route_hi:18;
	u32 unknown:14; /* highest order bit is set on replies */
	u32 route_lo;
} __packed;

struct tb_cfg_address {
	u32 offset:13; /* in dwords */
	u32 length:6; /* in dwords */
	u32 port:6;
	enum tb_cfg_space space:2;
	u32 zero:5;
} __packed;

struct cfg_reset_pkg {
	struct tb_cfg_header header;
} __packed;

struct cfg_error_pkg {
	struct tb_cfg_header header;
	enum tb_cfg_error error:4;
	u32 zero1:4;
	u32 port:6;
	u32 zero2:2; /* Both should be zero, still they are different fields. */
	u32 zero3:16;
} __packed;

struct cfg_plug_pkg {
	struct tb_cfg_header header;
	u32 port:6;
	u32 zero:25;
	bool unplug:1;
} __packed;

struct cfg_read_pkg {
	struct tb_cfg_header header;
	struct tb_cfg_address addr;
} __packed;

struct cfg_write_pkg {
	struct tb_cfg_header header;
	struct tb_cfg_address addr;
	u32 data[64]; /* tb_cfg_address.length has 6 bits */
} __packed;

/* helper methods */

static u64 get_route(struct tb_cfg_header header)
{
	return (u64) header.route_hi << 32 | header.route_lo;
}

static struct tb_cfg_header make_header(u64 route)
{
	struct tb_cfg_header header = {
		.route_hi = route >> 32,
		.route_lo = route,
	};
	/* check for overflow */
	WARN_ON(get_route(header) != route);
	return header;
}

static int decode_error(struct ring_packet *response)
{
	struct cfg_error_pkg *pkg = response->buffer;
	u32 raw;
	WARN_ON(response->eof != TB_CFG_PKG_ERROR);
	if (WARN_ON(response->size != sizeof(*pkg)))
		return -EIO;
	raw = *(u32 *) (response->buffer + 8);
	WARN_ON(pkg->zero1);
	WARN_ON(pkg->zero2);
	WARN_ON(pkg->zero3);
	WARN_ON(pkg->header.unknown != 1 << 13);
	switch (pkg->error) {
	case TB_CFG_ERROR_PORT_NOT_CONNECTED:
		/* Port is not connected. This can happen during surprise
		 * removal. Do not warn. */
		return -ENODEV;
	case TB_CFG_ERROR_INVALID_CONFIG_SPACE:
		/*
		 * Invalid cfg_space/offset/length combination in
		 * cfg_read/cfg_write.
		 */
		WARN(1,
		     "CFG_ERROR(raw: %#x route: %#llx port: %d): Invalid config space of offset\n",
		     raw,
		     get_route(pkg->header),
		     pkg->port);
		return -ENXIO;
	case TB_CFG_ERROR_NO_SUCH_PORT:
		/*
		 * - The route contains a non-existent port.
		 * - The route contains a non-PHY port (e.g. PCIe).
		 * - The port in cfg_read/cfg_write does not exist.
		 */
		WARN(1,
		     "CFG_ERROR(raw: %#x route: %#llx port: %d): Invalid port\n",
		     raw,
		     get_route(pkg->header),
		     pkg->port);
		return -ENXIO;
	case TB_CFG_ERROR_LOOP:
		WARN(1,
		     "CFG_ERROR(raw: %#x route: %#llx port: %d): Route contains a loop\n",
		     raw,
		     get_route(pkg->header),
		     pkg->port);
		return -EIO;
	default:
		/* 5,6,7,9 and 11 are also valid error codes */
		WARN(1,
		     "CFG_ERROR(raw: %#x route: %#llx port: %d): Unknown error\n",
		     raw,
		     get_route(pkg->header),
		     pkg->port);
		return -EIO;
	}
}

static int check_header(struct ring_packet *pkg, size_t expected_len,
			u64 expected_route, enum tb_cfg_pkg_type expected_type)
{
	struct tb_cfg_header *header = pkg->buffer;
	if (pkg->eof == TB_CFG_PKG_ERROR)
		return decode_error(pkg);
	if (WARN_ON(pkg->size != expected_len))
		return -EIO;
	if (WARN_ON(header->unknown != 1 << 13))
		return -EIO;
	if (WARN_ON(get_route(*header) != expected_route))
		return -EIO;
	if (WARN_ON(pkg->eof != expected_type))
		return -EIO;
	if (WARN_ON(pkg->sof != 0))
		return -EIO;
	return 0;
}

static bool check_config_address(const struct tb_cfg_address *addr,
				 enum tb_cfg_space space, uint32_t offset,
				 uint32_t length)
{
	if (WARN_ON(addr->zero))
		return -EIO;
	if (WARN_ON(addr->space != space))
		return -EIO;
	if (WARN_ON(addr->offset != offset))
		return -EIO;
	if (WARN_ON(addr->length != length))
		return -EIO;
	/*
	 * We cannot check addr->port as it is set to the upstream port of the
	 * sender.
	 */
	return 0;
}

static void cpu_to_be32_array(__be32 *dst, u32 *src, size_t len)
{
	int i;
	for (i = 0; i < len; i++)
		dst[i] = cpu_to_be32(src[i]);
}

static void be32_to_cpu_array(u32 *dst, __be32 *src, size_t len)
{
	int i;
	for (i = 0; i < len; i++)
		dst[i] = be32_to_cpu(src[i]);
}

static __be32 tb_crc(void *data, size_t len)
{
	return cpu_to_be32(~__crc32c_le(~0, data, len));
}

/* RX/TX handling */

/**
 * tb_ctl_handle_plug_event() - acknowledge a plug event and invoke cfg->callback
 */
static void tb_ctl_handle_plug_event(struct tb_cfg *cfg,
				     struct ring_packet *resp)
{
	struct cfg_plug_pkg *pkg = resp->buffer;
	u64 route = get_route(pkg->header);
	if (check_header(resp, sizeof(*pkg), route, TB_CFG_PKG_EVENT))
		return;
	if (tb_cfg_error(cfg, route, pkg->port))
		tb_cfg_warn(cfg,
			    "could not reset plug event on %llx:%x\n",
			    route,
			    pkg->port);
	WARN_ON(pkg->zero);
	cfg->callback(cfg->callback_data, route, pkg->port, pkg->unplug);
}

static void tb_ctl_free_packet(struct tb_cfg *cfg, struct ring_packet *pkg)
{
	dma_pool_free(cfg->packet_pool, pkg->buffer, pkg->buffer_phy);
	kfree(pkg);
}

/**
 * tb_ctl_alloc_packets() - allocate multiple packets safely
 *
 * Return: Returns 0 on success or an error code on failure.
 */
static int tb_ctl_alloc_packets(struct tb_cfg *cfg,
				struct ring_packet **packets, int num)
{
	int i;
	for (i = 0; i < num; i++) {
		packets[i] = kzalloc(sizeof(*packets[i]), GFP_KERNEL);
		if (!packets[i])
			goto err;
		packets[i]->buffer = dma_pool_alloc(cfg->packet_pool,
		GFP_KERNEL,
						    &packets[i]->buffer_phy);
		if (!packets[i]->buffer)
			goto err;
	}
	return 0;
err:
	if (!packets[i])
		i--;
	while (--i > 0) {
		if (packets[i]->buffer)
			dma_pool_free(cfg->packet_pool,
				      packets[i]->buffer,
				      packets[i]->buffer_phy);

		kfree(packets[i]);
	}
	return -ENOMEM;
}

/**
 * tb_cfg_tx() - transmit a packet on the config channel
 *
 * len must be a multiple of four.
 *
 * Return: Returns 0 on success or an error code on failure.
 */
static int tb_cfg_tx(struct tb_cfg *cfg, void *data, size_t len,
		     enum tb_cfg_pkg_type type)
{
	struct ring_packet *pkg;
	if (len % 4 != 0) { /* required for le->be conversion */
		tb_cfg_WARN(cfg, "TX: invalid size: %zu\n", len);
		return -EINVAL;
	}
	if (len > TB_FRAME_SIZE - 4) { /* checksum is 4 bytes */
		tb_cfg_WARN(cfg,
			    "TX: packet too large: %zu/%d\n",
			    len,
			    TB_FRAME_SIZE - 4);
		return -EINVAL;
	}
	if (tb_ctl_alloc_packets(cfg, &pkg, 1))
		return -ENOMEM;
	cpu_to_be32_array(pkg->buffer, data, len / 4);
	*(u32 *) (pkg->buffer + len) = tb_crc(pkg->buffer, len);
	pkg->size = len + 4;
	pkg->sof = type;
	pkg->eof = type;
	ring_tx(cfg->tx, pkg);
	return 0;
}

static void tb_cfg_tx_callback(struct tb_ring *ring, void *priv)
{
	struct tb_cfg *cfg = priv;
	struct ring_packet *pkg;
	while ((pkg = ring_poll(ring)))
		tb_ctl_free_packet(cfg, pkg);
}

/**
 * tb_cfg_rx() - receive a packet from the config channel
 *
 * Will timeout after a few seconds. The caller should requeue the packet as
 * soon as possible.
 *
 * Return: Returns a a received ring_packet or NULL.
 */
static struct ring_packet *tb_cfg_rx(struct tb_cfg *cfg)
{
	struct ring_packet *pkg;
	/**
	 * Most operations finish within a few milliseconds. But pci up/down
	 * ports will block during activation/deactivation. In the future we might
	 * have to implement variable timeouts or find a way to handle received
	 * packets out of order. Until then we just use a high timeout.
	 */
	if (!wait_for_completion_timeout(&cfg->response_ready,
					 msecs_to_jiffies(30000))) {
		tb_cfg_WARN(cfg, "RX: timeout\n");
		return NULL;
	}

	if (!kfifo_get(&cfg->response_fifo, &pkg)) {
		tb_cfg_err(cfg,
			   "RX: completion was signaled, but fifo was empty\n");
		return NULL;
	}
	return pkg;
}

static void tb_cfg_rx_callback(struct tb_ring *ring, void *priv)
{
	struct tb_cfg *cfg = priv;
	struct ring_packet *pkg;
	while ((pkg = ring_poll(ring))) {
		if (pkg->canceled) {
			tb_ctl_free_packet(cfg, pkg);
			continue;
		}
		if (pkg->size < 4 || pkg->size % 4 != 0) {
			tb_cfg_err(cfg,
				   "RX: invalid size %d, dropping packet\n",
				   pkg->size);
			goto rx;
		}

		pkg->size -= 4; /* remove checksum */
		if (*(u32 *) (pkg->buffer + pkg->size)
		    != tb_crc(pkg->buffer, pkg->size)) {
			tb_cfg_err(cfg,
				   "RX: checksum mismatch, dropping packet\n");
			goto rx;
		}
		be32_to_cpu_array(pkg->buffer, pkg->buffer, pkg->size / 4);

		if (pkg->eof == TB_CFG_PKG_EVENT) {
			tb_ctl_handle_plug_event(cfg, pkg);
			goto rx;
		}
		if (!kfifo_put(&cfg->response_fifo, pkg)) {
			tb_cfg_err(cfg, "RX: fifo is full\n");
			goto rx;
		}
		complete(&cfg->response_ready);
		continue;
rx:
		ring_rx(cfg->rx, pkg);
	}
}


/* public interface */

/**
 * tb_cfg_alloc() - allocate a config channel
 *
 * cb will be invoked once for every hot plug event.
 *
 * Return: Returns a pointer on success or NULL on failure.
 */
struct tb_cfg *tb_cfg_alloc(struct tb_nhi *nhi, hotplug_cb cb, void *cb_data)
{
	int i;
	struct tb_cfg *cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
	struct ring_packet *rx_packets[10] = { };
	if (!cfg)
		return NULL;
	cfg->nhi = nhi;
	cfg->callback = cb;
	cfg->callback_data = cb_data;

	init_completion(&cfg->response_ready);
	INIT_KFIFO(cfg->response_fifo);
	cfg->packet_pool = dma_pool_create("dls3510 cfg", &nhi->pdev->dev,
					   TB_FRAME_SIZE, 4, 0);
	if (!cfg->packet_pool)
		goto err;

	/* tb_ctl_handle_plug_event uses cfg->tx. Allocate cfg->tx first. */
	cfg->tx = ring_alloc_tx(nhi, 0, 10, tb_cfg_tx_callback, cfg);
	if (!cfg->tx)
		goto err;

	cfg->rx = ring_alloc_rx(nhi, 0, 10, tb_cfg_rx_callback, cfg);
	if (!cfg->rx)
		goto err;

	/*
	 * After the first call to ring_rx we will start to receive plug
	 * notifications and call cfg->callback. After this we can no longer
	 * bail out cleanly. To avoid this situation we allocate all packets
	 * BEFORE calling ring_rx.
	 */
	if (tb_ctl_alloc_packets(cfg, rx_packets, 10))
		goto err;
	for (i = 0; i < 10; i++)
		ring_rx(cfg->rx, rx_packets[i]);

	tb_cfg_info(cfg, "config channel created\n");
	return cfg;
err:
	tb_cfg_free(cfg);
	return NULL;
}

/**
 * tb_cfg_free() - shutdown and free a config channel
 *
 * All calls to cfg->callback will have finished on return.
 *
 * Must NOT be called from cfg->callback.
 */
void tb_cfg_free(struct tb_cfg *cfg)
{
	struct ring_packet *pkg;
	/*
	 * cfg->tx is used in tb_ctl_handle_plug_event. Free cfg->tx
	 * after cfg->rx.
	 */
	if (cfg->rx)
		ring_drain_and_free(cfg->rx);
	if (cfg->tx)
		ring_drain_and_free(cfg->tx);
	while (kfifo_get(&cfg->response_fifo, &pkg))
		tb_ctl_free_packet(cfg, pkg);
	if (cfg->packet_pool)
		dma_pool_destroy(cfg->packet_pool);
	kfree(cfg);
}

/**
 * tb_cfg_error() - send error packet
 *
 * Currently the error code is hardcoded to TB_CFG_ERROR_ACK_PLUG_EVENT.
 *
 * Return: Returns 0 on success or an error code on failure.
 */
int tb_cfg_error(struct tb_cfg *cfg, u64 route, u32 port)
{
	struct cfg_error_pkg pkg = {
		.header = make_header(route),
		.port = port,
		.error = TB_CFG_ERROR_ACK_PLUG_EVENT,
	};
	tb_cfg_info(cfg, "resetting error on %llx:%x.\n", route, port);
	return tb_cfg_tx(cfg, &pkg, sizeof(pkg), TB_CFG_PKG_ERROR);
}

/**
 * tb_cfg_reset() - send an reset packet
 *
 * Return: Returns 0 on success or an error code on failure.
 */
int tb_cfg_reset(struct tb_cfg *cfg, u64 route)
{
	struct cfg_reset_pkg pkg = { .header = make_header(route) };
	struct ring_packet *response;
	int res = tb_cfg_tx(cfg, &pkg, sizeof(pkg), TB_CFG_PKG_RESET);
	if (res)
		return res;

	response = tb_cfg_rx(cfg);
	if (!response)
		return -EIO;

	res = check_header(response, 8, route, TB_CFG_PKG_RESET);
	ring_rx(cfg->rx, response);
	return res;
}

/**
 * tb_cfg_read() - read from config space into buffer
 *
 * offset and length are in dwords.
 *
 * Return: Returns 0 on success or an error code on failure.
 */
int tb_cfg_read(struct tb_cfg *cfg, void *buffer, uint64_t route, uint32_t port,
		enum tb_cfg_space space, uint32_t offset, uint32_t length)
{
	struct cfg_read_pkg pkg = {
		.header = make_header(route),
		.addr = {
			.port = port,
			.space = space,
			.offset = offset,
			.length = length,
		},
	};
	struct ring_packet *response;
	struct cfg_write_pkg *reply;
	int res;

	res = tb_cfg_tx(cfg, &pkg, sizeof(pkg), TB_CFG_PKG_READ);
	if (res)
		return res;

	response = tb_cfg_rx(cfg);
	if (!response)
		return -EIO;

	res = check_header(response, 12 + 4 * length, route, TB_CFG_PKG_READ);
	if (res)
		goto out;

	reply = response->buffer;
	res = check_config_address(&reply->addr, space, offset, length);
	if (res)
		goto out;

	memcpy(buffer, &reply->data, 4 * length);
out:
	ring_rx(cfg->rx, response);
	return res;
}

/**
 * tb_cfg_write() - write from buffer into config space
 *
 * offset and length are in dwords.
 *
 * Return: Returns 0 on success or an error code on failure.
 */
int tb_cfg_write(struct tb_cfg *cfg, void *buffer, uint64_t route,
		 uint32_t port, enum tb_cfg_space space, uint32_t offset,
		 uint32_t length)
{
	struct cfg_write_pkg pkg = {
		.header = make_header(route),
		.addr = {
			.port = port,
			.space = space,
			.offset = offset,
			.length = length,
		},
	};
	struct ring_packet *response;
	struct cfg_read_pkg *reply;
	int res;

	memcpy(&pkg.data, buffer, length * 4);

	res = tb_cfg_tx(cfg, &pkg, 12 + 4 * length, TB_CFG_PKG_WRITE);
	if (res)
		return res;

	response = tb_cfg_rx(cfg);
	if (!response)
		return -EIO;

	res = check_header(response, sizeof(*reply), route, TB_CFG_PKG_WRITE);
	if (res)
		goto out;

	reply = response->buffer;
	res = check_config_address(&reply->addr, space, offset, length);
out:
	ring_rx(cfg->rx, response);
	return res;
}

/**
 * tb_cfg_get_upstream_port() - get upstream port number of switch at route
 *
 * Reads the first dword from the switches TB_CFG_SWITCH config area and
 * returns the port number from which the reply originated.
 *
 * This information is in principle present in every read/write response. We
 * expose it only here in order to keep the normal API streamlined.
 *
 * Return: Returns the upstream port number on success or an error code on
 * failure.
 */
int tb_cfg_get_upstream_port(struct tb_cfg *cfg, u64 route)
{
	struct cfg_read_pkg pkg = {
		.header = make_header(route),
		.addr = {
			.port = 0,
			.space = TB_CFG_SWITCH,
			.offset = 0,
			.length = 1,
		},
	};
	struct ring_packet *response;
	struct cfg_write_pkg *reply;
	int res;

	res = tb_cfg_tx(cfg, &pkg, sizeof(pkg), TB_CFG_PKG_READ);
	if (res)
		return res;

	response = tb_cfg_rx(cfg);
	if (!response)
		return -EIO;

	res = check_header(response, 12 + 4, route, TB_CFG_PKG_READ);
	if (res)
		goto out;

	reply = response->buffer;
	res = check_config_address(&reply->addr, TB_CFG_SWITCH, 0, 1);
	if (res)
		goto out;
	res = reply->addr.port;
out:
	ring_rx(cfg->rx, response);
	return res;
}
