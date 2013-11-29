/*
 * Device independent Thunderbolt bus logic
 *
 * Copyright (c) 2013 Andreas Noever <andreas.noever@gmail.com>
 */

#ifndef TB_H_
#define TB_H_

#include <linux/pci.h>

#include "tb_cfg.h"

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
	bool shutdown;	/*
			 * Once this is set tb_handle_hotplug will exit (once it
			 * can aquire lock at least once). Used to drain wq.
			 */
};

struct tb *thunderbolt_alloc_and_start(struct tb_nhi *nhi);
void thunderbolt_shutdown_and_free(struct tb *tb);

#endif
