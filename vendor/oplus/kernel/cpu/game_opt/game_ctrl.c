// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Oplus. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#include "game_ctrl.h"
#ifdef CONFIG_HMBIRD_SCHED
#include "es4g_assist.h"
#endif /* CONFIG_HMBIRD_SCHED */

struct proc_dir_entry *game_opt_dir = NULL;
struct proc_dir_entry *early_detect_dir = NULL;

static int __init game_ctrl_init(void)
{
	game_opt_dir = proc_mkdir("game_opt", NULL);
	if (!game_opt_dir) {
		pr_err("fail to mkdir /proc/game_opt\n");
		return -ENOMEM;
	}
	early_detect_dir = proc_mkdir("early_detect", game_opt_dir);
	if (!early_detect_dir) {
		pr_err("fail to mkdir /proc/game_opt/early_detect\n");
		return -ENOMEM;
	}

	cpu_load_init();
	cpufreq_limits_init();
	early_detect_init();
	task_util_init();
	rt_info_init();
	fake_cpufreq_init();
	debug_init();
#if defined(CONFIG_HMBIRD_SCHED)
	es4g_assist_init();
#endif /* CONFIG_HMBIRD_SCHED */

	return 0;
}

static void __exit game_ctrl_exit(void)
{
#if defined(CONFIG_HMBIRD_SCHED)
	es4g_assist_exit();
#endif /* CONFIG_HMBIRD_SCHED */
}

module_init(game_ctrl_init);
module_exit(game_ctrl_exit);
MODULE_LICENSE("GPL v2");
