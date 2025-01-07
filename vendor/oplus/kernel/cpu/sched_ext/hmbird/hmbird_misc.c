#include "hmbird_sched.h"
#include "locking_main.h"
#include "binder_sched.h"

static void android_vh_hmbird_update_load_handler(
					void *unused, struct task_struct *p,
					struct rq *rq, int event, u64 wallclock)
{
#ifdef CONFIG_SCX_USE_UTIL_TRACK
	scx_update_task_ravg(p, rq, event, wallclock);
#endif
}

static void android_vh_hmbird_init_task_handler(
					void *unused, struct task_struct *p)
{
#ifdef CONFIG_SCX_USE_UTIL_TRACK
	scx_sched_init_task(p);
#endif
}

static void android_vh_hmbird_update_load_enable_handler(
					void *unused, bool enable)
{
	if (enable) {
		oplus_lk_feat_enable(LK_FEATURE_MASK, false);
		oplus_bd_feat_enable(BD_FEATURE_MASK, false);
	} else {
		oplus_lk_feat_enable(LK_FEATURE_MASK, true);
		oplus_bd_feat_enable(BD_FEATURE_MASK, true);
	}
#ifdef CONFIG_SCX_USE_UTIL_TRACK
	slim_walt_enable(enable);
	preempt_enable();
	if (enable)
		walt_disable_wait_for_completion();
	else
		walt_enable_wait_for_completion();
	preempt_disable();
#endif
}

static void android_vh_get_util_handler(
			void *unused, int cpu, struct task_struct *p, u64 *util)
{
	struct walt_task_struct *wts;
	struct walt_rq *wrq;
	u64 prev_runnable_sum;

	if ((cpu < 0) && NULL == p)
		return;

	if (p) {
		wts = (struct walt_task_struct *) p->android_vendor_data1;
		*util = wts->demand_scaled;
	} else {
		wrq = &per_cpu(walt_rq, cpu);
		prev_runnable_sum = wrq->prev_runnable_sum_fixed;
		do_div(prev_runnable_sum, wrq->prev_window_size >> SCHED_CAPACITY_SHIFT);
		*util = prev_runnable_sum;
	}
}

struct hmbird_ops_t {
	bool (*task_is_scx)(struct task_struct *p);
};
struct walt_ops_t {
	bool (*scx_enable)(void);
	bool (*check_non_task)(void);
};
extern void register_hmbird_sched_ops(struct hmbird_ops_t *ops);
extern void register_walt_ops(struct walt_ops_t *ops);
/* Ops must global variables */
static struct hmbird_ops_t hops;
static struct walt_ops_t wops;

static noinline bool check_scx_enabled(void)
{
	return atomic_read(&__scx_ops_enabled);
}

static noinline bool check_non_ext_task(void)
{
	return  atomic_read(&non_ext_task);
}

static void register_helper_ops(void)
{
	hops.task_is_scx = task_is_scx;
	wops.scx_enable = check_scx_enabled;
	wops.check_non_task = check_non_ext_task;
	register_hmbird_sched_ops(&hops);
	register_walt_ops(&wops);
}

static void register_hooks(void)
{
        int ret;

	REGISTER_TRACE_VH(android_vh_hmbird_update_load,
				android_vh_hmbird_update_load_handler);
	REGISTER_TRACE_VH(android_vh_hmbird_init_task,
				android_vh_hmbird_init_task_handler);
	REGISTER_TRACE_VH(android_vh_hmbird_update_load_enable,
				android_vh_hmbird_update_load_enable_handler);
	REGISTER_TRACE_VH(android_vh_get_util,
				android_vh_get_util_handler);
	register_helper_ops();
}

void hmbird_misc_init(void)
{
	register_hooks();
}
