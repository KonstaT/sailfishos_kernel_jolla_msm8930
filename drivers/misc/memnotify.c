/*
 * memnotify.c - system-wide memory meter and notifier pseudo-device
 *
 * Copyright (C) 2012 Nokia Corporation.
 *      Leonid Moiseichuk
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/highmem.h>
#include <linux/swap.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/timer.h>

/*
 * How often [ms] information will be updated.
 */
#define MN_UPDATE_PERIOD	250

/*
 * Maximal delay [ms] if no changes detected.
 */
#define MN_MAX_UPDATE_PERIOD	(15 * 1000)

/*
 * Which minimal [kb] allocation change will produce notification for user-space
 * to avoid too often jittering.
 */
#define MN_UPDATE_SPACE	1024

/*
 * Which memory types we should have, report and track.
 * Total reported last because expected is not changed between reboots,
 * so to minimize number of comparison it has sense to order like pointed.
 *
 * Note:
 * If you need to report more values, add them here and
 * modify get_memory_status function to fill more fields
 *
 * Warning:
 * The length of list is limited by used flags mask (unsigned = 32)
 */
static const char * const memtypes[] = {
	"used",
	"active",
	"total",
};
#define MN_TYPES_SIZE		(ARRAY_SIZE(memtypes))
#define MN_LINE_BUFFER_SIZE	(MN_TYPES_SIZE * 32)

/* Memory values indexed by memtypes */
struct memvalue {
	unsigned long	v[MN_TYPES_SIZE];
};


/* subscriber information to be notified when level changed */
struct observer {
	/* list data to check from notify_memory_usage and wakeup user-space */
	struct list_head	list;

	/* related file structure for open/close/read/write and poll */
	struct file		*file;
	/* thresholds [pages] when we should trigger notification */
	struct memvalue		threshold;
	/* bitmask: did we crossed theshold on last validation? */
	unsigned		active;
	/* flag about new notification is required */
	bool			updated;
};



MODULE_AUTHOR("Leonid Moiseichuk (leonid.moiseichuk@nokia.com)");
MODULE_DESCRIPTION("System memory meter/notification pseudo-device");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.3.0");

static unsigned update_period __read_mostly = MN_UPDATE_PERIOD;
module_param(update_period, uint, 0);
MODULE_PARM_DESC(update_period, "Base update interval [ms]");

static unsigned max_update_period __read_mostly = MN_MAX_UPDATE_PERIOD;
module_param(max_update_period, uint, 0);
MODULE_PARM_DESC(max_update_period, "Maximal update interval [ms]");

static unsigned update_space __read_mostly;
module_param(update_space, uint, 0);
MODULE_PARM_DESC(update_space, "Clients granularity space in [kb], 0 - auto");

/* The device pointer, mostly for dev_XXX */
static struct device *dev __read_mostly;

/* Validated parameters in adequate units */
static unsigned long update_period_jiffies     __read_mostly;
static unsigned long max_update_period_jiffies __read_mostly;
static unsigned      update_space_pages        __read_mostly;

/* Memory values which is used in measurements and notification */
static unsigned long available_pages      __read_mostly;
#ifdef CONFIG_SWAP
static unsigned long available_swap_pages __read_mostly;
#endif

/* Amount of memory measured and notified last time */
/* That is safe to have these values as a normal data by design */
static struct memvalue	last_measured	__read_mostly;
static struct memvalue	last_notified	__read_mostly;

/* Timer which is used to requesting vm statistics */
static struct timer_list timer;
static unsigned long update_timer_jiffies __read_mostly;

/* Subscribers in poll() call to be validated and notified */
static atomic_t observer_counter = ATOMIC_INIT(0);
static DEFINE_SPINLOCK(observer_lock);
static LIST_HEAD(observer_list);
static DECLARE_WAIT_QUEUE_HEAD(watcher_queue);



/* Validates two memvalues to be equal or not */
static inline bool memvalue_about_equal(
	const struct memvalue *a, const struct memvalue *b)
{
	unsigned m = 1;
	unsigned i = 0;

	while (i < MN_TYPES_SIZE) {
		const unsigned long av = a->v[i];
		const unsigned long bv = b->v[i];
		/* if field set to zero = not tracked */
		if (av && bv &&
			(av < bv ? bv - av : av - bv) >= update_space_pages)
			return false;
		m <<= 1;
		i++;
	}

	return true;
}

/* Produces bitmask of active memory thresholds */
static inline unsigned memvalue_active(
	const struct memvalue *t, const struct memvalue *c)
{
	unsigned a = 0;
	unsigned m = 1;
	unsigned i = 0;

	while (i < MN_TYPES_SIZE) {
		const unsigned long tv = t->v[i];
		const unsigned long cv = c->v[i];
		/* if field set to zero = not tracked */
		if (tv && cv && cv >= tv)
			a |= m;
		m <<= 1;
		i++;
	}

	return a;
}


static inline bool validate_observer(
	struct observer *obs, const struct memvalue *now)
{
	/* evaluation of current state and compare to old one */
	const unsigned active = memvalue_active(&obs->threshold, now);

	/*
	 * If we evaluated status just before and did not send update
	 * yet to user-space we must preserve update flag.
	 */
	if (active != obs->active) {
		obs->active  = active;
		obs->updated = true;
	}

	return obs->updated;
}

/* Please update this function if contents memtypes is changed */
static inline void get_memory_status(struct memvalue *value)
{
	/* field #0 -- used memory by substracting free memories */
	value->v[0] = available_pages;

	/* RAM part: free + slab rec + cached - shared - mlocked */
	value->v[0] -= global_page_state(NR_FREE_PAGES);
	value->v[0] -= global_page_state(NR_SLAB_RECLAIMABLE);
	value->v[0] -= global_page_state(NR_FILE_PAGES);
	value->v[0] += global_page_state(NR_SHMEM);
	value->v[0] += global_page_state(NR_MLOCK);
#ifdef CONFIG_SWAP
	/* Swap if we have */
	if (available_swap_pages) {
		struct sysinfo si;
		si_swapinfo(&si);
		value->v[0] -= si.freeswap;
	}
#endif

	/* field #1 -- active pages */
	value->v[1]  = global_page_state(NR_ACTIVE_FILE);
	value->v[1] += global_page_state(NR_ACTIVE_ANON);

	/* field #2 -- total available pages */
	value->v[2] = available_pages;
}

/* this method invoked from timer to re-check statistics */
static void timer_function(unsigned long __always_unused data)
{
	/* query current memory statistics */
	get_memory_status(&last_measured);

	/* do we have value changed? */
	if (!memvalue_about_equal(&last_measured, &last_notified)) {
		last_notified = last_measured;
		update_timer_jiffies = update_period_jiffies;
		if (atomic_read(&observer_counter) > 0) {
			bool updated = false;
			struct list_head *pos;

			spin_lock(&observer_lock);
			list_for_each(pos, &observer_list) {
				struct observer *obs = (struct observer *)pos;
				if (validate_observer(obs, &last_measured)) {
					updated = true;
					/*
					 * some watcher changed status
					 * the rest will be done in mn_poll
					 */
					break;
				}
			}
			spin_unlock(&observer_lock);

			if (updated) {
				/*
				 * Wakeup of tasks should happened rare, only
				 * when at least one theshold changed. So has
				 * sense to show this information in logs.
				 */
				wake_up_all(&watcher_queue);
			}
		}
	} else {
		update_timer_jiffies <<= 1;
		if (update_timer_jiffies > max_update_period_jiffies)
			update_timer_jiffies = max_update_period_jiffies;
	}

	mod_timer(&timer, jiffies + update_timer_jiffies);
}

static int vm_shrink(struct shrinker __always_unused *sh,
		struct shrink_control __always_unused *sc)
{
	/* we are in reclaim mode - recheck memory situation later */
	if (update_timer_jiffies != update_period_jiffies) {
		/* adjust timer only in case it has sense */
		update_timer_jiffies = update_period_jiffies;
		mod_timer(&timer, jiffies + update_timer_jiffies);
	}

	return 0;
}

static struct shrinker vm_shrinker = {
	.shrink = vm_shrink,
	.seeks = DEFAULT_SEEKS
};


static inline void subscribe_vm_stats(void)
{
	/* the initial delay always started from specified value */
	update_timer_jiffies = update_period_jiffies;

	/* update memory statistics */
	get_memory_status(&last_measured);
	last_notified = last_measured;

	/* create timer and register shrinker */
	init_timer_deferrable(&timer);
	timer.data = 0;
	timer.function = timer_function;
	timer.expires = jiffies + update_timer_jiffies;
	add_timer(&timer);
	register_shrinker(&vm_shrinker);
}

static inline void unsubscribe_vm_stats(void)
{
	unregister_shrinker(&vm_shrinker);
	del_timer_sync(&timer);
}

static int mn_open(struct inode *inode, struct file *file)
{
	struct observer *obs;

	obs = kzalloc(sizeof(*obs), GFP_KERNEL);
	if (!obs)
		return -ENOMEM;

	get_device(dev);

	/* object initialization */
	obs->file      = file;
	file->private_data = obs;

	/* place it into checking list */
	spin_lock_bh(&observer_lock);
	list_add(&obs->list, &observer_list);
	spin_unlock_bh(&observer_lock);

	/* subscribe to vm stat updates */
	if (1 == atomic_add_return(1, &observer_counter))
		subscribe_vm_stats();

	dev_dbg(dev, "0x%p - observer %u created\n",
			obs, atomic_read(&observer_counter));

	return 0;
}

static int mn_release(struct inode *inode, struct file *file)
{
	struct observer *obs = (struct observer *)file->private_data;

	dev_dbg(dev, "0x%p - observer released\n", obs);
	if (!obs)
		return -EINVAL;

	/* unsubscribe from vm stat updates */
	if (0 == atomic_sub_return(1, &observer_counter))
		unsubscribe_vm_stats();

	/* remove from checking list */
	spin_lock_bh(&observer_lock);
	list_del(&obs->list);
	spin_unlock_bh(&observer_lock);

	/* cleanup the memory */
	file->private_data = NULL;
	kfree(obs);

	put_device(dev);

	return 0;
}

static ssize_t mn_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	char tmp[MN_LINE_BUFFER_SIZE];
	ssize_t pos = 0;
	const struct memvalue mv = last_measured;
	int idx;

	for (idx = 0; idx < MN_TYPES_SIZE && pos < sizeof(tmp) - 2; idx++) {
		ssize_t retval;

		if (pos > 0)
			tmp[pos++] = ' ';
		retval = snprintf(&tmp[pos], sizeof(tmp) - pos,
				"%s %lu",
				memtypes[idx], mv.v[idx]);
		if (retval > 0)
			pos += retval;
		else
			return -EINVAL;
	}

	if (pos < sizeof(tmp))
		tmp[pos++] = '\n';

	if (pos > count)
		pos = count;

	return copy_to_user(buf, tmp, pos) ? -EINVAL : pos;
}

static ssize_t mn_write(struct file *file, const char __user *buf,
					size_t count, loff_t *offset)
{
	struct observer *obs = (struct observer *)file->private_data;
	char tmp[MN_LINE_BUFFER_SIZE];
	ssize_t retval = min(count, sizeof(tmp) - 1);
	int index;

	if (copy_from_user(tmp, buf, retval))
		return -EINVAL;

	tmp[retval]  = 0;
	obs->updated = false;
	obs->active  = 0;
	for (index = 0; index < MN_TYPES_SIZE; index++) {
		const char *ptr = strstr(tmp, memtypes[index]);
		char nmb[32];
		int  i;

		if (!ptr) {
			obs->threshold.v[index] = 0;
			continue;
		}

		ptr += strlen(memtypes[index]) + 1;
		while (*ptr && !isdigit(*ptr))
			ptr++;

		for (i = 0; i < sizeof(nmb) - 1 && isdigit(*ptr); i++, ptr++)
			nmb[i] = *ptr;
		nmb[i] = 0;

		if (kstrtoul(nmb, 10, &obs->threshold.v[index]) < 0) {
			dev_dbg(dev,
				"0x%p - cannot parse '%s' as '%s'\n",
					obs, memtypes[index], nmb);
			obs->threshold.v[index] = 0;
			return -EINVAL;
		}
	}
	obs->active = memvalue_active(&obs->threshold, &last_measured);
	dev_dbg(dev, "0x%p - threshold set to 0x%x\n", obs, obs->active);

	return (ssize_t)count;
}

static unsigned int mn_poll(struct file *file, poll_table *wait)
{
	struct observer *obs = (struct observer *)file->private_data;

	if (NULL == obs)
		return 0;

	poll_wait(file, &watcher_queue, wait);
	if (validate_observer(obs, &last_measured)) {
		dev_info(dev, "0x%p - threshold updated to 0x%x\n",
					obs, obs->active);
		obs->updated = false;
		return POLLIN;
	}
	return 0;
}



static const struct file_operations mn_fops = {
	.owner   = THIS_MODULE,
	.llseek  = noop_llseek,
	.open    = mn_open,
	.release = mn_release,
	.read    = mn_read,
	.write   = mn_write,
	.poll    = mn_poll,
};

static struct miscdevice mn_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "memnotify",
	.fops  = &mn_fops,
	.mode  = S_IRUGO|S_IWUGO,
};


static int __init mn_init(void)
{
	struct sysinfo si;
	int error;
#ifdef DEBUG
	int i;
#endif

	error = misc_register(&mn_device);
	if (error) {
		pr_err("unable to register device %d\n", error);
		return error;
	}
	dev = mn_device.this_device;

	update_period_jiffies = msecs_to_jiffies(update_period);
	if (!update_period_jiffies)
		update_period_jiffies = msecs_to_jiffies(MN_UPDATE_PERIOD);

	max_update_period_jiffies = msecs_to_jiffies(max_update_period);
	if (max_update_period_jiffies < update_period_jiffies)
		max_update_period_jiffies = update_period_jiffies;

	/* query amount of available ram and swap, mem_unit is PAGE_SIZE */
	si_meminfo(&si);
#ifdef CONFIG_SWAP
	si_swapinfo(&si);
	available_pages = si.totalram + si.totalswap;
	available_swap_pages = si.totalswap;
#else
	available_pages = si.totalram;
#endif
	/* if autodetect then set granularity to ~1.4% from available memory */
	/* update_space_pages extra divided by 2 due to it is an offset      */
	if (update_space)
		update_space_pages = update_space >> (PAGE_SHIFT - 10 + 1);
	else
		update_space_pages = available_pages >> 7;
	if (!update_space_pages)
		update_space_pages = MN_UPDATE_SPACE >> (PAGE_SHIFT - 10 + 1);

	dev_dbg(dev, "update period set to %u ms or %lu jiffies\n",
				update_period, update_period_jiffies);
	dev_dbg(dev, "update period limit set to %u ms or %lu jiffies\n",
				max_update_period, max_update_period_jiffies);
	dev_dbg(dev, "update space set to %u kb or -+%u pages\n",
				update_space, update_space_pages);

#ifdef CONFIG_SWAP
	dev_dbg(dev, "%lu available pages found (%lu ram + %lu swap)\n",
				available_pages, si.totalram, si.totalswap);
#else
	dev_dbg(dev, "%lu available pages found (only ram)\n",
						available_pages);
#endif

#ifdef DEBUG
	get_memory_status(&last_measured);
	for (i = 0; i < MN_TYPES_SIZE; i++) {
		dev_dbg(dev, "%lu %s pages, utilization %lu percents\n",
						last_measured.v[i],
							memtypes[i],
			(100 * last_measured.v[i]) / available_pages);
	}
#endif

	dev_dbg(dev, "overhead per client connection is %u bytes\n",
				(unsigned)sizeof(struct observer));

	return 0;
}

static void __exit mn_exit(void)
{
	dev = NULL;
	misc_deregister(&mn_device);
}

module_init(mn_init);
module_exit(mn_exit);
