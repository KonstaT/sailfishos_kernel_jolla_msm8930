/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
/* Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog {*/
#include <linux/kallsyms.h>	    
/* } Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog */

//20121102, Terry Cheng, Check whether abnormal compoents using
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif  //CONFIG_PM_LOG

#include "power.h"

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;
/* Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog {*/
static void earlysuspend_timeout(unsigned long data);
static DEFINE_TIMER(earlysuspend_wd, earlysuspend_timeout, 0, 0);
static struct task_struct *suspend_task = NULL;
/* }Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog */

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);
/* Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog {*/
static void earlysuspend_timeout(unsigned long data)
{
	//Terry Cheng, 20121214, Fine tune the check process
	unsigned long irqflags;

//Terry Cheng, 20121214, Fine tune the check process
#ifdef CONFIG_PM_LOG
	spin_lock_irqsave(&state_lock, irqflags);
	if (!wake_lock_active(&main_wake_lock)){
		//Check abnormal camera components when main wake unlock 
		pmlog_check_camera_components();
	}else if (state & SUSPEND_REQUESTED){
		//Terry Cheng, 20130604, Show stack only when executing early suspend callback timeout
		if (suspend_task)
			show_stack(suspend_task, NULL);
		//Check whether abnormal compoents using when main wake lock 
		pmlog_trigger_abnormal_earlysuspend_components();
	}	
	spin_unlock_irqrestore(&state_lock, irqflags);
#endif	//CONFIG_PM_LOG	

}
static void earlysuspend_wdset(void *func)
{
	earlysuspend_wd.data = (unsigned long)func;
	mod_timer(&earlysuspend_wd, jiffies + HZ*5);//Terry Cheng, 20121210, Increase timeout since camera need more time to turn off
}
static void earlysuspend_wdclr(void *func)
{
	del_timer_sync(&earlysuspend_wd);
}
/* }Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog */
static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");
	/* Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog {*/
	suspend_task = current;	  	
	earlysuspend_wdset(NULL);
	/* }Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog */
	list_for_each_entry(pos, &early_suspend_handlers, link) {
		if (pos->suspend != NULL) {
			if (debug_mask & DEBUG_VERBOSE)
				pr_info("early_suspend: calling %pf\n", pos->suspend);
			pos->suspend(pos);
		}
	}
	//Terry Cheng, 20121214, Remove clear dog timer to detect abnormal camera usage
	//earlysuspend_wdclr(NULL);
	//suspend_task = NULL;

	mutex_unlock(&early_suspend_lock);

	suspend_sys_sync_queue();
abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		wake_unlock(&main_wake_lock);
	spin_unlock_irqrestore(&state_lock, irqflags);
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	/* Terry Cheng, 20120511, Raise late resume thread as Real time to improve resume time {*/
#ifdef ENABLE_RAISE_LATE_RESUME_THREAD_PRIORITY
	struct sched_param earlysuspend_s = { .sched_priority = 66 };
	struct sched_param earlysuspend_v = { .sched_priority = 0 };
	int earlysuspend_old_prio = 0;
	int earlysuspend_old_policy = 0;

	earlysuspend_old_prio = current->rt_priority; 
	earlysuspend_old_policy = current->policy; 
	printk(KERN_ERR "[SEAN]before late_resume, rt_pri=%d,pri=%d\n",current->rt_priority,current->prio); 
	/* just for this write, set us real-time */ 
	if (!(unlikely(earlysuspend_old_policy == SCHED_FIFO) || unlikely(earlysuspend_old_policy == SCHED_RR)))
	{ 
		if ((sched_setscheduler(current, SCHED_RR, &earlysuspend_s)) < 0) 
                    printk(KERN_ERR "[SEAN]up late_resume failed\n"); 
                else 
                    printk(KERN_ERR "[SEAN]up late_resume, rt_pri=%d,pri=%d\n",current->rt_priority,current->prio); 
	} 
#endif //ENABLE_RAISE_LATE_RESUME_THREAD_PRIORITY	
	/* } Terry Cheng, 20120511, Raise late resume thread as Real time to improve resume time */
	
	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	/* Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog {*/
	suspend_task = current;	  
	earlysuspend_wdset(NULL);
	/* } Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog */
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link) {
		if (pos->resume != NULL) {
			if (debug_mask & DEBUG_VERBOSE)
				pr_info("late_resume: calling %pf\n", pos->resume);

			pos->resume(pos);
		}
	}
	/* Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog {*/
	earlysuspend_wdclr(NULL);		
	suspend_task = NULL;
	/* } Terry Cheng, 20111125, Port from 8250 platform. Add Early suspend watch dog */
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);

	/* Terry Cheng, 20120511, Raise late resume thread as Real time to improve resume time {*/
#ifdef ENABLE_RAISE_LATE_RESUME_THREAD_PRIORITY
	/* restore scheduling policy and priority */ 
	if (!(unlikely(earlysuspend_old_policy == SCHED_FIFO) || unlikely(earlysuspend_old_policy == SCHED_RR)))
	{ 
		earlysuspend_v.sched_priority = earlysuspend_old_prio; 
		if ((sched_setscheduler(current, earlysuspend_old_policy, &earlysuspend_v)) < 0)
			printk(KERN_ERR "[SEAN]down late_resume failed\n"); 
		else 
			printk(KERN_ERR "[SEAN]down late_resume, rt_pri=%d,pri=%d\n",current->rt_priority,current->prio); 
	} 
#endif //ENABLE_RAISE_LATE_RESUME_THREAD_PRIORITY	
	/* } Terry Cheng, 20120511, Raise late resume thread as Real time to improve resume time */

}

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		state |= SUSPEND_REQUESTED;
		queue_work(suspend_work_queue, &early_suspend_work);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
		state &= ~SUSPEND_REQUESTED;
		wake_lock(&main_wake_lock);
		queue_work(suspend_work_queue, &late_resume_work);
	}
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}
