#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/remote_spinlock.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/string.h>
#include <mach/msm_iomap.h>
#include <mach/pm_log.h>
#include "smd_private.h"
#include <asm/segment.h>
#include <linux/android_alarm.h>
#include <linux/ktime.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/syscalls.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/suspend.h>  //Terry Cheng, 20130313, Include get suspend_stats information
/* Terry Cheng, 20120117, Add update battery information { */
#include <linux/mfd/pm8xxx/pm8921-charger.h>
/* } Terry Cheng, 20120117, Add update battery information */
#ifdef CONFIG_MSM_MPM
#include <mach/mpm.h>
#endif //CONFIG_MSM_MPM

/* Terry Cheng, Dump rpm stats to bufer for pm log {*/
#include "rpm_stats.h"
/* }Terry Cheng, Dump rpm stats to bufer for pm log */

/* Terry Cheng, 20120626, Get timezone offset from share memory {*/
#include <mach/oem_smem_struct.h>
/* }Terry Cheng, 20120626, Get timezone offset from share memory */

#include <linux/cpufreq.h>   // Terry Cheng, 20121011, Save cpu freq run time and backlight in pmlog 

#include "pm.h"	//Terry Cheng, 20121101, Parse rpm log
#include <mach/kevent.h>	//Terry Cheng, 20121101, Parse rpm log						
#include "rpm_resources.h"	//20120919, Terry Cheng, Show RPM resource log

#define PM_LOG_SUPPORT_LOG_WAKE_SOURCE
#ifdef CONFIG_MSM_MPM
#define PLATFORM_NUM_WAKEUP_SOURCE	MSM_MPM_NR_MPM_IRQS
#else
#define PLATFORM_NUM_WAKEUP_SOURCE	32
#endif	//CONFIG_MSM_MPM

//Terry Cheng, 20121101, Parse rpm log
#define NUM_ABNORMAL_PXO_ON_COUNT	5
#define RPM_TICK_TIME	32768

//Every one hour to save pm log
#define PMLOG_DEFAULT_TIMEOUT_SECONDS (1*60*60)
#define MAX_PMLOG_SAVE_TIMES	20
#define PMLOG_SAVED_PATH "/var/systemlog/pmlog"
#define PMLOG_NAME "pmlog.log"
#define PMLOG_FILE PMLOG_SAVED_PATH "/" PMLOG_NAME
#define POWEROFF_SAVED_PATH "/var/systemlog/poweroff"
#define POWEROFF_NAME "poweroff.log"
#define POWEROFF_FILE POWEROFF_SAVED_PATH "/" POWEROFF_NAME
#define MAX_COPY_PATH_LEN	100		//Terry Cheng, 20120117, Use define size to replace hot code
#define MAX_COMMAND_LEN		80		//Terry Cheng, 20120117, Use define size to replace hot code
#define PM_LOG_DPRINTK(level, fmt, args...) \
	do { \
		if (level <= PMLOG_DLL) \
			printk(KERN_INFO "%s: thread %s " fmt, __FUNCTION__, current->comm, ##args); \
	} while (0)
	
#define	FORMAT_COLUMN_R(buffer, length, format, arg...) \
	format_string(buffer, length, 25, ALIGN_RIGHT, false, format, ## arg);

#define FORMAT_COLUMN_RN(buffer, length, format, arg...) \
	format_string(buffer, length, 25, ALIGN_RIGHT, true, format, ## arg);

#define FORMAT_COLUMN_C(buffer, length, format, arg...) \
	format_string(buffer, length, 25, ALIGN_CENTER, false, format, ## arg);

#define	FORMAT_COLUMN_L(buffer, length, format, arg...) \
	format_string(buffer, length, 25, ALIGN_LEFT, false, format, ## arg);

#define FORMAT_COLUMN_N(buffer, length, format, arg...) \
	format_string(buffer, length, 25, ALIGN_RIGHT, true, format, ## arg);

#define	FORMAT_COLUMN_CN(buffer, length, format, arg...) \
	format_string(buffer, length, 25, ALIGN_CENTER, true, format, ## arg);

#define	FORMAT_COLUMN_SR(buffer, length, size, format, arg...) \
	format_string(buffer, length, size, ALIGN_RIGHT, false, format, ## arg);

#define FORMAT_COLUMN_SRN(buffer, length, size, format, arg...) \
	format_string(buffer, length, size, ALIGN_RIGHT, true, format, ## arg);

#define FORMAT_COLUMN_SC(buffer, length, size, format, arg...) \
	format_string(buffer, length, size, ALIGN_CENTER, false, format, ## arg);

#define	FORMAT_COLUMN_SCN(buffer, length, size, format, arg...) \
	format_string(buffer, length, size, ALIGN_CENTER, true, format, ## arg);

//Dynamic log level debugfs dir extern from board.c
static int PMLOG_DLL = PMLOG_DBG_ERR;	
//Not define for current project       		
static uint32_t poweroff_reason = 0x998877BB; 	
//For save pm log in kernel
struct wake_lock wake_lock_pmlog;
struct workqueue_struct *pmlog_wqueue;		//pmlogd work queue
struct work_struct pmlog_work;				//save pm log work
static struct list_head pmlog_list_head;	//Device register list head
static DEFINE_MUTEX(pmlog_mutex);
static struct timespec latest_update_time;	//For update device run time 
static struct timespec system_time;
static struct timespec time_interval = { PMLOG_DEFAULT_TIMEOUT_SECONDS, 0 }; // 1hr.
static struct timespec alarm_time = { 0, 0 };
static struct timespec time_24hr = { 24*60*60, 0 }; 	// 24hr
static struct timespec expire_time = { 0, 0 };
static struct alarm pmlog_alarm;
static unsigned long long total_deepsleep_time = 0;		//The deep sleep time from boot
static unsigned long long deepsleep_time = 0;			//The deep sleep time from evey save pm log save interval 

//Terry Cheng, 20121101, Parse rpm log
static unsigned long app_vote_xooff_time = 0;			
static struct msm_pm_sleep_ops pmlog_sleep_ops;

static char *buffer;
static char shutdown_cmd[MAX_COMMAND_LEN];
#ifdef PM_LOG_SUPPORT_LOG_WAKE_SOURCE
static unsigned long wake_count[PLATFORM_NUM_WAKEUP_SOURCE];
static unsigned long wake_time_arr[PLATFORM_NUM_WAKEUP_SOURCE][2];
static unsigned int wake_source;
#endif //PM_LOG_SUPPORT_LOG_WAKE_SOURCE
//dynamically adjust the pm log timeout interval
static long pmlog_timeout_s = PMLOG_DEFAULT_TIMEOUT_SECONDS;
//Terry Cheng, 20120117, Add update battery information
struct battery_inform_data battery_data;
extern struct dentry *kernel_debuglevel_dir;

/* Terry Cheng, 20120626, Get timezone offset from share memory {*/
smem_vendor_id0_amss_data *smem_vendor0_data;
/* } Terry Cheng, 20120626, Get timezone offset from share memory */


//Constant string
static const char * const bat_status_str[] = { "UNKNOWN", "CHARGING", "DISCHARGING", "NOT_CHARGING", "FULL"};
static const char * const bat_health_str[] = { "UNKNOWN", "GOOD", "OVERHEAT", "DEAD", "OVERVOLTAGE", "UNSPEC_FAILURE", "COLD"};
//Terry Cheng, 20120118, Add move pmlog and power log path and file name
static const char * const pm_move_path_str[] = { PMLOG_SAVED_PATH, POWEROFF_SAVED_PATH};
static const char * const pm_move_name_str[] = { PMLOG_NAME, POWEROFF_NAME};

//For save power off stack to power off log
extern size_t dump_stack_to_buf(char *buffer, size_t length) __cold;
static int pmlog_set_pmlog_timeout(void *data, u64 val);
static int pmlog_get_pmlog_timeout(void *data, u64 *val);
/* Terry Cheng, 20120903 , Port show kernel TOP {*/

#if 1
static int pmlog_set_pmlog_kernel_top_enable(void *data, u64 val);
static int pmlog_get_pmlog_kernel_top_enable(void *data, u64 *val);
#include <linux/kernel_stat.h>
#include <linux/vmalloc.h>
#include <linux/tick.h>

#define MAX_PID 32768
#define NUM_BUSY_THREAD_CHECK 5
static u32 full_loading_counter = 0;
unsigned int *prev_proc_stat = NULL;
int *curr_proc_delta = NULL;
struct task_struct **task_ptr_array = NULL;
//Terry Cheng, 20120906, JB change to struct 
struct kernel_cpustat  new_cpu_stat, old_cpu_stat;	
static spinlock_t lock;
static long enable_show_kernel_top = 0;
struct delayed_work show_kernel_top_work;				//show kernel top work
static int pmlog_kernel_top_delay_time = 10000;		//Default 10s 
static int pmlog_show_stack_threshold = 30;			//Defult idle 30%

//Terry, Cheng, 20121218, Add wifi hotspot and fm using time 
int wifi_hotspot_status=0;
EXPORT_SYMBOL(wifi_hotspot_status);
struct timespec wifi_hotspot_run_time;
struct timespec fm_run_time;

module_param_named(
	delay_time, pmlog_kernel_top_delay_time, int, S_IRUGO | S_IWUSR | S_IWGRP
);
module_param_named(
	show_stack_threshold, pmlog_show_stack_threshold, int, S_IRUGO | S_IWUSR | S_IWGRP
);


int findBiggestInRange(int *array, int max_limit_idx)
{
	int largest_idx = 0, i;

	for (i = 0 ; i < MAX_PID ; i++) {
		if (array[i] > array[largest_idx] && (max_limit_idx == -1 || array[i] < array[max_limit_idx]))
			largest_idx = i;
	}

	return largest_idx;
}

/* sorting from large to small */
void sorting(int *source, int *output)
{
	int i;
	for (i = 0 ; i < NUM_BUSY_THREAD_CHECK ; i++) {
		if (i == 0)
			output[i] = findBiggestInRange(source, -1);
		else
			output[i] = findBiggestInRange(source, output[i-1]);
	}
}

#ifdef arch_idle_time

static cputime64_t get_idle_time(int cpu)
{
	cputime64_t idle;

	idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
		idle += arch_idle_time(cpu);
	return idle;
}

static cputime64_t get_iowait_time(int cpu)
{
	cputime64_t iowait;

	iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	if (cpu_online(cpu) && nr_iowait_cpu(cpu))
		iowait += arch_idle_time(cpu);
	return iowait;
}

#else

static u64 get_idle_time(int cpu)
{
	u64 idle, idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		/* !NO_HZ so we can rely on cpustat.idle */
		idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	else
		idle = usecs_to_cputime64(idle_time);

	return idle;
}

static u64 get_iowait_time(int cpu)
{
	u64 iowait, iowait_time = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_time == -1ULL)
		/* !NO_HZ so we can rely on cpustat.iowait */
		iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	else
		iowait = usecs_to_cputime64(iowait_time);

	return iowait;
}

#endif
/* Sync fs/proc/stat.c to caculate all cpu statistics */
static void get_all_cpu_stat(struct kernel_cpustat *cpu_stat)
{
	int i;
	cputime64_t user, nice, system, idle, iowait, irq, softirq, steal;
	cputime64_t guest, guest_nice;

	if (!cpu_stat)
		return;

	user = nice = system = idle = iowait =
		irq = softirq = steal = 0;
	guest = guest_nice = 0;

	for_each_possible_cpu(i) {
		user += kcpustat_cpu(i).cpustat[CPUTIME_USER];
		nice += kcpustat_cpu(i).cpustat[CPUTIME_NICE];
		system += kcpustat_cpu(i).cpustat[CPUTIME_SYSTEM];
		idle += get_idle_time(i);
		iowait += get_iowait_time(i);
		irq += kcpustat_cpu(i).cpustat[CPUTIME_IRQ];
		softirq += kcpustat_cpu(i).cpustat[CPUTIME_SOFTIRQ];
		steal += kcpustat_cpu(i).cpustat[CPUTIME_STEAL];
		guest += kcpustat_cpu(i).cpustat[CPUTIME_GUEST];
		guest_nice += kcpustat_cpu(i).cpustat[CPUTIME_GUEST_NICE];
	}
	cpu_stat->cpustat[CPUTIME_USER] = user;
	cpu_stat->cpustat[CPUTIME_NICE] = nice;
	cpu_stat->cpustat[CPUTIME_SYSTEM] = system;
	cpu_stat->cpustat[CPUTIME_SOFTIRQ] = softirq;
	cpu_stat->cpustat[CPUTIME_IRQ] = irq;
	cpu_stat->cpustat[CPUTIME_IDLE] = idle;
	cpu_stat->cpustat[CPUTIME_IOWAIT] = iowait;
	cpu_stat->cpustat[CPUTIME_STEAL] = steal;
	cpu_stat->cpustat[CPUTIME_GUEST] = guest;
	cpu_stat->cpustat[CPUTIME_GUEST_NICE] = guest_nice;
}
void pmlog_show_kernel_top_work_func(struct work_struct *work)
{
	struct task_struct *p;
	int top_loading[NUM_BUSY_THREAD_CHECK], i;
	unsigned long user_time, system_time, io_time;
	unsigned long irq_time, idle_time, delta_time;
	ulong flags;
	struct task_cputime cputime;
	unsigned long cpu_idle ;
	int dump_top_stack = 0;

	if (task_ptr_array == NULL ||
			curr_proc_delta == NULL ||
			prev_proc_stat == NULL)
		return;

	spin_lock_irqsave(&lock, flags);
	get_all_cpu_stat(&new_cpu_stat);

	/* calculate the cpu time of each process */
	for_each_process(p) {
		thread_group_cputime(p, &cputime);

		if (p->pid < MAX_PID) {
			curr_proc_delta[p->pid] =
				(cputime.utime + cputime.stime)
				- (prev_proc_stat[p->pid]);
			task_ptr_array[p->pid] = p;
		}
	}

	/* sorting to get the top cpu consumers */
	sorting(curr_proc_delta, top_loading);

	/* calculate the total delta time */
	user_time = (unsigned long)((new_cpu_stat.cpustat[CPUTIME_USER] + new_cpu_stat.cpustat[CPUTIME_NICE])
			- (old_cpu_stat.cpustat[CPUTIME_USER] + old_cpu_stat.cpustat[CPUTIME_NICE]));
	system_time = (unsigned long)(new_cpu_stat.cpustat[CPUTIME_SYSTEM] - old_cpu_stat.cpustat[CPUTIME_SYSTEM]);
	io_time = (unsigned long)(new_cpu_stat.cpustat[CPUTIME_IOWAIT] - old_cpu_stat.cpustat[CPUTIME_IOWAIT]);
	irq_time = (unsigned long)((new_cpu_stat.cpustat[CPUTIME_IRQ] + new_cpu_stat.cpustat[CPUTIME_SOFTIRQ])
			- (old_cpu_stat.cpustat[CPUTIME_IRQ] + old_cpu_stat.cpustat[CPUTIME_SOFTIRQ]));
	idle_time = (unsigned long)
	((new_cpu_stat.cpustat[CPUTIME_IDLE] + new_cpu_stat.cpustat[CPUTIME_STEAL] + new_cpu_stat.cpustat[CPUTIME_GUEST])
	 - (old_cpu_stat.cpustat[CPUTIME_IDLE] + old_cpu_stat.cpustat[CPUTIME_STEAL] + old_cpu_stat.cpustat[CPUTIME_GUEST]));
	delta_time = user_time + system_time + io_time + irq_time + idle_time;

	cpu_idle = ((unsigned int)(((unsigned int)idle_time)*100));
	cpu_idle = 	cpu_idle /delta_time;
	printk(KERN_INFO "CPU Usage = %lu%% \n", 100-cpu_idle);
	//Check whether high loading
	if (cpu_idle <  pmlog_show_stack_threshold) 
		full_loading_counter++;
	else if (full_loading_counter)
		full_loading_counter = 0;
		
	/*
	 * Check if we need to dump the call stack of top CPU consumers
	 * If CPU idle time below pmlog_show_stack_threshold % for 90 secs
	 */
	if ((full_loading_counter >= 9) && (full_loading_counter % 3 == 0))
		 dump_top_stack = 1;

	/* print most time consuming processes */
	printk(KERN_INFO "CPU Usage\tPID\t\tName\n");
	for (i = 0 ; i < NUM_BUSY_THREAD_CHECK ; i++) {
		printk(KERN_INFO "%lu%%\t\t%d\t\t%s\t\t\t%d\n",
				curr_proc_delta[top_loading[i]] * 100 / delta_time,
				top_loading[i],
				task_ptr_array[top_loading[i]]->comm,
				curr_proc_delta[top_loading[i]]);
	}

	/* check if dump busy thread stack */
	if (dump_top_stack) {
	   struct task_struct *t;
	   for (i = 0 ; i < NUM_BUSY_THREAD_CHECK ; i++) {
		if (task_ptr_array[top_loading[i]] != NULL && task_ptr_array[top_loading[i]]->stime > 0) {
			t = task_ptr_array[top_loading[i]];
			/* dump all the thread stack of this process */
			do {
				printk(KERN_INFO "\n###pid:%d name:%s state:%lu ppid:%d stime:%lu utime:%lu\n",
				t->pid, t->comm, t->state, t->real_parent->pid, t->stime, t->utime);
				show_stack(t, t->stack);
				t = next_thread(t);
			} while (t != task_ptr_array[top_loading[i]]);
		}
	   }
	}
	/* save old values */
	for_each_process(p) {
		if (p->pid < MAX_PID) {
			thread_group_cputime(p, &cputime);
			prev_proc_stat[p->pid] = cputime.stime + cputime.utime;
		}
	}

	old_cpu_stat = new_cpu_stat;

	memset(curr_proc_delta, 0, sizeof(int) * MAX_PID);
	memset(task_ptr_array, 0, sizeof(int) * MAX_PID);
	spin_unlock_irqrestore(&lock, flags);
	queue_delayed_work(pmlog_wqueue, &show_kernel_top_work, msecs_to_jiffies(pmlog_kernel_top_delay_time));

}
#endif
/* } Terry Cheng, 20120903 , Port show kernel TOP */

/* Terry Cheng, 20130313, print suspend dump stats for debug when linux kernel suspend fail {*/
static char *suspend_step_name(enum suspend_stat_step step)
{
	switch (step) {
	case SUSPEND_FREEZE:
		return "freeze";
	case SUSPEND_PREPARE:
		return "prepare";
	case SUSPEND_SUSPEND:
		return "suspend";
	case SUSPEND_SUSPEND_NOIRQ:
		return "suspend_noirq";
	case SUSPEND_RESUME_NOIRQ:
		return "resume_noirq";
	case SUSPEND_RESUME:
		return "resume";
	default:
		return "";
	}
}
static void dump_suspend_stats(void)
{
	int i, index, last_dev, last_errno, last_step;

	last_dev = suspend_stats.last_failed_dev + REC_FAILED_NUM - 1;
	last_dev %= REC_FAILED_NUM;
	last_errno = suspend_stats.last_failed_errno + REC_FAILED_NUM - 1;
	last_errno %= REC_FAILED_NUM;
	last_step = suspend_stats.last_failed_step + REC_FAILED_NUM - 1;
	last_step %= REC_FAILED_NUM;
	pr_err("%s\n", __FUNCTION__);
	pr_err("%s: %d\n%s: %d\n%s: %d\n%s: %d\n%s: %d\n"
			"%s: %d\n%s: %d\n%s: %d\n%s: %d\n%s: %d\n",
			"success", suspend_stats.success,
			"fail", suspend_stats.fail,
			"failed_freeze", suspend_stats.failed_freeze,
			"failed_prepare", suspend_stats.failed_prepare,
			"failed_suspend", suspend_stats.failed_suspend,
			"failed_suspend_late",
				suspend_stats.failed_suspend_late,
			"failed_suspend_noirq",
				suspend_stats.failed_suspend_noirq,
			"failed_resume", suspend_stats.failed_resume,
			"failed_resume_early",
				suspend_stats.failed_resume_early,
			"failed_resume_noirq",
				suspend_stats.failed_resume_noirq);
	pr_err("failures:\n  last_failed_dev:\t%-s\n",
			suspend_stats.failed_devs[last_dev]);
	for (i = 1; i < REC_FAILED_NUM; i++) {
		index = last_dev + REC_FAILED_NUM - i;
		index %= REC_FAILED_NUM;
		pr_err("\t\t\t%-s\n",
			suspend_stats.failed_devs[index]);
	}
	pr_err("  last_failed_errno:\t%-d\n",
			suspend_stats.errno[last_errno]);
	for (i = 1; i < REC_FAILED_NUM; i++) {
		index = last_errno + REC_FAILED_NUM - i;
		index %= REC_FAILED_NUM;
		pr_err("\t\t\t%-d\n",
			suspend_stats.errno[index]);
	}
	pr_err("  last_failed_step:\t%-s\n",
			suspend_step_name(
				suspend_stats.failed_steps[last_step]));
	for (i = 1; i < REC_FAILED_NUM; i++) {
		index = last_step + REC_FAILED_NUM - i;
		index %= REC_FAILED_NUM;
		pr_err("\t\t\t%-s\n",
			suspend_step_name(
				suspend_stats.failed_steps[index]));
	}
}
/* } Terry Cheng, 20130313, print suspend dump stats for debug when linux kernel suspend fail */

/* Terry Cheng, 20121031, Parse RPM vdd min and xo time whether too short {*/
/* Terry Cheng, 20121130, Fix get wrong rpm sleep time stats {*/
extern unsigned long rpm_vdd_min_time;
extern unsigned long rpm_vdd_xo_off_time;
/* } Terry Cheng, 20121130, Fix get wrong rpm sleep time stats */

/* Terry Cheng, 20130332, Dump more log for modem debug {*/
static void update_alarm_time(void)
{	
	system_time = ktime_to_timespec(alarm_get_elapsed_realtime());
	//Check the pm log time interval
	PM_LOG_DPRINTK(PMLOG_DBG_ERR,"%s time_interval.tv_sec = %d\n", __FUNCTION__, (int) time_interval.tv_sec);
	if (time_interval.tv_sec > 0){
		alarm_time = timespec_add_safe(time_interval, system_time);
		alarm_start_range(&pmlog_alarm, 
			timespec_to_ktime(alarm_time),
			timespec_to_ktime(alarm_time));
	}
}
/* } Terry Cheng, 20130332, Dump more log for modem debug */
static void pmlog_parse_rpm_sleep_time(void)
{
	static unsigned long pre_rpm_vddmin_time = 0;
	static unsigned long pre_rpm_xo_off_time = 0;
	static unsigned int app_pxo_on_count = 0;
	
	unsigned long cur_rpm_vdd_min_time = 0;
	unsigned long cur_rpm_xo_off_time = 0;
	//20130313, Add check total_try_suspend_count. If total_try_suspend_count == 0, it means wakelock does not unlock 
	unsigned int total_rpm_sleep_time = 0;
	static int	pre_suspend_stats_entry = 0;
	int	total_try_suspend_count = 0 ;
	static int abnormal_rpm_count = 0; //Terry Cheng, 20130332, Dump more log for modem debug 


	PM_LOG_DPRINTK(PMLOG_DBG_ERR, "app_vote_xooff_time = %lu, rpm_vdd_xo_off_time = %lu, rpm_vdd_min_time = %lu \n", app_vote_xooff_time, rpm_vdd_xo_off_time, rpm_vdd_min_time);	
	PM_LOG_DPRINTK(PMLOG_DBG_ERR, "pre_suspend_stats_entry = %d\n", pre_suspend_stats_entry);	
	
	//Check kernel suspend stats
	total_try_suspend_count = suspend_stats.entry - pre_suspend_stats_entry;
	pre_suspend_stats_entry =  suspend_stats.entry;	
	PM_LOG_DPRINTK(PMLOG_DBG_ERR, "suspend_stats.entry = %d, total_try_suspend_count = %d \n"
					, suspend_stats.entry
					,total_try_suspend_count );	

	/* Terry Cheng, 20130523, Only capture log when abnormal rpm occure. Needn't to check app sleep time. {*/
	PM_LOG_DPRINTK(PMLOG_DBG_ERR, "abnormal_rpm_count %d \n", abnormal_rpm_count);
	if (abnormal_rpm_count > 0 ){
		abnormal_rpm_count++;	
		if(abnormal_rpm_count > 3){
			kevent_trigger(KEVENT_ABNORMAL_SUBSYTEM);
			pmlog_timeout_s = PMLOG_DEFAULT_TIMEOUT_SECONDS;
			time_interval.tv_sec = pmlog_timeout_s;
			update_alarm_time();
			abnormal_rpm_count = 0;
		}
		goto out;
	}
	/* } Terry Cheng, 20130523, Only capture log when abnormal rpm occure. Needn't to check app sleep time. */
		
	//Check rpm vdd min and xo off time when krait vote xo off
	if (app_vote_xooff_time  )
	{	
		cur_rpm_xo_off_time = rpm_vdd_xo_off_time/RPM_TICK_TIME; //Terry Cheng, 20121130, Fix get wrong rpm sleep time stats
		cur_rpm_vdd_min_time = rpm_vdd_min_time/RPM_TICK_TIME;  //Terry Cheng, 20121130, Fix get wrong rpm sleep time stats
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "cur_rpm_vdd_min_time %lu s, cur_rpm_xo_off_time %lu s \n", cur_rpm_vdd_min_time, cur_rpm_xo_off_time);	
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "pre_rpm_vddmin_time %lu s, pre_rpm_xo_off_time %lu s \n", pre_rpm_vddmin_time, pre_rpm_xo_off_time);	

		//Check rpm vdd min time whether over flow
		if (cur_rpm_vdd_min_time >= pre_rpm_vddmin_time)
			total_rpm_sleep_time = cur_rpm_vdd_min_time - pre_rpm_vddmin_time;
		else
			total_rpm_sleep_time = cur_rpm_vdd_min_time + (0xffffffff - pre_rpm_vddmin_time);
		//Check rpm xo off time wheter over flow
		if (cur_rpm_xo_off_time >= pre_rpm_xo_off_time)
			total_rpm_sleep_time += (cur_rpm_xo_off_time - pre_rpm_xo_off_time);
		else
			total_rpm_sleep_time += (cur_rpm_xo_off_time + (0xffffffff - pre_rpm_xo_off_time));

		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "total_rpm_sleep_time %d s wifi_hotspot_run_time.tv_sec = %ld s fm_run_time = %ld s \n", 
				total_rpm_sleep_time, 
				wifi_hotspot_run_time.tv_sec,
				fm_run_time.tv_sec);	

		/* Terry Cheng, 20130332, Dump more log for modem debug {*/
		//Check rpm sleep time whether normal		
		if( ( app_vote_xooff_time > 600) ){
			/*
			RPM should sleep long enough. 
			1. It should more than 1/8 app_vote_xooff_time 
			2. wifi hotspot using time whether too long. 
			3. FM using time whether too long
			*/
			
			if(total_rpm_sleep_time < (app_vote_xooff_time >> 3) && 
				((wifi_hotspot_run_time.tv_sec + fm_run_time.tv_sec) < (app_vote_xooff_time >> 2)) )
			{
				//First time to detect abnormal rpm 
				abnormal_rpm_count++;	
				if (abnormal_rpm_count == 1){
					pmlog_timeout_s = 180;//180 second
					time_interval.tv_sec = pmlog_timeout_s;
					update_alarm_time();
				}								
			}	
		}
		/* } Terry Cheng, 20130332, Dump more log for modem debug */
		pre_rpm_vddmin_time = cur_rpm_vdd_min_time;
		pre_rpm_xo_off_time = cur_rpm_xo_off_time;

		//Reset app pxo on count
		app_pxo_on_count = 0;
	}
	else
	{
		//Check whether chager or usb plug or audio playback to prevent krait enter power collapse
		//Add check total_try_suspend_count. If total_try_suspend_count == 0, it means wakelock does not unlock. 
		if(is_charger_usb_plugin() || pmlog_check_audio_components() || (total_try_suspend_count == 0)){
			//Reset app pxo on count
			app_pxo_on_count=0;
			goto out;
		}	
		//App processor vote pxo on for NUM_ABNORMAL_PXO_ON_COUNT hour
		if(++app_pxo_on_count > NUM_ABNORMAL_PXO_ON_COUNT){
			//print suspend dump stats for debug when linux kernel suspend fail
			dump_suspend_stats();
			kevent_trigger(KEVENT_ABNORMAL_SUBSYTEM);
		}	
	}
out:	
	//Reset app vote xo off time and wifi hotsopt using time 	
	app_vote_xooff_time = 0;
	wifi_hotspot_run_time.tv_sec=0;
	wifi_hotspot_run_time.tv_nsec=0;
	fm_run_time.tv_sec=0;
	fm_run_time.tv_nsec=0;

}


/* } Terry Cheng, 20121031, Parse RPM vdd min and xo time whether too short */
void pmlog_update_wakeup(unsigned long wake_time)
{

	PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "wake source = %d, Wakeup %lu Sec.\n", wake_source, wake_time);
#ifdef PM_LOG_SUPPORT_LOG_WAKE_SOURCE
	if (wake_source > PLATFORM_NUM_WAKEUP_SOURCE)
    {
        PM_LOG_DPRINTK(PMLOG_DBG_ERR,"invalid wake source : %d\n", wake_source);
        return;
    }
	wake_time_arr[wake_source][0] += wake_time;
    if (wake_time_arr[wake_source][1] < wake_time)
    {
	    wake_time_arr[wake_source][1] = wake_time;
    }
#endif	//PM_LOG_SUPPORT_LOG_WAKE_SOURCE	
}

void pmlog_update_suspend(unsigned long suspended_time)
{
	uint32_t power;
	struct msm_rpmrs_limits *rs_limits = NULL;
	struct msm_pm_time_params time_param;

	time_param.latency_us = -1;
	time_param.sleep_us = -1;
	time_param.next_event_us = 0;

	if (pmlog_sleep_ops.lowest_limits)
			rs_limits = pmlog_sleep_ops.lowest_limits(false,
					MSM_PM_SLEEP_MODE_POWER_COLLAPSE, &time_param, &power);
		
	PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "Suspend %lu Sec. rs_limits->pxo = %d, app_vote_xooff_time = %lu\n", suspended_time, rs_limits->pxo, app_vote_xooff_time);
	deepsleep_time += suspended_time;
	if( rs_limits->pxo == 0)
		app_vote_xooff_time += suspended_time;
	total_deepsleep_time += suspended_time;
}

#ifdef PM_LOG_SUPPORT_LOG_WAKE_SOURCE
void pmlog_update_status(int i, unsigned long wake_status)
{

	i = i*32;
	while (wake_status) {
		if (wake_status & 0x1) {
			wake_count[i]++;
            wake_source = i;
		}
		wake_status>>=1;
		i++;
	}	
}
#endif //PM_LOG_SUPPORT_LOG_WAKE_SOURCE

static int pmlog_file_exist(char *path)
{
	struct file *filp = NULL;
	filp = filp_open(path, O_RDWR, S_IRWXU);
	if (!IS_ERR(filp)) {
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE,"pm_move_oldfile %s exist\n", path);
		filp_close(filp, NULL);
		return 1;
	}
	PM_LOG_DPRINTK(PMLOG_DBG_TRACE,"pm_move_oldfile %s\n", path);
	return 0;
}
static void pm_move_oldfile(enum pm_move_path_type type )
{
	char old_path[MAX_COPY_PATH_LEN];
	char new_path[MAX_COPY_PATH_LEN];
	mm_segment_t oldfs;
	int i;
	oldfs = get_fs();
	set_fs(get_ds());
	

	PM_LOG_DPRINTK(PMLOG_DBG_TRACE,"pm_move_oldfile %d\n", type);

	for (i=MAX_PMLOG_SAVE_TIMES; i>=0; i--) {	
		snprintf(old_path, MAX_COPY_PATH_LEN, "%s/%s.%d", pm_move_path_str[type], pm_move_name_str[type], i);
		snprintf(new_path, MAX_COPY_PATH_LEN, "%s/%s.%d", pm_move_path_str[type], pm_move_name_str[type], i+1);
		if (pmlog_file_exist(old_path)) {
			if (i==MAX_PMLOG_SAVE_TIMES) 
				sys_unlink(old_path);
			else 
				sys_rename(old_path, new_path);
		}
	}
	snprintf(old_path, MAX_COPY_PATH_LEN, "%s/%s",  pm_move_path_str[type], pm_move_name_str[type]);
	snprintf(new_path, MAX_COPY_PATH_LEN, "%s/%s.0",pm_move_path_str[type], pm_move_name_str[type]);
	if (pmlog_file_exist(old_path)) sys_rename(old_path, new_path);
	sys_sync();	//It could move file when rebooting so we must do sync to make sure the modification is syncto emmc. 
	set_fs(oldfs);
}
static int format_string(char *buffer, size_t length, 
		int column_size, ALIGNMENT align, 
		bool newline, const char *fmt, ...)
{
	char string_buffer[MAX_COMMAND_LEN];
	va_list args;
	int size = 0;
	int shift = 0;
	int min_length = (newline) ? (column_size+2) : (column_size+1);
	
	if (column_size >= 80) return 0;
	if (min_length >= length) return 0;

	memset(string_buffer, ' ', MAX_COMMAND_LEN);
	
	va_start(args, fmt);
	size = vsnprintf(string_buffer, MAX_COMMAND_LEN, fmt, args);
	va_end(args);
	
	memset(buffer, ' ', column_size);
	if (newline) buffer[column_size] = '\n';
	buffer[min_length-1] = 0;

	switch(align) {
		case ALIGN_CENTER: 
			shift = (column_size - size)/2;
			break;
		case ALIGN_RIGHT:
			shift = column_size - size;
			break;
		case ALIGN_LEFT:
		default:
			shift = 0;
			break;
	}

	memcpy((buffer + shift), string_buffer, size);
	return min_length-1;				
}
static void pmlog_flush_to_file(void)
{
	mm_segment_t oldfs;
	struct file *filp = NULL;
	struct pmlog_device *pmlog_dev;
	struct list_head *dev_list;
	struct timespec now;
	struct rtc_time tm;
	unsigned long long offset = 0;
	int size = 0;
	int i= 0;
	int cpu; // Terry Cheng, 20121011, Save cpu freq run time and backlight in pmlog 
	const char *subsys_name;//Terry Cheng, 20121226, Save smd and smsm wakeup app statistics	
	struct interrupt_stat *stats = interrupt_stats;//Terry Cheng, 20121226, Save smd and smsm wakeup app statistics

	PM_LOG_DPRINTK(PMLOG_DBG_TRACE,"Updating power management log.\n");

	oldfs = get_fs();
	set_fs(get_ds());

	filp = filp_open(PMLOG_FILE, O_RDWR|O_APPEND|O_CREAT, S_IRWXU|S_IRWXG|S_IRWXO);

	if (IS_ERR(filp)) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "%s: Can't open %s. %ld\n", __func__, 
			PMLOG_FILE, PTR_ERR(filp));
		set_fs(oldfs);
		return;
	}

	getnstimeofday(&now);
	/* Terry Cheng, 20120626, Get timezone offset from share memory and do timezone offset {*/
	//Do timezone offset
	if(smem_vendor0_data){
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE,"Time zone = %d\n", smem_vendor0_data->time_zone);	
		now.tv_sec += (smem_vendor0_data->time_zone*60);
	}
	/* } Terry Cheng, 20120626, Get timezone offset from share memory and do timezone offset */

	rtc_time_to_tm(now.tv_sec, &tm);
	size = FORMAT_COLUMN_R(buffer, PAGE_SIZE, "Current Time: ");
	size += snprintf(buffer+size, PAGE_SIZE-size,
		"%d-%02d-%02d %02d:%02d:%02d.%09lu \n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, now.tv_nsec);
	vfs_write(filp, buffer, size, &offset);

	now = ktime_to_timespec(alarm_get_elapsed_realtime());
	size = FORMAT_COLUMN_R(buffer, PAGE_SIZE, "Up Time: ");
	size += snprintf(buffer+size, PAGE_SIZE-size,"%ld.%ld Sec.\n", 
		now.tv_sec, now.tv_nsec/NSEC_PER_MSEC);
	vfs_write(filp, buffer, size, &offset);

	size = FORMAT_COLUMN_R(buffer, PAGE_SIZE, "Deep Sleep Time: ");
	size += snprintf(buffer+size, PAGE_SIZE-size, "%llu Sec.\n", deepsleep_time);
	size += FORMAT_COLUMN_R(buffer+size, PAGE_SIZE-size, "Total Sleep Time: ");
	size += snprintf(buffer+size, PAGE_SIZE-size, "%llu Sec.\n", total_deepsleep_time);
	vfs_write(filp, buffer, size, &offset);
	deepsleep_time = 0;

	//Discard to record cuurent wake source since it does not record the wake up time 
	if(wake_source) 
		wake_count[wake_source]--;

#ifdef PM_LOG_SUPPORT_LOG_WAKE_SOURCE
	size = snprintf(buffer, PAGE_SIZE, "Wakeup Source-Wakeup counter-total time-longest time:\n");
	for (i=0; i< PLATFORM_NUM_WAKEUP_SOURCE; i+=4) {
		size += FORMAT_COLUMN_C(buffer+size, PAGE_SIZE-size, "%lu: ", i);
		size += FORMAT_COLUMN_SR(buffer+size, PAGE_SIZE-size, 27, "%5lu:   %5lu:   %5lu\n",
                wake_count[i], wake_time_arr[i][0], wake_time_arr[i][1]);
		size += FORMAT_COLUMN_C(buffer+size, PAGE_SIZE-size, "%lu: ", i+1);
		size += FORMAT_COLUMN_SR(buffer+size, PAGE_SIZE-size, 27, "%5lu:   %5lu:   %5lu\n",
                wake_count[i+1], wake_time_arr[i+1][0], wake_time_arr[i+1][1]);
		size += FORMAT_COLUMN_C(buffer+size, PAGE_SIZE-size, "%lu: ", i+2);
		size += FORMAT_COLUMN_SR(buffer+size, PAGE_SIZE-size, 27, "%5lu:   %5lu:   %5lu\n",
                wake_count[i+2], wake_time_arr[i+2][0], wake_time_arr[i+2][1]);
		size += FORMAT_COLUMN_C(buffer+size, PAGE_SIZE-size, "%lu: ", i+3);
		size += FORMAT_COLUMN_SR(buffer+size, PAGE_SIZE-size, 27, "%5lu:   %5lu:   %5lu\n",
                wake_count[i+3], wake_time_arr[i+3][0], wake_time_arr[i+3][1]);
	}
	vfs_write(filp, buffer, size, &offset);
	memset(wake_count, 0, sizeof(wake_count));
	//Recover current wake source since it does not record the wake up time 
	if(wake_source) 
		wake_count[wake_source]++;
    for(i=0; i<PLATFORM_NUM_WAKEUP_SOURCE; i++)
    {
        PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "%ld, %ld\n", wake_time_arr[i][0], wake_time_arr[i][1]);
        wake_time_arr[i][0] = wake_time_arr[i][1] = 0;
    }
#endif	//PM_LOG_SUPPORT_LOG_WAKE_SOURCE

	size = FORMAT_COLUMN_R(buffer, PAGE_SIZE, "Battery Info time:  ");
	size += snprintf(buffer+size, PAGE_SIZE-size,
		"%d-%02d-%02d %02d:%02d:%02d \n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec);
	//Update battery information
	pmic8921_get_battery_inform_data(&battery_data);
	size += FORMAT_COLUMN_R(buffer+size, PAGE_SIZE-size, "Status: ");
	size += snprintf(buffer+size, PAGE_SIZE-size, "%s\n", bat_status_str[battery_data.status]);
	size += FORMAT_COLUMN_R(buffer+size, PAGE_SIZE-size, "Health: ");
	size += snprintf(buffer+size, PAGE_SIZE-size, "%s\n", bat_health_str[battery_data.health]);
	size += FORMAT_COLUMN_R(buffer+size, PAGE_SIZE-size, "Capacity: ");
	size += snprintf(buffer+size, PAGE_SIZE-size, "%d %%\n", battery_data.capacity);
	size += FORMAT_COLUMN_R(buffer+size, PAGE_SIZE-size, "Voltage: ");
	size += snprintf(buffer+size, PAGE_SIZE-size, "%d uV\n", battery_data.voltage);
	size += FORMAT_COLUMN_R(buffer+size, PAGE_SIZE-size, "Temperature: ");
	size += snprintf(buffer+size, PAGE_SIZE-size, "%d degC\n", battery_data.temp);
	//Terry Cheng, 20120822, add save current information
	size += FORMAT_COLUMN_R(buffer+size, PAGE_SIZE-size, "Curr: ");
	size += snprintf(buffer+size, PAGE_SIZE-size, "%d uA\n", battery_data.cur);
	vfs_write(filp, buffer, size, &offset);

	size = snprintf(buffer, PAGE_SIZE, "\n");
	size += FORMAT_COLUMN_C(buffer+size, PAGE_SIZE-size, "Device Name");
	size += FORMAT_COLUMN_C(buffer+size, PAGE_SIZE-size, "Run Time");
	size += FORMAT_COLUMN_CN(buffer+size, PAGE_SIZE-size, "Last Start");
	size += snprintf(buffer+size, PAGE_SIZE-size, "===============================================================\n");
	vfs_write(filp, buffer, size, &offset);
	
	list_for_each(dev_list, &pmlog_list_head) {
		pmlog_dev = container_of(dev_list, struct pmlog_device, pmlog_list);
		size = FORMAT_COLUMN_C(buffer, PAGE_SIZE, "%s.%d", dev_name(pmlog_dev->dev), pmlog_dev->index);
		size += FORMAT_COLUMN_C(buffer+size, PAGE_SIZE-size, "%ld.%ld", 
			pmlog_dev->run_time.tv_sec, pmlog_dev->run_time.tv_nsec/NSEC_PER_MSEC);
		size += FORMAT_COLUMN_CN(buffer+size, PAGE_SIZE-size, "%ld.%ld",
			pmlog_dev->start_time.tv_sec, pmlog_dev->start_time.tv_nsec/NSEC_PER_MSEC);
		vfs_write(filp, buffer, size, &offset);
	}
	/* Terry Cheng, 20120522, Add dump RPM stats log, 20120601, Add rpm sleep mode stats {*/
	//Dump RPM log to file
	memset(buffer,0, PAGE_SIZE);
	size = snprintf(buffer, PAGE_SIZE, "Dump RPM log\n");	
	size += msm_rpm_sleep_stats_dump(buffer+size, PAGE_SIZE-size);
	vfs_write(filp, buffer, size, &offset);
	//Terry Cheng, 20121130, Fix get wrong rpm sleep time stats
	//Parse RPM vdd min and xo time whether too short
	pmlog_parse_rpm_sleep_time();

	memset(buffer,0, PAGE_SIZE);
	size = msm_rpmstats_dump(buffer, PAGE_SIZE);
	vfs_write(filp, buffer, size, &offset); //Flush to file before parser. Logmaster may copy pmlog before pmlog flush done

	memset(buffer,0, PAGE_SIZE);	
	size = snprintf(buffer, PAGE_SIZE, "\n==============================================================\n");
	size += snprintf(buffer+size, PAGE_SIZE-size, "\n");
	vfs_write(filp, buffer, size, &offset);
	/* }Terry Cheng, 20120522, Add dump RPM stats log , 20120601, Add rpm sleep mode stats */
	/* Terry Cheng, 20130221, Add dump modem sleep mode stats log { */
	memset(buffer,0, PAGE_SIZE);
	size = snprintf(buffer, PAGE_SIZE, "Dump Modem log\n");
	if (((PAGE_SIZE - size) < sizeof(ModemPMLog)) || (smem_vendor0_data == NULL ))
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "modem log size too large or cannot get vendor0 data\n");
	else
	{

#ifdef DEBUG
		for (i = 0; i< Modem_PM_LOG_LPMSTATUS_NUM; i++)
		{
			PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "Modem_PM_log_lpm_status[%d] = %lu\n", i, smem_vendor0_data->Modem_PM_Log.Modem_PM_Log_body.Modem_PM_log_lpm_status[i]);
		}
		for (i = 0; i< DAL_RPMFW_MASTER_clone_COUNT; i++)
		{
			PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "Modem_PM_log_RPMhalt_MasterStatus_ActiveCount[%d] = %lu\n", i, smem_vendor0_data->Modem_PM_Log.Modem_PM_Log_body.Modem_PM_log_RPMhalt_MasterStatus_ActiveCount[i]);
			PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "Modem_PM_Log_Master_SelectedSet[%d] = %lu\n", i, smem_vendor0_data->Modem_PM_Log.Modem_PM_Log_body.Modem_PM_Log_Master_SelectedSet[i]);
			PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "Modem_PM_Log_Master_spm_subsystem_last_status[%d] = %lu\n", i, smem_vendor0_data->Modem_PM_Log.Modem_PM_Log_body.Modem_PM_Log_Master_spm_subsystem_last_status[i]);
		}	
#endif //DEBUG	
 		memcpy(buffer+size, &(smem_vendor0_data->Modem_PM_Log) , sizeof(ModemPMLog));
		//Terry Cheng, 20130412, Reset modem log flag after dump 
		smem_vendor0_data->Modem_PM_Log.Modem_PM_Log_reset_count_flag = 1;
 		
		size += sizeof(ModemPMLog);
	}
	size += snprintf(buffer+size, PAGE_SIZE-size, "\n");
	vfs_write(filp, buffer, size, &offset);
	/* } Terry Cheng, 20130221, Add dump modem sleep mode stats log */	

	/* Terry Cheng, 20120822, Improve pmlog to save cpu pm stats {*/
#ifdef CONFIG_MSM_IDLE_STATS
	memset(buffer,0, PAGE_SIZE);
	size = snprintf(buffer, PAGE_SIZE, "CPU pm stats\n");	
	vfs_write(filp, buffer, size, &offset);
	for (i=0; i< (MSM_PM_STAT_COUNT*num_possible_cpus()) ;i++) //Terry Cheng, 20121226, Change to MSM_PM_STAT_COUNT
	{
		memset(buffer,0, PAGE_SIZE);
		size = msm_pm_stats_dump(buffer, PAGE_SIZE, i);
		if(!size)
			continue;
		vfs_write(filp, buffer, size, &offset);
	}
	memset(buffer,0, PAGE_SIZE);
	size = snprintf(buffer, PAGE_SIZE, "==============================================================\n");
	vfs_write(filp, buffer, size, &offset);
#endif	
	/* } Terry Cheng, 20120822, Improve pmlog to save cpu pm stats */
	/* Terry Cheng, 20121011, Save cpu freq run time and backlight in pmlog { */
	for_each_possible_cpu(cpu) {
		memset(buffer,0, PAGE_SIZE);
		size = snprintf(buffer, PAGE_SIZE, "CPU%d cpufreq  stats\n", cpu);	
		size += dump_cpu_time_in_state(buffer+size, PAGE_SIZE-size, cpu);
		size += snprintf(buffer+size, PAGE_SIZE-size, "==============================================================\n");
		vfs_write(filp, buffer, size, &offset);
	}
	memset(buffer,0, PAGE_SIZE);
	size = snprintf(buffer, PAGE_SIZE, "Backlight using time\n");	
	size+=dump_bl_using_time(buffer+size, PAGE_SIZE-size);
	size += snprintf(buffer+size, PAGE_SIZE-size, "==============================================================\n");
	vfs_write(filp, buffer, size, &offset);
	/* } Terry Cheng, 20121011, Save cpu freq run time and backlight in pmlog */

	/* Terry Cheng, 20121226, Save smd and smsm wakeup app statistics {*/
	memset(buffer,0, PAGE_SIZE);
	size = snprintf(buffer,PAGE_SIZE,
		"   Subsystem    |     In    | Out (Hardcoded) |"
		" Out (Configured) | wakeup app\n");
	
	for (i = 0; i < NUM_SMD_SUBSYSTEMS; ++i) {
		subsys_name = smd_pid_to_subsystem(i);
		if (subsys_name) {
			size += snprintf(buffer + size, PAGE_SIZE - size,
				"%-10s %4s | %9u |       %9u |        %9u|        %9u |\n",
				subsys_name , "smd",
				stats->smd_in_count,
				stats->smd_out_hardcode_count,
				stats->smd_out_config_count, stats->smd_in_count_resume);

			size += snprintf(buffer + size, PAGE_SIZE - size,
				"%-10s %4s | %9u |       %9u |        %9u|        %9u  |\n",
				subsys_name, "smsm",
				stats->smsm_in_count,
				stats->smsm_out_hardcode_count,
				stats->smsm_out_config_count, stats->smsm_in_count_resume);
		}
		//Reset smd and smsm interrupt count 
		stats->smd_in_count_resume = 0;
		stats->smsm_in_count_resume = 0;
		++stats;
	}
	size += snprintf(buffer+size, PAGE_SIZE-size, "==============================================================\n");
	vfs_write(filp, buffer, size, &offset);
	/* } Terry Cheng, 20121226, Save smd and smsm wakeup app statistics */

	sys_sync();
	filp_close(filp, NULL);
	set_fs(oldfs);

	PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "Updating power management log done .\n");

}
/* 20121102, Terry Cheng, Check whether abnormal compoents usage {*/
void pmlog_trigger_abnormal_earlysuspend_components()
{

	struct pmlog_device *pmlog_dev;
	struct list_head *dev_list;
	PM_LOG_DPRINTK(PMLOG_DBG_ERR, "\n");
	
	//TODO: Remove hard code
	list_for_each(dev_list, &pmlog_list_head) {
		pmlog_dev = container_of(dev_list, struct pmlog_device, pmlog_list);
		if( (pmlog_dev->start_time.tv_sec > 0) || (pmlog_dev->start_time.tv_nsec > 0)){
			if (!strncmp(dev_name(pmlog_dev->dev), "pm8xxx-led.0", 12)){
				kevent_trigger(KEVENT_COMP_BACKLIGHT);
				return;
			} else if (!strncmp(dev_name(pmlog_dev->dev), "mipi_dsi.591105", 15)){
				kevent_trigger(KEVENT_COMP_LCD);
				return;
			}else if (!strncmp(dev_name(pmlog_dev->dev), "3-0020", 6)){
				kevent_trigger(KEVENT_COMP_TOUCH);		
				return;				
			}else if (!strncmp(dev_name(pmlog_dev->dev), "3-0038", 6)){
				kevent_trigger(KEVENT_COMP_TOUCH);		
				return;				
			}
		}	
	}
}
EXPORT_SYMBOL(pmlog_trigger_abnormal_earlysuspend_components);
/* } 20121102, Terry Cheng, Check whether abnormal compoents usage */

/* 20121102, Terry Cheng, Check whether abnormal camera compoents usage {*/
void pmlog_check_camera_components()
{

	struct pmlog_device *pmlog_dev;
	struct list_head *dev_list;
	unsigned int is_rear_camera_on = 0;
	unsigned int is_flash_light_on = 0;
	
	//TODO: Remove hard code
	list_for_each(dev_list, &pmlog_list_head) {
		pmlog_dev = container_of(dev_list, struct pmlog_device, pmlog_list);
		if( (pmlog_dev->start_time.tv_sec > 0) || (pmlog_dev->start_time.tv_nsec > 0)){
			if (!strncmp(dev_name(pmlog_dev->dev), "4-002a", 6)){
				//Rear camera 
				PM_LOG_DPRINTK(PMLOG_DBG_ERR, "Rear camera is on\n");
				is_rear_camera_on = 1;
			} else if (!strncmp(dev_name(pmlog_dev->dev), "4-0048", 6)){
				//Front camera 
				PM_LOG_DPRINTK(PMLOG_DBG_ERR, "Front camera is on\n");
				kevent_trigger(KEVENT_COMP_CAMERA);
				return;
			} else if (!strncmp(dev_name(pmlog_dev->dev), "4-0030", 6)){
				//Flash light
				PM_LOG_DPRINTK(PMLOG_DBG_ERR, "Flash light is on\n");
				is_flash_light_on = 1;
			}	
		}				
	}
	if((is_rear_camera_on == 1) && (is_flash_light_on == 0))
		kevent_trigger(KEVENT_COMP_CAMERA);
}
EXPORT_SYMBOL(pmlog_check_camera_components);
/* } 20121102, Terry Cheng, Check whether abnormal camera compoents usage */


/* 20121102, Terry Cheng, Check audio devices whether using {*/
int pmlog_check_audio_components()
{

	struct pmlog_device *pmlog_dev;
	struct list_head *dev_list;

	//TODO: Remove hard code
	list_for_each(dev_list, &pmlog_list_head) {
		pmlog_dev = container_of(dev_list, struct pmlog_device, pmlog_list);
		if( (pmlog_dev->start_time.tv_sec > 0) || (pmlog_dev->start_time.tv_nsec > 0)){
			if (!strncmp(dev_name(pmlog_dev->dev), "sitar_codec", 11)){
				PM_LOG_DPRINTK(PMLOG_DBG_ERR, "sitar_codec %d is on\n", pmlog_dev->index);
				return 1;
			} 
		}				
	}
	return 0;
}
EXPORT_SYMBOL(pmlog_check_audio_components);
/* } 20121102, Terry Cheng, Check audio devices whether using  */

int pmlog_device_on(struct pmlog_device *dev)
{
	if (!dev) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "%s: Error! Null Node.\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&pmlog_mutex);
	if (dev->count++ == 0) {
		dev->start_time = ktime_to_timespec(alarm_get_elapsed_realtime());
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "dev: %s start time: %ld.%ld dev->count = %lu\n", dev_name(dev->dev),  dev->start_time.tv_sec, dev->start_time.tv_nsec/NSEC_PER_MSEC, dev->count);
	}
	mutex_unlock(&pmlog_mutex);
	return 0;
}
EXPORT_SYMBOL(pmlog_device_on);

int pmlog_device_off(struct pmlog_device *dev)
{
	struct timespec end_time;
	struct timespec diff_time;//Terry Cheng, 20130116, Fix wifi hotspot and fm runtime eroor
	if (!dev) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "%s: Error! Null Node.\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&pmlog_mutex);
	if (dev->count) {
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "dev: %s before run time: %ld.%ld dev->count = %lu\n", dev_name(dev->dev), dev->run_time.tv_sec, dev->run_time.tv_nsec/NSEC_PER_MSEC, dev->count);
		end_time = ktime_to_timespec(alarm_get_elapsed_realtime());
		/* Terry Cheng, 20120822, Fix count twice device using time {*/
		if (timespec_compare(&dev->start_time, &latest_update_time) > 0){
			diff_time =  timespec_sub(end_time, dev->start_time);			
		}else{
			diff_time =  timespec_sub(end_time, latest_update_time);
		}
		dev->run_time = timespec_add_safe(dev->run_time,diff_time);
		//Terry Cheng, 20121218, update wifi hotspot and fm using time
		if(!strncmp(dev_name(dev->dev), "wcnss_wlan", 10) && (wifi_hotspot_status == 1)){
				 wifi_hotspot_run_time =timespec_add_safe(wifi_hotspot_run_time, diff_time);
		}else if (!strncmp(dev_name(dev->dev), "iris_fm", 7)){
				fm_run_time =timespec_add_safe(fm_run_time, diff_time);
		}
		/* } Terry Cheng, 20120822, Fix count twice device using time */
		dev->count = 0;
		dev->start_time.tv_sec = 0;
		dev->start_time.tv_nsec = 0;
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "dev: %s end run time: %ld.%ld\n", dev_name(dev->dev), dev->run_time.tv_sec, dev->run_time.tv_nsec/NSEC_PER_MSEC);
	}
	mutex_unlock(&pmlog_mutex);
	return 0;	
}
EXPORT_SYMBOL(pmlog_device_off);
static void pmlog_force_update(void)
{

	struct pmlog_device *pmlog_dev;
	struct list_head *dev_list;
	struct timespec end_time;
	struct timespec diff_time;//Terry Cheng, 20130116, Fix wifi hotspot and fm runtime eroor

	mutex_lock(&pmlog_mutex);
	end_time = ktime_to_timespec(alarm_get_elapsed_realtime());
	list_for_each(dev_list, &pmlog_list_head) {
		pmlog_dev = container_of(dev_list, struct pmlog_device, pmlog_list);
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "dev name %s.%d time = %lu\n", dev_name(pmlog_dev->dev), pmlog_dev->index, timespec_to_jiffies(&pmlog_dev->start_time));
		//Check whether start 
		if (0 == timespec_to_jiffies(&pmlog_dev->start_time))
			continue;
		if (pmlog_dev->count) {
			if (timespec_compare(&pmlog_dev->start_time, &latest_update_time) > 0){
				diff_time =  timespec_sub(end_time, pmlog_dev->start_time);			
			}else{
				diff_time =  timespec_sub(end_time, latest_update_time);
			}
			pmlog_dev->run_time = timespec_add_safe(pmlog_dev->run_time,diff_time);
			//Terry Cheng, 20121218, update wifi hotspot and fm using time
			if(!strncmp(dev_name(pmlog_dev->dev), "wcnss_wlan", 10) && (wifi_hotspot_status == 1)){
					 wifi_hotspot_run_time =timespec_add_safe(wifi_hotspot_run_time, diff_time);
			}else if(!strncmp(dev_name(pmlog_dev->dev), "iris_fm", 7)){
					fm_run_time =timespec_add_safe(fm_run_time, diff_time);
			}			
		}		
	}
	latest_update_time = ktime_to_timespec(alarm_get_elapsed_realtime());
	mutex_unlock(&pmlog_mutex);
}
static void pmlog_work_func(struct work_struct *work)
{
	int expired = 1;
	int new_round = 1;

	system_time = ktime_to_timespec(alarm_get_elapsed_realtime());
	if (timespec_compare(&alarm_time, &system_time) > 0) expired = 0;
	if (timespec_compare(&expire_time, &system_time) > 0) new_round = 0;
	/* Terry Cheng, 20130332, Dump more log for modem debug {*/
	update_alarm_time();
	/* } Terry Cheng, 20130332, Dump more log for modem debug */

	if (expired) {
		pmlog_force_update();
		if (new_round) {
			pm_move_oldfile(PMLOG);
			expire_time = timespec_add_safe(time_24hr, system_time);
		}
		pmlog_flush_to_file();
	}
	wake_unlock(&wake_lock_pmlog);
}

static void pmlog_alarm_func(struct alarm *alarm)
{
	wake_lock(&wake_lock_pmlog); // wake lock for 500ms.
	queue_work(pmlog_wqueue, &pmlog_work);
}

struct pmlog_device* pmlog_register_device(struct device *dev)
{
	struct pmlog_device *new;
	struct list_head *dev_list;
	/* Terry Cheng, 20120525, Support multiple register {*/
	int count = 0;

	if (!dev) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "%s: Error! Null device\n", __func__);
		return NULL;
	}

	list_for_each(dev_list, &pmlog_list_head) {
		new = container_of(dev_list, struct pmlog_device, pmlog_list);
		if (new->dev == dev) count++;
	}

	new = kzalloc(sizeof(struct pmlog_device), GFP_KERNEL);
	if (!new) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "%s: not enough memory?\n", __func__);
		return NULL;
	}

	new->dev = dev;
	new->index = count;
	INIT_LIST_HEAD(&new->pmlog_list);

	mutex_lock(&pmlog_mutex);
	list_add(&new->pmlog_list, &pmlog_list_head);
	mutex_unlock(&pmlog_mutex);

	PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "register device %s.%d\n", dev_name(dev), new->index);
	/* } Terry Cheng, 20120525, Support multiple register */
	return new;
}
EXPORT_SYMBOL(pmlog_register_device);

void pmlog_unregister_device(struct pmlog_device *dev )
{
	mutex_lock(&pmlog_mutex);
	list_del(&dev->pmlog_list);
	mutex_unlock(&pmlog_mutex);
	kfree(dev);	
}
EXPORT_SYMBOL(pmlog_unregister_device);
#if defined(CONFIG_DEBUG_FS)
#define MAX_DEBUG_BUFFER	2048  //Terry Cheng, 20120529, Incrase pmlog debugfs debug buffer
static char debug_buffer[MAX_DEBUG_BUFFER];
DEFINE_SIMPLE_ATTRIBUTE(pmlog_timeout_fops, pmlog_get_pmlog_timeout,
						pmlog_set_pmlog_timeout, "%llu\n");
/* Terry Cheng, 20120903 , Port show kernel TOP {*/
#if 1
DEFINE_SIMPLE_ATTRIBUTE(pmlog_kernel_top_enable_fops, pmlog_get_pmlog_kernel_top_enable,
						pmlog_set_pmlog_kernel_top_enable, "%llu\n");

static int pmlog_set_pmlog_kernel_top_enable(void *data, u64 val)
{
	enable_show_kernel_top = (long)val;
	if(enable_show_kernel_top == 1)
	{
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "queue show_kernel_top_work\n");
		queue_delayed_work(pmlog_wqueue, &show_kernel_top_work, msecs_to_jiffies(pmlog_kernel_top_delay_time));
	}
	else
	{
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "cancel show_kernel_top_work\n");
		cancel_delayed_work_sync(&show_kernel_top_work);
	}
	return 0;
}
static int pmlog_get_pmlog_kernel_top_enable(void *data, u64 *val)
{
	*val = (u64)enable_show_kernel_top;
	return 0;
}
/* } Terry Cheng, 20120903 , Port show kernel TOP */
#endif 

//Update to use debugfs interface 
static int pmlog_set_pmlog_timeout(void *data, u64 val)
{
	pmlog_timeout_s = (long)val;
	time_interval.tv_sec = pmlog_timeout_s;
	wake_lock(&wake_lock_pmlog); 
	queue_work(pmlog_wqueue, &pmlog_work);
	return 0;
}
static int pmlog_get_pmlog_timeout(void *data, u64 *val)
{
	*val = (u64)pmlog_timeout_s;
	return 0;
}
static int debug_read_pmlog(char *buf, int max)
{

	struct pmlog_device *pmlog_dev;
	struct list_head *dev_list;
	int size = 0;
	/* Terry Cheng, 20120520, Fix data abort {*/
	size += FORMAT_COLUMN_C(buf+size, MAX_DEBUG_BUFFER-size, "Device Name");
	size += FORMAT_COLUMN_C(buf+size, MAX_DEBUG_BUFFER-size, "Run Time");
	size += FORMAT_COLUMN_CN(buf+size, MAX_DEBUG_BUFFER-size, "Last Start");
	size += snprintf(buf+size, PAGE_SIZE-size, "===============================================================\n");

	list_for_each(dev_list, &pmlog_list_head) {
		pmlog_dev = container_of(dev_list, struct pmlog_device, pmlog_list);
		size += FORMAT_COLUMN_C(buf+size, MAX_DEBUG_BUFFER-size, "%s.%d", dev_name(pmlog_dev->dev), pmlog_dev->index);
		size += FORMAT_COLUMN_C(buf+size, MAX_DEBUG_BUFFER-size, "%ld.%ld", 
			pmlog_dev->run_time.tv_sec, pmlog_dev->run_time.tv_nsec/NSEC_PER_MSEC);
		size += FORMAT_COLUMN_CN(buf+size, MAX_DEBUG_BUFFER-size, "%ld.%ld",
			pmlog_dev->start_time.tv_sec, pmlog_dev->start_time.tv_nsec/NSEC_PER_MSEC);
	}
	/* }Terry Cheng, 20120520, Fix data abort */
	return size;
}
static ssize_t debug_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, MAX_DEBUG_BUFFER);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
};

#endif	/* CONFIG_DEBUG_FS */
//Save power off reason
static int pmlog_notify_sys(struct notifier_block *this, unsigned long code,
	void *unused)
{
	char *cmd = unused;
	int len;
	wake_lock(&wake_lock_pmlog);
	//Update pm log
	pmlog_force_update();
	pmlog_flush_to_file();
	pm_move_oldfile(PMLOG);
	
	if (cmd)
	{
		strncpy(shutdown_cmd, cmd, MAX_COMMAND_LEN);
		shutdown_cmd[MAX_COMMAND_LEN-1] = 0;
		PM_LOG_DPRINTK(PMLOG_DBG_TRACE, "Command:%s\n", cmd);
		if ((SYS_POWER_OFF == code) || (SYS_RESTART == code))
		{
			struct timespec ts;
			struct rtc_time tm;						
			//move old file
			pm_move_oldfile(POWEROFF);
			//Get Time information
			getnstimeofday(&ts);
			/* Terry Cheng, 20120626, Get timezone offset from share memory and do timezone offset {*/
			if(smem_vendor0_data)
			{
				//Do timezone offset
				PM_LOG_DPRINTK(PMLOG_DBG_TRACE,"Time zone = %d\n", smem_vendor0_data->time_zone);
				ts.tv_sec += (smem_vendor0_data->time_zone*60);
			}
			/* } Terry Cheng, 20120626, Get timezone offset from share memory and do timezone offset */

			rtc_time_to_tm(ts.tv_sec, &tm);			
			memset(buffer,0, PAGE_SIZE);
			//Save power off time
			len = snprintf(buffer, PAGE_SIZE, "Power Off Time: %d-%02d-%02d %02d:%02d:%02d.%09lu\n",
					tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
	                tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

			//Check power off reason
			if (SYS_POWER_OFF == code){
				if (!strncmp(cmd, "User", 5)) {
					poweroff_reason = 0x99887700;
				} else if (!strncmp(cmd, "Sys-Rq o", 9)) {
					poweroff_reason = 0x99887701;
				} else if (!strncmp(cmd, "Hibernation Shutdown", 21)) {
					poweroff_reason = 0x99887702;
				} else if (!strncmp(cmd, "Orderly", 8)) {
					poweroff_reason = 0x99887703;
				} else if (!strncmp(cmd, "Watchdog - Over Heat", 21)) {
					poweroff_reason = 0x99887704;
				} else if (!strncmp(cmd, "Power Key", 10)) {
					poweroff_reason = 0x99887705;
				}
				//Save power off reason
				if (poweroff_reason != 0x998877BB) 
					len += snprintf( buffer + len, PAGE_SIZE-len, "PowerOff reason: %s(%x)\n", cmd, poweroff_reason);
				else 
					len += snprintf( buffer+len, PAGE_SIZE-len, "PowerOff reason: Unknown(%x)\n", poweroff_reason);
			}else {
				len += snprintf( buffer + len, PAGE_SIZE-len, "Reboot reason: %s\n", cmd);
			}	
			//Save call stack
			if (!(len += dump_stack_to_buf(buffer+len, PAGE_SIZE-len)))
				PM_LOG_DPRINTK(PMLOG_DBG_ERR, "Copy kernel log failed\n");
			else {
				//Save log to file 	
				mm_segment_t oldfs;
				struct file *filp = NULL;
				unsigned long long offset = 0;

				oldfs = get_fs();
				set_fs(get_ds());
	            filp = filp_open(POWEROFF_FILE, O_CREAT|O_RDWR, S_IRWXU);
				if ( IS_ERR(filp) ) {
					set_fs(oldfs);
					PM_LOG_DPRINTK(PMLOG_DBG_ERR, "%s: Open file failed\n", __func__);
				}
				else {
					vfs_write(filp, buffer, len, &offset);
					vfs_fsync(filp, 0);
					sys_sync();
					filp_close(filp, NULL);
					set_fs(oldfs);
				}
			}			
		}
	}
	wake_unlock(&wake_lock_pmlog);
	return NOTIFY_DONE;
}

void pmlog_set_sleep_ops(struct msm_pm_sleep_ops *ops)
{
	if (ops)
		pmlog_sleep_ops = *ops;
}
static struct notifier_block pmlog_notifier = {
	.notifier_call = pmlog_notify_sys,
	.priority = INT_MAX,	/* before any real devices */
};

static int __devinit pmlog_init(void)
{
	INIT_LIST_HEAD(&pmlog_list_head);
	mutex_init(&pmlog_mutex);
	return 0;
}
arch_initcall(pmlog_init);
//pmlogd will save pm log to file every pm_log_save_time_interval
//DEFAULT pm_log_save_time_interval is every hour
static int __devinit pmlogd_init(void)
{
	int ret = 0;
	struct dentry *dent;

	memset(&battery_data, 0, sizeof(struct battery_inform_data));
	pmlog_wqueue = alloc_workqueue("pmlogd", WQ_UNBOUND, 1);	
	if (!pmlog_wqueue) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR,"%s: Create single thread real time workqueue failed.\n", __func__);
		goto fail;
	}
	buffer = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buffer) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "%s: Create debug buffer failed\n", __func__);
		goto err_destroy_workqueue;
	}

	//Register reboot notifier to flush pmlog to file when shutdown or reboot
	ret = register_reboot_notifier(&pmlog_notifier);
	if (ret) {
		PM_LOG_DPRINTK(PMLOG_DBG_ERR,"%s: cannot register reboot notifier. (err=0x%x)\n", __func__, ret);
		goto err_free_page;
	}

	//Iint a alarm to to notify pmlogd then flush pmlog to file
	alarm_init(&pmlog_alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP, pmlog_alarm_func);
	//Init wake lock when flushing log to file
	wake_lock_init(&wake_lock_pmlog, WAKE_LOCK_SUSPEND, "pmlog_lock");
	//Init work to flush pm log to file when alarm trigger
	INIT_WORK(&pmlog_work, pmlog_work_func);

#if 1
	/* Terry Cheng, 20120903 , Port show kernel TOP {*/
	//Init work to show kernel top
	INIT_DELAYED_WORK(&show_kernel_top_work, pmlog_show_kernel_top_work_func);
#endif
	/* } Terry Cheng, 20120903 , Port show kernel TOP */
	if (time_interval.tv_sec > 0){
		//Get the elapsed real time
		system_time = ktime_to_timespec(alarm_get_elapsed_realtime());
		//Set the alarm time via timespec_add_safe 
		alarm_time = timespec_add_safe(time_interval, system_time);
		//Start alarm
		alarm_start_range(&pmlog_alarm, 
			timespec_to_ktime(alarm_time),
			timespec_to_ktime(alarm_time));
	}
	latest_update_time = ktime_to_timespec(alarm_get_elapsed_realtime());	
#ifdef PM_LOG_SUPPORT_LOG_WAKE_SOURCE	
	memset(wake_count, 0 , sizeof(wake_count));
#endif	//PM_LOG_SUPPORT_LOG_WAKE_SOURCE

#if defined(CONFIG_DEBUG_FS)
	dent = debugfs_create_dir("pm_log", 0);
	debugfs_create_file("log", 0444, dent, debug_read_pmlog, &debug_ops);
	debugfs_create_file("interval", 0644, dent, 0, &pmlog_timeout_fops);
#if 1
	/* Terry Cheng, 20120903 , Port show kernel TOP {*/
	debugfs_create_file("enable_kernel_top", 0644, dent, 0, &pmlog_kernel_top_enable_fops);
#endif 
	/* } Terry Cheng, 20120903 , Port show kernel TOP */
	if (kernel_debuglevel_dir!=NULL)
	{
		debugfs_create_u32("pmlog_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&PMLOG_DLL));
	}
	else
	{
		PM_LOG_DPRINTK(PMLOG_DBG_ERR, "kernel_debuglevel_dir pmlog_dll Fail\n");
	}
#endif	//CONFIG_DEBUG_FS	

	/* Terry Cheng, 20120626, Get timezone offset from share memory {*/
	smem_vendor0_data = smem_alloc(SMEM_ID_VENDOR0, sizeof (smem_vendor_id0_amss_data));

	if (!smem_vendor0_data)
		pr_err("Alloc share memory SMEM_ID_VENDOR0 fail size = %d\n", sizeof (smem_vendor_id0_amss_data));
	/* } Terry Cheng, 20120626, Get timezone offset from share memory */

#if 1
	/* Terry Cheng, 20120903 , Port show kernel TOP {*/
	prev_proc_stat = vmalloc(sizeof(int) * MAX_PID);
	curr_proc_delta = vmalloc(sizeof(int) * MAX_PID);
	task_ptr_array = vmalloc(sizeof(int) * MAX_PID);
	memset(prev_proc_stat, 0, sizeof(int) * MAX_PID);
	memset(curr_proc_delta, 0, sizeof(int) * MAX_PID);
	memset(task_ptr_array, 0, sizeof(int) * MAX_PID);
	get_all_cpu_stat(&new_cpu_stat);
	get_all_cpu_stat(&old_cpu_stat);
	/* } Terry Cheng, 20120903 , Port show kernel TOP */
#endif
	spin_lock_init(&lock); //Terry Cheng, 20121015, Fix not init spin lock

	return 0;

err_free_page:
	kfree(buffer);
err_destroy_workqueue:
	destroy_workqueue(pmlog_wqueue);
fail:
	return ret;
}
late_initcall_sync(pmlogd_init);

