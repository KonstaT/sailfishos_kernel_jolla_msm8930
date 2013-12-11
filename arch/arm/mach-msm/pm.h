/* arch/arm/mach-msm/pm.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2012, The Linux Foundation. All rights reserved.
 * Author: San Mehat <san@android.com>
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

#ifndef __ARCH_ARM_MACH_MSM_PM_H
#define __ARCH_ARM_MACH_MSM_PM_H

#include <linux/types.h>
#include <linux/cpuidle.h>

#ifdef CONFIG_SMP
extern void msm_secondary_startup(void);
#else
#define msm_secondary_startup NULL
#endif

extern int power_collapsed;

struct msm_pm_irq_calls {
	unsigned int (*irq_pending)(void);
	int (*idle_sleep_allowed)(void);
	void (*enter_sleep1)(bool modem_wake, int from_idle, uint32_t
								*irq_mask);
	int (*enter_sleep2)(bool modem_wake, int from_idle);
	void (*exit_sleep1)(uint32_t irq_mask, uint32_t wakeup_reason,
							uint32_t pending_irqs);
	void (*exit_sleep2)(uint32_t irq_mask, uint32_t wakeup_reason,
							uint32_t pending_irqs);
	void (*exit_sleep3)(uint32_t irq_mask, uint32_t wakeup_reason,
							uint32_t pending_irqs);
};

enum msm_pm_sleep_mode {
	MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT = 0,
	MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT = 1,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE = 2,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE = 3,
	MSM_PM_SLEEP_MODE_APPS_SLEEP = 4,
	MSM_PM_SLEEP_MODE_RETENTION = MSM_PM_SLEEP_MODE_APPS_SLEEP,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND = 5,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN = 6,
	MSM_PM_SLEEP_MODE_NR
};

#define MSM_PM_MODE(cpu, mode_nr)  ((cpu) * MSM_PM_SLEEP_MODE_NR + (mode_nr))

struct msm_pm_time_params {
	uint32_t latency_us;
	uint32_t sleep_us;
	uint32_t next_event_us;
	uint32_t modified_time_us;
};

struct msm_pm_platform_data {
	u8 idle_supported;   /* Allow device to enter mode during idle */
	u8 suspend_supported; /* Allow device to enter mode during suspend */
	u8 suspend_enabled;  /* enabled for suspend */
	u8 idle_enabled;     /* enabled for idle low power */
	u32 latency;         /* interrupt latency in microseconds when entering
				and exiting the low power mode */
	u32 residency;       /* time threshold in microseconds beyond which
				staying in the low power mode saves power */
};

extern struct msm_pm_platform_data msm_pm_sleep_modes[];

struct msm_pm_sleep_ops {
	struct msm_rpmrs_limits *(*lowest_limits)(bool from_idle,
			//20120919, Terry Cheng, Show RPM resource log when suspend 
			enum msm_pm_sleep_mode sleep_mode,
			struct msm_pm_time_params *time_param, uint32_t *power);
	int (*enter_sleep)(uint32_t sclk_count, void *limits,
			bool from_idle, bool notify_rpm);
	void (*exit_sleep)(void *limits, bool from_idle,
			bool notify_rpm, bool collapsed);
};

struct msm_pm_cpr_ops {
	void (*cpr_suspend)(void);
	void (*cpr_resume)(void);
};

void msm_pm_set_platform_data(struct msm_pm_platform_data *data, int count);
int msm_pm_idle_prepare(struct cpuidle_device *dev,
			struct cpuidle_driver *drv, int index);
void msm_pm_set_irq_extns(struct msm_pm_irq_calls *irq_calls);
int msm_pm_idle_enter(enum msm_pm_sleep_mode sleep_mode);
void msm_pm_cpu_enter_lowpower(unsigned int cpu);
void __init msm_pm_set_tz_retention_flag(unsigned int flag);

#ifdef CONFIG_MSM_PM8X60
void msm_pm_set_rpm_wakeup_irq(unsigned int irq);
void msm_pm_set_sleep_ops(struct msm_pm_sleep_ops *ops);
//Terry Cheng, 20121101, Parse rpm log
#ifdef CONFIG_PM_LOG
void pmlog_set_sleep_ops(struct msm_pm_sleep_ops *ops);
#endif //CONFIG_PM_LOG
#else
static inline void msm_pm_set_rpm_wakeup_irq(unsigned int irq) {}
static inline void msm_pm_set_sleep_ops(struct msm_pm_sleep_ops *ops) {}
//Terry Cheng, 20121101, Parse rpm log
#ifdef CONFIG_PM_LOG
static inline void pmlog_set_sleep_ops(struct msm_pm_sleep_ops *ops){}
#endif	//CONFIG_PM_LOG
#endif
#ifdef CONFIG_HOTPLUG_CPU
int msm_platform_secondary_init(unsigned int cpu);
#else
static inline int msm_platform_secondary_init(unsigned int cpu) { return 0; }
#endif

enum msm_pm_time_stats_id {
	MSM_PM_STAT_REQUESTED_IDLE = 0,
	MSM_PM_STAT_IDLE_SPIN,
	MSM_PM_STAT_IDLE_WFI,
	MSM_PM_STAT_RETENTION,
	MSM_PM_STAT_IDLE_STANDALONE_POWER_COLLAPSE,
	MSM_PM_STAT_IDLE_FAILED_STANDALONE_POWER_COLLAPSE,
	MSM_PM_STAT_IDLE_POWER_COLLAPSE,
	MSM_PM_STAT_IDLE_FAILED_POWER_COLLAPSE,
	MSM_PM_STAT_SUSPEND,
	MSM_PM_STAT_FAILED_SUSPEND,
	MSM_PM_STAT_NOT_IDLE,
	MSM_PM_STAT_COUNT
};

#ifdef CONFIG_MSM_IDLE_STATS
void msm_pm_add_stats(enum msm_pm_time_stats_id *enable_stats, int size);
void msm_pm_add_stat(enum msm_pm_time_stats_id id, int64_t t);
#else
static inline void msm_pm_add_stats(enum msm_pm_time_stats_id *enable_stats,
		int size) {}
static inline void msm_pm_add_stat(enum msm_pm_time_stats_id id, int64_t t) {}
#endif

void msm_pm_set_cpr_ops(struct msm_pm_cpr_ops *ops);
extern void *msm_pc_debug_counters;
extern unsigned long msm_pc_debug_counters_phys;
#endif  /* __ARCH_ARM_MACH_MSM_PM_H */
