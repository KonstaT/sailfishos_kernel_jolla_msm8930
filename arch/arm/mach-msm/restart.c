/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>
#include <linux/mfd/pm8xxx/misc.h>

#include <asm/mach-types.h>

#include <mach/msm_iomap.h>
#include <mach/restart.h>
#include <mach/socinfo.h>
#include <mach/irqs.h>
#include <mach/scm.h>
#include "msm_watchdog.h"
#include "timer.h"

#include <mach/hwid.h>
#include <mach/boot_mode.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#ifdef CONFIG_PANIC_LASTLOG
#include <linux/kallsyms.h>
#endif
/* Terry Cheng, 20120525, Add flush cache */
#include <asm/cacheflush.h>
/* }Terry Cheng, 20120525, Add flush cache {*/
/* Terry Cheng, 20120612, Vibrate to notify QA that kernel trigger capture hotkey log process {*/
#include <linux/mfd/pm8xxx/vibrator.h>
/* }Terry Cheng, 20120612, Vibrate to notify QA that kernel trigger capture hotkey log process */


#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/delay.h>
typedef enum
{
	DDRTEST_READY  = 0,
	DDRTEST_RUN    = 1,
	DDRTEST_STOP   = 2,
	DDRTEST_REPORT = 3,
	DDRTEST_UNKNOW = 4,
}  DDRTEST_STATE_TYPE;
#define DDRTEST_STATE_BUF_LEN 10
char ddrtest_report_str[DDRTEST_UNKNOW+1][DDRTEST_STATE_BUF_LEN]={"ready","run","stop","report","unknow"};
static char ddrtest_state_buf[DDRTEST_STATE_BUF_LEN];
static DDRTEST_STATE_TYPE current_ddrtest_state=DDRTEST_UNKNOW;
static int ddrtest_state_open(struct inode *inode, struct file *filp)  {
    return 0;
}

static ssize_t ddrtest_state_read(struct file *file, char __user *buff, size_t count, loff_t *ppos)  {
	int len = 0;
	if (*ppos)	return 0;	/* the end */
	len = snprintf(ddrtest_state_buf, sizeof(ddrtest_state_buf), "%s\n",ddrtest_report_str[current_ddrtest_state]);
	if (copy_to_user(buff, ddrtest_state_buf, len))
		return -EFAULT;
	*ppos += len;
	return len;
}

static ssize_t ddrtest_state_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos)  {
	if (count >=sizeof(ddrtest_state_buf))	return -EFAULT;
	if (copy_from_user(ddrtest_state_buf,buff,count))	return -EFAULT;
	ddrtest_state_buf[count-1]='\0';
	     if (strcmp(ddrtest_state_buf,ddrtest_report_str[DDRTEST_READY])==0)         current_ddrtest_state=DDRTEST_READY;
	else if (strcmp(ddrtest_state_buf,ddrtest_report_str[DDRTEST_RUN])==0)           current_ddrtest_state=DDRTEST_RUN;
	else if (strcmp(ddrtest_state_buf,ddrtest_report_str[DDRTEST_STOP])==0)          current_ddrtest_state=DDRTEST_STOP;
	else if (strcmp(ddrtest_state_buf,ddrtest_report_str[DDRTEST_REPORT])==0)        current_ddrtest_state=DDRTEST_REPORT;
	else                                                                             current_ddrtest_state=DDRTEST_UNKNOW;
	/*
	printk(KERN_ERR "%s, %s, %s, %s\n",
		ddrtest_report_str[DDRTEST_READY],ddrtest_report_str[DDRTEST_STOP],
		ddrtest_report_str[DDRTEST_REPORT],ddrtest_report_str[DDRTEST_UNKNOW]);
        printk(KERN_ERR "len= %d, %d, %d, %d\n",
                strlen(ddrtest_report_str[DDRTEST_READY]),strlen(ddrtest_report_str[DDRTEST_STOP]),
                strlen(ddrtest_report_str[DDRTEST_REPORT]),strlen(ddrtest_report_str[DDRTEST_UNKNOW]));
	printk(KERN_ERR "%s, %d, %d\n",ddrtest_state_buf,strlen(ddrtest_state_buf),count);
	*/
	return count;
}

static const struct file_operations ddrtest_state_fops = {
.owner = THIS_MODULE,
.open =  ddrtest_state_open,
.read =  ddrtest_state_read,
.write = ddrtest_state_write,
};
#endif

#define WDT0_RST	0x38
#define WDT0_EN		0x40
#define WDT0_BARK_TIME	0x4C
#define WDT0_BITE_TIME	0x5C

#define PSHOLD_CTL_SU (MSM_TLMM_BASE + 0x820)

#define RESTART_REASON_ADDR 0x65C
#define DLOAD_MODE_ADDR     0x0

#define SCM_IO_DISABLE_PMIC_ARBITER	1

#ifdef CONFIG_MSM_RESTART_V2
#define use_restart_v2()	1
#else
#define use_restart_v2()	0
#endif

static int restart_mode;
void *restart_reason;

int pmic_reset_irq;
static void __iomem *msm_tmr0_base;

#ifdef CONFIG_MSM_DLOAD_MODE
static int in_panic;
static void *dload_mode_addr;

/* Download mode master kill-switch */
static int dload_set(const char *val, struct kernel_param *kp);
static int download_mode = 0;
module_param_call(download_mode, dload_set, param_get_int,
			&download_mode, 0644);

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
/* Bright Lee, 20130222, back door to notify tool about modem crash in factory build { */
#ifdef CONFIG_BUILD_FACTORY
	if (strstr((void *)ptr, "modem crashed") != NULL) {
		in_panic = 2;
	} else 
#endif
/* } Bright Lee, 20130222 */
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

static void set_dload_mode(int on)
{
	if (dload_mode_addr) {
/* Bright Lee, 20130222, back door to notify tool about modem crash in factory build { */
#ifdef CONFIG_BUILD_FACTORY
		int magic[] = {0, DLOAD_MAGIC_NUM2, DLOAD_MAGIC_NUM3};
#endif
/* } Bright Lee, 20130222 */
		__raw_writel(on ? DLOAD_MAGIC_NUM1: 0, dload_mode_addr);
/* Bright Lee, 20130222, back door to notify tool about modem crash in factory build { */
#ifdef CONFIG_BUILD_FACTORY
		__raw_writel(magic[on], dload_mode_addr + sizeof(unsigned int));
#else
		__raw_writel(on ?  DLOAD_MAGIC_NUM2: 0, 
			dload_mode_addr + sizeof(unsigned int));
#endif
/* } Bright Lee, 20130222 */
		mb();
	}
}

static int dload_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = download_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not zero or one, ignore. */
	if (download_mode >> 1) {
		download_mode = old_val;
		return -EINVAL;
	}

	set_dload_mode(download_mode);

	return 0;
}
#else
#define set_dload_mode(x) do {} while (0)
#endif

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

static void __msm_power_off(int lower_pshold)
{
	printk(KERN_CRIT "Powering off the SoC\n");
#ifdef CONFIG_MSM_DLOAD_MODE
	set_dload_mode(0);
#endif
	pm8xxx_reset_pwr_off(0);

	if (lower_pshold) {
		if (!use_restart_v2())
			__raw_writel(0, PSHOLD_CTL_SU);
		else
			__raw_writel(0, MSM_MPM2_PSHOLD_BASE);

		mdelay(10000);
		printk(KERN_ERR "Powering off has failed\n");
	}
	return;
}

static void msm_power_off(void)
{
	/* MSM initiated power off, lower ps_hold */
	__msm_power_off(1);
}

static void cpu_power_off(void *data)
{
	int rc;

	pr_err("PMIC Initiated shutdown %s cpu=%d\n", __func__,
						smp_processor_id());
	if (smp_processor_id() == 0) {
		/*
		 * PMIC initiated power off, do not lower ps_hold, pmic will
		 * shut msm down
		 */
		__msm_power_off(0);

		pet_watchdog();
		pr_err("Calling scm to disable arbiter\n");
		/* call secure manager to disable arbiter and never return */
		rc = scm_call_atomic1(SCM_SVC_PWR,
						SCM_IO_DISABLE_PMIC_ARBITER, 1);

		pr_err("SCM returned even when asked to busy loop rc=%d\n", rc);
		pr_err("waiting on pmic to shut msm down\n");
	}

	preempt_disable();
	while (1)
		;
}

/* Bright Lee, 20120622, save bootup reason for hotkey { */
#ifdef CONFIG_PANIC_LASTLOG
void store_panic_caller(int addr);
static void hotkey(void)
{
	return;
}
#endif
/* } Bright Lee, 20120622 */

static irqreturn_t resout_irq_handler(int irq, void *dev_id)
{
	/* Terry Cheng, 20120528, Add Volume up and volume done key status for restart mode {*/
	extern unsigned int vol_down_key_status;		//Keycode 114
	/*} Terry Cheng, 20120528, Add Volume up and volume done key status for restart mode */
	/* Terry Cheng, Check download_mode whether enable. If download_mode enable, just abnormal reboot {*/
	pr_warn("%s PMIC Initiated shutdown, download_mode = %d vol_down_key_status = %d\n", __func__, download_mode, vol_down_key_status);
	if ((download_mode != 0) && (vol_down_key_status != 0)) {
		/* Terry Cheng, 20120612, Vibrate to notify QA that kernel trigger capture hotkey log process {*/
		struct pm8xxx_vib_config vib_config;
		memset(&vib_config, 0, sizeof(struct pm8xxx_vib_config));
		/* Bright Lee, 20120622, save bootup reason for hotkey { */
                #ifdef CONFIG_PANIC_LASTLOG
		store_panic_caller((int)&hotkey);
                #endif
		/* } Bright Lee, 20120622 */
		//First disable hard reset
		pm8xxx_hard_reset_config(PM8XXX_DISABLE_HARD_RESET);
		vib_config.active_low = 0;
		vib_config.drive_mV = 3000;
		vib_config.enable_mode = PM8XXX_VIB_MANUAL;
		pm8xxx_vibrator_config(&vib_config);
		//Vibrate 0.5s
		mdelay(500);
		//Disable vibrate
		vib_config.drive_mV = 0;
		pm8xxx_vibrator_config(&vib_config);
		/* } Terry Cheng, 20120612, Vibrate to notify QA that kernel trigger capture hotkey log process */
		msm_restart('0', ABNORMAL_REBOOT_STR);
	}
	/* } Terry Cheng, Check download_mode whether enable. If download_mode enable, just abnormal reboot */

	oops_in_progress = 1;
	smp_call_function_many(cpu_online_mask, cpu_power_off, NULL, 0);
	if (smp_processor_id() == 0)
		cpu_power_off(NULL);
	preempt_disable();
	while (1)
		;

	return IRQ_HANDLED;
}

static void msm_restart_prepare(const char *cmd)
{
#ifdef CONFIG_MSM_DLOAD_MODE

	/* Bright Lee, 20140423, ramdump feature enabled only by qdlmode { */
	#if 0
	/* This looks like a normal reboot at this point. */
	set_dload_mode(download_mode);
	#endif
	/* } Bright Lee, 20140423 */

	/* Bright Lee, 20140423, ramdump feature enabled only by qdlmode { */
	/* Write download mode flags if we're panic'ing */
	set_dload_mode(download_mode?in_panic:0);
	/* } Bright Lee, 20140423 */

	/* Write download mode flags if restart_mode says so */
	if (restart_mode == RESTART_DLOAD)
		set_dload_mode(1);

	/* Bright Lee, 20140423, ramdump feature enabled only by qdlmode { */
	#if 0
	/* Kill download mode if master-kill switch is set */
	if (!download_mode)
		set_dload_mode(0);
	#endif
	/* } Bright Lee, 20140423 */
#endif

	pm8xxx_reset_pwr_off(1);

	if (cmd != NULL) {
		if (!strncmp(cmd, "bootloader", 10)) {
			__raw_writel(0x77665500, restart_reason);
		} else if (!strncmp(cmd, "recovery", 8)) {
			__raw_writel(0x77665502, restart_reason);
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned long code;
			code = simple_strtoul(cmd + 4, NULL, 16) & 0xff;
			__raw_writel(0x6f656d00 | code, restart_reason);
		} else {
			BOOT_INFO_TABLE;
			unsigned int i, n;
			unsigned int reason = NONE_MODE;  // NONE_MODE==0x77665501

			n = sizeof(bootup_reasons) / sizeof(bootup_reasons[0]);
			for (i=0; i<n; i++) {
				if (!strcmp(cmd, bootup_reasons[i].str_reason)) {
					reason = bootup_reasons[i].reason;
					break;
				}
			}
			if (i == n)
				goto normal_reboot;

			switch(reason) {
				/* not presented in original codes */
				case FASTBOOT_MODE:
				case FASTBOOT_REBOOT_MODE:
				case EUU_HEX_2_FASTBOOT:
				case RD_HEX_2_FASTBOOT:
				case SD_RECOVERY_MODE:
				case MENU_RECOVERY_MODE:
					goto normal_reboot;

				/* need factory cable, or reboot normally */
				case REBOOT_FOR_DDRTEST:
				case RECOVERY_REBOOT_MODE:
				case ENABLE_SECURE_BOOT:
					if (QcableVar != QcableFACTORY)
						goto normal_reboot;
					break;

				/* not allowed in ship version */
#ifdef CONFIG_BUILD_SHIP
				case ENABLE_SBL2_DDRTEST:
				case ENABLE_BIST_MODE:
				case ENTER_MASS_STORAGE_MODE:
					goto normal_reboot;
#endif

				default:
					break;
			}
			__raw_writel(reason, restart_reason);
		}
		return;

normal_reboot:
		__raw_writel(0x77665501, restart_reason);
	}
}

void msm_restart(char mode, const char *cmd)
{
	printk(KERN_NOTICE "Going down for restart now\n");

	msm_restart_prepare(cmd);

	/* Terry Cheng, 20120525, Add flush cache {*/
	flush_cache_all();
	outer_flush_all();
	/* }Terry Cheng, 20120525, Add flush cache */

	if (!use_restart_v2()) {
		__raw_writel(0, msm_tmr0_base + WDT0_EN);
		if (!(machine_is_msm8x60_fusion() ||
		      machine_is_msm8x60_fusn_ffa())) {
			mb();
			 /* Actually reset the chip */
			__raw_writel(0, PSHOLD_CTL_SU);
			mdelay(5000);
			pr_notice("PS_HOLD didn't work, falling back to watchdog\n");
		}

		__raw_writel(1, msm_tmr0_base + WDT0_RST);
		__raw_writel(5*0x31F3, msm_tmr0_base + WDT0_BARK_TIME);
		__raw_writel(0x31F3, msm_tmr0_base + WDT0_BITE_TIME);
		__raw_writel(1, msm_tmr0_base + WDT0_EN);
	} else
		__raw_writel(0, MSM_MPM2_PSHOLD_BASE);

	mdelay(10000);
	printk(KERN_ERR "Restarting has failed\n");
}

static int __init msm_pmic_restart_init(void)
{
	int rc;

	if (pmic_reset_irq != 0) {
		rc = request_any_context_irq(pmic_reset_irq,
					resout_irq_handler, IRQF_TRIGGER_HIGH,
					"restart_from_pmic", NULL);
		if (rc < 0)
			pr_err("pmic restart irq fail rc = %d\n", rc);
	} else {
		pr_warn("no pmic restart interrupt specified\n");
	}

	return 0;
}

late_initcall(msm_pmic_restart_init);

static int __init msm_restart_init(void)
{
#ifdef CONFIG_MSM_DLOAD_MODE
	extern int ramdump_enabled;
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	/* Bright Lee, 20140423, ramdump feature enabled only by qdlmode { */
	download_mode = ramdump_enabled;
	/* } Bright Lee, 20140423 */
	dload_mode_addr = MSM_IMEM_BASE + DLOAD_MODE_ADDR;
	set_dload_mode(download_mode);
#endif
	msm_tmr0_base = msm_timer_get_timer0_base();
	restart_reason = MSM_IMEM_BASE + RESTART_REASON_ADDR;
	pm_power_off = msm_power_off;

	/* Bright Lee, 20121211, inject abnormal reboot if watchdog reset { */
	__raw_writel(ABNORMAL_REBOOT, restart_reason);
	/* } Bright Lee, 20121211 */

	return 0;
}
early_initcall(msm_restart_init);

/* Bright Lee, 20111123, save bootup_reason { */
static char *bootup_reason_ptr = NULL;

static int __init parse_bootup_reason(char *str)
{
	bootup_reason_ptr = str;
	return 0;
}


static int proc_read_bootup_reason (char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
	#ifdef CONFIG_PANIC_LASTLOG
	extern unsigned int panic_caller;
	#endif
	len = sprintf (page, "%s", bootup_reason_ptr);
	#ifdef CONFIG_PANIC_LASTLOG
	if (!strcmp(bootup_reason_ptr, ABNORMAL_REBOOT_STR))
	{
		if (panic_caller != 0) {
			page[len++] = '-';
			kallsyms_lookup(panic_caller, NULL, NULL, NULL, page+len);
		}
	}
	#endif
	return strlen(page);
}


static int __init proc_bootup_reason(void)
{
	struct proc_dir_entry *pe;
	if (bootup_reason_ptr == NULL) {
		return 0;
	}

	pe = create_proc_read_entry("bootup_reason", S_IRUGO, NULL, proc_read_bootup_reason, NULL);
	if (!pe) {
		printk ("create bootup_reason proc failed\n");
		return -1;
	}
	else if (!strcmp(bootup_reason_ptr,REBOOT_FOR_DDRTEST_STR))  {
#ifdef CONFIG_DEBUG_FS
		if (debugfs_create_file("ddrtest_state", 0666, NULL, NULL, &ddrtest_state_fops)==NULL)  {
			printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",__FILE__,__LINE__);
		}
		else
			current_ddrtest_state=DDRTEST_READY;
#endif
	}
	return 0;

}

__setup("bootup_reason=", parse_bootup_reason);
module_init(proc_bootup_reason);
/* } Bright Lee, 20111123 */

