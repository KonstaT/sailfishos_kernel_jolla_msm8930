/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/stringify.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>

#include <mach/irqs.h>
#include <mach/scm.h>
#include <mach/peripheral-loader.h>
#include <mach/subsystem_restart.h>
#include <mach/subsystem_notif.h>
#include <mach/socinfo.h>
#include <mach/msm_smsm.h>

#include "smd_private.h"
#include "modem_notifier.h"
#include "ramdump.h"

static int crash_shutdown;

/* Terry Cheng, 20120516, Notify logmaster modem restarted and need to move ramdump file {*/
#include <mach/kevent.h>
/*} Terry Cheng, 20120516, Notify logmaster modem restarted and need to move ramdump file */

/* Terry Cheng, 20120531, Check modem restart reason to determine whether save ramdump {*/
#include <mach/boot_mode.h>
#include <mach/oem_smem_struct.h>
/* }Terry Cheng, 20120531, Check modem restart reason to determine whether save ramdump */

/* Terry Cheng, 20120529, Add modem crash count for auto pin  {*/
#include <linux/proc_fs.h>
static int modem_crash_count = 0;
/* }Terry Cheng, 20120529, Add modem crash count for auto pin  */

/* Terry Cheng, 20120531, Check modem restart reason to determine whether save ramdump {*/
static unsigned long int modem_reset_reason =  0;
smem_vendor_id0_amss_data *vendor0_data;
/* } Terry Cheng, 20120531, Check modem restart reason to determine whether save ramdump */


static struct subsys_device *modem_8960_dev;

#define MAX_SSR_REASON_LEN 81U

static void log_modem_sfr(void)
{
	u32 size;
	char *smem_reason, reason[MAX_SSR_REASON_LEN];

	smem_reason = smem_get_entry(SMEM_SSR_REASON_MSS0, &size);
	if (!smem_reason || !size) {
		pr_err("modem subsystem failure reason: (unknown, smem_get_entry failed).\n");
		return;
	}
	if (!smem_reason[0]) {
		pr_err("modem subsystem failure reason: (unknown, init string found).\n");
		return;
	}

	size = min(size, MAX_SSR_REASON_LEN-1);
	memcpy(reason, smem_reason, size);
	reason[size] = '\0';
	pr_err("modem subsystem failure reason: %s.\n", reason);

	smem_reason[0] = '\0';
	wmb();
}

static void restart_modem(void)
{
	log_modem_sfr();
	subsystem_restart_dev(modem_8960_dev);
}

static void smsm_state_cb(void *data, uint32_t old_state, uint32_t new_state)
{
	/* Ignore if we're the one that set SMSM_RESET */
	if (crash_shutdown)
		return;

	if (new_state & SMSM_RESET) {
		pr_err("Probable fatal error on the modem.\n");
		restart_modem();
	}
}

#define Q6_FW_WDOG_ENABLE		0x08882024
#define Q6_SW_WDOG_ENABLE		0x08982024

static int modem_shutdown(const struct subsys_desc *subsys)
{
	void __iomem *q6_fw_wdog_addr;
	void __iomem *q6_sw_wdog_addr;

	/*
	 * Disable the modem watchdog since it keeps running even after the
	 * modem is shutdown.
	 */
	q6_fw_wdog_addr = ioremap_nocache(Q6_FW_WDOG_ENABLE, 4);
	if (!q6_fw_wdog_addr)
		return -ENOMEM;

	q6_sw_wdog_addr = ioremap_nocache(Q6_SW_WDOG_ENABLE, 4);
	if (!q6_sw_wdog_addr) {
		iounmap(q6_fw_wdog_addr);
		return -ENOMEM;
	}

	writel_relaxed(0x0, q6_fw_wdog_addr);
	writel_relaxed(0x0, q6_sw_wdog_addr);
	mb();
	iounmap(q6_sw_wdog_addr);
	iounmap(q6_fw_wdog_addr);

	pil_force_shutdown("modem");
	pil_force_shutdown("modem_fw");
	disable_irq_nosync(Q6FW_WDOG_EXPIRED_IRQ);
	disable_irq_nosync(Q6SW_WDOG_EXPIRED_IRQ);

	/* Terry Cheng, 20120531, Move show modem crash err log and flush err log to file after diable modem watch dog {*/
	smd_diag();
	/* }Terry Cheng, 20120531, Move show modem crash err log and flush err log to file after diable modem watch dog */

/* Terry Cheng, 20120903, Disable modem subsystem restart in Factory version {*/
#ifdef CONFIG_BUILD_FACTORY
	if (vendor0_data){	
		modem_reset_reason = vendor0_data->modem_reboot_reason;
		pr_info("%s modem_reset_reason = %lu\n", __FUNCTION__, modem_reset_reason);
	}
	if(modem_reset_reason != NONE_MODE)
		panic("subsys-restart: modem crashed.\n");
#endif //CONFIG_BUILD_FACTORY
/* } Terry Cheng, 20120903, Disable modem subsystem restart in Factory version */
	return 0;
}

#define MODEM_WDOG_CHECK_TIMEOUT_MS 10000

static int modem_powerup(const struct subsys_desc *subsys)
{
	/* Bright Lee, 20130206, reset modem_reboot_reason when powerdown, will cause ramdump each time { */
	if (vendor0_data){
		 vendor0_data->modem_reboot_reason = 0;
	}
	/* Bright Lee, 20130206 */
	pil_force_boot("modem_fw");
	pil_force_boot("modem");
	enable_irq(Q6FW_WDOG_EXPIRED_IRQ);
	enable_irq(Q6SW_WDOG_EXPIRED_IRQ);
	/* Terry Cheng, 20120529, Add modem crash count for auto pin  {*/
	modem_crash_count++;
	/* Terry Cheng, 20120529, Add modem crash count for auto pin  {*/
	return 0;
}

void modem_crash_shutdown(const struct subsys_desc *subsys)
{
	crash_shutdown = 1;
	smsm_reset_modem(SMSM_RESET);
}

/* FIXME: Get address, size from PIL */
static struct ramdump_segment modemsw_segments[] = {
	{0x89000000, 0x8D400000 - 0x89000000},
};

static struct ramdump_segment modemfw_segments[] = {
	{0x8D400000, 0x8DA00000 - 0x8D400000},
};

static struct ramdump_segment smem_segments[] = {
	{0x80000000, 0x00200000},
};

static void *modemfw_ramdump_dev;
static void *modemsw_ramdump_dev;
static void *smem_ramdump_dev;

static int modem_ramdump(int enable, const struct subsys_desc *crashed_subsys)
{
	int ret = 0;
	/* Terry Cheng, 20120531, Check modem restart reason to determine whether save ramdump {*/
	if (vendor0_data){
		modem_reset_reason = vendor0_data->modem_reboot_reason;
		pr_info("%s modem_reset_reason = %lu\n", __FUNCTION__, modem_reset_reason);
	}

	if (enable && (modem_reset_reason != NONE_MODE)) {
		ret = do_ramdump(modemsw_ramdump_dev, modemsw_segments,
			ARRAY_SIZE(modemsw_segments));

		if (ret < 0) {
			pr_err("Unable to dump modem sw memory (rc = %d).\n",
			       ret);
			goto out;
		}

		ret = do_ramdump(modemfw_ramdump_dev, modemfw_segments,
			ARRAY_SIZE(modemfw_segments));

		if (ret < 0) {
			pr_err("Unable to dump modem fw memory (rc = %d).\n",
				ret);
			goto out;
		}

		ret = do_ramdump(smem_ramdump_dev, smem_segments,
			ARRAY_SIZE(smem_segments));

		if (ret < 0) {
			pr_err("Unable to dump smem memory (rc = %d).\n", ret);
			goto out;
		}
		/* Terry Cheng, 20120516, Notify logmaster modem restarted and need to move ramdump file {*/
		kevent_trigger(KEVENT_MODEM_CRASH);
		/* } Terry Cheng, 20120516, Notify logmaster modem restarted and need to move ramdump file */
	}
	/* } Terry Cheng, 20120531, Check modem restart reason to determine whether save ramdump */
	/* Bright Lee, 20130322, send uevent to logmaster for save smem log only { */
	else {
		kevent_trigger (KEVENT_MODEM_SMEM_LOG);
	}
	/* } Bright Lee, 20130322 */
out:
	return ret;
}

static irqreturn_t modem_wdog_bite_irq(int irq, void *dev_id)
{
	switch (irq) {

	case Q6SW_WDOG_EXPIRED_IRQ:
		pr_err("Watchdog bite received from modem software!\n");
		restart_modem();
		break;
	case Q6FW_WDOG_EXPIRED_IRQ:
		pr_err("Watchdog bite received from modem firmware!\n");
		restart_modem();
		break;
	break;

	default:
		pr_err("%s: Unknown IRQ!\n", __func__);
	}

	return IRQ_HANDLED;
}

static struct subsys_desc modem_8960 = {
	.name = "modem",
	.shutdown = modem_shutdown,
	.powerup = modem_powerup,
	.ramdump = modem_ramdump,
	.crash_shutdown = modem_crash_shutdown
};

static int modem_subsystem_restart_init(void)
{
	modem_8960_dev = subsys_register(&modem_8960);
	if (IS_ERR(modem_8960_dev))
		return PTR_ERR(modem_8960_dev);
	return 0;
}

static int modem_debug_set(void *data, u64 val)
{
	if (val == 1)
		subsystem_restart_dev(modem_8960_dev);
	else if (val == 2)
		subsystem_restart("riva");
	/* Bright Lee, 20130206 { */
	else if (val == 3) {
		if (vendor0_data){
			vendor0_data->modem_reboot_reason = NONE_MODE;
			subsystem_restart_dev(modem_8960_dev);
		}
	}
	/* } Bright Lee, 20130206 */

	return 0;
}

static int modem_debug_get(void *data, u64 *val)
{
	*val = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(modem_debug_fops, modem_debug_get, modem_debug_set,
				"%llu\n");

static int modem_debugfs_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("modem_debug", 0);

	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("reset_modem", 0644, dent, NULL,
		&modem_debug_fops);
	return 0;
}

/* Terry Cheng, 20120529, Add modem crash count for auto pin  {*/
static int proc_read_modem_restart_count (char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;

	len = snprintf (page, 10, "%d", modem_crash_count);
	return strlen(page);
}
/* } Terry Cheng, 20120529, Add modem crash count for auto pin  */
static int __init modem_8960_init(void)
{
	int ret;
	/* Terry Cheng, 20120529, Add modem crash count for auto pin  {*/
	struct proc_dir_entry *pe;
	/* }Terry Cheng, 20120529, Add modem crash count for auto pin  */

	if (cpu_is_apq8064() || cpu_is_apq8064ab())
		return -ENODEV;

	/* Terry Cheng, 20120531, Trigger reboot from modem subsystem request {*/
	vendor0_data = smem_alloc(SMEM_ID_VENDOR0, sizeof (smem_vendor_id0_amss_data));

	if (!vendor0_data)
		pr_err("Alloc share memory SMEM_ID_VENDOR0 fail\n");
	/* }Terry Cheng, 20120531, Trigger reboot from modem subsystem request */

	ret = smsm_state_cb_register(SMSM_MODEM_STATE, SMSM_RESET,
		smsm_state_cb, 0);

	if (ret < 0)
		pr_err("%s: Unable to register SMSM callback! (%d)\n",
				__func__, ret);

	ret = request_irq(Q6FW_WDOG_EXPIRED_IRQ, modem_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "modem_wdog_fw", NULL);

	if (ret < 0) {
		pr_err("%s: Unable to request q6fw watchdog IRQ. (%d)\n",
				__func__, ret);
		goto out;
	}

	ret = request_irq(Q6SW_WDOG_EXPIRED_IRQ, modem_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "modem_wdog_sw", NULL);

	if (ret < 0) {
		pr_err("%s: Unable to request q6sw watchdog IRQ. (%d)\n",
				__func__, ret);
		disable_irq_nosync(Q6FW_WDOG_EXPIRED_IRQ);
		goto out;
	}

	ret = modem_subsystem_restart_init();

	if (ret < 0) {
		pr_err("%s: Unable to reg with subsystem restart. (%d)\n",
				__func__, ret);
		goto out;
	}

	modemfw_ramdump_dev = create_ramdump_device("modem_fw");

	if (!modemfw_ramdump_dev) {
		pr_err("%s: Unable to create modem fw ramdump device. (%d)\n",
				__func__, -ENOMEM);
		ret = -ENOMEM;
		goto out;
	}

	modemsw_ramdump_dev = create_ramdump_device("modem_sw");

	if (!modemsw_ramdump_dev) {
		pr_err("%s: Unable to create modem sw ramdump device. (%d)\n",
				__func__, -ENOMEM);
		ret = -ENOMEM;
		goto out;
	}

	smem_ramdump_dev = create_ramdump_device("smem-modem");

	if (!smem_ramdump_dev) {
		pr_err("%s: Unable to create smem ramdump device. (%d)\n",
				__func__, -ENOMEM);
		ret = -ENOMEM;
		goto out;
	}

	ret = modem_debugfs_init();

	/* Terry Cheng, 20120529, Add modem crash count for auto pin  {*/
	pe = create_proc_read_entry("modem_restart_count", S_IRUGO, NULL, proc_read_modem_restart_count, NULL);
	if (!pe) {
		printk ("create modem_restart_count proc failed\n");
		return -1;
	}
	/* } Terry Cheng, 20120529, Add modem crash count for auto pin */
	pr_info("%s: modem fatal driver init'ed.\n", __func__);
out:
	return ret;
}

module_init(modem_8960_init);
