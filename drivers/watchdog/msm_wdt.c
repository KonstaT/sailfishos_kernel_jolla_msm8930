/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
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
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>
#include <mach/socinfo.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <../msm_watchdog.h>  // located in arch/arm/mach-msm/msm_watchdog.h


#define MODULE_NAME "msm_watchdog"

#define WDT_RST		0x0
#define WDT_EN		0x8
#define WDT_STS		0xC
#define WDT_BARK_TIME	0x14
#define WDT_BITE_TIME	0x24

#define WDT_HZ		32768
#define WDT_MAXTIMEOUT  0x3FFF

static void __iomem *msm_wdt_base;
static DEFINE_SPINLOCK(wdt_lock);

static int enable = 0;

static void msm_wdt_enable(void)
{
	pr_debug ("%s %d\n", __func__, __LINE__);
	spin_lock(&wdt_lock);
	__raw_writel(1, msm_wdt_base + WDT_EN);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	mb();
	enable = 1;
	spin_unlock(&wdt_lock);
}

static void msm_wdt_disable(void)
{
	pr_debug ("%s %d\n", __func__, __LINE__);
	spin_lock(&wdt_lock);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	__raw_writel(0, msm_wdt_base + WDT_EN);
	mb();
	enable = 0;
	spin_unlock(&wdt_lock);
}

static int msm_wdt_settimeout(struct watchdog_device *wdd, unsigned int time)
{
	unsigned long timeout = (time * WDT_HZ);
	pr_debug ("%s %d %d\n", __func__, __LINE__, time);
	spin_lock(&wdt_lock);
	__raw_writel(timeout, msm_wdt_base + WDT_BARK_TIME);
	__raw_writel(timeout + 3*WDT_HZ, msm_wdt_base + WDT_BITE_TIME);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	spin_unlock(&wdt_lock);
	wdd->timeout = time;
	return 0;
}

static void msm_wdt_keepalive(void)
{
	pr_debug ("%s %d\n", __func__, __LINE__);
	spin_lock(&wdt_lock);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	spin_unlock(&wdt_lock);
}

void pet_watchdog(void)
{
	pr_debug ("%s %d\n", __func__, __LINE__);
	msm_wdt_keepalive();
}

static int msm_wdt_start (struct watchdog_device *wdd)
{
	pr_debug ("%s %d\n", __func__, __LINE__);
	msm_wdt_enable();
	return 0;
}

static int msm_wdt_stop (struct watchdog_device *wdd)
{
	pr_debug ("%s %d\n", __func__, __LINE__);
	msm_wdt_disable();

	return 0;
}

static int msm_wdt_ping (struct watchdog_device *wdd)
{
	pr_debug ("%s %d\n", __func__, __LINE__);
	msm_wdt_keepalive();
	return 0;
}

static int msm_wdt_suspend(struct device *dev)
{
	if (!enable)
		return 0;

	__raw_writel(1, msm_wdt_base + WDT_RST);
	__raw_writel(0, msm_wdt_base + WDT_EN);
	mb();
	return 0;
}

static int msm_wdt_resume(struct device *dev)
{
	if (!enable)
		return 0;

	__raw_writel(1, msm_wdt_base + WDT_EN);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	mb();
	return 0;
}

static const struct watchdog_info msm_wdt_info = {
	.identity	= "Qualcomm MSM Watchdog",
	.options 	= WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING,
};

static const struct watchdog_ops  msm_wdt_fops = {
	.owner		= THIS_MODULE,
	.start		= msm_wdt_start,
	.stop 		= msm_wdt_stop,
	.ping		= msm_wdt_ping,
	.set_timeout	= msm_wdt_settimeout,
};

static struct watchdog_device msm_wdt_dev = {
	.info = 	&msm_wdt_info,
	.ops = 		&msm_wdt_fops,
	.min_timeout = 	1,
	.max_timeout =	WDT_MAXTIMEOUT,
};

static int msm_wdt_probe(struct platform_device *pdev)
{
	struct msm_watchdog_pdata *pdata = pdev->dev.platform_data;
	pr_debug ("%s %d\n", __func__, __LINE__);

	if (!pdata) {
		pr_debug(KERN_INFO "MSM Watchdog Not Initialized\n");
		return -ENODEV;
	}

	msm_wdt_base = pdata->base;

	if (pdata->needs_expired_enable)
		__raw_writel(0x1, MSM_CLK_CTL_BASE + 0x3820);
		
        __raw_writel(10 * WDT_HZ, msm_wdt_base + WDT_BARK_TIME); // set default timeout to prevent 
	__raw_writel(13 * WDT_HZ, msm_wdt_base + WDT_BITE_TIME); // immediately dog bark once dog enable

	watchdog_register_device (&msm_wdt_dev);

	return 0;
}

static void msm_wdt_shutdown(struct platform_device *dev)
{
        msm_wdt_disable();
}

static int __devexit msm_wdt_remove(struct platform_device *pdev)
{
	watchdog_unregister_device(&msm_wdt_dev);
	return 0;
}

static const struct dev_pm_ops msm_wdt_dev_pm_ops = {
	.suspend_noirq = msm_wdt_suspend,
	.resume_noirq = msm_wdt_resume,
};

static struct platform_driver msm_wdt_driver = {
	.probe = msm_wdt_probe,
	.remove = msm_wdt_remove,
	.shutdown = msm_wdt_shutdown,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &msm_wdt_dev_pm_ops,
	},
};

static int __init msm_wdt_init(void)
{
	pr_debug ("%s %d\n", __func__, __LINE__);
	return platform_driver_register(&msm_wdt_driver);
}

static void __exit msm_wdt_exit(void)
{
	pr_debug ("%s %d\n", __func__, __LINE__);
	platform_driver_unregister(&msm_wdt_driver);
}


module_init(msm_wdt_init);
module_exit(msm_wdt_exit);

MODULE_LICENSE("GPL");

