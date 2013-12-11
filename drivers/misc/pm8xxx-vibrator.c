/* Copyright (c) 2010-2011, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "../../arch/arm/mach-msm/pmic.h"
#include <mach/pmic.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/vibrator.h>

#include "../staging/android/timed_output.h"

/*add for and debug log */
#include <linux/debugfs.h>
extern struct dentry *kernel_debuglevel_dir;
static unsigned int VIBRATOR_DLL=0;
#define VIB_DEBUG_LEVEL 0
#define VIB_INFO_LEVEL  1
#define VIBRATOR_PRINTK(level, fmt, args...) if(level <= VIBRATOR_DLL) printk(fmt, ##args);
/* Emily Jiang, 20120201 */

/*add for PM_log*/
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif
/*Carl Chang,20120528*/

#define VIB_DRV			0x4A

#define VIB_DRV_SEL_MASK	0xf8
#define VIB_DRV_SEL_SHIFT	0x03
#define VIB_DRV_EN_MANUAL_MASK	0xfc
#define VIB_DRV_LOGIC_SHIFT	0x2

#define VIB_MAX_LEVEL_mV	3100
#define VIB_MIN_LEVEL_mV	1200

struct pm8xxx_vib {
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	//spinlock_t lock;
	struct mutex lock;
	struct work_struct work;
	struct device *dev;
	const struct pm8xxx_vibrator_platform_data *pdata;
	int state;
	int level;
	u8  reg_vib_drv;
	int volt;
/*add for PM_log*/
#ifdef CONFIG_PM_LOG
	struct pmlog_device *pmlog_device;
#endif
/*Carl Chang,20120528*/
};

static struct pm8xxx_vib *vib_dev;

int pm8xxx_vibrator_config(struct pm8xxx_vib_config *vib_config)
{
	u8 reg = 0;
	int rc;

	if (vib_dev == NULL) {
		pr_err("%s: vib_dev is NULL\n", __func__);
		return -EINVAL;
	}

	if (vib_config->drive_mV) {
		if ((vib_config->drive_mV < VIB_MIN_LEVEL_mV) ||
			(vib_config->drive_mV > VIB_MAX_LEVEL_mV)) {
			pr_err("Invalid vibrator drive strength\n");
			return -EINVAL;
		}
	}

	reg = (vib_config->drive_mV / 100) << VIB_DRV_SEL_SHIFT;

	reg |= (!!vib_config->active_low) << VIB_DRV_LOGIC_SHIFT;

	reg |= vib_config->enable_mode;

	rc = pm8xxx_writeb(vib_dev->dev->parent, VIB_DRV, reg);
	if (rc)
		pr_err("%s: pm8xxx write failed: rc=%d\n", __func__, rc);

	return rc;
}
EXPORT_SYMBOL(pm8xxx_vibrator_config);

/* REVISIT: just for debugging, will be removed in final working version */
static void __dump_vib_regs(struct pm8xxx_vib *vib, char *msg)
{
	u8 temp;

	dev_dbg(vib->dev, "%s\n", msg);

	pm8xxx_readb(vib->dev->parent, VIB_DRV, &temp);
	dev_dbg(vib->dev, "VIB_DRV - %X\n", temp);
}

static int pm8xxx_vib_read_u8(struct pm8xxx_vib *vib,
				 u8 *data, u16 reg)
{
	int rc;

	rc = pm8xxx_readb(vib->dev->parent, reg, data);
	if (rc < 0)
		dev_warn(vib->dev, "Error reading pm8xxx: %X - ret %X\n",
				reg, rc);

	return rc;
}

static int pm8xxx_vib_write_u8(struct pm8xxx_vib *vib,
				 u8 data, u16 reg)
{
	int rc;

	rc = pm8xxx_writeb(vib->dev->parent, reg, data);
	if (rc < 0)
		dev_warn(vib->dev, "Error writing pm8xxx: %X - ret %X\n",
				reg, rc);
	return rc;
}

static int pm8xxx_vib_set(struct pm8xxx_vib *vib, int on)
{
	int rc;
	u8 val;

	if (on) {
		val = vib->reg_vib_drv;
		val |= ((vib->level << VIB_DRV_SEL_SHIFT) & VIB_DRV_SEL_MASK);
		rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
		if (rc < 0)
			return rc;
		vib->reg_vib_drv = val;
/*add for PM_log*/
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_on(vib->pmlog_device);
		if (rc)
			printk("pmlog_device_on fall rc = %d\n",rc);
#endif
/*Carl Chang,20120528*/
	} else {
		val = vib->reg_vib_drv;
		val &= ~VIB_DRV_SEL_MASK;
		rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
		if (rc < 0)
			return rc;
		vib->reg_vib_drv = val;
/*add for PM_log*/
#ifdef CONFIG_PM_LOG
                rc = pmlog_device_off(vib->pmlog_device);
                if (rc)
                        printk("pmlog_device_off fall rc = %d\n",rc);
#endif
/*Carl Chang,20120528*/
	}
	__dump_vib_regs(vib, "vib_set_end");

	return rc;
}

static void pm8xxx_vib_enable(struct timed_output_dev *dev, int value)
{
	struct pm8xxx_vib *vib = container_of(dev, struct pm8xxx_vib,
					 timed_dev);
	//unsigned long flags;

retry:
	//spin_lock_irqsave(&vib->lock, flags);
	mutex_lock(&vib->lock);
	if (hrtimer_try_to_cancel(&vib->vib_timer) < 0) {
		//spin_unlock_irqrestore(&vib->lock, flags);
		mutex_unlock(&vib->lock);
		cpu_relax();
		goto retry;
	}

	if (value == 0)
		vib->state = 0;
	else {
		value = (value > vib->pdata->max_timeout_ms ?
				 vib->pdata->max_timeout_ms : value);
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	VIBRATOR_PRINTK(1, "%s: value(%d)\n", __func__, value);
	//spin_unlock_irqrestore(&vib->lock, flags);
	mutex_unlock(&vib->lock);
	schedule_work(&vib->work);
}

static void pm8xxx_vib_update(struct work_struct *work)
{
	struct pm8xxx_vib *vib = container_of(work, struct pm8xxx_vib,
					 work);

	VIBRATOR_PRINTK(1, "%s ++\n", __func__);
	pm8xxx_vib_set(vib, vib->state);
	VIBRATOR_PRINTK(1, "%s --\n", __func__);
}

static int pm8xxx_vib_get_time(struct timed_output_dev *dev)
{
	struct pm8xxx_vib *vib = container_of(dev, struct pm8xxx_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		VIBRATOR_PRINTK(1, "%s: time=%d \n", __func__, (int)ktime_to_us(r));
		return (int)ktime_to_us(r);
	} else {
		VIBRATOR_PRINTK(1, "%s: no timer device atcived--\n", __func__);
		return 0;
}
}

static enum hrtimer_restart pm8xxx_vib_timer_func(struct hrtimer *timer)
{
	struct pm8xxx_vib *vib = container_of(timer, struct pm8xxx_vib,
							 vib_timer);

	vib->state = 0;
	schedule_work(&vib->work);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_PM
static int pm8xxx_vib_suspend(struct device *dev)
{
	struct pm8xxx_vib *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	/* turn-off vibrator */
	pm8xxx_vib_set(vib, 0);
	VIBRATOR_PRINTK(1, "turn-off vibrator after suspend\n");
	return 0;
}

static const struct dev_pm_ops pm8xxx_vib_pm_ops = {
	.suspend = pm8xxx_vib_suspend,
};
#endif

#ifdef CONFIG_DEBUG_FS
#define VIBRATOR_DRIVER_NAME	"pm8xxx-vibrator-dbg"
static struct dentry *dent;

static int dbg_vibrator_time_get(void *data, u64 *value)
{
	struct pm8xxx_vib *vib = (struct pm8xxx_vib *)data;
	int time = 0;

	time = pm8xxx_vib_get_time(&vib->timed_dev);

	VIBRATOR_PRINTK(1, "%s: remaining vibrator time is %d\n",
		__func__, time);

	*value = time;

	return 0;
}

static int dbg_vibrator_time_set(void *data, u64 value)
{
	struct pm8xxx_vib *vib = (struct pm8xxx_vib *)data;
	int time = 0;

	if (value > vib->pdata->max_timeout_ms) {
		time = vib->pdata->max_timeout_ms;
		VIBRATOR_PRINTK(1, "%s: The set value(%lld) is too large, set as 15000\n",
			__func__, value);
	} else {
		time = value;
		VIBRATOR_PRINTK(1, "%s: set time(%d) to vibrator\n",
					__func__, time);
	}
		
	pm8xxx_vib_enable(&vib->timed_dev, time);
		
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vibrator_fops, dbg_vibrator_time_get, dbg_vibrator_time_set, "%llu\n");

/* HW to test vibrator continuous after suspend */
static int dbg_vibrator_onoff_get(void *data, u64 *value)
{
	struct pm8xxx_vib *vib = (struct pm8xxx_vib *)data;

	VIBRATOR_PRINTK(1, "%s: vibrator status is %s\n",
		__func__, vib->state ? "On":"Off");

	*value = vib->state;

	return 0;
}

static int dbg_vibrator_onoff_set(void *data, u64 value)
{
	struct pm8xxx_vib *vib = (struct pm8xxx_vib *)data;

	if (value == 0) {
		mutex_lock(&vib->lock);
		vib->state = 0;
		mutex_unlock(&vib->lock);
	} else if (value == 1) {
		mutex_lock(&vib->lock);
		vib->state = 1;
		mutex_unlock(&vib->lock);
	} else {
			VIBRATOR_PRINTK(1, "%s: Error command(%lld)\n",
				__func__, value);
	}
			
	VIBRATOR_PRINTK(1, "%s: turn %s vibrator\n",
		__func__, vib->state ? "on":"off");
			
	pm8xxx_vib_set(vib, vib->state);
	
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vibrator_onoff_fops, dbg_vibrator_onoff_get, dbg_vibrator_onoff_set, "%llu\n");

static void vibrator_create_debugfs_entries(struct pm8xxx_vib *vib)
{
	dent = debugfs_create_dir(VIBRATOR_DRIVER_NAME, NULL);
	if (dent) {
		debugfs_create_file("vib_time", S_IRUGO | S_IWUGO, dent, vib, &vibrator_fops);
		debugfs_create_file("vib_onoff", S_IRUGO | S_IWUGO, dent, vib, &vibrator_onoff_fops);
	}
}

/* ************************************************************************
 * Description: KERNEL_DEBUGLEVEL
 * ************************************************************************ */
static void vibrator_create_kernel_debuglevel_entries(void)
{
	if (kernel_debuglevel_dir != NULL) {
		debugfs_create_u8("vibrator_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u8 *)(&VIBRATOR_DLL));
	} else {
		VIBRATOR_PRINTK(0, "vibrator kernel debuglevel dir falied\n");
	}
}
#else
static void vibrator_create_debugfs_entries(struct msm_vib *vibe)
{
}
static void vibrator_create_kernel_debuglevel_entries(void)
{
}
#endif
/* Emily Jiang, 20120201 */

static int __devinit pm8xxx_vib_probe(struct platform_device *pdev)

{
	const struct pm8xxx_vibrator_platform_data *pdata =
						pdev->dev.platform_data;
	struct pm8xxx_vib *vib;
	u8 val;
	int rc;
	printk("[vibrator] %s ,probe+++++ \n",__func__ );/*Carl Chang*/

	if (!pdata)
		return -EINVAL;

	if (pdata->level_mV < VIB_MIN_LEVEL_mV ||
			 pdata->level_mV > VIB_MAX_LEVEL_mV)
		return -EINVAL;

	vib = kzalloc(sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->pdata	= pdata;
	vib->level	= pdata->level_mV / 100;
	vib->dev	= &pdev->dev;

	//spin_lock_init(&vib->lock);
	mutex_init(&vib->lock);
	/*Register PM_log*/
#ifdef CONFIG_PM_LOG
	vib->pmlog_device = pmlog_register_device(&pdev->dev);
#endif
	/*Carl Chang,20120528*/
	INIT_WORK(&vib->work, pm8xxx_vib_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = pm8xxx_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = pm8xxx_vib_get_time;
	vib->timed_dev.enable = pm8xxx_vib_enable;

	__dump_vib_regs(vib, "boot_vib_default");

	/*
	 * Configure the vibrator, it operates in manual mode
	 * for timed_output framework.
	 */
	rc = pm8xxx_vib_read_u8(vib, &val, VIB_DRV);
	if (rc < 0)
		goto err_read_vib;
	val &= ~VIB_DRV_EN_MANUAL_MASK;
	rc = pm8xxx_vib_write_u8(vib, val, VIB_DRV);
	if (rc < 0)
		goto err_read_vib;

	vib->reg_vib_drv = val;

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		goto err_read_vib;

	//pm8xxx_vib_enable(&vib->timed_dev, pdata->initial_vibrate_ms);
	// Carl Chang, 20120829, remove initial vibrate

	platform_set_drvdata(pdev, vib);

	vib_dev = vib;

	/* Add for debugfs and fvs mode test */
	vibrator_create_debugfs_entries(vib_dev);
	vibrator_create_kernel_debuglevel_entries();
	/* Emily Jiang, 20120201 */
	printk("[vibrator] %s ,probe----- \n",__func__ );/*Carl Chang*/
	
	return 0;

err_read_vib:
	kfree(vib);
	return rc;
}

static int __devexit pm8xxx_vib_remove(struct platform_device *pdev)
{
	struct pm8xxx_vib *vib = platform_get_drvdata(pdev);

	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);
	platform_set_drvdata(pdev, NULL);
	kfree(vib);

	return 0;
}

static struct platform_driver pm8xxx_vib_driver = {
	.probe		= pm8xxx_vib_probe,
	.remove		= __devexit_p(pm8xxx_vib_remove),
	.driver		= {
		.name	= PM8XXX_VIBRATOR_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm8xxx_vib_pm_ops,
#endif
	},
};

static int __init pm8xxx_vib_init(void)
{
	int ret = 0;
	printk("BootLog, +%s\n", __func__);
	ret = platform_driver_register(&pm8xxx_vib_driver);
	printk("BootLog, -%s, ret=%d\n", __func__,ret);
        return ret;
}
module_init(pm8xxx_vib_init);

static void __exit pm8xxx_vib_exit(void)
{
	platform_driver_unregister(&pm8xxx_vib_driver);
	/*UnRegister PM_log*/
#ifdef CONFIG_PM_LOG
	pmlog_unregister_device(vib_dev->pmlog_device);
#endif
	/*Carl Chang,20120528*/
}
module_exit(pm8xxx_vib_exit);

MODULE_ALIAS("platform:" PM8XXX_VIBRATOR_DEV_NAME);
MODULE_DESCRIPTION("pm8xxx vibrator driver");
MODULE_LICENSE("GPL v2");
