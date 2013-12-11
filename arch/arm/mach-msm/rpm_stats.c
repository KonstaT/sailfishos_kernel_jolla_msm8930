/* Copyright (c) 2011, The Linux Foundation. All rights reserved.
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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <asm/uaccess.h>

#include <mach/msm_iomap.h>
#include "rpm_stats.h"

/* Terry Cheng, Dump rpm stats to bufer for pm log {*/
#define RPM_SHARE_DATA_PHY 	0x10B180
/* }Terry Cheng, Dump rpm stats to bufer for pm log */

#ifdef CONFIG_PM_LOG
struct msm_rpmstats_platform_data *msm_rpmstats_pdata;
struct msm_rpmstats_private_data *msm_rpmstats_prvdata;

/* Terry Cheng, 20121130, Fix get wrong rpm sleep time stats {*/
unsigned long rpm_vdd_min_time = 0;
unsigned long rpm_vdd_xo_off_time = 0;
/* }Terry Cheng, 20121130, Fix get wrong rpm sleep time stats */

#endif	//CONFIG_PM_LOG
enum {
	ID_COUNTER,
	ID_ACCUM_TIME_SCLK,
	ID_MAX,
};

static char *msm_rpmstats_id_labels[ID_MAX] = {
	[ID_COUNTER] = "Count",
	[ID_ACCUM_TIME_SCLK] = "Total time(uSec)",
};

#define SCLK_HZ 32768
struct msm_rpmstats_record{
	char		name[32];
	uint32_t	id;
	uint32_t	val;
};

struct msm_rpmstats_private_data{
	void __iomem *reg_base;
	u32 num_records;
	u32 read_idx;
	u32 len;
	char buf[128];
	struct msm_rpmstats_platform_data *platform_data;
};

static inline unsigned long  msm_rpmstats_read_register(void __iomem *regbase,
		int index, int offset)
{
	return  readl_relaxed(regbase + index * 12 + (offset + 1) * 4);
}
static void msm_rpmstats_strcpy(char *dest, char  *src)
{
	union {
		char ch[4];
		unsigned long word;
	} string;
	int index = 0;

	do  {
		int i;
		string.word = readl_relaxed(src + 4 * index);
		for (i = 0; i < 4; i++) {
			*dest++ = string.ch[i];
			if (!string.ch[i])
				break;
		}
		index++;
	} while (*(dest-1));

}
static int msm_rpmstats_copy_stats(struct msm_rpmstats_private_data *pdata)
{

	struct msm_rpmstats_record record;
	unsigned long ptr;
	unsigned long offset;
	char *str;
	uint64_t usec;

	ptr = msm_rpmstats_read_register(pdata->reg_base, pdata->read_idx, 0);
	offset = (ptr - (unsigned long)pdata->platform_data->phys_addr_base);

	if (offset > pdata->platform_data->phys_size)
		str = (char *)ioremap(ptr, SZ_256);
	else
		str = (char *) pdata->reg_base + offset;

	msm_rpmstats_strcpy(record.name, str);

	if (offset > pdata->platform_data->phys_size)
		iounmap(str);

	record.id = msm_rpmstats_read_register(pdata->reg_base,
						pdata->read_idx, 1);
	record.val = msm_rpmstats_read_register(pdata->reg_base,
						pdata->read_idx, 2);

	if (record.id == ID_ACCUM_TIME_SCLK) {
		//Terry Cheng , 20121105, Fix save wrong value issue
		usec = (uint64_t)record.val * (uint64_t)USEC_PER_SEC;		
		do_div(usec, SCLK_HZ);
		pr_info("usec = %llu, record.val = %u\n", usec, record.val);
		/* Terry Cheng, 20121130, Fix get wrong rpm sleep time stats {*/
		if(!strncmp(record.name, "xo shutdown", 11))
			rpm_vdd_xo_off_time = record.val;
		else if (!strncmp(record.name, "vdd min", 7))
			rpm_vdd_min_time = record.val;
		/* } Terry Cheng, 20121130, Fix get wrong rpm sleep time stats */
	}  else
		usec = (unsigned long)record.val;

	pdata->read_idx++;

	return snprintf(pdata->buf, sizeof(pdata->buf),
			"RPM Mode:%s\n\t%s:%llu\n",
			record.name,
			msm_rpmstats_id_labels[record.id],
			usec);
}

/* Terry Cheng, Dump rpm stats to bufer for pm log {*/
//RPM log Structucture 
/*
4 bytes: 			Magic number
4 bytes: 			size
4 bytes: 			read flag
(size -7) bytes: 	data 
4 bytes:			Magic number

Retun: number of bytes need to save, including magic number
*/
#ifdef CONFIG_PM_LOG
int msm_rpmstats_dump(char *buffer, int bufer_size)
{

	void __iomem *rpm_pm_log_reg_base;
	uint32_t value;
	uint32_t size;
	static int count = 0;

	if(!buffer)
		return 0;
	if(!bufer_size)
		return 0;

	//First map magic number and size
	rpm_pm_log_reg_base =  ioremap(RPM_SHARE_DATA_PHY, 2*sizeof(uint32_t));	
	if(!rpm_pm_log_reg_base)
	{
		pr_err("ioremap fail\n");
		return 0;
	}	
	value = readl_relaxed(rpm_pm_log_reg_base);
	if (value != 0x12345678)
	{
		pr_err("Wrong magic number");
		return 0;
	}	
	size = readl_relaxed(rpm_pm_log_reg_base+4);
	iounmap(rpm_pm_log_reg_base);
	rpm_pm_log_reg_base = NULL;
	if(size > bufer_size){	
		pr_err("buffer size is too small, bufer_size = %d\n", bufer_size);
		return 0;
	}	
	mb();
	//Remap again
	rpm_pm_log_reg_base =  ioremap(RPM_SHARE_DATA_PHY, size+4);	//one more 4 byte, magic number

	/* Terry Cheng, 20121105,  Notify RPM update the stats { */
	//Write read flag to notify RPM update the stats
	writel_relaxed(0x1, rpm_pm_log_reg_base+8);
	while (readl_relaxed(rpm_pm_log_reg_base+8) == 2)
	{
		msleep(10);
		if(++count > 10){
			pr_err("read flasg = %d, count = %d\n", readl_relaxed(rpm_pm_log_reg_base+8), count);
			count = 0;
			break;
		}	
	}
	/* } Terry Cheng, 20121105,  Notify RPM update the stats  */
	if(rpm_pm_log_reg_base)
	{
		int i = 0;
		int len = size+4;	//one more 4 byte, magic number
		int index = 0;
		uint32_t data = 0;

		for (i = 0;  i < len; )
		{
			data = readl_relaxed(rpm_pm_log_reg_base + i);	//Needn't read read flag
			memcpy(buffer+index, &data, sizeof(data));
			index+=sizeof(data);
			i+=4;
		}
		iounmap(rpm_pm_log_reg_base);
		return len;
	}
	else
	{	
		printk("rpm_pm_log_reg_base io remap fail\n");
		return 0;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(msm_rpmstats_dump);
/* } Terry Cheng, Dump rpm stats to bufer for pm log */
/* Terry Cheng, 20120601, Dump rpm sleep mode stats to bufer for pm log {*/
int msm_rpm_sleep_stats_dump(char *buffer, int bufer_size)
{

	int size = 0;
	
	if(!buffer){
		pr_err("%s buffer is NULL!!", __FUNCTION__);
		goto out;
	}
	msm_rpmstats_prvdata = kmalloc(sizeof(struct msm_rpmstats_private_data), GFP_KERNEL);
	memset(msm_rpmstats_prvdata, 0, sizeof(struct msm_rpmstats_private_data) );
	if(!msm_rpmstats_prvdata)
	{
		pr_err("%s could not alloc memory for msm_rpmstats_prvdata\n", __FUNCTION__);
		goto out;
	}	
	msm_rpmstats_prvdata->reg_base = ioremap(msm_rpmstats_pdata->phys_addr_base, msm_rpmstats_pdata->phys_size);
	if (!msm_rpmstats_prvdata->reg_base) {
		pr_err("%s could not ioremap\n", __FUNCTION__);
		goto out;
	}
	msm_rpmstats_prvdata->read_idx = msm_rpmstats_prvdata->num_records =  msm_rpmstats_prvdata->len = 0;
	msm_rpmstats_prvdata->platform_data = msm_rpmstats_pdata;
	
	if (!msm_rpmstats_prvdata->num_records)
	{	
		msm_rpmstats_prvdata->num_records = readl_relaxed(msm_rpmstats_prvdata->reg_base);
	}
	while ( msm_rpmstats_prvdata->read_idx < msm_rpmstats_prvdata->num_records) {
		msm_rpmstats_prvdata->len = msm_rpmstats_copy_stats(msm_rpmstats_prvdata);
		if( (msm_rpmstats_prvdata->len+size) < bufer_size){
			memcpy(buffer+size, msm_rpmstats_prvdata->buf, msm_rpmstats_prvdata->len);
			size += msm_rpmstats_prvdata->len;
		}else{
			pr_err("Log size is too larger\n");
		}	
	}
out:
	if(msm_rpmstats_prvdata->reg_base)
		iounmap(msm_rpmstats_prvdata->reg_base);
	kfree(msm_rpmstats_prvdata);
	msm_rpmstats_prvdata = NULL;
	return size;
}
EXPORT_SYMBOL_GPL(msm_rpm_sleep_stats_dump);
#endif	//CONFIG_PM_LOG
/* } Terry Cheng, 20120601, Dump rpm sleep mode stats to bufer for pm log */
static int msm_rpmstats_file_read(struct file *file, char __user *bufu,
				  size_t count, loff_t *ppos)
{
	struct msm_rpmstats_private_data *prvdata;
	prvdata = file->private_data;

	if (!prvdata)
		return -EINVAL;

	if (!bufu || count < 0)
		return -EINVAL;

	if (!prvdata->num_records)
		prvdata->num_records = readl_relaxed(prvdata->reg_base);

	if ((*ppos >= prvdata->len)
			&& (prvdata->read_idx < prvdata->num_records)) {
		prvdata->len = msm_rpmstats_copy_stats(prvdata);
		*ppos = 0;
	}

	return simple_read_from_buffer(bufu, count, ppos,
			prvdata->buf, prvdata->len);
}

static int msm_rpmstats_file_open(struct inode *inode, struct file *file)
{
	struct msm_rpmstats_private_data *prvdata;
	struct msm_rpmstats_platform_data *pdata;

	pdata = inode->i_private;

	file->private_data =
		kmalloc(sizeof(struct msm_rpmstats_private_data), GFP_KERNEL);

	if (!file->private_data)
		return -ENOMEM;
	prvdata = file->private_data;

	prvdata->reg_base = ioremap(pdata->phys_addr_base, pdata->phys_size);
	if (!prvdata->reg_base) {
		kfree(file->private_data);
		prvdata = NULL;
		pr_err("%s: ERROR could not ioremap start=%p, len=%u\n",
			__func__, (void *)pdata->phys_addr_base,
			pdata->phys_size);
		return -EBUSY;
	}

	prvdata->read_idx = prvdata->num_records =  prvdata->len = 0;
	prvdata->platform_data = pdata;
	return 0;
}

static int msm_rpmstats_file_close(struct inode *inode, struct file *file)
{
	struct msm_rpmstats_private_data *private = file->private_data;

	if (private->reg_base)
		iounmap(private->reg_base);
	kfree(file->private_data);

	return 0;
}

static const struct file_operations msm_rpmstats_fops = {
	.owner	  = THIS_MODULE,
	.open	  = msm_rpmstats_file_open,
	.read	  = msm_rpmstats_file_read,
	.release  = msm_rpmstats_file_close,
	.llseek   = no_llseek,
};

static  int __devinit msm_rpmstats_probe(struct platform_device *pdev)
{
	struct dentry *dent;
	struct msm_rpmstats_platform_data *pdata;

/* Terry Cheng, 20120601, Dump rpm sleep mode stats to bufer for pm log {*/
#ifdef CONFIG_PM_LOG
	//Save msm rpm stats platform data
	msm_rpmstats_pdata =pdev->dev.platform_data;
	if (!msm_rpmstats_pdata)
		return -EINVAL;
#endif	//CONFIG_PM_LOG
/* } Terry Cheng, 20120601, Dump rpm sleep mode stats to bufer for pm log */

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -EINVAL;
	dent = debugfs_create_file("rpm_stats", S_IRUGO, NULL,
			pdev->dev.platform_data, &msm_rpmstats_fops);

	if (!dent) {
		pr_err("%s: ERROR debugfs_create_file failed\n", __func__);
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, dent);
	return 0;
}

static int __devexit msm_rpmstats_remove(struct platform_device *pdev)
{
	struct dentry *dent;

	dent = platform_get_drvdata(pdev);
	debugfs_remove(dent);
	platform_set_drvdata(pdev, NULL);
	return 0;
}
static struct platform_driver msm_rpmstats_driver = {
	.probe	= msm_rpmstats_probe,
	.remove = __devexit_p(msm_rpmstats_remove),
	.driver = {
		.name = "msm_rpm_stat",
		.owner = THIS_MODULE,
	},
};
static int __init msm_rpmstats_init(void)
{
	return platform_driver_register(&msm_rpmstats_driver);
}
static void __exit msm_rpmstats_exit(void)
{
	platform_driver_unregister(&msm_rpmstats_driver);
}
module_init(msm_rpmstats_init);
module_exit(msm_rpmstats_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM RPM Statistics driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_stat_log");
