/*
 * Driver for the PN544 NFC chip.
 *
 * Copyright (C) Nokia Corporation
 *
 * Author: Jari Vanhala <ext-jari.vanhala@nokia.com>
 * Contact: Matti Aaltonen <matti.j.aaltonen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/completion.h>
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nfc/pn544.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/serial_core.h> /* for TCGETS */
#include <linux/slab.h>
/* Jen Chang add for debugfs */
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <asm/atomic.h> //add for FTD to get/set it_enable status
#include <linux/seq_file.h> //add for FTD to get signed curr value

//add for pm log
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif
/* Jen Chang, 20121217 */

#define DRIVER_CARD	"PN544 NFC"
#define DRIVER_DESC	"NFC driver for PN544"

 /* Jen Chang add for debug log */
extern struct dentry *kernel_debuglevel_dir;
union PN544_DEBUGLEVEL PN544_DEBUG_DLL;
/* Jen Chang, 20121217 */

static struct i2c_device_id pn544_id_table[] = {
	{ PN544_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pn544_id_table);

#define HCI_MODE	0
#define FW_MODE		1

enum pn544_state {
	PN544_ST_COLD,
	PN544_ST_FW_READY,
	PN544_ST_READY,
};

enum pn544_irq {
	PN544_NONE,
	PN544_INT,
};

struct pn544_info {
	struct miscdevice miscdev;
	struct i2c_client *i2c_dev;
/* Jen Chang mark sim power because it is controlled by sim driver */
//	struct regulator_bulk_data regs[3];
	struct regulator_bulk_data regs[1];
/* Jen Chang, 20130206 */
	enum pn544_state state;
	wait_queue_head_t read_wait;
	loff_t read_offset;
	enum pn544_irq read_irq;
	struct mutex read_mutex; /* Serialize read_irq access */
	struct mutex mutex; /* Serialize info struct access */
	spinlock_t irq_enabled_lock;
/* Jen Chang add for wake up cpu by reading nfc tag card */
	struct wake_lock wake_lock;
/* Jen Chang, 20121217 */
	u8 *buf;
	size_t buflen;
	bool irq_enabled;
	int irq_gpio;
#ifdef CONFIG_PM_LOG
	 struct pmlog_device *pmlog_device;
#endif //CONFIG_PM_LOG
};

/* Jen Chang modify for requesting requlator */
static const char reg_vdd_io[]	= "NFC_VEN"; //Vdd_IO
static const char reg_vbat[]	= "NFC_VBAT"; //VBat
static const char reg_vsim[]	= "NFC_PMVCC"; //VSim

//add for FTD to get i2c response id
static u8 dev_i2c_response = 0;
//add for script test
static atomic_t antenna_sleftest_status = ATOMIC_INIT(0);
static u8 force_antenna_test = 0;

/* Jen Chang mark sim power because it is controlled by sim driver */
static struct regulator_bulk_data nfc_regs[] = {
	{ .supply = reg_vdd_io,	.min_uV = 1800000,	.max_uV = 1800000 },
//	{ .supply = reg_vsim, 	.min_uV = 1800000,	.max_uV = 2950000 },
};
/* Jen Chang, 20130206 */

//add for debugfs with i2c test
static char test_addr_result = 0;
static ssize_t pn544_i2c_test_set(struct device *dev,
			  struct device_attribute *attr,  const char *buf, size_t count)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_dev;
	char test_addr = PN544_I2C_ADDR;
	int ret = 0;

	ret = i2c_master_send(client, &test_addr, 1);
	pn544_printk(NFC_DEBUGFS, "--%s: send(0x%x) to pn544, ret(%d)--\n", __func__, test_addr, ret);

	ret = i2c_master_recv(client, &test_addr_result, 1);
	pn544_printk(NFC_DEBUGFS, "--%s: recv(0x%x) from pn544, ret(%d)--\n", __func__, test_addr_result, ret);

	return count;
}

static ssize_t pn544_i2c_test_get(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
//	struct pn544_info *info = dev_get_drvdata(dev);
//	struct i2c_client *client = info->i2c_dev;
//	struct pn544_nfc_platform_data *pdata = client->dev.platform_data;

/* Jen Chang modify for check nfc chip is ok or not */
	return snprintf(buf, PAGE_SIZE, "%d,%d\n", dev_i2c_response, test_addr_result);
/* Jen Chang, 20130221 */
}
/* Jen Chang, 20130114 */

/* sysfs interface */
static ssize_t pn544_test(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_dev;
	struct pn544_nfc_platform_data *pdata = client->dev.platform_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", pdata->test());
}

static int pn544_enable(struct pn544_info *info, int mode)
{
	struct pn544_nfc_platform_data *pdata;
	struct i2c_client *client = info->i2c_dev;

	int r;
#ifdef CONFIG_PM_LOG
	int rc = 0;
#endif	//CONFIG_PM_LOG

	r = regulator_bulk_enable(ARRAY_SIZE(info->regs), info->regs);
	if (r < 0)
		return r;

/* Jen Chang add for pm log */
#ifdef CONFIG_PM_LOG
	rc = pmlog_device_on(info->pmlog_device);
	if(rc)
		 printk("pmlog_device_on fall rc = %d\n",rc);
#endif
/* Jen Chang, 20121217 */

	pdata = client->dev.platform_data;
	info->read_irq = PN544_NONE;
	if (pdata->enable)
		pdata->enable(mode);

	if (mode) {
		info->state = PN544_ST_FW_READY;
		pn544_printk(NFC_INFO, "%s: now in FW-mod\n", __func__);
	} else {
		info->state = PN544_ST_READY;
		pn544_printk(NFC_INFO, "%s: now in HCI-mod\n", __func__);
	}

	usleep_range(10000, 15000);

	return 0;
}

static void pn544_disable(struct pn544_info *info)
{
	struct pn544_nfc_platform_data *pdata;
	struct i2c_client *client = info->i2c_dev;

#ifdef CONFIG_PM_LOG
	int rc = 0;
#endif	//CONFIG_PM_LOG

/* Jen Chang add for pm log */
#ifdef CONFIG_PM_LOG
	rc = pmlog_device_off(info->pmlog_device);
	if(rc)
		 printk("pmlog_device_off fall rc = %d\n",rc);
#endif
/* Jen Chang, 20121217 */

	pdata = client->dev.platform_data;
	if (pdata->disable)
		pdata->disable();

	pn544_printk(NFC_INFO, "%s: now in OFF-mod\n", __func__);

	msleep(PN544_RESETVEN_TIME);

	regulator_bulk_disable(ARRAY_SIZE(info->regs), info->regs);
}

static void pn544_disable_irq(struct pn544_info *pn544_dev)
{
	unsigned long flags = 0;;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->i2c_dev->irq);
		pn544_dev->irq_enabled = false;
	}

	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_info *pn544_dev = dev_id;

	pn544_printk(NFC_DEBUG, "--%s--\n", __func__);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		pn544_printk(NFC_DEBUG, "%s: irq release!\n", __func__);
		return IRQ_HANDLED;
}

	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wait);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
			  size_t count, loff_t *offset)
{

	struct pn544_info *pn544_dev = filp->private_data;
	char tmp[PN544_MAX_BUFFER_SIZE];
	int ret,i;

	if (count > PN544_MAX_BUFFER_SIZE)
		count = PN544_MAX_BUFFER_SIZE;

	pn544_printk(NFC_INFO, "%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
	}
		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->i2c_dev->irq);
		ret = wait_event_interruptible(pn544_dev->read_wait,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);
		if (ret)
			goto fail;
		}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->i2c_dev, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
		}

	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
		}

	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	pn544_printk(NFC_DEBUG, "IFD->PC:");
	for(i = 0; i < ret; i++)
		pn544_printk(NFC_DEBUG, " %02X", tmp[i]);

	pn544_printk(NFC_DEBUG, "\n");

/* Jen Chang add to wait cmd stable according to Edom FAE suggestion */
//	usleep_range(6000, 10000);
/* Jen Chang, 20121217 */

	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
	}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{

	struct pn544_info  *pn544_dev;
	char tmp[PN544_MAX_BUFFER_SIZE];
	int ret,i;

	pn544_dev = filp->private_data;

	if (count > PN544_MAX_BUFFER_SIZE)
		count = PN544_MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
		}

	pn544_printk(NFC_INFO, "%s : writing %zu bytes.\n", __func__, count);

	/* Write data */
	ret = i2c_master_send(pn544_dev->i2c_dev, tmp, count);

	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
		}

	pn544_printk(NFC_DEBUG, "PC->IFD:");

	for(i = 0; i < count; i++)
		pn544_printk(NFC_DEBUG, " %02X", tmp[i]);

	pn544_printk(NFC_DEBUG, "\n");

/* Jen Chang add to wait cmd stable according to Edom FAE suggestion */
//	usleep_range(6000, 10000);
/* Jen Chang, 20121217 */

	return ret;
}

static long pn544_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pn544_info *info = file->private_data;
	struct i2c_client *client = info->i2c_dev;
	struct pn544_nfc_platform_data *pdata;
	int r = 0;

	pn544_printk(NFC_DEBUG, "%s: info: %p, cmd: 0x%x\n", __func__, info, cmd);

	mutex_lock(&info->mutex);
 	pdata = info->i2c_dev->dev.platform_data;

	switch (cmd) {
	case PN544_SET_PWR:
		pdata = client->dev.platform_data;
		pn544_printk(NFC_INFO, "%s: PN544_SET_PWR_MODE\n", __func__);

 		if (arg == 2) {
 			pn544_disable(info);
			pn544_enable(info, FW_MODE);
		} else if (arg == 1) {
			pn544_enable(info, HCI_MODE);
		} else if (arg == 0) {
			pn544_disable(info);
		} else {
			dev_err(&client->dev, "[PN544] the pwr set error...");
			r = -EINVAL;
				goto out;
		}
		break;
 	default:
		dev_err(&client->dev, "[PN544] The IOCTL isn't exist!!\n");

		r = -EINVAL;
		break;
	}
out:
	mutex_unlock(&info->mutex);
	return r;
}

static int pn544_open(struct inode *inode, struct file *file)
{
	struct pn544_info *info = container_of(file->private_data,
					       struct pn544_info, miscdev);
	struct i2c_client *client = info->i2c_dev;
	int r = 0;

	pn544_printk(NFC_INFO, "%s: info: %p, client %p\n", __func__, info, client);

	file->private_data = info;

	mutex_lock(&info->mutex);
	r = pn544_enable(info, HCI_MODE);
	mutex_unlock(&info->mutex);

	return r;
}

static int pn544_close(struct inode *inode, struct file *file)
{
	struct pn544_info *info = container_of(file->private_data,
					       struct pn544_info, miscdev);
	struct i2c_client *client = info->i2c_dev;

	pn544_printk(NFC_INFO, "%s: info: %p, client %p\n", __func__, info, client);

	mutex_lock(&info->mutex);
	pn544_disable(info);
	mutex_unlock(&info->mutex);

	return 0;
}

static const struct file_operations pn544_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= pn544_dev_read,
	.write		= pn544_dev_write,
	.open		= pn544_open,
	.release	= pn544_close,
	.unlocked_ioctl	= pn544_ioctl,
};

#ifdef CONFIG_PM
static int pn544_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pn544_info *info = i2c_get_clientdata(client);
	int r = 0;

	pn544_printk(NFC_INFO, "%s: info: %p, client %p\n", __func__, info, client);
	/* Terry Cheng, 20130306, Check whether has nfc hardware. If no nfc hardware, return OK {*/
	if (!dev_i2c_response)
		return 0;
	/* } Terry Cheng, 20130306, Check whether has nfc hardware. If no nfc hardware, return OK */
	mutex_lock(&info->mutex);

	switch (info->state) {
	case PN544_ST_FW_READY:
		/* Do not suspend while upgrading FW, please! */
		r = -EPERM;
		break;

	case PN544_ST_READY:
		/*
		 * CHECK: Device should be in standby-mode. No way to check?
		 * Allowing low power mode for the regulator is potentially
		 * dangerous if pn544 does not go to suspension.
		 */
	/* Jen Chang mark it to avoid to unsleep */
	// To wake up cpu by reading nfc tag card
	//	enable_irq_wake(info->i2c_dev->irq);
	/* Jen Chang, 20130114 */
		break;

	case PN544_ST_COLD:
		break;
	};

	mutex_unlock(&info->mutex);
	return r;
}

static int pn544_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pn544_info *info = i2c_get_clientdata(client);
	int r = 0;

	pn544_printk(NFC_INFO, "%s: info: %p, client %p\n", __func__, info, client);

	mutex_lock(&info->mutex);

	switch (info->state) {
	case PN544_ST_READY:
		/*
		 * CHECK: If regulator low power mode is allowed in
		 * pn544_suspend, we should go back to normal mode
		 * here.
		 */
	/* Jen Chang mark it to avoid to unsleep */
	// To wake up cpu by reading nfc tag card
	//	disable_irq_wake(info->i2c_dev->irq);
	//	wake_lock_timeout(&info->wake_lock, 5 * HZ);
	/* Jen Chang, 20130114 */
		break;

	case PN544_ST_COLD:
		break;

	case PN544_ST_FW_READY:
		break;
	};

	mutex_unlock(&info->mutex);

	return r;
}

static SIMPLE_DEV_PM_OPS(pn544_pm_ops, pn544_suspend, pn544_resume);
#endif

static struct device_attribute pn544_attr[] = {
	__ATTR(nfc_test, S_IRUGO, pn544_test, NULL),
/* Jen Chang add to do i2c test */
	__ATTR(i2c_test, (S_IWUSR | S_IWGRP | S_IRUGO), pn544_i2c_test_get, pn544_i2c_test_set),
/* Jen Chang, 20121217 */
};

/* Jen Chang modify for FTD and EM mode */
#ifdef CONFIG_DEBUG_FS
static struct dentry *dent;

//add for debugfs with irq gpio test
static int pn544_host_int_gpio_get(void *data, u64 *value)
{
	int gpio;

	gpio = gpio_get_value(PN544_HOST_INT_GPIO);

	pn544_printk(NFC_DEBUGFS, "host_int_gpio is %s\n", gpio? "high":"low");

	*value = gpio;

	return 0;
}

static int pn544_host_int_gpio_set(void *data, u64 value)
{
	if(value == 1)
	{
		gpio_set_value(PN544_HOST_INT_GPIO, 1);
		pn544_printk(NFC_DEBUGFS, "--%s: set host_int_gpio to be high!!--\n", __func__);
	}
	else if(value == 0)
	{
		gpio_set_value(PN544_HOST_INT_GPIO, 0);
		pn544_printk(NFC_DEBUGFS, "--%s: set host_int_gpio to be low!!--\n", __func__);
	}
	else
		pn544_printk(NFC_DEBUGFS, "--%s: Unsupport value!!--\n", __func__);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pn544_host_int_gpio_fops, pn544_host_int_gpio_get, pn544_host_int_gpio_set, "%llu\n");

//add for debugfs with enable gpio test
static int pn544_enable_gpio_get(void *data, u64 *value)
{
	int gpio;

	gpio = gpio_get_value(PN544_ENABLE_GPIO);

	pn544_printk(NFC_DEBUGFS, "enable_gpio is %s\n", gpio? "high":"low");

	*value = gpio;

	return 0;
}

static int pn544_enable_gpio_set(void *data, u64 value)
{
	if(value == 1)
	{
		gpio_set_value(PN544_ENABLE_GPIO, 1);
		pn544_printk(NFC_DEBUGFS, "--%s: set enable_gpio to be high!!--\n", __func__);
	}
	else if(value == 0)
	{
		gpio_set_value(PN544_ENABLE_GPIO, 0);
		pn544_printk(NFC_DEBUGFS, "--%s: set enable_gpio to be low!!--\n", __func__);
	}
	else
		pn544_printk(NFC_DEBUGFS, "--%s: Unsupport value!!--\n", __func__);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pn544_enable_gpio_fops, pn544_enable_gpio_get, pn544_enable_gpio_set, "%llu\n");

//add for debugfs with fw_reset gpio test
static int pn544_fw_reset_get(void *data, u64 *value)
{
	int gpio;

	gpio = gpio_get_value(PN544_FW_RESET_GPIO);

	pn544_printk(NFC_DEBUGFS, "fw_reset_gpio_fops is %s\n", gpio? "high":"low");

	*value = gpio;

	return 0;
}

static int pn544_fw_reset_set(void *data, u64 value)
{
	if(value == 1)
	{
		gpio_set_value(PN544_FW_RESET_GPIO, 1);
		pn544_printk(NFC_DEBUGFS, "--%s: set fw_reset_gpio_fops to be high!!--\n", __func__);
	}
	else if(value == 0)
	{
		gpio_set_value(PN544_FW_RESET_GPIO, 0);
		pn544_printk(NFC_DEBUGFS, "--%s: set fw_reset_gpio_fops to be low!!--\n", __func__);
	}
	else
		pn544_printk(NFC_DEBUGFS, "--%s: Unsupport value!!--\n", __func__);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pn544_fw_reset_gpio_fops, pn544_fw_reset_get, pn544_fw_reset_set, "%llu\n");

/* Jen Chang add for check nfc chip is ok or not */
//add for debugfs with dev_i2c_response
static int pn544_dev_i2c_response_get(void *data, u64 *value)
{
	pn544_printk(NFC_DEBUGFS, "dev_i2c_response is %d\n", dev_i2c_response);

	*value = dev_i2c_response;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pn544_dev_i2c_response_fops, pn544_dev_i2c_response_get, NULL, "%llu\n");
/* Jen Chang, 20130221 */

#ifdef CONFIG_PN544_TEST
//add for reading chip id
static int pn544_read_chip_id_open(struct inode *inode, struct file *file)
{
	pn544_printk(NFC_DEBUG, "--%s--\n", __func__);

	file->private_data = inode->i_private;

	return 0;
}

static int pn544_read_chip_id_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	struct i2c_client *client = info->i2c_dev;
	u16 chip_id = 0;
	int ret = 0, copy_size = 0;
	char buf[64];

	pn544_printk(NFC_DEBUG, "--%s--\n", __func__);

	/* add *ppos check for cat command from shell to avoid to execute this operation twice
		(cat command will issue twice because of checking eof) */
	if (*ppos == 0) {
		pn544_disable(info);
		pn544_enable(info, HCI_MODE);
		PN544_NFC_TEST(r_ver, client, &pn544_test_info);
		pn544_disable (info);

		chip_id = (dev_i2c_response << 8) | pn544_test_info.chip_version.sw_ver.value_byte[2];

		pn544_printk(NFC_DEBUGFS, "--%s: recv chip id[0x%x] from pn544--\n",
			__func__, chip_id);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "%d,", dev_i2c_response);
		ret += snprintf(buf + ret, copy_size - ret, "%d\n", pn544_test_info.chip_version.sw_ver.value_byte[2]);

		pn544_printk(NFC_DEBUGFS, "--%s: ret = %d, buf(%s)--\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static const struct file_operations pn544_read_chip_id_fops = {
	.open		= pn544_read_chip_id_open,
	.read		= pn544_read_chip_id_read,
};

//add for reading FW version
static int pn544_read_sw_version_open(struct inode *inode, struct file *file)
{
	pn544_printk(NFC_DEBUG, "--%s--\n", __func__);

	file->private_data = inode->i_private;

	return 0;
}

static int pn544_read_sw_version_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	struct i2c_client *client = info->i2c_dev;
	int ret = 0, copy_size = 0;
	char buf[64];

	pn544_printk(NFC_DEBUG, "--%s--\n", __func__);

	/* add *ppos check for cat command from shell to avoid to execute this operation twice
		(cat command will issue twice because of checking eof) */
	if (*ppos == 0) {
		pn544_disable(info);
		pn544_enable(info, HCI_MODE);
		PN544_NFC_TEST(r_ver, client, &pn544_test_info);
		pn544_disable (info);

		pn544_printk(NFC_DEBUGFS, "--%s: recv sw version[0x%x] from pn544--\n",
			__func__,  pn544_test_info.chip_version.sw_ver.value);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "chip-%d", pn544_test_info.chip_version.sw_ver.value_byte[1]);
		ret += snprintf(buf + ret, copy_size - ret, "-%d\n", pn544_test_info.chip_version.sw_ver.value_byte[0]);

		pn544_printk(NFC_DEBUGFS, "--%s: ret = %d, buf(%s)--\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static const struct file_operations pn544_read_sw_version_fops = {
	.open		= pn544_read_sw_version_open,
	.read		= pn544_read_sw_version_read,
};

//add for self test with test script via HCI command
//Antenna self-test
static int pn544_antenna_selftset_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	pn544_printk(NFC_TEST, "--%s: start nfc antenna selftest process...--\n", __func__);

	atomic_set(&antenna_sleftest_status, 1);

	return 0;
}

static int pn544_antenna_selftset_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	int ret = 0, copy_size = 0;
	char buf[64];

	pn544_printk(NFC_TEST, "--%s: count = %Zd--\n", __func__, count);

	/* add *ppos check for cat command from shell to avoid to execute this operation twice
		(cat command will issue twice because of checking eof) */
	if (*ppos == 0) {
			if (atomic_read(&antenna_sleftest_status) != 0 && force_antenna_test == 0) {
				pn544_printk(NFC_TEST, "--%s:  Need to write tolerance to do antenna selftest first--\n",
					__func__);
				return -EBUSY;
			}

		#if 0
			if (count >= sizeof(pn544_test_info.antenna_selftest_threshold)) {
				pn544_printk(NFC_TEST, "--%s: Read byte is too big!--\n",
					__func__);
				return -EINVAL;
			}
		#endif

			pn544_disable(info);
			pn544_enable(info, HCI_MODE);
			PN544_NFC_TEST(antenna_self_test, info->i2c_dev, &pn544_test_info);
			pn544_disable (info);
			force_antenna_test = 0;

		pn544_printk(NFC_DEBUGFS, "--%s: test result is [0x%08x]\n",
			__func__, pn544_test_info.antenna_selftest_threshold.value);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "%d,", pn544_test_info.antenna_selftest_threshold.value_byte[0]);
		ret += snprintf(buf + ret, copy_size - ret, "%d,", pn544_test_info.antenna_selftest_threshold.value_byte[1]);
		ret += snprintf(buf + ret, copy_size - ret, "%d,", pn544_test_info.antenna_selftest_threshold.value_byte[2]);
		ret += snprintf(buf + ret, copy_size - ret, "%d\n", pn544_test_info.antenna_selftest_threshold.value_byte[3]);

		pn544_printk(NFC_DEBUGFS, "--%s: ret = %d, buf(%s)--\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static ssize_t pn544_antenna_selftset_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret, i = 0;
	char *buf = (char *) __get_free_page(GFP_USER);
	char *tmp = NULL;

	pn544_printk(NFC_TEST, "--%s--\n", __func__);

	if (!buf) {
		pn544_printk(NFC_DEBUGFS, "--%s: can't get free GFP_USER page--\n", __func__);
		ret = -ENOMEM;
		atomic_set(&antenna_sleftest_status, 0);
		goto out;
	}

	if (count >= PAGE_SIZE) {
		pn544_printk(NFC_DEBUGFS, "--%s: count(%d) is bigger than PAGE_SIZE(%ld)--\n",
			__func__, count, PAGE_SIZE);
		ret = -EINVAL;
		atomic_set(&antenna_sleftest_status, 0);
		goto out;
	}

	ret = copy_from_user(buf, buffer, count);
	if (ret) {
		pn544_printk(NFC_DEBUGFS, "--%s: copy_from_user failed, ret(%d)--\n",
			__func__, ret);
		ret = -EFAULT;
		atomic_set(&antenna_sleftest_status, 0);
		goto out;
	}

	ret = count;
	buf[count] = '\0';

	tmp = buf;
	while (i < 5) {
		if (i < 4) {
			pn544_test_info.antenna_loop_tolerance.value_byte[i] =
				simple_strtol(tmp, NULL, 0);
		} else {

			force_antenna_test = simple_strtol(tmp, NULL, 0);
		}
		tmp = strchr(tmp, ',');
		if (tmp == NULL)
			break;

		tmp++;
		i++;
	}

	pn544_printk(NFC_TEST, "--%s: tolerance is [%08x]--\n",
		__func__, pn544_test_info.antenna_loop_tolerance.value);

	atomic_set(&antenna_sleftest_status, 0);

 out:
	free_page((unsigned long)buf);
	return ret;
}

static const struct file_operations pn544_antenna_sleftest_operations = {
	.open		= pn544_antenna_selftset_open,
	.read		= pn544_antenna_selftset_read,
	.write		= pn544_antenna_selftset_write,
};

//SWP self-test
static int pn544_swp_selftset_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	pn544_printk(NFC_TEST, "--%s: start nfc swp selftest process...--\n", __func__);

	return 0;
}

static int pn544_swp_selftset_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	int ret = 0, copy_size = 0;
	char buf[64];

	pn544_printk(NFC_TEST, "--%s: count = %Zd--\n", __func__, count);

	/* add *ppos check for cat command from shell to avoid to execute this operation twice
		(cat command will issue twice because of checking eof) */
	if (*ppos == 0) {
		pn544_disable(info);
		pn544_enable(info, HCI_MODE);
		PN544_NFC_TEST(swp_self_test, info->i2c_dev, &pn544_test_info);
		pn544_disable (info);

		pn544_printk(NFC_DEBUGFS, "--%s: test result is [0x%08x]\n",
			__func__, pn544_test_info.swp_selftest_result.value);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "%d,", pn544_test_info.swp_selftest_result.value_byte[0]);
		ret += snprintf(buf + ret, copy_size - ret, "%d\n", pn544_test_info.swp_selftest_result.value_byte[1]);

		pn544_printk(NFC_DEBUGFS, "--%s: ret = %d, buf(%s)--\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static const struct file_operations pn544_swp_sleftest_operations = {
	.open		= pn544_swp_selftset_open,
	.read		= pn544_swp_selftset_read,
};

//add for RF certification to switch Tx on/off
static int pn544_switchTx_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	pn544_printk(NFC_TEST, "--%s: start nfc switch Tx on process...--\n", __func__);

	return 0;
}

static int pn544_switchTx_read(struct file *file, __user char *buffer, size_t count,
	loff_t *ppos)
{
	int ret = 0, copy_size = 0;
	char buf[64];

	pn544_printk(NFC_TEST, "--%s: count = %Zd--\n", __func__, count);

	/* add *ppos check for cat command from shell to avoid to execute this operation twice
		(cat command will issue twice because of checking eof) */
	if (*ppos == 0) {
		copy_size = sizeof(buf);
		ret = snprintf(buf, copy_size - ret, "%d\n", pn544_test_info.switch_Tx_onoff);
		pn544_printk(NFC_DEBUGFS, "--%s: ret = %d, buf(%s)--\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static ssize_t pn544_switchTx_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int ret, value;
	char *buf = (char *) __get_free_page(GFP_USER);
	struct pn544_info *info = file->private_data;

	pn544_printk(NFC_TEST, "--%s--\n", __func__);

	if (!buf)
		return -ENOMEM;

	ret = -EINVAL;
	if (count >= PAGE_SIZE)
		goto out;

	ret = -EFAULT;
	if (copy_from_user(buf, buffer, count))
		goto out;

	ret = count;
	buf[count] = '\0';

	value = simple_strtol(buf, NULL, 10) ? 1 : 0;
	if (value == 1) {
		pn544_test_info.switch_Tx_onoff = 1;
		pn544_printk(NFC_TEST, "--%s: switch Tx on...--\n", __func__);
	} else {
		pn544_printk(NFC_TEST, "--%s: switch Tx off...--\n", __func__);
		pn544_test_info.switch_Tx_onoff = 0;
	}

		pn544_disable(info);
		pn544_enable(info, HCI_MODE);
		PN544_NFC_TEST(switch_Tx_only, info->i2c_dev, &pn544_test_info);

 out:
	free_page((unsigned long)buf);
	return ret;
}

static const struct file_operations pn544_switchTx_operations = {
	.open		= pn544_switchTx_open,
	.read		= pn544_switchTx_read,
	.write		= pn544_switchTx_write,
};

//add for hci TypeA reader on/off
static int pn544_hci_TypeA_reader_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	pn544_printk(NFC_TEST, "--%s: start nfc TypeA reader process...--\n", __func__);

	return 0;
}

static int pn544_hci_TypeA_reader_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	int ret = 0, copy_size = 0;
	char buf[64];

	pn544_printk(NFC_TEST, "--%s: count = %Zd--\n", __func__, count);

	/* add *ppos check for cat command from shell to avoid to execute this operation twice
		(cat command will issue twice because of checking eof) */
	if (*ppos == 0) {
		pn544_disable(info);
		pn544_enable(info, HCI_MODE);
		PN544_NFC_TEST(hci_TypeA_reader, info->i2c_dev, &pn544_test_info);
		pn544_disable (info);


		pn544_printk(NFC_DEBUGFS, "--%s: test result: uid_led[%d], uid[0x%08x 0x%08x]\n",
			__func__, pn544_test_info.card_data.uid_len,
			pn544_test_info.card_data.uid[0].value, pn544_test_info.card_data.uid[1].value);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "uid len = %x\n", pn544_test_info.card_data.uid_len);
		ret += snprintf(buf + ret, copy_size - ret, "uid is %x ", pn544_test_info.card_data.uid[0].value_byte[0]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[0].value_byte[1]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[0].value_byte[2]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[0].value_byte[3]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[1].value_byte[0]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[1].value_byte[1]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[1].value_byte[2]);
		ret += snprintf(buf + ret, copy_size - ret, "%x\n", pn544_test_info.card_data.uid[1].value_byte[3]);

		pn544_printk(NFC_DEBUGFS, "--%s: ret = %d, buf(%s)--\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static const struct file_operations pn544_TypeA_reader_operations = {
	.open		= pn544_hci_TypeA_reader_open,
	.read		= pn544_hci_TypeA_reader_read,
};
#endif

//add for debugfs entries
static void pn544_create_debugfs_entries(struct pn544_info *info)
{
	pn544_printk(NFC_DEBUG, "--%s--\n", __func__);

	dent = debugfs_create_dir(PN544_DRIVER_NAME, NULL);
	if (dent) {
	//add for self test with HCI command test script
	#ifdef CONFIG_PN544_TEST
		debugfs_create_file("read_chip_id", S_IRUGO, dent, info, &pn544_read_chip_id_fops);
		debugfs_create_file("read_sw_version", S_IRUGO, dent, info, &pn544_read_sw_version_fops);
		debugfs_create_file("antenna_self_test", S_IRUGO | S_IWUGO, dent, info, &pn544_antenna_sleftest_operations);
		debugfs_create_file("swp_self_test", S_IRUGO, dent, info, &pn544_swp_sleftest_operations);
		debugfs_create_file("switch_Tx", S_IWUGO, dent, info, &pn544_switchTx_operations);
		debugfs_create_file("TypeA_reader", S_IRUGO, dent, info, &pn544_TypeA_reader_operations);
	#endif
		debugfs_create_file("host_int_gpio", S_IRUGO | S_IWUGO, dent, info, &pn544_host_int_gpio_fops);
		debugfs_create_file("enable_gpio", S_IRUGO | S_IWUGO, dent, info, &pn544_enable_gpio_fops);
		debugfs_create_file("reset_gpio", S_IRUGO | S_IWUGO, dent, info, &pn544_fw_reset_gpio_fops);
	/* Jen Chang add for check nfc chip is ok or not */
		debugfs_create_file("dev_i2c_response", S_IRUGO | S_IWUGO, dent, info, &pn544_dev_i2c_response_fops);
	/* Jen Chang, 20130221 */
	}
}

static void pn544_destroy_debugfs_entries(void)
{
	pn544_printk(NFC_DEBUG, "--%s--\n", __func__);

	if (dent)
		debugfs_remove_recursive(dent);
}

//add for kernel debug level entries
static void pn544_create_kernel_debuglevel_entries(void)
{
	pn544_printk(NFC_DEBUG, "--%s--\n", __func__);

	memset(&PN544_DEBUG_DLL, 0, sizeof(PN544_DEBUG_DLL));

	printk("PN544_DEBUG_DLL(%d)\n", PN544_DEBUG_DLL.debug_level);

	if (kernel_debuglevel_dir != NULL) {
		debugfs_create_u8("pn544_test_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u8 *)(&PN544_DEBUG_DLL.debug_byte[0]));
		debugfs_create_u8("pn544_info_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u8 *)(&PN544_DEBUG_DLL.debug_byte[1]));
		debugfs_create_u8("pn544_debug_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u8 *)(&PN544_DEBUG_DLL.debug_byte[2]));
	} else {
		pn544_printk(NFC_CRITICIAL, "smb136 kernel debuglevel dir falied\n");
	}
}

static void pn544_destroy_kernel_debuglevel_entries(void)
{
	pn544_printk(NFC_DEBUG, "--%s--\n", __func__);

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}
#else
static void pn544_create_debugfs_entries(struct pn544_data *info)
{
}
static void pn544_destroy_debugfs_entries(void)
{
}
static void pn544_create_kernel_debuglevel_entries(void)
{
}
static void pn544_destroy_kernel_debuglevel_entries(void)
{
}
#endif
/* Jen Chang, 20130114 */

static int __devinit pn544_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct pn544_info *info;
	struct pn544_nfc_platform_data *pdata;
	int i = 0, r = 0;
/* Jen Chang add for i2c address */
	char test_addr = PN544_I2C_ADDR;
/* Jen Chang, 20121217 */

	/* private data allocation */
	info = kzalloc(sizeof(struct pn544_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev,
			"Cannot allocate memory for pn544_info.\n");
		r = -ENOMEM;
		goto err_info_alloc;
	}

	info->buflen = max(PN544_MSG_MAX_SIZE, PN544_MAX_I2C_TRANSFER);
	info->buf = kzalloc(info->buflen, GFP_KERNEL);
	if (!info->buf) {
		dev_err(&client->dev,
			"Cannot allocate memory for pn544_info->buf.\n");
		r = -ENOMEM;
		goto err_buf_alloc;
	}

/* Jen Chang modify for requesting requlator */
/* Fix me -- We only need to control Vdd_io power source,
			Vbat can't turn off and Vsim is controlled by nfc driver */
	info->regs[0] = nfc_regs[0];
//	info->regs[1] = nfc_regs[1];
//	info->regs[2] = nfc_regs[2];

	r = regulator_bulk_get(&client->dev, ARRAY_SIZE(info->regs),
				 info->regs);
	if (r < 0)
		goto err_kmalloc;

	r = regulator_bulk_set_voltage(ARRAY_SIZE(info->regs), info->regs);
	if (r < 0)
		goto err_kmalloc;

	r = regulator_bulk_enable(ARRAY_SIZE(info->regs), info->regs);
	if (r < 0)
		goto err_kmalloc;
/* Jen Chang, 20130206 */

	info->i2c_dev = client;
	mutex_init(&info->read_mutex);
	mutex_init(&info->mutex);
	init_waitqueue_head(&info->read_wait);
	spin_lock_init(&info->irq_enabled_lock);
	i2c_set_clientdata(client, info);

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "No platform data\n");
		r = -EINVAL;
		goto err_reg;
	}

	if (!pdata->request_resources) {
		dev_err(&client->dev, "request_resources() missing\n");
		r = -EINVAL;
		goto err_reg;
	}

	r = pdata->request_resources(client);
	if (r) {
		dev_err(&client->dev, "Cannot get platform resources\n");
		goto err_reg;
	}

/* Jen Chang modify to check i2c response from nfc chip and register pmlog device */
#ifdef CONFIG_PM_LOG
	info->pmlog_device = pmlog_register_device(&client->dev);
#endif

//add for reset pn544 power to wait stable
	pn544_disable(info);
	pn544_enable(info, HCI_MODE);

//add to check i2c response from nfc chip
	do {
		r = i2c_master_send(client, &test_addr, 1);
		pn544_printk(NFC_PROBE, "%s: (%d) send(0x%x) to pn544, ret(%d)\n",
			__func__, i, test_addr, r);

		r = i2c_master_recv(client, &dev_i2c_response, 1);
		pn544_printk(NFC_PROBE, "%s: (%d) recv(0x%x) from pn544, ret(%d)\n",
			__func__, i, dev_i2c_response, r);

		if (dev_i2c_response == ((test_addr << 1) + 1))
			break;
		else {
			i++;
			/* delay time range is depended on 0x9C27,0x9c28(AckHostTimeout) and
				0x9c31,0x9c32(GuardHostTimeout) */
			usleep_range(200000, 250000);
		}

		if (i >= 5) {
			dev_i2c_response = 0;
			pn544_printk(NFC_PROBE, "%s: No nfc device exist!!\n", __func__);
		/* Jen Chang mark it to establish /dev/pn544 device node for nfc force download */
		//	goto err_res;
		/* Jen Chang, 20130221 */
		}
	} while (i < 5);
/* Jen Chang, 20130114 */

	pr_info("[NFC]PN544 probe : misc devices setting....\n");
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = PN544_DRIVER_NAME;
	info->miscdev.fops = &pn544_fops;
	info->miscdev.parent = &client->dev;
	r = misc_register(&info->miscdev);
	if (r < 0) {
		dev_err(&client->dev, "Device registration failed\n");
		goto err_sysfs;
	}

	pr_info("[NFC]PN544 probe : IRQ setting....\n");
	info->irq_enabled = true;
	info->irq_gpio = pdata->irq_gpio;

	r = request_irq(client->irq, pn544_dev_irq_handler,
					IRQF_TRIGGER_HIGH, PN544_DRIVER_NAME, info);
	if (r < 0) {
		dev_err(&client->dev, "Unable to register IRQ handler\n");
		goto err_res;
	}

	pn544_disable_irq(info);

	/* If we don't have the test we don't need the sysfs file */
	if (pdata->test) {
		/* sysfs entries */
		for (i = 0; i < ARRAY_SIZE(pn544_attr); i++) {
			r = device_create_file(&client->dev, &pn544_attr[i]);
		if (r) {
			dev_err(&client->dev,
				"sysfs registration failed, error %d\n", r);
			goto err_irq;
		}
	}
	}

/* Jen Chang add for wake up cpu by reading nfc tag card */
	wake_lock_init(&info->wake_lock, WAKE_LOCK_SUSPEND, PN544_DRIVER_NAME);

//add for debugfs enteries and kerenl debug level entries
	pn544_create_debugfs_entries(info);
	pn544_create_kernel_debuglevel_entries();

	pn544_disable(info);
	pn544_enable(info, HCI_MODE);

//add for getting chip id
#ifdef CONFIG_PN544_TEST
	memset(&pn544_test_info, 0, sizeof(struct chip_test_data_t));

#if 0 //mark it for reducing booting time
	//add for getting device id with HCI command test script
	pn544_printk(NFC_TEST, "[NFC_TEST] start nfc version getting...");
	PN544_NFC_TEST(r_ver, client, &pn544_test_info);
	pn544_disable (info);

	pn544_printk(NFC_PROBE, "%s success: nfc sw version is %d-%d\n",
		__func__,
		pn544_test_info.chip_version.sw_ver.value_byte[1],
		pn544_test_info.chip_version.sw_ver.value_byte[0]);

	return 0;
#endif
#endif

	pn544_disable (info);

	pn544_printk(NFC_PROBE, "%s success!!\n", __func__);

	return 0;

err_sysfs:
	if (pdata->test) {
		for (i = 0; i < ARRAY_SIZE(pn544_attr); i++)
			device_remove_file(&client->dev, &pn544_attr[i]);
	}
err_irq:
//add for wake up cpu by reading nfc tag card
	wake_lock_destroy(&info->wake_lock);
	free_irq(client->irq, info);
err_res:
	if (pdata->free_resources)
		pdata->free_resources();
err_reg:
	regulator_bulk_free(ARRAY_SIZE(info->regs), info->regs);
err_kmalloc:
	kfree(info->buf);
err_buf_alloc:
	kfree(info);
err_info_alloc:
	return r;
}
/* Jen Chang, 20121217 */

static __devexit int pn544_remove(struct i2c_client *client)
{
	struct pn544_info *info = i2c_get_clientdata(client);
	struct pn544_nfc_platform_data *pdata = client->dev.platform_data;

	int i;

	misc_deregister(&info->miscdev);

	if (pdata->test) {
		for (i = 0; i < ARRAY_SIZE(pn544_attr); i++)
			device_remove_file(&client->dev, &pn544_attr[i]);
	}
/* Jen Chang add for wake up cpu by reading nfc tag card */
	wake_lock_destroy(&info->wake_lock);

//add for debugfs entries and kernel debug level entries
	pn544_destroy_debugfs_entries();
	pn544_destroy_kernel_debuglevel_entries();
/* Jen Chang, 20121217 */

	if (info->state != PN544_ST_COLD) {
		if (pdata->disable)
			pdata->disable();

		info->read_irq = PN544_NONE;
	}

	free_irq(client->irq, info);
	if (pdata->free_resources)
		pdata->free_resources();

	regulator_bulk_free(ARRAY_SIZE(info->regs), info->regs);

	kfree(info->buf);
	kfree(info);

	pn544_printk(NFC_PROBE, "%s remove success!\n", __func__);

	return 0;
}

static struct i2c_driver pn544_driver = {
	.driver = {
		.name = PN544_DRIVER_NAME,
#ifdef CONFIG_PM
		.pm = &pn544_pm_ops,
#endif
	},
	.probe = pn544_probe,
	.id_table = pn544_id_table,
	.remove = __devexit_p(pn544_remove),
};

static int __init pn544_init(void)
{
	int r;

	r = i2c_add_driver(&pn544_driver);
	if (r) {
		pr_err(PN544_DRIVER_NAME ": driver registration failed\n");
		return r;
	}
	pn544_printk(NFC_PROBE, "%s initialize success!\n", __func__);

	return 0;
}

static void __exit pn544_exit(void)
{
	i2c_del_driver(&pn544_driver);
	pn544_printk(NFC_PROBE, "%s exit success!\n", __func__);
}

module_init(pn544_init);
module_exit(pn544_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
