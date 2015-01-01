/*
 *  The Other Half smart cover driver
 *
 *  Copyright (c) 2013 Jolla ltd.
 *  Contact: Kalle Jokiniemi <kalle.jokiniemi@jollamobile.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/memory.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/mfd/toh.h>
#include <linux/regulator/consumer.h>

#define TOH_NAME		"toh-core"
#define TOH_EEPROM_FIXED_DATA	0x30
#define TOH_EEPROM_ENTRIES	8
#define TOH_EEPROM_DETECT_RETRY	4
#define TOH_EEPROM_DETECT_DELAY	75
#define TOH_DATA_ENTRIES	1
#define TOH_SYSFS_ENTRIES	(TOH_EEPROM_ENTRIES + TOH_DATA_ENTRIES)

/* EEPROM field identifier macros */
#define TOH_EEPROM_VENDOR	"vendor"
#define TOH_EEPROM_PRODUCT	"product"
#define TOH_EEPROM_REV		"rev"
#define TOH_EEPROM_EEPROM_SIZE	"eeprom_size"
#define TOH_EEPROM_CFG_ADDR	"cfg_addr"
#define TOH_EEPROM_CFG_SIZE	"cfg_size"
#define TOH_EEPROM_UDATA_ADDR	"udata_addr"
#define TOH_EEPROM_UDATA_SIZE	"udata_size"

struct toh_eeprom_entry {
	char name[12];
	u16 size;	/* entry size in 8-bit bytes */
	u8 offset;	/* address offset of the data */
};

struct toh_eeprom_entry toh_eeprom[] = {
	{ TOH_EEPROM_VENDOR, 2, 0 },
	{ TOH_EEPROM_PRODUCT, 2, 2 },
	{ TOH_EEPROM_REV, 1, 4 },
	{ TOH_EEPROM_EEPROM_SIZE, 2, 5 },
	{ TOH_EEPROM_CFG_ADDR, 2, 7 },
	{ TOH_EEPROM_CFG_SIZE, 2, 9 },
	{ TOH_EEPROM_UDATA_ADDR, 2, 11 },
	{ TOH_EEPROM_UDATA_SIZE, 2, 13 },
};

struct toh_data {
	struct device_attribute *attrs;
	struct device		*dev;
	struct memory_accessor	*macc;
	struct toh_eeprom_entry	*eeprom;
	struct i2c_client	*i2c_dev;
	struct regulator	*vdd;
	int			i2c_bus;
	int 			sysfs_enabled;
	u16			cfg_size;
	u16			cfg_addr;
	u16			eeprom_size;
	u16			udata_addr;
};

/*
 * Use static struct instead of dynamic allocation to be able to pass it to
 * at24 platform data.
 */
static struct toh_data other_half;

void toh_setup_eeprom(struct memory_accessor *macc, void *context)
{
	struct toh_data *toh = context;

	toh->macc = macc;
}

static void memcpy_inv(void *dest, void *src, size_t count)
{
	unsigned char *d = (unsigned char *)dest;
	unsigned char *s = (unsigned char *)src;

	s += count;

	while (count--)
		*d++ = *(--s);

}

static int toh_find_entry(struct toh_data *toh, char *entry_name)
{
	int i = 0;

	while (strcmp(toh->eeprom[i].name, entry_name))
		i++;

	return i;
}

static ssize_t toh_eeprom_read(struct toh_data *toh, char *buf,
				unsigned int address, unsigned int size)
{
	int ret = 0;

	ret = regulator_enable(toh->vdd);
	if (ret) {
		dev_err(toh->dev, "Could not enable regulator\n");
		return ret;
	}

	if (toh->macc) {
		ret = toh->macc->read(toh->macc, buf, address, size);
	} else {
		dev_err(toh->dev, "No memory accessor for eeprom\n");
		ret = -ENODEV;
	}

	regulator_disable(toh->vdd);
	return ret;
}

static int toh_eeprom_read_entry(struct toh_data *toh, void *dest,
							char *entry_name)
{
	int i = toh_find_entry(toh, entry_name);
	ssize_t count;
	char temp[toh->eeprom[i].size];

	count = toh_eeprom_read(toh, temp, toh->eeprom[i].offset,
						toh->eeprom[i].size);
	if (count != toh->eeprom[i].size) {
		dev_err(toh->dev, "Could not read entry %s\n", entry_name);
		return count;
	}

	memcpy_inv(dest, temp, toh->eeprom[i].size);
	return 0;
}

static ssize_t toh_cfg_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
				container_of(dev, struct platform_device, dev);
	struct toh_data *toh = platform_get_drvdata(pdev);
	char cbuf[toh->cfg_size + 1];
	int count = 0;

	if (toh->cfg_addr < TOH_EEPROM_FIXED_DATA) {
		dev_err(toh->dev, "Invalid cfg_data eeprom address 0x%x\n",
						toh->cfg_addr);
		return -EFAULT;
	}
	if ((toh->cfg_addr + toh->cfg_size) > toh->eeprom_size) {
		dev_err(toh->dev, "Config size %d bigger than eeprom size %d\n",
			toh->cfg_addr + toh->cfg_size, toh->eeprom_size);
		return -EINVAL;
	}
	if (toh->udata_addr && (toh->udata_addr < (toh->cfg_addr +
							toh->cfg_size))) {
		dev_err(toh->dev, "Config end %d overlaps user data at %d\n",
				toh->cfg_addr + toh->cfg_size, toh->udata_addr);
		return -EINVAL;
	}

	count = toh_eeprom_read(toh, cbuf, toh->cfg_addr, toh->cfg_size);
	if (count != toh->cfg_size) {
		dev_err(toh->dev, "Could not read config data\n");
		return count;
	}

	cbuf[toh->cfg_size] = 0;

	return snprintf(buf, toh->cfg_size, "%s\n", cbuf);
}

static ssize_t toh_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct platform_device *pdev =
				container_of(dev, struct platform_device, dev);
	struct toh_data *toh = platform_get_drvdata(pdev);
	char ebuf[8];
	char inv_buf[8];
	int i = 0;
	ssize_t count;

	if (!toh->macc) {
		dev_err(dev, "No memory accessor set\n");
		return -ENXIO;
	}

	memset(ebuf, 0, 8);
	memset(inv_buf, 0, 8);

	/* find the index for which attribute this call is meant to */
	while (&toh->attrs[i] != attr)
		i++;

	count = toh_eeprom_read(toh, ebuf, toh->eeprom[i].offset,
							toh->eeprom[i].size);

	if (count != toh->eeprom[i].size) {
		dev_err(dev, "Could not read eeprom\n");
		return count;
	}

	/* Reverse byte order of the buffer to make it cast properly to u64 */
	memcpy_inv(inv_buf, ebuf, toh->eeprom[i].size);

	return snprintf(buf, PAGE_SIZE, "%llu\n", *(u64 *)inv_buf);
}

static ssize_t toh_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/* Not allowed */
	return -EPERM;
}

static int toh_sysfs_create(struct toh_data *toh)
{
	int i = 0;

	toh->attrs = kmalloc(sizeof(struct device_attribute) *
			TOH_SYSFS_ENTRIES, GFP_KERNEL);
	if (!toh->attrs) {
		dev_err(toh->dev, "Not enough memory for attributes\n");
		goto toh_sysfs_exit;
	}

	/* Fixed entries */
	for (i = 0; i < TOH_EEPROM_ENTRIES; i++) {
		toh->attrs[i].attr.name = toh->eeprom[i].name;
		toh->attrs[i].attr.mode = 0440;
		toh->attrs[i].show = toh_show;
		toh->attrs[i].store = toh_store;
		if (sysfs_create_file(&toh->dev->kobj, &toh->attrs[i].attr)) {
			dev_err(toh->dev, "Not enough memory for sysfs\n");
			goto toh_sysfs_clean;
		}
	}

	/* String entries, append on top of EEPROM entry attributes */
	toh->attrs[i].attr.name = "config_data";
	toh->attrs[i].attr.mode = 0440;
	toh->attrs[i].show = toh_cfg_show;
	toh->attrs[i].store = toh_store;
	if (sysfs_create_file(&toh->dev->kobj, &toh->attrs[i].attr)) {
		dev_err(toh->dev, "Not enough memory for sysfs\n");
		goto toh_sysfs_clean;
	}

	kobject_uevent(&toh->dev->kobj, KOBJ_ADD);

	return 0;

toh_sysfs_clean:
	while (i > 0) {
		i--;
		sysfs_remove_file(&toh->dev->kobj, &toh->attrs[i].attr);
	}
	kfree(toh->attrs);

toh_sysfs_exit:
	return -ENOMEM;
}

void toh_sysfs_remove(struct toh_data *toh)
{
	int i;

	for (i = 0; i < TOH_SYSFS_ENTRIES; i++)
		sysfs_remove_file(&toh->dev->kobj, &toh->attrs[i].attr);

	kfree(toh->attrs);
}

static struct at24_platform_data toh_eeprom_data = {
	.byte_len       = (16384 / 8),
	.page_size	= 1,	/* play safe so all EEPROMS should work */
	.flags          = AT24_FLAG_READONLY,
	.setup          = toh_setup_eeprom,
	.context	= &other_half,
};

static struct i2c_board_info toh_i2c_eeprom_info = {
	I2C_BOARD_INFO("at24", 0x50),
	.platform_data  = &toh_eeprom_data,
};

static int __devinit toh_probe(struct platform_device *pdev)
{
	struct toh_data *toh = &other_half;
	struct i2c_adapter *i2c_adap;
	struct toh_platform_data *pdata = pdev->dev.platform_data;
	ssize_t count = 0;
	int ret = 0;
	char buf;
	int retries;

	toh->i2c_bus = pdata->i2c_bus;
	toh->eeprom = toh_eeprom;
	toh->macc = NULL;
	toh->dev = &pdev->dev;

	toh->vdd = regulator_get(toh->dev, "toh_vdd");

	if (IS_ERR(toh->vdd)) {
		dev_err(toh->dev, "Could not find toh_vdd regulator\n");
		ret = PTR_ERR(toh->vdd);
		goto probe_exit;
	}

	ret = regulator_enable(toh->vdd);
	if (ret) {
		dev_err(toh->dev, "Could not enable regulator\n");
		goto regulator_err;
	}

	i2c_adap = i2c_get_adapter(toh->i2c_bus);
	if (!i2c_adap) {
		ret = -ENODEV;
		dev_err(toh->dev, "Could not find I2C bus %d\n",
							toh->i2c_bus);
		goto i2c_err;
	}

	/* toh_setup_eeprom gets called as result of this */
	toh->i2c_dev = i2c_new_device(i2c_adap, &toh_i2c_eeprom_info);
	if (!toh->i2c_dev) {
		ret = -ENOMEM;
		dev_err(toh->dev, "Not enough memory for eeprom device %d\n",
							toh->i2c_bus);
		goto i2c_err;
	}

	/* Test if the eeprom is really readable / present on the cover */
	retries =  TOH_EEPROM_DETECT_RETRY;
	while (retries && count <= 0) {
		msleep(TOH_EEPROM_DETECT_DELAY);
		count = toh_eeprom_read(toh, &buf, 1, 1);
		retries--;
	}

	regulator_disable(toh->vdd);

	if (count <= 0) {
		/* No eeprom on this cover, note this is a valid condition */
		dev_info(toh->dev, "No valid eeprom present\n");
		toh->sysfs_enabled = 0;
	} else {
		toh_eeprom_read_entry(toh, &toh->cfg_addr, TOH_EEPROM_CFG_ADDR);
		toh_eeprom_read_entry(toh, &toh->cfg_size, TOH_EEPROM_CFG_SIZE);
		toh_eeprom_read_entry(toh, &toh->eeprom_size,
							TOH_EEPROM_EEPROM_SIZE);
		toh_eeprom_read_entry(toh, &toh->udata_addr,
							TOH_EEPROM_UDATA_ADDR);
		toh_sysfs_create(toh);
		toh->sysfs_enabled = 1;
	}

	platform_set_drvdata(pdev, toh);
	dev_dbg(toh->dev, "TOH driver probed succesfully");

	return ret;

i2c_err:
	regulator_disable(toh->vdd);
regulator_err:
	regulator_put(toh->vdd);
probe_exit:
	return ret;
}

static int __devexit toh_remove(struct platform_device *pdev)
{
	struct toh_data *toh = platform_get_drvdata(pdev);

	regulator_put(toh->vdd);

	if (toh->sysfs_enabled)
		toh_sysfs_remove(toh);
	if (toh->i2c_dev)
		i2c_unregister_device(toh->i2c_dev);

	dev_dbg(toh->dev, "TOH driver removed succesfully");

	return 0;
}

static struct platform_driver toh_driver = {
	.probe		= toh_probe,
	.remove		= toh_remove,
	.driver = {
		.name = TOH_NAME,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(toh_driver);

MODULE_DESCRIPTION("The Other Half smart cover driver");
MODULE_AUTHOR("Kalle Jokiniemi <kalle.jokiniemi@jollamobile.com>");
MODULE_LICENSE("GPL v2");
