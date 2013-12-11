/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name		: l3gd20_gyr_sysfs.c
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.2.1 sysfs
* Date			: 2012/Jul/10
* Description		: L3GD20 digital output gyroscope sensor API
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS	  | DESCRIPTION
* 1.0		| 2010/May/02	| Carmine Iascone | First Release
* 1.1.3		| 2011/Jun/24	| Matteo Dameno	  | Corrects ODR Bug
* 1.1.4		| 2011/Sep/02	| Matteo Dameno	  | SMB Bus Mng,
*		|		|		  | forces BDU setting
* 1.1.5		| 2011/Sep/24	| Matteo Dameno	  | Introduces FIFO Feat.
* 1.1.5.2	| 2011/Nov/11	| Matteo Dameno	  | enable gpio_int to be
*		|		|		  | passed as parameter at
*		|		|		  | module loading time;
*		|		|		  | corrects polling
*		|		|		  | bug at end of probing;
* 1.1.5.3	| 2011/Dec/20	| Matteo Dameno	  | corrects error in
*		|		|		  | I2C SADROOT; Modifies
*		|		|		  | resume suspend func.
* 1.1.5.4	| 2012/Jan/09	| Matteo Dameno	  | moved under input/misc;
* 1.1.5.5	| 2012/Mar/30	| Matteo Dameno	  | moved watermark, use_smbus,
*		|		|		  | fifomode @ struct foo_status
*		|		|		  | sysfs range input format
*		|		|		  | changed to decimal
* 1.2		| 2012/Jul/10	| Denis Ciocca	  | input_poll_dev removal
* 1.2.1		| 2012/Jul/10	| Denis Ciocca	  | added high resolution timers
*******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/delay.h>

#include <linux/input/l3gd20.h>
/*#include "l3gd20.h"*/

/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif
/* WH Lee, 20130107 */

/* Maximum polled-device-reported rot speed value value in dps */
#define FS_MAX		32768
#define MS_TO_NS(x)	(x*1000000L)

/* l3gd20 gyroscope registers */
#define WHO_AM_I	(0x0F)

#define SENSITIVITY_250		8750		/*	udps/LSB */
#define SENSITIVITY_500		17500		/*	udps/LSB */
#define SENSITIVITY_2000	70000		/*	udps/LSB */

#define CTRL_REG1	(0x20)    /* CTRL REG1 */
#define CTRL_REG2	(0x21)    /* CTRL REG2 */
#define CTRL_REG3	(0x22)    /* CTRL_REG3 */
#define CTRL_REG4	(0x23)    /* CTRL_REG4 */
#define CTRL_REG5	(0x24)    /* CTRL_REG5 */
#define	REFERENCE	(0x25)    /* REFERENCE REG */
#define	FIFO_CTRL_REG	(0x2E)    /* FIFO CONTROL REGISTER */
#define FIFO_SRC_REG	(0x2F)    /* FIFO SOURCE REGISTER */
#define	OUT_X_L		(0x28)    /* 1st AXIS OUT REG of 6 */

#define AXISDATA_REG	OUT_X_L

/* CTRL_REG1 */
#define ALL_ZEROES	(0x00)
#define PM_OFF		(0x00)
#define PM_NORMAL	(0x08)
#define ENABLE_ALL_AXES	(0x07)
#define ENABLE_NO_AXES	(0x00)
#define BW00		(0x00)
#define BW01		(0x10)
#define BW10		(0x20)
#define BW11		(0x30)
#define ODR095		(0x00)  /* ODR =  95Hz */
#define ODR190		(0x40)  /* ODR = 190Hz */
#define ODR380		(0x80)  /* ODR = 380Hz */
#define ODR760		(0xC0)  /* ODR = 760Hz */

/* CTRL_REG3 bits */
#define	I2_DRDY		(0x08)
#define	I2_WTM		(0x04)
#define	I2_OVRUN	(0x02)
#define	I2_EMPTY	(0x01)
#define	I2_NONE		(0x00)
#define	I2_MASK		(0x0F)

/* CTRL_REG4 bits */
#define	FS_MASK		(0x30)
#define	BDU_ENABLE	(0x80)

/* CTRL_REG5 bits */
#define	FIFO_ENABLE	(0x40)
#define HPF_ENALBE	(0x11)

/* FIFO_CTRL_REG bits */
#define	FIFO_MODE_MASK		(0xE0)
#define	FIFO_MODE_BYPASS	(0x00)
#define	FIFO_MODE_FIFO		(0x20)
#define	FIFO_MODE_STREAM	(0x40)
#define	FIFO_MODE_STR2FIFO	(0x60)
#define	FIFO_MODE_BYPASS2STR	(0x80)
#define	FIFO_WATERMARK_MASK	(0x1F)

#define FIFO_STORED_DATA_MASK	(0x1F)

#define I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_FIFO_CTRL_REG	5
#define	RESUME_ENTRIES		6


#define MIN_X        175
#define MAX_X        875
#define MIN_Y        175
#define MAX_Y        875
#define MIN_Z        175
#define MAX_Z        875

/* #define DEBUG 1 */

/** Registers Contents */
#define WHOAMI_L3GD20_GYR	(0xD4)  /* Expected content for WAI register*/

static int int1_gpio = L3GD20_GYR_DEFAULT_INT1_GPIO;
static int int2_gpio = L3GD20_GYR_DEFAULT_INT2_GPIO;
/* module_param(int1_gpio, int, S_IRUGO); */
module_param(int2_gpio, int, S_IRUGO);

/*
 * L3GD20 gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * s32
 */

struct l3gd20_gyr_triple {
	s32	x,	/* x-axis angular rate data. */
		y,	/* y-axis angluar rate data. */
		z;	/* z-axis angular rate data. */
};

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {

	{	2,	ODR760|BW10},
	{	3,	ODR380|BW01},
	{	6,	ODR190|BW00},
	{	11,	ODR095|BW00},
};

static struct l3gd20_gyr_platform_data default_l3gd20_gyr_pdata = {
	.fs_range = L3GD20_GYR_FS_250DPS,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 1,
	.negate_z = 0,
	/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
	/* improve gyro-sensor performance */
	.poll_interval = 10,
	/* Original code */
	.poll_interval = 100,
	/* WH Lee, 20130307 */
	.min_interval = L3GD20_GYR_MIN_POLL_PERIOD_MS, /* 2ms */

	.gpio_int1 = L3GD20_GYR_DEFAULT_INT1_GPIO,
	.gpio_int2 = L3GD20_GYR_DEFAULT_INT2_GPIO,	/* int for fifo */

};

struct workqueue_struct *l3gd20_gyr_workqueue = 0;

struct l3gd20_gyr_status {
	struct i2c_client *client;
	struct l3gd20_gyr_platform_data *pdata;

	struct mutex lock;

	struct input_dev *input_dev;

	int hw_initialized;
	atomic_t enabled;
	/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
	/* Add for cancel/stop timer */
	atomic_t timer_canceled;
	/* WH Lee, 20130218 */
	int use_smbus;

	u8 reg_addr;
	u8 resume_state[RESUME_ENTRIES];

	u32 sensitivity;

	/* interrupt related */
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	bool polling_enabled;
	/* fifo related */
	u8 watermark;
	u8 fifomode;

	struct hrtimer hr_timer;
	ktime_t ktime;
	struct work_struct polling_task;
	/* Detroit3.0 CR #:XXX, WH Lee, 20121107 */
	struct l3gd20_gyr_triple data;
	/* WH Lee, 20121107 */
	/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
	struct pmlog_device *pmlog_device;
#endif
	/* WH Lee, 20130107 */
};

/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
/* Improve gyro-sensor performance */
static void l3gd20_gyr_report_triple(struct l3gd20_gyr_status *stat);
/* WH Lee, 20130307 */

static int l3gd20_gyr_i2c_read(struct l3gd20_gyr_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;

/*
	if (len > sizeof(buf))
			dev_err(&stat->client->dev,
				"read error insufficient buffer length: "
				"len:%d, buf size=%d\n",
				len, sizeof(buf));
*/
	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
				"command=0x%02x, ",
				ret, len, cmd);
			unsigned int ii;
			for (ii = 0; ii < len; ii++)
				printk(KERN_DEBUG "buf[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&stat->client->dev,
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			return 0; /* failure */
		}
		return len; /* success */
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int l3gd20_gyr_i2c_write(struct l3gd20_gyr_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg, value;

	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_i2c_block_data: ret=%d, "
				"len:%d, command=0x%02x, ",
				ret, len, reg);
			unsigned int ii;
			for (ii = 0; ii < (len + 1); ii++)
				printk(KERN_DEBUG "value[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}


static int l3gd20_gyr_register_write(struct l3gd20_gyr_status *stat,
		u8 *buf, u8 reg_address, u8 new_value)
{
	int err;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = l3gd20_gyr_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;

	return err;
}

static int l3gd20_gyr_register_read(struct l3gd20_gyr_status *stat,
							u8 *buf, u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = l3gd20_gyr_i2c_read(stat, buf, 1);
	return err;
}

static int l3gd20_gyr_register_update(struct l3gd20_gyr_status *stat,
			u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = l3gd20_gyr_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = l3gd20_gyr_register_write(stat, buf, reg_address,
				updated_val);
	}
	return err;
}


static int l3gd20_gyr_update_watermark(struct l3gd20_gyr_status *stat,
								u8 watermark)
{
	int res = 0;
	u8 buf[2];
	u8 new_value;

	mutex_lock(&stat->lock);
	new_value = (watermark % 0x20);
	res = l3gd20_gyr_register_update(stat, buf, FIFO_CTRL_REG,
			 FIFO_WATERMARK_MASK, new_value);
	if (res < 0) {
		dev_err(&stat->client->dev, "failed to update watermark\n");
		return res;
	}
	dev_dbg(&stat->client->dev, "%s new_value:0x%02x,watermark:0x%02x\n",
						__func__, new_value, watermark);

	stat->resume_state[RES_FIFO_CTRL_REG] =
		((FIFO_WATERMARK_MASK & new_value) |
		(~FIFO_WATERMARK_MASK &
				stat->resume_state[RES_FIFO_CTRL_REG]));
	stat->watermark = new_value;
	mutex_unlock(&stat->lock);
	return res;
}

static int l3gd20_gyr_update_fifomode(struct l3gd20_gyr_status *stat,
								u8 fifomode)
{
	int res;
	u8 buf[2];
	u8 new_value;

	new_value = fifomode;
	res = l3gd20_gyr_register_update(stat, buf, FIFO_CTRL_REG,
					FIFO_MODE_MASK, new_value);
	if (res < 0) {
		dev_err(&stat->client->dev, "failed to update fifoMode\n");
		return res;
	}
	/*
	dev_dbg(&stat->client->dev, "new_value:0x%02x,prev fifomode:0x%02x\n",
				__func__, new_value, stat->fifomode);
	 */
	stat->resume_state[RES_FIFO_CTRL_REG] =
		((FIFO_MODE_MASK & new_value) |
		(~FIFO_MODE_MASK &
				stat->resume_state[RES_FIFO_CTRL_REG]));
	stat->fifomode = new_value;

	return res;
}

static int l3gd20_gyr_fifo_reset(struct l3gd20_gyr_status *stat)
{
	u8 oldmode;
	int res;

	oldmode = stat->fifomode;
	res = l3gd20_gyr_update_fifomode(stat, FIFO_MODE_BYPASS);
	if (res < 0)
		return res;
	res = l3gd20_gyr_update_fifomode(stat, oldmode);
	if (res >= 0)
		dev_dbg(&stat->client->dev, "%s fifo reset to: 0x%02x\n",
							__func__, oldmode);

	return res;
}

static int l3gd20_gyr_fifo_hwenable(struct l3gd20_gyr_status *stat,
								u8 enable)
{
	int res;
	u8 buf[2];
	u8 set = 0x00;
	if (enable)
		set = FIFO_ENABLE;

	res = l3gd20_gyr_register_update(stat, buf, CTRL_REG5,
			FIFO_ENABLE, set);
	if (res < 0) {
		dev_err(&stat->client->dev, "fifo_hw switch to:0x%02x failed\n",
									set);
		return res;
	}
	stat->resume_state[RES_CTRL_REG5] =
		((FIFO_ENABLE & set) |
		(~FIFO_ENABLE & stat->resume_state[RES_CTRL_REG5]));
	dev_dbg(&stat->client->dev, "%s set to:0x%02x\n", __func__, set);
	return res;
}

static int l3gd20_gyr_manage_int2settings(struct l3gd20_gyr_status *stat,
								u8 fifomode)
{
	int res;
	u8 buf[2];
	bool enable_fifo_hw;
	bool recognized_mode = false;
	u8 int2bits = I2_NONE;
/*
	if (stat->polling_enabled) {
		fifomode = FIFO_MODE_BYPASS;
		dbg_warn(&stat->client->dev, "in polling mode, fifo mode forced"
							" to BYPASS mode\n");
	}
*/


	switch (fifomode) {
	case FIFO_MODE_FIFO:
		recognized_mode = true;

		if (stat->polling_enabled) {
			int2bits = I2_NONE;
			enable_fifo_hw = false;
		} else {
			int2bits = (I2_WTM | I2_OVRUN);
			enable_fifo_hw = true;
		}
		res = l3gd20_gyr_register_update(stat, buf, CTRL_REG3,
					I2_MASK, int2bits);
		if (res < 0) {
			dev_err(&stat->client->dev, "%s : failed to update "
							"CTRL_REG3:0x%02x\n",
							__func__, fifomode);
			goto err_mutex_unlock;
		}
		stat->resume_state[RES_CTRL_REG3] =
			((I2_MASK & int2bits) |
			(~(I2_MASK) & stat->resume_state[RES_CTRL_REG3]));
		/* enable_fifo_hw = true; */
		break;

	case FIFO_MODE_BYPASS:
		recognized_mode = true;

		if (stat->polling_enabled)
			int2bits = I2_NONE;
		else
			int2bits = I2_DRDY;

		res = l3gd20_gyr_register_update(stat, buf, CTRL_REG3,
					I2_MASK, int2bits);
		if (res < 0) {
			dev_err(&stat->client->dev, "%s : failed to update"
						" to CTRL_REG3:0x%02x\n",
							__func__, fifomode);
			goto err_mutex_unlock;
		}
		stat->resume_state[RES_CTRL_REG3] =
			((I2_MASK & int2bits) |
			(~I2_MASK & stat->resume_state[RES_CTRL_REG3]));
		enable_fifo_hw = false;
		break;

	default:
		recognized_mode = false;
		res = l3gd20_gyr_register_update(stat, buf, CTRL_REG3,
					I2_MASK, I2_NONE);
		if (res < 0) {
			dev_err(&stat->client->dev, "%s : failed to update "
						"CTRL_REG3:0x%02x\n",
						__func__, fifomode);
			goto err_mutex_unlock;
		}
		enable_fifo_hw = false;
		stat->resume_state[RES_CTRL_REG3] =
			((I2_MASK & 0x00) |
			(~I2_MASK & stat->resume_state[RES_CTRL_REG3]));
		break;

	}
	if (recognized_mode) {
		res = l3gd20_gyr_update_fifomode(stat, fifomode);
		if (res < 0) {
			dev_err(&stat->client->dev, "%s : failed to "
						"set fifoMode\n", __func__);
			goto err_mutex_unlock;
		}
	}
	res = l3gd20_gyr_fifo_hwenable(stat, enable_fifo_hw);

err_mutex_unlock:

	return res;
}


static int l3gd20_gyr_update_fs_range(struct l3gd20_gyr_status *stat,
							u8 new_fs)
{
	int res ;
	u8 buf[2];

	u32 sensitivity;

	switch(new_fs) {
		case L3GD20_GYR_FS_250DPS:
			sensitivity = SENSITIVITY_250;
			break;
		case L3GD20_GYR_FS_500DPS:
			sensitivity = SENSITIVITY_500;
			break;
		case L3GD20_GYR_FS_2000DPS:
			sensitivity = SENSITIVITY_2000;
			break;
		default:
			dev_err(&stat->client->dev, "invalid g range "
						"requested: %u\n", new_fs);
			return -EINVAL;
	}


	buf[0] = CTRL_REG4;

	res = l3gd20_gyr_register_update(stat, buf, CTRL_REG4,
							FS_MASK, new_fs);

	if (res < 0) {
		dev_err(&stat->client->dev, "%s : failed to update fs:0x%02x\n",
							__func__, new_fs);
		return res;
	}
	stat->resume_state[RES_CTRL_REG4] =
		((FS_MASK & new_fs) |
		(~FS_MASK & stat->resume_state[RES_CTRL_REG4]));

	stat->sensitivity = sensitivity;
	return res;
}


static int l3gd20_gyr_update_odr(struct l3gd20_gyr_status *stat,
			unsigned int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	for (i = ARRAY_SIZE(odr_table) - 1; i >= 0; i--) {
		if ((odr_table[i].poll_rate_ms <= poll_interval_ms) || (i == 0))
			break;
	}

	config[1] = odr_table[i].mask;
	config[1] |= (ENABLE_ALL_AXES + PM_NORMAL);

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = l3gd20_gyr_i2c_write(stat, config, 1);
		if (err < 0)
			return err;
		stat->resume_state[RES_CTRL_REG1] = config[1];
		stat->ktime = ktime_set(0, MS_TO_NS(poll_interval_ms));
	}

	return err;
}

/* gyroscope data readout */
static int l3gd20_gyr_get_data(struct l3gd20_gyr_status *stat,
			     struct l3gd20_gyr_triple *data)
{
	int err;
	unsigned char gyro_out[6];
	/* y,p,r hardware data */
	s32 hw_d[3] = { 0 };

	gyro_out[0] = (AXISDATA_REG);

	err = l3gd20_gyr_i2c_read(stat, gyro_out, 6);

	if (err < 0)
		return err;

	hw_d[0] = (s32) ((s16)((gyro_out[1]) << 8) | gyro_out[0]);
	hw_d[1] = (s32) ((s16)((gyro_out[3]) << 8) | gyro_out[2]);
	hw_d[2] = (s32) ((s16)((gyro_out[5]) << 8) | gyro_out[4]);

	//hw_d[0] = hw_d[0] * stat->sensitivity;
	//hw_d[1] = hw_d[1] * stat->sensitivity;
	//hw_d[2] = hw_d[2] * stat->sensitivity;

	data->x = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	data->y = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	data->z = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));

	/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
	data->x = (data->x * 1148) / 1000;
	data->y = (data->y * 1148) / 1000;
	data->z = (data->z * 1148) / 1000;
	/* WH Lee, 20130307 */

	/* Detroit 3.0 CR #: XXX, WH Lee, 20121107 */
	stat->data.x = data->x;
	stat->data.y = data->y;
	stat->data.z = data->z;
	/* WH Lee, 20121107 */

#ifdef DEBUG
	/* dev_info(&stat->client->dev, "gyro_out: x = %d, y = %d, z = %d\n",
		data->x, data->y, data->z); */
#endif

	return err;
}

static void l3gd20_gyr_report_values(struct l3gd20_gyr_status *stat,
					struct l3gd20_gyr_triple *data)
{
	input_report_abs(stat->input_dev, ABS_X, data->x);
	input_report_abs(stat->input_dev, ABS_Y, data->y);
	input_report_abs(stat->input_dev, ABS_Z, data->z);
	input_sync(stat->input_dev);
}

static int l3gd20_gyr_hw_init(struct l3gd20_gyr_status *stat)
{
	int err;
	u8 buf[6];

	dev_info(&stat->client->dev, "hw init\n");

	buf[0] = (CTRL_REG1);
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	buf[2] = stat->resume_state[RES_CTRL_REG2];
	buf[3] = stat->resume_state[RES_CTRL_REG3];
	buf[4] = stat->resume_state[RES_CTRL_REG4];
	buf[5] = stat->resume_state[RES_CTRL_REG5];

	err = l3gd20_gyr_i2c_write(stat, buf, 5);
	if (err < 0)
		return err;

	buf[0] = (FIFO_CTRL_REG);
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = l3gd20_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
			return err;

	stat->hw_initialized = 1;

	return err;
}

static void l3gd20_gyr_device_power_off(struct l3gd20_gyr_status *stat)
{
	int err;
	u8 buf[2];

	dev_info(&stat->client->dev, "power off\n");

	buf[0] = (CTRL_REG1);
	buf[1] = (PM_OFF);
	err = l3gd20_gyr_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed\n");

	if (stat->pdata->power_off) {
		/* disable_irq_nosync(acc->irq1); */
		disable_irq_nosync(stat->irq2);
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}

	if (stat->hw_initialized) {
		/*if (stat->pdata->gpio_int1 >= 0)*/
		/*	disable_irq_nosync(stat->irq1);*/
		if (stat->pdata->gpio_int2 >= 0) {
			disable_irq_nosync(stat->irq2);
			dev_info(&stat->client->dev,
					"power off: irq2 disabled\n");
		}
		stat->hw_initialized = 0;
	}
}

static int l3gd20_gyr_device_power_on(struct l3gd20_gyr_status *stat)
{
	int err;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0)
			return err;
		if (stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}


	if (!stat->hw_initialized) {
		err = l3gd20_gyr_hw_init(stat);
		if (err < 0) {
			l3gd20_gyr_device_power_off(stat);
			return err;
		}
	}

	if (stat->hw_initialized) {
		/* if (stat->pdata->gpio_int1) {
			enable_irq(stat->irq1);
			dev_info(&stat->client->dev,
						"power on: irq1 enabled\n");
		} */
		dev_dbg(&stat->client->dev, "stat->pdata->gpio_int2 = %d\n",
						stat->pdata->gpio_int2);
		if (stat->pdata->gpio_int2 >= 0) {
			enable_irq(stat->irq2);
			dev_info(&stat->client->dev,
					"power on: irq2 enabled\n");
		}
	}

	return 0;
}

static int l3gd20_gyr_enable(struct l3gd20_gyr_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {

		err = l3gd20_gyr_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}

		if (stat->polling_enabled) {
			/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
			/* Add for cancel/stop timer */
			mutex_lock(&stat->lock);
			dev_info(&stat->client->dev,"[WH Lee] %s::hrtimer_start\n", __func__);
			atomic_set(&stat->timer_canceled, 0);
			/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
			/* sensord polling the data so we do not use timer */
			//hrtimer_start(&(stat->hr_timer), stat->ktime, HRTIMER_MODE_REL);
			/* WH Lee, 20130307 */
			mutex_unlock(&stat->lock);
			/* WH Lee, 20130218 */
		}

		/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
		err = pmlog_device_on(stat->pmlog_device);
		if (err)
			dev_err(&stat->client->dev, "%s: pmlog_device_on fail rc = %d\n", __func__,err);
#endif
		/* WH Lee, 20130107 */
	}

	return 0;
}

static int l3gd20_gyr_disable(struct l3gd20_gyr_status *stat)
{
	/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
	int err;
#endif
	/* WH Lee, 20130107 */

	dev_dbg(&stat->client->dev, "%s: stat->enabled = %d\n", __func__,
						atomic_read(&stat->enabled));

	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {

		l3gd20_gyr_device_power_off(stat);
		/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
		/* Add for cancel/stop timer */
		mutex_lock(&stat->lock);
		dev_info(&stat->client->dev,"[WH Lee] %s::hrtimer_cancel\n", __func__);
		atomic_set(&stat->timer_canceled, 1);
		/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
		/* sensord polling the data so we do not use timer */
		//hrtimer_cancel(&stat->hr_timer);
		/* WH Lee, 20130307 */
		dev_dbg(&stat->client->dev, "%s: cancel_hrtimer ", __func__);
		mutex_unlock(&stat->lock);
		/* WH Lee, 20130218 */
		/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
		err = pmlog_device_off(stat->pmlog_device);
		if (err)
			dev_err(&stat->client->dev, "%s: pmlog_device_off fail rc = %d\n", __func__,err);
#endif
		/* WH Lee, 20130107 */
	}
	return 0;
}

static ssize_t attr_polling_rate_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err;
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	mutex_lock(&stat->lock);
	err = l3gd20_gyr_update_odr(stat, interval_ms);
	if(err >= 0)
		stat->pdata->poll_interval = interval_ms;
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_range_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	int range = 0;
	u8 val;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range;

	switch (val) {
	case L3GD20_GYR_FS_250DPS:
		range = 250;
		break;
	case L3GD20_GYR_FS_500DPS:
		range = 500;
		break;
	case L3GD20_GYR_FS_2000DPS:
		range = 2000;
		break;
	}
	mutex_unlock(&stat->lock);
	/* return sprintf(buf, "0x%02x\n", val); */
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 250:
		range = L3GD20_GYR_FS_250DPS;
		break;
	case 500:
		range = L3GD20_GYR_FS_500DPS;
		break;
	case 2000:
		range = L3GD20_GYR_FS_2000DPS;
		break;
	default:
		dev_err(&stat->client->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = l3gd20_gyr_update_fs_range(stat, range);
	if (err >= 0)
		stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(&stat->client->dev, "range set to: %lu dps\n", val);
	return size;
}

static ssize_t attr_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		l3gd20_gyr_enable(stat);
	else
		l3gd20_gyr_disable(stat);

	return size;
}

static ssize_t attr_polling_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	if (stat->polling_enabled)
		val = 1;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	if (val) {
		stat->polling_enabled = true;
		l3gd20_gyr_manage_int2settings(stat, stat->fifomode);
		dev_info(dev, "polling mode enabled\n");
		if (atomic_read(&stat->enabled)) {
			dev_info(&stat->client->dev,"[WH Lee] %s::hrtimer_start\n", __func__);
			/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
			/* Add for cancel/stop timer */
			atomic_set(&stat->timer_canceled, 0);
			/* WH Lee, 20130218 */
			/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
			/* sensord polling the data so we do not use timer */
			//hrtimer_start(&(stat->hr_timer), stat->ktime, HRTIMER_MODE_REL);
			/* WH Lee, 20130307 */
		}
	} else {
		if (stat->polling_enabled) {
			dev_info(&stat->client->dev,"[WH Lee] %s::hrtimer_cancel\n", __func__);
			/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
			/* Add for cancel/stop timer */
			atomic_set(&stat->timer_canceled, 1);
			/* WH Lee, 20130218 */
			/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
			/* sensord polling the data so we do not use timer */
			//hrtimer_cancel(&stat->hr_timer);
			/* WH Lee, 20130307 */
		}
		stat->polling_enabled = false;
		l3gd20_gyr_manage_int2settings(stat, stat->fifomode);
		dev_info(dev, "polling mode disabled\n");
	}
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_watermark_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long watermark;
	int res;

	if (strict_strtoul(buf, 16, &watermark))
		return -EINVAL;

	res = l3gd20_gyr_update_watermark(stat, watermark);
	if (res < 0)
		return res;

	return size;
}

static ssize_t attr_watermark_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	int val = stat->watermark;
	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_fifomode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long fifomode;
	int res;

	if (strict_strtoul(buf, 16, &fifomode))
		return -EINVAL;
	/* if (!fifomode)
		return -EINVAL; */

	dev_dbg(dev, "%s, got value:0x%02x\n", __func__, (u8)fifomode);

	mutex_lock(&stat->lock);
	res = l3gd20_gyr_manage_int2settings(stat, (u8) fifomode);
	mutex_unlock(&stat->lock);

	if (res < 0)
		return res;
	return size;
}

static ssize_t attr_fifomode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	u8 val = stat->fifomode;
	return sprintf(buf, "0x%02x\n", val);
}

/* 8930 Boston_CR #:XXX, WH Lee, 20130305 */
static ssize_t attr_op_mode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	return size;
}

static ssize_t attr_op_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val = 0;
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_bandwidth_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	return size;
}

static ssize_t attr_bandwidth_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val = 0;
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_delay_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (data <= 0) {
		err = -EINVAL;
		return err;
	}

	mutex_lock(&stat->lock);
	err = l3gd20_gyr_update_odr(stat, data);
	if(err >= 0)
		stat->pdata->poll_interval = data;
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_delay_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}
/* WH Lee, 20130305 */

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = l3gd20_gyr_i2c_write(stat, x, 1);
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = l3gd20_gyr_i2c_read(stat, &data, 1);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct l3gd20_gyr_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);

	stat->reg_addr = val;

	mutex_unlock(&stat->lock);

	return size;
}
#endif /* DEBUG */

static struct device_attribute attributes[] = {
	/* 8930 Boston_CR #:XXX, WH Lee, 20130129 */
	/* Fix permission for CtsPermissionTest */
	__ATTR(pollrate_ms, 0660, attr_polling_rate_show,
						attr_polling_rate_store),
	__ATTR(range, 0660, attr_range_show, attr_range_store),
	__ATTR(enable_device, 0660, attr_enable_show, attr_enable_store),
	__ATTR(enable_polling, 0660, attr_polling_mode_show,
						attr_polling_mode_store),
	__ATTR(fifo_samples, 0660, attr_watermark_show, attr_watermark_store),
	__ATTR(fifo_mode, 0660, attr_fifomode_show, attr_fifomode_store),
	/* WH Lee, 20130129 */
	/* 8930 Boston_CR #:XXX, WH Lee, 20130305 */
	__ATTR(op_mode, 0660, attr_op_mode_show, attr_op_mode_store),
	__ATTR(bandwidth, 0660, attr_bandwidth_show, attr_bandwidth_store),
	__ATTR(enable, 0660, attr_enable_show, attr_enable_store),
	__ATTR(delay, 0660, attr_delay_show, attr_delay_store),
	/* WH Lee, 20130305 */
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

/* Detroit 3.0 CR #:XXX, WH Lee, 20121106 */
static ssize_t l3gd20_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);
	int err;
	u8 buffer[2];

	mutex_lock(&stat->lock);
	err = l3gd20_gyr_register_read(stat, buffer, WHO_AM_I);
	if (err < 0)
		dev_err(&stat->client->dev,
			"error reading fifo source reg\n");
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", buffer[0]);
}

static ssize_t l3gd20_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val = 0;
	return sprintf(buf, "%d\n", val);
}

static ssize_t l3gd20_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}

static ssize_t l3gd20_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);
	int count;

	/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
	/* Improve gyro-sensor performance */
	l3gd20_gyr_report_triple(stat);
	/* WH Lee, 20130307 */

	mutex_lock(&stat->lock);
	count = sprintf(buf, "%hd %hd %hd\n",
			stat->data.x,
			stat->data.y,
			stat->data.z);
	mutex_unlock(&stat->lock);

	return count;
}

static ssize_t l3gd20_show_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);
	int range = 0;
	u8 val;

	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range;

	switch (val) {
	case L3GD20_GYR_FS_250DPS:
		range = 250;
		break;
	case L3GD20_GYR_FS_500DPS:
		range = 500;
		break;
	case L3GD20_GYR_FS_2000DPS:
		range = 2000;
		break;
	}
	mutex_unlock(&stat->lock);
	/* return sprintf(buf, "0x%02x\n", val); */
	return sprintf(buf, "%d\n", range);
}

static ssize_t l3gd20_store_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long val;
	u8 range;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 250:
		range = L3GD20_GYR_FS_250DPS;
		break;
	case 500:
		range = L3GD20_GYR_FS_500DPS;
		break;
	case 2000:
		range = L3GD20_GYR_FS_2000DPS;
		break;
	default:
		dev_err(&stat->client->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = l3gd20_gyr_update_fs_range(stat, range);
	if (err >= 0)
		stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(&stat->client->dev, "range set to: %lu dps\n", val);
	return count;
}

static ssize_t l3gd20_show_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val = 0;
	return sprintf(buf, "%d\n", val);
}

static ssize_t l3gd20_store_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}

static ssize_t l3gd20_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t l3gd20_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (data)
		l3gd20_gyr_enable(stat);
	else
		l3gd20_gyr_disable(stat);

	return count;
}

static ssize_t l3gd20_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t l3gd20_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (data <= 0) {
		err = -EINVAL;
		return err;
	}

	mutex_lock(&stat->lock);
	err = l3gd20_gyr_update_odr(stat, data);
	if(err >= 0)
		stat->pdata->poll_interval = data;
	mutex_unlock(&stat->lock);
	return count;
}

static ssize_t l3gd20_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err, i;
	int val = 1;
	u8 buffer[2];
	u8 gyro_out[6];
	signed short xyz[3] = {0};
	signed short xyz_new[3] = {0};
	signed short temp_xyz[3] = {0};
	signed short xyz_result[3] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

  	//write initial configuration
	// [CTRL_REG2] = 0x0;
	buffer[0] = CTRL_REG2;
	buffer[1] = 0x0;
	err = l3gd20_gyr_i2c_write(stat, buffer, 1);
	if (err < 0)
	{
		dev_err(&stat->client->dev, "error i2c write\n");
		goto err_exit;
	}

	// [CTRL_REG3] = 0x0;
	buffer[0] = CTRL_REG3;
	buffer[1] = 0x0;
	err = l3gd20_gyr_i2c_write(stat, buffer, 1);
	if (err < 0)
	{
		dev_err(&stat->client->dev, "error i2c write\n");
		goto err_exit;
	}

	// [CTRL_REG4] = 0xA0;
	buffer[0] = CTRL_REG4;
	buffer[1] = 0xA0;
	err = l3gd20_gyr_i2c_write(stat, buffer, 1);
	if (err < 0)
	{
		dev_err(&stat->client->dev, "error i2c write\n");
		goto err_exit;
	}

	// [CTRL_REG1] = 0x0F;
	buffer[0] = CTRL_REG1;
	buffer[1] = 0x0F;
	err = l3gd20_gyr_i2c_write(stat, buffer, 1);
	if (err < 0)
	{
		dev_err(&stat->client->dev, "error i2c write\n");
		goto err_exit;
	}

        msleep(100);

	for(i = 0; i < 5; i++)
	{
		gyro_out[0] = (AXISDATA_REG);
		err = l3gd20_gyr_i2c_read(stat, gyro_out, 6);
		if (err < 0)
		{
			dev_err(&stat->client->dev, "error i2c read\n");
			goto err_exit;
		}
		temp_xyz[0] = (gyro_out[1] << 8) | gyro_out[0];
		temp_xyz[1] = (gyro_out[3] << 8) | gyro_out[2];
		temp_xyz[2] = (gyro_out[5] << 8) | gyro_out[4];
		xyz[0] += temp_xyz[0];
		xyz[1] += temp_xyz[1];
		xyz[2] += temp_xyz[2];
		printk("XH XL X = 0x%02x 0x%02x 0x%04x(%d)\n", gyro_out[1], gyro_out[0], temp_xyz[0], temp_xyz[0]);
		printk("YH YL Y = 0x%02x 0x%02x 0x%04x(%d)\n", gyro_out[3], gyro_out[2], temp_xyz[1], temp_xyz[1]);
		printk("ZH ZL Z = 0x%02x 0x%02x 0x%04x(%d)\n", gyro_out[5], gyro_out[4], temp_xyz[2], temp_xyz[2]);
	}

	xyz[0]/=5;
	xyz[1]/=5;
	xyz[2]/=5;

	printk("before enable self test\n");
	printk("X = %d\n", xyz[0]);
	printk("Y = %d\n", xyz[1]);
	printk("Z = %d\n", xyz[2]);

	// enable selftest
	// [CTRL_REG4] = 0x22;
	buffer[0] = CTRL_REG4;
	buffer[1] = 0x22;
	err = l3gd20_gyr_i2c_write(stat, buffer, 1);
	if (err < 0)
	{
		dev_err(&stat->client->dev, "error i2c write\n");
		goto err_exit;
	}

	msleep(100);

	for(i = 0; i < 5; i++)
	{
		gyro_out[0] = (AXISDATA_REG);
		err = l3gd20_gyr_i2c_read(stat, gyro_out, 6);
		if (err < 0)
		{
			dev_err(&stat->client->dev, "error i2c read\n");
			goto err_exit;
		}
		temp_xyz[0] = (gyro_out[1] << 8) | gyro_out[0];
		temp_xyz[1] = (gyro_out[3] << 8) | gyro_out[2];
		temp_xyz[2] = (gyro_out[5] << 8) | gyro_out[4];
		xyz_new[0] += temp_xyz[0];
		xyz_new[1] += temp_xyz[1];
		xyz_new[2] += temp_xyz[2];
		printk("XH XL X = 0x%02x 0x%02x 0x%04x(%d)\n", gyro_out[1], gyro_out[0], temp_xyz[0], temp_xyz[0]);
		printk("YH YL Y = 0x%02x 0x%02x 0x%04x(%d)\n", gyro_out[3], gyro_out[2], temp_xyz[1], temp_xyz[1]);
		printk("ZH ZL Z = 0x%02x 0x%02x 0x%04x(%d)\n", gyro_out[5], gyro_out[4], temp_xyz[2], temp_xyz[2]);

	}

	// [CTRL_REG4] = 0xA0;
	buffer[0] = CTRL_REG4;
	buffer[1] = 0xA0;
	err = l3gd20_gyr_i2c_write(stat, buffer, 1);
	if (err < 0)
	{
		dev_err(&stat->client->dev, "error i2c write\n");
		goto err_exit;
	}

	l3gd20_gyr_device_power_off(stat);

	stat->resume_state[RES_CTRL_REG1] = ALL_ZEROES | ENABLE_ALL_AXES
								| PM_NORMAL;
	stat->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES | BDU_ENABLE;
	stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;

	err = l3gd20_gyr_device_power_on(stat);
	if (err < 0) {
		dev_err(&stat->client->dev, "power on failed: %d\n", err);
		goto err_exit;
	}

	atomic_set(&stat->enabled, 1);

	err = l3gd20_gyr_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&stat->client->dev, "update_fs_range failed\n");
		goto err_exit;
	}

	err = l3gd20_gyr_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&stat->client->dev, "update_odr failed\n");
		goto err_exit;
	}

	l3gd20_gyr_device_power_off(stat);
	atomic_set(&stat->enabled, 0);

	xyz_new[0]/=5;
	xyz_new[1]/=5;
	xyz_new[2]/=5;

	printk("after enable self test\n");
	printk("X = %d\n", xyz_new[0]);
	printk("Y = %d\n", xyz_new[1]);
	printk("Z = %d\n", xyz_new[2]);

	xyz_result[0] = (abs(xyz_new[0] - xyz[0]) * 70)/1000;
	xyz_result[1] = (abs(xyz_new[1] - xyz[1]) * 70)/1000;
	xyz_result[2] = (abs(xyz_new[2] - xyz[2]) * 70)/1000;

	printk("X = %d, expect to %d to %d\n", xyz_result[0], MIN_X, MAX_X);
	printk("Y = %d, expect to %d to %d\n", xyz_result[1], MIN_Y, MAX_Y);
	printk("Z = %d, expect to %d to %d\n", xyz_result[2], MIN_Z, MAX_Z);

	if(  ((xyz_result[0] >= MIN_X) && (xyz_result[0] <= MAX_X))
		&& ((xyz_result[1] >= MIN_Y) && (xyz_result[1] <= MAX_Y))
		&& ((xyz_result[2] >= MIN_Z) && (xyz_result[2] <= MAX_Z)))
	{
		printk("gyro selftest -PASS\n");
		val = 0;
	}
	else
	{
		printk("gyro selftest - FAIL\n");
	}
err_exit:
	return sprintf(buf, "%d\n", val);
}

static ssize_t l3gd20_show_pollrate_ms(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t l3gd20_store_pollrate_ms(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (data <= 0) {
		err = -EINVAL;
		return err;
	}

	mutex_lock(&stat->lock);
	err = l3gd20_gyr_update_odr(stat, data);
	if(err >= 0)
		stat->pdata->poll_interval = data;
	mutex_unlock(&stat->lock);
	return count;
}

static ssize_t l3gd20_show_enable_device(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t l3gd20_store_enable_device(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (data)
		l3gd20_gyr_enable(stat);
	else
		l3gd20_gyr_disable(stat);

	return count;
}

static ssize_t l3gd20_show_enable_polling(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val = 0;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	mutex_lock(&stat->lock);
	if (stat->polling_enabled)
		val = 1;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t l3gd20_store_enable_polling(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	mutex_lock(&stat->lock);
	if (data) {
		stat->polling_enabled = true;
		l3gd20_gyr_manage_int2settings(stat, stat->fifomode);
		dev_info(dev, "polling mode enabled\n");
		if (atomic_read(&stat->enabled)) {
			dev_info(&stat->client->dev,"[WH Lee] %s::hrtimer_start\n", __func__);
			/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
			/* Add for cancel/stop timer */
			atomic_set(&stat->timer_canceled, 0);
			/* WH Lee, 20130218 */
			/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
			/* sensord polling the data so we do not use timer */
			//hrtimer_start(&(stat->hr_timer), stat->ktime, HRTIMER_MODE_REL);
			/* WH Lee, 20130307 */
		}
	} else {
		if (stat->polling_enabled) {
			dev_info(&stat->client->dev,"[WH Lee] %s::hrtimer_cancel\n", __func__);
			/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
			/* Add for cancel/stop timer */
			atomic_set(&stat->timer_canceled, 1);
			/* WH Lee, 20130218 */
			/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
			/* sensord polling the data so we do not use timer */
			//hrtimer_cancel(&stat->hr_timer);
			/* WH Lee, 20130307 */
		}
		stat->polling_enabled = false;
		l3gd20_gyr_manage_int2settings(stat, stat->fifomode);
		dev_info(dev, "polling mode disabled\n");
	}
	mutex_unlock(&stat->lock);
	return count;
}

static ssize_t l3gd20_show_fifo_samples(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	int val = stat->watermark;
	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t l3gd20_store_fifo_samples(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 16, &data);
	if (err)
		return err;

	err = l3gd20_gyr_update_watermark(stat, data);
	if (err < 0)
		return err;

	return count;
}

static ssize_t l3gd20_show_fifo_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	u8 val = stat->fifomode;
	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t l3gd20_store_fifo_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 16, &data);
	if (err)
		return err;

	dev_dbg(dev, "%s, got value:0x%02x\n", __func__, (u8)data);

	mutex_lock(&stat->lock);
	err = l3gd20_gyr_manage_int2settings(stat, (u8) data);
	mutex_unlock(&stat->lock);

	if (err < 0)
		return err;
	return count;
}

#ifdef DEBUG
static ssize_t l3gd20_show_reg_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	u8 data;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	err = l3gd20_gyr_i2c_read(stat, &data, 1);
	if (err < 0)
		return err;
	return sprintf(buf, "0x%02x\n", data);
}

static ssize_t l3gd20_store_reg_value(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	u8 x[2];
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 16, &data);
	if (err)
		return err;

	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = data;
	err = l3gd20_gyr_i2c_write(stat, x, 1);
	if (err < 0)
		return err;
	return count;

}

static ssize_t l3gd20_store_reg_addr(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);

	err = strict_strtoul(buf, 16, &data);
	if (err)
		return err;

	mutex_lock(&stat->lock);
	stat->reg_addr = data;
	mutex_unlock(&stat->lock);
	return count;
}
#endif

static DEVICE_ATTR(chip_id, S_IRUGO,
		l3gd20_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR,
		l3gd20_show_op_mode, l3gd20_store_op_mode);
static DEVICE_ATTR(value, S_IRUGO,
		l3gd20_show_value, NULL);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR,
		l3gd20_show_range, l3gd20_store_range);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR,
		l3gd20_show_bandwidth, l3gd20_store_bandwidth);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR,
		l3gd20_show_enable, l3gd20_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR,
		l3gd20_show_delay, l3gd20_store_delay);
/*
static DEVICE_ATTR(fastoffset_en, S_IRUGO|S_IWUSR,
		NULL, bmg_store_fastoffset_en);
static DEVICE_ATTR(slowoffset_en, S_IRUGO|S_IWUSR,
		NULL, bmg_store_slowoffset_en);
static DEVICE_ATTR(sleepdur, S_IRUGO|S_IWUSR,
		bmg_show_sleepdur, bmg_store_sleepdur);
static DEVICE_ATTR(autosleepdur, S_IRUGO|S_IWUSR,
		bmg_show_autosleepdur, bmg_store_autosleepdur);
*/
static DEVICE_ATTR(selftest, S_IRUGO,
		l3gd20_show_selftest, NULL);
/* 8930 Boston_CR #:XXX, WH Lee, 20130129 */
/* Fix permission for CtsPermissionTest */
static DEVICE_ATTR(pollrate_ms, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		l3gd20_show_pollrate_ms, l3gd20_store_pollrate_ms);
static DEVICE_ATTR(enable_device, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		l3gd20_show_enable_device, l3gd20_store_enable_device);
static DEVICE_ATTR(enable_polling, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		l3gd20_show_enable_polling, l3gd20_store_enable_polling);
static DEVICE_ATTR(fifo_samples, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		l3gd20_show_fifo_samples, l3gd20_store_fifo_samples);
static DEVICE_ATTR(fifo_mode, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		l3gd20_show_fifo_mode, l3gd20_store_fifo_mode);
/* WH Lee, 20130129 */
#ifdef DEBUG
static DEVICE_ATTR(reg_value, S_IRUSR|S_IWUSR,
		l3gd20_show_reg_value, l3gd20_store_reg_value);
static DEVICE_ATTR(reg_addr, S_IRUSR,
		NULL, l3gd20_store_reg_addr);
#endif

static struct attribute *l3gd20_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
//	&dev_attr_fastoffset_en.attr,
//	&dev_attr_slowoffset_en.attr,
	&dev_attr_selftest.attr,
//	&dev_attr_sleepdur.attr,
//	&dev_attr_autosleepdur.attr,
	&dev_attr_pollrate_ms.attr,
	&dev_attr_enable_device.attr,
	&dev_attr_enable_polling.attr,
	&dev_attr_fifo_samples.attr,
	&dev_attr_fifo_mode.attr,
#ifdef DEBUG
	&dev_attr_reg_value.attr,
	&dev_attr_reg_addr.attr,
#endif
	NULL
};

static struct attribute_group l3gd20_attribute_group = {
	.attrs = l3gd20_attributes
};
/* WH Lee, 20121106 */

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void l3gd20_gyr_report_triple(struct l3gd20_gyr_status *stat)
{
	int err;
	struct l3gd20_gyr_triple data_out;

	err = l3gd20_gyr_get_data(stat, &data_out);
	if (err < 0)
		dev_err(&stat->client->dev, "get_gyroscope_data failed\n");
	else
		l3gd20_gyr_report_values(stat, &data_out);
}


static void l3gd20_gyr_irq2_fifo(struct l3gd20_gyr_status *stat)
{
	int err;
	u8 buf[2];
	u8 int_source;
	u8 samples;
	u8 workingmode;
	u8 stored_samples;

	mutex_lock(&stat->lock);

	workingmode = stat->fifomode;


	dev_dbg(&stat->client->dev, "%s : fifomode:0x%02x\n", __func__,
								workingmode);


	switch (workingmode) {
	case FIFO_MODE_BYPASS:
	{
		dev_dbg(&stat->client->dev, "%s : fifomode:0x%02x\n", __func__,
							stat->fifomode);
		l3gd20_gyr_report_triple(stat);
		break;
	}
	case FIFO_MODE_FIFO:
		samples = (stat->watermark)+1;
		dev_dbg(&stat->client->dev,
			"%s : FIFO_SRC_REG init samples:%d\n",
							__func__, samples);
		err = l3gd20_gyr_register_read(stat, buf, FIFO_SRC_REG);
		if (err < 0)
			dev_err(&stat->client->dev,
					"error reading fifo source reg\n");

		int_source = buf[0];
		dev_dbg(&stat->client->dev, "%s :FIFO_SRC_REG content:0x%02x\n",
							__func__, int_source);

		stored_samples = int_source & FIFO_STORED_DATA_MASK;
		dev_dbg(&stat->client->dev, "%s : fifomode:0x%02x\n", __func__,
						stat->fifomode);

		dev_dbg(&stat->client->dev, "%s : samples:%d stored:%d\n",
				__func__, samples, stored_samples);

		for (; samples > 0; samples--) {
#ifdef DEBUG
			input_report_abs(stat->input_dev, ABS_MISC, 1);
			input_sync(stat->input_dev);
#endif
			dev_dbg(&stat->client->dev, "%s : current sample:%d\n",
							__func__, samples);

			l3gd20_gyr_report_triple(stat);

#ifdef DEBUG
			input_report_abs(stat->input_dev, ABS_MISC, 0);
			input_sync(stat->input_dev);
#endif
		}
		l3gd20_gyr_fifo_reset(stat);
		break;
	}
#ifdef DEBUG
	input_report_abs(stat->input_dev, ABS_MISC, 3);
	input_sync(stat->input_dev);
#endif

	mutex_unlock(&stat->lock);
}

static irqreturn_t l3gd20_gyr_isr2(int irq, void *dev)
{
	struct l3gd20_gyr_status *stat = dev;

	disable_irq_nosync(irq);
#ifdef DEBUG
	input_report_abs(stat->input_dev, ABS_MISC, 2);
	input_sync(stat->input_dev->input);
#endif
	queue_work(stat->irq2_work_queue, &stat->irq2_work);
	pr_debug("%s %s: isr2 queued\n", L3GD20_GYR_DEV_NAME, __func__);

	return IRQ_HANDLED;
}

static void l3gd20_gyr_irq2_work_func(struct work_struct *work)
{

	struct l3gd20_gyr_status *stat =
		container_of(work, struct l3gd20_gyr_status, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:l3gd20_gyr_irq2_XXX(stat); */
	l3gd20_gyr_irq2_fifo(stat);
	/*  */
	pr_debug("%s %s: IRQ2 served\n", L3GD20_GYR_DEV_NAME, __func__);
/* exit: */
	enable_irq(stat->irq2);
}

/* Boston CR #:XXX, WH Lee, 20121211 */
/* Do not implement input device open/close function */
/*
int l3gd20_gyr_input_open(struct input_dev *input)
{
	struct l3gd20_gyr_status *stat = input_get_drvdata(input);
	dev_dbg(&stat->client->dev, "%s\n", __func__);
	return l3gd20_gyr_enable(stat);
}

void l3gd20_gyr_input_close(struct input_dev *dev)
{
	struct l3gd20_gyr_status *stat = input_get_drvdata(dev);
	dev_dbg(&stat->client->dev, "%s\n", __func__);
	l3gd20_gyr_disable(stat);
}
*/
/* WH Lee, 20121211 */

static int l3gd20_gyr_validate_pdata(struct l3gd20_gyr_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int) L3GD20_GYR_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
	    stat->pdata->axis_map_y > 2 ||
	    stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			stat->pdata->axis_map_x,
			stat->pdata->axis_map_y,
			stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 ||
	    stat->pdata->negate_y > 1 ||
	    stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			stat->pdata->negate_x,
			stat->pdata->negate_y,
			stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev,
			"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}

static int l3gd20_gyr_input_init(struct l3gd20_gyr_status *stat)
{
	int err = -1;

	dev_dbg(&stat->client->dev, "%s\n", __func__);

	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev,
			"input device allocation failed\n");
		goto err0;
	}

	/* Boston CR #:XXX, WH Lee, 20121211 */
	/* Do not implement input device open/close function */
	/*
	stat->input_dev->open = l3gd20_gyr_input_open;
	stat->input_dev->close = l3gd20_gyr_input_close;
	*/
	/* WH Lee, 20121211 */
	stat->input_dev->name = L3GD20_GYR_DEV_NAME;

	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);

#ifdef DEBUG
	set_bit(EV_KEY, stat->input_dev->keybit);
	set_bit(KEY_LEFT, stat->input_dev->keybit);
	input_set_abs_params(stat->input_dev, ABS_MISC, 0, 1, 0, 0);
#endif

	input_set_abs_params(stat->input_dev, ABS_X, -FS_MAX-1, FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Y, -FS_MAX-1, FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_Z, -FS_MAX-1, FS_MAX, 0, 0);


	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
			"unable to register input polled device %s\n",
			stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void l3gd20_gyr_input_cleanup(struct l3gd20_gyr_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static void poll_function_work(struct work_struct *polling_task)
{
	struct l3gd20_gyr_status *stat;
	struct l3gd20_gyr_triple data_out;
	int err;

	stat = container_of((struct work_struct *)polling_task,
					struct l3gd20_gyr_status, polling_task);

	err = l3gd20_gyr_get_data(stat, &data_out);
	if (err < 0)
		dev_err(&stat->client->dev, "get_rotation_data failed.\n");
	else
		l3gd20_gyr_report_values(stat, &data_out);

	/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
	/* sensord polling the data so we do not use timer */
	#if 0
	/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
	/* Add for cancel/stop timer */
	mutex_lock(&stat->lock);
	//dev_info(&stat->client->dev,"[WH Lee] %s::hrtimer_start\n", __func__);
	if (!atomic_read(&stat->timer_canceled))
		hrtimer_start(&stat->hr_timer, stat->ktime, HRTIMER_MODE_REL);
	mutex_unlock(&stat->lock);
	/* WH Lee, 20130218 */
	#endif
	/* WH Lee, 20130307 */
}

enum hrtimer_restart poll_function_read(struct hrtimer *timer)
{
	struct l3gd20_gyr_status *stat;

	stat = container_of((struct hrtimer *)timer,
				struct l3gd20_gyr_status, hr_timer);

	queue_work(l3gd20_gyr_workqueue, &stat->polling_task);
	return HRTIMER_NORESTART;
}

static int l3gd20_gyr_probe(struct i2c_client *client,
					const struct i2c_device_id *devid)
{
	struct l3gd20_gyr_status *stat;

	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK ;

	int err = -1;

	dev_info(&client->dev, "probe start.\n");


	stat = kzalloc(sizeof(*stat), GFP_KERNEL);
	if (stat == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}


	/* Support for both I2C and SMBUS adapter interfaces. */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			stat->use_smbus = 0;
			goto err0;
		}
	}

	if(l3gd20_gyr_workqueue == 0)
		l3gd20_gyr_workqueue = create_workqueue("l3gd20_gyr_workqueue");

	hrtimer_init(&stat->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer.function = &poll_function_read;

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);
	stat->client = client;

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err1;
	}

	if (client->dev.platform_data == NULL) {
		default_l3gd20_gyr_pdata.gpio_int1 = int1_gpio;
		default_l3gd20_gyr_pdata.gpio_int2 = int2_gpio;
		memcpy(stat->pdata, &default_l3gd20_gyr_pdata,
							sizeof(*stat->pdata));
		dev_info(&client->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, client->dev.platform_data,
						sizeof(*stat->pdata));
	}

	err = l3gd20_gyr_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, stat);

	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err1_1;
		}
	}


	/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
	stat->pmlog_device = pmlog_register_device(&client->dev);
#endif
	/* WH Lee, 20130107 */

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL_REG1] = ALL_ZEROES | ENABLE_ALL_AXES
								| PM_NORMAL;
	stat->resume_state[RES_CTRL_REG2] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES | BDU_ENABLE;
	stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;

	stat->polling_enabled = true;
	dev_info(&client->dev, "polling mode enabled\n");

	err = l3gd20_gyr_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&stat->enabled, 1);

	err = l3gd20_gyr_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto err2;
	}

	err = l3gd20_gyr_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = l3gd20_gyr_input_init(stat);
	if (err < 0)
		goto err3;

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
			"%s device register failed\n", L3GD20_GYR_DEV_NAME);
		goto err4;
	}

	/* Detroit3.0 CR #:XXX, WH Lee, 20121106 */
	err = sysfs_create_group(&stat->input_dev->dev.kobj,
			&l3gd20_attribute_group);
	/* WH Lee, 20121106 */

	l3gd20_gyr_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);
	/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
	/* Add for cancel/stop timer */
	atomic_set(&stat->timer_canceled, 0);
	/* WH Lee, 20130218 */


	if (stat->pdata->gpio_int2 >= 0) {
		stat->irq2 = gpio_to_irq(stat->pdata->gpio_int2);
		dev_info(&client->dev, "%s: %s has set irq2 to irq:"
						" %d mapped on gpio:%d\n",
			L3GD20_GYR_DEV_NAME, __func__, stat->irq2,
							stat->pdata->gpio_int2);

		INIT_WORK(&stat->irq2_work, l3gd20_gyr_irq2_work_func);
		stat->irq2_work_queue =
			create_singlethread_workqueue("l3gd20_gyr_irq2_wq");
		if (!stat->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev, "cannot create "
						"work queue2: %d\n", err);
			goto err5;
		}

		err = request_irq(stat->irq2, l3gd20_gyr_isr2,
				IRQF_TRIGGER_HIGH, "l3gd20_gyr_irq2", stat);

		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err6;
		}
		disable_irq_nosync(stat->irq2);
	}

	mutex_unlock(&stat->lock);

	INIT_WORK(&stat->polling_task, poll_function_work);
	dev_info(&client->dev, "%s probed: device created successfully\n",
							L3GD20_GYR_DEV_NAME);


	return 0;

/*err7:
	free_irq(stat->irq2, stat);
*/
err6:
	destroy_workqueue(stat->irq2_work_queue);
err5:
	l3gd20_gyr_device_power_off(stat);
	remove_sysfs_interfaces(&client->dev);
err4:
	l3gd20_gyr_input_cleanup(stat);
err3:
	l3gd20_gyr_device_power_off(stat);
err2:
	/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
	if (stat->pmlog_device)
		pmlog_unregister_device(stat->pmlog_device);
#endif
	/* WH Lee, 20130107 */
	if (stat->pdata->exit)
		stat->pdata->exit();
err1_1:
	mutex_unlock(&stat->lock);
	kfree(stat->pdata);
err1:
	destroy_workqueue(l3gd20_gyr_workqueue);
	kfree(stat);
err0:
		pr_err("%s: Driver Initialization failed\n",
							L3GD20_GYR_DEV_NAME);
		return err;
}

static int l3gd20_gyr_remove(struct i2c_client *client)
{
	struct l3gd20_gyr_status *stat = i2c_get_clientdata(client);

	dev_info(&stat->client->dev, "driver removing\n");

	cancel_work_sync(&stat->polling_task);
	if(!l3gd20_gyr_workqueue) {
		flush_workqueue(l3gd20_gyr_workqueue);
		destroy_workqueue(l3gd20_gyr_workqueue);
	}
	/*
	if (stat->pdata->gpio_int1 >= 0)
	{
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
		destroy_workqueue(stat->irq1_work_queue);
	}
	*/
	if (stat->pdata->gpio_int2 >= 0) {
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
	if (stat->pmlog_device)
		pmlog_unregister_device(stat->pmlog_device);
#endif
	/* WH Lee, 20130107 */

	l3gd20_gyr_disable(stat);
	l3gd20_gyr_input_cleanup(stat);

	remove_sysfs_interfaces(&client->dev);

	kfree(stat->pdata);
	kfree(stat);
	return 0;
}

static int l3gd20_gyr_suspend(struct device *dev)
{
	int err = 0;
/* 8930 Boston CR #:XXX, WH Lee, 20130219 */
/* Set power off mode for power consumption */
//#define SLEEP
/* WH Lee, 20130219 */
#ifdef CONFIG_PM
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_gyr_status *stat = i2c_get_clientdata(client);
	u8 buf[2];

	dev_info(&client->dev, "suspend\n");

	dev_dbg(&client->dev, "%s\n", __func__);
	if (atomic_read(&stat->enabled)) {
		mutex_lock(&stat->lock);
		if (stat->polling_enabled) {
			dev_info(&stat->client->dev, "polling mode disabled\n");
			dev_info(&stat->client->dev,"[WH Lee] %s::hrtimer_cancel\n", __func__);
			/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
			/* Add for cancel/stop timer */
			atomic_set(&stat->timer_canceled, 1);
			/* WH Lee, 20130218 */
			/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
			/* sensord polling the data so we do not use timer */
			//hrtimer_cancel(&stat->hr_timer);
			/* WH Lee, 20130307 */
		}
#ifdef SLEEP
		err = l3gd20_gyr_register_update(stat, buf, CTRL_REG1,
				0x0F, (ENABLE_NO_AXES | PM_NORMAL));
#else
		err = l3gd20_gyr_register_update(stat, buf, CTRL_REG1,
				0x08, PM_OFF);
#endif /*SLEEP*/
		mutex_unlock(&stat->lock);
	}
#endif /*CONFIG_PM*/
	return err;
}

static int l3gd20_gyr_resume(struct device *dev)
{
	int err = 0;
#ifdef CONFIG_PM
	struct i2c_client *client = to_i2c_client(dev);
	struct l3gd20_gyr_status *stat = i2c_get_clientdata(client);
	u8 buf[2];


	dev_info(&client->dev, "resume\n");

	dev_dbg(&client->dev, "%s\n", __func__);
	if (atomic_read(&stat->enabled)) {
		mutex_lock(&stat->lock);
		if (stat->polling_enabled) {
			dev_info(&stat->client->dev, "polling mode enabled\n");
			dev_info(&stat->client->dev,"[WH Lee] %s::hrtimer_start\n", __func__);
			/* 8930 Boston CR #:XXX, WH Lee, 20130218 */
			/* Add for cancel/stop timer */
			//hrtimer_init(&stat->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			atomic_set(&stat->timer_canceled, 0);
			/* 8930 Boston CR #:XXX, WH Lee, 20130307 */
			/* sensord polling the data so we do not use timer */
			//hrtimer_start(&stat->hr_timer, stat->ktime, HRTIMER_MODE_REL);
			/* WH Lee, 20130307 */
			/* WH Lee, 20130218 */
		}
#ifdef SLEEP
		err = l3gd20_gyr_register_update(stat, buf, CTRL_REG1,
				0x0F, (ENABLE_ALL_AXES | PM_NORMAL));
#else
		err = l3gd20_gyr_register_update(stat, buf, CTRL_REG1,
				0x08, PM_NORMAL);
#endif
		mutex_unlock(&stat->lock);

	}
#endif /*CONFIG_PM*/
	return err;
}


static const struct i2c_device_id l3gd20_gyr_id[] = {
	{ L3GD20_GYR_DEV_NAME , 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, l3gd20_gyr_id);

static const struct dev_pm_ops l3gd20_gyr_pm = {
	.suspend = l3gd20_gyr_suspend,
	.resume = l3gd20_gyr_resume,
};

static struct i2c_driver l3gd20_gyr_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = L3GD20_GYR_DEV_NAME,
			.pm = &l3gd20_gyr_pm,
	},
	.probe = l3gd20_gyr_probe,
	.remove = __devexit_p(l3gd20_gyr_remove),
	.id_table = l3gd20_gyr_id,

};

static int __init l3gd20_gyr_init(void)
{
	/* 8930 Boston_CR #:XXX, WH Lee, 20130115 */
	/* Add BootLog */
	int ret;
	/* WH Lee, 20130115 */

	pr_info("%s: gyroscope sysfs driver init\n", L3GD20_GYR_DEV_NAME);

	/* 8930 Boston_CR #:XXX, WH Lee, 20130115 */
	/* Add BootLog */
	printk("BootLog, +%s\n", __func__);
	ret = i2c_add_driver(&l3gd20_gyr_driver);
	printk("BootLog, -%s, ret=%d\n", __func__,ret);
	return ret;
	/* Original code */
	//return i2c_add_driver(&l3gd20_gyr_driver);
	/* WH Lee, 20130115 */
}

static void __exit l3gd20_gyr_exit(void)
{

	pr_info("%s exit\n", L3GD20_GYR_DEV_NAME);

	i2c_del_driver(&l3gd20_gyr_driver);
	return;
}

/* 8930 Boston_CR #:XXX, WH Lee, 20130322 */
/* init it after touch driver */
late_initcall(l3gd20_gyr_init);
/* Original code */
//module_init(l3gd20_gyr_init);
/* WH Lee, 20130322 */
module_exit(l3gd20_gyr_exit);

MODULE_DESCRIPTION("l3gd20 gyroscope driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");
