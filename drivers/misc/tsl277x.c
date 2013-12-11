/* drivers/staging/taos/tsl277x.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/i2c/tsl2772.h>
#include <linux/module.h>

/* 8930 Boston_CR #:810, WH Lee, 20130314 */
/* Add wakelock before report p-sensor data */
#include <linux/wakelock.h>
/* WH Lee, 20130314 */

#include <mach/gpio.h>
#include <mach/irqs.h>

/* 8930 Boston_CR #:XXX, WH Lee, 20121226 */
/* Always return far in EVT1 */
#include <mach/hwid.h>
/* WH Lee, 20121226 */

/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif
/* WH Lee, 20130107 */

/* 8930 Boston_CR #:XXX, WH Lee, 20130424 */
#include <linux/debugfs.h>
static unsigned int TSL277X_PSEN_LOG_DLL=0;
static unsigned int TSL277X_LSEN_LOG_DLL=0;
extern struct dentry *kernel_debuglevel_dir;
#define TSL277X_PSEN_PRINTK(level, args...) if(level <= TSL277X_PSEN_LOG_DLL) printk( "[TSL277X_P]" args);
#define TSL277X_LSEN_PRINTK(level, args...) if(level <= TSL277X_LSEN_LOG_DLL) printk( "[TSL277X_L]" args);
/* WH Lee, 20130424 */

enum tsl277x_regs {
	TSL277X_ENABLE,
	TSL277X_ALS_TIME,
	TSL277X_PRX_TIME,
	TSL277X_WAIT_TIME,
	TSL277X_ALS_MINTHRESHLO,
	TSL277X_ALS_MINTHRESHHI,
	TSL277X_ALS_MAXTHRESHLO,
	TSL277X_ALS_MAXTHRESHHI,
	TSL277X_PRX_MINTHRESHLO,
	TSL277X_PRX_MINTHRESHHI,
	TSL277X_PRX_MAXTHRESHLO,
	TSL277X_PRX_MAXTHRESHHI,
	TSL277X_PERSISTENCE,
	TSL277X_CONFIG,
	TSL277X_PRX_PULSE_COUNT,
	TSL277X_CONTROL,

	TSL277X_REVID = 0x11,
	TSL277X_CHIPID,
	TSL277X_STATUS,
	TSL277X_ALS_CHAN0LO,
	TSL277X_ALS_CHAN0HI,
	TSL277X_ALS_CHAN1LO,
	TSL277X_ALS_CHAN1HI,
	TSL277X_PRX_LO,
	TSL277X_PRX_HI,

	TSL277X_REG_PRX_OFFS = 0x1e,
	TSL277X_REG_MAX,
};

enum tsl277x_cmd_reg {
	TSL277X_CMD_REG           = (1 << 7),
	TSL277X_CMD_INCR          = (0x1 << 5),
	TSL277X_CMD_SPL_FN        = (0x3 << 5),
	TSL277X_CMD_PROX_INT_CLR  = (0x5 << 0),
	TSL277X_CMD_ALS_INT_CLR   = (0x6 << 0),
};

enum tsl277x_en_reg {
	TSL277X_EN_PWR_ON   = (1 << 0),
	TSL277X_EN_ALS      = (1 << 1),
	TSL277X_EN_PRX      = (1 << 2),
	TSL277X_EN_WAIT     = (1 << 3),
	TSL277X_EN_ALS_IRQ  = (1 << 4),
	TSL277X_EN_PRX_IRQ  = (1 << 5),
	TSL277X_EN_SAI      = (1 << 6),
};

enum tsl277x_status {
	TSL277X_ST_ALS_VALID  = (1 << 0),
	TSL277X_ST_PRX_VALID  = (1 << 1),
	TSL277X_ST_ALS_IRQ    = (1 << 4),
	TSL277X_ST_PRX_IRQ    = (1 << 5),
	TSL277X_ST_PRX_SAT    = (1 << 6),
};

enum {
	TSL277X_ALS_GAIN_MASK = (3 << 0),
	TSL277X_ALS_AGL_MASK  = (1 << 2),
	TSL277X_ALS_AGL_SHIFT = 2,
	TSL277X_ATIME_PER_100 = 273,
	TSL277X_ATIME_DEFAULT_MS = 50,
	SCALE_SHIFT = 11,
	RATIO_SHIFT = 10,
	MAX_ALS_VALUE = 0xffff,
	MIN_ALS_VALUE = 10,
	GAIN_SWITCH_LEVEL = 100,
	GAIN_AUTO_INIT_VALUE = 16,
};

static u8 const tsl277x_ids[] = {

//針對不同料號使用對應的ID
	0x39,  //TSL27721 OR TSL27725
	0x30,  //TSL27723 OR TSL27727

	//0x20,	//TSL27711 OR TSL27715
	0x29,	//TSL27713 OR TSL27717

	//0x20,	//TSL27711 OR TSL27715 OR TMD27711 OR TMD26711
	//0x29,	//TSL27713 OR TSL27717 OR TMD27713 OR TMD26713

	//0x00,	//TSL26711 OR TSL26715
	//0x09,	//TSL26713 OR TSL26717

	//0x04,	//TSL25711 & TSL25715
	//0x0D,	//TSL25713 & TSL25717

////////////////////////

};

static char const *tsl277x_names[] = {
	"tsl27721 / tsl27725",
	"tsl27723 / tsl2777",
	"tsl27713 / tsl27717",
};

static u8 const restorable_regs[] = {
	TSL277X_ALS_TIME,
	TSL277X_PRX_TIME,
	TSL277X_WAIT_TIME,
	TSL277X_PERSISTENCE,
	TSL277X_CONFIG,
	TSL277X_PRX_PULSE_COUNT,
	TSL277X_CONTROL,
	TSL277X_REG_PRX_OFFS,
};

static u8 const als_gains[] = {
	1,
	8,
	16,
	120
};

struct taos_als_info {
	int ch0;
	int ch1;
	u32 cpl;
	u32 saturation;
	int lux;
	int ga;
	/* Boston CR #:XXX, WH Lee, 20121203 */
	int cover_ratio;
	/* WH Lee, 20121203 */
	/* Sapporo CR #:XXX, WH Lee, 20131008 */
	/* Fix lux of fluorescent lamp is too small */
	int ratio_index;
	/* WH Lee, 20131008 */
};

struct taos_prox_info {
	int raw;
	int detected;
	/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
	/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
	int curr_far;
	int curr_near;
	int last_status;
	int new_status;
	/* 8930 Boston_CR #:1670, WH Lee, 20130411 */
	/* Improve black card 0 cm issue*/
	int min_raw;
	/* WH Lee, 20130411 */
#endif
	/* WH Lee, 20130123 */
};

static struct lux_segment segment_default[] = {
	{
		.ratio = (435 << RATIO_SHIFT) / 1000,
		.k0 = (46516 << SCALE_SHIFT) / 1000,
		.k1 = (95381 << SCALE_SHIFT) / 1000,
	},
	{
		.ratio = (551 << RATIO_SHIFT) / 1000,
		.k0 = (23740 << SCALE_SHIFT) / 1000,
		.k1 = (43044 << SCALE_SHIFT) / 1000,
	},
};

struct tsl2772_chip {
	struct mutex lock;
	struct i2c_client *client;
	struct taos_prox_info prx_inf;
	struct taos_als_info als_inf;
	struct taos_parameters params;
	struct tsl2772_i2c_platform_data *pdata;
	u8 shadow[TSL277X_REG_MAX];
	struct input_dev *p_idev;
	struct input_dev *a_idev;
	int in_suspend;
	int wake_irq;
	int irq_pending;
	bool unpowered;
	bool als_enabled;
	bool prx_enabled;
	struct lux_segment *segment;
	int segment_num;
	int seg_num_max;
	bool als_gain_auto;
	/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
	/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
	bool prx_calib_auto;
#endif
	/* WH Lee, 20130123 */
	/* Detroit3.0 CR #:XXX, WH Lee, 2012113 */
	u8 chip_id;
	/* WH Lee, 20121123 */
	/* 8930 Boston CR #:XXX, WH Lee, 20130219 */
	/* Fix for power consumption */
	bool als_enabled_resume;
	bool prx_enabled_resume;
	/* WH Lee, 20130219 */
	/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
	struct pmlog_device *als_pmlog_device;
	struct pmlog_device *prx_pmlog_device;
#endif
	/* WH Lee, 20130107 */
	/* 8930 Boston_CR #:810, WH Lee, 20130314 */
	/* Add wakelock before report p-sensor data */
	struct wake_lock prox_wake_lock;
	/* WH Lee, 20130314 */
};

static int taos_i2c_read(struct tsl2772_chip *chip, u8 reg, u8 *val)
{
	int ret;
	s32 read;
	struct i2c_client *client = chip->client;

	ret = i2c_smbus_write_byte(client, (TSL277X_CMD_REG | reg));
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to write register %x\n",
				__func__, reg);
		return ret;
	}
	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		dev_err(&client->dev, "%s: failed to read from register %x\n",
				__func__, reg);
		return ret;
	}
	*val = read;
	return 0;
}

static int taos_i2c_blk_read(struct tsl2772_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	ret =  i2c_smbus_read_i2c_block_data(client,
			TSL277X_CMD_REG | TSL277X_CMD_INCR | reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
	return ret;
}

static int taos_i2c_write(struct tsl2772_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;

	ret = i2c_smbus_write_byte_data(client, TSL277X_CMD_REG | reg, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed to write register %x\n",
				__func__, reg);
	return ret;
}

static int taos_i2c_blk_write(struct tsl2772_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	ret =  i2c_smbus_write_i2c_block_data(client,
			TSL277X_CMD_REG | TSL277X_CMD_INCR | reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
	return ret;
}

static int set_segment_table(struct tsl2772_chip *chip,
		struct lux_segment *segment, int seg_num)
{
	int i;
	struct device *dev = &chip->client->dev;

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);

	if (!chip->segment) {
		dev_dbg(dev, "%s: allocating segment table\n", __func__);
		chip->segment = kzalloc(sizeof(*chip->segment) *
				chip->seg_num_max, GFP_KERNEL);
		if (!chip->segment) {
			dev_err(dev, "%s: no memory!\n", __func__);
			return -ENOMEM;
		}
	}
	if (seg_num > chip->seg_num_max) {
		dev_warn(dev, "%s: %d segment requested, %d applied\n",
				__func__, seg_num, chip->seg_num_max);
		chip->segment_num = chip->seg_num_max;
	} else {
		chip->segment_num = seg_num;
	}
	memcpy(chip->segment, segment,
			chip->segment_num * sizeof(*chip->segment));
	dev_dbg(dev, "%s: %d segment requested, %d applied\n", __func__,
			seg_num, chip->seg_num_max);
	for (i = 0; i < chip->segment_num; i++)
		dev_dbg(dev, "segment %d: ratio %6u, k0 %6u, k1 %6u\n",
				i, chip->segment[i].ratio,
				chip->segment[i].k0, chip->segment[i].k1);
	return 0;
}

static void taos_calc_cpl(struct tsl2772_chip *chip) //計算CPL & 飽和值（為80％的飽和值）
{
	u32 cpl;
	u32 sat;
	u8 atime = chip->shadow[TSL277X_ALS_TIME];
	u8 agl = (chip->shadow[TSL277X_CONFIG] & TSL277X_ALS_AGL_MASK)  //TSL277X_ALS_AGL_MASK = 4 , TSL277X_ALS_AGL_SHIFT = 2
			>> TSL277X_ALS_AGL_SHIFT;			//確認AGL是否啟用

//針對不同料號使用對應的式子

	/* 8930 Boston CR #:XXX, WH Lee, 20130313 */
	/* For more precise als value */
	/* 8930 Dublin CR #:XXX, WH Lee, 20130708 */
	/* New parameter */
	/*TMD2772*/ u32 time_scale = (256 - atime ) * 2720 / 52;   //2730 =2.73*1000  ( CPL 放大 1000倍 ）
	/* WH Lee, 20130708 */
	/*TMD2772*/ //u32 time_scale = (256 - atime ) * 2720 / 20;   //2730 =2.73*1000  ( CPL 放大 1000倍 ）
	/* Original code */
	/*TMD2772*/ //u32 time_scale = (256 - atime ) * 2720 / 20 * 10;   //2730 =2.73*1000  ( CPL 放大 1000倍 ）
	/* WH Lee, 20130313 */

	/*TSL2772*/ //u32 time_scale = (256 - atime ) * 2730 / 60 * 10;   //2730 =2.73*1000  ( CPL 放大 1000倍 ）

	/*TSL2771 OR TSL2571*/	/*u32 time_scale = (256 - atime ) * 2720 / 53 * 10;*/   //2730 =2.73*1000  ( CPL 放大 1000倍 ）

	/*TMD2771*/	/*u32 time_scale = (256 - atime ) * 2720 / 24 * 10;*/   //2730 =2.73*1000  ( CPL 放大 1000倍 ）

/*20120830 johnny
	u32 time_scale = ((256 - atime) << SCALE_SHIFT) *		//atime = TSL277X_ALS_TIME 預設 = 238,SCALE_SHIFT = 11,
		TSL277X_ATIME_PER_100 / (TSL277X_ATIME_DEFAULT_MS * 100); // TSL277X_ATIME_PER_100 = 273,TSL277X_ATIME_DEFAULT_MS = 50
*/
										//time_scale = ((256-238)<<11)*273/(50*100) = 2012
	cpl = time_scale * chip->params.als_gain;				// cpl = 2012 * 8 = 16096
	if (agl)   							//AGL 功能是否被開啟 （ 初始值為 0 ）
		cpl = cpl * 16 / 100;						// cpl = 16096*16/1000 = 257
	sat = min_t(u32, MAX_ALS_VALUE, (u32)(256 - atime) << 10); 	//  (256-238)<<10 = 18432,min_t(u32,X,u32,Y)
	sat = sat * 8 / 10;					  	// ＝飽和值的80％
	dev_dbg(&chip->client->dev,
			"%s: cpl = %u [time_scale %u, gain %u, agl %u], "
			"saturation %u\n", __func__, cpl, time_scale,
			chip->params.als_gain, agl, sat);
	chip->als_inf.cpl = cpl;					//CPL = chip->als_inf.cpl
	chip->als_inf.saturation = sat;					//sat = chip->als_inf.saturation
}


static int set_als_gain(struct tsl2772_chip *chip, int gain)
{
	int rc;
	u8 ctrl_reg  = chip->shadow[TSL277X_CONTROL] & ~TSL277X_ALS_GAIN_MASK;

	switch (gain) {
	case 1:
		ctrl_reg |= AGAIN_1;
		break;
	case 8:
		ctrl_reg |= AGAIN_8;
		break;
	case 16:
		ctrl_reg |= AGAIN_16;
		break;
	case 120:
		ctrl_reg |= AGAIN_120;
		break;
	default:
		dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
				__func__, gain);
		return -EINVAL;
	}
	rc = taos_i2c_write(chip, TSL277X_CONTROL, ctrl_reg);
	if (!rc) {
		chip->shadow[TSL277X_CONTROL] = ctrl_reg;
		chip->params.als_gain = gain;
		dev_dbg(&chip->client->dev, "%s: new gain %d\n",
				__func__, gain);
	}
	return rc;
}

static int taos_get_lux(struct tsl2772_chip *chip)
{
	int ret = 0;
	struct device *dev = &chip->client->dev;	/*????*/
	u32 c0 = chip->als_inf.ch0;
	u32 c1 = chip->als_inf.ch1;
	u32 sat = chip->als_inf.saturation;
	u32 ratio;
	u64 lux_0, lux_1;
	u32 cpl = chip->als_inf.cpl;
	u32 lux;

	if (!chip->als_gain_auto) {  					//當als_gain 不是auto時
									//就看ch0是否小於10,若小於則LUX=0,若於大於則切換gain
		if (c0 <= MIN_ALS_VALUE) {  				//MIN_ALS_VALUE = 10
			dev_dbg(dev, "%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (c0 >= sat) {
			dev_dbg(dev, "%s: saturation, keep lux\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	} else {							//chip->als_gain_auto = "AUTO" = 非零 則切換 gain
		u8 gain = chip->params.als_gain;			// auto gain , 1x , 16x , 120x
		int rc = -EIO;

		if (gain == 16 && c0 >= sat) {
			rc = set_als_gain(chip, 1);
		/* 8930 Boston_CR #:XXX, WH Lee, 20130206 */
		/* Vendor recommand to remove 120x */
		} else if (gain == 1 && c0 < GAIN_SWITCH_LEVEL) {
			rc = set_als_gain(chip, 16);
		}
		/* 8930 Boston_CR #:XXX, WH Lee, 20130206 */
		/* Original code */
		/*
		} else if (gain == 16 && c0 < GAIN_SWITCH_LEVEL) {	//GAIN_SWITCH_LEVEL = 100
			rc = set_als_gain(chip, 120);
		} else if ((gain == 120 && c0 >= sat) ||
				(gain == 1 && c0 < GAIN_SWITCH_LEVEL)) {
			rc = set_als_gain(chip, 16);
		}
		*/
		/* WH Lee, 20130206 */
		if (!rc) {							//rc = 寫入gain設定成功為 非零
			dev_dbg(dev, "%s: gain adjusted, skip\n", __func__);	//則正常不會切入於此
			taos_calc_cpl(chip);
			ret = -EAGAIN;
			lux = chip->als_inf.lux;
			goto exit;
		}

		if (c0 <= MIN_ALS_VALUE) {
			dev_dbg(dev, "%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (c0 >= sat) {
			dev_dbg(dev, "%s: saturation, keep lux\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	}

	//atime = 50ms, gain = 8x , ch0 = 3891 , ch1 = 424

	ratio = (c1 << RATIO_SHIFT) / c0;						//ratio = c1 * 1024 / c0     //ratio = 111

//針對不同料號使用對應的式子

	/* Boston CR #:XXX, WH Lee, 20130307 */
	/* Vendor recommend */
	/* 8930 Dublin CR #:XXX, WH Lee, 20130708 */
	/* New parameter */
	if ((c0 * 100) > (c1 * 198))
		lux_0 = ( ( ( c0 * 100 ) - ( c1 * 198 ) ) * 100 ) / cpl;
	else
		lux_0 = 0;

	if ((c0 * 35) > (c1 * 63))
		lux_1 = ( ( ( c0 *  35 ) - ( c1 * 63 ) ) * 100 ) / cpl;
	else
		lux_1 = 0;
	#if 0
	if ((c0 * 100) > (c1 * 175))
		lux_0 = ( ( ( c0 * 100 ) - ( c1 * 175 ) ) * 100 ) / cpl;
	else
		lux_0 = 0;

	if ((c0 * 63) > (c1 * 100))
		lux_1 = ( ( ( c0 *  63 ) - ( c1 * 100 ) ) * 100 ) / cpl;
	else
		lux_1 = 0;
	#endif
	/* WH Lee, 20130708 */
	/* Original code */
	#if 0
	/* Boston CR #:XXX, WH Lee, 20121203 */
	/* cpl * 10000 not * 1000 */
	/*TMD2772*/	lux_0 = ( ( ( c0 * 100 ) - ( c1 * 175 ) ) * 100 ) / cpl; //( CPL 已經放大 1000 ） 	//20121101 	lux
	/*TMD2772*/	lux_1 = ( ( ( c0 *  63 ) - ( c1 * 100 ) ) * 100 ) / cpl; 				//20121101	lux
	#endif
	/* Original code */
	#if 0
	/*TMD2772*/	lux_0 = ( ( ( c0 * 100 ) - ( c1 * 175 ) ) * 10 ) / cpl; //( CPL 已經放大 1000 ） 	//20121101 	lux
	/*TMD2772*/	lux_1 = ( ( ( c0 *  63 ) - ( c1 * 100 ) ) * 10 ) / cpl; 				//20121101	lux
	#endif
	/* WH Lee, 20121203 */
	/* WH Lee, 20130307 */

	/*TSL2772*/	//lux_0 = ( ( ( c0 * 100 ) - ( c1 * 187 ) ) * 10 ) / cpl; //( CPL 已經放大 1000 ） 	//20120830 	lux
	/*TSL2772*/	//lux_1 = ( ( ( c0 *  63 ) - ( c1 * 100 ) ) * 10 ) / cpl; 				//20120830	lux

	//TSL2771 & TSL2571	//lux_0 = ( ( ( c0 * 100 ) - ( c1 * 200 ) ) * 10 ) / cpl; //( CPL 已經放大 1000 ） 	//20120830 	lux
	//TSL2771 & TSL2571	//lux_1 = ( ( ( c0 *  60 ) - ( c1 * 100 ) ) * 10 ) / cpl; 				//20120830	lux
///////////////////////

	//20120830	lux 驗證計算結果
	//snprintf( buf, "lux_0 = 0x%16llx , lux_1 = 0x%16llx , c0 = 0x%16llx , c1 = 0x%16llx ", lux_0 , lux_1 , c0 , c1 );

	/* 8930 Boston CR #:XXX, WH Lee, 20130227 */
	/* Vendor recommend */
	/* 8930 Dublin CR #:XXX, WH Lee, 20130708 */
	/* New parameter */
	lux = max(lux_0, lux_1);
	lux = max(lux , (u32)0);

	/* Sapporo CR #:XXX, WH Lee, 20131008 */
	/* Fix lux of fluorescent lamp is too small */
	#ifndef CONFIG_BUILD_FACTORY
	if (chip->als_inf.ratio_index == 0)
	{
		if (ratio >= 164 && ratio <= 266)
			chip->als_inf.ratio_index = 1;
	}
	else
	{
		if (ratio < 164 || ratio > 271)
			chip->als_inf.ratio_index = 0;
	}

	if (chip->als_inf.ratio_index == 1)
	{
		lux = (((5392 * ratio)/100 - 7646) * lux) / 1000;
	}
	#endif
	/* WH Lee, 20131008 */

	#if 0
	if (lux_0 >= lux_1)
	{
		lux = lux_0;
		//#if 1
		#ifndef CONFIG_BUILD_FACTORY
		//if (ratio < 165)
		//	lux = lux;
		if (ratio >= 165 && ratio < 190)
			lux = (lux * 3713) / 1000;
		if (ratio >= 190 && ratio < 250)
			lux = (lux * 1561) / 1000;
		if (ratio >= 250 && ratio < 500)
			lux = lux * 8;
		#endif
	}
	else	//lux_1 > lux_0
	{
		lux = (lux_1 * 461) >> 10; //0.45 = 461 / 1024
	}
	#endif
	/* WH Lee, 20130708 */

	if (0 >= lux)
		lux = 0;

	/* Original code */
	//lux = max(lux_0, lux_1);							//20120830	lux
	//lux = max(lux , (u32)0);							//20120830	lux
	/* WH Lee, 20130227 */

	/* Boston CR #:XXX, WH Lee, 20121130 */
	/* Vendor tune GA value */
	if( chip->als_inf.ga <= 0 ) 						//20120829	cal
		chip->als_inf.ga = 252;

	lux = (lux * chip->als_inf.ga) / 100;						//20120829	cal

	/* 8930_Boston_CR #:XXX, WH Lee, 20130412 */
	/* Fix light-sensor default value */
	if (chip->als_inf.cover_ratio <= 0)
		chip->als_inf.cover_ratio = 99;
	/* Original code */
	//if (chip->als_inf.cover_ratio <= 0)
	//	chip->als_inf.cover_ratio = 100;
	/* WH Lee, 20130412 */

	lux = (lux * chip->als_inf.cover_ratio) / 100;

	/* 8930 Boston CR #:XXX, WH Lee, 20130313 */
	/* For more precise als value */
	lux /= 10;
	/* WH Lee, 20130313 */

	/* Boston CR #:XXX, WH Lee, 20130307 */
	if (60000 < lux)
		lux = 60000;
	/* WH Lee, 20130307 */

	/* Original code */
	/*
	if( chip->als_inf.ga <= 0 ) 						//20120829	cal
		chip->als_inf.ga = 1 ;

	lux *= chip->als_inf.ga;						//20120829	cal
	*/
	/* WH Lee, 20121130 */

/*	lux_0 = k0 * (s64)c0;
	lux_1 = k1 * (s64)c1;
	if (lux_1 >= lux_0) {
		dev_dbg(&chip->client->dev, "%s: negative result - darkness\n",
				__func__);
		lux = 0;
		goto exit;
	}
	lux_0 -= lux_1;
	while (lux_0 & ((u64)0xffffffff << 32)) {				// lux0 & ( 0xFFFF FFFF << 32 )
		dev_dbg(&chip->client->dev, "%s: overwlow lux64 = 0x%16llx",
				__func__, lux_0);
		lux_0 >>= 1;
		cpl >>= 1;
	}

	if (!cpl) {
		dev_dbg(&chip->client->dev, "%s: zero cpl - darkness\n",
				__func__);
		lux = 0;
		goto exit;
	}
*/

/*
	lux = lux_0;								//lux0 = 11283680H
	lux = lux / cpl;
*/
							//LUX = 17883 or 1120031
	TSL277X_LSEN_PRINTK(1, "====================================\n[LSENSOR] lux_0=%lld\n[LSENSOR] lux_1=%lld\n[LSENSOR] ratio=%d\n[LSENSOR] ratio_index=%d\n", lux_0, lux_1, ratio, chip->als_inf.ratio_index);
exit:
	TSL277X_LSEN_PRINTK(1, "------------------------------------\n[LSENSOR] lux=%d\n[LSENSOR] c0=%d\n[LSENSOR] c1=%d\n[LSENSOR] gain=%d\n", lux, c0, c1, chip->params.als_gain);
	chip->als_inf.lux = lux;
	return ret;
}

static int pltf_power_on(struct tsl2772_chip *chip)
{
	int rc = 0;
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
			POWER_ON);
		msleep(10);
	}
	chip->unpowered = rc != 0;
	return rc;
}

static int pltf_power_off(struct tsl2772_chip *chip)
{
	int rc = 0;
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
			POWER_OFF);
		chip->unpowered = rc == 0;
	} else {
		chip->unpowered = false;
	}
	return rc;
}

static int taos_irq_clr(struct tsl2772_chip *chip, u8 bits)
{
	int ret = i2c_smbus_write_byte(chip->client, TSL277X_CMD_REG |
			TSL277X_CMD_SPL_FN | bits);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: failed, bits %x\n",
				__func__, bits);
	return ret;
}

static void taos_get_als(struct tsl2772_chip *chip)
{
	u32 ch0, ch1;
	u8 *buf = &chip->shadow[TSL277X_ALS_CHAN0LO];

	ch0 = le16_to_cpup((const __le16 *)&buf[0]);
	ch1 = le16_to_cpup((const __le16 *)&buf[2]);
	chip->als_inf.ch0 = ch0;
	chip->als_inf.ch1 = ch1;
	dev_dbg(&chip->client->dev, "%s: ch0 %u, ch1 %u\n", __func__, ch0, ch1);
}

static void taos_get_prox(struct tsl2772_chip *chip)
{
	u8 *buf = &chip->shadow[TSL277X_PRX_LO];
	bool d = chip->prx_inf.detected;

	chip->prx_inf.raw = (buf[1] << 8) | buf[0];

	/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
	/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
	if (chip->prx_calib_auto)
	{
		/* 8930 Boston_CR #:1670, WH Lee, 20130411 */
		/* Improve black card 0 cm issue*/
		if (chip->prx_inf.min_raw > chip->prx_inf.raw)
			chip->prx_inf.min_raw = chip->prx_inf.raw;
		/* WH Lee, 20130411 */

		if (chip->prx_inf.raw >= chip->prx_inf.curr_near)
			chip->prx_inf.new_status = NEAR;
		else if (chip->prx_inf.raw <= chip->prx_inf.curr_far)
			chip->prx_inf.new_status = FAR;
		else
		{
			chip->prx_inf.new_status = chip->prx_inf.last_status;
			dev_dbg(&chip->client->dev, "%s: Unknown status, raw=%d, curr_near=%d, curr_far=%d\n", __func__, chip->prx_inf.raw, chip->prx_inf.curr_near, chip->prx_inf.curr_far);
		}
	}
	else
	{
		chip->prx_inf.detected =
				(d && (chip->prx_inf.raw > chip->params.prox_th_min)) ||
				(!d && (chip->prx_inf.raw > chip->params.prox_th_max));
		dev_dbg(&chip->client->dev, "%s: raw %d, detected %d\n", __func__,
				chip->prx_inf.raw, chip->prx_inf.detected);
	}
#else
	chip->prx_inf.detected =
			(d && (chip->prx_inf.raw > chip->params.prox_th_min)) ||
			(!d && (chip->prx_inf.raw > chip->params.prox_th_max));
	dev_dbg(&chip->client->dev, "%s: raw %d, detected %d\n", __func__,
			chip->prx_inf.raw, chip->prx_inf.detected);
#endif
	/* Original code */
	/*
	chip->prx_inf.detected =
			(d && (chip->prx_inf.raw > chip->params.prox_th_min)) ||
			(!d && (chip->prx_inf.raw > chip->params.prox_th_max));
	dev_dbg(&chip->client->dev, "%s: raw %d, detected %d\n", __func__,
			chip->prx_inf.raw, chip->prx_inf.detected);
	*/
	/* WH Lee, 20130123 */
}

static int taos_read_all(struct tsl2772_chip *chip)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	ret = taos_i2c_blk_read(chip, TSL277X_STATUS,
			&chip->shadow[TSL277X_STATUS],
			TSL277X_PRX_HI - TSL277X_STATUS + 1);
	return (ret < 0) ? ret : 0;
}

/* Sapporo_CR #:XXX, WH Lee, 20131001 */
static int update_pers_reg(struct tsl2772_chip *chip)
{
	dev_dbg(&chip->client->dev, "%s: %02x\n", __func__,
			chip->shadow[TSL277X_PERSISTENCE]);
	return taos_i2c_write(chip, TSL277X_PERSISTENCE,
			chip->shadow[TSL277X_PERSISTENCE]);
}

static int update_prox_persist(struct tsl2772_chip *chip, int persist)
{
	int rc = 0;

	if ((chip->shadow[TSL277X_PERSISTENCE] & 0xF0) != PRX_PERSIST(persist))
	{
		chip->shadow[TSL277X_PERSISTENCE] = ((chip->shadow[TSL277X_PERSISTENCE] & 0xF) | PRX_PERSIST(persist));
		rc = update_pers_reg(chip);
	}

	return rc;
}

static int update_als_persist(struct tsl2772_chip *chip, int persist)
{
	int rc = 0;

	if ((chip->shadow[TSL277X_PERSISTENCE] & 0xF) != ALS_PERSIST(persist))
	{
		chip->shadow[TSL277X_PERSISTENCE] = ((chip->shadow[TSL277X_PERSISTENCE] & 0xF0) | ALS_PERSIST(persist));
		rc = update_pers_reg(chip);
	}

	return rc;
}
/* WH Lee, 20131001 */

static int update_prox_thresh(struct tsl2772_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TSL277X_PRX_MINTHRESHLO];
	u16 from, to;
	int algoritm;

	if (on_enable) {
		/* zero gate to force irq */
		from = to = 0;
	} else {
	/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
	/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
		if (chip->prx_calib_auto)
		{
			/* 8930 Boston_CR #:1670, WH Lee, 20130411 */
			/* Add log for debug black card 0 cm issue */
			TSL277X_PSEN_PRINTK(0, "[Before] raw=%d, status=%d, far=%d, near=%d\n",
				chip->prx_inf.raw, chip->prx_inf.new_status, chip->prx_inf.curr_far, chip->prx_inf.curr_near);
			/* WH Lee, 20130411 */

			if (system_rev < DVT0) //EVT1, EVT2
				algoritm = PROX_AUTO_CALIBRATOR_ALGORITHM_EVT;
			else //DVT
				algoritm = PROX_AUTO_CALIBRATOR_ALGORITHM;

			if(algoritm == PROX_AUTO_CALIBRATOR_ALGORITHM_EVT)
			{
				if (chip->prx_inf.raw < chip->prx_inf.curr_far - DELTA_OFFSET_FAR_EVT)
					chip->prx_inf.curr_far = chip->prx_inf.raw + DELTA_OFFSET_FAR_EVT;

				if (chip->prx_inf.curr_far < DELTA_OFFSET_FAR_EVT + 1)
					chip->prx_inf.curr_far = DELTA_OFFSET_FAR_EVT + 1;

				// min_far = FA_FAR + DELTA_OFFSET_FAR_EVT;
				if (chip->prx_inf.curr_far - DELTA_OFFSET_FAR_EVT < chip->params.prox_th_min_default)
					chip->prx_inf.curr_far = chip->params.prox_th_min_default + DELTA_OFFSET_FAR_EVT;

				if (chip->prx_inf.curr_far + chip->params.prox_th_max_default - chip->params.prox_th_min_default < MAX_VALUE - DELTA_OFFSET_MAX)
					chip->prx_inf.curr_near = chip->prx_inf.curr_far + chip->params.prox_th_max_default - chip->params.prox_th_min_default;
				else
					chip->prx_inf.curr_near = MAX_VALUE - DELTA_OFFSET_MAX;

				if (chip->prx_inf.curr_far + MIN_GATE > MAX_VALUE - DELTA_OFFSET_MAX)
					chip->prx_inf.curr_near = MAX_VALUE + 1;

				if (chip->prx_inf.new_status == NEAR && chip->prx_inf.raw >= RESET_THREADHOLD_VALUE)
				{
					/* 8930 Boston_CR #:1670, WH Lee, 20130411 */
					/* Improve black card 0 cm issue*/
					if (chip->prx_inf.min_raw + DELTA_OFFSET_FAR_EVT < 600)
					{
						if (chip->params.prox_th_min_default > 600)
						{
							chip->prx_inf.curr_far = chip->params.prox_th_min_default;
							chip->prx_inf.curr_near = chip->params.prox_th_max_default;
						}
						else
						{
							chip->prx_inf.curr_far = 600;
							chip->prx_inf.curr_near = 600 + chip->params.prox_th_max_default - chip->params.prox_th_min_default;
						}
					}
					else if (chip->prx_inf.min_raw + DELTA_OFFSET_FAR_EVT > RESET_FAR)
					{
						chip->prx_inf.curr_far = RESET_FAR;
						chip->prx_inf.curr_near = RESET_NEAR;
					}
					else
					{
						chip->prx_inf.curr_far = chip->prx_inf.min_raw + DELTA_OFFSET_FAR_EVT;
						if (chip->prx_inf.min_raw + DELTA_OFFSET_FAR_EVT + chip->params.prox_th_max_default - chip->params.prox_th_min_default <= MAX_VALUE - DELTA_OFFSET_MAX)
							chip->prx_inf.curr_near = chip->prx_inf.min_raw + DELTA_OFFSET_FAR_EVT + chip->params.prox_th_max_default - chip->params.prox_th_min_default;
						else
							chip->prx_inf.curr_near = MAX_VALUE - DELTA_OFFSET_MAX;
					}
					/* Original code */
					/*
					chip->prx_inf.curr_far = RESET_FAR;
					chip->prx_inf.curr_near = RESET_NEAR;
					*/
					/* WH Lee, 20130411 */
				}

				if (chip->prx_inf.new_status == NEAR)
				{
					/* 8930 Boston_CR #:XXX, WH Lee, 20130206 */
					/* Eragon recommand algorithm */
					if (chip->prx_inf.raw < RESET_THREADHOLD_VALUE &&
							(chip->params.prox_th_max_default - chip->params.prox_th_min_default > 200) &&
							(((chip->params.prox_th_max_default - chip->params.prox_th_min_default) / 2) + chip->prx_inf.curr_far < chip->prx_inf.curr_near - MIN_GATE))
						from = ((chip->params.prox_th_max_default - chip->params.prox_th_min_default) / 2) + chip->prx_inf.curr_far;
					else
						from = chip->prx_inf.curr_far;
					//from = chip->prx_inf.curr_far /* + DELTA_OFFSET_FAR_EVT */;
					/* WH Lee, 20130206 */
					//to = 0xFFFF;
					if (chip->prx_inf.raw >= RESET_THREADHOLD_VALUE)
						to = 0xFFFF;
					else
						to = RESET_THREADHOLD_VALUE;
				}
				else
				{
					// Don't detect min_far when min_far = FA_FAR + DELTA_OFFSET_FAR_EVT
					if (chip->prx_inf.curr_far <= chip->params.prox_th_min_default + DELTA_OFFSET_FAR_EVT)
						from = 0;
					else
						from = chip->prx_inf.curr_far - DELTA_OFFSET_FAR_EVT - 1;
					to = chip->prx_inf.curr_near;
				}
			}
			else //PROX_AUTO_CALIBRATOR_ALGORITHM
			{
				if (chip->prx_inf.raw < chip->prx_inf.curr_far - DELTA_OFFSET_FAR)
					chip->prx_inf.curr_far = chip->prx_inf.raw + DELTA_OFFSET_FAR;

				if (chip->prx_inf.curr_far < chip->params.prox_th_min_default + DELTA_OFFSET_FAR)
					chip->prx_inf.curr_far = chip->params.prox_th_min_default + DELTA_OFFSET_FAR;

				if (chip->params.prox_th_max_default < MAX_VALUE - DELTA_OFFSET_MAX)
					chip->prx_inf.curr_near = chip->params.prox_th_max_default;
				else
					chip->prx_inf.curr_near = MAX_VALUE - DELTA_OFFSET_MAX;

				if (chip->prx_inf.curr_near < chip->prx_inf.curr_far)
					chip->prx_inf.curr_near = chip->prx_inf.curr_far;

				if (chip->prx_inf.new_status == NEAR && chip->prx_inf.raw >= RESET_THREADHOLD_VALUE)
				{
					/* 8930 Boston_CR #:1670, WH Lee, 20130411 */
					/* Improve black card 0 cm issue*/
					if (chip->prx_inf.min_raw + 50 < 500)
					{
						chip->prx_inf.curr_far = 500;
						if (chip->params.prox_th_max_default > chip->prx_inf.curr_far + MIN_GATE)
							chip->prx_inf.curr_near = chip->params.prox_th_max_default;
						else
							chip->prx_inf.curr_near = 500 + MIN_GATE;
					}
					else if (chip->prx_inf.min_raw + 50 + MIN_GATE > chip->params.prox_th_max_default)
					{
						chip->prx_inf.curr_far = chip->params.prox_th_max_default - MIN_GATE;
						chip->prx_inf.curr_near = chip->params.prox_th_max_default;
					}
					else
					{
						chip->prx_inf.curr_far = chip->prx_inf.min_raw + 50;
						if (chip->params.prox_th_max_default > chip->prx_inf.curr_far + MIN_GATE)
							chip->prx_inf.curr_near = chip->params.prox_th_max_default;
						else
							chip->prx_inf.curr_near = chip->prx_inf.curr_far + MIN_GATE;
					}
					/* WH Lee, 20130411 */
				}

				if (chip->prx_inf.new_status == NEAR)
				{
					from = chip->prx_inf.curr_far;
					if (chip->prx_inf.raw >= RESET_THREADHOLD_VALUE || chip->prx_inf.curr_far >= 500)
						to = 0xFFFF;
					else
						to = RESET_THREADHOLD_VALUE;
				}
				else
				{
					// Don't detect min_far when min_far = FA_FAR + DELTA_OFFSET_FAR
					if (chip->prx_inf.curr_far <= chip->params.prox_th_min_default + DELTA_OFFSET_FAR)
						from = 0;
					else
						from = chip->prx_inf.curr_far - DELTA_OFFSET_FAR - 1;
					to = chip->prx_inf.curr_near;
				}
			}
			/* 8930 Boston_CR #:1670, WH Lee, 20130411 */
			/* Add log for debug black card 0 cm issue */
			TSL277X_PSEN_PRINTK(0, "[After] far=%d, near=%d, from=%d, to=%d\n",
				chip->prx_inf.curr_far, chip->prx_inf.curr_near, from, to);
			/* WH Lee, 20130411 */
		}
		else
		{
			if (chip->prx_inf.detected) {
				from = chip->params.prox_th_min;
				to = 0xffff;
			} else {
				from = 0;
				to = chip->params.prox_th_max;
			}
		}
#else
		if (chip->prx_inf.detected) {
			from = chip->params.prox_th_min;
			to = 0xffff;
		} else {
			from = 0;
			to = chip->params.prox_th_max;
		}
#endif
		/* Original code */
		/*
		if (chip->prx_inf.detected) {
			from = chip->params.prox_th_min;
			to = 0xffff;
		} else {
			from = 0;
			to = chip->params.prox_th_max;
		}
		*/
		/* WH Lee, 20130123 */
	}
	dev_dbg(&chip->client->dev, "%s: %u - %u\n", __func__, from, to);
	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = taos_i2c_blk_write(chip, TSL277X_PRX_MINTHRESHLO,
			&chip->shadow[TSL277X_PRX_MINTHRESHLO],
			TSL277X_PRX_MAXTHRESHHI - TSL277X_PRX_MINTHRESHLO + 1);
	return (ret < 0) ? ret : 0;
}

static int update_als_thres(struct tsl2772_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TSL277X_ALS_MINTHRESHLO];
	u16 gate = chip->params.als_gate;
	u16 from, to, cur;

	cur = chip->als_inf.ch0;
	if (on_enable) {
		/* zero gate far away form current position to force an irq */
		from = to = cur > 0xffff / 2 ? 0 : 0xffff;
	} else {
		gate = cur * gate / 100;
		if (!gate)
			gate = 1;
		if (cur > gate)
			from = cur - gate;
		else
			from = 0;
		if (cur < (0xffff - gate))
			to = cur + gate;
		else
			to = 0xffff;
	}
	dev_dbg(&chip->client->dev, "%s: [%u - %u]\n", __func__, from, to);
	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = taos_i2c_blk_write(chip, TSL277X_ALS_MINTHRESHLO,
			&chip->shadow[TSL277X_ALS_MINTHRESHLO],
			TSL277X_ALS_MAXTHRESHHI - TSL277X_ALS_MINTHRESHLO + 1);
	return (ret < 0) ? ret : 0;
}

static void report_prox(struct tsl2772_chip *chip)
{
	if (chip->p_idev) {
		/* 8930 Boston CR #:XXX, WH Lee, 20130123 */
		/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
		if (chip->prx_calib_auto)
		{
			if (chip->prx_inf.new_status ^ chip->prx_inf.last_status)
 				input_report_abs(chip->p_idev, ABS_DISTANCE, chip->prx_inf.new_status ? 0 : 1);
 			dev_dbg(&chip->client->dev, "%s: %s\n", __func__,
 				chip->prx_inf.new_status ? "NEAR":"FAR");
		}
		else
		{
			input_report_abs(chip->p_idev, ABS_DISTANCE,
					chip->prx_inf.detected ? 0 : 1);
		}
#else
		/* 8930 Boston CR #:XXX, WH Lee, 20121226 */
		/* Always return far in EVT1 */
		if (msm_project_id <= BOSTON && system_rev <= EVT1_2)
		{
			input_report_abs(chip->p_idev, ABS_DISTANCE, 1);
		}
		else
		{
			input_report_abs(chip->p_idev, ABS_DISTANCE,
					chip->prx_inf.detected ? 0 : 1);
		}
		/* Original code */
		//input_report_abs(chip->p_idev, ABS_DISTANCE,
		//		chip->prx_inf.detected ? 0 : 1);
		/* WH Lee, 20121226 */
#endif
		/* WH Lee, 20130123 */
		/* 8930 Boston CR #:XXX, WH Lee, 20121224 */
		/* Add for tuning p-sensor in UI */
#ifdef CONFIG_BUILD_FACTORY
		input_report_abs(chip->p_idev, ABS_PRESSURE,
				chip->prx_inf.raw);
#endif
		/* WH Lee, 20121224 */
		/* 8930 Boston CR #:XXX, WH Lee, 20130123 */
		/* Add for prox auto calibrator */
#ifdef CONFIG_BUILD_FACTORY
    #ifdef PROX_AUTO_CALIBRATOR
		if (chip->prx_calib_auto)
 			chip->prx_inf.last_status = chip->prx_inf.new_status;
		input_sync(chip->p_idev);
    #else
		input_sync(chip->p_idev);
    #endif
#else
    #ifdef PROX_AUTO_CALIBRATOR
		if (chip->prx_calib_auto)
		{
			if (chip->prx_inf.new_status ^ chip->prx_inf.last_status)
				input_sync(chip->p_idev);
 			chip->prx_inf.last_status = chip->prx_inf.new_status;
 		}
 		else
			input_sync(chip->p_idev);
    #else
		input_sync(chip->p_idev);
    #endif
#endif
		/* Original code */
		//input_sync(chip->p_idev);
		/* WH Lee, 20130123 */
	}
}

static void report_als(struct tsl2772_chip *chip)
{
	if (chip->a_idev) {
		int rc = taos_get_lux(chip);
		if (!rc) {
			int lux = chip->als_inf.lux;
			input_report_abs(chip->a_idev, ABS_MISC, lux);
 			dev_dbg(&chip->client->dev, "%s: %d\n", __func__, lux);
			/* 8930 Boston CR #:XXX, WH Lee, 20121224 */
			/* Add for tuning l-sensor in UI */
#ifdef CONFIG_BUILD_FACTORY
			input_report_abs(chip->a_idev, ABS_DISTANCE,
					chip->als_inf.ch0);
			input_report_abs(chip->a_idev, ABS_PRESSURE,
					chip->als_inf.ch1);
#endif
			/* WH Lee, 20121224 */
			input_sync(chip->a_idev);
			/* Sapporo_CR #:XXX, WH Lee, 20131001 */
			update_als_persist(chip, 3);
			/* WH Lee, 20131001 */
			update_als_thres(chip, 0);
		} else {
			update_als_thres(chip, 1);
		}
	}
}

static int taos_check_and_report(struct tsl2772_chip *chip)
{
	u8 status;

	int ret = taos_read_all(chip);
	if (ret)
		goto exit_clr;

	status = chip->shadow[TSL277X_STATUS];
	dev_dbg(&chip->client->dev, "%s: status 0x%02x\n", __func__, status);

	if ((status & (TSL277X_ST_PRX_VALID | TSL277X_ST_PRX_IRQ)) ==
			(TSL277X_ST_PRX_VALID | TSL277X_ST_PRX_IRQ)) {
		taos_get_prox(chip);
		report_prox(chip);
		update_prox_thresh(chip, 0);
		/* Sapporo_CR #:XXX, WH Lee, 20131001 */
		update_prox_persist(chip, 3);
		/* WH Lee, 20131001 */
	}

	if ((status & (TSL277X_ST_ALS_VALID | TSL277X_ST_ALS_IRQ)) ==
			(TSL277X_ST_ALS_VALID | TSL277X_ST_ALS_IRQ)) {
		taos_get_als(chip);
		report_als(chip);
	}
exit_clr:
	taos_irq_clr(chip, TSL277X_CMD_PROX_INT_CLR | TSL277X_CMD_ALS_INT_CLR);
	return ret;
}

static irqreturn_t taos_irq(int irq, void *handle)
{
	struct tsl2772_chip *chip = handle;
	struct device *dev = &chip->client->dev;

	/* 8930 Boston_CR #:XXX, WH Lee, 20130429 */
	/* Only wakelock when prox-sensor enable */
	if (chip->prx_enabled)
	{
		/* 8930 Boston_CR #:810, WH Lee, 20130318 */
		/* Add wakelock before report p-sensor data */
		wake_lock_timeout(&chip->prox_wake_lock, HZ);
		/* WH Lee, 20130318 */
	}
	/* WH Lee, 20130429 */

	mutex_lock(&chip->lock);
	if (chip->in_suspend) {
		dev_dbg(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		disable_irq_nosync(chip->client->irq);
		goto bypass;
	}
	dev_dbg(dev, "%s\n", __func__);
	(void)taos_check_and_report(chip);
bypass:
	mutex_unlock(&chip->lock);
	return IRQ_HANDLED;
}

static void set_pltf_settings(struct tsl2772_chip *chip)
{
	struct taos_raw_settings const *s = chip->pdata->raw_settings;
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

	if (s) {
		dev_dbg(dev, "%s: form pltf data\n", __func__);
		sh[TSL277X_ALS_TIME] = s->als_time;
		sh[TSL277X_PRX_TIME] = s->prx_time;
		sh[TSL277X_WAIT_TIME] = s->wait_time;
		sh[TSL277X_PERSISTENCE] = s->persist;
		sh[TSL277X_CONFIG] = s->cfg_reg;
		sh[TSL277X_PRX_PULSE_COUNT] = s->prox_pulse_cnt;
		sh[TSL277X_CONTROL] = s->ctrl_reg;
		sh[TSL277X_REG_PRX_OFFS] = s->prox_offs;
	} else {
		dev_dbg(dev, "%s: use defaults\n", __func__);
		/* 8930 Dublin #:XXX, WH Lee, 20130708 */
		/* New parameter */
		/* sh[TSL277X_ALS_TIME] = 219; */ /* ~100 ms */
		sh[TSL277X_ALS_TIME] = 238; /* ~50 ms */
		/* WH Lee, 20130708 */
		/* Boston CR #:XXX, WH Lee, 20121225 */
		/* Vendor tune value */
		sh[TSL277X_PRX_TIME] = 255;
		/* Original code */
		//sh[TSL277X_PRX_TIME] = 255;
		/* WH Lee, 20121225 */
		/* Sapporo CR #:XXX, WH Lee, 20131030 */
		sh[TSL277X_WAIT_TIME] = 237;
		sh[TSL277X_ENABLE] |= TSL277X_EN_WAIT;
		/* WH Lee, 20131030 */
		sh[TSL277X_PERSISTENCE] = PRX_PERSIST(1) | ALS_PERSIST(1);
		sh[TSL277X_CONFIG] = 0;
		/* Detroit 3.0 CR #:XXX, WH Lee, 20121122 */
		/* Sapporo CR #:XXX, WH Lee, 20131015 */
		if (system_rev < DVT0) //EVT1, EVT2
		{
			sh[TSL277X_PRX_PULSE_COUNT] = 27;
			sh[TSL277X_CONTROL] = AGAIN_8 | PGAIN_1 |
					PDIOD_CH1 | PDRIVE_60MA;
		}
		else //DVT
		{
			sh[TSL277X_PRX_PULSE_COUNT] = 20;
			sh[TSL277X_CONTROL] = AGAIN_8 | PGAIN_1 |
					PDIOD_CH1 | PDRIVE_120MA;
		}
		/* WH Lee, 20131015 */
		/* 8930_Boston_CR #:XXX, WH Lee, 20130412 */
		/* Fix p-sensor default value */
		/* sh[TSL277X_REG_PRX_OFFS] = (0x80 | 0xF); */ //-15
		sh[TSL277X_REG_PRX_OFFS] = 2;
		/* Original code */
		//sh[TSL277X_REG_PRX_OFFS] = 55;
		/* WH Lee, 20130412 */
		/* Original code */
		//sh[TSL277X_PRX_PULSE_COUNT] = 10;
		//sh[TSL277X_CONTROL] = AGAIN_8 | PGAIN_4 |
		//		PDIOD_CH0 | PDRIVE_120MA;
		//sh[TSL277X_REG_PRX_OFFS] = 0;
		/* WH Lee, 20121122 */

	}
	chip->params.als_gate = chip->pdata->parameters.als_gate;
	chip->params.prox_th_max = chip->pdata->parameters.prox_th_max;
	chip->params.prox_th_min = chip->pdata->parameters.prox_th_min;
	/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
	/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
	chip->params.prox_th_max_default = chip->pdata->parameters.prox_th_max_default;
	chip->params.prox_th_min_default = chip->pdata->parameters.prox_th_min_default;
	chip->prx_calib_auto = true;
#endif
	/* WH Lee, 20130123 */
	chip->params.als_gain = chip->pdata->parameters.als_gain;
	if (chip->pdata->parameters.als_gain) {
		chip->params.als_gain = chip->pdata->parameters.als_gain;
	} else {
		chip->als_gain_auto = true;
		chip->params.als_gain = GAIN_AUTO_INIT_VALUE;
		dev_dbg(&chip->client->dev, "%s: auto als gain.\n", __func__);
	}
	(void)set_als_gain(chip, chip->params.als_gain);
	taos_calc_cpl(chip);
}

static int flush_regs(struct tsl2772_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	dev_dbg(&chip->client->dev, "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = taos_i2c_write(chip, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}
	return rc;
}

static int update_enable_reg(struct tsl2772_chip *chip)
{
	dev_dbg(&chip->client->dev, "%s: %02x\n", __func__,
			chip->shadow[TSL277X_ENABLE]);
	return taos_i2c_write(chip, TSL277X_ENABLE,
			chip->shadow[TSL277X_ENABLE]);
}

static int taos_prox_enable(struct tsl2772_chip *chip, int on)
{
	int rc;

	dev_dbg(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
	/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
	/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
		/* Sapporo CR #:XXX, WH Lee, 20131015 */
		if (system_rev < DVT0) //EVT1, EVT2
		{
			chip->prx_inf.curr_far = INIT_FAR_EVT;
			chip->prx_inf.curr_near = INIT_NEAR_EVT;
		}
		else //DVT
		{
			chip->prx_inf.curr_far = INIT_FAR(chip->params.prox_th_min_default);
			chip->prx_inf.curr_near = INIT_NEAR(chip->params.prox_th_min_default);
		}
		/* WH Lee, 20131015 */
		chip->prx_inf.last_status = UNKNOW_STAT;
		/* 8930 Boston_CR #:1670, WH Lee, 20130411 */
		/* Improve black card 0 cm issue*/
		chip->prx_inf.min_raw = MAX_VALUE;
		/* WH Lee, 20130411 */
#endif
	/* WH Lee, 20130123 */

		/* Sapporo_CR #:XXX, WH Lee, 20131001 */
		update_prox_persist(chip, 1);
		/* WH Lee, 20131001 */

		taos_irq_clr(chip, TSL277X_CMD_PROX_INT_CLR);
		update_prox_thresh(chip, 1);
		chip->shadow[TSL277X_ENABLE] |=
				(TSL277X_EN_PWR_ON | TSL277X_EN_PRX |
				TSL277X_EN_PRX_IRQ);
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		msleep(3);
		/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_on(chip->prx_pmlog_device);
		if (rc)
                        dev_err(&chip->client->dev, "%s: pmlog_device_on fail rc = %d\n", __func__,rc);
#endif
		/* WH Lee, 20130107 */
	} else {
		chip->shadow[TSL277X_ENABLE] &=
				~(TSL277X_EN_PRX_IRQ | TSL277X_EN_PRX);
		if (!(chip->shadow[TSL277X_ENABLE] & TSL277X_EN_ALS))
			chip->shadow[TSL277X_ENABLE] &= ~TSL277X_EN_PWR_ON;
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		taos_irq_clr(chip, TSL277X_CMD_PROX_INT_CLR);
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_off(chip->prx_pmlog_device);
		if (rc)
                        dev_err(&chip->client->dev, "%s: pmlog_device_off fail rc = %d\n", __func__,rc);
#endif
		/* WH Lee, 20130107 */
	}
	if (!rc)
		chip->prx_enabled = on;
	return rc;
}

static int taos_als_enable(struct tsl2772_chip *chip, int on)
{
	int rc;

	dev_dbg(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		/* 8930 Boston CR #:XXX, WH Lee, 20130313 */
		/* Workaround ALS/input event do not update value when value is the same. */
		if (chip->als_inf.lux != 1)
			input_report_abs(chip->a_idev, ABS_MISC, 1);
		else
			input_report_abs(chip->a_idev, ABS_MISC, 0);
		/* WH Lee, 20130313 */
		/* Sapporo CR #:XXX, WH Lee, 20131008 */
		/* Sapporo_CR #:XXX, WH Lee, 20131001 */
		update_als_persist(chip, 1);
		/* WH Lee, 20131001 */
		/* Fix lux of fluorescent lamp is too small */
		chip->als_inf.ratio_index = 0;
		/* WH Lee, 20131008 */
		taos_irq_clr(chip, TSL277X_CMD_ALS_INT_CLR);
		update_als_thres(chip, 1);
		chip->shadow[TSL277X_ENABLE] |=
				(TSL277X_EN_PWR_ON | TSL277X_EN_ALS |
				TSL277X_EN_ALS_IRQ);
		/* Sapporo CR #:XXX, WH Lee, 20131030 */
		chip->shadow[TSL277X_ENABLE] &= ~TSL277X_EN_WAIT;
		/* WH Lee, 20131030 */
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		msleep(3);
		/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_on(chip->als_pmlog_device);
		if (rc)
                        dev_err(&chip->client->dev, "%s: pmlog_device_on fail rc = %d\n", __func__,rc);
#endif
		/* WH Lee, 20130107 */
	} else {
		chip->shadow[TSL277X_ENABLE] &=
				~(TSL277X_EN_ALS_IRQ | TSL277X_EN_ALS);
		/* Sapporo CR #:XXX, WH Lee, 20131030 */
		chip->shadow[TSL277X_ENABLE] |= TSL277X_EN_WAIT;
		/* WH Lee, 20131030 */
		if (!(chip->shadow[TSL277X_ENABLE] & TSL277X_EN_PRX))
			chip->shadow[TSL277X_ENABLE] &= ~TSL277X_EN_PWR_ON;
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		taos_irq_clr(chip, TSL277X_CMD_ALS_INT_CLR);
		/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
		rc = pmlog_device_off(chip->als_pmlog_device);
		if (rc)
                        dev_err(&chip->client->dev, "%s: pmlog_device_off fail rc = %d\n", __func__,rc);
#endif
		/* WH Lee, 20130107 */
	}
	if (!rc)
		chip->als_enabled = on;
	/* 8930 Boston CR #:XXX, WH Lee, 20130226 */
	/* Add als enable/disable log for debug */
	printk("[WH Lee] %s::on=%d, rc=%d\n", __func__, on, rc);
	/* WH Lee, 20130226 */
	return rc;
}

static ssize_t taos_device_als_ch0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.ch0);
}

static ssize_t taos_device_als_ch1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.ch1);
}

static ssize_t taos_device_als_cpl(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cpl);
}

static ssize_t taos_device_als_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t taos_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	struct lux_segment *s = chip->segment;
	int i, k;

	for (i = k = 0; i < chip->segment_num; i++)
		k += snprintf(buf + k, PAGE_SIZE - k, "%d:%u,%u,%u\n", i,
				(s[i].ratio * 1000) >> RATIO_SHIFT,
				(s[i].k0 * 1000) >> SCALE_SHIFT,
				(s[i].k1 * 1000) >> SCALE_SHIFT);
	return k;
}

static ssize_t taos_lux_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int i;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	u32 ratio, k0, k1;

	if (4 != sscanf(buf, "%10d:%10u,%10u,%10u", &i, &ratio, &k0, &k1))
		return -EINVAL;
	if (i >= chip->segment_num)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->segment[i].ratio = (ratio << RATIO_SHIFT) / 1000;
	chip->segment[i].k0 = (k0 << SCALE_SHIFT) / 1000;
	chip->segment[i].k1 = (k1 << SCALE_SHIFT) / 1000;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tsl2772_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tsl2772_als_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		taos_als_enable(chip,1);
	else
		taos_als_enable(chip,0);

	return size;
}

static ssize_t tsl2772_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t tsl2772_prox_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	/* Sapporo CR #:XXX, WH Lee, 20131030 */
	if (chip->prx_enabled == value)
		return size;
	/* Sapporo CR #:XXX, WH Lee, 20131030 */

	if (value)
		taos_prox_enable(chip,1);
	else
		taos_prox_enable(chip,0);

	return size;
}

static ssize_t taos_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n", chip->params.als_gain,
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t taos_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gain;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &gain);
	if (rc)
		return -EINVAL;
	if (gain != 0 && gain != 1 && gain != 8 && gain != 16 && gain != 120)
		return -EINVAL;
	mutex_lock(&chip->lock);
	if (gain) {
		chip->als_gain_auto = false;
		rc = set_als_gain(chip, gain);
		if (!rc)
			taos_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	mutex_unlock(&chip->lock);
	return rc ? rc : size;
}

static ssize_t taos_als_gate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (in %%)\n", chip->params.als_gate);
}

static ssize_t taos_als_gate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gate;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &gate);
	if (rc || gate > 100)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->params.als_gate = gate;
	mutex_unlock(&chip->lock);
	return size;
}

/* Detroit 3,.0 CR #:XXX, WH Lee, 20121115 */
static ssize_t taos_als_ga_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (in %%)\n", chip->als_inf.ga);
}

static ssize_t taos_als_ga_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long ga;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &ga);
	if (rc)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->als_inf.ga = ga;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_als_cover_ratio_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (in %%)\n", chip->als_inf.cover_ratio);
}

static ssize_t taos_als_cover_ratio_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long cover_ratio;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &cover_ratio);
	if (rc)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->als_inf.cover_ratio = cover_ratio;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_device_als_lux_polling(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	int ret;
	ret = taos_read_all(chip);
	if (ret)
		return -EINVAL;

	taos_get_als(chip);
	taos_get_lux(chip);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t taos_prox_th_min_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_th_min);
}

static ssize_t taos_prox_th_min_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long threadhold;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &threadhold);
	if (rc)
		return -EINVAL;
	mutex_lock(&chip->lock);
/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
	if (threadhold) {
		chip->prx_calib_auto = false;
		chip->params.prox_th_min = threadhold;
	} else {
		chip->prx_calib_auto = true;
		/* Sapporo CR #:XXX, WH Lee, 20131015 */
		if (system_rev < DVT0) //EVT1, EVT2
		{
			chip->prx_inf.curr_far = INIT_FAR_EVT;
			chip->prx_inf.curr_near = INIT_NEAR_EVT;
		}
		else //DVT
		{
			chip->prx_inf.curr_far = INIT_FAR(chip->params.prox_th_min_default);
			chip->prx_inf.curr_near = INIT_NEAR(chip->params.prox_th_min_default);
		}
		/* WH Lee, 20131015 */
		chip->prx_inf.last_status = UNKNOW_STAT;
	}
#else
	chip->params.prox_th_min = threadhold;
#endif
	/* Original code */
	//chip->params.prox_th_min = threadhold;
/* WH Lee, 20130123 */
	update_prox_thresh(chip, 0);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_prox_th_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_th_max);
}

static ssize_t taos_prox_th_max_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long threadhold;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &threadhold);
	if (rc)
		return -EINVAL;
	mutex_lock(&chip->lock);
/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
	if (threadhold) {
		chip->prx_calib_auto = false;
		chip->params.prox_th_max = threadhold;
	} else {
		chip->prx_calib_auto = true;
		/* Sapporo CR #:XXX, WH Lee, 20131015 */
		if (system_rev < DVT0) //EVT1, EVT2
		{
			chip->prx_inf.curr_far = INIT_FAR_EVT;
			chip->prx_inf.curr_near = INIT_NEAR_EVT;
		}
		else //DVT
		{
			chip->prx_inf.curr_far = INIT_FAR(chip->params.prox_th_min_default);
			chip->prx_inf.curr_near = INIT_NEAR(chip->params.prox_th_min_default);
		}
		/* WH Lee, 20131015 */
		chip->prx_inf.last_status = UNKNOW_STAT;
	}
#else
	chip->params.prox_th_max = threadhold;
#endif
	/* Original code */
	//chip->params.prox_th_max = threadhold;
/* WH Lee, 20130123 */
	update_prox_thresh(chip, 0);
	mutex_unlock(&chip->lock);
	return size;
}

/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
static ssize_t taos_prox_th_min_default_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_th_min_default);
}

static ssize_t taos_prox_th_min_default_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long threadhold;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &threadhold);
	if (rc)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->params.prox_th_min_default = threadhold;
	update_prox_thresh(chip, 0);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_prox_th_max_default_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->params.prox_th_max_default);
}

static ssize_t taos_prox_th_max_default_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long threadhold;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &threadhold);
	if (rc)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->params.prox_th_max_default = threadhold;
	update_prox_thresh(chip, 0);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_prox_curr_far_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.curr_far);
}

static ssize_t taos_prox_curr_near_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.curr_near);
}
#endif
/* WH Lee, 20130123 */

static ssize_t taos_chip_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->chip_id);
}

static ssize_t taos_dump_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	for (i = TSL277X_ENABLE; i < TSL277X_REG_MAX; i++)
		dev_dbg(&chip->client->dev, "[%2x]=%2x\n", i, chip->shadow[i]);

	return snprintf(buf, PAGE_SIZE, "Refer kmsg log\n");
}

static ssize_t taos_device_prx_raw_polling(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = taos_read_all(chip);
	if (rc)
		return -EINVAL;

	chip->prx_inf.raw = (chip->shadow[TSL277X_PRX_HI] << 8) | chip->shadow[TSL277X_PRX_LO];
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t taos_device_prx_detected_polling(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	rc = taos_read_all(chip);
	if (rc)
		return -EINVAL;

	taos_get_prox(chip);

	/* 8930 Boston CR #:XXX, WH Lee, 20130123 */
	/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
	if (chip->prx_calib_auto)
		return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.new_status);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
#else
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
#endif
	/* Original code */
	//return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
	/* WH Lee, 20130123 */
}

static ssize_t taos_prox_pulse_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->shadow[TSL277X_PRX_PULSE_COUNT]);
}

static ssize_t taos_prox_pulse_count_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long count;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &count);
	if (rc)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->shadow[TSL277X_PRX_PULSE_COUNT] = count;
	flush_regs(chip);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_prox_pdrive_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int drive;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	drive = chip->shadow[TSL277X_CONTROL] & (0x3 << 6);
	switch(drive)
	{
		case PDRIVE_120MA:
			drive = 120;
			break;
		case PDRIVE_60MA:
			drive = 60;
			break;
		case PDRIVE_30MA:
			drive = 30;
			break;
		case PDRIVE_15MA:
			drive = 15;
			break;
		default:
			drive = -1;
			break;
	}
	return snprintf(buf, PAGE_SIZE, "%d mA\n", drive);
}

static ssize_t taos_prox_pdrive_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long drive;
	int rc, temp;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &drive);
	if (rc)
		return -EINVAL;

	switch (drive)
	{
		case 120:
			drive = PDRIVE_120MA;
			break;
		case 60:
			drive = PDRIVE_60MA;
			break;
		case 30:
			drive = PDRIVE_30MA;
			break;
		case 15:
			drive = PDRIVE_15MA;
			break;
		default:
			return -EINVAL;
	}
	mutex_lock(&chip->lock);
	temp = chip->shadow[TSL277X_CONTROL];
	temp &= ~(0x3 << 6);
	temp |= drive;
	chip->shadow[TSL277X_CONTROL] = temp;
	flush_regs(chip);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_prox_channel_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int channel;
	char temp[20];
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	channel = chip->shadow[TSL277X_CONTROL] & (0x3 << 4);
	memset(temp, 0, sizeof(temp));

	switch(channel)
	{
		case PDIOD_NO:
			snprintf(temp, sizeof(temp), "%s", "NO");
			break;
		case PDIOD_CH0:
			snprintf(temp, sizeof(temp), "%s", "CH0");
			break;
		case PDIOD_CH1:
			snprintf(temp, sizeof(temp), "%s", "CH1");
			break;
		default:
			snprintf(temp, sizeof(temp), "%s", "ERR");
			break;
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", temp);
}

static ssize_t taos_prox_channel_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long channel;
	int rc, temp;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &channel);
	if (rc)
		return -EINVAL;

	switch (channel)
	{
		case 0:
			channel = PDIOD_CH0;
			break;
		case 1:
			channel = PDIOD_CH1;
			break;
		case 2:
			channel = PDIOD_NO;
			break;
		default:
			return -EINVAL;
	}
	mutex_lock(&chip->lock);
	temp = chip->shadow[TSL277X_CONTROL];
	temp &= ~(0x3 << 4);
	temp |= channel;
	chip->shadow[TSL277X_CONTROL] = temp;
	flush_regs(chip);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_prox_offset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int offset;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	offset = chip->shadow[TSL277X_REG_PRX_OFFS];

	if (offset & 0x80)
		offset = (offset & 0x7F) * -1;

	return snprintf(buf, PAGE_SIZE, "%d\n", offset);
}

static ssize_t taos_prox_offset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	s16 offset;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = kstrtos16(buf, 10, &offset);
	if (rc)
		return -EINVAL;

	if (offset > 127 || offset < -127)
		return -EINVAL;

	if (offset < 0)
		offset = 0x80 | (offset * -1);

	mutex_lock(&chip->lock);
	chip->shadow[TSL277X_REG_PRX_OFFS] = offset;
	flush_regs(chip);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_prox_pgain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int pgain;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	pgain = chip->shadow[TSL277X_CONTROL] & (0x3 << 2);

	switch (pgain)
	{
		case PGAIN_1:
			pgain = 1;
			break;
		case PGAIN_2:
			pgain = 2;
			break;
		case PGAIN_4:
			pgain = 4;
			break;
		case PGAIN_8:
			pgain = 8;
			break;
		default:
			return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", pgain);
}

static ssize_t taos_prox_pgain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	s16 pgain;
	int rc, temp;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = kstrtos16(buf, 10, &pgain);
	if (rc)
		return -EINVAL;

	switch (pgain)
	{
		case 1:
			pgain = PGAIN_1;
			break;
		case 2:
			pgain = PGAIN_2;
			break;
		case 4:
			pgain = PGAIN_4;
			break;
		case 8:
			pgain = PGAIN_8;
			break;
		default:
			return -EINVAL;
	}

	mutex_lock(&chip->lock);
	temp = chip->shadow[TSL277X_CONTROL];
	temp &= ~(0x3 << 2);
	temp |= pgain;
	chip->shadow[TSL277X_CONTROL] = temp;
	flush_regs(chip);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_prox_ptime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->shadow[TSL277X_PRX_TIME]);
}

static ssize_t taos_prox_ptime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long ptime;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &ptime);
	if (rc)
		return -EINVAL;

	mutex_lock(&chip->lock);
	chip->shadow[TSL277X_PRX_TIME] = ptime & 0xFF;
	flush_regs(chip);
	mutex_unlock(&chip->lock);
	return size;
}
/* WH Lee, 20121115 */

static ssize_t taos_device_prx_raw(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t taos_device_prx_detected(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	/* 8930 Boston CR #:XXX, WH Lee, 20130123 */
	/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
	if (chip->prx_calib_auto)
		return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.new_status);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
#else
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
#endif
	/* Original code */
	//return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
	/* WH Lee, 20130123 */
}

/* Sapporo CR #:XXX, WH Lee, 20131009 */
static ssize_t taos_prox_can_wake_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->pdata->proximity_can_wake ? 1 : 0);
}

static ssize_t taos_prox_can_wake_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long can_wake;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &can_wake);
	if (rc)
		return -EINVAL;

	mutex_lock(&chip->lock);
	chip->pdata->proximity_can_wake = can_wake ? true : false;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_als_can_wake_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->pdata->als_can_wake ? 1 : 0);
}

static ssize_t taos_als_can_wake_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long can_wake;
	int rc;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &can_wake);
	if (rc)
		return -EINVAL;

	mutex_lock(&chip->lock);
	chip->pdata->als_can_wake = can_wake ? true : false;
	mutex_unlock(&chip->lock);
	return size;
}
/* WH Lee, 20131009 */

static struct device_attribute prox_attrs[] = {
	__ATTR(prx_raw, 0444, taos_device_prx_raw, NULL),
	__ATTR(prx_detect, 0444, taos_device_prx_detected, NULL),
	/* Detroit 3.0 CR #:XXX, WH Lee, 20121115 */
	__ATTR(prx_raw_polling, 0444, taos_device_prx_raw_polling, NULL),
	__ATTR(prx_detect_polling, 0444, taos_device_prx_detected_polling, NULL),
	__ATTR(prox_power_state, 0644, tsl2772_prox_enable_show, tsl2772_prox_enable),
	__ATTR(prox_th_min, 0644, taos_prox_th_min_show, taos_prox_th_min_store),
	__ATTR(prox_th_max, 0644, taos_prox_th_max_show, taos_prox_th_max_store),
	/* 8930 Boston_CR #:XXX, WH Lee, 20130123 */
	/* Add for prox auto calibrator */
#ifdef PROX_AUTO_CALIBRATOR
	__ATTR(prox_th_min_default, 0644, taos_prox_th_min_default_show, taos_prox_th_min_default_store),
	__ATTR(prox_th_max_default, 0644, taos_prox_th_max_default_show, taos_prox_th_max_default_store),
	__ATTR(prox_curr_far, 0444, taos_prox_curr_far_show, NULL),
	__ATTR(prox_curr_near, 0444, taos_prox_curr_near_show, NULL),
#endif
	/* WH Lee, 20130123 */
	__ATTR(chip_id, 0444, taos_chip_id_show, NULL),
	__ATTR(dump_reg, 0444, taos_dump_reg_show, NULL),
	__ATTR(prx_pulse_count, 0644, taos_prox_pulse_count_show, taos_prox_pulse_count_store),
	__ATTR(prx_pdrive, 0644, taos_prox_pdrive_show, taos_prox_pdrive_store),
	__ATTR(prx_channel, 0644, taos_prox_channel_show, taos_prox_channel_store),
	__ATTR(prx_offset, 0644, taos_prox_offset_show, taos_prox_offset_store),
	__ATTR(prx_pgain, 0644, taos_prox_pgain_show, taos_prox_pgain_store),
	__ATTR(prx_ptime, 0644, taos_prox_ptime_show, taos_prox_ptime_store),
	/* WH Lee, 20121115 */
	/* Sapporo CR #:XXX, WH Lee, 20131009 */
	__ATTR(prx_can_wake, 0644, taos_prox_can_wake_show, taos_prox_can_wake_store),
	/* WH Lee, 20131009 */
};

static struct device_attribute als_attrs[] = {
	__ATTR(als_ch0, 0444, taos_device_als_ch0, NULL),
	__ATTR(als_ch1, 0444, taos_device_als_ch1, NULL),
	__ATTR(als_cpl, 0444, taos_device_als_cpl, NULL),
	__ATTR(als_lux, 0444, taos_device_als_lux, NULL),
	__ATTR(als_gain, 0644, taos_als_gain_show, taos_als_gain_store),
	__ATTR(als_gate, 0644, taos_als_gate_show, taos_als_gate_store),
	/* Detroit 3.0 CR #:XXX, WH Lee, 20121115 */
	__ATTR(als_ga, 0644, taos_als_ga_show, taos_als_ga_store),
	__ATTR(als_cover_ratio, 0644, taos_als_cover_ratio_show, taos_als_cover_ratio_store),
	__ATTR(als_lux_polling, 0444, taos_device_als_lux_polling, NULL),
	/* WH Lee, 20121115 */
	__ATTR(lux_table, 0644, taos_lux_table_show, taos_lux_table_store),
	__ATTR(als_power_state, 0644, tsl2772_als_enable_show, tsl2772_als_enable),
	/* Detroit 3.0 CR #:XXX, WH Lee, 20121115 */
	__ATTR(chip_id, 0444, taos_chip_id_show, NULL),
	__ATTR(dump_reg, 0444, taos_dump_reg_show, NULL),
	/* Original code */
	//__ATTR(prox_power_state, 0644, tsl2772_prox_enable_show, tsl2772_prox_enable),
	/* WH Lee, 20121115 */
	/* Sapporo CR #:XXX, WH Lee, 20131009 */
	__ATTR(als_can_wake, 0644, taos_als_can_wake_show, taos_als_can_wake_store),
	/* WH Lee, 20131009 */
};

static int add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int taos_get_id(struct tsl2772_chip *chip, u8 *id, u8 *rev)
{
	int rc = taos_i2c_read(chip, TSL277X_REVID, rev);
	if (rc)
		return rc;
	return taos_i2c_read(chip, TSL277X_CHIPID, id);
}

static int power_on(struct tsl2772_chip *chip)
{
	int rc;
	rc = pltf_power_on(chip);
	if (rc)
		return rc;
	dev_dbg(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return flush_regs(chip);
}

/* Boston CR #:XXX, WH Lee, 20121211 */
/* Do not implement input device open/close function */
/*
static int prox_idev_open(struct input_dev *idev)
{
	struct tsl2772_chip *chip = dev_get_drvdata(&idev->dev);
	int rc;
	bool als = chip->a_idev && chip->a_idev->users;

	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		rc = power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = taos_prox_enable(chip, 1);
	if (rc && !als)
		pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return rc;
}

static void prox_idev_close(struct input_dev *idev)
{
	struct tsl2772_chip *chip = dev_get_drvdata(&idev->dev);

	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	taos_prox_enable(chip, 0);
	if (!chip->a_idev || !chip->a_idev->users)
		pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

static int als_idev_open(struct input_dev *idev)
{
	struct tsl2772_chip *chip = dev_get_drvdata(&idev->dev);
	int rc;
	bool prox = chip->p_idev && chip->p_idev->users;

	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		rc = power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = taos_als_enable(chip, 1);
	if (rc && !prox)
		pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return rc;
}

static void als_idev_close(struct input_dev *idev)
{
	struct tsl2772_chip *chip = dev_get_drvdata(&idev->dev);
	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	taos_als_enable(chip, 0);
	if (!chip->p_idev || !chip->p_idev->users)
		pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}
*/
/* WH Lee, 20121211 */

/* 8930 Boston_CR #:XXX, WH Lee, 20130424 */
static void taos_create_kernel_debuglevel(void)
{
	if (kernel_debuglevel_dir!=NULL) {
		debugfs_create_u32("psensor_log_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&TSL277X_PSEN_LOG_DLL));
		debugfs_create_u32("lsensor_log_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&TSL277X_LSEN_LOG_DLL));
	} else {
		printk(KERN_ERR "failed to create TSL277x log dll in kernel_debuglevel_dir!!!\n");
	}

}

static void taos_destroy_kernel_debuglevel(void)
{
	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}
/* WH Lee 20130424 */

static int __devinit taos_probe(struct i2c_client *client,
	const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct tsl2772_chip *chip;
	struct tsl2772_i2c_platform_data *pdata = dev->platform_data;
	bool powered = 0;

	dev_info(dev, "%s: client->irq = %d\n", __func__, client->irq);
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		dev_err(dev, "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (!(pdata->prox_name || pdata->als_name) || client->irq < 0) {
		dev_err(dev, "%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (pdata->platform_init) {
		ret = pdata->platform_init(dev);
		if (ret)
			goto init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);
		if (ret) {
			dev_err(dev, "%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		msleep(10);
	}
	chip = kzalloc(sizeof(struct tsl2772_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);
	if (chip->pdata->segment)
		ret = set_segment_table(chip, chip->pdata->segment,
			chip->pdata->segment_num);
	else
		ret =  set_segment_table(chip, segment_default,
			ARRAY_SIZE(segment_default));
	if (ret)
		goto set_segment_failed;

	ret = taos_get_id(chip, &id, &rev);
	if (ret)
		goto id_failed;
	for (i = 0; i < ARRAY_SIZE(tsl277x_ids); i++) {
		if (id == tsl277x_ids[i])
			break;
	}
	if (i < ARRAY_SIZE(tsl277x_names)) {
		dev_info(dev, "%s: '%s rev. %d' detected\n", __func__,
			tsl277x_names[i], rev);
	} else {
		dev_err(dev, "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}
	chip->chip_id = id;
	mutex_init(&chip->lock);
	set_pltf_settings(chip);
	ret = flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}

	if (!pdata->prox_name)
		goto bypass_prox_idev;

	/* 8930 Boston_CR #:810, WH Lee, 20130314 */
	/* Add wakelock before report p-sensor data */
	wake_lock_init(&chip->prox_wake_lock , WAKE_LOCK_SUSPEND, pdata->prox_name);
	/* WH Lee, 20130314 */

	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	chip->p_idev->name = pdata->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->p_idev->evbit);
	set_bit(ABS_DISTANCE, chip->p_idev->absbit);
	input_set_abs_params(chip->p_idev, ABS_DISTANCE, 0, 1, 0, 0);
	/* Boston CR #:XXX, WH Lee, 20121224 */
	/* Add for tuning p-sensor in UI */
#ifdef CONFIG_BUILD_FACTORY
	input_set_abs_params(chip->p_idev, ABS_PRESSURE, 0, 1023, 0, 0);
#endif
	/* WH Lee, 20121224 */
	/* Boston CR #:XXX, WH Lee, 20121211 */
	/* Do not implement input device open/close function */
	/*
	chip->p_idev->open = prox_idev_open;
	chip->p_idev->close = prox_idev_close;
	*/
	/* WH Lee, 20121211 */

	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		input_free_device(chip->p_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_p_alloc_failed;
	}
	ret = add_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
	if (ret)
		goto input_p_sysfs_failed;
	/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
	chip->prx_pmlog_device = pmlog_register_device(&chip->p_idev->dev);
#endif
	/* WH Lee, 20130107 */
bypass_prox_idev:
	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	/* Boston CR #:XXX, WH Lee, 20121224 */
	/* Add for tuning l-sensor in UI */
#ifdef CONFIG_BUILD_FACTORY
	input_set_abs_params(chip->a_idev, ABS_DISTANCE, 0, 65535, 0, 0);
	input_set_abs_params(chip->a_idev, ABS_PRESSURE, 0, 65535, 0, 0);
#endif
	/* WH Lee, 20121224 */
	/* Boston CR #:XXX, WH Lee, 20121211 */
	/* Do not implement input device open/close function */
	/*
	chip->a_idev->open = als_idev_open;
	chip->a_idev->close = als_idev_close;
	*/
	/* WH Lee, 20121211 */
	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_a_alloc_failed;
	}
	ret = add_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;
	/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
	chip->als_pmlog_device = pmlog_register_device(&chip->a_idev->dev);
#endif
	/* WH Lee, 20130107 */
bypass_als_idev:
	ret = request_threaded_irq(client->irq, NULL, taos_irq,
		      IRQF_TRIGGER_FALLING,
		      dev_name(dev), chip);
	if (ret) {
		dev_info(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}

	/* 8930 Boston_CR #:XXX, WH Lee, 20130424 */
	taos_create_kernel_debuglevel();
	/* WH Lee, 20130424 */

	dev_info(dev, "Probe ok.\n");
	return 0;

irq_register_fail:
	if (chip->a_idev) {
		remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
		/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
		pmlog_unregister_device(chip->als_pmlog_device);
#endif
		/* WH Lee, 20130107 */
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
	}
input_a_alloc_failed:
	if (chip->p_idev) {
		remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
		/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
		pmlog_unregister_device(chip->prx_pmlog_device);
#endif
		/* WH Lee, 20130107 */
input_p_sysfs_failed:
		input_unregister_device(chip->p_idev);
	}
input_p_alloc_failed:
flush_regs_failed:
id_failed:
	kfree(chip->segment);
set_segment_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	dev_err(dev, "Probe failed.\n");
	return ret;
}

static int taos_suspend(struct device *dev)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	struct tsl2772_i2c_platform_data *pdata = dev->platform_data;

	dev_dbg(dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	chip->in_suspend = 1;
	/* 8930 Boston_CR #:XXX, WH Lee, 20130219 */
	/* Fix for power consumption */
	chip->als_enabled_resume = chip->als_enabled;
	chip->prx_enabled_resume = chip->prx_enabled;

	if (chip->prx_enabled_resume) {
	/* Original code */
	//if (chip->p_idev && chip->p_idev->users) {
	/* WH Lee, 20130219 */
		if (pdata->proximity_can_wake) {
			dev_dbg(dev, "set wake on proximity\n");
			chip->wake_irq = 1;
		} else {
			dev_dbg(dev, "proximity off\n");
			taos_prox_enable(chip, 0);
		}
	}
	/* 8930 Boston_CR #:XXX, WH Lee, 20130219 */
	/* Fix for power consumption */
	if (chip->als_enabled_resume) {
	/* Original code */
	//if (chip->a_idev && chip->a_idev->users) {
	/* WH Lee, 20130219 */
		if (pdata->als_can_wake) {
			dev_dbg(dev, "set wake on als\n");
			chip->wake_irq = 1;
		} else {
			dev_dbg(dev, "als off\n");
			taos_als_enable(chip, 0);
		}
	}
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_dbg(dev, "powering off\n");
		pltf_power_off(chip);
	}
	mutex_unlock(&chip->lock);

	return 0;
}

/* Sapporo CR #:XXX, WH Lee, 20131030 */
static int taos_suspend_noirq(struct device *dev)
{
	int rc = 0;
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	mutex_lock(&chip->lock);
	if (chip->irq_pending) {
		rc = -EAGAIN;
	}
	mutex_unlock(&chip->lock);
	return rc;
}
/* WH Lee, 20131030 */

static int taos_resume(struct device *dev)
{
	struct tsl2772_chip *chip = dev_get_drvdata(dev);
	bool als_on, prx_on;
	int rc = 0;
	mutex_lock(&chip->lock);
	/* 8930 Boston_CR #:XXX, WH Lee, 20130219 */
	/* Fix for power consumption */
	prx_on = chip->prx_enabled_resume;
	als_on = chip->als_enabled_resume;
	/* Original code */
	//prx_on = chip->p_idev && chip->p_idev->users;
	//als_on = chip->a_idev && chip->a_idev->users;
	/* WH Lee, 20130219 */
	chip->in_suspend = 0;
	dev_dbg(dev, "%s: powerd %d, als: needed %d  enabled %d,"
			" prox: needed %d  enabled %d\n", __func__,
			!chip->unpowered, als_on, chip->als_enabled,
			prx_on, chip->prx_enabled);
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}
	if (chip->unpowered && (prx_on || als_on)) {
		dev_dbg(dev, "powering on\n");
		rc = power_on(chip);
		if (rc)
		{
			/*prx_on*/
			goto err_power;
		}
	}

	// if not use prox
	if (prx_on && !chip->prx_enabled)
		(void)taos_prox_enable(chip, 1);

	if (als_on && !chip->als_enabled)
		(void)taos_als_enable(chip, 1);
	if (chip->irq_pending) {
		dev_dbg(dev, "%s: pending interrupt\n", __func__);
		chip->irq_pending = 0;
		(void)taos_check_and_report(chip);
		enable_irq(chip->client->irq);
	}
err_power:
	mutex_unlock(&chip->lock);

	return 0;
}

static int __devexit taos_remove(struct i2c_client *client)
{
	struct tsl2772_chip *chip = i2c_get_clientdata(client);
	free_irq(client->irq, chip);
	if (chip->a_idev) {
		remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
		/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
		pmlog_unregister_device(chip->als_pmlog_device);
#endif
		/* WH Lee, 20130107 */
		input_unregister_device(chip->a_idev);
	}
	if (chip->p_idev) {
		remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
		/* 8930 Boston_CR #:XXX, WH Lee, 20130107 */
#ifdef CONFIG_PM_LOG
		pmlog_unregister_device(chip->prx_pmlog_device);
#endif
		/* WH Lee, 20130107 */
		input_unregister_device(chip->p_idev);
		/* 8930 Boston_CR #:810, WH Lee, 20130314 */
		/* Add wakelock before report p-sensor data */
		wake_lock_destroy(&chip->prox_wake_lock);
		/* WH Lee, 20130314 */
	}

	/* 8930 Boston_CR #:XXX, WH Lee, 20130424 */
	taos_destroy_kernel_debuglevel();
	/* WH Lee, 20130424 */

	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	kfree(chip->segment);
	kfree(chip);
	return 0;
}

static struct i2c_device_id taos_idtable[] = {
	{ "tsl2772", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, taos_idtable);

static const struct dev_pm_ops taos_pm_ops = {
	/* Sapporo CR #:XXX, WH Lee, 20131030 */
	.suspend_noirq = taos_suspend_noirq,
	/* WH Lee, 20131030 */
	.suspend = taos_suspend,
	.resume  = taos_resume,
};

static struct i2c_driver taos_driver = {
	.driver = {
		.name = "tsl2772",
		.pm = &taos_pm_ops,
	},
	.id_table = taos_idtable,
	.probe = taos_probe,
	.remove = __devexit_p(taos_remove),
};

static int __init taos_init(void)
{
	/* 8930 Boston_CR #:XXX, WH Lee, 20130115 */
	/* Add BootLog */
	int ret;
	printk("BootLog, +%s\n", __func__);
	ret = i2c_add_driver(&taos_driver);
	printk("BootLog, -%s, ret=%d\n", __func__,ret);
	return ret;
	/* Original code */
	//return i2c_add_driver(&taos_driver);
	/* WH Lee, 20130115 */

}

static void __exit taos_exit(void)
{
	i2c_del_driver(&taos_driver);
}

/* 8930 Boston_CR #:XXX, WH Lee, 20130322 */
/* init it after touch driver */
late_initcall(taos_init);
/* Original code */
//module_init(taos_init);
/* WH Lee, 20130322 */
module_exit(taos_exit);

MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonyericsson.com>");
MODULE_DESCRIPTION("TAOS tsl2772 ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");
