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
 *
 */

#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>
#include "devices.h"
#include "board-8930.h"
/* Jen Chang add for pn544 nfc driver */
#ifdef CONFIG_PN544_NFC
#include <linux/nfc/pn544.h>
#endif
/* Jen Chang, 20121217 */

/* Terry Cheng, 20121207, Add compiler option when porting boston {*/
#if defined(CONFIG_SPI_QUP)
/* The SPI configurations apply to GSBI 1*/
static struct gpiomux_setting spi_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting spi_suspended_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
#endif	//CONFIG_SPI_QUP
/* } Terry Cheng, 20121207, Add compiler option when porting boston */

static struct gpiomux_setting gsbi3_suspended_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi3_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
/* Terry Cheng, 20121207, Remove UART config in kernel since aboot already config {*/
#if 0
static struct gpiomux_setting gsbi5 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
#endif
/* Terry Cheng, 20121207, Remove UART config in kernel since aboot already config {*/

//Terry Cheng, 20130116, Remove for fine tune current consumption +
#if 0
static struct gpiomux_setting gsbi9 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	//Eric Liu+
	#ifdef CONFIG_BATTERY_27520
	.drv = GPIOMUX_DRV_8MA,
	#endif
	//Eric Liu-
};
#endif 
//Terry Cheng, 20130116, Remove for fine tune current consumption -
/* Terry Cheng , 20121207, Add compiler option for gsbi 10 {*/
#ifdef CONFIG_ISL9519_CHARGER
static struct gpiomux_setting gsbi10 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif	//CONFIG_ISL9519_CHARGER
/* } Terry Cheng , 20121207, Add compiler option for gsbi 10 */

//Terry Cheng, 20130116, Remove for fine tune current consumption +
#if 0
static struct gpiomux_setting gsbi12 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif 
//Terry Cheng, 20130116, Remove for fine tune current consumption -

static struct gpiomux_setting cdc_mclk = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* Terry Cheng, 20121207, Remove auxpcm config for Boston project {*/
#if 0
static struct gpiomux_setting audio_auxpcm[] = {
	/* Suspended state */
	{
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	/* Active state */
	{
		.func = GPIOMUX_FUNC_1,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
};
#endif
/* } Terry Cheng, 20121207, Remove auxpcm config for Boston project */

static struct gpiomux_setting audio_mbhc = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* Terry Cheng 20121211, Remove Audio speaker boost setting on Boston {*/
#if 0
static struct gpiomux_setting audio_spkr_boost = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
#endif
/* } Terry Cheng 20121211, Remove Audio speaker boost setting on Boston */

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
static struct gpiomux_setting gpio_eth_suspend_1_cfg = {
	.pull = GPIOMUX_PULL_DOWN,
	.drv = GPIOMUX_DRV_2MA,
	.func = GPIOMUX_FUNC_GPIO,
};

static struct gpiomux_setting gpio_eth_suspend_2_cfg = {
	.pull = GPIOMUX_PULL_NONE,
	.drv = GPIOMUX_DRV_2MA,
	.func = GPIOMUX_FUNC_GPIO,
};
#endif

static struct gpiomux_setting slimbus = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting wcnss_5wire_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5wire_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};
/* Emily Jiang, 20121105, Add for Focaltech FT5316 TouchScreen { */
#if 0
static struct gpiomux_setting atmel_resout_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting atmel_resout_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting atmel_ldo_en_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting atmel_ldo_en_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting atmel_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting atmel_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
#endif
/* } Emily Jiang, 20121105, Add for Focaltech FT5316 TouchScreen */

/* Terry Cheng, 201211, Add compiler option for USB OTG  { */
#if  defined(MSM8930_PHASE_2) && defined(CONFIG_USB_OTG)
static struct gpiomux_setting hsusb_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct msm_gpiomux_config msm8930_hsusb_configs[] = {
	{
		.gpio = 63,     /* HSUSB_EXTERNAL_5V_LDO_EN */
		.settings = {
			[GPIOMUX_SUSPENDED] = &hsusb_sus_cfg,
		},
	},
	{
		.gpio = 97,     /* HSUSB_5V_EN */
		.settings = {
			[GPIOMUX_SUSPENDED] = &hsusb_sus_cfg,
		},
	},
};
#endif	//MSM8930_PHASE_2 && CONFIG_USB_OTG
/* } Terry Cheng, 201211, Add compiler option for USB OTG  */

/* Terry Cheng, 20121207, Remove hap lvl shft config for Boston project {*/
#if 0
static struct gpiomux_setting hap_lvl_shft_suspended_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting hap_lvl_shft_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};
#endif	
/* } Terry Cheng, 20121207, Remove hap lvl shft config for Boston project */

static struct gpiomux_setting ap2mdm_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_status_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_errfatal_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting ap2mdm_kpdpwr_n_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdp_vsync_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdp_vsync_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct gpiomux_setting hdmi_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting hdmi_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting hdmi_active_2_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting hdmi_active_3_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting hdmi_active_4_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting hdmi_active_5_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_HIGH,
};

#endif	//CONFIG_FB_MSM_HDMI_MSM_PANEL

static struct gpiomux_setting sitar_reset = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

/* Boston CR #:XXX, WH Lee, 20121128 */
#ifdef CONFIG_SENSORS_TSL2772
static struct gpiomux_setting tsl2772_int_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
#endif
/* WH Lee, 20121128 */

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
static struct msm_gpiomux_config msm8960_ethernet_configs[] = {
	{
		.gpio = 89,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_eth_suspend_1_cfg,
		}
	},
	{
		.gpio = 90,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_eth_suspend_2_cfg,
		}
	},
};
#endif

//Terry Cheng, 20130116, Add for fine tune current consumption +
static struct gpiomux_setting gsbi12_i2c_suspended_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi12_i2c_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting gsbi9_i2c_suspended_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi9_i2c_active_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
//Terry Cheng, 20130116, Add for fine tune current consumption -

/* Terry Cheng, 20130711, Add sapporo smart cover config { */
#ifdef CONFIG_SMART_COVER_DETECTION
static struct gpiomux_setting gsbi1_i2c_suspended_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi1_i2c_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting samrt_cover_plug_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting samrt_cover_int_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting samrt_cover_3v3_en_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct msm_gpiomux_config msm8960_smart_cover_configs[] __initdata = {
	{
		.gpio      = 43,		/*COVER_DET */
		.settings = {
			[GPIOMUX_SUSPENDED] = &samrt_cover_plug_suspend_cfg,
		},
	},
	{
		.gpio      = 63,		/* SCOVER_3V3_EN*/
		.settings = {
			[GPIOMUX_SUSPENDED] = &samrt_cover_3v3_en_suspend_cfg,
		},
	},
	{
		.gpio      = 67,		/* SCOVER_nINT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &samrt_cover_int_suspend_cfg,
		},
	},	
};	
#endif //CONFIG_SMART_COVER_DETECTION
/* } Terry Cheng, 20130711, Add sapporo smart cover config */

static struct msm_gpiomux_config msm8960_gsbi_configs[] __initdata = {
	//Terry Cheng, 20121207, Add compiler option when porting Boston
	#if defined(CONFIG_SPI_QUP)
	{
		.gpio      = 6,		/* GSBI1 QUP SPI_DATA_MOSI */
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE] = &spi_active,
		},
	},
	{
		.gpio      = 7,		/* GSBI1 QUP SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE] = &spi_active,
		},
	},
	{
		.gpio      = 8,		/* GSBI1 QUP SPI_CS_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE] = &spi_active,
		},
	},
	{
		.gpio      = 9,		/* GSBI1 QUP SPI_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE] = &spi_active,
		},
	},
	#endif	//CONFIG_SPI_QUP
	/* Terry Cheng, 20130711, Add gsbi1 i2c config { */
	#ifdef CONFIG_SMART_COVER_DETECTION
	//Sapporo smart cover
	{
		.gpio      = 8,		/* GSBI1 I2C QUP SDA*/
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi1_i2c_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi1_i2c_active_cfg,
		},
	},
	{
		.gpio      = 9,		/* GSBI1 I2C QUP SCL  */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi1_i2c_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi1_i2c_active_cfg,
		},
	},
	/* } Terry Cheng, 20130711, Add gsbi1 i2c config  */
	#endif //CONFIG_SMART_COVER_DETECTION		
	//Boston Touch
	{
		.gpio      = 16,	/* GSBI3 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi3_active_cfg,
		},
	},
	{
		.gpio      = 17,	/* GSBI3 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi3_active_cfg,
		},
	},
/* Terry Cheng, 20120323, Remove UART config in kernel since aboot already config {*/
#if 0
	{
		.gpio      = 22,	/* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5,
		},
	},
	{
		.gpio      = 23,	/* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5,
		},
	},
#endif
/* } Terry Cheng, 20120323, Remove UART config in kernel since aboot already config */
	//Boston Sensor
	{
		.gpio      = 44,	/* GSBI12 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi12_i2c_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi12_i2c_active_cfg,
		},
	},
	{
		.gpio      = 45,	/* GSBI12 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi12_i2c_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi12_i2c_active_cfg,
		},
	},
	//Boston Gauge I2C
	{
		.gpio      = 95,	/* GSBI9 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi9_i2c_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi9_i2c_active_cfg,
		},
	},
	{
		.gpio      = 96,	/* GSBI9 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi9_i2c_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi9_i2c_active_cfg,
		},
	},

	/* Terry Cheng , 20121207, Add compiler option for gsbi 10 {*/
	#ifdef CONFIG_ISL9519_CHARGER
	{
		.gpio      = 73,	/* GSBI10 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},
	{
		.gpio      = 74,	/* GSBI10 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},
	#endif //CONFIG_ISL9519_CHARGER
	/* } Terry Cheng , 20121207, Add compiler option for gsbi 10 */
	
};

static struct msm_gpiomux_config msm8960_slimbus_config[] __initdata = {
	{
		.gpio	= 60,		/* slimbus data */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio	= 61,		/* slimbus clk */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
};

static struct msm_gpiomux_config msm8960_audio_codec_configs[] __initdata = {
	{
		.gpio = 59,
		.settings = {
			[GPIOMUX_SUSPENDED] = &cdc_mclk,
		},
	},
};

static struct msm_gpiomux_config msm8960_audio_mbhc_configs[] __initdata = {
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_mbhc,
		},
	},
};
/* Terry Cheng 20121211, Remove Audio speaker boost setting on Boston {*/
#if 0
static struct msm_gpiomux_config msm8960_audio_spkr_configs[] __initdata = {
	{
		.gpio = 15,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_spkr_boost,
		},
	},
};
#endif
/* } Terry Cheng 20121211, Remove Audio speaker boost setting on Boston */

/* Terry Cheng, 20121207, Remove auxpcm config for Boston project {*/
#if 0
static struct msm_gpiomux_config msm8960_audio_auxpcm_configs[] __initdata = {
	{
		.gpio = 63,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_auxpcm[0],
			[GPIOMUX_ACTIVE] = &audio_auxpcm[1],
		},
	},
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_auxpcm[0],
			[GPIOMUX_ACTIVE] = &audio_auxpcm[1],
		},
	},
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_auxpcm[0],
			[GPIOMUX_ACTIVE] = &audio_auxpcm[1],
		},
	},
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_SUSPENDED] = &audio_auxpcm[0],
			[GPIOMUX_ACTIVE] = &audio_auxpcm[1],
		},
	},
};
#endif
/* Terry Cheng, 20121207, Remove auxpcm config for Boston project {*/


static struct msm_gpiomux_config wcnss_5wire_interface[] = {
	{
		.gpio = 84,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 85,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 86,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 87,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 88,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
};

/* Emily Jiang, 20121105, Add for Focaltech FT5316 TouchScreen { */
#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_FT5316) || defined(CONFIG_TOUCHSCREEN_SYNAPTICS_S3202)
#include <linux/input/focaltech_ft5316_ts.h>

/* Terry Cheng, 20130116, Change config to fine tune current consumption {*/
static struct gpiomux_setting focaltech_irq_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir  = GPIOMUX_IN,
};
/* } Terry Cheng, 20130116, Change config to fine tune current consumption */
static struct gpiomux_setting focaltech_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN, // Terry Cheng, 20130116, Change config to fine tune current consumption
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting focaltech_model_id_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir  = GPIOMUX_IN,
};
static struct msm_gpiomux_config msm8960_focaltech_ts_configs[] __initdata = {
	{	/* TS INTERRUPT */
		.gpio = TS_IRQ_GPIO_NUM,
		.settings = {
			//[GPIOMUX_ACTIVE]    = &atmel_int_act_cfg,
			[GPIOMUX_SUSPENDED] = &focaltech_irq_suspend_cfg, // Terry Cheng, 20130116, Change config to fine tune current consumption
		},
	},
	{	/* TS LDO ENABLE */
		.gpio = TS_VENDOR_ID_GPIO_NUM,
		.settings = {
			//[GPIOMUX_ACTIVE]    = &atmel_ldo_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &focaltech_model_id_cfg,
		},
	},
	{	/* TS RESOUT */
		.gpio = TS_RST_GPIO_NUM,
		.settings = {
			//[GPIOMUX_ACTIVE]    = &atmel_resout_act_cfg,
			[GPIOMUX_SUSPENDED] = &focaltech_suspend_cfg,
		},
	},
};
#endif //CONFIG_TOUCHSCREEN_FOCALTECH_FT5316
/* } Emily Jiang, 20121105, Add for Focaltech FT5316 TouchScreen */

/* Terry Cheng, 20121207, Remove hap lvl shft config for Boston project {*/
#if 0
static struct msm_gpiomux_config hap_lvl_shft_config[] __initdata = {
	{
		.gpio = 47,
		.settings = {
			[GPIOMUX_SUSPENDED] = &hap_lvl_shft_suspended_config,
			[GPIOMUX_ACTIVE] = &hap_lvl_shft_active_config,
		},
	},
};
#endif 
/* } Terry Cheng, 20121207, Remove hap lvl shft config for Boston project */


static struct msm_gpiomux_config mdm_configs[] __initdata = {
	/* AP2MDM_STATUS */
	{
		.gpio = 94,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* MDM2AP_STATUS */
	{
		.gpio = 69,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_status_cfg,
		}
	},
	/* MDM2AP_ERRFATAL */
	{
		.gpio = 70,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_errfatal_cfg,
		}
	},
	/* AP2MDM_ERRFATAL */
	{
		.gpio = 95,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* AP2MDM_KPDPWR_N */
	{
		.gpio = 81,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_kpdpwr_n_cfg,
		}
	},
	/* AP2MDM_PMIC_RESET_N */
	{
		.gpio = 80,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_kpdpwr_n_cfg,
		}
	}
};

static struct msm_gpiomux_config msm8960_mdp_vsync_configs[] __initdata = {
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE]    = &mdp_vsync_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdp_vsync_suspend_cfg,
		},
	}
};

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct msm_gpiomux_config msm8960_hdmi_configs[] __initdata = {
	{
		.gpio = 99,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 100,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 101,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 102,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},

};

static struct msm_gpiomux_config msm8930_mhl_configs[] __initdata = {
	{
		.gpio = 72,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 71,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_4_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 73,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_5_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},

};
#endif	//CONFIG_FB_MSM_HDMI_MSM_PANEL

//Terry Cheng, 20121207, Add compiler option when porting Boston
#ifdef CONFIG_HAPTIC_ISA1200
static struct gpiomux_setting haptics_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting haptics_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm8930_haptics_configs[] __initdata = {
	{
		.gpio = 77,
		.settings = {
			[GPIOMUX_ACTIVE] = &haptics_active_cfg,
			[GPIOMUX_SUSPENDED] = &haptics_suspend_cfg,
		},
	},
	{
		.gpio = 78,
		.settings = {
			[GPIOMUX_ACTIVE] = &haptics_active_cfg,
			[GPIOMUX_SUSPENDED] = &haptics_suspend_cfg,
		},
	},
};
#endif //CONFIG_HAPTIC_ISA1200

static struct gpiomux_setting sd_det_line = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm8930_sd_det_config[] __initdata = {
	{
		.gpio = 94,	/* SD Card Detect Line */
		.settings = {
			[GPIOMUX_SUSPENDED] = &sd_det_line,
			[GPIOMUX_ACTIVE] = &sd_det_line,
		},
	},
};

static struct gpiomux_setting gyro_int_line = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,// Terry Cheng, 20130116, Change config to fine tune current consumption
};

static struct msm_gpiomux_config msm8930_gyro_int_config[] __initdata = {
	{
		.gpio = 69,	/* Gyro Interrupt Line */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gyro_int_line,
			[GPIOMUX_ACTIVE] = &gyro_int_line,
		},
	},
};

static struct msm_gpiomux_config msm_sitar_config[] __initdata = {
	{
		.gpio   = 42,           /* SYS_RST_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &sitar_reset,
		},
	}
};

/* Boston CR #:XXX, WH Lee, 20121128 */
#ifdef CONFIG_SENSORS_TSL2772
static struct msm_gpiomux_config tsl2772_int_configs[] __initdata = {
	{
		.gpio   = 49,           /* PROXIMITY_INT_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &tsl2772_int_cfg,
		},
	}
};
#endif
/* WH Lee, 20121128 */

//Eric Liu+
#ifdef CONFIG_BATTERY_27520
static struct gpiomux_setting battery_27520_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,  //no pull
		.drv  = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN, // Terry Cheng, 20130116, Change config to fine tune current consumption
		.dir  = GPIOMUX_IN,
	},
	{
		.func = GPIOMUX_FUNC_GPIO,  //open drain
		.drv  = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
		.dir  = GPIOMUX_IN,
	},
};
static struct msm_gpiomux_config msm8960_battery_27520_configs[] __initdata = {
	{
		.gpio = 110,  //gpio_gag_int (battery_27520.c)
		.settings = {
			[GPIOMUX_ACTIVE]    = &battery_27520_settings[1],
			[GPIOMUX_SUSPENDED] = &battery_27520_settings[0],
		},
	},
	{
		.gpio = 109,  //gpio_bat_low (battery_27520.c)
		.settings = {
			[GPIOMUX_ACTIVE]    = &battery_27520_settings[0],
			[GPIOMUX_SUSPENDED] = &battery_27520_settings[0],
		},
	},
};
#endif
//Eric Liu-

/* Jen Chang add for pn544 nfc driver */
#ifdef CONFIG_PN544_NFC
static struct gpiomux_setting pn544_nfc_irq_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,
		.drv  = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir  = GPIOMUX_IN,
	},
};

static struct gpiomux_setting pn544_nfc_enable_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,
		.drv  = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir  = GPIOMUX_OUT_LOW,
	},
};

static struct gpiomux_setting pn544_nfc_fw_reset_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,
		.drv  = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir  = GPIOMUX_OUT_LOW,
	},
};

static struct msm_gpiomux_config msm8960_pn544_nfc_configs[] __initdata = {
	{
		.gpio = PN544_HOST_INT_GPIO, //irq gpio
		.settings = {
			[GPIOMUX_ACTIVE]    = &pn544_nfc_irq_settings[0],
			[GPIOMUX_SUSPENDED] = &pn544_nfc_irq_settings[0],
		},
	},
	{
		.gpio = PN544_ENABLE_GPIO, //enable gpio (VEN)
		.settings = {
			[GPIOMUX_ACTIVE]    = &pn544_nfc_enable_settings[0],
			[GPIOMUX_SUSPENDED] = &pn544_nfc_enable_settings[0],
		},
	},
	{
		.gpio = PN544_FW_RESET_GPIO, //fw update gpio (GPIO4)
		.settings = {
			[GPIOMUX_ACTIVE]    = &pn544_nfc_fw_reset_settings[0],
			[GPIOMUX_SUSPENDED] = &pn544_nfc_fw_reset_settings[0],
		},
	},
};
#endif
/* Jen Chang, 20121217 */

int __init msm8930_init_gpiomux(void)
{
	int rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err(KERN_ERR "msm_gpiomux_init failed %d\n", rc);
		return rc;
	}

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	msm_gpiomux_install(msm8960_ethernet_configs,
			ARRAY_SIZE(msm8960_ethernet_configs));
#endif

	msm_gpiomux_install(msm8960_gsbi_configs,
			ARRAY_SIZE(msm8960_gsbi_configs));
/* Emily Jiang, 20121105, Add for Focaltech FT5316 TouchScreen { */
#if defined(CONFIG_TOUCHSCREEN_FOCALTECH_FT5316) || defined(CONFIG_TOUCHSCREEN_SYNAPTICS_S3202)
	msm_gpiomux_install(msm8960_focaltech_ts_configs,
			ARRAY_SIZE(msm8960_focaltech_ts_configs));
#endif //CONFIG_TOUCHSCREEN_FOCALTECH_FT5316
/* } Emily Jiang, 20121105, Add for Focaltech FT5316 TouchScreen */
	msm_gpiomux_install(msm8960_slimbus_config,
			ARRAY_SIZE(msm8960_slimbus_config));

	msm_gpiomux_install(msm8960_audio_codec_configs,
			ARRAY_SIZE(msm8960_audio_codec_configs));

	msm_gpiomux_install(msm8960_audio_mbhc_configs,
			ARRAY_SIZE(msm8960_audio_mbhc_configs));

/* Terry Cheng 20121211, Remove Audio speaker boost setting on Boston {*/
#if 0
	msm_gpiomux_install(msm8960_audio_spkr_configs,
			ARRAY_SIZE(msm8960_audio_spkr_configs));
#endif	
/* } Terry Cheng 20121211, Remove Audio speaker boost setting on Boston */

/* Terry Cheng, 20121207, Remove auxpcm config for Boston project {*/
#if 0
	msm_gpiomux_install(msm8960_audio_auxpcm_configs,
			ARRAY_SIZE(msm8960_audio_auxpcm_configs));
#endif
/* } Terry Cheng, 20121207, Remove auxpcm config for Boston project */

	msm_gpiomux_install(wcnss_5wire_interface,
			ARRAY_SIZE(wcnss_5wire_interface));

	if (machine_is_msm8930_mtp() || machine_is_msm8930_fluid() ||
		machine_is_msm8930_cdp()) {
/* Terry Cheng, 20121207, Remove hap lvl shft config for Boston project {*/
#if 0		
		msm_gpiomux_install(hap_lvl_shft_config,
			ARRAY_SIZE(hap_lvl_shft_config));
#endif
/* } Terry Cheng, 20121207, Remove hap lvl shft config for Boston project {*/

/* Terry Cheng, 201211, Add compiler option for USB OTG  { */
#if  defined(MSM8930_PHASE_2) && defined(CONFIG_USB_OTG)
		msm_gpiomux_install(msm8930_hsusb_configs,
			ARRAY_SIZE(msm8930_hsusb_configs));
#endif	//MSM8930_PHASE_2 && CONFIG_USB_OTG
/* } Terry Cheng, 201211, Add compiler option for USB OTG   */
	}

	if (PLATFORM_IS_CHARM25())
		msm_gpiomux_install(mdm_configs,
			ARRAY_SIZE(mdm_configs));

//Terry Cheng, 20121207, Add compiler option when porting Boston
#ifdef CONFIG_HAPTIC_ISA1200
	if (machine_is_msm8930_cdp() || machine_is_msm8930_mtp()
		|| machine_is_msm8930_fluid())
		msm_gpiomux_install(msm8930_haptics_configs,
			ARRAY_SIZE(msm8930_haptics_configs));
#endif	//CONFIG_HAPTIC_ISA1200

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	msm_gpiomux_install(msm8960_hdmi_configs,
			ARRAY_SIZE(msm8960_hdmi_configs));
	if (msm8930_mhl_display_enabled())
		msm_gpiomux_install(msm8930_mhl_configs,
				ARRAY_SIZE(msm8930_mhl_configs));
#endif

	msm_gpiomux_install(msm8960_mdp_vsync_configs,
			ARRAY_SIZE(msm8960_mdp_vsync_configs));

	msm_gpiomux_install(msm8930_sd_det_config,
			ARRAY_SIZE(msm8930_sd_det_config));

	if (machine_is_msm8930_fluid() || machine_is_msm8930_mtp())
		msm_gpiomux_install(msm8930_gyro_int_config,
			ARRAY_SIZE(msm8930_gyro_int_config));

	msm_gpiomux_install(msm_sitar_config, ARRAY_SIZE(msm_sitar_config));

	/* Boston CR #:XXX, WH Lee, 20121128 */
#ifdef CONFIG_SENSORS_TSL2772
	msm_gpiomux_install(tsl2772_int_configs,
			ARRAY_SIZE(tsl2772_int_configs));
#endif
	/* WH Lee, 20121128 */

//Eric Liu+
#ifdef CONFIG_BATTERY_27520
	msm_gpiomux_install(msm8960_battery_27520_configs,
			ARRAY_SIZE(msm8960_battery_27520_configs));
#endif
//Eric Liu-

/* Jen Chang add for pn544 nfc driver */
#ifdef CONFIG_PN544_NFC
	msm_gpiomux_install(msm8960_pn544_nfc_configs,
			ARRAY_SIZE(msm8960_pn544_nfc_configs));
#endif
/* Jen Chang, 20121217 */

/* Terry Cheng, 20130711, Add sapporo smart cover config { */
#ifdef CONFIG_SMART_COVER_DETECTION
	msm_gpiomux_install(msm8960_smart_cover_configs,
			ARRAY_SIZE(msm8960_smart_cover_configs));
#endif 	//CONFIG_SMART_COVER_DETECTION
/* } Terry Cheng, 20130711, Add sapporo smart cover config  */

	return 0;
}
