/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <linux/interrupt.h>
#include <linux/mfd/pm8xxx/pm8038.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/msm_ssbi.h>
#include <asm/mach-types.h>
#include <mach/msm_bus_board.h>
#include <mach/restart.h>
#include <mach/socinfo.h>
#include "devices.h"
#include "board-8930.h"

struct pm8xxx_gpio_init {
	unsigned			gpio;
	struct pm_gpio			config;
};

struct pm8xxx_mpp_init {
	unsigned			mpp;
	struct pm8xxx_mpp_config_data	config;
};

#define PM8038_GPIO_INIT(_gpio, _dir, _buf, _val, _pull, _vin, _out_strength, \
			_func, _inv, _disable) \
{ \
	.gpio	= PM8038_GPIO_PM_TO_SYS(_gpio), \
	.config	= { \
		.direction	= _dir, \
		.output_buffer	= _buf, \
		.output_value	= _val, \
		.pull		= _pull, \
		.vin_sel	= _vin, \
		.out_strength	= _out_strength, \
		.function	= _func, \
		.inv_int_pol	= _inv, \
		.disable_pin	= _disable, \
	} \
}

#define PM8038_MPP_INIT(_mpp, _type, _level, _control) \
{ \
	.mpp	= PM8038_MPP_PM_TO_SYS(_mpp), \
	.config	= { \
		.type		= PM8XXX_MPP_TYPE_##_type, \
		.level		= _level, \
		.control	= PM8XXX_MPP_##_control, \
	} \
}

#define PM8038_GPIO_DISABLE(_gpio) \
	PM8038_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, 0, 0, 0, PM8038_GPIO_VIN_L11, \
			 0, 0, 0, 1)

#define PM8038_GPIO_OUTPUT(_gpio, _val) \
	PM8038_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM8038_GPIO_VIN_L11, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8038_GPIO_INPUT(_gpio, _pull) \
	PM8038_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, PM_GPIO_OUT_BUF_CMOS, 0, \
			_pull, PM8038_GPIO_VIN_L11, \
			PM_GPIO_STRENGTH_NO, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8038_GPIO_OUTPUT_FUNC(_gpio, _val, _func) \
	PM8038_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM8038_GPIO_VIN_L11, \
			PM_GPIO_STRENGTH_HIGH, \
			_func, 0, 0)

#define PM8038_GPIO_OUTPUT_VIN(_gpio, _val, _vin) \
	PM8038_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, _vin, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8917_GPIO_INIT(_gpio, _dir, _buf, _val, _pull, _vin, _out_strength, \
			_func, _inv, _disable) \
{ \
	.gpio	= PM8917_GPIO_PM_TO_SYS(_gpio), \
	.config	= { \
		.direction	= _dir, \
		.output_buffer	= _buf, \
		.output_value	= _val, \
		.pull		= _pull, \
		.vin_sel	= _vin, \
		.out_strength	= _out_strength, \
		.function	= _func, \
		.inv_int_pol	= _inv, \
		.disable_pin	= _disable, \
	} \
}

#define PM8917_MPP_INIT(_mpp, _type, _level, _control) \
{ \
	.mpp	= PM8917_MPP_PM_TO_SYS(_mpp), \
	.config	= { \
		.type		= PM8XXX_MPP_TYPE_##_type, \
		.level		= _level, \
		.control	= PM8XXX_MPP_##_control, \
	} \
}

#define PM8917_GPIO_DISABLE(_gpio) \
	PM8917_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, 0, 0, 0, PM_GPIO_VIN_S4, \
			 0, 0, 0, 1)

#define PM8917_GPIO_OUTPUT(_gpio, _val) \
	PM8917_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8917_GPIO_INPUT(_gpio, _pull) \
	PM8917_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, PM_GPIO_OUT_BUF_CMOS, 0, \
			_pull, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_NO, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8917_GPIO_OUTPUT_FUNC(_gpio, _val, _func) \
	PM8917_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_HIGH, \
			_func, 0, 0)

#define PM8917_GPIO_OUTPUT_VIN(_gpio, _val, _vin) \
	PM8917_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, _vin, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

/* GPIO and MPP configurations for MSM8930 + PM8038 targets */

/* Initial PM8038 GPIO configurations */
static struct pm8xxx_gpio_init pm8038_gpios[] __initdata = {
	/* keys GPIOs */
	PM8038_GPIO_INPUT(3, PM_GPIO_PULL_UP_30),
	PM8038_GPIO_INPUT(8, PM_GPIO_PULL_UP_30),
	PM8038_GPIO_INPUT(10, PM_GPIO_PULL_UP_30),
	PM8038_GPIO_INPUT(11, PM_GPIO_PULL_UP_30),
	/* haptics gpio */
	PM8038_GPIO_OUTPUT_FUNC(7, 0, PM_GPIO_FUNC_1),
	/* MHL PWR EN */
	PM8038_GPIO_OUTPUT_VIN(5, 1, PM8038_GPIO_VIN_VPH),
};

/* Initial PM8038 MPP configurations */
static struct pm8xxx_mpp_init pm8038_mpps[] __initdata = {
};

/*  Terry Cheng, 20121127, Port Boston EVT0 PMIC 8038 GPIO and MPP config {*/
static struct pm8xxx_gpio_init pm8038_gpios_boston_evt0[] __initdata = {
};
//GPIO configuration with detroit lte NC pin
static struct pm8xxx_gpio_init pm8038_gpios_boston_evt0_nc_pins[] __initdata = {
	PM8038_GPIO_DISABLE(1),				 	 /* Disable NC */
	PM8038_GPIO_DISABLE(2),				 	 /* Disable NC */
	PM8038_GPIO_DISABLE(5),				 	 /* Disable NC */	
	PM8038_GPIO_DISABLE(6),				 	 /* Disable NC */
	PM8038_GPIO_DISABLE(7),				 	 /* Disable NC */
	PM8038_GPIO_DISABLE(10),				 	 /* Disable NC */
	PM8038_GPIO_DISABLE(11),				 	 /* Disable NC */
	PM8038_GPIO_DISABLE(12),				 	 /* Disable NC */
};
static struct pm8xxx_mpp_init pm8038_mpps_boston_evt0_nc_pins[] __initdata = {
	PM8038_MPP_INIT(6, SINK, PM8XXX_MPP_CS_OUT_25MA, CS_CTRL_DISABLE),
};
/* } Terry Cheng, 20121127, Port Boston EVT0 PMIC 8038 GPIO and MPP config */

/* GPIO and MPP configurations for MSM8930 + PM8917 targets */

/* Initial PM8917 GPIO configurations */
static struct pm8xxx_gpio_init pm8917_gpios[] __initdata = {
	/* Backlight enable control */
	PM8917_GPIO_OUTPUT(24, 1),
	/* keys GPIOs */
	PM8917_GPIO_INPUT(27, PM_GPIO_PULL_UP_30),
	PM8917_GPIO_INPUT(28, PM_GPIO_PULL_UP_30),
	PM8917_GPIO_INPUT(36, PM_GPIO_PULL_UP_30),
	PM8917_GPIO_INPUT(37, PM_GPIO_PULL_UP_30),
	/* haptics gpio */
	PM8917_GPIO_OUTPUT_FUNC(38, 0, PM_GPIO_FUNC_2),
	/* MHL PWR EN */
	PM8917_GPIO_OUTPUT_VIN(25, 1, PM_GPIO_VIN_VPH),
};

/* Initial PM8917 MPP configurations */
static struct pm8xxx_mpp_init pm8917_mpps[] __initdata = {
	/* Configure MPP01 for USB ID detection */
	PM8917_MPP_INIT(1, D_INPUT, PM8921_MPP_DIG_LEVEL_S4, DIN_TO_INT),
};

void __init msm8930_pm8038_gpio_mpp_init(void)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(pm8038_gpios); i++) {
		rc = pm8xxx_gpio_config(pm8038_gpios[i].gpio,
					&pm8038_gpios[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_gpio_config: rc=%d\n", __func__, rc);
			break;
		}
	}

	/* Initial MPP configuration. */
	for (i = 0; i < ARRAY_SIZE(pm8038_mpps); i++) {
		rc = pm8xxx_mpp_config(pm8038_mpps[i].mpp,
					&pm8038_mpps[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_mpp_config: rc=%d\n", __func__, rc);
			break;
		}
	}
	/*  Terry Cheng, 20121127, Port Boston EVT0 PMIC 8038 GPIO and MPP config {*/
	printk("system_rev = 0x%x\n", system_rev);
	if ( msm_project_id >= BOSTON )
	{
		if(system_rev >= EVT0){
			//Boston evt0 pmic 8038 gpio config
			for (i = 0; i < ARRAY_SIZE(pm8038_gpios_boston_evt0); i++) {
				rc = pm8xxx_gpio_config(pm8038_gpios_boston_evt0[i].gpio,
							&pm8038_gpios_boston_evt0[i].config);
				if (rc) {
					pr_err("%s at %d: pm8xxx_gpio_config: rc=%d\n", __func__, __LINE__, rc);
					break;
				}
			}
			//Boston evt0 pmic 8038 gpio NC pin
			for (i = 0; i < ARRAY_SIZE(pm8038_gpios_boston_evt0_nc_pins); i++) {
				rc = pm8xxx_gpio_config(pm8038_gpios_boston_evt0_nc_pins[i].gpio,
							&pm8038_gpios_boston_evt0_nc_pins[i].config);
				if (rc) {
					pr_err("%s at %d: pm8xxx_gpio_config: rc=%d\n", __func__, __LINE__, rc);
					break;
				}
			}
			//Boston evt0 pmic 8038 mpp NC pin
			for (i = 0; i < ARRAY_SIZE(pm8038_mpps_boston_evt0_nc_pins); i++) {
				rc = pm8xxx_mpp_config(pm8038_mpps_boston_evt0_nc_pins[i].mpp,
							&pm8038_mpps_boston_evt0_nc_pins[i].config);
				if (rc) {
					pr_err("%s at %d: pm8xxx_mpp_config: rc=%d\n", __func__, __LINE__, rc);
					break;
				}
			}
		}
	}
	/* } Terry Cheng, 20121127, Port Boston EVT0 PMIC 8038 GPIO and MPP config */	
}

void __init msm8930_pm8917_gpio_mpp_init(void)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(pm8917_gpios); i++) {
		rc = pm8xxx_gpio_config(pm8917_gpios[i].gpio,
					&pm8917_gpios[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_gpio_config: rc=%d\n", __func__, rc);
			break;
		}
	}

	/* Initial MPP configuration. */
	for (i = 0; i < ARRAY_SIZE(pm8917_mpps); i++) {
		rc = pm8xxx_mpp_config(pm8917_mpps[i].mpp,
					&pm8917_mpps[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_mpp_config: rc=%d\n", __func__, rc);
			break;
		}
	}
}

static struct pm8xxx_adc_amux pm8038_adc_channels_data[] = {
	{"vcoin", CHANNEL_VCOIN, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vbat", CHANNEL_VBAT, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"dcin", CHANNEL_DCIN, CHAN_PATH_SCALING4, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ichg", CHANNEL_ICHG, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vph_pwr", CHANNEL_VPH_PWR, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ibat", CHANNEL_IBAT, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"batt_therm", CHANNEL_BATT_THERM, CHAN_PATH_SCALING1, AMUX_RSV2,
		ADC_DECIMATION_TYPE2, ADC_SCALE_BATT_THERM},
	{"batt_id", CHANNEL_BATT_ID, CHAN_PATH_SCALING1, AMUX_RSV2,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"usbin", CHANNEL_USBIN, CHAN_PATH_SCALING3, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"pmic_therm", CHANNEL_DIE_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PMIC_THERM},
	{"625mv", CHANNEL_625MV, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"125v", CHANNEL_125V, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"chg_temp", CHANNEL_CHG_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"pa_therm1", ADC_MPP_1_AMUX4, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PA_THERM},
	{"xo_therm", CHANNEL_MUXOFF, CHAN_PATH_SCALING1, AMUX_RSV0,
		ADC_DECIMATION_TYPE2, ADC_SCALE_XOTHERM},
	{"pa_therm0", ADC_MPP_1_AMUX3, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PA_THERM},
	{"mpp_amux6", ADC_MPP_2_AMUX6, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT}, //Sonia add it to initialize mpp_amux6
};

static struct pm8xxx_adc_properties pm8038_adc_data = {
	.adc_vdd_reference	= 1800, /* milli-voltage for this adc */
	.bitresolution		= 15,
	.bipolar                = 0,
};

static struct pm8xxx_adc_platform_data pm8038_adc_pdata = {
	.adc_channel            = pm8038_adc_channels_data,
	.adc_num_board_channel  = ARRAY_SIZE(pm8038_adc_channels_data),
	.adc_prop               = &pm8038_adc_data,
	.adc_mpp_base		= PM8038_MPP_PM_TO_SYS(1),
};

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata __devinitdata = {
	.irq_base		= PM8038_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(104),
	.irq_trigger_flag	= IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata __devinitdata = {
	.gpio_base	= PM8038_GPIO_PM_TO_SYS(1),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata __devinitdata = {
	.mpp_base	= PM8038_MPP_PM_TO_SYS(1),
};

static struct pm8xxx_rtc_platform_data pm8xxx_rtc_pdata __devinitdata = {
	.rtc_write_enable	= true,
	.rtc_alarm_powerup	= true,
};

static struct pm8xxx_pwrkey_platform_data pm8xxx_pwrkey_pdata = {
	.pull_up		= 1,
	.kpd_trigger_delay_us	= 15625,
	.wakeup			= 1,
};

static int pm8921_therm_mitigation[] = {
	1000, //Carl Chang, Sapporo HW require max 1000mA charge to battery
	700,
	600,
	325,
};
//Carl Chang, 20130503, HW team spec
//  t<0,        stop charge
//  0<t<5,     4.35V 500mA charge
//  5<t<45,    4.35V 1000mA charge
//  45<t<55,   4.25V 1000mA charge
//  t>55,       stop charge
#define MAX_VOLTAGE_MV		4350  //Eric Liu, Boston use 4.35V battery
#define CHG_TERM_MA		100
static struct pm8921_charger_platform_data pm8921_chg_pdata __devinitdata = {
	.update_time		= 60000,
	.max_voltage		= MAX_VOLTAGE_MV,
	.min_voltage		= 3400,       //Eric Liu, Boston use 3.4V as 0%
	.uvd_thresh_voltage	= 4050,
	.alarm_low_mv		= 3400,
	.alarm_high_mv		= 4000,
	.resume_voltage_delta	= 100,  //Carl Chang, Boston HW require vbatdet = 4.25V
	.resume_charge_percent	= 99,
	.term_current		= CHG_TERM_MA,
	.cool_temp		= 5,   //Carl Chang, set cool_temp to 5'C
	.warm_temp		= 45,  //Carl Chang, set warm_temp to 45'C
	.temp_check_period	= 1,
	.max_bat_chg_current	= 1000, //Carl Chang, Sapporo HW require max 1000mA charge to battery
	.cool_bat_chg_current	= 500, //Carl Chang, Boston 500mA charge to battery ,when cool
	.warm_bat_chg_current	= 1000,//Carl Chang, Sapporo 1000mA charge to battery ,when warm
	.cool_bat_voltage	= 4350,//Carl Chang, Boston use 4.35V ,when cool
	.warm_bat_voltage	= 4250,//Carl Chang, Boston use 4.25V ,when warm
	.thermal_mitigation	= pm8921_therm_mitigation,
	.thermal_levels		= ARRAY_SIZE(pm8921_therm_mitigation),
	.led_src_config		= LED_SRC_VPH_PWR, //Carl Chang, enable the LED power from VPH_PWR
	.rconn_mohm		= 37,  //Carl Chang, Boston HW require 37 mOhm

	//Eric Liu+, 20130409, use btc_override to replace HW charging protect
	.btc_override = 1,
	.btc_override_cold_degc = 0,  // t < 0'c
	.btc_override_hot_degc = 55,  // t> 55'c , Carl Chang, Boston HW require hot_degc to 55'C
	.btc_delay_ms = 10000,        // polling interval = 10s
	.btc_panic_if_cant_stop_chg = 0,
	//Eric Liu-
};

#if defined CONFIG_FB_MSM_MIPI_DSI_TRULY_OTM9608A || defined CONFIG_FB_MSM_MIPI_DSI_TRUST_NT35516
#define PM8038_WLED_MAX_CURRENT		20
#else
#define PM8038_WLED_MAX_CURRENT		25
#endif
#define PM8XXX_LED_PWM_PERIOD		1000
#define PM8XXX_LED_PWM_DUTY_MS		20
#define PM8038_RGB_LED_MAX_CURRENT	1  /* I = 1mA */

static struct led_info pm8038_led_info[] = {
	[0] = {
		.name			= "wled",
		.default_trigger	= "bkl_trigger",
	},
	[1] = {
		.name			= "led:rgb_red",
	},
	[2] = {
		.name			= "led:rgb_green",
	},
	[3] = {
		.name			= "led:rgb_blue",
	},
};

static struct led_platform_data pm8038_led_core_pdata = {
	.num_leds = ARRAY_SIZE(pm8038_led_info),
	.leds = pm8038_led_info,
};

#if defined CONFIG_FB_MSM_MIPI_DSI_TRULY_OTM9608A || defined CONFIG_FB_MSM_MIPI_DSI_TRUST_NT35516
static struct wled_config_data wled_cfg = {
	.dig_mod_gen_en = true,
	.cs_out_en = true,
	.ctrl_delay_us = 0,
	.op_fdbck = false,
	.ovp_val = WLED_OVP_35V,
	.boost_curr_lim = WLED_CURR_LIMIT_525mA,
	.num_strings = 2,
};
#else
static struct wled_config_data wled_cfg = {
	.dig_mod_gen_en = true,
	.cs_out_en = true,
	.ctrl_delay_us = 0,
	.op_fdbck = true,
	.ovp_val = WLED_OVP_32V,
	.boost_curr_lim = WLED_CURR_LIMIT_525mA,
	.num_strings = 1,
};
#endif
# if 0
static int pm8038_led0_pwm_duty_pcts[56] = {
		1, 4, 8, 12, 16, 20, 24, 28, 32, 36,
		40, 44, 46, 52, 56, 60, 64, 68, 72, 76,
		80, 84, 88, 92, 96, 100, 100, 100, 98, 95,
		92, 88, 84, 82, 78, 74, 70, 66, 62, 58,
		58, 54, 50, 48, 42, 38, 34, 30, 26, 22,
		14, 10, 6, 4, 1
};

/*
 * Note: There is a bug in LPG module that results in incorrect
 * behavior of pattern when LUT index 0 is used. So effectively
 * there are 63 usable LUT entries.
 */
static struct pm8xxx_pwm_duty_cycles pm8038_led0_pwm_duty_cycles = {
	.duty_pcts = (int *)&pm8038_led0_pwm_duty_pcts,
	.num_duty_pcts = ARRAY_SIZE(pm8038_led0_pwm_duty_pcts),
	.duty_ms = PM8XXX_LED_PWM_DUTY_MS,
	.start_idx = 1,
};
#endif //Carl Chang, 20121130
static struct pm8xxx_led_config pm8038_led_configs[] = {
	[0] = {
		.id = PM8XXX_ID_WLED,
		.mode = PM8XXX_LED_MODE_MANUAL,
		.max_current = PM8038_WLED_MAX_CURRENT,
		.default_state = 0,
		.wled_cfg = &wled_cfg,
	},
	[1] = {
		.id = PM8XXX_ID_RGB_LED_RED,
		.mode = PM8XXX_LED_MODE_PWM1,
		.max_current = PM8038_RGB_LED_MAX_CURRENT,
		.pwm_channel = 5,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
		//.pwm_duty_cycles = &pm8038_led0_pwm_duty_cycles,
	},
	[2] = {
		.id = PM8XXX_ID_RGB_LED_GREEN,
		.mode = PM8XXX_LED_MODE_PWM1,
		.max_current = PM8038_RGB_LED_MAX_CURRENT,
		.pwm_channel = 4,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
		//.pwm_duty_cycles = &pm8038_led0_pwm_duty_cycles,
	},
	[3] = {
		.id = PM8XXX_ID_RGB_LED_BLUE,
		.mode = PM8XXX_LED_MODE_PWM1,
		.max_current = PM8038_RGB_LED_MAX_CURRENT,
		.pwm_channel = 3,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
		//.pwm_duty_cycles = &pm8038_led0_pwm_duty_cycles,
	},
};

static struct pm8xxx_led_platform_data pm8xxx_leds_pdata = {
	.led_core = &pm8038_led_core_pdata,
	.configs = pm8038_led_configs,
	.num_configs = ARRAY_SIZE(pm8038_led_configs),
};

static struct pm8xxx_ccadc_platform_data pm8xxx_ccadc_pdata = {
	.r_sense_uohm		= 10000,
	.calib_delay_ms		= 600000,
};

static struct pm8xxx_misc_platform_data pm8xxx_misc_pdata = {
	.priority		= 0,
};

/*
 *	0x254=0xC8 (Threshold=110, preamp bias=01)
 *	0x255=0xC1 (Hold=110, max attn=0000, mute=1)
 *	0x256=0xB0 (decay=101, attack=10, delay=0)
 */

static struct pm8xxx_spk_platform_data pm8xxx_spk_pdata = {
	.spk_add_enable		= false,
	.cd_ng_threshold	= 0x0,
	.cd_nf_preamp_bias	= 0x1,
	.cd_ng_hold		= 0x6,
	.cd_ng_max_atten	= 0x0,
	.noise_mute		= 1,
	.cd_ng_decay_rate	= 0x5,
	.cd_ng_attack_rate	= 0x2,
	.cd_delay		= 0x0,
};

static struct pm8921_bms_platform_data pm8921_bms_pdata __devinitdata = {
	.battery_type			= BATT_SBJ,//Carl Chang, add SBJ battery data
	.r_sense_uohm			= 10000,
	.v_cutoff			= 3400,      //Eric Liu, SW treat v_cutoff as min_voltage
	.max_voltage_uv			= MAX_VOLTAGE_MV * 1000,
	.shutdown_soc_valid_limit	= 20,
	.adjust_soc_low_threshold	= 25,
	.chg_term_ua			= CHG_TERM_MA * 1000,
	.rconn_mohm			= 37,        //Carl Chang, Boston HW require 37 mOhm
	.normal_voltage_calc_ms		= 20000,
	.low_voltage_calc_ms		= 1000,
	.alarm_low_mv			= 3400,
	.alarm_high_mv			= 4000,
};
/* Add for vibrator */
struct pm8xxx_vibrator_platform_data pm8xxx_vib_pdata = {
 .max_timeout_ms = 15000,
 .level_mV = 3100, //Carl Chang, Boston HW require vibrator voltage 3.1V
};
//Carl Chang, 20121130
static struct pm8038_platform_data pm8038_platform_data __devinitdata = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
	.rtc_pdata              = &pm8xxx_rtc_pdata,
	.pwrkey_pdata		= &pm8xxx_pwrkey_pdata,
	.misc_pdata		= &pm8xxx_misc_pdata,
	.regulator_pdatas	= msm8930_pm8038_regulator_pdata,
	.charger_pdata		= &pm8921_chg_pdata,
	.bms_pdata		= &pm8921_bms_pdata,
	.adc_pdata		= &pm8038_adc_pdata,
	.leds_pdata		= &pm8xxx_leds_pdata,
	.ccadc_pdata		= &pm8xxx_ccadc_pdata,
	.spk_pdata		= &pm8xxx_spk_pdata,
	.vibrator_pdata    = &pm8xxx_vib_pdata,//Carl Chang, 20121130
};

static struct msm_ssbi_platform_data msm8930_ssbi_pm8038_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name			= "pm8038-core",
		.platform_data		= &pm8038_platform_data,
	},
};

/* PM8917 platform data */

static struct pm8xxx_adc_amux pm8917_adc_channels_data[] = {
	{"vcoin", CHANNEL_VCOIN, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vbat", CHANNEL_VBAT, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"dcin", CHANNEL_DCIN, CHAN_PATH_SCALING4, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ichg", CHANNEL_ICHG, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vph_pwr", CHANNEL_VPH_PWR, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ibat", CHANNEL_IBAT, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"batt_therm", CHANNEL_BATT_THERM, CHAN_PATH_SCALING1, AMUX_RSV2,
		ADC_DECIMATION_TYPE2, ADC_SCALE_BATT_THERM},
	{"batt_id", CHANNEL_BATT_ID, CHAN_PATH_SCALING1, AMUX_RSV2,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"usbin", CHANNEL_USBIN, CHAN_PATH_SCALING3, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"pmic_therm", CHANNEL_DIE_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PMIC_THERM},
	{"625mv", CHANNEL_625MV, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"125v", CHANNEL_125V, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"chg_temp", CHANNEL_CHG_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"xo_therm", CHANNEL_MUXOFF, CHAN_PATH_SCALING1, AMUX_RSV0,
		ADC_DECIMATION_TYPE2, ADC_SCALE_XOTHERM},
	{"pa_therm0", ADC_MPP_1_AMUX3, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PA_THERM},
};

static struct pm8xxx_adc_properties pm8917_adc_data = {
	.adc_vdd_reference	= 1800, /* milli-voltage for this adc */
	.bitresolution		= 15,
	.bipolar                = 0,
};

static struct pm8xxx_adc_platform_data pm8917_adc_pdata = {
	.adc_channel            = pm8917_adc_channels_data,
	.adc_num_board_channel  = ARRAY_SIZE(pm8917_adc_channels_data),
	.adc_prop               = &pm8917_adc_data,
	.adc_mpp_base		= PM8917_MPP_PM_TO_SYS(1),
};

static struct pm8921_platform_data pm8917_platform_data __devinitdata = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
	.rtc_pdata              = &pm8xxx_rtc_pdata,
	.pwrkey_pdata		= &pm8xxx_pwrkey_pdata,
	.misc_pdata		= &pm8xxx_misc_pdata,
	.regulator_pdatas	= msm8930_pm8917_regulator_pdata,
	.charger_pdata		= &pm8921_chg_pdata,
	.bms_pdata		= &pm8921_bms_pdata,
	.adc_pdata		= &pm8917_adc_pdata,
	.ccadc_pdata		= &pm8xxx_ccadc_pdata,
};

static struct msm_ssbi_platform_data msm8930_ssbi_pm8917_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name			= "pm8921-core",
		.platform_data		= &pm8917_platform_data,
	},
};

void __init msm8930_init_pmic(void)
{
	if (socinfo_get_pmic_model() != PMIC_MODEL_PM8917) {
		/* PM8038 configuration */
		pmic_reset_irq = PM8038_IRQ_BASE + PM8038_RESOUT_IRQ;
		msm8960_device_ssbi_pmic.dev.platform_data =
					&msm8930_ssbi_pm8038_pdata;
		pm8038_platform_data.num_regulators
			= msm8930_pm8038_regulator_pdata_len;
#if 0 //Carl Chang ,we don't use this define
		if (machine_is_msm8930_mtp())
			pm8921_bms_pdata.battery_type = BATT_PALLADIUM;
		else if (machine_is_msm8930_cdp())
			pm8921_chg_pdata.has_dc_supply = true;
#endif
	} else {
		/* PM8917 configuration */
		pmic_reset_irq = PM8917_IRQ_BASE + PM8921_RESOUT_IRQ;
		msm8960_device_ssbi_pmic.dev.platform_data =
					&msm8930_ssbi_pm8917_pdata;
		pm8917_platform_data.num_regulators
			= msm8930_pm8917_regulator_pdata_len;
		if (machine_is_msm8930_mtp())
			pm8921_bms_pdata.battery_type = BATT_PALLADIUM;
		else if (machine_is_msm8930_cdp())
			pm8921_chg_pdata.has_dc_supply = true;
	}

//Eric Liu+, Boston don't use this define
#if 0
	if (!machine_is_msm8930_mtp())
		pm8921_chg_pdata.battery_less_hardware = 1;
#endif
//Eric Liu-
}
