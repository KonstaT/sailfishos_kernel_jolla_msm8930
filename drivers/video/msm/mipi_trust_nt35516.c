/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
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

#include <linux/leds.h>
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_trust_nt35516.h"
#include "mdp4.h"
#include "../../leds/leds-pm8xxx.h"
#include <mach/hwid.h>

static unsigned int panel_exist = 0;
static int panel_initialled = 0;
static char DETECT_PANEL_PROC_ENTRY[] = "detect_panel";
static struct proc_dir_entry *proc_entry;

extern struct dentry *kernel_debuglevel_dir;

#define FB_MSM_MIPI_DSI_NT35516_LCMINFO 1 // 0: disable, 1: enable for get LCM information(ex: PWM).

// Jackie 20121203, add LCD backlight debug log
#define LCD_BL_LOG_AFTER_SCREENON 1

#if LCD_BL_LOG_AFTER_SCREENON
atomic_t LcdBlLogAfterResume = ATOMIC_INIT(0);
#endif

#if FB_MSM_MIPI_DSI_NT35516_LCMINFO
#define NT35516_DEBUG_BUF 64
static char debug_buf[NT35516_DEBUG_BUF];
static int system_bl_level_old;
struct msm_fb_data_type *mfd_bkl = NULL;
static int em_cabc_type = 0;
static int em_gamma_type = 1;
#endif

#ifdef CONFIG_PM_LOG
#include <linux/android_alarm.h>
struct pmlog_device *pmlog_device_lcd_bkl;
#define NUM_OF_BACKLIGHT_LOG_LEVEL	8	//Max 256
#define NUM_OF_BACKLIGHT_PER_LEVEL	32	//256 / NUM_OF_BACKLIGHT_LOG_LEVEL
static struct timespec backlight_runtime[NUM_OF_BACKLIGHT_LOG_LEVEL];	
static struct timespec start_time;
static struct timespec latest_update_time;	//For last update run time 
static  int current_bl_log_level = -1;
static spinlock_t pm_log_lock;
#endif

static struct mipi_dsi_panel_platform_data *mipi_trust_nt35516_pdata;

static struct dsi_buf trust_nt35516_tx_buf;
static struct dsi_buf trust_nt35516_rx_buf;
static int mipi_trust_nt35516_lcd_init(void);
void mipi_dsi_cdp_panel_reset(void);

static int wled_trigger_initialized;

//static char sw_reset[2] = {0x01, 0x00}; /* DTYPE_DCS_WRITE */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */
//static char rgb_888[2] = {0x3A, 0x77}; /* DTYPE_DCS_WRITE1 */
//static char tear_off[2] = {0x34, 0x00};		// DTYPE_DCS_WRITE1

#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
static char wr_disbv[2] = {0x51, 0x40};	// [Write Display Brightness] 0~0xff, set 0x40(~25%) as default.
static char wr_ctrld[2] = {0x53, 0x24}; // [Write CTRL Display] Reg0x53[2]: CABC_PEM_OUT on/off ()
static char wr_ctrld_off[2] = {0x53, 0x00}; // [Write CTRL Display]
static char wr_cabc[2] = {0x55, 0x01}; // [Write CABC CTRL] 0:CABC is off, 1:UI Image, 2:Still Picture, 3:Moving Image
static char wr_cabc_off[2] = {0x55, 0x00}; // [Write CABC CTRL] 0:CABC is off.
static char wr_cabc_mb[2] = {0x5E, 0x1E}; // [Write CABC minimun brightness] 0x1E(30): ICS default is 20.
//static char led_orise_cmd_1[4] = {0xFF, 0x96, 0x01, 0x01}; // Enable Orise command mode
//static char led_orise_cmd_2[5] = {0x00, 0x80, 0xFF, 0x96, 0x01}; // Enable Orise command mode
//static char led_pwm_freq_1[4] =  {0x00, 0xB4, 0xC6, 0x0}; // select PWM frequency 1, PWM table select
//static char led_pwm_freq_2[4] =  {0x00, 0xB1, 0xC6, 0x6}; // select PWM frequency 2, 0x6: 9.486Khz

static struct dsi_cmd_desc nt35516_cmd_backlight_cmd = {
	DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_disbv), wr_disbv};

#if FB_MSM_MIPI_DSI_NT35516_LCMINFO
static char wr_cabc_01[2] = {0x55, 0x01}; // [Write CABC CTRL] 1:UI Picture
static char wr_cabc_02[2] = {0x55, 0x02}; // [Write CABC CTRL] 2:Still Picture
static char wr_cabc_03[2] = {0x55, 0x03}; // [Write CABC CTRL] 3:Moving Image
static char wr_cabc_mb_clear[2] = {0x5E, 0x00}; // [Write CABC minimun brightness] clear to 0.

static struct dsi_cmd_desc nt35516_cmd_em_cabc_off_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(wr_cabc_mb_clear), wr_cabc_mb_clear},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_cabc_off), wr_cabc_off},
};

static struct dsi_cmd_desc nt35516_cmd_em_cabc_01_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_cabc_01), wr_cabc_01},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(wr_cabc_mb_clear), wr_cabc_mb_clear},
};
static struct dsi_cmd_desc nt35516_cmd_em_cabc_02_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_cabc_02), wr_cabc_02},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(wr_cabc_mb_clear), wr_cabc_mb_clear},
};
static struct dsi_cmd_desc nt35516_cmd_em_cabc_03_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_cabc_03), wr_cabc_03},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(wr_cabc_mb_clear), wr_cabc_mb_clear},
};
#endif // end of FB_MSM_MIPI_DSI_NT35516_LCMINFO
#endif // end of CONFIG_FB_MSM_BACKLIGHT_LCMPWM

//----------------------------------------------------
/*manufactrue page 0*/
//static char trust_cmd_enable[5] = {0xFF, 0xAA, 0x55, 0x25, 0x01};
static char trust_select_page0[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
static char trust_display_option[2] = {0xB1, 0xEC};//{0xB1, 0xEC},cmd
static char trust_display_option_video[2] = {0xB1, 0xFC};//{0xB1, 0xFC},video mode?
static char trust_set_data_hold_time[2] = {0xB6, 0x05};
static char trust_set_gate_signal[3] = {0xB7, 0x72, 0x72};
static char trust_set_source_driver[5] = {0xB8, 0x01, 0x04, 0x04, 0x04};
//static char trust_set_bias_current[2] = {0xBB, 0x33};
static char trust_set_inversion_mode[4] = {0xBC,0x00,0x00,0x00};
static char trust_set_display_time[6] = {0xBD, 0x01, 0x4E, 0x10, 0x20, 0x01};
static char trust_set_c9h[7] = {0xC9, 0x61, 0x06, 0x0d, 0x17, 0x17,0x00};
/*disable CABC*/
static char trust_display_ctrl_off[2] = {0x53, 0x00};
static char trust_cabc_mode_select_off[2] = {0x55, 0x00};
static char trust_led_on_pwm_ctrl_off[2] = {0xD0, 0x00};
/*manufactrue page 1*/
static char trust_select_page1[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01};
/*avdd 5.5v*/
static char trust_set_avdd_voltage[4] = {0xB0, 0x0B,0x0B,0x0B};
/*boosting time and frequence 3.0x of circuit 1*/
static char trust_set_boost_time_circuit1[4] = {0xB6, 0x34,0x34,0x34};
/*avee voltage -5.5v*/
static char trust_set_set_avee_voltage[4] = {0xB1, 0x0B,0x0B,0x0B};
/*boosting time and frequence -2.0x of circuit 2*/
static char trust_set_boost_time_circuit2[4] = {0xB7, 0x34,0x34,0x34};
/*set vcl voltage -4.0v*/
static char trust_set_set_vcl_voltage[4] = {0xB2, 0x02, 0x02, 0x02};
/*boosting time and frequence -2.0x of circuit 3*/
static char trust_set_boost_time_circuit3[4] = {0xB8, 0x20, 0x20, 0x20};
/*set vgh voltage 14v*/
static char trust_set_set_vgh_voltage[4] = {0xB3, 0x10, 0x10, 0x10};
/*VGH: AVDD - AVEE + VDDB*/
static char trust_set_boost_time_circuit4[4] = {0xB9, 0x34, 0x34, 0x34};
/*VGLX:-12.0v*/
static char trust_set_set_vglx_voltage[4] = {0xB4, 0x06, 0x06, 0x06};
/*VGLX: AVEE + VCL - AVDD0*/
static char trust_set_boost_time_circuit5[4] = {0xBA, 0x14, 0x14, 0x14};
/*VGMP: 5.0V, VGSP: 0.3V*/
static char trust_set_vgmp_vgsp_voltage[4] = {0xBC, 0x00, 0x98, 0x00};
/*VGMN: -5.0V, VGSN: -0.3V*/
static char trust_set_vgmn_vgsn_voltage[4] = {0xBD, 0x00, 0x98, 0x00};
/*adjust vcom voltage offset*/
static char trust_set_vcom_offset_voltage[2] = {0xBE, 0X73};
/*DC/DC enable control*/
static char trust_enable_dc_convert[2] = {0xC2, 0x00};
/*led pwm,brightness*/
static char trust_led_pwm_dimming[5] = {0xD0, 0x0f, 0x0f, 0x10,0X10};
/*set positive red gamma*/
static char trust_positive_red_gamma1[17] = {
	0xD1, 
	0x00,0x70,0x00,0x95,
	0x00,0xBD,0x00,0xD2,
	0x00,0xE0,0x01,0x06,
	0x01,0x24,0x01,0x53
};
static char trust_positive_red_gamma2[17] = {
	0xD2, 
	0x01,0x7A,0x01,0xB4,
	0x01,0xE1,0x02,0x28,
	0x02,0x61,0x02,0x63,
	0x02,0x98,0x02,0xD1
};
static char trust_positive_red_gamma3[17] = {
	0xD3,
	0x02,0xF5,0x03,0x26,
	0x03,0x47,0x03,0x73,
	0x03,0x8F,0x03,0xB9,
	0x03,0xD8,0x03,0xFD
};
static char trust_positive_red_gamma4[5] = {
	0xD4, 
	0x03,0xFF,0x03,0xFF
};
/*set positive green gamma*/
static char trust_positive_green_gamma1[17] = {
	0xD5,
	0x00,0x70,0x00,0x95,
	0x00,0xBD,0x00,0xD2,
	0x00,0xE0,0x01,0x06,
	0x01,0x24,0x01,0x53
};
static char trust_positive_green_gamma2[17] = {
	0xD6,
	0x01,0x7A,0x01,0xB4,
	0x01,0xE1,0x02,0x28,
	0x02,0x61,0x02,0x63,
	0x02,0x98,0x02,0xD1
};
static char trust_positive_green_gamma3[17] = {
	0xD7,
	0x02,0xF5,0x03,0x26,
	0x03,0x47,0x03,0x73,
	0x03,0x8F,0x03,0xB9,
	0x03,0xD8,0x03,0xFD
};
static char trust_positive_green_gamma4[5] = {
	0xD8, 
	0x03,0xFF,0x03,0xFF 
};
/*set positive blue gamma*/
static char trust_positive_blue_gamma1[17] = {
	0xD9,
	0x00,0x70,0x00,0x95,
	0x00,0xBD,0x00,0xD2,
	0x00,0xE0,0x01,0x06,
	0x01,0x24,0x01,0x53
};
static char trust_positive_blue_gamma2[17] = {
	0xDD,
	0x01,0x7A,0x01,0xB4,
	0x01,0xE1,0x02,0x28,
	0x02,0x61,0x02,0x63,
	0x02,0x98,0x02,0xD1
};
static char trust_positive_blue_gamma3[17] = {
	0xDE,
	0x02,0xF5,0x03,0x26,
	0x03,0x47,0x03,0x73,
	0x03,0x8F,0x03,0xB9,
	0x03,0xD8,0x03,0xFD
};
static char trust_positive_blue_gamma4[5] = {
	0xDF,
	0x03,0xFF,0x03,0xFF
};
/*set negative red gamma*/
static char trust_negative_red_gamma1[17] = {
	0xE0, 
	0x00,0x70,0x00,0x95,
	0x00,0xBD,0x00,0xD2,
	0x00,0xE0,0x01,0x06,
	0x01,0x24,0x01,0x53
};
static char trust_negative_red_gamma2[17] = {
	0xE1,
	0x01,0x7A,0x01,0xB4,
	0x01,0xE1,0x02,0x28,
	0x02,0x61,0x02,0x63,
	0x02,0x98,0x02,0xD1
};
static char trust_negative_red_gamma3[17] = {
	0xE2, 
	0x02,0xF5,0x03,0x26,
	0x03,0x47,0x03,0x73,
	0x03,0x8F,0x03,0xB9,
	0x03,0xD8,0x03,0xFD
};
static char trust_negative_red_gamma4[5] = {
	0xE3, 
	0x03,0xFF,0x03,0xFF
};
/*set negative green gamma*/
static char trust_negative_green_gamma1[17] = {
	0xE4,
	0x00,0x70,0x00,0x95,
	0x00,0xBD,0x00,0xD2,
	0x00,0xE0,0x01,0x06,
	0x01,0x24,0x01,0x53
};
static char trust_negative_green_gamma2[17] = {
	0xE5,
	0x01,0x7A,0x01,0xB4,
	0x01,0xE1,0x02,0x28,
	0x02,0x61,0x02,0x63,
	0x02,0x98,0x02,0xD1
};
static char trust_negative_green_gamma3[17] = {
	0xE6,
	0x02,0xF5,0x03,0x26,
	0x03,0x47,0x03,0x73,
	0x03,0x8F,0x03,0xB9,
	0x03,0xD8,0x03,0xFD
};
static char trust_negative_green_gamma4[5] = {
	0xE7,
	0x03,0xFF,0x03,0xFF 
};
/*set negative blue gamma*/
static char trust_negative_blue_gamma1[17] = {
	0xE8,
	0x00,0x70,0x00,0x95,
	0x00,0xBD,0x00,0xD2,
	0x00,0xE0,0x01,0x06,
	0x01,0x24,0x01,0x53
};
static char trust_negative_blue_gamma2[17] = {
	0xE9,
	0x01,0x7A,0x01,0xB4,
	0x01,0xE1,0x02,0x28,
	0x02,0x61,0x02,0x63,
	0x02,0x98,0x02,0xD1
};
static char trust_negative_blue_gamma3[17] = {
	0xEA,
	0x02,0xF5,0x03,0x26,
	0x03,0x47,0x03,0x73,
	0x03,0x8F,0x03,0xB9,
	0x03,0xD8,0x03,0xFD
};
static char trust_negative_blue_gamma4[5] = {
	0xEB,
	0x03,0xFF,0x03,0xFF 
};
static char trust_roate[2] = {0x36,0x00};//no rotate by default.
static char trust_pixel_format[2] ={0x3A, 0x77};
static char trust_tearing_effect_on[2] = {0x35, 0x00};
static char trust_page_off[6] = {0xF0, 0x55, 0xAA, 0x52, 0x00, 0x00};//lock pag0 and page1,disable write.

#define TRUST_CMD_DELAY		0
#define TRUST_SLEEP_OFF_DELAY	120

static struct dsi_cmd_desc trust_nt35516_cmd_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,  // Jackie 20121203, 150=>120, need to check this with vendor.
		sizeof(exit_sleep), exit_sleep},
	//{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_cmd_enable), trust_cmd_enable},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_select_page0), trust_select_page0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_display_option), trust_display_option},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_data_hold_time), trust_set_data_hold_time},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_gate_signal), trust_set_gate_signal},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_source_driver), trust_set_source_driver},
	//{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_bias_current), trust_set_bias_current},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_inversion_mode), trust_set_inversion_mode},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_display_time), trust_set_display_time},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_c9h), trust_set_c9h},
	// disable CABC{
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_display_ctrl_off), trust_display_ctrl_off},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_cabc_mode_select_off), trust_cabc_mode_select_off},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_led_on_pwm_ctrl_off), trust_led_on_pwm_ctrl_off},
	// disable CABC}
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_select_page1), trust_select_page1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_avdd_voltage), trust_set_avdd_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit1), trust_set_boost_time_circuit1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_set_avee_voltage), trust_set_set_avee_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit2), trust_set_boost_time_circuit2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_set_vcl_voltage), trust_set_set_vcl_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit3), trust_set_boost_time_circuit3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_set_vgh_voltage), trust_set_set_vgh_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit4), trust_set_boost_time_circuit4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_set_vglx_voltage), trust_set_set_vglx_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit5), trust_set_boost_time_circuit5},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_vgmp_vgsp_voltage), trust_set_vgmp_vgsp_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_vgmn_vgsn_voltage), trust_set_vgmn_vgsn_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_vcom_offset_voltage), trust_set_vcom_offset_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_enable_dc_convert), trust_enable_dc_convert},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_led_pwm_dimming), trust_led_pwm_dimming},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_red_gamma1), trust_positive_red_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_red_gamma2), trust_positive_red_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_red_gamma3), trust_positive_red_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_red_gamma4), trust_positive_red_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_green_gamma1), trust_positive_green_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_green_gamma2), trust_positive_green_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_green_gamma3), trust_positive_green_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_green_gamma4), trust_positive_green_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_blue_gamma1), trust_positive_blue_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_blue_gamma2), trust_positive_blue_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_blue_gamma3), trust_positive_blue_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_blue_gamma4), trust_positive_blue_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_red_gamma1), trust_negative_red_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_red_gamma2), trust_negative_red_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_red_gamma3), trust_negative_red_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_red_gamma4), trust_negative_red_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_green_gamma1), trust_negative_green_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_green_gamma2), trust_negative_green_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_green_gamma3), trust_negative_green_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_green_gamma4), trust_negative_green_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_blue_gamma1), trust_negative_blue_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_blue_gamma2), trust_negative_blue_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_blue_gamma3), trust_negative_blue_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_blue_gamma4), trust_negative_blue_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_roate), trust_roate},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_pixel_format), trust_pixel_format},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_tearing_effect_on), trust_tearing_effect_on},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_page_off), trust_page_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, // Jackie 20121019, 50 => 10
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc trust_nt35516_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,  // Jackie 20121203, 150=>120, need to check this with vendor.
		sizeof(exit_sleep), exit_sleep},
	//{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_cmd_enable), trust_cmd_enable},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_select_page0), trust_select_page0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_display_option_video), trust_display_option_video},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_data_hold_time), trust_set_data_hold_time},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_gate_signal), trust_set_gate_signal},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_source_driver), trust_set_source_driver},
	//{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_bias_current), trust_set_bias_current},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_inversion_mode), trust_set_inversion_mode},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_display_time), trust_set_display_time},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_c9h), trust_set_c9h},
	// disable CABC{
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_display_ctrl_off), trust_display_ctrl_off},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_cabc_mode_select_off), trust_cabc_mode_select_off},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_led_on_pwm_ctrl_off), trust_led_on_pwm_ctrl_off},
	// disable CABC}
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_select_page1), trust_select_page1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_avdd_voltage), trust_set_avdd_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit1), trust_set_boost_time_circuit1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_set_avee_voltage), trust_set_set_avee_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit2), trust_set_boost_time_circuit2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_set_vcl_voltage), trust_set_set_vcl_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit3), trust_set_boost_time_circuit3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_set_vgh_voltage), trust_set_set_vgh_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit4), trust_set_boost_time_circuit4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_set_vglx_voltage), trust_set_set_vglx_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_boost_time_circuit5), trust_set_boost_time_circuit5},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_vgmp_vgsp_voltage), trust_set_vgmp_vgsp_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_vgmn_vgsn_voltage), trust_set_vgmn_vgsn_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_set_vcom_offset_voltage), trust_set_vcom_offset_voltage},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_enable_dc_convert), trust_enable_dc_convert},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_led_pwm_dimming), trust_led_pwm_dimming},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_red_gamma1), trust_positive_red_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_red_gamma2), trust_positive_red_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_red_gamma3), trust_positive_red_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_red_gamma4), trust_positive_red_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_green_gamma1), trust_positive_green_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_green_gamma2), trust_positive_green_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_green_gamma3), trust_positive_green_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_green_gamma4), trust_positive_green_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_blue_gamma1), trust_positive_blue_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_blue_gamma2), trust_positive_blue_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_blue_gamma3), trust_positive_blue_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_positive_blue_gamma4), trust_positive_blue_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_red_gamma1), trust_negative_red_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_red_gamma2), trust_negative_red_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_red_gamma3), trust_negative_red_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_red_gamma4), trust_negative_red_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_green_gamma1), trust_negative_green_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_green_gamma2), trust_negative_green_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_green_gamma3), trust_negative_green_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_green_gamma4), trust_negative_green_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_blue_gamma1), trust_negative_blue_gamma1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_blue_gamma2), trust_negative_blue_gamma2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_blue_gamma3), trust_negative_blue_gamma3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_negative_blue_gamma4), trust_negative_blue_gamma4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_roate), trust_roate},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_pixel_format), trust_pixel_format},
	{DTYPE_DCS_LWRITE, 1, 0, 0, TRUST_CMD_DELAY,sizeof(trust_page_off), trust_page_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, // Jackie 20121019, 50 => 10
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc trust_nt35516_540960_display_off_cmds[] = {
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(tear_off), tear_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, // Jackie 20121019, need 10 for hw spec
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};
//----------------------------------------------------

#ifdef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
// 20120616 Jackie, for turn on backlight after draw FB0.
static struct dsi_cmd_desc nt35516_bkl_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(wr_ctrld), wr_ctrld},
//	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
//		sizeof(wr_disbv_on), wr_disbv_on},
};

static struct dsi_cmd_desc nt35516_bkl_off_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(wr_ctrld_off), wr_ctrld_off},
};

void mipi_nt35516_lcd_bkl_on(void)
{
	struct dcs_cmd_req cmdreq;

#if LCD_BL_LOG_AFTER_SCREENON
	LCD_PRINTK(0, "%s()\n", __func__);
#endif
	cmdreq.cmds = nt35516_bkl_on_cmds;
	cmdreq.cmds_cnt = ARRAY_SIZE(nt35516_bkl_on_cmds);
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
}

void mipi_nt35516_lcd_bkl_off(void)
{
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = nt35516_bkl_off_cmds;
	cmdreq.cmds_cnt = ARRAY_SIZE(nt35516_bkl_off_cmds);
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
}
#else
void mipi_nt35516_lcd_bkl_on(void)
{
#if LCD_BL_LOG_AFTER_SCREENON
	LCD_PRINTK(0, "%s(), mfd_bkl->bl_leve%d, system_bl_level_old=%d\n", __func__, mfd_bkl->bl_level, system_bl_level_old);
	//LCD_PRINTK(0, "%s()\n", __func__);
#endif
	mutex_lock(&mfd_bkl->dma->ov_mutex);
#if defined(PM8038_WLED_OUTPUT_WA)
	led_wled_set_backlight(PM8038_WLED_PWM_MIN_LEVEL);// min. backlight value
#else
	led_wled_set_backlight(1);// min. backlight value
#endif
	mutex_unlock(&mfd_bkl->dma->ov_mutex);
}
void mipi_nt35516_lcd_bkl_off(void)
{
	mutex_lock(&mfd_bkl->dma->ov_mutex);
	led_wled_set_backlight(0);
	mutex_unlock(&mfd_bkl->dma->ov_mutex);
}
#endif

// Jackie 20121211, detect panel exist or not by reading manufacture id.
static char manufacture_id[2] = {0x04, 0x00}; // DTYPE_DCS_READ

static struct dsi_cmd_desc trust_nt35516_manufacture_id_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

static u32 manu_id;

static void mipi_trust_nt35516_manufacture_cb(u32 data)
{
	manu_id = data & 0xFF; // manufacture id is only one byte.
	pr_info("%s: manufacture_id=0x%x\n", __func__, manu_id);
}

static uint32 mipi_trust_nt35516_manufacture_id(struct msm_fb_data_type *mfd)
{
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = &trust_nt35516_manufacture_id_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 3;
	cmdreq.cb = mipi_trust_nt35516_manufacture_cb; /* call back */
	mipi_dsi_cmdlist_put(&cmdreq);
	/*
	 * blocked here, untill call back called
	 */

	return manu_id;
}

static int mipi_trust_nt35516_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct msm_panel_info *pinfo;
	struct dcs_cmd_req cmdreq;
	uint32 lcm_manu_id = 0;
#ifdef CONFIG_PM_LOG
	int ret = 0;
#endif

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

#if FB_MSM_MIPI_DSI_NT35516_LCMINFO
	if(!mfd_bkl)
	{
		mfd_bkl = mfd;
		printk(KERN_ERR "%s: mfd_bkl=%08X\n", __func__, (unsigned)mfd_bkl);
	}
#endif

	pinfo = &mfd->panel_info;

	mipi  = &mfd->panel_info.mipi;

	LCD_PRINTK(0, "%s(), mipi_mode=%d\n", __func__, mipi->mode);

	// Jackie 20121203
	mipi_dsi_cdp_panel_reset();

	if (mipi->mode == DSI_VIDEO_MODE) {
		cmdreq.cmds = trust_nt35516_video_on_cmds;
		cmdreq.cmds_cnt = ARRAY_SIZE(trust_nt35516_video_on_cmds);
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		mipi_dsi_cmdlist_put(&cmdreq);
	} else {
		cmdreq.cmds = trust_nt35516_cmd_on_cmds;
		cmdreq.cmds_cnt = ARRAY_SIZE(trust_nt35516_cmd_on_cmds);
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		mipi_dsi_cmdlist_put(&cmdreq);

		/* clean up ack_err_status */
		mipi_dsi_cmd_bta_sw_trigger();
		// Jackie 20121211, detect panel exist or not by reading manufacture id.
		if(panel_initialled == 0)
		{
			// 20130110 Jackie, it may be removed later. just keep it for double check here.
			lcm_manu_id = mipi_trust_nt35516_manufacture_id(mfd);
			pr_err("%s: panel_exist=%d, manufacture_id=0x%x, system_rev=0x%x\n", __func__, panel_exist, lcm_manu_id, system_rev);

			panel_initialled = 1;
		}
	}

#if FB_MSM_MIPI_DSI_NT35516_LCMINFO
#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
	em_cabc_type = 3; // LCDM's CABC: default is [Moving Image] mode
#else
	em_cabc_type = 4; // QC's CABL: default is on. [4:on, 0:off]
#endif
#endif

#ifdef CONFIG_PM_LOG
		ret = pmlog_device_on(pmlog_device_lcd_bkl);
		if (ret)
			pr_err("%s: pmlog_device_on fail rc = %d\n", __func__, ret);
#endif

#if LCD_BL_LOG_AFTER_SCREENON
	// set counter. print 3 times log.
	atomic_set(&LcdBlLogAfterResume,3);
#endif

	LCD_PRINTK(0, "%s()--\n", __func__);
	return 0;
}

static int mipi_trust_nt35516_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct dcs_cmd_req cmdreq;
#ifdef CONFIG_PM_LOG
	int ret = 0;
#endif

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	LCD_PRINTK(0, "%s()\n", __func__);

	// 20130325 Jackie, both PMIC_PWM and LCM_PWM case, turn off PMIC_WLED backlight
	// Even LCM_PWM mode, PMIC_WLED still need to be off.
	// WLED output current = CABC(LCM duty cycle)*PWM(PM8038 setting)*ILED(LED current)
	led_wled_set_backlight(0);

	cmdreq.cmds = trust_nt35516_540960_display_off_cmds;
	cmdreq.cmds_cnt = ARRAY_SIZE(trust_nt35516_540960_display_off_cmds);
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

#ifdef CONFIG_PM_LOG
		ret = pmlog_device_off(pmlog_device_lcd_bkl);
		if (ret)
			pr_err("%s: pmlog_device_off fail rc = %d\n", __func__, ret);
#endif

#if LCD_BL_LOG_AFTER_SCREENON
	// clear counter.
	atomic_set(&LcdBlLogAfterResume,0);
#endif

	LCD_PRINTK(0, "%s()--\n", __func__);
	return 0;
}


#if FB_MSM_MIPI_DSI_NT35516_LCMINFO
// #################################################
// EMList - Debugfs: PWM, BL_mode, CABC
// #################################################

//--------------
// EMList - PWM
//--------------
static int mipi_nt35516_pwm_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_nt35516_pwm_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_nt35516_pwm_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
#ifdef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
	struct dcs_cmd_req cmdreq;
#endif
	uint32 val, cnt;

	if(panel_exist == 1)
	{
		if((!mfd_bkl) || (!mfd_bkl->panel_power_on)) {
			LCD_PRINTK(1, "%s: ## panel_power is off\n", __func__);
			return count;
		}
	}

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	sscanf(debug_buf, "%d %d", &val, &cnt);

	if (cnt <= 0)
		cnt = 1;

	// Todo here
	if (val > 0xff || val<0)
	{
		printk(KERN_ERR "%s: Backlight level should be 0~255\n", __func__);
		return -EINVAL;
	}

	LCD_PRINTK(1, "%s(), PWM value is %d\n", __func__, val);
	system_bl_level_old=(int)val;

// if define "CONFIG_FB_MSM_BACKLIGHT_LCMPWM",
// use Backlight IC (LCM to generate PWM signal)
#ifdef CONFIG_FB_MSM_BACKLIGHT_LCMPWM

	if(em_cabc_type != 0)
	{
		cmdreq.cmds = nt35516_cmd_em_cabc_off_cmds;
		cmdreq.cmds_cnt = 2;
		cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;
		mipi_dsi_cmdlist_put(&cmdreq);

		em_cabc_type = 0; // CABC is off in EM.
		LCD_PRINTK(1, "Turn off CABC first in EM mode\n");
	}

	wr_disbv[1] = (unsigned char) system_bl_level_old;

	cmdreq.cmds = &nt35516_cmd_backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
#else

	if(em_cabc_type != 0)
	{
		// FIXME
		// Jackie: it shoud turn off QC's CABL here first

		em_cabc_type = 0; // CABC is off in EM.
		LCD_PRINTK(1, "Turn off CABC first in EM mode\n");
	}

	if(panel_exist == 1)
	{
		mutex_lock(&mfd_bkl->dma->ov_mutex);
		led_wled_set_backlight(system_bl_level_old);
		mutex_unlock(&mfd_bkl->dma->ov_mutex);
	}
	else
	{
		led_wled_set_backlight(system_bl_level_old);
	}
	#endif
	// Done

	return count;
}

static ssize_t mipi_nt35516_pwm_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;


	if (*ppos)
		return 0;	/* the end */

	len = snprintf(debug_buf, sizeof(debug_buf), "pwm is %d\n", system_bl_level_old);

	if (len < 0)
		return 0;

	if (copy_to_user(buff, debug_buf, len))
		return -EFAULT;

	*ppos += len;	/* increase offset */

	return len;
}
static const struct file_operations mipi_nt35516_pwm_fops = {
	.open = mipi_nt35516_pwm_open,
	.release = mipi_nt35516_pwm_release,
	.read = mipi_nt35516_pwm_read,
	.write = mipi_nt35516_pwm_write,
};

//------------------
// EMList - BL_mode
//------------------
static void nt35516_lcd_bkl_onOff(unsigned int onOff)
{
#ifdef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
	struct dcs_cmd_req cmdreq;

	static struct dsi_cmd_desc nt35516_cmd_backlight_on_cmd = {
		DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_ctrld), wr_ctrld};
	static struct dsi_cmd_desc nt35516_cmd_backlight_off_cmd = {
		DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_ctrld_off), wr_ctrld_off};

	LCD_PRINTK(1, "%s = %d\n", __func__, onOff);

	if (onOff)
	{
		cmdreq.cmds = &nt35516_cmd_backlight_on_cmd;
	}
	else
	{
		cmdreq.cmds = &nt35516_cmd_backlight_off_cmd;
	}
	
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
#else
	int bkl_onOff_bl_level;

	if (onOff)
	{
		// avoid No-Panel case, if system_bl_level_old=0, assign it as 102(40%).
		if(system_bl_level_old == 0)
			bkl_onOff_bl_level = 102; // 40%
		else
			bkl_onOff_bl_level = system_bl_level_old;
	}
	else
	{
		bkl_onOff_bl_level = 0;
	}

	if(panel_exist == 1)
	{
		mutex_lock(&mfd_bkl->dma->ov_mutex);
		led_wled_set_backlight(bkl_onOff_bl_level);
		mutex_unlock(&mfd_bkl->dma->ov_mutex);
	}
	else
	{
		led_wled_set_backlight(bkl_onOff_bl_level);
	}

	LCD_PRINTK(1, "%s, onOff=%d, bl_level=%d\n", __func__, onOff, bkl_onOff_bl_level);
#endif
}

static void nt35516_lcd_gamma(unsigned int gamma_sel)
{
	em_gamma_type = gamma_sel;
	LCD_PRINTK(0, "%s, gamma_sel=%d\n", __func__, gamma_sel);
}

// Jackie 20130116
#ifndef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
static void nt35516_lcd_bkl_string_onOff(unsigned int string, unsigned int onOff)
{
	LCD_PRINTK(1, "%s string=%d, onOff=%d\n", __func__, string, onOff);

	if(panel_exist == 1)
		mutex_lock(&mfd_bkl->dma->ov_mutex);

	if( ((string==1)||(string==2)) &&  ((onOff==0)||(onOff==1)) )
	{
	 	switch_wled(string, onOff); // WLED_DRV1/WLED_DRV2, enable/disable
	}
	else
	 	LCD_PRINTK(0, "%s, !!! Wrong onOff=%d, should be 0~3\n", __func__, onOff);

	if(panel_exist == 1)
		mutex_unlock(&mfd_bkl->dma->ov_mutex);
}
#endif

static void nt35516_lcd_onOff(unsigned int onOff)
{
// Todo here
}

static int mipi_nt35516_mode_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_nt35516_mode_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_nt35516_mode_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
	uint32 val, cnt;

	if(panel_exist == 1)
	{
		if((!mfd_bkl) || (!mfd_bkl->panel_power_on)) {
			LCD_PRINTK(1, "%s: ## panel_power is off\n", __func__);
			return count;
		}
	}
	if(count < 1)
	{
		printk(KERN_ERR "%s: input invalid, count = %d\n", __func__, count);
		return -EFAULT;
	}

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	sscanf(debug_buf, "%d %d", &val, &cnt);

	// Todo here
	LCD_PRINTK(1, "%s: mode = %c, count = %d\n", __func__, debug_buf[0], count);

	switch(debug_buf[0])
	{
		case 'b': //turn on off backlight
			if(debug_buf[1] == '0')
			nt35516_lcd_bkl_onOff(0);
			else if(debug_buf[1] == '1')
			nt35516_lcd_bkl_onOff(1);
			break;

		case 'g': // test for gamma
			if(debug_buf[1] == '0')
				// orignal gamma setting
				nt35516_lcd_gamma(0);
			else if(debug_buf[1] == '1')
				// new gamma setting, Gamma 2.2
				nt35516_lcd_gamma(1);
			break;

#ifndef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
		case 's': // 20130116 Jackie, turn on/off WLED_DRV1/WLED_DRV2
			if(debug_buf[1] == '0')
			nt35516_lcd_bkl_string_onOff(1, 0); // WLED_DRV1, off
			else if(debug_buf[1] == '1')
			nt35516_lcd_bkl_string_onOff(1, 1); // WLED_DRV1, on
			else if(debug_buf[1] == '2')
			nt35516_lcd_bkl_string_onOff(2, 0); // WLED_DRV2, off
			else if(debug_buf[1] == '3')
			nt35516_lcd_bkl_string_onOff(2, 1); // WLED_DRV2, on
			break;
#endif

		case 'l': // turn on off lcd
			if (debug_buf[1] == '0')
			nt35516_lcd_onOff(0);
			else if (debug_buf[1] == '1')
			nt35516_lcd_onOff(1);
			break;
	}
	// Done

	return count;
}

static const struct file_operations mipi_nt35516_mode_fops = {
	.open = mipi_nt35516_mode_open,
	.release = mipi_nt35516_mode_release,
	.write = mipi_nt35516_mode_write,
};

//--------------
// EMList - CABC
//--------------
static int mipi_nt35516_cabc_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_nt35516_cabc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_nt35516_cabc_write(
	struct file *file,
	const char __user *buff,
	size_t count,
	loff_t *ppos)
{
#ifdef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
	struct dcs_cmd_req cmdreq;
#endif
	uint32 val, cnt;

	if((!mfd_bkl) || (!mfd_bkl->panel_power_on)) {
		LCD_PRINTK(1, "%s: ## panel_power is off\n", __func__);
		return count;
	}

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;	/* end of string */

	sscanf(debug_buf, "%d %d", &val, &cnt);

	if (cnt <= 0)
		cnt = 1;

	// Todo here
	if (val > 0x4 || val<0)
	{
		printk(KERN_ERR "CABC value should be 0~4, yours is %d\n",val);
		return -EINVAL;
	}

	LCD_PRINTK(1, "CABC mode=%d\n", val);

#ifdef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
//use LCDM's CABC
	if(val == 0)
	{
		cmdreq.cmds = nt35516_cmd_em_cabc_off_cmds;
		LCD_PRINTK(1, "CABC is off in EM mode\n");
	}
	else if(val == 1)
	{
		cmdreq.cmds = nt35516_cmd_em_cabc_01_cmds;
		LCD_PRINTK(1, "set CABC as UI Image in EM mode\n");
	}
	else if(val == 2)
	{
		cmdreq.cmds = nt35516_cmd_em_cabc_02_cmds;
		LCD_PRINTK(1, "set CABC as Still picture in EM mode\n");
	}
	else if(val == 3)
	{
		cmdreq.cmds = nt35516_cmd_em_cabc_03_cmds;
		LCD_PRINTK(1, "set CABC as Moving Image in EM mode\n");
	}
	else
	{
		LCD_PRINTK(0, "invalid value for LCM's CABC mode in EM mode, nothing applied\n");
		return count;
	}
	em_cabc_type = (int)val;

	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
#else //NOT CONFIG_FB_MSM_BACKLIGHT_LCMPWM
//use PMIC QC's CABL
	
	// FIXME
	if(val == 0)
	{
		LCD_PRINTK(1, "QC's CABL is off in EM mode\n");
	}
	else if(val == 4)
	{
		LCD_PRINTK(1, "QC's CABL is on in EM mode\n");
	}
	else
	{
		LCD_PRINTK(0, "invalid value for QC's CABL mode in EM mode, nothing applied\n");
		return count;
	}
	em_cabc_type = (int)val;
#endif

	return count;
}

static ssize_t mipi_nt35516_cabc_read(
	struct file *file,
	char __user *buff,
	size_t count,
	loff_t *ppos)
{
	int len = 0;


	if (*ppos)
		return 0;	/* the end */

	len = snprintf(debug_buf, sizeof(debug_buf), "[ATTR] CABC type is %d\n", em_cabc_type);

	if (len < 0)
		return 0;

	if (copy_to_user(buff, debug_buf, len))
		return -EFAULT;

	*ppos += len;	/* increase offset */

	return len;
}
static const struct file_operations mipi_nt35516_cabc_fops = {
	.open = mipi_nt35516_cabc_open,
	.release = mipi_nt35516_cabc_release,
	.read = mipi_nt35516_cabc_read,
	.write = mipi_nt35516_cabc_write,
};

int nt35516_lcd_debugfs_init(void)
{
	struct dentry *dent = debugfs_create_dir("mipi_nt35516", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return -1;
	}

	if (debugfs_create_file("pwm", 0666, dent, 0, &mipi_nt35516_pwm_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	if (debugfs_create_file("mode", 0222, dent, 0, &mipi_nt35516_mode_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	if (debugfs_create_file("cabc", 0666, dent, 0, &mipi_nt35516_cabc_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	return 0;
}

#endif // end of FB_MSM_MIPI_DSI_NT35516_LCMINFO, EMList - Debugfs: PWM, BL_mode, CABC.



DEFINE_LED_TRIGGER(bkl_led_trigger);

/* Terry Cheng, 20121011, Log backlight using time {*/
#ifdef CONFIG_PM_LOG
ssize_t dump_bl_using_time( char *buf, int bufer_size)
{
	ssize_t len = 0;
	int i;
	ulong flags;
	struct timespec end_time;
	//For debug
	//struct timespec total_time;
	//memset(&total_time, 0,  sizeof(struct timespec));

	//Force Update using time 
	for (i = 0; i < NUM_OF_BACKLIGHT_LOG_LEVEL; i++) {
		LCD_PRINTK(1,"Before update BL[%d] runtime %ld.%ld\n", i, backlight_runtime[i].tv_sec, backlight_runtime[i].tv_nsec/NSEC_PER_MSEC);
	}
	end_time = ktime_to_timespec(alarm_get_elapsed_realtime());
	
	//backlight_runtime[current_bl_log_level] = timespec_add_safe(backlight_runtime[current_bl_log_level], timespec_sub(latest_update_time, start_time));
	if (timespec_compare(&start_time, &latest_update_time) > 0){
		spin_lock_irqsave(&pm_log_lock, flags);	
		backlight_runtime[current_bl_log_level] = timespec_add_safe(backlight_runtime[current_bl_log_level], timespec_sub(end_time, start_time));
		spin_unlock_irqrestore(&pm_log_lock, flags);
	}else{
		spin_lock_irqsave(&pm_log_lock, flags);	
		backlight_runtime[current_bl_log_level] = timespec_add_safe(backlight_runtime[current_bl_log_level], timespec_sub(end_time, latest_update_time));
		spin_unlock_irqrestore(&pm_log_lock, flags);
	}

	LCD_PRINTK(1, "%s(),current_bl_log_level %d, latest_update_time: %ld.%ld  \n", __func__, current_bl_log_level, latest_update_time.tv_sec, latest_update_time.tv_nsec/NSEC_PER_MSEC);	

	for (i = 0; i < NUM_OF_BACKLIGHT_LOG_LEVEL; i++) {
		len += snprintf(buf + len,bufer_size -len, "BL[%d] runtime %ld.%ld\n", i, backlight_runtime[i].tv_sec, backlight_runtime[i].tv_nsec/NSEC_PER_MSEC);
		//For debug
		//total_time = timespec_add_safe(total_time, backlight_runtime[i]);
		LCD_PRINTK(1, "After update BL[%d] runtime %ld.%ld\n", i, backlight_runtime[i].tv_sec, backlight_runtime[i].tv_nsec/NSEC_PER_MSEC);
	}
	//For debug
	//len += snprintf(buf + len,bufer_size -len, "total_time:  %ld.%ld\n", total_time.tv_sec, total_time.tv_nsec/NSEC_PER_MSEC);
	latest_update_time = ktime_to_timespec(alarm_get_elapsed_realtime());
	//Reset backlight using time
	spin_lock_irqsave(&pm_log_lock, flags);	
	memset(backlight_runtime, 0, NUM_OF_BACKLIGHT_LOG_LEVEL* sizeof(struct timespec));
	spin_unlock_irqrestore(&pm_log_lock, flags);
	return len;
}
EXPORT_SYMBOL(dump_bl_using_time);
#endif //CONFIG_PM_LOG

static void mipi_trust_nt35516_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
	struct dcs_cmd_req cmdreq;
#endif
#ifdef CONFIG_PM_LOG
	int tmp = 0;
	ulong flags;
#endif
	mipi  = &mfd->panel_info.mipi;

	if (!mfd->panel_power_on)
		return;

	if (mipi_trust_nt35516_pdata &&
	    mipi_trust_nt35516_pdata->gpio_set_backlight) {
		mipi_trust_nt35516_pdata->gpio_set_backlight(mfd->bl_level);
		return;
	}

#if LCD_BL_LOG_AFTER_SCREENON
	if (atomic_read(&LcdBlLogAfterResume) > 0 )
	{
		LCD_PRINTK(0, "%s(), BL value is %d, #%d\n", __func__, mfd->bl_level, atomic_read(&LcdBlLogAfterResume));
		atomic_dec(&LcdBlLogAfterResume);
	}
#endif

#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
// PWM from LCM.

	LCD_PRINTK(1, "%s(), [LCM] BL value is %d\n", __func__, mfd->bl_level);
/*
	if ((mipi_trust_nt35516_pdata->enable_wled_bl_ctrl)
	    && (wled_trigger_initialized)) {
		led_trigger_event(bkl_led_trigger, mfd->bl_level);
		return;
	}
*/
	wr_disbv[1] = (unsigned char)mfd->bl_level;

	cmdreq.cmds = &nt35516_cmd_backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

	// LCM_PWM mode, set PMIC_WLED as 100% (PMIC-PWM) duty cycle
	// WLED output current = CABC(LCM duty cycle)*PWM(PM8038 setting)*ILED(LED current)
	mutex_lock(&mfd->dma->ov_mutex);
	led_wled_set_backlight(MIPI_TRULY_OTM9608A_PWM_LEVEL);
	mutex_unlock(&mfd->dma->ov_mutex);
#else
// PWM from PMIC.

	LCD_PRINTK(1, "%s(), [PMIC] BL value is %d\n", __func__, mfd->bl_level);

#if defined(PM8038_WLED_OUTPUT_WA)
	mutex_lock(&mfd->dma->ov_mutex);
	if(mfd->bl_level <= PM8038_WLED_PWM_MIN_LEVEL)
		led_wled_set_backlight(PM8038_WLED_PWM_MIN_LEVEL);
	else
		led_wled_set_backlight(mfd->bl_level);
	mutex_unlock(&mfd->dma->ov_mutex);
#else
	mutex_lock(&mfd->dma->ov_mutex);
	led_wled_set_backlight(mfd->bl_level);
	mutex_unlock(&mfd->dma->ov_mutex);
#endif
#endif

#ifdef CONFIG_PM_LOG
	//Check backlight wheter disable. If disabled, we do not to count the run time
	if (mfd->bl_level == 0)
		tmp = -1;
	else
		tmp = mfd->bl_level / NUM_OF_BACKLIGHT_PER_LEVEL;
	LCD_PRINTK(1, "%s(),current_bl_log_level %d, tmp = %d\n", __func__, current_bl_log_level, tmp);			
	if((current_bl_log_level == -1) && (tmp != -1))
	{
		start_time = ktime_to_timespec(alarm_get_elapsed_realtime());
		current_bl_log_level = tmp;
	}
	else
	{
		//Check whether level whether the same if not, update using time 
		if( current_bl_log_level !=  tmp)
		{
			struct timespec end_time = ktime_to_timespec(alarm_get_elapsed_realtime());
			LCD_PRINTK(1, "%s(),current_bl_log_level %d, latest_update_time : %ld.%ld\n", __func__, current_bl_log_level, latest_update_time.tv_sec, latest_update_time.tv_nsec/NSEC_PER_MSEC);				
			if (timespec_compare(&start_time, &latest_update_time) > 0){
				spin_lock_irqsave(&pm_log_lock, flags);					
				backlight_runtime[current_bl_log_level] = timespec_add_safe(backlight_runtime[current_bl_log_level], timespec_sub(end_time, start_time));
				spin_unlock_irqrestore(&pm_log_lock, flags);		
			}else{
				spin_lock_irqsave(&pm_log_lock, flags);	
				backlight_runtime[current_bl_log_level] = timespec_add_safe(backlight_runtime[current_bl_log_level], timespec_sub(end_time, latest_update_time));
				spin_unlock_irqrestore(&pm_log_lock, flags);		
			}
			LCD_PRINTK(1, "%s(),current_bl_log_level %d,end time: %ld.%ld  backlight_runtime[%d] : %ld.%ld\n", __func__, current_bl_log_level,
									end_time.tv_sec, end_time.tv_nsec/NSEC_PER_MSEC,
									current_bl_log_level, 
									backlight_runtime[current_bl_log_level].tv_sec,
									backlight_runtime[current_bl_log_level].tv_nsec/NSEC_PER_MSEC);				
			//Check backlight wheter disable. If disabled, we do not to count the run time
			if (tmp != -1){
				current_bl_log_level = tmp;
				start_time = ktime_to_timespec(alarm_get_elapsed_realtime());
			}	
			else{
				current_bl_log_level = -1;
			}	
		}		
	}
	LCD_PRINTK(1, "%s(),current_bl_log_level %d, tmp = %d, start time : %ld.%ld\n", __func__, current_bl_log_level, tmp, start_time.tv_sec, start_time.tv_nsec/NSEC_PER_MSEC);				
#endif 	//CONFIG_PM_LOG	


#if FB_MSM_MIPI_DSI_NT35516_LCMINFO
#if defined(PM8038_WLED_OUTPUT_WA)
	if(mfd->bl_level <= PM8038_WLED_PWM_MIN_LEVEL)
		system_bl_level_old = PM8038_WLED_PWM_MIN_LEVEL;
	else
	system_bl_level_old = mfd->bl_level;
#else
	system_bl_level_old = mfd->bl_level;
#endif
#endif

}
/* } Terry Cheng, 20121011, Log backlight using time */


static void lcd_nt35516_create_kernel_debuglevel_entries(void)
{
	printk(KERN_ERR "-- lcd create kernel debuglevel --\n");

	if (kernel_debuglevel_dir != NULL) {
		debugfs_create_u32("lcd_nt35516_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&LCD_LOGLEVEL_DLL));
	} else {
		printk(KERN_ERR "LCD_TFT3P2634-E kernel debuglevel dir falied\n");
	}
}

static int proc_detect_panel_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "%u\n", panel_exist);
}

static int __devinit mipi_trust_nt35516_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct platform_device *current_pdev;
	static struct mipi_dsi_phy_ctrl *phy_settings;
	static char dlane_swap;

	if (pdev->id == 0) {
		mipi_trust_nt35516_pdata = pdev->dev.platform_data;

		if (mipi_trust_nt35516_pdata
			&& mipi_trust_nt35516_pdata->phy_ctrl_settings) {
			phy_settings = (mipi_trust_nt35516_pdata->phy_ctrl_settings);
		}

		if (mipi_trust_nt35516_pdata
			&& mipi_trust_nt35516_pdata->dlane_swap) {
			dlane_swap = (mipi_trust_nt35516_pdata->dlane_swap);
		}

#if FB_MSM_MIPI_DSI_NT35516_LCMINFO
	// debugfs for EMList
	nt35516_lcd_debugfs_init();
#endif	

	lcd_nt35516_create_kernel_debuglevel_entries();

#ifdef CONFIG_PM_LOG
	// Register PM log
	pmlog_device_lcd_bkl = pmlog_register_device(&pdev->dev);
#endif

	// read proc entries
	proc_entry = create_proc_entry(DETECT_PANEL_PROC_ENTRY, S_IRUGO, NULL);
	if (proc_entry == NULL) {
		pr_err("failed to create detect_panel entry\n");
		return -EPERM;
	}
	proc_entry->read_proc = proc_detect_panel_read;

		return 0;
	}

	current_pdev = msm_fb_add_device(pdev);

	if (current_pdev) {
		mfd = platform_get_drvdata(current_pdev);
		if (!mfd)
			return -ENODEV;
		if (mfd->key != MFD_KEY)
			return -EINVAL;

		mipi  = &mfd->panel_info.mipi;

		if (phy_settings != NULL)
			mipi->dsi_phy_db = phy_settings;

		if (dlane_swap)
			mipi->dlane_swap = dlane_swap;
	}
	return 0;
}

static int mipi_trust_nt35516_lcd_remove(struct platform_device *pdev)
{
#ifdef CONFIG_PM_LOG
	// UnRegister PM log
	pmlog_unregister_device(pmlog_device_lcd_bkl);
#endif
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_trust_nt35516_lcd_probe,
	.remove  = mipi_trust_nt35516_lcd_remove,
	.driver = {
		.name   = "mipi_trust_nt35516",
	},
};

static struct msm_fb_panel_data trust_nt35516_panel_data = {
	.on		= mipi_trust_nt35516_lcd_on,
	.off		= mipi_trust_nt35516_lcd_off,
	.set_backlight = mipi_trust_nt35516_set_backlight,
};

static int ch_used[3];

int mipi_trust_nt35516_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_trust_nt35516_lcd_init();
	if (ret) {
		pr_err("mipi_trust_nt35516_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_trust_nt35516", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	trust_nt35516_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &trust_nt35516_panel_data,
		sizeof(trust_nt35516_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_trust_nt35516_lcd_init(void)
{

	led_trigger_register_simple("bkl_trigger", &bkl_led_trigger);
	pr_info("%s: SUCCESS (WLED TRIGGER)\n", __func__);
	wled_trigger_initialized = 1;

	mipi_dsi_buf_alloc(&trust_nt35516_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&trust_nt35516_rx_buf, DSI_BUF_SIZE);

/* Terry Cheng, 20121011, Log backlight using time {*/
#ifdef CONFIG_PM_LOG
	memset(backlight_runtime, 0, NUM_OF_BACKLIGHT_LOG_LEVEL* sizeof(struct timespec));
	latest_update_time = ktime_to_timespec(alarm_get_elapsed_realtime());
	spin_lock_init(&pm_log_lock);
#endif	//CONFIG_PM_LOG
/* } Terry Cheng, 20121011, Log backlight using time */

	return platform_driver_register(&this_driver);
}

// 20130110 Jackie, add for panel detection result which passed from apps-bootloader
static int __init panel_exist_setup(char *str)
{
	panel_exist = simple_strtol(str, NULL, 3);
	printk("from aboot, panel_exist = %d\n", panel_exist);
	return 1;
}
__setup("lcm=", panel_exist_setup);
