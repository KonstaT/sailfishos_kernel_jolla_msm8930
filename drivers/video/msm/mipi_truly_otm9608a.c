/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
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
#include "mipi_truly_otm9608a.h"
#include "mdp4.h"
#include "../../leds/leds-pm8xxx.h"
#include <mach/hwid.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>

static unsigned int panel_exist = 0;
static int panel_initialled = 0;
static char DETECT_PANEL_PROC_ENTRY[] = "detect_panel";
static struct proc_dir_entry *proc_entry;

extern struct dentry *kernel_debuglevel_dir;

#define FB_MSM_MIPI_DSI_OTM9608A_LCMINFO 1 // 0: disable, 1: enable for get LCM information(ex: PWM).

// Jackie 20121203, add LCD backlight debug log
#define LCD_BL_LOG_AFTER_SCREENON 1

#if LCD_BL_LOG_AFTER_SCREENON
atomic_t LcdBlLogAfterResume = ATOMIC_INIT(0);
#endif

/* support LCM recovery */
#define LCM_RECOVERY_SUPPORT 1

#if LCM_RECOVERY_SUPPORT
void lcm_pwr_ctrl(int on);
static int recovery_lcm_param = 0;
#define AUTO_DETECT_RECOVERY 1
#if AUTO_DETECT_RECOVERY
static struct workqueue_struct *lcm_recovery_wq;
static struct delayed_work g_lcm_recovery_work;
#endif
#endif

#if FB_MSM_MIPI_DSI_OTM9608A_LCMINFO
#define OTM9608A_DEBUG_BUF 64
static char debug_buf[OTM9608A_DEBUG_BUF];
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

static struct mipi_dsi_panel_platform_data *mipi_truly_otm9608a_pdata;

static struct dsi_buf truly_otm9608a_tx_buf;
static struct dsi_buf truly_otm9608a_rx_buf;
static int mipi_truly_otm9608a_lcd_init(void);
void mipi_dsi_cdp_panel_reset(void);

static int wled_trigger_initialized;

/* truly_otm9608a panel */
//static char sw_reset[2] = {0x01, 0x00}; /* DTYPE_DCS_WRITE */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */
//static char rgb_888[2] = {0x3A, 0x77}; /* DTYPE_DCS_WRITE1 */
//static char tear_off[2] = {0x34, 0x00};		// DTYPE_DCS_WRITE1
static char tear_on[2] = {0x35, 0x00}; /* DTYPE_DCS_WRITE1 */

//static char vci_switch1[2] = {0x00,0x00};
static char vci_switch2[4] = {0xFF,0x96,0x08,0x01};

static char vci_switch3[2] = {0x00,0x80};
static char vci_switch4[3] = {0xFF,0x96,0x08};

static char vci_switch5[2] = {0x00,0x00};
static char vci_switch6[2] = {0xA0,0x00};

static char vci_switch7[2] = {0x00,0x80};
static char vci_switch8[6] = {0xB3,0x00,0x00,0x00,0x21,0x00};

static char vci_switch9[2] = {0x00,0x92};
static char vci_switch10[2] = {0xB3,0x01};

static char vci_switch11[2] = {0x00,0xC0};
static char vci_switch12[2] = {0xB3,0x19};

static char vci_switch13[2] = {0x00,0x80};
static char vci_switch14[10] = {0xC0,0x00,0x48,0x00,0x10,0x10,0x00,0x47,0x1F,0x1F};

static char vci_switch15[2] = {0x00,0x92};
static char vci_switch16[5] = {0xC0,0x00,0x0E,0x00,0x11};

static char vci_switch17[2] = {0x00,0xA2};
static char vci_switch18[4] = {0xC0,0x01,0x10,0x00};

static char vci_switch19[2] = {0x00,0xB3};
static char vci_switch20[3] = {0xc0,0x00,0x50};
static char vci_switch21[2] = {0x00,0x81};
static char vci_switch22[2] = {0xC1,0x55};

static char vci_switch23[2] = {0x00,0x80};
static char vci_switch24[4] = {0xC4,0x00,0x84,0xFA};

static char vci_switch25[2] = {0x00,0xA0};
static char vci_switch26[9] = {0xC4,0x33,0x09,0x90,0x2B,0x33,0x09,0x90,0x54};

static char vci_switch27[2] = {0x00,0x80};
static char vci_switch28[5] = {0xC5,0x08,0x00,0x90,0x11};

static char vci_switch29[2] = {0x00,0x90};	
static char vci_switch30[8] = {0xC5,0x84,0x76,0x00,0x76,0x33,0x33,0x34};

static char vci_switch31[2] = {0x00,0xA0};
static char vci_switch32[8] = {0xC5,0x96,0x76,0x06,0x76,0x33,0x33,0x34};

static char vci_switch33[2] = {0x00,0xB0};
static char vci_switch34[3] = {0xC5,0x04,0xF8};

static char vci_switch35[2] = {0x00,0x80};
static char vci_switch36[2] = {0xC6,0x64};

static char vci_switch37[2] = {0x00,0xB0};
// Jackie test 20130325, LCM PWM frequemce, Reg[0xC6B1]: 0x10=>0x02: 45kHz
static char vci_switch38[6] = {0xC6,0x03,0x02,0x00,0x1F,0x12};

static char vci_switch39[2] = {0x00,0xE1};
static char vci_switch40[2] = {0xC0,0x9F};

//static char vci_switch41[2] = {0x00,0x00};
//static char vci_switch42[2] = {0xD0,0x01};

static char vci_switch43[2] = {0x00,0x00};
static char vci_switch44[3] = {0xD1,0x01,0x01};

static char vci_switch45[2] = {0x00,0xB7};
static char vci_switch46[2] = {0xB0,0x10};

static char vci_switch47[2] = {0x00,0xC0};
static char vci_switch48[2] = {0xB0,0x55};

static char vci_switch49[2] = {0x00,0xB1};
static char vci_switch50[3] = {0xB0,0x03,0x06};

static char vci_switch51[2] = {0x00,0x80};
static char vci_switch52[11] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static char vci_switch53[2] = {0x00,0x90};
static char vci_switch54[16] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static char vci_switch55[2] = {0x00,0xA0};
static char vci_switch56[16] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


static char vci_switch57[2] = {0x00,0xB0};
static char vci_switch58[11] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


static char vci_switch59[2] = {0x00,0xC0};
static char vci_switch60[16] = {0xCB,0x00,0x00,0x00,0x04,0x00,0x00,0x04,0x04,0x00,0x00,0x04,0x04,0x04,0x00,0x00};

static char vci_switch61[2] = {0x00,0xD0};
static char vci_switch62[16] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x00,0x00,0x04,0x04};

static char vci_switch63[2] = {0x00,0xE0};
static char vci_switch64[11] = {0xCB,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00};

static char vci_switch65[2] = {0x00,0xF0};
static char vci_switch66[11] = {0xCB,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static char vci_switch67[2] = {0x00,0x80};
static char vci_switch68[11] = {0xCC,0x00,0x00,0x00,0x02,0x00,0x00,0x0A,0x0E,0x00,0x00};

static char vci_switch69[2] = {0x00,0x90};
static char vci_switch70[16] = {0xCC,0x0C,0x10,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x09};

static char vci_switch71[2] = {0x00,0xA0};
static char vci_switch72[16] = {0xCC,0x0D,0x00,0x00,0x0B,0x0F,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00};

static char vci_switch73[2] = {0x00,0xB0};
static char vci_switch74[11] = {0xCC,0x00,0x00,0x00,0x02,0x00,0x00,0x0A,0x0E,0x00,0x00};

static char vci_switch75[2] = {0x00,0xC0};
static char vci_switch76[16] = {0xCC,0x0C,0x10,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x09};

static char vci_switch77[2] = {0x00,0xD0};
static char vci_switch78[16] = {0xCC,0x0D,0x00,0x00,0x0B,0x0F,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00};

static char vci_switch79[2] = {0x00,0x80};
static char vci_switch80[13] = {0xCE,0x84,0x03,0x18,0x83,0x03,0x18,0x00,0x0F,0x00,0x00,0x0F,0x00};

static char vci_switch81[2] = {0x00,0x90};
static char vci_switch82[15] = {0xCE,0x33,0xBF,0x18,0x33,0xC0,0x18,0xF0,0x00,0x00,0xF0,0x00,0x00,0x00,0x00};

static char vci_switch83[2] = {0x00,0xA0};
static char vci_switch84[15] = {0xCE,0x38,0x02,0x03,0xC1,0x00,0x18,0x00,0x38,0x01,0x03,0xC2,0x00,0x18,0x00};

static char vci_switch85[2] = {0x00,0xB0};
static char vci_switch86[15] = {0xCE,0x38,0x00,0x03,0xC3,0x00,0x18,0x00,0x30,0x00,0x03,0xC4,0x00,0x18,0x00};

static char vci_switch87[2] = {0x00,0xC0};
static char vci_switch88[15] = {0xCE,0x30,0x01,0x03,0xC5,0x00,0x18,0x00,0x30,0x02,0x03,0xC6,0x00,0x18,0x00};

static char vci_switch89[2] = {0x00,0xD0};
static char vci_switch90[15] = {0xCE,0x30,0x03,0x03,0xC7,0x00,0x18,0x00,0x30,0x04,0x03,0xC8,0x00,0x18,0x00};

static char vci_switch91[2] = {0x00,0x80};
static char vci_switch92[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};

static char vci_switch93[2] = {0x00,0x90};
static char vci_switch94[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};

static char vci_switch95[2] = {0x00,0xA0};
static char vci_switch96[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};

static char vci_switch97[2] = {0x00,0xB0};
static char vci_switch98[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};

static char vci_switch99[2] = {0x00,0xC0};
static char vci_switch100[11] = {0xCF,0x01,0x01,0x20,0x20,0x00,0x00,0x02,0x00,0x00,0x00};

static char vci_switch101[2] = {0x00,0x80};
static char vci_switch102[2] = {0xD6,0x00};

static char vci_switch103[2] = {0x00,0x00};
static char vci_switch104[2] = {0xD7,0x00};

//static char vci_switch103[2] = {0x00,0x00}; //Leon add
static char vci_switch105[3] = {0xD8,0x6F,0x6F};
//static char vci_switch103[2] = {0x00,0x00}; //Leon add
// Jackie 20121217, VCOM, original=>0x21, new=> 0x71
static char vci_switch106[2] = {0xD9,0x71};

static char vci_switch107[2] = {0x00,0x00};	
static char vci_switch108[17] = {0xE1,0x09,0x11,0x17,0x0D,0x06,0x0E,0x0A,0x08,0x05,0x09,0x0D,0x07,0x0E,0x0E,0x0A,0x08};

static char vci_switch109[2] = {0x00,0x00};
static char vci_switch110[17] = {0xE2,0x09,0x11,0x17,0x0D,0x06,0x0E,0x0A,0x08,0x05,0x09,0x0D,0x07,0x0E,0x0E,0x0A,0x08};

// 20130129 Jackie for Gamma test
//------------
static char vci_switch107_2p2[2] = {0x00,0x00};	
static char vci_switch108_2p2[17] = {0xE1,0x09,0x0E,0x11,0x0D,0x05,0x0C,0x0B,0x0A,0x05,0x08,0x0F,0x09,0x10,0x13,0x0D,0x08};
static char vci_switch109_2p2[2] = {0x00,0x00};
static char vci_switch110_2p2[17] = {0xE2,0x09,0x0E,0x11,0x0D,0x05,0x0C,0x0B,0x0A,0x05,0x08,0x0F,0x09,0x10,0x13,0x0D,0x08};
//------------

static char vci_switch111[2] = {0x00,0x00};

static char vci_switch112[4] = {0xFF,0xFF,0xFF,0xFF};

/* 20130716 Jackie, for SAPPORO and later project with LG glass { */
//static char lg_vci_switch1[2] = {0x00,0x00};
static char lg_vci_switch2[4] = {0xFF,0x96,0x08,0x01};

static char lg_vci_switch3[2] = {0x00,0x80};
static char lg_vci_switch4[3] = {0xFF,0x96,0x08};

static char lg_vci_switch5[2] = {0x00,0x00};
static char lg_vci_switch6[2] = {0xA0,0x00};

static char lg_vci_switch7[2] = {0x00,0x80};
static char lg_vci_switch8[6] = {0xB3,0x00,0x00,0x20,0x00,0x00}; // Jonas, 20130401 for LG glass

static char lg_vci_switch9[2] = {0x00,0xC0}; // Jonas, 20130401 for LG glass
// SRAM setting
static char lg_vci_switch10[2] = {0xB3,0x09};  // Jonas, 20130401 for LG glass

//static char lg_vci_switch11[2] = {0x00,0xC0}; // Not used
//static char lg_vci_switch12[2] = {0xB3,0x19}; // Not used

static char lg_vci_switch13[2] = {0x00,0x80};
// TCON setting parameters
static char lg_vci_switch14[10] = {0xC0,0x00,0x48,0x00,0x10,0x10,0x00,0x47,0x10,0x10}; // Jonas, 20130401 for LG glass

static char lg_vci_switch15[2] = {0x00,0x92};
// Panel timing setting parameter
static char lg_vci_switch16[5] = {0xC0,0x00,0x10,0x00,0x13}; // Jonas, 20130401 for LG glass

static char lg_vci_switch17[2] = {0x00,0xA2};
// Panel timing setting parameter
static char lg_vci_switch18[4] = {0xC0,0x0C,0x05,0x02}; // Jonas, 20130401 for LG glass

static char lg_vci_switch19[2] = {0x00,0xB3};
static char lg_vci_switch20[2] = {0x00,0x50}; // Jonas, 20130401 for LG glass
static char lg_vci_switch21[2] = {0x00,0x81};
static char lg_vci_switch22[2] = {0xC1,0x55}; // Oscillator adjustment for idle/normal mode

static char lg_vci_switch23[2] = {0x00,0x80};
static char lg_vci_switch24[4] = {0xC4,0x00,0x84,0xFC}; // Jonas, 20130401 for LG glass

static char lg_vci_switch241[2] = {0x00,0xA0}; // Jonas, 20130401 for LG glass
static char lg_vci_switch242[3] = {0xB3,0x10,0x00}; // Jonas, 20130401 for LG glass
static char lg_vci_switch243[2] = {0x00,0xA0}; // Jonas, 20130401 for LG glass
static char lg_vci_switch244[2] = {0xC0,0x00}; // Jonas, 20130401 for LG glass

static char lg_vci_switch25[2] = {0x00,0xA0};
static char lg_vci_switch26[9] = {0xC4,0x33,0x09,0x90,0x2B,0x33,0x09,0x90,0x54}; // DC2DC setting

static char lg_vci_switch27[2] = {0x00,0x80};
static char lg_vci_switch28[5] = {0xC5,0x08,0x00,0xA0,0x11}; // Jonas, 20130401 for LG glass

static char lg_vci_switch29[2] = {0x00,0x90};	
// Power Control setting2 for normal mode
static char lg_vci_switch30[8] = {0xC5,0x96,0x57,0x01,0x57,0x33,0x33,0x34}; // Jonas, 20130401 for LG glass

static char lg_vci_switch31[2] = {0x00,0xA0};
// Power control setting3 for idle mode
static char lg_vci_switch32[8] = {0xC5,0x96,0x57,0x00,0x57,0x33,0x33,0x34};

static char lg_vci_switch33[2] = {0x00,0xB0};
// Power control setting3 for DC voltage settings
static char lg_vci_switch34[8] = {0xC5,0x04,0xAC,0x01,0x00,0x71,0xB1,0x83};

static char lg_vci_switch35[2] = {0x00,0x80};
static char lg_vci_switch36[2] = {0xC6,0x64}; // ABC parameter

static char lg_vci_switch37[2] = {0x00,0xB0};
static char lg_vci_switch38[6] = {0xC6,0x03,0x10,0x00,0x1F,0x12}; // ABC parameter

static char lg_vci_switch39[2] = {0x00,0xB4}; // Column inversion
static char lg_vci_switch40[2] = {0xC0,0x55};

static char lg_vci_switch41[2] = {0x00,0x00}; 
static char lg_vci_switch42[2] = {0xD0,0x40}; // ID1

static char lg_vci_switch43[2] = {0x00,0x00};
static char lg_vci_switch44[3] = {0xD1,0x00,0x00}; // ID2 & ID3

static char lg_vci_switch45[2] = {0x00,0xB7};
static char lg_vci_switch46[2] = {0xB0,0x10};

static char lg_vci_switch47[2] = {0x00,0xC0};
static char lg_vci_switch48[2] = {0xB0,0x55};

static char lg_vci_switch49[2] = {0x00,0xB1};
static char lg_vci_switch50[2] = {0xB0,0x03};

static char lg_vci_switch51[2] = {0x00,0x80};
static char lg_vci_switch52[11] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static char lg_vci_switch53[2] = {0x00,0x90};
static char lg_vci_switch54[16] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

static char lg_vci_switch55[2] = {0x00,0xA0};
static char lg_vci_switch56[16] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


static char lg_vci_switch57[2] = {0x00,0xB0};
static char lg_vci_switch58[11] = {0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


static char lg_vci_switch59[2] = {0x00,0xC0};
static char lg_vci_switch60[16] = {0xCB,0x04,0x04,0x04,0x04,0x08,0x04,0x08,0x04,0x08,0x04,0x08,0x04,0x04,0x04,0x08};

static char lg_vci_switch61[2] = {0x00,0xD0};
static char lg_vci_switch62[16] = {0xCB,0x08,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x08,0x04,0x08,0x04,0x08,0x04};

static char lg_vci_switch63[2] = {0x00,0xE0};
static char lg_vci_switch64[11] = {0xCB,0x08,0x04,0x04,0x04,0x08,0x08,0x00,0x00,0x00,0x00};

static char lg_vci_switch65[2] = {0x00,0xF0};
static char lg_vci_switch66[11] = {0xCB,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static char lg_vci_switch67[2] = {0x00,0x80};
static char lg_vci_switch68[11] = {0xCC,0x26,0x25,0x23,0x24,0x00,0x0F,0x00,0x0D,0x00,0x0B};

static char lg_vci_switch69[2] = {0x00,0x90};
static char lg_vci_switch70[16] = {0xCC,0x00,0x09,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x25,0x21,0x22,0x00};

static char lg_vci_switch71[2] = {0x00,0xA0};
static char lg_vci_switch72[16] = {0xCC,0x10,0x00,0x0E,0x00,0x0C,0x00,0x0A,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00};

static char lg_vci_switch73[2] = {0x00,0xB0};
static char lg_vci_switch74[11] = {0xCC,0x25,0x26,0x21,0x22,0x00,0x0A,0x00,0x0C,0x00,0x0E};

static char lg_vci_switch75[2] = {0x00,0xC0};
static char lg_vci_switch76[16] = {0xCC,0x00,0x10,0x04,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x26,0x23,0x24,0x00};

static char lg_vci_switch77[2] = {0x00,0xD0};
static char lg_vci_switch78[16] = {0xCC,0x09,0x00,0x0B,0x00,0x0D,0x00,0x0F,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00};

static char lg_vci_switch79[2] = {0x00,0x80};
static char lg_vci_switch80[13] = {0xCE,0x8A,0x03,0x06,0x89,0x03,0x06,0x88,0x03,0x06,0x87,0x03,0x06};

static char lg_vci_switch81[2] = {0x00,0x90};
// GOA VEND and group setting
static char lg_vci_switch82[15] = {0xCE,0xF0,0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0xF0,0x00,0x00,0x00,0x00};

static char lg_vci_switch83[2] = {0x00,0xA0};
// GOA CLK1 and GOA CLK2 setting
static char lg_vci_switch84[15] = {0xCE,0x38,0x02,0x03,0xC1,0x00,0x06,0x00,0x38,0x01,0x03,0xC2,0x00,0x06,0x00};

static char lg_vci_switch85[2] = {0x00,0xB0};
// GOA CLK3 and GOA CLK4 setting
static char lg_vci_switch86[15] = {0xCE,0x38,0x00,0x03,0xC3,0x00,0x06,0x00,0x30,0x00,0x03,0xC4,0x00,0x06,0x00};

static char lg_vci_switch87[2] = {0x00,0xC0};
// GOA CLKB1 and GOA CLKB2 setting
static char lg_vci_switch88[15] = {0xCE,0x38,0x06,0x03,0xBD,0x00,0x06,0x00,0x38,0x05,0x03,0xBE,0x00,0x06,0x00};

static char lg_vci_switch89[2] = {0x00,0xD0};
// GOA CLKB3 and GOA CLKB4 setting
static char lg_vci_switch90[15] = {0xCE,0x38,0x04,0x03,0xBF,0x00,0x06,0x00,0x38,0x03,0x03,0xC0,0x00,0x06,0x00};

static char lg_vci_switch91[2] = {0x00,0x80};
// GOA CLKC1 and GOA CLKC2 setting
static char lg_vci_switch92[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};

static char lg_vci_switch93[2] = {0x00,0x90};
// GOA CLKC3 and GOA CLKC4 setting
static char lg_vci_switch94[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};

static char lg_vci_switch95[2] = {0x00,0xA0};
// GOA CLKD1 and GOA CLKD2
static char lg_vci_switch96[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};

static char lg_vci_switch97[2] = {0x00,0xB0};
// GOA CLKD3 and GOA CLKD4
static char lg_vci_switch98[15] = {0xCF,0xF0,0x00,0x00,0x10,0x00,0x00,0x00,0xF0,0x00,0x00,0x10,0x00,0x00,0x00};

static char lg_vci_switch99[2] = {0x00,0xC0};
// GOA ECLK setting and GOA other options1 and GOA signal toggle option setting
static char lg_vci_switch100[11] = {0xCF,0x02,0x02,0x20,0x20,0x00,0x00,0x01,0x00,0x00,0x02};

static char lg_vci_switch101[2] = {0x00,0x81};
static char lg_vci_switch102[2] = {0xD6,0x00}; //Sharpness off

//static char lg_vci_switch103[2] = {0x00,0x00};
//static char lg_vci_switch104[2] = {0xD7,0x00};

static char lg_vci_switch103[2] = {0x00,0x00}; //Leon add
static char lg_vci_switch104[3] = {0xD8,0x87,0x87};
//static char lg_vci_switch103[2] = {0x00,0x00}; //Leon add
// Jackie 20121217, VCOM, original=>0x21, new=> 0x61
static char lg_vci_switch105[2] = {0x00,0x00};
static char lg_vci_switch106[2] = {0xD9,0x61}; // Vcom setting

static char lg_vci_switch107[2] = {0x00,0x00};	
static char lg_vci_switch108[17] = {0xE1,0x01,0x0D,0x13,0x0F,0x07,0x11,0x0B,0x0A,0x03,0x06,0x0B,0x08,0x0D,0x0E,0x09,0x01};

static char lg_vci_switch109[2] = {0x00,0x00};
static char lg_vci_switch110[17] = {0xE2,0x02,0x0F,0x15,0x0E,0x08,0x10,0x0B,0x0C,0x02,0x04,0x0B,0x04,0x0E,0x0D,0x08,0x00};

static char lg_vci_switch111[2] = {0x00,0x00};

static char lg_vci_switch112[4] = {0xFF,0xFF,0xFF,0xFF};
/* } 20130716 Jackie, for SAPPORO and later project with LG glass */

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

static struct dsi_cmd_desc otm9608a_cmd_backlight_cmd = {
	DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_disbv), wr_disbv};

#if FB_MSM_MIPI_DSI_OTM9608A_LCMINFO
static char wr_cabc_01[2] = {0x55, 0x01}; // [Write CABC CTRL] 1:UI Picture
static char wr_cabc_02[2] = {0x55, 0x02}; // [Write CABC CTRL] 2:Still Picture
static char wr_cabc_03[2] = {0x55, 0x03}; // [Write CABC CTRL] 3:Moving Image
static char wr_cabc_mb_clear[2] = {0x5E, 0x00}; // [Write CABC minimun brightness] clear to 0.

static struct dsi_cmd_desc otm9608a_cmd_em_cabc_off_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(wr_cabc_mb_clear), wr_cabc_mb_clear},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_cabc_off), wr_cabc_off},
};

static struct dsi_cmd_desc otm9608a_cmd_em_cabc_01_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_cabc_01), wr_cabc_01},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(wr_cabc_mb_clear), wr_cabc_mb_clear},
};
static struct dsi_cmd_desc otm9608a_cmd_em_cabc_02_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_cabc_02), wr_cabc_02},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(wr_cabc_mb_clear), wr_cabc_mb_clear},
};
static struct dsi_cmd_desc otm9608a_cmd_em_cabc_03_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_cabc_03), wr_cabc_03},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(wr_cabc_mb_clear), wr_cabc_mb_clear},
};
#endif // end of FB_MSM_MIPI_DSI_OTM9608A_LCMINFO
#endif // end of CONFIG_FB_MSM_BACKLIGHT_LCMPWM

static struct dsi_cmd_desc truly_otm9608a_video_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch2), vci_switch2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch3), vci_switch3},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch4), vci_switch4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch5), vci_switch5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch6), vci_switch6},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch7), vci_switch7},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch8), vci_switch8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch9), vci_switch9},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch10), vci_switch10},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch11), vci_switch11},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch12), vci_switch12},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch13), vci_switch13},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch14), vci_switch14},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch15), vci_switch15},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch16), vci_switch16},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch17), vci_switch17},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch18), vci_switch18},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch19), vci_switch19},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch20), vci_switch20},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch21), vci_switch21},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch22), vci_switch22},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch23), vci_switch23},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch24), vci_switch24},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch25), vci_switch25},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch26), vci_switch26},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch27), vci_switch27},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch28), vci_switch28},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch29), vci_switch29},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch30), vci_switch30},  //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch31), vci_switch31},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch32), vci_switch32},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch33), vci_switch33},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch34), vci_switch34},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch35), vci_switch35},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch36), vci_switch36},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch37), vci_switch37},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch38), vci_switch38},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch39), vci_switch39},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch40), vci_switch40}, //ok	
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(vci_switch41), vci_switch41},
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(vci_switch42), vci_switch42},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch43), vci_switch43},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch44), vci_switch44},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch45), vci_switch45},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch46), vci_switch46},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch47), vci_switch47},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch48), vci_switch48},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch49), vci_switch49},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch50), vci_switch50}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch51), vci_switch51},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch52), vci_switch52},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch53), vci_switch53},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch54), vci_switch54},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch55), vci_switch55},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch56), vci_switch56},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch57), vci_switch57},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch58), vci_switch58},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch59), vci_switch59},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch60), vci_switch60}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch61), vci_switch61},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch62), vci_switch62},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch63), vci_switch63},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch64), vci_switch64},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch65), vci_switch65},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch66), vci_switch66},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch67), vci_switch67},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch68), vci_switch68},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch69), vci_switch69},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch70), vci_switch70},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch71), vci_switch71},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch72), vci_switch72},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch73), vci_switch73},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch74), vci_switch74},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch75), vci_switch75},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch76), vci_switch76},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch77), vci_switch77},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch78), vci_switch78},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch79), vci_switch79},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch80), vci_switch80}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(vci_switch81), vci_switch81},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch82), vci_switch82},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch83), vci_switch83},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch84), vci_switch84},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch85), vci_switch85},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch86), vci_switch86},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch87), vci_switch87},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch88), vci_switch88},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch89), vci_switch89},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch90), vci_switch90}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(vci_switch91), vci_switch91},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch92), vci_switch92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch93), vci_switch93},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch94), vci_switch94},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch95), vci_switch95},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch96), vci_switch96},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch97), vci_switch97},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch98), vci_switch98},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch99), vci_switch99},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch100), vci_switch100},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(vci_switch101), vci_switch101},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch102), vci_switch102},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch103), vci_switch103},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch104), vci_switch104},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch103), vci_switch103}, //Leon add
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch105), vci_switch105},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch103), vci_switch103}, //Leon add 		
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch106), vci_switch106},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch107), vci_switch107},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch108), vci_switch108},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch109), vci_switch109},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch110), vci_switch110},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch111), vci_switch111},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 10, // Jackie 20121203, 100=>10, need to check this with vendor.
		sizeof(vci_switch112), vci_switch112},
		
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, // Jackie 20121203, 150=>120, need to check this with vendor.
		sizeof(exit_sleep), exit_sleep},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(rgb_888), rgb_888},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(mdac), mdac},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(tear_on), tear_on},

#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_ctrld), wr_ctrld},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_disbv), wr_disbv},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc), wr_cabc},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc_mb), wr_cabc_mb},
#endif
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, // Jackie 20121019, 50 => 10
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc truly_otm9608a_cmd_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch2), vci_switch2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch3), vci_switch3},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch4), vci_switch4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch5), vci_switch5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch6), vci_switch6},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch7), vci_switch7},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch8), vci_switch8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch9), vci_switch9},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch10), vci_switch10},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch11), vci_switch11},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch12), vci_switch12},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch13), vci_switch13},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch14), vci_switch14},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch15), vci_switch15},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch16), vci_switch16},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch17), vci_switch17},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch18), vci_switch18},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch19), vci_switch19},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch20), vci_switch20},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch21), vci_switch21},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch22), vci_switch22},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch23), vci_switch23},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch24), vci_switch24},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch25), vci_switch25},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch26), vci_switch26},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch27), vci_switch27},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch28), vci_switch28},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch29), vci_switch29},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch30), vci_switch30},  //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch31), vci_switch31},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch32), vci_switch32},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch33), vci_switch33},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch34), vci_switch34},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch35), vci_switch35},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch36), vci_switch36},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch37), vci_switch37},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch38), vci_switch38},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch39), vci_switch39},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch40), vci_switch40}, //ok	
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(vci_switch41), vci_switch41},
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(vci_switch42), vci_switch42},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch43), vci_switch43},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch44), vci_switch44},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch45), vci_switch45},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch46), vci_switch46},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch47), vci_switch47},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch48), vci_switch48},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch49), vci_switch49},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch50), vci_switch50}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch51), vci_switch51},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch52), vci_switch52},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch53), vci_switch53},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch54), vci_switch54},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch55), vci_switch55},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch56), vci_switch56},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch57), vci_switch57},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch58), vci_switch58},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch59), vci_switch59},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch60), vci_switch60}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch61), vci_switch61},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch62), vci_switch62},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch63), vci_switch63},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch64), vci_switch64},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch65), vci_switch65},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch66), vci_switch66},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch67), vci_switch67},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch68), vci_switch68},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch69), vci_switch69},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch70), vci_switch70},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch71), vci_switch71},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch72), vci_switch72},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch73), vci_switch73},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch74), vci_switch74},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch75), vci_switch75},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch76), vci_switch76},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch77), vci_switch77},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch78), vci_switch78},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch79), vci_switch79},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch80), vci_switch80}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(vci_switch81), vci_switch81},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch82), vci_switch82},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch83), vci_switch83},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch84), vci_switch84},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch85), vci_switch85},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch86), vci_switch86},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch87), vci_switch87},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch88), vci_switch88},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch89), vci_switch89},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch90), vci_switch90}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(vci_switch91), vci_switch91},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch92), vci_switch92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch93), vci_switch93},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch94), vci_switch94},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch95), vci_switch95},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch96), vci_switch96},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch97), vci_switch97},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch98), vci_switch98},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch99), vci_switch99},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch100), vci_switch100},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(vci_switch101), vci_switch101},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch102), vci_switch102},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch103), vci_switch103},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch104), vci_switch104},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch103), vci_switch103}, //Leon add
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch105), vci_switch105},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch103), vci_switch103}, //Leon add 		
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch106), vci_switch106},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch107), vci_switch107},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch108), vci_switch108},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch109), vci_switch109},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch110), vci_switch110},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch111), vci_switch111},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 10,  // Jackie 20121203, 100=>10, need to check this with vendor.
		sizeof(vci_switch112), vci_switch112},
		
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,  // Jackie 20121203, 150=>120, need to check this with vendor.
		sizeof(exit_sleep), exit_sleep},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(rgb_888), rgb_888},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(mdac), mdac},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(tear_on), tear_on},

#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_ctrld), wr_ctrld},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_disbv), wr_disbv},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc), wr_cabc},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc_mb), wr_cabc_mb},
#endif
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, // Jackie 20121019, 50 => 10
		sizeof(display_on), display_on},
};

/* 20130716 Jackie, for SAPPORO and later project with LG glass { */
static struct dsi_cmd_desc lg_truly_otm9608a_video_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch2), lg_vci_switch2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch3), lg_vci_switch3},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch4), lg_vci_switch4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch5), lg_vci_switch5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch6), lg_vci_switch6},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch7), lg_vci_switch7},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch8), lg_vci_switch8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch9), lg_vci_switch9},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch10), lg_vci_switch10},
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(lg_vci_switch11), lg_vci_switch11}, // Jonas remove
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(lg_vci_switch12), lg_vci_switch12}, // Jonas remove
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch13), lg_vci_switch13},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch14), lg_vci_switch14},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch15), lg_vci_switch15},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch16), lg_vci_switch16},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch17), lg_vci_switch17},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch18), lg_vci_switch18},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch19), lg_vci_switch19},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch20), lg_vci_switch20},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch21), lg_vci_switch21},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch22), lg_vci_switch22},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch23), lg_vci_switch23},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch24), lg_vci_switch24},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,			
		sizeof(lg_vci_switch241), lg_vci_switch241}, // Jonas add
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch242), lg_vci_switch242}, // Jonas add
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch243), lg_vci_switch243}, // Jonas add
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch244), lg_vci_switch244}, // Jonas add
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch25), lg_vci_switch25},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch26), lg_vci_switch26},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch27), lg_vci_switch27},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch28), lg_vci_switch28},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch29), lg_vci_switch29},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch30), lg_vci_switch30},  //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch31), lg_vci_switch31},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch32), lg_vci_switch32},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch33), lg_vci_switch33},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch34), lg_vci_switch34},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch35), lg_vci_switch35},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch36), lg_vci_switch36},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch37), lg_vci_switch37},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch38), lg_vci_switch38},	
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,				// Jonas remove
//		sizeof(lg_vci_switch39), lg_vci_switch39},	
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,				// Jonas remove
//		sizeof(lg_vci_switch40), lg_vci_switch40}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch41), lg_vci_switch41},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch42), lg_vci_switch42},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch43), lg_vci_switch43},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch44), lg_vci_switch44},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch45), lg_vci_switch45},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch46), lg_vci_switch46},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch47), lg_vci_switch47},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch48), lg_vci_switch48},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch49), lg_vci_switch49},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch50), lg_vci_switch50}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch51), lg_vci_switch51},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch52), lg_vci_switch52},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch53), lg_vci_switch53},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch54), lg_vci_switch54},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch55), lg_vci_switch55},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch56), lg_vci_switch56},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch57), lg_vci_switch57},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch58), lg_vci_switch58},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch59), lg_vci_switch59},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch60), lg_vci_switch60}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch61), lg_vci_switch61},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch62), lg_vci_switch62},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch63), lg_vci_switch63},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch64), lg_vci_switch64},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch65), lg_vci_switch65},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch66), lg_vci_switch66},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch67), lg_vci_switch67},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch68), lg_vci_switch68},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch69), lg_vci_switch69},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch70), lg_vci_switch70},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch71), lg_vci_switch71},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch72), lg_vci_switch72},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch73), lg_vci_switch73},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch74), lg_vci_switch74},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch75), lg_vci_switch75},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch76), lg_vci_switch76},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch77), lg_vci_switch77},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch78), lg_vci_switch78},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch79), lg_vci_switch79},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch80), lg_vci_switch80}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(lg_vci_switch81), lg_vci_switch81},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch82), lg_vci_switch82},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch83), lg_vci_switch83},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch84), lg_vci_switch84},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch85), lg_vci_switch85},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch86), lg_vci_switch86},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch87), lg_vci_switch87},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch88), lg_vci_switch88},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch89), lg_vci_switch89},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch90), lg_vci_switch90}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(lg_vci_switch91), lg_vci_switch91},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch92), lg_vci_switch92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch93), lg_vci_switch93},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch94), lg_vci_switch94},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch95), lg_vci_switch95},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch96), lg_vci_switch96},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch97), lg_vci_switch97},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch98), lg_vci_switch98},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch99), lg_vci_switch99},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch100), lg_vci_switch100},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(lg_vci_switch101), lg_vci_switch101},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch102), lg_vci_switch102},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch103), lg_vci_switch103},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch104), lg_vci_switch104},
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(lg_vci_switch103), lg_vci_switch103}, // Jonas remove
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch105), lg_vci_switch105},
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(lg_vci_switch103), lg_vci_switch103}, // Jonas remove	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch106), lg_vci_switch106},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch107), lg_vci_switch107},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch108), lg_vci_switch108},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch109), lg_vci_switch109},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch110), lg_vci_switch110},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch111), lg_vci_switch111},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 10, // Jackie 20121203, 100=>10, need to check this with vendor.
		sizeof(lg_vci_switch112), lg_vci_switch112},
		
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, // Jackie 20121203, 150=>120, need to check this with vendor.
		sizeof(exit_sleep), exit_sleep},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(rgb_888), rgb_888},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(mdac), mdac},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(tear_on), tear_on},

#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_ctrld), wr_ctrld},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_disbv), wr_disbv},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc), wr_cabc},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc_mb), wr_cabc_mb},
#endif
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, // Jackie 20121019, 50 => 10
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc lg_truly_otm9608a_cmd_on_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch2), lg_vci_switch2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch3), lg_vci_switch3},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch4), lg_vci_switch4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch5), lg_vci_switch5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch6), lg_vci_switch6},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch7), lg_vci_switch7},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch8), lg_vci_switch8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch9), lg_vci_switch9},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch10), lg_vci_switch10},
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(lg_vci_switch11), lg_vci_switch11}, // Jonas remove
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(lg_vci_switch12), lg_vci_switch12}, // Jonas remove
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch13), lg_vci_switch13},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch14), lg_vci_switch14},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch15), lg_vci_switch15},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch16), lg_vci_switch16},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch17), lg_vci_switch17},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch18), lg_vci_switch18},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch19), lg_vci_switch19},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch20), lg_vci_switch20},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch21), lg_vci_switch21},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch22), lg_vci_switch22},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch23), lg_vci_switch23},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch24), lg_vci_switch24},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,			
		sizeof(lg_vci_switch241), lg_vci_switch241}, // Jonas add
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch242), lg_vci_switch242}, // Jonas add
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch243), lg_vci_switch243}, // Jonas add
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch244), lg_vci_switch244}, // Jonas add
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch25), lg_vci_switch25},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch26), lg_vci_switch26},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch27), lg_vci_switch27},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch28), lg_vci_switch28},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch29), lg_vci_switch29},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch30), lg_vci_switch30},  //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch31), lg_vci_switch31},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch32), lg_vci_switch32},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch33), lg_vci_switch33},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch34), lg_vci_switch34},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch35), lg_vci_switch35},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch36), lg_vci_switch36},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch37), lg_vci_switch37},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch38), lg_vci_switch38},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch39), lg_vci_switch39}, // Jonas column inversion
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch40), lg_vci_switch40}, // Jonas column inversion
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch41), lg_vci_switch41},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch42), lg_vci_switch42},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch43), lg_vci_switch43},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch44), lg_vci_switch44},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch45), lg_vci_switch45},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch46), lg_vci_switch46},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch47), lg_vci_switch47},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch48), lg_vci_switch48},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch49), lg_vci_switch49},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch50), lg_vci_switch50}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch51), lg_vci_switch51},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch52), lg_vci_switch52},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch53), lg_vci_switch53},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch54), lg_vci_switch54},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch55), lg_vci_switch55},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch56), lg_vci_switch56},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch57), lg_vci_switch57},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch58), lg_vci_switch58},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch59), lg_vci_switch59},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch60), lg_vci_switch60}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch61), lg_vci_switch61},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch62), lg_vci_switch62},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch63), lg_vci_switch63},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch64), lg_vci_switch64},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch65), lg_vci_switch65},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch66), lg_vci_switch66},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch67), lg_vci_switch67},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch68), lg_vci_switch68},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch69), lg_vci_switch69},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch70), lg_vci_switch70},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch71), lg_vci_switch71},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch72), lg_vci_switch72},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch73), lg_vci_switch73},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch74), lg_vci_switch74},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch75), lg_vci_switch75},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch76), lg_vci_switch76},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch77), lg_vci_switch77},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch78), lg_vci_switch78},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch79), lg_vci_switch79},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch80), lg_vci_switch80}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(lg_vci_switch81), lg_vci_switch81},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch82), lg_vci_switch82},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch83), lg_vci_switch83},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch84), lg_vci_switch84},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch85), lg_vci_switch85},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch86), lg_vci_switch86},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch87), lg_vci_switch87},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch88), lg_vci_switch88},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch89), lg_vci_switch89},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch90), lg_vci_switch90}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(lg_vci_switch91), lg_vci_switch91},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch92), lg_vci_switch92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch93), lg_vci_switch93},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch94), lg_vci_switch94},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch95), lg_vci_switch95},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch96), lg_vci_switch96},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch97), lg_vci_switch97},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch98), lg_vci_switch98},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch99), lg_vci_switch99},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch100), lg_vci_switch100},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(lg_vci_switch101), lg_vci_switch101},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch102), lg_vci_switch102},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch103), lg_vci_switch103},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch104), lg_vci_switch104},
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(lg_vci_switch103), lg_vci_switch103}, //Jonas remove
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch105), lg_vci_switch105},
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(lg_vci_switch103), lg_vci_switch103}, //Jonas remove
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch106), lg_vci_switch106},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch107), lg_vci_switch107},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch108), lg_vci_switch108},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch109), lg_vci_switch109},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(lg_vci_switch110), lg_vci_switch110},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(lg_vci_switch111), lg_vci_switch111},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 10,  // Jackie 20121203, 100=>10, need to check this with vendor.
		sizeof(lg_vci_switch112), lg_vci_switch112},
		
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,  // Jackie 20121203, 150=>120, need to check this with vendor.
		sizeof(exit_sleep), exit_sleep},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(rgb_888), rgb_888},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(mdac), mdac},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(tear_on), tear_on},

#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_ctrld), wr_ctrld},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_disbv), wr_disbv},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc), wr_cabc},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc_mb), wr_cabc_mb},
#endif
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, // Jackie 20121019, 50 => 10
		sizeof(display_on), display_on},
};
/* } 20130716 Jackie, for SAPPORO and later project with LG glass */

static struct dsi_cmd_desc truly_otm9608a_cmd_on_cmds_gamma_2p2[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch2), vci_switch2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch3), vci_switch3},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch4), vci_switch4},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch5), vci_switch5},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch6), vci_switch6},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch7), vci_switch7},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch8), vci_switch8},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch9), vci_switch9},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch10), vci_switch10},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch11), vci_switch11},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch12), vci_switch12},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch13), vci_switch13},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch14), vci_switch14},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch15), vci_switch15},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch16), vci_switch16},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch17), vci_switch17},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch18), vci_switch18},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch19), vci_switch19},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch20), vci_switch20},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch21), vci_switch21},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch22), vci_switch22},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch23), vci_switch23},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch24), vci_switch24},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch25), vci_switch25},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch26), vci_switch26},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch27), vci_switch27},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch28), vci_switch28},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch29), vci_switch29},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch30), vci_switch30},  //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch31), vci_switch31},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch32), vci_switch32},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch33), vci_switch33},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch34), vci_switch34},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch35), vci_switch35},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch36), vci_switch36},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch37), vci_switch37},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch38), vci_switch38},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch39), vci_switch39},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch40), vci_switch40}, //ok	
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(vci_switch41), vci_switch41},
//	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
//		sizeof(vci_switch42), vci_switch42},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch43), vci_switch43},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch44), vci_switch44},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch45), vci_switch45},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch46), vci_switch46},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch47), vci_switch47},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch48), vci_switch48},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch49), vci_switch49},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch50), vci_switch50}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch51), vci_switch51},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch52), vci_switch52},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch53), vci_switch53},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch54), vci_switch54},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch55), vci_switch55},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch56), vci_switch56},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch57), vci_switch57},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch58), vci_switch58},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch59), vci_switch59},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch60), vci_switch60}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch61), vci_switch61},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch62), vci_switch62},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch63), vci_switch63},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch64), vci_switch64},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch65), vci_switch65},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch66), vci_switch66},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch67), vci_switch67},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch68), vci_switch68},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch69), vci_switch69},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch70), vci_switch70},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch71), vci_switch71},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch72), vci_switch72},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch73), vci_switch73},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch74), vci_switch74},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch75), vci_switch75},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch76), vci_switch76},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch77), vci_switch77},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch78), vci_switch78},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch79), vci_switch79},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch80), vci_switch80}, //ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(vci_switch81), vci_switch81},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch82), vci_switch82},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch83), vci_switch83},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch84), vci_switch84},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch85), vci_switch85},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch86), vci_switch86},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch87), vci_switch87},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch88), vci_switch88},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch89), vci_switch89},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch90), vci_switch90}, //ok	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(vci_switch91), vci_switch91},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch92), vci_switch92},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch93), vci_switch93},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch94), vci_switch94},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch95), vci_switch95},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch96), vci_switch96},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch97), vci_switch97},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch98), vci_switch98},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch99), vci_switch99},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch100), vci_switch100},	//ok
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,		
		sizeof(vci_switch101), vci_switch101},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch102), vci_switch102},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch103), vci_switch103},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch104), vci_switch104},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch103), vci_switch103}, //Leon add
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch105), vci_switch105},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch103), vci_switch103}, //Leon add 		
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch106), vci_switch106},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch107), vci_switch107_2p2},	
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch108), vci_switch108_2p2},	
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch109), vci_switch109_2p2},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
		sizeof(vci_switch110), vci_switch110_2p2},
	{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
		sizeof(vci_switch111), vci_switch111},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 10,  // Jackie 20121203, 100=>10, need to check this with vendor.
		sizeof(vci_switch112), vci_switch112},
		
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,  // Jackie 20121203, 150=>120, need to check this with vendor.
		sizeof(exit_sleep), exit_sleep},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(rgb_888), rgb_888},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(mdac), mdac},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(tear_on), tear_on},

#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_ctrld), wr_ctrld},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_disbv), wr_disbv},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc), wr_cabc},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(wr_cabc_mb), wr_cabc_mb},
#endif
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, // Jackie 20121019, 50 => 10
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc truly_otm9608a_display_off_cmds[] = {
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(tear_off), tear_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, // Jackie 20121019, need 10 for hw spec
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

#ifdef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
// 20120616 Jackie, for turn on backlight after draw FB0.
static struct dsi_cmd_desc otm9608a_bkl_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(wr_ctrld), wr_ctrld},
//	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
//		sizeof(wr_disbv_on), wr_disbv_on},
};

static struct dsi_cmd_desc otm9608a_bkl_off_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(wr_ctrld_off), wr_ctrld_off},
};

void mipi_otm9608a_lcd_bkl_on(void)
{
	struct dcs_cmd_req cmdreq;

#if LCD_BL_LOG_AFTER_SCREENON
	LCD_PRINTK(0, "%s()\n", __func__);
#endif
	cmdreq.cmds = otm9608a_bkl_on_cmds;
	cmdreq.cmds_cnt = ARRAY_SIZE(otm9608a_bkl_on_cmds);
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
}

void mipi_otm9608a_lcd_bkl_off(void)
{
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = otm9608a_bkl_off_cmds;
	cmdreq.cmds_cnt = ARRAY_SIZE(otm9608a_bkl_off_cmds);
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
}
#else
void mipi_otm9608a_lcd_bkl_on(void)
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
void mipi_otm9608a_lcd_bkl_off(void)
{
	mutex_lock(&mfd_bkl->dma->ov_mutex);
	led_wled_set_backlight(0);
	mutex_unlock(&mfd_bkl->dma->ov_mutex);
}
#endif

// Jackie 20121211, detect panel exist or not by reading manufacture id.
static char manufacture_id[2] = {0x04, 0x00}; // DTYPE_DCS_READ

static struct dsi_cmd_desc truly_otm9608a_manufacture_id_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

static u32 manu_id;

static void mipi_truly_otm9608a_manufacture_cb(u32 data)
{
	manu_id = data & 0xFF; // manufacture id is only one byte.
	pr_info("%s: manufacture_id=0x%x\n", __func__, manu_id);
}

static uint32 mipi_truly_otm9608a_manufacture_id(struct msm_fb_data_type *mfd)
{
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = &truly_otm9608a_manufacture_id_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 3;
	cmdreq.cb = mipi_truly_otm9608a_manufacture_cb; /* call back */
	mipi_dsi_cmdlist_put(&cmdreq);
	/*
	 * blocked here, untill call back called
	 */

	return manu_id;
}

// 20130906 Jackie add it to detect panel vendor
static int lcm_detect_get_adc(void)
{
	struct pm8xxx_adc_chan_result result;
        int err = 0, adc = 0;

	err = pm8xxx_adc_mpp_config_read(PM8XXX_AMUX_MPP_6,
                                        ADC_MPP_2_AMUX6, &result);
        if (err) {
		printk(KERN_ERR "%s get adc fail, err %d\n",__func__,err);
                return err;
        }
        printk(KERN_ERR "%s chan=%d, adc_code=%d, measurement=%lld physical=%lld\n",__func__, result.chan, result.adc_code,result.measurement, result.physical);
        adc = result.physical;
        adc = adc / 1000 *3; // ADC_MPP_2_AMUX6 scale is 1:3, Div 1000 => uV->mV
        
        if (adc < 500) // ADC value, 0v~0.5v
        {
        	printk(KERN_ERR "%s: Truly Panel, adc=%d mV\n",__func__, adc);
        }
        else if ((1400 < adc)&&(adc < 1900)) // ADC value, 1.4v~1.9v
        {
        	printk(KERN_ERR "%s: TCL Panel, adc=%d mV\n",__func__, adc);
        }
        else
        	printk(KERN_ERR "%s: Unknown Panel, adc=%d mV\n",__func__, adc);
        
        return adc;
}

#if LCM_RECOVERY_SUPPORT
static void recovery_lcm(void)
{
	struct mipi_panel_info *mipi;
	struct dcs_cmd_req cmdreq;

	if((!mfd_bkl) || (!mfd_bkl->panel_power_on)) {
		LCD_PRINTK(0, "%s: ## panel_power is off\n", __func__);
		return;
	}

	mipi  = &mfd_bkl->panel_info.mipi;

	LCD_PRINTK(0, "!!! %s()++, mipi_mode=%d, bl_level=%d\n", __func__, mipi->mode, mfd_bkl->bl_level);

	if(!mfd_bkl->panel_power_on) {
		LCD_PRINTK(0, "%s: ## panel_power is already off after mutex got\n", __func__);
		return;
	}

	/* power off LCM */
	lcm_pwr_ctrl(0);
	/* power on LCM */
	lcm_pwr_ctrl(1);
	/* LCM HW reset */
	mipi_dsi_cdp_panel_reset();

	if (mipi->mode == DSI_VIDEO_MODE) {
		if(msm_project_id < SAPPORO)
		{
		cmdreq.cmds = truly_otm9608a_video_on_cmds;
		cmdreq.cmds_cnt = ARRAY_SIZE(truly_otm9608a_video_on_cmds);
		}
		else
		{
			cmdreq.cmds = lg_truly_otm9608a_video_on_cmds;
			cmdreq.cmds_cnt = ARRAY_SIZE(lg_truly_otm9608a_video_on_cmds);
		}
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		mipi_dsi_cmdlist_put(&cmdreq);
	} else {
		if(msm_project_id < SAPPORO)
		{
			if(em_gamma_type == 0)
			{
				cmdreq.cmds = truly_otm9608a_cmd_on_cmds;
				cmdreq.cmds_cnt = ARRAY_SIZE(truly_otm9608a_cmd_on_cmds);
			}
			else if(em_gamma_type == 1)
			{
				cmdreq.cmds = truly_otm9608a_cmd_on_cmds_gamma_2p2;
				cmdreq.cmds_cnt = ARRAY_SIZE(truly_otm9608a_cmd_on_cmds_gamma_2p2);
			}
			else
			{
				cmdreq.cmds = truly_otm9608a_cmd_on_cmds;
				cmdreq.cmds_cnt = ARRAY_SIZE(truly_otm9608a_cmd_on_cmds);
			}
		}	
		else
		{
			cmdreq.cmds = lg_truly_otm9608a_cmd_on_cmds;
			cmdreq.cmds_cnt = ARRAY_SIZE(lg_truly_otm9608a_cmd_on_cmds);
		}
		cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		mipi_dsi_cmdlist_put(&cmdreq);

		/* clean up ack_err_status */
		mipi_dsi_cmd_bta_sw_trigger();

		/* force to update current content in FB0 to LCM. */
		/* since LCM just finished HW-reset, and LCM's FB is empty. */
		mdp4_dsi_cmd_vsync_ctrl(mfd_bkl->fbi, 1);
		mdp4_dsi_cmd_overlay_nolock(mfd_bkl);
		mdp4_dsi_cmd_vsync_ctrl(mfd_bkl->fbi, 0);
	}

	LCD_PRINTK(0, "%s()--\n", __func__);
}

static char display_power_mode[2] = {0x0A, 0x00};
static struct dsi_cmd_desc truly_otm9608a_read_display_power_mode_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(display_power_mode), display_power_mode};
static u32 lcm_power_mode;
static void mipi_truly_otm9608a_display_power_mode_cb(u32 data)
{
	lcm_power_mode = data & 0xFF; /* display power mode is only one byte. */
	LCD_PRINTK(1,"[LCM] power mode=0x%x\n", lcm_power_mode);
}
static uint32 mipi_read_lcm_power_mode(struct msm_fb_data_type *mfd)
{
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = &truly_otm9608a_read_display_power_mode_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 1;
	cmdreq.cb = mipi_truly_otm9608a_display_power_mode_cb; /* call back */
	mipi_dsi_cmdlist_put(&cmdreq);
	/*
	 * blocked here, untill call back called
	 */

	return lcm_power_mode;
}

static char display_signal_mode[2] = {0x0E, 0x00};
static struct dsi_cmd_desc truly_otm9608a_read_display_signal_mode_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(display_signal_mode), display_signal_mode};
static u32 lcm_signal_mode;
static void mipi_truly_otm9608a_display_signal_mode_cb(u32 data)
{
	lcm_signal_mode = data & 0xFF; /* display signal mode is only one byte. */
	LCD_PRINTK(1,"[LCM] signal mode=0x%x\n", lcm_signal_mode);
}
static uint32 mipi_read_lcm_signal_mode(struct msm_fb_data_type *mfd)
{
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = &truly_otm9608a_read_display_signal_mode_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 1;
	cmdreq.cb = mipi_truly_otm9608a_display_signal_mode_cb; /* call back */
	mipi_dsi_cmdlist_put(&cmdreq);
	/*
	 * blocked here, untill call back called
	 */

	return lcm_signal_mode;
}

static int recovery_lcm_param_set(const char *val, struct kernel_param *kp)
{
	int ret;
	unsigned int pwrmode=0, pwrsignal=0;

	LCD_PRINTK(1, "%s +\n", __func__);

	if((!mfd_bkl) || (!mfd_bkl->panel_power_on)) {
		LCD_PRINTK(0, "%s: ## panel_power is off\n", __func__);
		return -1;
	}

	ret = param_set_int(val, kp);

	if(recovery_lcm_param == 1)
	{
		LCD_PRINTK(1, "[LCM] Bypass Recovery, lcm_param=%d\n", recovery_lcm_param);
		/* recovery_lcm(); */
	}
	else if(recovery_lcm_param==2)
	{
		LCD_PRINTK(1, "### [LCM] Bypass check Recovery, lcm_param=%d\n", recovery_lcm_param);
		/* recovery_lcm(); */
	}
	else if(recovery_lcm_param==3)
	{
		mutex_lock(&mfd_bkl->dma->ov_mutex);
		/* read LCM power mode */
		pwrmode = mipi_read_lcm_power_mode(mfd_bkl);

		if(pwrmode!=0x9C)
		{
			LCD_PRINTK(0, "[LCM] ### pwrmode=0x%x, pwrsignal=0x%x ###\n", pwrmode, pwrsignal);
			recovery_lcm();
		}
		else
		{
			/* read LCM signal mode */
			pwrsignal = mipi_read_lcm_signal_mode(mfd_bkl);

			if(pwrsignal!=0x80)
			{
				LCD_PRINTK(0, "[LCM] ### pwrmode=0x%x, pwrsignal=0x%x ###\n", pwrmode, pwrsignal);
				recovery_lcm();
			}
		}
		mutex_unlock(&mfd_bkl->dma->ov_mutex);
	}
	else
	{
		printk(KERN_ERR "### unknown command ###\n");
		ret = -1;
	}

	LCD_PRINTK(1, "%s -, ret=%d\n", __func__ , ret);
	return ret;
}

module_param_call(recovery, recovery_lcm_param_set, param_get_long,
		  &recovery_lcm_param, S_IWUSR | S_IRUGO);

#if AUTO_DETECT_RECOVERY
static void lcm_recovery_timer_handler( struct work_struct *work )
{
	unsigned int pwrmode=0, pwrsignal=0;

	LCD_PRINTK(1, "%s +\n", __func__);

	if((!mfd_bkl) || (!mfd_bkl->panel_power_on)) {
		LCD_PRINTK(0, "%s: ## panel_power is off\n", __func__);
		return;
	}

	mutex_lock(&mfd_bkl->dma->ov_mutex);

	/* read LCM power mode */
	pwrmode = mipi_read_lcm_power_mode(mfd_bkl);
	LCD_PRINTK(1, "[LCM] read power mode=0x%x\n", pwrmode);

	if(pwrmode!=0x9C)
	{
		LCD_PRINTK(0, "[LCM] ### pwrmode=0x%x, pwrsignal=0x%x ###\n", pwrmode, pwrsignal);
		recovery_lcm();
	}
	else
	{
		/* read LCM signal mode */
		pwrsignal = mipi_read_lcm_signal_mode(mfd_bkl);
		LCD_PRINTK(1, "[LCM] read signal mode=0x%x\n", pwrsignal);

		if(pwrsignal!=0x80)
		{
			LCD_PRINTK(0, "[LCM] ### pwrmode=0x%x, pwrsignal=0x%x ###\n", pwrmode, pwrsignal);
			recovery_lcm();
		}
	}

	mutex_unlock(&mfd_bkl->dma->ov_mutex);
	queue_delayed_work(lcm_recovery_wq, &g_lcm_recovery_work, (5000 * HZ / 1000));

	LCD_PRINTK(1, "%s -\n", __func__);
}
#endif

#endif

static int mipi_truly_otm9608a_lcd_on(struct platform_device *pdev)
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

#if FB_MSM_MIPI_DSI_OTM9608A_LCMINFO
	if(!mfd_bkl)
	{
		mfd_bkl = mfd;
		printk(KERN_ERR "%s: mfd_bkl=%08X\n", __func__, (unsigned)mfd_bkl);
	}
#endif

	pinfo = &mfd->panel_info;

	mipi  = &mfd->panel_info.mipi;

	LCD_PRINTK(0, "%s(), mipi_mode=%d, msm_project_id=%d\n", __func__, mipi->mode, msm_project_id);

#ifdef CONFIG_PANEL_ABSENCE_SUPPORT
	// Jackie 20130125, panel did NOT exist, bypass DSI commands.
	if(panel_exist==0)
	{
		LCD_PRINTK(0, "%s(), ## panel NOT exist\n", __func__);
		return 0;
	}
#endif

	if(panel_initialled == 0)
	{
		// read ADC to detect panel vendor
		lcm_detect_get_adc();
		panel_initialled = 1;
	}

	// Jackie 20121203
	mipi_dsi_cdp_panel_reset();

	if (mipi->mode == DSI_VIDEO_MODE) {
		if(msm_project_id < SAPPORO)
		{
		cmdreq.cmds = truly_otm9608a_video_on_cmds;
		cmdreq.cmds_cnt = ARRAY_SIZE(truly_otm9608a_video_on_cmds);
		}
		else // for Project SAPPORO
		{
			cmdreq.cmds = lg_truly_otm9608a_video_on_cmds;
			cmdreq.cmds_cnt = ARRAY_SIZE(lg_truly_otm9608a_video_on_cmds);
		}
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		mipi_dsi_cmdlist_put(&cmdreq);
	} else {
		if(msm_project_id < SAPPORO)
		{
			if(em_gamma_type == 0)// original setting
			{
				cmdreq.cmds = truly_otm9608a_cmd_on_cmds;
				cmdreq.cmds_cnt = ARRAY_SIZE(truly_otm9608a_cmd_on_cmds);
			}
			else if(em_gamma_type == 1) //gamma 2.2
			{
				cmdreq.cmds = truly_otm9608a_cmd_on_cmds_gamma_2p2;
				cmdreq.cmds_cnt = ARRAY_SIZE(truly_otm9608a_cmd_on_cmds_gamma_2p2);
			}
			else // use default
			{
				cmdreq.cmds = truly_otm9608a_cmd_on_cmds;
				cmdreq.cmds_cnt = ARRAY_SIZE(truly_otm9608a_cmd_on_cmds);
			}
		}	
		else // for Project SAPPORO
		{
			cmdreq.cmds = lg_truly_otm9608a_cmd_on_cmds;
			cmdreq.cmds_cnt = ARRAY_SIZE(lg_truly_otm9608a_cmd_on_cmds);
		}
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
			lcm_manu_id = mipi_truly_otm9608a_manufacture_id(mfd);
			pr_err("%s: panel_exist=%d, manufacture_id=0x%x, system_rev=0x%x\n", __func__, panel_exist, lcm_manu_id, system_rev);

			panel_initialled = 1;
		}
	}

#if FB_MSM_MIPI_DSI_OTM9608A_LCMINFO
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


#if AUTO_DETECT_RECOVERY
	queue_delayed_work(lcm_recovery_wq, &g_lcm_recovery_work, (1000 * HZ / 1000));
#endif

	LCD_PRINTK(0, "%s()--\n", __func__);
	return 0;
}

static int mipi_truly_otm9608a_lcd_off(struct platform_device *pdev)
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

#ifdef CONFIG_PANEL_ABSENCE_SUPPORT
	// Jackie 20130125, panel did NOT exist, bypass DSI commands.
	if(panel_exist==0)
	{
		LCD_PRINTK(0, "%s(), ## panel NOT exist\n", __func__);
	  	return 0;
	}
#endif

	cmdreq.cmds = truly_otm9608a_display_off_cmds;
	cmdreq.cmds_cnt = ARRAY_SIZE(truly_otm9608a_display_off_cmds);
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


#if FB_MSM_MIPI_DSI_OTM9608A_LCMINFO
// #################################################
// EMList - Debugfs: PWM, BL_mode, CABC
// #################################################

//--------------
// EMList - PWM
//--------------
static int mipi_otm9608a_pwm_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_otm9608a_pwm_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_otm9608a_pwm_write(
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
		cmdreq.cmds = otm9608a_cmd_em_cabc_off_cmds;
		cmdreq.cmds_cnt = 2;
		cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;
		mipi_dsi_cmdlist_put(&cmdreq);

		em_cabc_type = 0; // CABC is off in EM.
		LCD_PRINTK(1, "Turn off CABC first in EM mode\n");
	}

	wr_disbv[1] = (unsigned char) system_bl_level_old;

	cmdreq.cmds = &otm9608a_cmd_backlight_cmd;
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

static ssize_t mipi_otm9608a_pwm_read(
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
static const struct file_operations mipi_otm9608a_pwm_fops = {
	.open = mipi_otm9608a_pwm_open,
	.release = mipi_otm9608a_pwm_release,
	.read = mipi_otm9608a_pwm_read,
	.write = mipi_otm9608a_pwm_write,
};

//------------------
// EMList - BL_mode
//------------------
static void otm9608a_lcd_bkl_onOff(unsigned int onOff)
{
#ifdef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
	struct dcs_cmd_req cmdreq;

	static struct dsi_cmd_desc otm9608a_cmd_backlight_on_cmd = {
		DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_ctrld), wr_ctrld};
	static struct dsi_cmd_desc otm9608a_cmd_backlight_off_cmd = {
		DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(wr_ctrld_off), wr_ctrld_off};

	LCD_PRINTK(1, "%s = %d\n", __func__, onOff);

	if (onOff)
	{
		cmdreq.cmds = &otm9608a_cmd_backlight_on_cmd;
	}
	else
	{
		cmdreq.cmds = &otm9608a_cmd_backlight_off_cmd;
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

static void otm9608a_lcd_gamma(unsigned int gamma_sel)
{

/*
	struct dcs_cmd_req cmdreq;

	// original gamma setting
	static struct dsi_cmd_desc otm9608a_cmd_gamma1_cmds[] = {
		{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
			sizeof(vci_switch2), vci_switch2},
		{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
			sizeof(vci_switch3), vci_switch3},
		{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
			sizeof(vci_switch4), vci_switch4},

		{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
			sizeof(vci_switch107), vci_switch107},	
		{DTYPE_GEN_LWRITE, 1, 0, 0, 10,
			sizeof(vci_switch108), vci_switch108},	
		{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
			sizeof(vci_switch109), vci_switch109},
		{DTYPE_GEN_LWRITE, 1, 0, 0, 10,
			sizeof(vci_switch110), vci_switch110},

		{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
			sizeof(vci_switch111), vci_switch111},
		{DTYPE_GEN_LWRITE, 1, 0, 0, 10, // Jackie 20121203, 100=>10, need to check this with vendor.
			sizeof(vci_switch112), vci_switch112},
	};

	// new gamma setting, Gamma 2.2
	static struct dsi_cmd_desc otm9608a_cmd_gamma2_cmds[] = {
		{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
			sizeof(vci_switch2), vci_switch2},
		{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
			sizeof(vci_switch3), vci_switch3},
		{DTYPE_GEN_LWRITE, 1, 0, 0, 1,
			sizeof(vci_switch4), vci_switch4},

		{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
			sizeof(vci_switch107_2p2), vci_switch107_2p2},	
		{DTYPE_GEN_LWRITE, 1, 0, 0, 10,
			sizeof(vci_switch108_2p2), vci_switch108_2p2},	
		{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
			sizeof(vci_switch109_2p2), vci_switch109_2p2},
		{DTYPE_GEN_LWRITE, 1, 0, 0, 10,
			sizeof(vci_switch110_2p2), vci_switch110_2p2},

		{DTYPE_GEN_WRITE1, 1, 0, 0, 1,
			sizeof(vci_switch111), vci_switch111},
		{DTYPE_GEN_LWRITE, 1, 0, 0, 10, // Jackie 20121203, 100=>10, need to check this with vendor.
			sizeof(vci_switch112), vci_switch112},
	};

	LCD_PRINTK(0, "%s gamma_sel= %d\n", __func__, gamma_sel);

	switch(gamma_sel)
	{
		case 0: // gamma setting 1
			cmdreq.cmds = otm9608a_cmd_gamma1_cmds;
			break;
		case 1: // gamma setting 2
			cmdreq.cmds = otm9608a_cmd_gamma2_cmds;
			break;
		default: // gamma setting 1
			cmdreq.cmds = otm9608a_cmd_gamma1_cmds;
			break;
	}
	
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);

*/

	em_gamma_type = gamma_sel;

	LCD_PRINTK(0, "%s, gamma_sel=%d\n", __func__, gamma_sel);
}

// Jackie 20130116
#ifndef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
static void otm9608a_lcd_bkl_string_onOff(unsigned int string, unsigned int onOff)
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

static void otm9608a_lcd_onOff(unsigned int onOff)
{
// Todo here
}

static int mipi_otm9608a_mode_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_otm9608a_mode_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_otm9608a_mode_write(
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
			otm9608a_lcd_bkl_onOff(0);
			else if(debug_buf[1] == '1')
			otm9608a_lcd_bkl_onOff(1);
			break;

		case 'g': // test for gamma
			if(debug_buf[1] == '0')
				// orignal gamma setting
				otm9608a_lcd_gamma(0);
			else if(debug_buf[1] == '1')
				// new gamma setting, Gamma 2.2
				otm9608a_lcd_gamma(1);
			break;

		case 'v': // read ADC to detect panel vendor
				lcm_detect_get_adc();
			break;

#if LCM_RECOVERY_SUPPORT
		case 'r':
				mutex_lock(&mfd_bkl->dma->ov_mutex);
				recovery_lcm();
				mutex_unlock(&mfd_bkl->dma->ov_mutex);
			break;
#endif

#ifndef CONFIG_FB_MSM_BACKLIGHT_LCMPWM
		case 's': // 20130116 Jackie, turn on/off WLED_DRV1/WLED_DRV2
			if(debug_buf[1] == '0')
			otm9608a_lcd_bkl_string_onOff(1, 0); // WLED_DRV1, off
			else if(debug_buf[1] == '1')
			otm9608a_lcd_bkl_string_onOff(1, 1); // WLED_DRV1, on
			else if(debug_buf[1] == '2')
			otm9608a_lcd_bkl_string_onOff(2, 0); // WLED_DRV2, off
			else if(debug_buf[1] == '3')
			otm9608a_lcd_bkl_string_onOff(2, 1); // WLED_DRV2, on
			break;
#endif

		case 'l': // turn on off lcd
			if (debug_buf[1] == '0')
			otm9608a_lcd_onOff(0);
			else if (debug_buf[1] == '1')
			otm9608a_lcd_onOff(1);
			break;
	}
	// Done

	return count;
}

static const struct file_operations mipi_otm9608a_mode_fops = {
	.open = mipi_otm9608a_mode_open,
	.release = mipi_otm9608a_mode_release,
	.write = mipi_otm9608a_mode_write,
};

//--------------
// EMList - CABC
//--------------
static int mipi_otm9608a_cabc_open(struct inode *inode, struct file *file)
{
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_otm9608a_cabc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mipi_otm9608a_cabc_write(
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
		cmdreq.cmds = otm9608a_cmd_em_cabc_off_cmds;
		LCD_PRINTK(1, "CABC is off in EM mode\n");
	}
	else if(val == 1)
	{
		cmdreq.cmds = otm9608a_cmd_em_cabc_01_cmds;
		LCD_PRINTK(1, "set CABC as UI Image in EM mode\n");
	}
	else if(val == 2)
	{
		cmdreq.cmds = otm9608a_cmd_em_cabc_02_cmds;
		LCD_PRINTK(1, "set CABC as Still picture in EM mode\n");
	}
	else if(val == 3)
	{
		cmdreq.cmds = otm9608a_cmd_em_cabc_03_cmds;
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

static ssize_t mipi_otm9608a_cabc_read(
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
static const struct file_operations mipi_otm9608a_cabc_fops = {
	.open = mipi_otm9608a_cabc_open,
	.release = mipi_otm9608a_cabc_release,
	.read = mipi_otm9608a_cabc_read,
	.write = mipi_otm9608a_cabc_write,
};

int otm9608a_lcd_debugfs_init(void)
{
	struct dentry *dent = debugfs_create_dir("mipi_otm9608a", NULL);

	if (IS_ERR(dent)) {
		printk(KERN_ERR "%s(%d): debugfs_create_dir fail, error %ld\n",
			__FILE__, __LINE__, PTR_ERR(dent));
		return -1;
	}

	if (debugfs_create_file("pwm", 0666, dent, 0, &mipi_otm9608a_pwm_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	if (debugfs_create_file("mode", 0222, dent, 0, &mipi_otm9608a_mode_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	if (debugfs_create_file("cabc", 0666, dent, 0, &mipi_otm9608a_cabc_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}

	return 0;
}

#endif // end of FB_MSM_MIPI_DSI_OTM9608A_LCMINFO, EMList - Debugfs: PWM, BL_mode, CABC.



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
	
	if (current_bl_log_level != -1){
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

static void mipi_truly_otm9608a_set_backlight(struct msm_fb_data_type *mfd)
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

#ifdef CONFIG_PANEL_ABSENCE_SUPPORT
	// Jackie 20130125, panel did NOT exist, bypass DSI commands.
	if(panel_exist==0)
	{
		LCD_PRINTK(1, "%s(), No panel, bypass backlight setting, bl=%d\n", __func__, mfd->bl_level);
		return;
	}
#endif

	if (mipi_truly_otm9608a_pdata &&
	    mipi_truly_otm9608a_pdata->gpio_set_backlight) {
		mipi_truly_otm9608a_pdata->gpio_set_backlight(mfd->bl_level);
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
	if ((mipi_truly_otm9608a_pdata->enable_wled_bl_ctrl)
	    && (wled_trigger_initialized)) {
		led_trigger_event(bkl_led_trigger, mfd->bl_level);
		return;
	}
*/
	wr_disbv[1] = (unsigned char)mfd->bl_level;

	cmdreq.cmds = &otm9608a_cmd_backlight_cmd;
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
	if((mfd->bl_level <= PM8038_WLED_PWM_MIN_LEVEL) && (mfd->bl_level != 0))
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


#if FB_MSM_MIPI_DSI_OTM9608A_LCMINFO
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


static void lcd_otm9608a_create_kernel_debuglevel_entries(void)
{
	printk(KERN_ERR "-- lcd create kernel debuglevel --\n");

	if (kernel_debuglevel_dir != NULL) {
		debugfs_create_u32("lcd_otm9608a_dll", S_IRUGO | S_IWUGO,
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

static int __devinit mipi_truly_otm9608a_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct platform_device *current_pdev;
	static struct mipi_dsi_phy_ctrl *phy_settings;
	static char dlane_swap;

	if (pdev->id == 0) {
		mipi_truly_otm9608a_pdata = pdev->dev.platform_data;

		if (mipi_truly_otm9608a_pdata
			&& mipi_truly_otm9608a_pdata->phy_ctrl_settings) {
			phy_settings = (mipi_truly_otm9608a_pdata->phy_ctrl_settings);
		}

		if (mipi_truly_otm9608a_pdata
			&& mipi_truly_otm9608a_pdata->dlane_swap) {
			dlane_swap = (mipi_truly_otm9608a_pdata->dlane_swap);
		}

#if FB_MSM_MIPI_DSI_OTM9608A_LCMINFO
	// debugfs for EMList
	otm9608a_lcd_debugfs_init();
#endif	

	lcd_otm9608a_create_kernel_debuglevel_entries();

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

static int mipi_truly_otm9608a_lcd_remove(struct platform_device *pdev)
{
#ifdef CONFIG_PM_LOG
	// UnRegister PM log
	pmlog_unregister_device(pmlog_device_lcd_bkl);
#endif
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_truly_otm9608a_lcd_probe,
	.remove  = mipi_truly_otm9608a_lcd_remove,
	.driver = {
		.name   = "mipi_truly_otm9608a",
	},
};

static struct msm_fb_panel_data truly_otm9608a_panel_data = {
	.on		= mipi_truly_otm9608a_lcd_on,
	.off		= mipi_truly_otm9608a_lcd_off,
	.set_backlight = mipi_truly_otm9608a_set_backlight,
};

static int ch_used[3];

int mipi_truly_otm9608a_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

#ifdef CONFIG_PANEL_ABSENCE_SUPPORT
	if(panel_exist == 0)
	{
		// Jackie 20121225, force to register Video Mode when panel does NOT exist.
		if(pinfo->type == MIPI_CMD_PANEL)
		{
			// copy from mipi_truly_otm9608a_video_qhd.c
			pinfo->type = MIPI_VIDEO_PANEL;		
			pinfo->mipi.mode = DSI_VIDEO_MODE;
			pinfo->mipi.pulse_mode_hsa_he = TRUE;
			pinfo->mipi.hfp_power_stop = FALSE;
			pinfo->mipi.hbp_power_stop = FALSE;
			pinfo->mipi.hsa_power_stop = FALSE;
			pinfo->mipi.eof_bllp_power_stop = TRUE;
			pinfo->mipi.bllp_power_stop = TRUE;
			pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
			pinfo->mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BGR;
		}
	}
#endif

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_truly_otm9608a_lcd_init();
	if (ret) {
		pr_err("mipi_truly_otm9608a_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_truly_otm9608a", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	truly_otm9608a_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &truly_otm9608a_panel_data,
		sizeof(truly_otm9608a_panel_data));
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

#if AUTO_DETECT_RECOVERY
	lcm_recovery_wq = create_singlethread_workqueue("lcm_recovery_wq");

	INIT_DELAYED_WORK( &g_lcm_recovery_work, lcm_recovery_timer_handler );
	printk(KERN_ERR "init LCM RECOVERY DELAYED_WORK\n");
#endif

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_truly_otm9608a_lcd_init(void)
{

	led_trigger_register_simple("bkl_trigger", &bkl_led_trigger);
	pr_info("%s: SUCCESS (WLED TRIGGER)\n", __func__);
	wled_trigger_initialized = 1;

	mipi_dsi_buf_alloc(&truly_otm9608a_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&truly_otm9608a_rx_buf, DSI_BUF_SIZE);

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

