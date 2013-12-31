/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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

#include "msm.h"
#include "msm_sensor.h"
#define SENSOR_NAME "ov2675"
#define PLATFORM_DRIVER_NAME "msm_camera_ov2675"
#define ov2675_obj ov2675_##obj


//for debugging only
//#define HIKO_DEBUG_800_600
//#define HIKO_DEBUG_1600_1200

#define SENSOR_PWDN_GPIO_PIN 7
#define USING_GROUP_HOLD_ON
//

//#define HIKO_DEBUG

#define FULL_SIZE_PREVIEW
static unsigned int ov2675_preview_shutter;
static unsigned int ov2675_preview_gain16;
static unsigned short ov2675_preview_binning;
static unsigned int ov2675_preview_sysclk;
static unsigned int ov2675_preview_HTS;
/* information in otp*/
static signed long long ov2675_fuse_id = 0;
static int16_t ov2675_lens_id = 0;

#ifdef CDBG
	#undef CDBG
	#define CDBG pr_err
#endif


////////////////////////
//FEATURE DEFINITIONS
////////////////////////
//#define HIKO_DEBUG
//#define USING_ISO
//#define UXGA_TO_SVGA_TRANSFER_FORMULA_BY_OV

int32_t msm_sensor_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf); 

#ifdef UXGA_TO_SVGA_TRANSFER_FORMULA_BY_OV
// Dummy pixel and Dummy line could be inserted for preview 
int32_t Preview_dummy_pixel = 0;
#define Preview_dummy_line 0
#define Default_Reg0x3028 0x07
#define Default_Reg0x3029 0x93
#define Default_Reg0x302a 0x04
#define Default_Reg0x302b 0xD4
#define Default_SVGA_maximum_shutter 618
#define Default_UXGA_maximum_shutter 1236
#define Default_UXGA_line_width 1940
#define Default_SVGA_line_width 1940
#define capture_max_gain 32
#define Capture_max_gain16 (capture_max_gain * 16)

// Dummy Pixel and Dummy Line could be inserted for capture 
#define Capture_dummy_pixel 0
#define Capture_dummy_line 0
#define Preview_PCLK_frequency 288000000
#define Capture_PCLK_frequency 576000000
uint16_t Reg0x3028 = 0x00;
uint16_t Reg0x3029 = 0x00;
uint16_t Reg0x302a = 0x00;
uint16_t Reg0x302b = 0x00;
uint16_t reg0x3000 = 0x00;
uint16_t Reg0x3000 = 0x00;
uint16_t reg0x3002 = 0x00; 
uint16_t Reg0x3002 = 0x00; 
uint16_t reg0x3003 = 0x00; 
uint16_t Reg0x3003 = 0x00; 
uint16_t reg0x3028 = 0x00; 
uint16_t reg0x3029 = 0x00; 
uint16_t reg0x302d = 0x00; 
uint16_t Reg0x302d = 0x00; 
uint16_t reg0x302e = 0x00; 
uint16_t Reg0x302e = 0x00; 
uint16_t reg0x3013 = 0x00; 
uint16_t Reg0x3013 = 0x00;
uint32_t Shutter;
uint32_t Extra_lines;
uint32_t Preview_Gain16;
uint32_t Preview_line_width;
uint32_t Capture_line_width;
uint32_t Capture_maximum_shutter;
uint32_t Capture_Gain16;
uint8_t Gain = 0x00;
uint32_t Capture_Exposure;
uint32_t Gain_Exposure;
uint32_t Preview_Exposure;
uint32_t Capture_banding_filter;
#endif //UXGA_TO_SVGA_TRANSFER_FORMULA_BY_OV


DEFINE_MUTEX(ov2675_mut);

static struct msm_sensor_ctrl_t ov2675_s_ctrl;

//static int16_t ov2675_effect = CAMERA_EFFECT_OFF;
//static int is_autoflash = 0;
static int effect_value = CAMERA_EFFECT_OFF;
static unsigned int SAT_U = 0x40;
static unsigned int SAT_V = 0x40;
static int curr_banding_type = MSM_V4L2_POWER_LINE_OFF;


typedef enum {
  SENSOR_MODE_SNAPSHOT,
  SENSOR_MODE_RAW_SNAPSHOT,
  SENSOR_MODE_PREVIEW,
  SENSOR_MODE_VIDEO,
  SENSOR_MODE_VIDEO_HD,
  SENSOR_MODE_HFR_60FPS,
  SENSOR_MODE_HFR_90FPS,
  SENSOR_MODE_HFR_120FPS,
  SENSOR_MODE_HFR_150FPS,
  SENSOR_MODE_ZSL,
  SENSOR_MODE_INVALID,
} sensor_mode_t;



static struct msm_camera_i2c_reg_conf ov2675_start_settings[] =
{
//	{ 0x3086, 0x00 },		// 0F
	{ 0x3086, 0x00 },
};

static struct msm_camera_i2c_reg_conf ov2675_stop_settings[] =
{
	{ 0x3086, 0x0f},
};

#ifdef USING_GROUP_HOLD_ON
static struct msm_camera_i2c_reg_conf ov2675_groupon_settings[] =
{
	{0x30FF, 0xFF},
};

static struct msm_camera_i2c_reg_conf ov2675_groupoff_settings[] =
{
	{0x30FF, 0x00},
};
#endif //USING_GROUP_HOLD_ON

/* move from ov2675_recommend_settings for waiting 1ms according to OV*/
static struct msm_camera_i2c_reg_conf ov2675_reset_settings[] = {
	{0x3012, 0x80},
};

static struct msm_camera_i2c_reg_conf ov2675_recommend_settings_lt1205[] =
{
    /* 20131204 version, co-work with OV for LT1205A lens*/

    //IO & Clock & Analog Setup
    {0x308c, 0x80}, 
    {0x308d, 0x0e},
    {0x360b, 0x00},
    {0x30b0, 0xff},
    {0x30b1, 0xff},
    {0x30b2, 0x24},

    {0x300e, 0x34}, 
    {0x300f, 0xa6},
    {0x3010, 0x80},
    {0x3082, 0x01},
    {0x30f4, 0x01},
    {0x3090, 0x03},
    {0x3091, 0xc0},
    {0x30ac, 0x42},

    {0x30d1, 0x08}, 
    {0x30a8, 0x54},
    {0x3015, 0x02},
    {0x3093, 0x00},
    {0x307e, 0xe5},
    {0x3079, 0x00},
    {0x30aa, 0x82},
    {0x3017, 0x40},
    {0x30f3, 0x83},
    {0x306a, 0x0c},
    {0x306d, 0x00},
    {0x336a, 0x3c},
    {0x3076, 0x6a},
    {0x30d9, 0x95},
    {0x3016, 0x52},
    {0x3601, 0x30},
    {0x304e, 0x88},
    {0x30f1, 0x82},
    {0x306f, 0x14},
    {0x3012, 0x10},
    {0x3011, 0x00},
    {0x302a, 0x03},
    {0x302b, 0x24},
    {0x302d, 0x00},
    {0x302e, 0x00},

    //AEC/AGC
    {0x3013, 0xf7}, 
    {0x3015, 0x02},
    {0x3018, 0x80},
    {0x3019, 0x70},
    {0x301a, 0xc4},

    //D5060
    {0x30af, 0x00}, 
    {0x3048, 0x1f}, 
    {0x3049, 0x4e},  
    {0x304a, 0x40},  
    {0x304f, 0x40},  
    {0x304b, 0x02}, 
    {0x304c, 0x00},  
    {0x304d, 0x42},  
    {0x304f, 0x40},  
    {0x30a3, 0x91},
    {0x30a1, 0x41},  
    {0x3013, 0xf7}, 
    {0x3014, 0x84},  
    {0x3071, 0x00},
    {0x3070, 0xb9},
    {0x3073, 0x00},
    {0x3072, 0x9a},
    {0x301c, 0x02},
    {0x301d, 0x03}, 
    {0x304d, 0x42},     
    {0x304a, 0x40},  
    {0x304f, 0x40},  
    {0x3095, 0x07},  
    {0x3096, 0x16}, 
    {0x3097, 0x1d},  

    //Window Setup
    {0x3020, 0x01},
    {0x3021, 0x1a},
    {0x3022, 0x00},
    {0x3023, 0x06},
    {0x3024, 0x06},
    {0x3025, 0x58},
    {0x3026, 0x02},
    {0x3027, 0x61},
    {0x3088, 0x02},
    {0x3089, 0x80},
    {0x308a, 0x01},
    {0x308b, 0xe0},
    {0x3316, 0x64},
    {0x3317, 0x25},
    {0x3318, 0x80},
    {0x3319, 0x08},
    {0x331a, 0x28},
    {0x331b, 0x1e},
    {0x331c, 0x00},
    {0x331d, 0x38},
    {0x3100, 0x00},

    //;AWB
    {0x3320, 0xfa},
    {0x3321, 0x11},
    {0x3322, 0x92},
    {0x3323, 0x01},
    {0x3324, 0x97},
    {0x3325, 0x02},
    {0x3326, 0xff},
    {0x3327, 0x10},
    {0x3328, 0x10},
    {0x3329, 0x1f},
    {0x332a, 0x56},
    {0x332b, 0x54},
    {0x332c, 0xbe},
    {0x332d, 0xce},
    {0x332e, 0x2e},
    {0x332f, 0x30},
    {0x3330, 0x4d},
    {0x3331, 0x44},
    {0x3332, 0xf0},
    {0x3333, 0x0a},
    {0x3334, 0xf0},
    {0x3335, 0xf0},
    {0x3336, 0xf0},
    {0x3337, 0x40},
    {0x3338, 0x40},
    {0x3339, 0x40},
    {0x333a, 0x00},
    {0x333b, 0x00},

    //;Color Matrix
    {0x3380, 0x28},
    {0x3381, 0x48},
    {0x3382, 0x12},
    {0x3383, 0x17},
    {0x3384, 0xae},
    {0x3385, 0xc5},
    {0x3386, 0xc5},
    {0x3387, 0xb8},
    {0x3388, 0x0d},
    {0x3389, 0x98},
    {0x338a, 0x01},
    {0x3398, 0x20},

    //;Gamma
    {0x3340, 0x06},
    {0x3341, 0x0c},
    {0x3342, 0x1c},
    {0x3343, 0x36},
    {0x3344, 0x4e},
    {0x3345, 0x5f},
    {0x3346, 0x6d},
    {0x3347, 0x78},
    {0x3348, 0x84},
    {0x3349, 0x95},
    {0x334a, 0xa5},
    {0x334b, 0xb4},
    {0x334c, 0xc8},
    {0x334d, 0xde},
    {0x334e, 0xf0},
    {0x334f, 0x15},

    //;Lens correction
    {0x3350, 0x33}, 
    {0x3351, 0x28}, 
    {0x3352, 0x00},
    {0x3353, 0x24},
    {0x3354, 0x00},
    {0x3355, 0x85},
    {0x3356, 0x35}, 
    {0x3357, 0x28}, 
    {0x3358, 0x00},
    {0x3359, 0x1e},
    {0x335a, 0x00},
    {0x335b, 0x85},
    {0x335c, 0x34}, 
    {0x335d, 0x28}, 
    {0x335e, 0x00},
    {0x335f, 0x1a},
    {0x3360, 0x00},
    {0x3361, 0x85},
    {0x3363, 0x70},
    {0x3364, 0x7f},
    {0x3365, 0x00},
    {0x3366, 0x00},
    {0x3362, 0x80},

    //;UVadjust
    {0x3301, 0xff},
    {0x338B, 0x13},
    {0x338c, 0x10},
    {0x338d, 0x40},

    //;Sharpness/De-noise
    {0x3370, 0xd0},
    {0x3371, 0x00},
    {0x3372, 0x00},
    {0x3373, 0x50},
    {0x3374, 0x10},
    {0x3375, 0x10},
    {0x3376, 0x09},
    {0x3377, 0x00},
    {0x3378, 0x04},
    {0x3379, 0x80},

    //;BLC
    {0x3069, 0x86},
    {0x307c, 0x10},
    {0x3087, 0x02},

    //;Other functions
    {0x3300, 0xfc},
    {0x3302, 0x11},
    {0x3400, 0x00},
    {0x3606, 0x20},
    {0x3601, 0x30},
    {0x30f3, 0x83},
    {0x304e, 0x88},

    //MIPI
    {0x363b, 0x01}, 
    {0x309e, 0x08},
    {0x3606, 0x00},
    {0x3630, 0x35},

    {0x304e, 0x04},// [7] DVP_CLK_snr
    {0x363b, 0x01},// disable cd
    {0x309e, 0x08},// disable lp_rx
    {0x3606, 0x00},// disable dvp
    {0x3084, 0x01},// scale_div_man_en
    {0x3634, 0x26},

    {0x300e, 0x34}, 
    {0x3011, 0x00},
    {0x3010, 0x80},  


    {0x3086, 0x0f}, // sleep on
    {0x3086, 0x00}, // sleep off

    /* saturation */
    {0x3391, 0x06},
    {0x3394, 0x50},
    {0x3395, 0x50},

    /* contrast */
    {0x3390, 0x41},
    {0x3398, 0x20},
    {0x3399, 0x20},

    /* AE window setting */
    {0x3030, 0x55},
    {0x3031, 0x7d},
    {0x3032, 0x7d},
    {0x3033, 0x55},   

};

static struct msm_camera_i2c_reg_conf ov2675_recommend_settings[] =
{
    /* 20131011 version, co-work with OV*/

    //IO & Clock & Analog Setup
    {0x308c, 0x80}, 
    {0x308d, 0x0e},
    {0x360b, 0x00},
    {0x30b0, 0xff},
    {0x30b1, 0xff},
    {0x30b2, 0x24},

    {0x300e, 0x34}, 
    {0x300f, 0xa6},
    {0x3010, 0x80},
    {0x3082, 0x01},
    {0x30f4, 0x01},
    {0x3090, 0x03},
    {0x3091, 0xc0},
    {0x30ac, 0x42},

    {0x30d1, 0x08}, 
    {0x30a8, 0x54},
    {0x3015, 0x02},
    {0x3093, 0x00},
    {0x307e, 0xe5},
    {0x3079, 0x00},
    {0x30aa, 0x82},
    {0x3017, 0x40},
    {0x30f3, 0x83},
    {0x306a, 0x0c},
    {0x306d, 0x00},
    {0x336a, 0x3c},
    {0x3076, 0x6a},
    {0x30d9, 0x95},
    {0x3016, 0x52},
    {0x3601, 0x30},
    {0x304e, 0x88},
    {0x30f1, 0x82},
    {0x306f, 0x14},
    {0x3012, 0x10},
    {0x3011, 0x00},
    {0x302a, 0x03},
    {0x302b, 0x24},
    {0x302d, 0x00},
    {0x302e, 0x00},

    //AEC/AGC
    {0x3013, 0xf7}, 
    {0x3015, 0x02},
    {0x3018, 0x80},
    {0x3019, 0x70},
    {0x301a, 0xc4},

    //D5060
    {0x30af, 0x00}, 
    {0x3048, 0x1f}, 
    {0x3049, 0x4e},  
    {0x304a, 0x40},  
    {0x304f, 0x40},  
    {0x304b, 0x02}, 
    {0x304c, 0x00},  
    {0x304d, 0x42},  
    {0x304f, 0x40},  
    {0x30a3, 0x91},
    {0x30a1, 0x41},  
    {0x3013, 0xf7}, 
    {0x3014, 0x84},  
    {0x3071, 0x00},
    {0x3070, 0xb9},
    {0x3073, 0x00},
    {0x3072, 0x9a},
    {0x301c, 0x02},
    {0x301d, 0x03}, 
    {0x304d, 0x42},     
    {0x304a, 0x40},  
    {0x304f, 0x40},  
    {0x3095, 0x07},  
    {0x3096, 0x16}, 
    {0x3097, 0x1d},  

    //Window Setup
    {0x3020, 0x01},
    {0x3021, 0x1a},
    {0x3022, 0x00},
    {0x3023, 0x06},
    {0x3024, 0x06},
    {0x3025, 0x58},
    {0x3026, 0x02},
    {0x3027, 0x61},
    {0x3088, 0x02},
    {0x3089, 0x80},
    {0x308a, 0x01},
    {0x308b, 0xe0},
    {0x3316, 0x64},
    {0x3317, 0x25},
    {0x3318, 0x80},
    {0x3319, 0x08},
    {0x331a, 0x28},
    {0x331b, 0x1e},
    {0x331c, 0x00},
    {0x331d, 0x38},
    {0x3100, 0x00},

    //;AWB
    {0x3320, 0xfa},
    {0x3321, 0x11},
    {0x3322, 0x92},
    {0x3323, 0x01},
    {0x3324, 0x97},
    {0x3325, 0x02},
    {0x3326, 0xff},
    {0x3327, 0x10},
    {0x3328, 0x10},
    {0x3329, 0x1f},
    {0x332a, 0x56},
    {0x332b, 0x54},
    {0x332c, 0xbe},
    {0x332d, 0xce},
    {0x332e, 0x2e},
    {0x332f, 0x30},
    {0x3330, 0x4d},
    {0x3331, 0x44},
    {0x3332, 0xf0},
    {0x3333, 0x0a},
    {0x3334, 0xf0},
    {0x3335, 0xf0},
    {0x3336, 0xf0},
    {0x3337, 0x40},
    {0x3338, 0x40},
    {0x3339, 0x40},
    {0x333a, 0x00},
    {0x333b, 0x00},

    //;Color Matrix
    {0x3380, 0x28},
    {0x3381, 0x48},
    {0x3382, 0x12},
    {0x3383, 0x15},
    {0x3384, 0x9e},
    {0x3385, 0xb3},
    {0x3386, 0xb3},
    {0x3387, 0xa7},
    {0x3388, 0x0c},
    {0x3389, 0x98},
    {0x338a, 0x01},
    {0x3398, 0x20},

    //;Gamma
    {0x3340, 0x06},
    {0x3341, 0x0c},
    {0x3342, 0x1c},
    {0x3343, 0x36},
    {0x3344, 0x4e},
    {0x3345, 0x5f},
    {0x3346, 0x6d},
    {0x3347, 0x78},
    {0x3348, 0x84},
    {0x3349, 0x95},
    {0x334a, 0xa5},
    {0x334b, 0xb4},
    {0x334c, 0xc8},
    {0x334d, 0xde},
    {0x334e, 0xf0},
    {0x334f, 0x15},

    //;Lens correction
    {0x3350, 0x33}, 
    {0x3351, 0x28}, 
    {0x3352, 0x00},
    {0x3353, 0x24},
    {0x3354, 0x00},
    {0x3355, 0x85},
    {0x3356, 0x35}, 
    {0x3357, 0x28}, 
    {0x3358, 0x00},
    {0x3359, 0x1e},
    {0x335a, 0x00},
    {0x335b, 0x85},
    {0x335c, 0x34}, 
    {0x335d, 0x28}, 
    {0x335e, 0x00},
    {0x335f, 0x1a},
    {0x3360, 0x00},
    {0x3361, 0x85},
    {0x3363, 0x70},
    {0x3364, 0x7f},
    {0x3365, 0x00},
    {0x3366, 0x00},
    {0x3362, 0x80},

    //;UVadjust
    {0x3301, 0xff},
    {0x338B, 0x13},
    {0x338c, 0x10},
    {0x338d, 0x40},

    //;Sharpness/De-noise
    {0x3370, 0xd0},
    {0x3371, 0x00},
    {0x3372, 0x00},
    {0x3373, 0x50},
    {0x3374, 0x10},
    {0x3375, 0x10},
    {0x3376, 0x05},
    {0x3377, 0x00},
    {0x3378, 0x04},
    {0x3379, 0x80},

    //;BLC
    {0x3069, 0x86},
    {0x307c, 0x10},
    {0x3087, 0x02},

    //;Other functions
    {0x3300, 0xfc},
    {0x3302, 0x11},
    {0x3400, 0x00},
    {0x3606, 0x20},
    {0x3601, 0x30},
    {0x30f3, 0x83},
    {0x304e, 0x88},

    //MIPI
    {0x363b, 0x01}, 
    {0x309e, 0x08},
    {0x3606, 0x00},
    {0x3630, 0x35},

    {0x304e, 0x04},// [7] DVP_CLK_snr
    {0x363b, 0x01},// disable cd
    {0x309e, 0x08},// disable lp_rx
    {0x3606, 0x00},// disable dvp
    {0x3084, 0x01},// scale_div_man_en
    {0x3634, 0x26},

    {0x300e, 0x34}, 
    {0x3011, 0x00},
    {0x3010, 0x80},  


    {0x3086, 0x0f}, // sleep on
    {0x3086, 0x00}, // sleep off

    /* saturation */
    {0x3391, 0x06},
    {0x3394, 0x50},
    {0x3395, 0x50},

    /* contrast */
    {0x3390, 0x41},
    {0x3398, 0x20},
    {0x3399, 0x20},

    /* AE window setting */
    {0x3030, 0x55},
    {0x3031, 0x7d},
    {0x3032, 0x7d},
    {0x3033, 0x55},   

};

static struct msm_camera_i2c_reg_conf ov2675_prev_settings[] =
{
#ifdef HIKO_DEBUG_800_600
    { 0x308c, 0x80 },
/*
    //@@size800x600-4:3
    //99 800 600
    //98 0 0
    //100 97 800 600
    {0x3013, 0xf7}, // AE ON
    {0x3012, 0x10}, // sXGA mode
    {0x3302, 0x11}, // Scale off, UV Average off
    {0x306f, 0x14}, // BLC target for short exposure of HDR mode
    {0x3362, 0x90}, // hskip
    {0x300E, 0x34}, //clk
    {0x300F, 0xA6},
    {0x3010, 0x81},
    {0x3011, 0x00},
    {0x3020, 0x01}, // HS = 296
    {0x3021, 0x28}, // HS
    {0x3022, 0x00}, // VS = 06
    {0x3023, 0x06}, // VS
    {0x3024, 0x06}, // HW = 1624
    {0x3025, 0x58}, // HW
    {0x3026, 0x02}, // VH = 606
    {0x3027, 0x5e}, // VH
    {0x3088, 0x03}, // ISP XOUT = 800
    {0x3089, 0x20}, // ISP XOUT
    {0x308a, 0x02}, // ISP YOUT = 600
    {0x308b, 0x58}, // ISP YOUT
    {0x3310, 0x65}, //ispHi
    {0x3311, 0x4b}, //ispvi
    {0x3312, 0xc8}, //v/h
    {0x3313, 0x00},
    {0x3314, 0x00},
    {0x3315, 0x00},
    {0x3316, 0x64}, // Scale X input = 1600
    {0x3317, 0x25}, // Scale Y input = 600
    {0x3318, 0x80}, // Scale Y/X input
    {0x3319, 0x08}, // Scale X offset = 00
    {0x331a, 0x28}, // Scale X output = 640
    {0x331b, 0x1e}, // Scale Y output = 360
    {0x331c, 0x00}, // Scale Y/X output
    {0x331d, 0x38}, // Scale Y offset = 4, Scale X offset = 0
    {0x3028, 0x07}, //hts
    {0x3029, 0x93}, //hts
    {0x302a, 0x02}, // VTS = 618
    {0x302b, 0x6a}, // VTS
    {0x302c, 0x00}, // EXHTS = 0
    {0x302d, 0x00}, // EXVTS = 0
    {0x302e, 0x00}, // EXTS
*/
#elif defined(HIKO_DEBUG_1600_1200)

    { 0x308c, 0x80 },
/*
   //@@size1600x1200-4:3
    //99 1600 1200
    //98 0 0
    //100 97 1600 1200
    {0x3013, 0xf7}, // AE ON
    {0x3012, 0x10}, // sXGA mode
    {0x3302, 0x11}, // Scale off, UV Average off
    {0x306f, 0x14}, // BLC target for short exposure of HDR mode
    {0x3362, 0x90}, // hskip
    {0x300E, 0x34}, //clk
    {0x300F, 0xA6},
//  {0x3010, 0x81},
    {0x3010, 0x81},
    {0x3011, 0x00},
    {0x3020, 0x01}, // HS = 280
    {0x3021, 0x18}, // HS
    {0x3022, 0x00}, // VS = 0a
    {0x3023, 0x0a}, // VS
    {0x3024, 0x06}, // HW = 1624
    {0x3025, 0x58}, // HW
    {0x3026, 0x04}, // VH = 1212
    {0x3027, 0xbc}, // VH
    {0x3088, 0x06}, // ISP XOUT = 1600
    {0x3089, 0x40}, // ISP XOUT
    {0x308a, 0x04}, // ISP YOUT = 1200
    {0x308b, 0xb0}, // ISP YOUT
    {0x3310, 0x65}, //ispHi
    {0x3311, 0x4b}, //ispvi
    {0x3312, 0xc8}, //v/h
    {0x3313, 0x00},
    {0x3314, 0x00},
    {0x3315, 0x00},
    {0x3316, 0x64}, // Scale X input = 1600
    {0x3317, 0x4b}, // Scale Y input = default
    {0x3318, 0x00}, // Scale Y/X input
    {0x3319, 0x6c}, // Scale X offset = default
    {0x331a, 0x64}, // Scale X output = default
    {0x331b, 0x4b}, // Scale Y output = default
    {0x331c, 0x00}, // Scale Y/X output
    {0x331d, 0x6c}, // Scale Y offset = default, Scale X offset = default
    {0x3028, 0x07}, //hts
    {0x3029, 0x93}, //hts
    {0x302a, 0x04}, // VTS = 1236
    {0x302b, 0xd4}, // VTS
    {0x302c, 0x00}, // EXHTS = 0
    {0x302d, 0x00}, // EXVTS = 0
    {0x302e, 0x00}, // EXTS
*/
#else
//    { 0x308c, 0x80 },
    //@@ OV2675 UXGA to SVGA
    /* 20131011 version, co-work with OV*/
    {0x3010 ,0x80},
    {0x300e ,0x34}, 
    {0x3011 ,0x00},
    {0x3012 ,0x10},
    {0x302A ,0x03},
    {0x302B ,0x23},
    {0x306f ,0x14},
    {0x3070 ,0xc8},
    {0x3072 ,0xa9},
    {0x301c ,0x03},
    {0x301d ,0x03},
    {0x3362 ,0x90},  

    {0x3020 ,0x01}, 
    {0x3021 ,0x1a},
    {0x3022 ,0x00},
    {0x3023 ,0x06},
    {0x3024 ,0x06},
    {0x3025 ,0x58},
    {0x3026 ,0x02},
    {0x3027 ,0x5e},
    {0x3088 ,0x03},
    {0x3089 ,0x20},
    {0x308a ,0x02},
    {0x308b ,0x58},
    {0x3316 ,0x64},
    {0x3317 ,0x25},
    {0x3318 ,0x80},
    {0x3319 ,0x08},
    {0x331a ,0x64},
    {0x331b ,0x4b},
    {0x330c ,0x00},
    {0x331d ,0x38},
    {0x3302 ,0x11},
    {0x3373 ,0x40},
    {0x3376 ,0x05},
#endif
};

static struct msm_camera_i2c_reg_conf ov2675_snap_settings[] =
{
#if defined(HIKO_DEBUG_1600_1200)

	{ 0x308c, 0x80 },
/*
   //@@size1600x1200-4:3
    //99 1600 1200
    //98 0 0
    //100 97 1600 1200
    {0x3013, 0xf7}, // AE ON
    {0x3012, 0x10}, // sXGA mode
    {0x3302, 0x11}, // Scale off, UV Average off
    {0x306f, 0x14}, // BLC target for short exposure of HDR mode
    {0x3362, 0x90}, // hskip
    {0x300E, 0x34}, //clk
    {0x300F, 0xA6},
//  {0x3010, 0x81},
    {0x3010, 0x81},
    {0x3011, 0x00},
    {0x3020, 0x01}, // HS = 280
    {0x3021, 0x18}, // HS
    {0x3022, 0x00}, // VS = 0a
    {0x3023, 0x0a}, // VS
    {0x3024, 0x06}, // HW = 1624
    {0x3025, 0x58}, // HW
    {0x3026, 0x04}, // VH = 1212
    {0x3027, 0xbc}, // VH
    {0x3088, 0x06}, // ISP XOUT = 1600
    {0x3089, 0x40}, // ISP XOUT
    {0x308a, 0x04}, // ISP YOUT = 1200
    {0x308b, 0xb0}, // ISP YOUT
    {0x3310, 0x65}, //ispHi
    {0x3311, 0x4b}, //ispvi
    {0x3312, 0xc8}, //v/h
    {0x3313, 0x00},
    {0x3314, 0x00},
    {0x3315, 0x00},
    {0x3316, 0x64}, // Scale X input = 1600
    {0x3317, 0x4b}, // Scale Y input = default
    {0x3318, 0x00}, // Scale Y/X input
    {0x3319, 0x6c}, // Scale X offset = default
    {0x331a, 0x64}, // Scale X output = default
    {0x331b, 0x4b}, // Scale Y output = default
    {0x331c, 0x00}, // Scale Y/X output
    {0x331d, 0x6c}, // Scale Y offset = default, Scale X offset = default
    {0x3028, 0x07}, //hts
    {0x3029, 0x93}, //hts
    {0x302a, 0x04}, // VTS = 1236
    {0x302b, 0xd4}, // VTS
    {0x302c, 0x00}, // EXHTS = 0
    {0x302d, 0x00}, // EXVTS = 0
    {0x302e, 0x00}, // EXTS
*/
#else
//    { 0x308c, 0x80 },
    //@@ OV2675 SVGA to UXGA
    /* 20131011 version, co-work with OV*/
    {0x3012 ,0x00},
    {0x302A ,0x04},
    {0x302B ,0xd4},
    {0x306f ,0x54},
    {0x3362 ,0x80},

    {0x3020 ,0x01}, 
    {0x3021 ,0x18},
    {0x3022 ,0x00},
    {0x3023 ,0x0a},
    {0x3024 ,0x06},
    {0x3025 ,0x58},
    {0x3026 ,0x04},
    {0x3027 ,0xbc},
    {0x3088 ,0x06},
    {0x3089 ,0x40},
    {0x308a ,0x04},
    {0x308b ,0xb0},
    {0x3316 ,0x64},
    {0x3317 ,0x4b},
    {0x3318 ,0x00},
    {0x3319 ,0x2c},
    {0x331a ,0x64},
    {0x331b ,0x4b},
    {0x331c ,0x00},
    {0x331d ,0x4c},
    {0x3302 ,0x01},
    {0x3373, 0x40},
#endif
};

static struct v4l2_subdev_info ov2675_subdev_info[] =
{
	{
		.code = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array ov2675_init_conf[] =
{
        {
		&ov2675_reset_settings[0],
		ARRAY_SIZE(ov2675_reset_settings),
		1,// dealy 1 ms after sw reset
		MSM_CAMERA_I2C_BYTE_DATA,
	},
	{
		&ov2675_recommend_settings[0],
		ARRAY_SIZE(ov2675_recommend_settings),
		0,
		MSM_CAMERA_I2C_BYTE_DATA,
	},
};
static struct msm_camera_i2c_conf_array ov2675_confs[] =
{
	{
		&ov2675_snap_settings[0],
		ARRAY_SIZE(ov2675_snap_settings),
		0,
		MSM_CAMERA_I2C_BYTE_DATA,
	},
	{
		&ov2675_prev_settings[0],
		ARRAY_SIZE(ov2675_prev_settings),
		0,
		MSM_CAMERA_I2C_BYTE_DATA,
	},
};
static struct msm_camera_i2c_conf_array ov2675_init_lt1205_conf[] =
{
        {
		&ov2675_reset_settings[0],
		ARRAY_SIZE(ov2675_reset_settings),
		1,// dealy 1 ms after sw reset
		MSM_CAMERA_I2C_BYTE_DATA,
	},
	{
		&ov2675_recommend_settings_lt1205[0],
		ARRAY_SIZE(ov2675_recommend_settings_lt1205),
		0,
		MSM_CAMERA_I2C_BYTE_DATA,
	},
};

static struct msm_camera_i2c_reg_conf ov2675_saturation[][4] = {
    {{0x3394, 0x00},
     {0x3395, 0x00},},/* SATURATION LEVEL0 */

    {{0x3394, 0x08},
     {0x3395, 0x08},},/* SATURATION LEVEL1 */

    {{0x3394, 0x18},
     {0x3395, 0x18},},/* SATURATION LEVEL2 */

    {{0x3394, 0x28},
     {0x3395, 0x28},},/* SATURATION LEVEL3 */

    {{0x3394, 0x30},
     {0x3395, 0x30},},/* SATURATION LEVEL4 */

    //Saturation x1 (Default)              
    {{0x3394, 0x40},
     {0x3395, 0x40},},/* SATURATION LEVEL5 */

    {{0x3394, 0x50},
     {0x3395, 0x50},},/* SATURATION LEVEL6 */

    {{0x3394, 0x58},
     {0x3395, 0x58},},/* SATURATION LEVEL7 */

    {{0x3394, 0x68},
     {0x3395, 0x68},},/* SATURATION LEVEL8 */

    {{0x3394, 0x78},
     {0x3395, 0x78},},/* SATURATION LEVEL9 */

    {{0x3394, 0x88},
     {0x3395, 0x88},},/* SATURATION LEVEL10 */
};
static struct msm_camera_i2c_conf_array ov2675_saturation_confs[][1] = {
	{{ov2675_saturation[0], ARRAY_SIZE(ov2675_saturation[0]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[1], ARRAY_SIZE(ov2675_saturation[1]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[2], ARRAY_SIZE(ov2675_saturation[2]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[3], ARRAY_SIZE(ov2675_saturation[3]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[4], ARRAY_SIZE(ov2675_saturation[4]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[5], ARRAY_SIZE(ov2675_saturation[5]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[6], ARRAY_SIZE(ov2675_saturation[6]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[7], ARRAY_SIZE(ov2675_saturation[7]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[8], ARRAY_SIZE(ov2675_saturation[8]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[9], ARRAY_SIZE(ov2675_saturation[9]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_saturation[10], ARRAY_SIZE(ov2675_saturation[10]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
};

static int ov2675_saturation_enum_map[] = {
	MSM_V4L2_SATURATION_L0,
	MSM_V4L2_SATURATION_L1,
	MSM_V4L2_SATURATION_L2,
	MSM_V4L2_SATURATION_L3,
	MSM_V4L2_SATURATION_L4,
	MSM_V4L2_SATURATION_L5,
	MSM_V4L2_SATURATION_L6,
	MSM_V4L2_SATURATION_L7,
	MSM_V4L2_SATURATION_L8,
	MSM_V4L2_SATURATION_L9,
	MSM_V4L2_SATURATION_L10,
};
static struct msm_sensor_output_info_t ov2675_dimensions[] =
{
#ifdef HIKO_DEBUG_800_600

	{   //Snapshot
		/* 1/2 * 1/2 */
		.x_output = 0x0320, /* 800 */
		.y_output = 0x0258, /* 600 */
		.line_length_pclk = 0x0658,   /* 1624 */
		.frame_length_lines = 0x025E, /* 606 */
		.vt_pixel_clk = 576000000,//288000000,
		.op_pixel_clk = 576000000,//288000000,
		.binning_factor = 0,//1,
	},
	{   //Preview
		/* 1/2 * 1/2 */
		.x_output = 0x0320, /* 800 */
		.y_output = 0x0258, /* 600 */
		.line_length_pclk = 0x0658,   /* 1624 */
		.frame_length_lines = 0x025E, /* 606 */
		.vt_pixel_clk = 288000000,
		.op_pixel_clk = 288000000,
		.binning_factor = 0,//1,
	},

#elif defined(HIKO_DEBUG_1600_1200)

	{   //Snapshot
		/* full size */
		.x_output = 0x0640, /* 1600 */
		.y_output = 0x04B0, /* 1200 */
		.line_length_pclk = 0x0658, /* 1624 */
		.frame_length_lines = 0x04BC, /* 1212 */
		.vt_pixel_clk = 576000000,
		.op_pixel_clk = 576000000,
		.binning_factor = 1,
	},
	{   //Preview
		/* full size */
		.x_output = 0x0640, /* 1600 */
		.y_output = 0x04B0, /* 1200 */
		.line_length_pclk = 0x0658, /* 1624 */
		.frame_length_lines = 0x04BC, /* 1212 */
		.vt_pixel_clk = 576000000,
		.op_pixel_clk = 576000000,
		.binning_factor = 1,
	},

#else

	{   //Snapshot
		/* full size */
		.x_output = 0x0640, /* 1600 */
		.y_output = 0x04B0, /* 1200 */
		.line_length_pclk = 0x0658, /* 1624 */
		.frame_length_lines = 0x04BC, /* 1212 */
		.vt_pixel_clk = 30000000, // 1624*1212*15 fps
		.op_pixel_clk = 72000000,
//		.vt_pixel_clk = 576000000,
//		.op_pixel_clk = 576000000,
		.binning_factor = 1,
	},
	{   //Preview
		/* 1/2 * 1/2 */
		.x_output = 0x0320, /* 800 */
		.y_output = 0x0258, /* 600 */
		.line_length_pclk = 0x0658,   /* 1624 */
		.frame_length_lines = 0x0261, /* 606 */
		.vt_pixel_clk = 30000000,
		.op_pixel_clk = 72000000,
		.binning_factor = 1,
	},

#endif
};

static struct msm_camera_i2c_enum_conf_array ov2675_saturation_enum_confs = {
	.conf = &ov2675_saturation_confs[0][0],
	.conf_enum = ov2675_saturation_enum_map,
	.num_enum = ARRAY_SIZE(ov2675_saturation_enum_map),
	.num_index = ARRAY_SIZE(ov2675_saturation_confs),
	.num_conf = ARRAY_SIZE(ov2675_saturation_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};

static struct msm_camera_i2c_reg_conf ov2675_contrast[][4] = {
    {
        //Contrast -4                             
        {0x3390, 0x45},
        {0x3398, 0x14},
        {0x3399, 0x14}, 
    },/* CONTRAST L0*/

    {
        //Contrast -3                             
        {0x3390, 0x45},
        {0x3398, 0x14},
        {0x3399, 0x14}, 
    },/* CONTRAST L1*/
    
    {
        //Contrast -2                             
        {0x3390, 0x45},
        {0x3398, 0x18},
        {0x3399, 0x18},
    },/* CONTRAST L2*/
    
    {
        //Contrast -1                             
        {0x3390, 0x45},
        {0x3398, 0x1c},
        {0x3399, 0x1c}, 
    },/* CONTRAST L3*/
    
    {
        //Contrast (Default)                             
        {0x3390, 0x41},
        {0x3398, 0x20},
        {0x3399, 0x20},
    },/* CONTRAST L4*/
    
    {
        //Contrast +1                            
        {0x3390, 0x45},
        {0x3398, 0x24},
        {0x3399, 0x24},
    },/* CONTRAST L5*/
    
    {
        //Contrast +2                             
        {0x3390, 0x45},
        {0x3398, 0x28},
        {0x3399, 0x28},
    },/* CONTRAST L6*/
    
    {
        //Contrast +2                             
        {0x3390, 0x45},
        {0x3398, 0x2c},
        {0x3399, 0x2c},
    },/* CONTRAST L7*/
    
    {
        //Contrast +2                             
        {0x3390, 0x45},
        {0x3398, 0x30},
        {0x3399, 0x30},
    },/* CONTRAST L8*/

};

static struct msm_camera_i2c_conf_array ov2675_contrast_confs[][1] = {
	{{ov2675_contrast[0], ARRAY_SIZE(ov2675_contrast[0]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_contrast[1], ARRAY_SIZE(ov2675_contrast[1]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_contrast[2], ARRAY_SIZE(ov2675_contrast[2]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_contrast[3], ARRAY_SIZE(ov2675_contrast[3]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_contrast[4], ARRAY_SIZE(ov2675_contrast[4]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_contrast[5], ARRAY_SIZE(ov2675_contrast[5]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_contrast[6], ARRAY_SIZE(ov2675_contrast[6]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_contrast[7], ARRAY_SIZE(ov2675_contrast[7]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_contrast[8], ARRAY_SIZE(ov2675_contrast[8]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
};


static int ov2675_contrast_enum_map[] = {
	MSM_V4L2_CONTRAST_L0,
	MSM_V4L2_CONTRAST_L1,
	MSM_V4L2_CONTRAST_L2,
	MSM_V4L2_CONTRAST_L3,
	MSM_V4L2_CONTRAST_L4,
	MSM_V4L2_CONTRAST_L5,
	MSM_V4L2_CONTRAST_L6,
	MSM_V4L2_CONTRAST_L7,
	MSM_V4L2_CONTRAST_L8,
};

static struct msm_camera_i2c_enum_conf_array ov2675_contrast_enum_confs = {
	.conf = &ov2675_contrast_confs[0][0],
	.conf_enum = ov2675_contrast_enum_map,
	.num_enum = ARRAY_SIZE(ov2675_contrast_enum_map),
	.num_index = ARRAY_SIZE(ov2675_contrast_confs),
	.num_conf = ARRAY_SIZE(ov2675_contrast_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};
static struct msm_camera_i2c_reg_conf ov2675_sharpness[][5] = {
    {
        //Sharpness 0                             
        {0x3306, 0x00},
        {0x3376, 0x01}, //8x
        {0x3377, 0x00},
        {0x3378, 0x10},
        {0x3379, 0x80},
    },  /* SHARPNESS LEVEL 0*/

    {
        //Sharpness 1                             
        {0x3306, 0x00},
        {0x3376, 0x02}, //8x
        {0x3377, 0x00},
        {0x3378, 0x08},
        {0x3379, 0x80},  
    },  /* SHARPNESS LEVEL 1*/

    {
        //Sharpness_Auto (Default)
        {0x3306, 0x00},
        {0x3376, 0x05}, //8x
        {0x3377, 0x00},
        {0x3378, 0x04},
        {0x3379, 0x80},
    },  /* SHARPNESS LEVEL 2*/

    {
        //Sharpness 3                             
        {0x3306, 0x00},
        {0x3376, 0x06}, //8x
        {0x3377, 0x00},
        {0x3378, 0x04},
        {0x3379, 0x80}, 
    },  /* SHARPNESS LEVEL 3*/

    {
        //Sharpness 4                            
        {0x3306, 0x00},
        {0x3376, 0x08}, //8x
        {0x3377, 0x00},
        {0x3378, 0x04},
        {0x3379, 0x80},
    },  /* SHARPNESS LEVEL 4*/

    {
        //Sharpness 5                             
        {0x3306, 0x00},
        {0x3376, 0x0a}, //8x
        {0x3377, 0x00},
        {0x3378, 0x04},
        {0x3379, 0x80},  
    },  /* SHARPNESS LEVEL 5*/

};

static struct msm_camera_i2c_conf_array ov2675_sharpness_confs[][1] = {
	{{ov2675_sharpness[0], ARRAY_SIZE(ov2675_sharpness[0]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_sharpness[1], ARRAY_SIZE(ov2675_sharpness[1]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_sharpness[2], ARRAY_SIZE(ov2675_sharpness[2]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_sharpness[3], ARRAY_SIZE(ov2675_sharpness[3]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_sharpness[4], ARRAY_SIZE(ov2675_sharpness[4]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_sharpness[5], ARRAY_SIZE(ov2675_sharpness[5]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
};

static int ov2675_sharpness_enum_map[] = {
	MSM_V4L2_SHARPNESS_L0,
	MSM_V4L2_SHARPNESS_L1,
	MSM_V4L2_SHARPNESS_L2,
	MSM_V4L2_SHARPNESS_L3,
	MSM_V4L2_SHARPNESS_L4,
	MSM_V4L2_SHARPNESS_L5,
};

static struct msm_camera_i2c_enum_conf_array ov2675_sharpness_enum_confs = {
	.conf = &ov2675_sharpness_confs[0][0],
	.conf_enum = ov2675_sharpness_enum_map,
	.num_enum = ARRAY_SIZE(ov2675_sharpness_enum_map),
	.num_index = ARRAY_SIZE(ov2675_sharpness_confs),
	.num_conf = ARRAY_SIZE(ov2675_sharpness_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};

/*
static struct msm_sensor_exp_gain_info_t ov2675_exp_gain_info =
{
	.coarse_int_time_addr = 0x3002,//0x3015,
	.global_gain_addr = 0x3000,
	.vert_offset = 5,
};
*/

static struct msm_camera_i2c_reg_conf ov2675_exposure[][3] = {

    {
        {0x3018,0x68},
        {0x3019,0x58},
        {0x301a,0xc4}, 
    }, /*EXPOSURE COMPENSATIONN -2*/

    {
        {0x3018,0x78},
        {0x3019,0x68},
        {0x301a,0xc4},
    }, /*EXPOSURE COMPENSATIONN -1*/

    {
        {0x3018,0x88},
        {0x3019,0x78},
        {0x301a,0xc4},
    }, /*EXPOSURE COMPENSATIONN 0*/

    {
        {0x3018,0x98},
        {0x3019,0x88},
        {0x301a,0xc4},
    }, /*EXPOSURE COMPENSATIONN +1*/

    {
        {0x3018,0xa8},
        {0x3019,0x98},
        {0x301a,0xc4}, 
    }, /*EXPOSURE COMPENSATIONN +2*/

};

#if 0 
static struct msm_camera_i2c_reg_conf ov2675_exposure[][3] = {

    {
        {0x3018,0x48},
        {0x3019,0x38},
        {0x301a,0xd4}, 
    }, /*EXPOSURE COMPENSATIONN -2*/

    {
        {0x3018,0x58},
        {0x3019,0x48},
        {0x301a,0xd4},
    }, /*EXPOSURE COMPENSATIONN -1*/

    {
        {0x3018,0x68},
        {0x3019,0x58},
        {0x301a,0xd4},
    }, /*EXPOSURE COMPENSATIONN 0*/

    {
        {0x3018,0x78},
        {0x3019,0x68},
        {0x301a,0xd4}, 
    }, /*EXPOSURE COMPENSATIONN +1*/

    {
        {0x3018,0x88},
        {0x3019,0x78},
        {0x301a,0xd4},             
    }, /*EXPOSURE COMPENSATIONN +2*/
};
#endif



#if 0
static struct msm_camera_i2c_reg_conf ov2675_exposure[][3] = {

    {
        {0x3018,0x38},
        {0x3019,0x28},
        {0x301a,0xd4}, 
    }, /*EXPOSURE COMPENSATIONN -2*/

    {
        {0x3018,0x48},
        {0x3019,0x38},
        {0x301a,0xd4},
    }, /*EXPOSURE COMPENSATIONN -1*/

    {
        {0x3018,0x58},
        {0x3019,0x48},
        {0x301a,0xd4},
    }, /*EXPOSURE COMPENSATIONN 0*/

    {
        {0x3018,0x68},
        {0x3019,0x58},
        {0x301a,0xd4}, 
    }, /*EXPOSURE COMPENSATIONN +1*/

    {
        {0x3018,0x78},
        {0x3019,0x68},
        {0x301a,0xd4},             
    }, /*EXPOSURE COMPENSATIONN +2*/
};
#endif

static struct msm_camera_i2c_conf_array ov2675_exposure_confs[][1] = {
	{{ov2675_exposure[0], ARRAY_SIZE(ov2675_exposure[0]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_exposure[1], ARRAY_SIZE(ov2675_exposure[1]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_exposure[2], ARRAY_SIZE(ov2675_exposure[2]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_exposure[3], ARRAY_SIZE(ov2675_exposure[3]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_exposure[4], ARRAY_SIZE(ov2675_exposure[4]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
};

static int ov2675_exposure_enum_map[] = {
	MSM_V4L2_EXPOSURE_N2,
	MSM_V4L2_EXPOSURE_N1,
	MSM_V4L2_EXPOSURE_D,
	MSM_V4L2_EXPOSURE_P1,
	MSM_V4L2_EXPOSURE_P2,
};

static struct msm_camera_i2c_enum_conf_array ov2675_exposure_enum_confs = {
	.conf = &ov2675_exposure_confs[0][0],
	.conf_enum = ov2675_exposure_enum_map,
	.num_enum = ARRAY_SIZE(ov2675_exposure_enum_map),
	.num_index = ARRAY_SIZE(ov2675_exposure_confs),
	.num_conf = ARRAY_SIZE(ov2675_exposure_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};

#ifdef USING_ISO

static struct msm_camera_i2c_reg_conf ov2675_iso[][] = {
#if 0
	{{0x14, 0x20, 0x00, 0x00, 0x8F},},   /*ISO_AUTO*/
	{{0x14, 0x20, 0x00, 0x00, 0x8F},},   /*ISO_DEBLUR*/
	{{0x14, 0x00, 0x00, 0x00, 0x8F},},   /*ISO_100*/
	{{0x14, 0x10, 0x00, 0x00, 0x8F},},   /*ISO_200*/
	{{0x14, 0x20, 0x00, 0x00, 0x8F},},   /*ISO_400*/
	{{0x14, 0x30, 0x00, 0x00, 0x8F},},   /*ISO_800*/
	{{0x14, 0x40, 0x00, 0x00, 0x8F},},   /*ISO_1600*/
#endif
};


static struct msm_camera_i2c_conf_array ov2675_iso_confs[][1] = {
	{{ov2675_iso[0], ARRAY_SIZE(ov2675_iso[0]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_iso[1], ARRAY_SIZE(ov2675_iso[1]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_iso[2], ARRAY_SIZE(ov2675_iso[2]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_iso[3], ARRAY_SIZE(ov2675_iso[3]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_iso[4], ARRAY_SIZE(ov2675_iso[4]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_iso[5], ARRAY_SIZE(ov2675_iso[5]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
};

static int ov2675_iso_enum_map[] = {
	MSM_V4L2_ISO_AUTO ,
	MSM_V4L2_ISO_DEBLUR,
	MSM_V4L2_ISO_100,
	MSM_V4L2_ISO_200,
	MSM_V4L2_ISO_400,
	MSM_V4L2_ISO_800,
	MSM_V4L2_ISO_1600,
};


static struct msm_camera_i2c_enum_conf_array ov2675_iso_enum_confs = {
	.conf = &ov2675_iso_confs[0][0],
	.conf_enum = ov2675_iso_enum_map,
	.num_enum = ARRAY_SIZE(ov2675_iso_enum_map),
	.num_index = ARRAY_SIZE(ov2675_iso_confs),
	.num_conf = ARRAY_SIZE(ov2675_iso_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};

#endif //USING_ISO
#if 0
static struct msm_camera_i2c_reg_conf ov2675_no_effect[] = {
#if 0
	{0x81, 0x00, 0x00, 0x00, 0xDF},
	{0x28, 0x00,},
	{0xd2, 0x00,},
	{0xda, 0x80,},
	{0xdb, 0x80,},
#endif
};

static struct msm_camera_i2c_conf_array ov2675_no_effect_confs[] = {
	{&ov2675_no_effect[0],
	ARRAY_SIZE(ov2675_no_effect), 0,
	MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},
};
#endif


static struct msm_camera_i2c_reg_conf ov2675_special_effect[][3] = {
    {
        {0x3391, 0x00},
    }, /* MSM_V4L2_EFFECT_OFF */

    {
        {0x3391, 0x00},
    }, /* MSM_V4L2_EFFECT_MONO */

    {
        {0x3391, 0x40},
    }, /* MSM_V4L2_EFFECT_NEGATIVE */

    {
        {0x3391, 0x00}, /* NOT SUPPORTED AS DEFAULT */
    }, /* MSM_V4L2_EFFECT_SOLARIZE */

    {
        {0x3391, 0x18},
        {0x3396, 0x40},
        {0x3397, 0xa6},
    }, /* MSM_V4L2_EFFECT_SEPIA */

    {
        {0x3391, 0x00}, /* NOT SUPPORTED AS DEFAULT */
    }, /* MSM_V4L2_EFFECT_POSTERAIZE */

    {
        {0x3391, 0x00}, /* NOT SUPPORTED AS DEFAULT */
    }, /* MSM_V4L2_EFFECT_WHITEBOARD */

    {
        {0x3391, 0x00}, /* NOT SUPPORTED AS DEFAULT */
    }, /* MSM_V4L2_EFFECT_BLACKBOARD */

    {
        {0x3391, 0x00}, /* NOT SUPPORTED AS DEFAULT */
    }, /* MSM_V4L2_EFFECT_AQUA */

    {
        {0x3391, 0x00}, /* NOT SUPPORTED AS DEFAULT */
    }, /* MSM_V4L2_EFFECT_EMBOSS */

    {
        {0x3391, 0x00}, /* NOT SUPPORTED AS DEFAULT */
    }, /* MSM_V4L2_EFFECT_SKETCH */

    {
        {0x3391, 0x00}, /* NOT SUPPORTED AS DEFAULT */
    }, /* MSM_V4L2_EFFECT_NEON */

    {
        {0x3391, 0x00}, /* NOT SUPPORTED AS DEFAULT */
    }, /* MSM_V4L2_EFFECT_MAX */

};

static struct msm_camera_i2c_conf_array ov2675_special_effect_confs[][1] = {
	{{ov2675_special_effect[0],  ARRAY_SIZE(ov2675_special_effect[0]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[1],  ARRAY_SIZE(ov2675_special_effect[1]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[2],  ARRAY_SIZE(ov2675_special_effect[2]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[3],  ARRAY_SIZE(ov2675_special_effect[3]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[4],  ARRAY_SIZE(ov2675_special_effect[4]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[5],  ARRAY_SIZE(ov2675_special_effect[5]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[6],  ARRAY_SIZE(ov2675_special_effect[6]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[7],  ARRAY_SIZE(ov2675_special_effect[7]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[8],  ARRAY_SIZE(ov2675_special_effect[8]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[9],  ARRAY_SIZE(ov2675_special_effect[9]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[10], ARRAY_SIZE(ov2675_special_effect[10]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[11], ARRAY_SIZE(ov2675_special_effect[11]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_special_effect[12], ARRAY_SIZE(ov2675_special_effect[12]), 0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
};

static int ov2675_special_effect_enum_map[] = {
	MSM_V4L2_EFFECT_OFF,
	MSM_V4L2_EFFECT_MONO,
	MSM_V4L2_EFFECT_NEGATIVE,
	MSM_V4L2_EFFECT_SOLARIZE,
	MSM_V4L2_EFFECT_SEPIA,
	MSM_V4L2_EFFECT_POSTERAIZE,
	MSM_V4L2_EFFECT_WHITEBOARD,
	MSM_V4L2_EFFECT_BLACKBOARD,
	MSM_V4L2_EFFECT_AQUA,
	MSM_V4L2_EFFECT_EMBOSS,
	MSM_V4L2_EFFECT_SKETCH,
	MSM_V4L2_EFFECT_NEON,
	MSM_V4L2_EFFECT_MAX,
};

static struct msm_camera_i2c_enum_conf_array
		 ov2675_special_effect_enum_confs = {
	.conf = &ov2675_special_effect_confs[0][0],
	.conf_enum = ov2675_special_effect_enum_map,
	.num_enum = ARRAY_SIZE(ov2675_special_effect_enum_map),
	.num_index = ARRAY_SIZE(ov2675_special_effect_confs),
	.num_conf = ARRAY_SIZE(ov2675_special_effect_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};

static struct msm_camera_i2c_reg_conf ov2675_antibanding[][7] = {
    {
        //Band  off  
        //{0x3014, 0x04}, //default value

        //{0x3070, 0xF7}, //default value
        //{0x3071, 0x00}, //default value
        //{0x3072, 0xC6}, //default value
        //{0x3073, 0x00}, //default value
        //{0x301c, 0x04}, //default value
        //{0x301d, 0x05}, //default value
    }, /* ANTIBANDING OFF */
    {
        //Band 60Hz   
        //{0x3014, 0x00},

        {0x3070, 0x5c},
        {0x3071, 0x00},
        {0x3072, 0x4d},
        {0x3073, 0x00},
        {0x301c, 0x06},
        {0x301d, 0x07},
    }, /* ANTIBANDING 60HZ */

    {
        //Band 50Hz   
        //{0x3014, 0x80},
 
        {0x3070, 0x5c},
        {0x3071, 0x00},
        {0x3072, 0x4d},
        {0x3073, 0x00},
        {0x301c, 0x06},
        {0x301d, 0x07},
    }, /* ANTIBANDING 50HZ */

    {
        //Auto-XCLK24MHz                  
        //{0x3014, 0xc0},
    }, /* ANTIBANDING AUTO */
};


static struct msm_camera_i2c_conf_array ov2675_antibanding_confs[][1] = {
	{{ov2675_antibanding[0], ARRAY_SIZE(ov2675_antibanding[0]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_antibanding[1], ARRAY_SIZE(ov2675_antibanding[1]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_antibanding[2], ARRAY_SIZE(ov2675_antibanding[2]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_antibanding[3], ARRAY_SIZE(ov2675_antibanding[3]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
};

static int ov2675_antibanding_enum_map[] = {
    MSM_V4L2_POWER_LINE_OFF,
	MSM_V4L2_POWER_LINE_60HZ,
	MSM_V4L2_POWER_LINE_50HZ,
	MSM_V4L2_POWER_LINE_AUTO,
};


static struct msm_camera_i2c_enum_conf_array ov2675_antibanding_enum_confs = {
	.conf = &ov2675_antibanding_confs[0][0],
	.conf_enum = ov2675_antibanding_enum_map,
	.num_enum = ARRAY_SIZE(ov2675_antibanding_enum_map),
	.num_index = ARRAY_SIZE(ov2675_antibanding_confs),
	.num_conf = ARRAY_SIZE(ov2675_antibanding_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};

static struct msm_camera_i2c_reg_conf ov2675_wb_oem[][4] = {
    {
        {0x3306,0x00},
    }, /* MSM_V4L2_WB_OFF */

    {
        {0x3306,0x00},
    }, /* MSM_V4L2_WB_AUTO */

    {
        {0x3306, 0x02},
        {0x3337, 0x44},
        {0x3338, 0x40},
        {0x3339, 0x70},
    }, /* MSM_V4L2_WB_CUSTOM */

    {
        {0x3306, 0x02},
        {0x3337, 0x52},
        {0x3338, 0x40},
        {0x3339, 0x58},  
    }, /* MSM_V4L2_WB_INCANDESCENT */

    {
        {0x3306,0x00}, /* NOT SUPPORTED AS WB_AUTO MODE */
    }, /* MSM_V4L2_WB_FLUORESCENT */

    {
        {0x3306, 0x02},
        {0x3337, 0x5e},
        {0x3338, 0x40},
        {0x3339, 0x46}, 
    }, /* MSM_V4L2_WB_DAYLIGHT */

    {
        {0x3306, 0x02},
        {0x3337, 0x68},
        {0x3338, 0x40},
        {0x3339, 0x4e},  
    }, /* MSM_V4L2_WB_CLOUDY_DAYLIGHT */

};

static struct msm_camera_i2c_conf_array ov2675_wb_oem_confs[][1] = {
	{{ov2675_wb_oem[0], ARRAY_SIZE(ov2675_wb_oem[0]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_wb_oem[1], ARRAY_SIZE(ov2675_wb_oem[1]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_wb_oem[2], ARRAY_SIZE(ov2675_wb_oem[2]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_wb_oem[3], ARRAY_SIZE(ov2675_wb_oem[3]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_wb_oem[4], ARRAY_SIZE(ov2675_wb_oem[4]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_wb_oem[5], ARRAY_SIZE(ov2675_wb_oem[5]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
	{{ov2675_wb_oem[6], ARRAY_SIZE(ov2675_wb_oem[6]),  0,
		MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA},},
};

static int ov2675_wb_oem_enum_map[] = {
	MSM_V4L2_WB_OFF,
	MSM_V4L2_WB_AUTO ,
	MSM_V4L2_WB_CUSTOM,
	MSM_V4L2_WB_INCANDESCENT,
	MSM_V4L2_WB_FLUORESCENT,
	MSM_V4L2_WB_DAYLIGHT,
	MSM_V4L2_WB_CLOUDY_DAYLIGHT,
};

static struct msm_camera_i2c_enum_conf_array ov2675_wb_oem_enum_confs = {
	.conf = &ov2675_wb_oem_confs[0][0],
	.conf_enum = ov2675_wb_oem_enum_map,
	.num_enum = ARRAY_SIZE(ov2675_wb_oem_enum_map),
	.num_index = ARRAY_SIZE(ov2675_wb_oem_confs),
	.num_conf = ARRAY_SIZE(ov2675_wb_oem_confs[0]),
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
};


int ov2675_saturation_msm_sensor_s_ctrl_by_enum(
		struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
    uint16_t temp;
#ifdef HIKO_DEBUG
    pr_err("[CAM]%s value=%d", __func__, value);
#endif //HIKO_DEBUG
	if (effect_value == CAMERA_EFFECT_OFF) {
        msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3301, &temp, MSM_CAMERA_I2C_BYTE_DATA);
        temp |= 0x80;
        msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3301, temp, MSM_CAMERA_I2C_BYTE_DATA);

        msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
        temp |= 0x02;
        msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
		rc = msm_sensor_write_enum_conf_array(
			s_ctrl->sensor_i2c_client,
			ctrl_info->enum_cfg_settings, value);
	}
	if (value <= MSM_V4L2_SATURATION_L8)
		SAT_U = SAT_V = value * 0x10;
	CDBG("--CAMERA-- %s ...(End)\n", __func__);
	return rc;
}


int ov2675_contrast_msm_sensor_s_ctrl_by_enum(
		struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
    uint16_t temp;
#ifdef HIKO_DEBUG
    pr_err("[CAM]%s value=%d", __func__, value);
#endif //HIKO_DEBUG
    
    msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
    temp |= 0x04;
    msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
	if (effect_value == CAMERA_EFFECT_OFF) {
		rc = msm_sensor_write_enum_conf_array(
			s_ctrl->sensor_i2c_client,
			ctrl_info->enum_cfg_settings, value);
	}
	return rc;
}

int ov2675_sharpness_msm_sensor_s_ctrl_by_enum(
		struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
#ifdef HIKO_DEBUG
    pr_err("[CAM]%s  effect_value :%d, value=%d", __func__, effect_value, value);
#endif //HIKO_DEBUG
	if (effect_value == CAMERA_EFFECT_OFF) {
		rc = msm_sensor_write_enum_conf_array(
			s_ctrl->sensor_i2c_client,
			ctrl_info->enum_cfg_settings, value);
	}
	return rc;
}

int ov2675_exposure_msm_sensor_s_ctrl_by_enum( 
		struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
#ifdef HIKO_DEBUG
    pr_err("[CAM]%s value=%d", __func__, value);
#endif //HIKO_DEBUG

//	if (effect_value == CAMERA_EFFECT_OFF) {
		rc = msm_sensor_write_enum_conf_array(
			s_ctrl->sensor_i2c_client,
			ctrl_info->enum_cfg_settings, value);
//	}

	return rc;
}


int ov2675_effect_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
    uint16_t temp;
#ifdef HIKO_DEBUG
    pr_err("[CAM]%s value=%d", __func__, value);
#endif //HIKO_DEBUG
    return rc;

	effect_value = value;
#if 0
	if (effect_value == CAMERA_EFFECT_OFF) {
#endif //0
        switch (effect_value) {
        case MSM_V4L2_EFFECT_OFF:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_MONO:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_NEGATIVE:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp |= 0x40;
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_SOLARIZE:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_SEPIA:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp |= 0x18;
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3396, 0x40, MSM_CAMERA_I2C_BYTE_DATA);
            
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3397, 0xa6, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_POSTERAIZE:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_WHITEBOARD:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_BLACKBOARD:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_AQUA:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_EMBOSS:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_SKETCH:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_NEON:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        case MSM_V4L2_EFFECT_MAX:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3391, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0x58);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3391, temp, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        };
/*		rc = msm_sensor_write_conf_array(
			s_ctrl->sensor_i2c_client,
			s_ctrl->msm_sensor_reg->no_effect_settings, 0);
*/
		if (rc < 0) {
			CDBG("write faield\n");
			return rc;
		}
//		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0xda, SAT_U,
//			MSM_CAMERA_I2C_BYTE_DATA);
//		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0xdb, SAT_V,
//			MSM_CAMERA_I2C_BYTE_DATA);

#if 0
    } else {
		rc = msm_sensor_write_enum_conf_array(
			s_ctrl->sensor_i2c_client,
			ctrl_info->enum_cfg_settings, value);
	}
#endif //0
	return rc;
}

int ov2675_antibanding_msm_sensor_s_ctrl_by_enum(
		struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
    uint16_t temp=0;

#ifdef HIKO_DEBUG
    pr_err("[CAM]%s value=%d", __func__, value);
#endif //HIKO_DEBUG

    curr_banding_type = value;

    if (curr_banding_type==MSM_V4L2_POWER_LINE_OFF) {
        msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3013, &temp, MSM_CAMERA_I2C_BYTE_DATA);
        temp &= ~(0x20);
        msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3013, temp, MSM_CAMERA_I2C_BYTE_DATA);
        
        msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3014, &temp, MSM_CAMERA_I2C_BYTE_DATA);
        temp &= ~(0x40);
        msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3014, temp, MSM_CAMERA_I2C_BYTE_DATA);
    } else {
        //Banding ON and Allow less than Min Banding
        msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3013, &temp, MSM_CAMERA_I2C_BYTE_DATA);
        temp |= 0x30; //Set the bit to be UP  (0x20=Banding on  0x10=Allow less than Min Banding)
        msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3013, temp, MSM_CAMERA_I2C_BYTE_DATA);
        switch (curr_banding_type) {
        case MSM_V4L2_POWER_LINE_60HZ:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3014, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0xC0);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3014, temp, MSM_CAMERA_I2C_BYTE_DATA);

//Hiko: for adopting Qisda QCS's 50Hz antibanding issue {
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3070, 0x5c, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3071, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3072, 0x4d, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3073, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301c, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301d, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
       //@@ 15FPS
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3071, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3070, 0xb9, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3073, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3072, 0x9b, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef FULL_SIZE_PREVIEW
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301c, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301d, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
#else 
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301c, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301d, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
#endif
//Hiko: for adopting Qisda QCS's 50Hz antibanding issue }
            break;
        case MSM_V4L2_POWER_LINE_50HZ:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3014, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp &= ~(0xC0);
            temp |= 0x80;
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3014, temp, MSM_CAMERA_I2C_BYTE_DATA);
            
//Hiko: for adopting Qisda QCS's 50Hz antibanding issue {
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3070, 0x5c, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3071, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3072, 0x4d, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3073, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301c, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
            //msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301d, 0x07, MSM_CAMERA_I2C_BYTE_DATA);
       //@@ 15FPS
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3071, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3070, 0xb9, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3073, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3072, 0x9b, MSM_CAMERA_I2C_BYTE_DATA);
#ifdef FULL_SIZE_PREVIEW
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301c, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301d, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
#else 
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301c, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301d, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
#endif
//Hiko: for adopting Qisda QCS's 50Hz antibanding issue }
            break;
        case MSM_V4L2_POWER_LINE_AUTO:

            
#ifdef FULL_SIZE_PREVIEW
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301c, 0x05, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301d, 0x06, MSM_CAMERA_I2C_BYTE_DATA);
#else 
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301c, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301d, 0x02, MSM_CAMERA_I2C_BYTE_DATA);
#endif
            
           
            //Hiko: following is the real auto setting procedure
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3014, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            temp |= 0xC0;
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3014, temp, MSM_CAMERA_I2C_BYTE_DATA);

            
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x30af, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3048, 0x1f, MSM_CAMERA_I2C_BYTE_DATA); 
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3049, 0x4e, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x304a, 0x20, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x304f, 0x20, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x304b, 0x02, MSM_CAMERA_I2C_BYTE_DATA); 
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x304c, 0x00, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x304d, 0x02, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x304f, 0x20, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x30a3, 0x10, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3013, 0xf7, MSM_CAMERA_I2C_BYTE_DATA); 
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3014, 0x44, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3070, 0xba, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3071, 0x00, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3072, 0x9a, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3073, 0x00, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x304d, 0x42, MSM_CAMERA_I2C_BYTE_DATA);     
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x304a, 0x40, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x304f, 0x40, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3095, 0x07, MSM_CAMERA_I2C_BYTE_DATA);  
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3096, 0x16, MSM_CAMERA_I2C_BYTE_DATA); 
            msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3097, 0x1d, MSM_CAMERA_I2C_BYTE_DATA);  
            break;
        default:
            break;
        };
    }

#if 0
    rc = msm_sensor_write_enum_conf_array(
            s_ctrl->sensor_i2c_client,
            ctrl_info->enum_cfg_settings, value);
#endif //0
    return rc;
}

int ov2675_wb_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
#ifdef HIKO_DEBUG
    pr_err("[CAM]%s value=%d", __func__, value);
#endif //HIKO_DEBUG
	rc = msm_sensor_write_enum_conf_array(
		s_ctrl->sensor_i2c_client,
		ctrl_info->enum_cfg_settings, value);
	if (rc < 0) {
		CDBG("write faield\n");
		return rc;
	}
    return rc;
}

#ifdef USING_ISO
int ov2675_iso_msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
#ifdef HIKO_DEBUG
    pr_err("[CAM]%s value=%d", __func__, value);
#endif //HIKO_DEBUG
	rc = msm_sensor_write_enum_conf_array(
		s_ctrl->sensor_i2c_client,
		ctrl_info->enum_cfg_settings, value);
	if (rc < 0) {
		CDBG("write faield\n");
		return rc;
	}
    return rc;
}
#endif //USING_ISO


int32_t ov2675_msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	//CDBG("%s: %d\n", __func__, __LINE__);
#ifdef HIKO_DEBUG
    pr_err("[CAM] %s has been called!", __func__);
#endif //HIKO_DEBUG

    msm_sensor_power_up(s_ctrl);        



	return rc;
}


#if 0
int32_t ov2675_msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
    struct msm_cam_clk_info cam_clk_info[] = {
    	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
    };

    CDBG("%s\n", __func__);
#ifdef HIKO_DEBUG
    pr_err("[CAM] %s has been called!", __func__);
#endif //HIKO_DEBUG

#if 0
    //The folowing register should be set before changing the regulator
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x30AB,
                0x00,
                MSM_CAMERA_I2C_BYTE_DATA);
    usleep(50);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x30AD,
                0x0A,
                MSM_CAMERA_I2C_BYTE_DATA);
    usleep(50);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x30AE,
                0x27,
                MSM_CAMERA_I2C_BYTE_DATA);
    usleep(50);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x363B,
                0x01,
                MSM_CAMERA_I2C_BYTE_DATA);
    usleep(50);
#endif //0

#ifdef SEATTLE_PRJ
    //gpio_request(SENSOR_PWDN_GPIO_PIN, "CAM_PWDN");
    gpio_set_value_cansleep(SENSOR_PWDN_GPIO_PIN, 1);
    usleep(1500);
    gpio_free(SENSOR_PWDN_GPIO_PIN);
#endif //SEATTLE_PRJ

    //msm_sensor_power_down(s_ctrl);

	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		msm_sensor_disable_i2c_mux(
			data->sensor_platform_info->i2c_conf);

	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(0);
    
    //HIKO: Letting the MCLK to be the latest thing off!
//	msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
//		cam_clk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 0);

    msm_camera_config_gpio_table(data, 0);
	msm_camera_enable_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
	msm_camera_config_vreg(&s_ctrl->sensor_i2c_client->client->dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
	msm_camera_request_gpio_table(data, 0);
    usleep(1500);

    //HIKO: Letting the MCLK to be the latest thing off!
    msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_clk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 0);

    kfree(s_ctrl->reg_ptr);

    return 0;
}
#endif

#ifdef UXGA_TO_SVGA_TRANSFER_FORMULA_BY_OV
void ov2675_msm_sensor_set_mode_preview(struct msm_sensor_ctrl_t *s_ctrl, int res)
{
    int32_t Preview_dummy_pixel_val;
	int rc = 0;

#ifdef HIKO_DEBUG
    pr_err("[CAM] %s ", __func__);
#endif //HIKO_DEBUG

    /////////////////////////////
    // Back to preview
    /////////////////////////////
    //Start AG/AE
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3013, &reg0x3013,
            MSM_CAMERA_I2C_BYTE_DATA);

    Reg0x3013 = reg0x3013 | 0x05;
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3013,
                Reg0x3013,
                MSM_CAMERA_I2C_BYTE_DATA);
    
    // Initialize OV2650 for preview
    // Different with other sensor, there have default values in dummy pixel, 
    // Default_Reg0x3028, Default_Reg0x3029,dummy lines register, Default_Reg0x302a, 
    // Default_Reg0x302b. And SVGA and UXGA have different default values. So dummy pixel 
    // values and dummy line values are new values minus default values of such registers.

    //SVGA
    if (res == MSM_SENSOR_RES_QTR) { 
        Preview_dummy_pixel_val = Preview_dummy_pixel * 2;
    } else {
        Preview_dummy_pixel_val = Preview_dummy_pixel;
    }
    Reg0x3029 = (Preview_dummy_pixel_val & 0x00ff) + Default_Reg0x3029;
    Reg0x3028 = (Preview_dummy_pixel_val >> 8) + Default_Reg0x3028;

    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3029,
                Reg0x3029,
                MSM_CAMERA_I2C_BYTE_DATA);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3028,
                Reg0x3028,
                MSM_CAMERA_I2C_BYTE_DATA);
    
    // update dummy line
    Reg0x302b = (Preview_dummy_line & 0x00ff) + Default_Reg0x302b;
    Reg0x302a = (Preview_dummy_line >> 8) + Default_Reg0x302a;
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302a,
                Reg0x302a,
                MSM_CAMERA_I2C_BYTE_DATA);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302b,
                Reg0x302b,
                MSM_CAMERA_I2C_BYTE_DATA);

}

void ov2675_msm_sensor_set_mode_snapshot_1(struct msm_sensor_ctrl_t *s_ctrl, int res)
{
	int rc = 0;
	uint16_t capture_sysclk;
	
#ifdef HIKO_DEBUG
    pr_err("[CAM] %s ", __func__);
#endif //HIKO_DEBUG

    ////////////////
    //STOP PREVIEW
    ////////////////
    //Stop AE/AG
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3013, &reg0x3013,
            MSM_CAMERA_I2C_BYTE_DATA);
    Reg0x3013 = reg0x3013 & 0xfa;
    msm_camera_i2c_write(
            s_ctrl->sensor_i2c_client,
            0x3013,
            Reg0x3013,
            MSM_CAMERA_I2C_BYTE_DATA);
    
    //Read back preview shutter
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3002, &reg0x3002,
            MSM_CAMERA_I2C_BYTE_DATA);
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3003, &reg0x3003,
            MSM_CAMERA_I2C_BYTE_DATA);
    Shutter = (reg0x3002<<8) + reg0x3003;
    
    //Read back extra line
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x302d, &reg0x302d,
            MSM_CAMERA_I2C_BYTE_DATA);
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x302e, &reg0x302e,
            MSM_CAMERA_I2C_BYTE_DATA);
    Extra_lines = reg0x302e + (reg0x302d<<8);
    Preview_Exposure = Shutter + Extra_lines;
    
    //Read Back Gain for preview
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3000, &reg0x3000,
            MSM_CAMERA_I2C_BYTE_DATA);
    Preview_Gain16 = (((Reg0x3000 & 0xf0)>>4) + 1) * (16 + (reg0x3000 & 0x0f));
    
    //Read back dummy pixels
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3028, &reg0x3028,
            MSM_CAMERA_I2C_BYTE_DATA);
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3029, &reg0x3029,
            MSM_CAMERA_I2C_BYTE_DATA);
    Preview_dummy_pixel = (((reg0x3028-Default_Reg0x3028) & 0xf0)<<8) + reg0x3029 - Default_Reg0x3029;
    
    if (res == MSM_SENSOR_RES_QTR) {
        Preview_dummy_pixel = Preview_dummy_pixel/2 ;
    }

     #ifdef HIKO_DEBUG
     pr_err("%s, Shutter:0x%x, Extra_lines:0x%x, Preview_Gain16= 0x%x, reg0x3028=0x%x, reg0x3029 =0x%x, Preview_dummy_pixel = 0x%x",
               __func__, Shutter, Extra_lines, Preview_Gain16, reg0x3028, reg0x3029, Preview_dummy_pixel);
     #endif
    /////////////////////////////
    //Calculate Capture Exposure
    /////////////////////////////
    // Capture maximum gain could be defined.
    // Capture_max_gain16 = capture_max_gain * 16
    //
    Preview_line_width = Default_SVGA_line_width + Preview_dummy_pixel;
    //SVGA
    if (res == MSM_SENSOR_RES_QTR) {
        Capture_line_width = Default_SVGA_line_width + Capture_dummy_pixel;
    } else {
        Capture_line_width = Default_UXGA_line_width + Capture_dummy_pixel;
    }
    //SVGA
    if (res == MSM_SENSOR_RES_QTR) {
        Capture_maximum_shutter = Default_SVGA_maximum_shutter + Capture_dummy_line;
    } else {
        Capture_maximum_shutter = Default_UXGA_maximum_shutter + Capture_dummy_line;
    }

    Capture_Exposure = Preview_Exposure * (Capture_PCLK_frequency/Preview_PCLK_frequency) * (Preview_line_width/Capture_line_width);
    //Calculate banding filter value 
    if (curr_banding_type == MSM_V4L2_POWER_LINE_50HZ) {
        //if (format == RGB) {//RGB indicates raw RGB
        //    Capture_banding_filter = Capture_PCLK_frequency /100 /Capture_line_width;
        //} else {
            //YUV422
            Capture_banding_filter = Capture_PCLK_frequency/ 100/ (2*Capture_line_width);
        //}
    } else {
        //if (format == RGB) {
        //    Capture_banding_filter = Capture_PCLK_frequency /120 /Capture_line_width;
        //} else {
            //YUV422
            Capture_banding_filter = Capture_PCLK_frequency /120 /(2*Capture_line_width);
        //}
    }
    #ifdef HIKO_DEBUG
    pr_err("%s,  Capture_Exposure = 0x%x, Preview_Exposure = 0x%x, Preview_line_width= 0x%x, Capture_line_width= 0x%x\n", __func__,
                       Capture_Exposure, Preview_Exposure, Preview_line_width, Capture_line_width);
    #endif
    
    //redistribute gain and exposure
    Gain_Exposure = Preview_Gain16 * Capture_Exposure;

    #ifdef HIKO_DEBUG
    pr_err("%s, Gain_Exposure= 0x%x,  Capture_banding_filter * 16= 0x%x, Capture_maximum_shutter * 16 = 0x%x\n ", 
             __func__, Gain_Exposure, Capture_banding_filter*16, Capture_maximum_shutter*16);
    #endif
    
    if (Gain_Exposure < Capture_banding_filter * 16) {
        // Exposure < 1/100
        Capture_Exposure = Gain_Exposure /16;
        Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2;
    } else {
        if (Gain_Exposure > Capture_maximum_shutter * 16) {
            // Exposure > Capture_maximum_shutter
            Capture_Exposure = Capture_maximum_shutter;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_maximum_shutter/2;
            if (Capture_Gain16 > Capture_max_gain16) {
                // gain reach maximum, insert extra line
                //Capture_Exposure = (Gain_Exposure*1.1)/Capture_max_gain16;
                Capture_Exposure = (Gain_Exposure+(Gain_Exposure/10))/Capture_max_gain16;
                // For 50Hz, Exposure = n/100; For 60Hz, Exposure = n/120
                #if 0 // mark by sophia
                Capture_Exposure = Gain_Exposure/16/Capture_banding_filter; 
                Capture_Exposure = Capture_Exposure * Capture_banding_filter;
                #endif
                Capture_Exposure = Gain_Exposure/16;
                
                Capture_Gain16 = (Gain_Exposure *2+1)/ Capture_Exposure/2;
            } else{
            
              #if 0 // mark by sophia
                Capture_Exposure = Capture_Exposure/Capture_banding_filter;
                Capture_Exposure = Capture_Exposure * Capture_banding_filter;
                #endif
                Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_maximum_shutter/2;
            }
        } else {
            // 1/100(120) < Exposure < Capture_maximum_shutter, Exposure = n/100(120)
            Capture_Exposure = Gain_Exposure/16/Capture_banding_filter;
            Capture_Exposure = Capture_Exposure * Capture_banding_filter;
            Capture_Gain16 = (Gain_Exposure*2 +1) / Capture_Exposure/2;
        }
    }

   #ifdef HIKO_DEBUG
	pr_err("%s, final  Capture_Exposure = 0x%x, Gain_Exposure = 0x%x, Capture_Gain16 = 0x%x\n",
	          __func__, Capture_Exposure, Gain_Exposure, Capture_Gain16);
   #endif

    /////////////////////////////
    // Switch to UXGA
    /////////////////////////////



    
 /*
    /////////////////////////////
    // Write Registers
    /////////////////////////////
    //write dummy pixels
    Reg0x3029 = Capture_dummy_pixel & 0x00ff;
    //reg0x3028 = read_i2c(0x3028);
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3028, &reg0x3028,
            MSM_CAMERA_I2C_BYTE_DATA);
    Reg0x3028 = (reg0x3028 & 0x0f) | ((Capture_dummy_pixel & 0x0f00)>>4);
    //write_i2c(0x3028, reg0x3028);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3028,
                Reg0x3028,
                MSM_CAMERA_I2C_BYTE_DATA);

    //write_i2c(0x302b, reg0x3029);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3029,
                Reg0x3029,
                MSM_CAMERA_I2C_BYTE_DATA);
    
    //Write Dummy Lines
    Reg0x302b = (Capture_dummy_line & 0x00ff) + Default_Reg0x302b;
    Reg0x302a = (Capture_dummy_line >>8) + Default_Reg0x302a;
    //write_i2c(0x302a, Reg0x302a);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302a,
                Reg0x302a,
                MSM_CAMERA_I2C_BYTE_DATA);
    //write_i2c(0x302b, Reg0x302b);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302b,
                Reg0x302b,
                MSM_CAMERA_I2C_BYTE_DATA);
    //Write Exposure
    if (Capture_Exposure > Capture_maximum_shutter) {
        Shutter = Capture_maximum_shutter;
        Extra_lines = Capture_Exposure - Capture_maximum_shutter;
    } else {
        Shutter = Capture_Exposure;
        Extra_lines = 0;
    }
    Reg0x3003 = Shutter & 0x00ff;
    Reg0x3002 = (Shutter >>8) & 0x00ff;
    //write_i2c(0x3003, Reg0x3003);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3003,
                Reg0x3003,
                MSM_CAMERA_I2C_BYTE_DATA);
    //write_i2c(0x3002, Reg0x3002);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3002,
                Reg0x3002,
                MSM_CAMERA_I2C_BYTE_DATA);
    // Write extra line
    Reg0x302e = Extra_lines & 0x00ff;
    Reg0x302d = Extra_lines >> 8;
    //write_i2c(0x302d, Reg0x302d);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302d,
                Reg0x302d,
                MSM_CAMERA_I2C_BYTE_DATA);
    //write_i2c(0x302e, Reg0x302e);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302e,
                Reg0x302e,
                MSM_CAMERA_I2C_BYTE_DATA);
    // Write Gain
    Gain = 0;
    if (Capture_Gain16 > 16) {
        Capture_Gain16 = Capture_Gain /2;
        Gain = 0x10;
    }
    if (Capture_Gain16 > 16) {
        Capture_Gain16 = Capture_Gain /2;
        Gain = Gain | 0x20;
    }
    if (Capture_Gain16 > 16) {
        Capture_Gain16 = Capture_Gain /2;
        Gain = Gain | 0x40;
    }
    if (Capture_Gain16 > 16) {
        Capture_Gain16 = Capture_Gain /2;
        Gain = Gain | 0x80;
    }
    Gain = Gain | (Capture_Gain16 -16);
    //write_i2c(0x3000, Gain);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3000,
                Gain,
                MSM_CAMERA_I2C_BYTE_DATA);

    
    
    
    /////////////////////////////
    // Capture
    /////////////////////////////
    
  */  
    
/*    
    /////////////////////////////
    // Back to preview
    /////////////////////////////
    //Start AG/AE
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3013, &reg0x3013,
            MSM_CAMERA_I2C_BYTE_DATA);

    Reg0x3013 = reg0x3013 | 0x05;
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3013,
                Reg0x3013,
                MSM_CAMERA_I2C_BYTE_DATA);
*/
}


void ov2675_msm_sensor_set_mode_snapshot_2(struct msm_sensor_ctrl_t *s_ctrl, int res)
{
	int rc = 0;
#ifdef HIKO_DEBUG
    pr_err("[CAM] %s ", __func__);
#endif //HIKO_DEBUG
    /////////////////////////////
    // Write Registers
    /////////////////////////////
    //write dummy pixels
    Reg0x3029 = Capture_dummy_pixel & 0x00ff;
    //reg0x3028 = read_i2c(0x3028);
	rc = msm_camera_i2c_read(
            s_ctrl->sensor_i2c_client,
            0x3028, &reg0x3028,
            MSM_CAMERA_I2C_BYTE_DATA);
    Reg0x3028 = (reg0x3028 & 0x0f) | ((Capture_dummy_pixel & 0x0f00)>>4);
    //write_i2c(0x3028, reg0x3028);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3028,
                Reg0x3028,
                MSM_CAMERA_I2C_BYTE_DATA);

    //write_i2c(0x302b, reg0x3029);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3029,
                Reg0x3029,
                MSM_CAMERA_I2C_BYTE_DATA);
    
    //Write Dummy Lines
    Reg0x302b = (Capture_dummy_line & 0x00ff) + Default_Reg0x302b;
    Reg0x302a = (Capture_dummy_line >>8) + Default_Reg0x302a;
    //write_i2c(0x302a, Reg0x302a);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302a,
                Reg0x302a,
                MSM_CAMERA_I2C_BYTE_DATA);
    //write_i2c(0x302b, Reg0x302b);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302b,
                Reg0x302b,
                MSM_CAMERA_I2C_BYTE_DATA);
    //Write Exposure
    if (Capture_Exposure > Capture_maximum_shutter) {
        Shutter = Capture_maximum_shutter;
        Extra_lines = Capture_Exposure - Capture_maximum_shutter;
    } else {
        Shutter = Capture_Exposure;
        Extra_lines = 0;
    }
    Reg0x3003 = Shutter & 0x00ff;
    Reg0x3002 = (Shutter >>8) & 0x00ff;
    //write_i2c(0x3003, Reg0x3003);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3003,
                Reg0x3003,
                MSM_CAMERA_I2C_BYTE_DATA);
    //write_i2c(0x3002, Reg0x3002);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3002,
                Reg0x3002,
                MSM_CAMERA_I2C_BYTE_DATA);
    // Write extra line
    Reg0x302e = Extra_lines & 0x00ff;
    Reg0x302d = Extra_lines >> 8;
    //write_i2c(0x302d, Reg0x302d);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302d,
                Reg0x302d,
                MSM_CAMERA_I2C_BYTE_DATA);
    //write_i2c(0x302e, Reg0x302e);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x302e,
                Reg0x302e,
                MSM_CAMERA_I2C_BYTE_DATA);
    // Write Gain
    Gain = 0;
    if (Capture_Gain16 > 16) {
        //Capture_Gain16 = Capture_Gain /2;
        Capture_Gain16 = Capture_Gain16 /2;
        Gain = 0x10;
    }
    if (Capture_Gain16 > 16) {
        //Capture_Gain16 = Capture_Gain /2;
        Capture_Gain16 = Capture_Gain16 /2;
        Gain = Gain | 0x20;
    }
    if (Capture_Gain16 > 16) {
        //Capture_Gain16 = Capture_Gain /2;
        Capture_Gain16 = Capture_Gain16 /2;
        Gain = Gain | 0x40;
    }
    if (Capture_Gain16 > 16) {
        //Capture_Gain16 = Capture_Gain /2;
        Capture_Gain16 = Capture_Gain16 /2;
        Gain = Gain | 0x80;
    }
    Gain = Gain | (Capture_Gain16 -16);
    //write_i2c(0x3000, Gain);
    msm_camera_i2c_write(
                s_ctrl->sensor_i2c_client,
                0x3000,
                Gain,
                MSM_CAMERA_I2C_BYTE_DATA);
}
#endif //UXGA_TO_SVGA_TRANSFER_FORMULA_BY_OV




static unsigned int OV2675_get_shutter(struct msm_sensor_ctrl_t *s_ctrl)
{
	// read shutter, in number of line period
	uint16_t shutter = 0, extra_line = 0;
	uint16_t ret_l,ret_h;

	ret_l = ret_h = 0;
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3002, &ret_h, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3003, &ret_l, MSM_CAMERA_I2C_BYTE_DATA);

	shutter = (ret_h << 8) | (ret_l & 0xff) ;
	ret_l = ret_h = 0;
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x302d, &ret_h, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x302e, &ret_l, MSM_CAMERA_I2C_BYTE_DATA);
	extra_line = (ret_h << 8) | (ret_l & 0xff) ;

	return shutter + extra_line;
}

static int OV2675_set_shutter(struct msm_sensor_ctrl_t * s_ctrl,unsigned int shutter)
{
	// write shutter, in number of line period
	int rc = 0;
	uint16_t temp;

	shutter = shutter & 0xffff;
	temp = shutter & 0xff;
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3003, temp, MSM_CAMERA_I2C_BYTE_DATA);
	temp = shutter >> 8;
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3002, temp, MSM_CAMERA_I2C_BYTE_DATA);

	return rc;
}

static unsigned int OV2675_get_gain16(struct msm_sensor_ctrl_t * s_ctrl)
{
	uint16_t gain16, temp;

	temp = 0;
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3000, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("%s:Reg(0x3000) = 0x%x\n",__func__,temp);
	gain16 = (((temp & 0xf0)>>4) + 1) * (16 + (temp & 0x0f));  //liujia 20121019

	return gain16;
}

static int OV2675_set_gain16(struct msm_sensor_ctrl_t * s_ctrl,unsigned int gain16)
{
	int rc = 0;
	uint16_t reg;

	gain16 = gain16 & 0x1ff;	// max gain is 32x
	reg = 0;

	if (gain16 > 32){
		gain16 = gain16 /2;
		reg = 0x10;
	}

	if (gain16 > 32){
		gain16 = gain16 /2;
		reg = reg | 0x20;
	}

	if (gain16 > 32){
		gain16 = gain16 /2;
		reg = reg | 0x40;
	}

	if (gain16 > 32){
		gain16 = gain16 /2;
		reg = reg | 0x80;
	}

	reg = reg | (gain16 -16);
	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client,0x3000,reg + 1, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(100);
	rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client,0x3000,reg, MSM_CAMERA_I2C_BYTE_DATA);

	return rc;
}

static int ov2675_set_nightmode(struct msm_sensor_ctrl_t * s_ctrl,int NightMode)

{
	int rc = 0;
	uint16_t temp;

	switch (NightMode) {
		case 0:{//Off
			msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3014, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp & 0xf7;			// night mode off, bit[3] = 0
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3014, temp, MSM_CAMERA_I2C_BYTE_DATA);
			// clear dummy lines
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x302d, 0, MSM_CAMERA_I2C_BYTE_DATA);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x302e, 0, MSM_CAMERA_I2C_BYTE_DATA);
		}
		break;
	case 1: {// On
			msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3014, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp | 0x08;			// night mode on, bit[3] = 1
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3014, temp, MSM_CAMERA_I2C_BYTE_DATA);
		}
		break;
	default:
		break;
	}

	return rc;
}


static unsigned int OV2675_get_sysclk(struct msm_sensor_ctrl_t * s_ctrl)
{
	// calculate sysclk
	uint16_t temp1, temp2, XVCLK;
	uint16_t Indiv2x, Bit8Div, FreqDiv2x, PllDiv, SensorDiv, ScaleDiv, DvpDiv, ClkDiv, VCO, sysclk;
	uint16_t Indiv2x_map[] = { 2, 3, 4, 6, 4, 6, 8, 12};
	uint16_t Bit8Div_map[] = { 1, 1, 4, 5};
	uint16_t FreqDiv2x_map[] = { 2, 3, 4, 6};
	uint16_t DvpDiv_map[] = { 1, 2, 8, 16};

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x300e, &temp1, MSM_CAMERA_I2C_BYTE_DATA);
	// bit[5:0] PllDiv
	PllDiv = 64 - (temp1 & 0x3f);
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x300f, &temp1, MSM_CAMERA_I2C_BYTE_DATA);
	// bit[2:0] Indiv
	temp2 = temp1 & 0x07;
	Indiv2x = Indiv2x_map[temp2];
	// bit[5:4] Bit8Div
	temp2 = (temp1 >> 4) & 0x03;
	Bit8Div = Bit8Div_map[temp2];
	// bit[7:6] FreqDiv
	temp2 = temp1 >> 6;
	FreqDiv2x = FreqDiv2x_map[temp2];
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3010, &temp1, MSM_CAMERA_I2C_BYTE_DATA);
	//bit[3:0] ScaleDiv
	temp2 = temp1 & 0x0f;

	if(temp2==0) {
		ScaleDiv = 1;
	} else {
		ScaleDiv = temp2 * 2;
	}

	// bit[4] SensorDiv
	if(temp1 & 0x10) {
		SensorDiv = 2;
	} else {
		SensorDiv = 1;
	}

	// bit[5] LaneDiv
	// bit[7:6] DvpDiv
	temp2 = temp1 >> 6;
	DvpDiv = DvpDiv_map[temp2];
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3011, &temp1, MSM_CAMERA_I2C_BYTE_DATA);
	// bit[5:0] ClkDiv
	temp2 = temp1 & 0x3f;
	ClkDiv = temp2 + 1;
	XVCLK = 24000000/10000;

	CDBG("%s:XVCLK = 0x%x\n",__func__,XVCLK);
	CDBG("%s:Bit8Div = 0x%x\n",__func__,Bit8Div);
	CDBG("%s:FreqDiv2x = 0x%x\n",__func__,FreqDiv2x);
	CDBG("%s:PllDiv = 0x%x\n",__func__,PllDiv);
	CDBG("%s:Indiv2x = 0x%x\n",__func__,Indiv2x);

	VCO = XVCLK * Bit8Div * FreqDiv2x * PllDiv / Indiv2x ;
	sysclk = VCO / Bit8Div / SensorDiv / ClkDiv / 4;

	CDBG("%s:ClkDiv = 0x%x\n",__func__,ClkDiv);
	CDBG("%s:SensorDiv = 0x%x\n",__func__,SensorDiv);
	CDBG("%s:sysclk = 0x%x\n",__func__,sysclk);

	return sysclk;
}

static unsigned int OV2675_get_HTS(struct msm_sensor_ctrl_t * s_ctrl)
{
	// read HTS from register settings
	uint16_t HTS, extra_HTS;
	uint16_t ret_l,ret_h;

	ret_l = ret_h = 0;
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3028, &ret_h, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3029, &ret_l, MSM_CAMERA_I2C_BYTE_DATA);

	HTS = (ret_h << 8) | (ret_l & 0xff) ;
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x302c, &ret_l, MSM_CAMERA_I2C_BYTE_DATA);
	extra_HTS = ret_l;

	return HTS + extra_HTS;
}

static unsigned int OV2675_get_VTS(struct msm_sensor_ctrl_t * s_ctrl)
{
	// read VTS from register settings
	uint16_t VTS, extra_VTS;
	uint16_t ret_l,ret_h;

	ret_l = ret_h = 0;
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x302a, &ret_h, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x302b, &ret_l, MSM_CAMERA_I2C_BYTE_DATA);

	VTS = (ret_h << 8) | (ret_l & 0xff) ;
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x302d, &ret_h, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x302e, &ret_l, MSM_CAMERA_I2C_BYTE_DATA);
	extra_VTS = (ret_h << 8) | (ret_l & 0xff) ;

	return VTS + extra_VTS;
}

static int OV2675_set_VTS(struct msm_sensor_ctrl_t * s_ctrl,unsigned int VTS)
{
	// write VTS to registers
	int rc = 0;
	uint16_t temp;

	temp = VTS & 0xff;
	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x302b, temp, MSM_CAMERA_I2C_BYTE_DATA);

	temp = VTS>>8;
	rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x302a, temp, MSM_CAMERA_I2C_BYTE_DATA);

	return rc;
}
static unsigned int OV2675_get_light_frequency(struct msm_sensor_ctrl_t * s_ctrl)
{
	// get banding filter value
	uint16_t temp, light_frequency;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3014, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	if (temp & 0x40) {
		// auto
		msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x508e, &temp, MSM_CAMERA_I2C_BYTE_DATA);

		if (temp & 0x01){
			light_frequency = 50;
		} else {
			light_frequency = 60;
		}
	} else {
		// manual
		if (temp & 0x80){
			// 50Hz
			light_frequency = 50;
		} else {
			// 60Hz
			light_frequency = 60;
		}
	}

	return light_frequency;
}


static int OV2675_set_bandingfilter(struct msm_sensor_ctrl_t * s_ctrl)
{
	int rc = 0;
	uint16_t preview_VTS;
	uint16_t band_step60, max_band60, band_step50, max_band50;

	// read preview PCLK
	ov2675_preview_sysclk = OV2675_get_sysclk(s_ctrl);
	// read preview HTS
	ov2675_preview_HTS = OV2675_get_HTS(s_ctrl);
	// read preview VTS
	preview_VTS = OV2675_get_VTS(s_ctrl);

	// calculate banding filter
	CDBG("%s:ov2675_preview_sysclk = 0x%x\n",__func__,ov2675_preview_sysclk);
	CDBG("%s:ov2675_preview_HTS = 0x%x\n",__func__,ov2675_preview_HTS);
	CDBG("%s:preview_VTS = 0x%x\n",__func__,preview_VTS);

	// 60Hz
	band_step60 = ov2675_preview_sysclk * 100/ov2675_preview_HTS * 100/120;
	rc = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3073, (band_step60 >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3072, (band_step60 & 0xff), MSM_CAMERA_I2C_BYTE_DATA);
	max_band60 = ((preview_VTS-4)/band_step60);
	rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301d, max_band60-1, MSM_CAMERA_I2C_BYTE_DATA);


	CDBG("%s:band_step60 = 0x%x\n",__func__,band_step60);
	CDBG("%s:max_band60 = 0x%x\n",__func__,max_band60);

	// 50Hz
	band_step50 = ov2675_preview_sysclk * 100/ov2675_preview_HTS;
	rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3071, (band_step50 >> 8), MSM_CAMERA_I2C_BYTE_DATA);
	rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3070, (band_step50 & 0xff), MSM_CAMERA_I2C_BYTE_DATA);
	max_band50 = ((preview_VTS-4)/band_step50);
	rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x301c, max_band50-1, MSM_CAMERA_I2C_BYTE_DATA);

	CDBG("%s:band_step50 = 0x%x\n",__func__,band_step50 );
	CDBG("%s:max_band50 = 0x%x\n",__func__,max_band50);

	return rc;
}

static int ov2675_get_preview_exposure_gain(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	ov2675_preview_shutter = OV2675_get_shutter(s_ctrl);
	// read preview gain
	ov2675_preview_gain16 = OV2675_get_gain16(s_ctrl);
	ov2675_preview_binning = 1;
	// turn off night mode for capture
	rc = ov2675_set_nightmode(s_ctrl,0);

	return rc;
}

static int ov2675_set_preview_exposure_gain(struct msm_sensor_ctrl_t * s_ctrl)
{
	int rc = 0;

	rc = OV2675_set_shutter(s_ctrl,ov2675_preview_shutter);
	rc = OV2675_set_gain16(s_ctrl,ov2675_preview_gain16);

#if 0 //sophia
	if(OV2675_CAMERA_WB_AUTO)
	{
		rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3306, 0x00, MSM_CAMERA_I2C_BYTE_DATA); //set to WB_AUTO
	}
#endif
	CDBG("ov2675_set_preview_exposure_shutter %d\n", ov2675_preview_shutter);
	CDBG("ov2675_set_preview_exposure_gain %d\n", ov2675_preview_gain16);

	return rc;
}

static int ov2675_set_capture_exposure_gain(struct msm_sensor_ctrl_t * s_ctrl)
{
	int rc = 0;
	uint16_t capture_shutter, capture_gain16, capture_sysclk, capture_HTS, capture_VTS;
	uint16_t light_frequency, capture_bandingfilter, capture_max_band;
	uint16_t capture_gain16_shutter;
	uint16_t temp;

       return 0;
	//Step3: calculate and set capture exposure and gain
	// turn off AEC, AGC
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3013, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	temp = temp & 0xfa;
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3013, temp, MSM_CAMERA_I2C_BYTE_DATA);
	// read capture sysclk
	capture_sysclk = OV2675_get_sysclk(s_ctrl);
	// read capture HTS
	capture_HTS = OV2675_get_HTS(s_ctrl);
	// read capture VTS
	capture_VTS = OV2675_get_VTS(s_ctrl);
	// calculate capture banding filter
	light_frequency = OV2675_get_light_frequency(s_ctrl);

	if (light_frequency == 60) {
		// 60Hz
		capture_bandingfilter = capture_sysclk * 100 / capture_HTS * 100 / 120;
	} else {
		// 50Hz
		capture_bandingfilter = capture_sysclk * 100 / capture_HTS;
	}

	capture_max_band = ((capture_VTS-4)/capture_bandingfilter);
	// calculate capture shutter
	capture_shutter = ov2675_preview_shutter;
	// gain to shutter
	capture_gain16 = ov2675_preview_gain16 * capture_sysclk/ov2675_preview_sysclk
			* ov2675_preview_HTS/capture_HTS * ov2675_preview_binning;

	if (capture_gain16 < 16) {
		capture_gain16 = 16;
	}
	capture_gain16_shutter = capture_gain16 * capture_shutter;

	if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
		// shutter < 1/100
		capture_shutter = capture_gain16_shutter/16;
		capture_gain16 = capture_gain16_shutter/capture_shutter;
	} else {
		if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
			// exposure reach max
			capture_shutter = capture_bandingfilter*capture_max_band;
			capture_gain16 = capture_gain16_shutter / capture_shutter;
		} else {
			// 1/100 < capture_shutter < max, capture_shutter = n/100
			capture_shutter = (capture_gain16_shutter/16/capture_bandingfilter)
					* capture_bandingfilter;
			capture_gain16 = capture_gain16_shutter / capture_shutter;
		}
	}

	// write capture gain
	rc |= OV2675_set_gain16(s_ctrl,capture_gain16);
	// write capture shutter
	if (capture_shutter > (capture_VTS - 4)) {
		capture_VTS = capture_shutter + 4;
		rc |= OV2675_set_VTS(s_ctrl,capture_VTS);
	}
	rc |= OV2675_set_shutter(s_ctrl,capture_shutter);


	//if(OV2675_CAMERA_WB_AUTO){
	//	rc |= ov2675_i2c_write_byte(ov2675_client->addr, 0x3306, 0x02);
	//	rc |= ov2675_i2c_write_byte(ov2675_client->addr, 0x3337, OV2675_preview_R_gain);
	//	rc |= ov2675_i2c_write_byte(ov2675_client->addr, 0x3338, OV2675_preview_G_gain);
	//	rc |= ov2675_i2c_write_byte(ov2675_client->addr, 0x3339, OV2675_preview_B_gain);
	//}
	//CDBG("ov2675_set_capture_shutter_gain %d\n", capture_shutter);
	//CDBG("ov2675_set_capture_exposure_gain %d\n", capture_gain16);

	return rc;
}


void ov2675_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	if (s_ctrl->curr_res >= s_ctrl->msm_sensor_reg->num_conf)
		return;

	if (s_ctrl->func_tbl->sensor_adjust_frame_lines)
		s_ctrl->func_tbl->sensor_adjust_frame_lines(s_ctrl);

	msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->start_stream_conf,
		s_ctrl->msm_sensor_reg->start_stream_conf_size,
		s_ctrl->msm_sensor_reg->default_data_type);


#if 1
       if(s_ctrl->curr_res == MSM_SENSOR_RES_QTR)
       ov2675_set_preview_exposure_gain(s_ctrl);
  #endif
	//Sophia Wang++

  msleep(300);

//	msleep(1000); // sophia, current , we need to delay almost one seconds, while back to preview, there is no obvious black screen.
	//Sophia Wang--
}

void ov2675_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
#ifndef FULL_SIZE_PREVIEW
	msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->stop_stream_conf,
		s_ctrl->msm_sensor_reg->stop_stream_conf_size,
		s_ctrl->msm_sensor_reg->default_data_type);
	msleep(66);
#endif
	//msm_sensor_delay_frames(s_ctrl);
}

// 20130722, sophia ++
int32_t ov2675_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;
	uint16_t temp = 0;

       pr_err("%s, update_type:%d, res:%d\n", __func__, update_type, res);
	if (update_type == MSM_SENSOR_REG_INIT) {
		CDBG("Register INIT\n");
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
		msm_sensor_enable_debugfs(s_ctrl);
              /* apply different tuning parameters for different lens*/
		if( (ov2675_fuse_id > 0 && ov2675_lens_id == 1) ||
		     (ov2675_fuse_id == 0 && ov2675_lens_id == 0)){
              		msm_sensor_write_all_conf_array(
              			s_ctrl->sensor_i2c_client,
              			&ov2675_init_lt1205_conf[0],
              			ARRAY_SIZE(ov2675_init_lt1205_conf));
              		pr_info("%s, V2.0 sensor setting\n", __func__);
              		
		} else if (ov2675_fuse_id > 0 && ov2675_lens_id == 0){	
			msm_sensor_write_init_settings(s_ctrl);
			pr_info("%s, V1.0 sensor setting\n", __func__);
			
	       } else {
			msm_sensor_write_init_settings(s_ctrl);
			pr_info("%s, default sensor setting\n", __func__);
	       }

		ov2675_preview_shutter = OV2675_get_shutter(s_ctrl);
		ov2675_preview_gain16 = OV2675_get_gain16(s_ctrl);
              
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		CDBG("PERIODIC : %d\n", res);

		msm_sensor_write_res_settings(s_ctrl, res);

		if( (ov2675_fuse_id > 0 && ov2675_lens_id == 1) ||
			(ov2675_fuse_id == 0 && ov2675_lens_id  == 0))
				msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 
					0x3376, 
					0x9, 
					MSM_CAMERA_I2C_BYTE_DATA);
		else if (ov2675_fuse_id > 0 && ov2675_lens_id == 0)
				msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 
					0x3376, 
					0x5, 
					MSM_CAMERA_I2C_BYTE_DATA);
		else 
				msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 
					0x3376, 
					0x5, 
					MSM_CAMERA_I2C_BYTE_DATA);

	if (res == MSM_SENSOR_RES_QTR)
	{
		msm_sensor_write_conf_array(
				s_ctrl->sensor_i2c_client,
				s_ctrl->msm_sensor_reg->mode_settings, res);
		rc |= OV2675_set_bandingfilter(s_ctrl);

		// turn on AEC, AGC
		rc |= msm_camera_i2c_read(s_ctrl->sensor_i2c_client,0x3013, &temp, MSM_CAMERA_I2C_BYTE_DATA);
		temp = temp | 0x05;
		rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3013, temp, MSM_CAMERA_I2C_BYTE_DATA);

		ov2675_set_preview_exposure_gain(s_ctrl);
//		msleep(1000);
		CDBG("%s, ov2675_preview_tbl_30fps is set\n",__func__);

	}
	else if(res ==MSM_SENSOR_RES_FULL)
	{
			ov2675_get_preview_exposure_gain(s_ctrl);
			msm_sensor_write_conf_array(
				s_ctrl->sensor_i2c_client,
				s_ctrl->msm_sensor_reg->mode_settings, res);
	//		msleep(300);
			CDBG("%s, ov2675_capture_tbl is set\n",__func__);
	
	/* if full size is used for preview, need to set the banding filter and cannot turn off AEC, AGC
	     0x3013 register
	*/
	#ifdef FULL_SIZE_PREVIEW
			rc |= OV2675_set_bandingfilter(s_ctrl);
			/* delay  two frames to wait stable frame*/
			msleep(130);
	#else
			rc |= msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3013, 0xf0, MSM_CAMERA_I2C_BYTE_DATA);
	#endif
			ov2675_set_capture_exposure_gain(s_ctrl);
	//		msleep(1000);

	}

		msleep(30);
	
      		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);

	}
	return rc;
}
// 20130722, sophia --



int32_t ov2675_msm_sensor_set_sensor_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int mode, int res)
{
	int32_t rc = 0;

#ifdef UXGA_TO_SVGA_TRANSFER_FORMULA_BY_OV
#if !defined(HIKO_DEBUG_800_600) && !defined(HIKO_DEBUG_1600_1200)
    switch(mode)
    {
        case SENSOR_MODE_SNAPSHOT:
        case SENSOR_MODE_RAW_SNAPSHOT:
            ov2675_msm_sensor_set_mode_snapshot_1(s_ctrl, res);
            break;
        case SENSOR_MODE_PREVIEW:
            ov2675_msm_sensor_set_mode_preview(s_ctrl, res);
            break;
    };
#endif //!defined(HIKO_DEBUG_800_600) && !defined(HIKO_DEBUG_1600_1200)
#endif //UXGA_TO_SVGA_TRANSFER_FORMULA_BY_OV

	if (s_ctrl->curr_res != res) {
		s_ctrl->curr_frame_length_lines =
			s_ctrl->msm_sensor_reg->
			output_settings[res].frame_length_lines;

		s_ctrl->curr_line_length_pclk =
			s_ctrl->msm_sensor_reg->
			output_settings[res].line_length_pclk;

		if (s_ctrl->is_csic ||
			!s_ctrl->sensordata->csi_if)
			rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl,
				MSM_SENSOR_UPDATE_PERIODIC, res);
		else
			rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
				MSM_SENSOR_UPDATE_PERIODIC, res);
		if (rc < 0)
			return rc;
		s_ctrl->curr_res = res;
	}

#ifdef UXGA_TO_SVGA_TRANSFER_FORMULA_BY_OV
#if !defined(HIKO_DEBUG_800_600) && !defined(HIKO_DEBUG_1600_1200)
    switch(mode)
    {
        case SENSOR_MODE_SNAPSHOT:
        case SENSOR_MODE_RAW_SNAPSHOT:
            ov2675_msm_sensor_set_mode_snapshot_2(s_ctrl, res);
            break;
    };
#endif //!defined(HIKO_DEBUG_800_600) && !defined(HIKO_DEBUG_1600_1200)
#endif //UXGA_TO_SVGA_TRANSFER_FORMULA_BY_OV

#ifdef HIKO_DEBUG
{
    uint16_t temp=0;
    switch(mode)
    {
        case SENSOR_MODE_SNAPSHOT:
        case SENSOR_MODE_RAW_SNAPSHOT:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3013, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3013 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3014, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3014 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3002, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3002 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3003, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3003 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3000, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3000 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3001, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3001 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3070, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3070 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3071, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3071 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3072, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3072 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3073, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3073 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x301c, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x301c = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x301d, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x301d = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3088, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3088 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3089, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x3089 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x308a, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x308a = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x308b, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]snapshot dump reg 0x308b = 0x%X", temp);
            break;
        case SENSOR_MODE_PREVIEW:
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3013, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3013 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3014, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3014 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3002, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3002 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3003, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3003 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3000, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3000 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3001, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3001 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3070, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3070 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3071, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3071 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3072, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3072 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3073, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3073 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x301c, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x301c = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x301d, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x301d = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3088, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3088 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x3089, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x3089 = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x308a, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x308a = 0x%X", temp);
            msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x308b, &temp, MSM_CAMERA_I2C_BYTE_DATA);
            pr_err("[CAM]preview dump reg 0x308b = 0x%X", temp);
            break;
    };
}
#endif //HIKO_DEBUG

//ov2675_exposure_msm_sensor_s_ctrl_by_enum(s_ctrl, &ov2675_s_ctrl.msm_sensor_v4l2_ctrl_info[3], 2);

	return rc;
}


uint8_t ov2675_write_otp(struct msm_sensor_ctrl_t *s_ctrl, unsigned long long input)
{
	uint8_t data[6];
       uint8_t rc1, rc2, rc3, rc4, rc5, rc6, rc7;
       int i = 0;

       memset( data, 0, sizeof(data));
       
       do
    	{
	 	data[i] = do_div(input, 100);

	 	CDBG("data[%d]:%d\n",i, data[i]);
	 	i++;
     	} while( i<= 5); // there should just be 12 number

       rc1 = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x30e6,data[5], MSM_CAMERA_I2C_BYTE_DATA);

	rc2 = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x30e7,data[4], MSM_CAMERA_I2C_BYTE_DATA);

       rc3 = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x30e8, data[3], MSM_CAMERA_I2C_BYTE_DATA);

       rc4 = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x30e9, data[2], MSM_CAMERA_I2C_BYTE_DATA);

	rc5 = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x30ea, data[1], MSM_CAMERA_I2C_BYTE_DATA);

       rc6 = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x30eb, data[0], MSM_CAMERA_I2C_BYTE_DATA);

       rc7 =msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x308f, 0xaa, MSM_CAMERA_I2C_BYTE_DATA);
       if(rc1 < 0 || rc2 < 0 || rc3 < 0 || rc4 < 0 || rc5 <0 || rc6<0 || rc7<0)
       {
		pr_err("%s, write OTP failed\n", __func__);
		return -1;
       }

       return 0;


}

signed long long ov2675_read_fuse_id( struct msm_sensor_ctrl_t *s_ctrl)
{
       int32_t rc1 = 0, rc2 = 0, rc3 = 0, rc4 = 0, rc5 =0, rc6 =0, rc7 =0;
	uint16_t id1 = 0, id2 = 0, id3 = 0, id4 = 0, id5=0, id6 =0 ;
	char buf[14];

	rc1 = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x308f, 0x55, MSM_CAMERA_I2C_BYTE_DATA);
	rc2 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x30e6, &id1, MSM_CAMERA_I2C_BYTE_DATA);
	rc3 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x30e7, &id2, MSM_CAMERA_I2C_BYTE_DATA);
	rc4 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x30e8, &id3, MSM_CAMERA_I2C_BYTE_DATA);
	rc5 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x30e9, &id4, MSM_CAMERA_I2C_BYTE_DATA);
	rc6 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x30ea, &id5, MSM_CAMERA_I2C_BYTE_DATA);
	rc7 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x30eb, &id6, MSM_CAMERA_I2C_BYTE_DATA);


	if(rc1 <0 || rc2<0 || rc3<0 || rc4<0 ||rc5<0 || rc6<0 || rc7 <0)
	{
               pr_err("%s, read if failed\n", __func__);
               return -1; 
	}

	snprintf(buf,14, "%02d%02d%02d%02d%02d%02d\n", id1, id2, id3, id4, id5, id6);

	pr_err("%s, buf is %s\n", __func__, buf);

	ov2675_fuse_id = simple_strtoull(buf, NULL, 10); 
	return simple_strtoull(buf, NULL, 10);

}

int16_t ov2675_read_lens_id( struct msm_sensor_ctrl_t* s_ctrl)
{
	int32_t rc1 = 0, rc2 = 0;
	uint16_t lens_id;

	rc1 = msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x308f, 0x55, MSM_CAMERA_I2C_BYTE_DATA);
	rc2 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x30ec, &lens_id, MSM_CAMERA_I2C_BYTE_DATA);

	if(rc1 < 0 || rc2 < 0){
		pr_err("%s, read lens id failed\n", __func__);
		return -1;
	}

       ov2675_lens_id = lens_id;
       
	return lens_id; 
}

struct msm_sensor_v4l2_ctrl_info_t ov2675_v4l2_ctrl_info[] = {
	{
		.ctrl_id = V4L2_CID_SATURATION,
		.min = MSM_V4L2_SATURATION_L0,
		.max = MSM_V4L2_SATURATION_L8,
		.step = 1,
		.enum_cfg_settings = &ov2675_saturation_enum_confs,
		.s_v4l2_ctrl = ov2675_saturation_msm_sensor_s_ctrl_by_enum,
	},
	{
		.ctrl_id = V4L2_CID_CONTRAST,
		.min = MSM_V4L2_CONTRAST_L0,
		.max = MSM_V4L2_CONTRAST_L8,
		.step = 1,
		.enum_cfg_settings = &ov2675_contrast_enum_confs,
		.s_v4l2_ctrl = ov2675_contrast_msm_sensor_s_ctrl_by_enum,
	},
	{
		.ctrl_id = V4L2_CID_SHARPNESS,
		.min = MSM_V4L2_SHARPNESS_L0,
		.max = MSM_V4L2_SHARPNESS_L5,
		.step = 1,
		.enum_cfg_settings = &ov2675_sharpness_enum_confs,
		.s_v4l2_ctrl = ov2675_sharpness_msm_sensor_s_ctrl_by_enum,
	},
	{
		.ctrl_id = V4L2_CID_EXPOSURE,
		.min = MSM_V4L2_EXPOSURE_N2,
		.max = MSM_V4L2_EXPOSURE_P2,
		.step = 1,
		.enum_cfg_settings = &ov2675_exposure_enum_confs,
		.s_v4l2_ctrl = ov2675_exposure_msm_sensor_s_ctrl_by_enum,
	},
#ifdef USING_ISO
	{
		.ctrl_id = MSM_V4L2_PID_ISO,
		.min = MSM_V4L2_ISO_AUTO,
		.max = MSM_V4L2_ISO_1600,
		.step = 1,
		.enum_cfg_settings = &ov2675_iso_enum_confs,
		.s_v4l2_ctrl = ov2675_iso_msm_sensor_s_ctrl_by_enum,
	},
#endif //USING_ISO
	{
		.ctrl_id = V4L2_CID_SPECIAL_EFFECT,
		.min = MSM_V4L2_EFFECT_OFF,
		.max = MSM_V4L2_EFFECT_NEGATIVE,
		.step = 1,
		.enum_cfg_settings = &ov2675_special_effect_enum_confs,
		.s_v4l2_ctrl = ov2675_effect_msm_sensor_s_ctrl_by_enum,
	},
	{
		.ctrl_id = V4L2_CID_POWER_LINE_FREQUENCY,
		.min = MSM_V4L2_POWER_LINE_60HZ,
		.max = MSM_V4L2_POWER_LINE_AUTO,
		.step = 1,
		.enum_cfg_settings = &ov2675_antibanding_enum_confs,
		.s_v4l2_ctrl = ov2675_antibanding_msm_sensor_s_ctrl_by_enum,
	},
	{
		.ctrl_id = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
		.min = MSM_V4L2_WB_OFF,
		.max = MSM_V4L2_WB_CLOUDY_DAYLIGHT,
		.step = 1,
		.enum_cfg_settings = &ov2675_wb_oem_enum_confs,
		.s_v4l2_ctrl = ov2675_wb_msm_sensor_s_ctrl_by_enum,
	},

};

#if 0
#if 0
static struct msm_camera_csi_params ov2675_csi_params =
{
    .data_format = CSI_8BIT,
    .lane_cnt    = 1,
    .lane_assign = 0xe4,
    .dpcm_scheme = 0,
    .settle_cnt  = 0x12,//0x14,
};
static struct msm_camera_csi_params *ov2675_csi_params_array[] =
{
    &ov2675_csi_params, 
};
#else
static struct msm_camera_csid_vc_cfg ov2675_cid_cfg[] =
{
	{ 0, CSI_YUV422_8, CSI_DECODE_8BIT },
	{ 1, CSI_EMBED_DATA, CSI_DECODE_8BIT },
};

static struct msm_camera_csi2_params ov2675_csi_params =
{
	.csid_params =
	{
		.lane_cnt = 1,
		.lut_params =
		{
			.num_cid = ARRAY_SIZE(ov2675_cid_cfg),
			.vc_cfg = ov2675_cid_cfg,
		},
	},
	.csiphy_params =
	{
		.lane_cnt = 1,
		.settle_cnt = 0x12,
	},
};
static struct msm_camera_csi2_params *ov2675_csi_params_array[] =
{
	&ov2675_csi_params,
	&ov2675_csi_params,
};
#endif
#endif

static struct msm_sensor_output_reg_addr_t ov2675_reg_addr =
{
	.x_output = 0x3088,
	.y_output = 0x308A,
	.line_length_pclk = 0x3024,
	.frame_length_lines = 0x3026,
};

static enum msm_camera_vreg_name_t ov2675_veg_seq_evt0[] = {
	CAM_VIO,
	//CAM_VDIG, //Eric Liu, front cam don't use VDIG
	CAM_VANA,
};

static enum msm_camera_vreg_name_t ov2675_veg_seq_evt1[] = {
	CAM_VIO,
	//CAM_VDIG, //Eric Liu, front cam don't use VDIG
	CAM_VANA_EXT, //Sophia Wang, 20130115, for cam_analog, we won't use l9 regulator, instead of gpio 35
};

static enum msm_camera_vreg_name_t ov2675_veg_seq[] = {
	CAM_VIO,
	//CAM_VDIG, //Eric Liu, front cam don't use VDIG
	CAM_VANA_EXT, //Sophia Wang, 20130115, for cam_analog, we won't use l9 regulator, instead of gpio 35
};

static struct msm_sensor_id_info_t ov2675_id_info =
{
	.sensor_id_reg_addr = 0x300A,
	.sensor_id = 0x2656,
};

static const struct i2c_device_id ov2675_i2c_id[] =
{
	{ SENSOR_NAME, (kernel_ulong_t)&ov2675_s_ctrl },
	{},
};

static struct i2c_driver ov2675_i2c_driver =
{
	.id_table = ov2675_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver =
	{
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov2675_sensor_i2c_client =
{
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init ov2675_sensor_init_module(void)
{
	return i2c_add_driver(&ov2675_i2c_driver);
}

static struct v4l2_subdev_core_ops ov2675_subdev_core_ops =
{
    .s_ctrl = msm_sensor_v4l2_s_ctrl,
    .queryctrl = msm_sensor_v4l2_query_ctrl,
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ov2675_subdev_video_ops =
{
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ov2675_subdev_ops =
{
	.core = &ov2675_subdev_core_ops,
	.video  = &ov2675_subdev_video_ops,
};

static struct msm_sensor_fn_t ov2675_func_tbl =
{
	.sensor_start_stream = ov2675_sensor_start_stream,
	.sensor_stop_stream = ov2675_sensor_stop_stream,
#ifdef USING_GROUP_HOLD_ON
    .sensor_group_hold_on = msm_sensor_group_hold_on,
    .sensor_group_hold_off = msm_sensor_group_hold_off,
#endif //USING_GROUP_HOLD_ON
    //Hiko: added
    //.sensor_csi_setting = msm_sensor_setting1,
    
    //.sensor_set_fps = msm_sensor_set_fps,
	//.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	//.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
	
    .sensor_setting = ov2675_sensor_setting,
	.sensor_set_sensor_mode = ov2675_msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov2675_msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down, //ov2675_msm_sensor_power_down,

    //Hiko: added
    //.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines,
	
    .sensor_get_csi_params = msm_sensor_get_csi_params,
    .sensor_writ_otp = ov2675_write_otp,
    .sensor_get_fuse_id = ov2675_read_fuse_id,
    .sensor_read_lens_id = ov2675_read_lens_id,
};

static struct msm_sensor_reg_t ov2675_regs =
{
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = ov2675_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(ov2675_start_settings),
	.stop_stream_conf = ov2675_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(ov2675_stop_settings),
#ifdef USING_GROUP_HOLD_ON
	.group_hold_on_conf = ov2675_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(ov2675_groupon_settings),
	.group_hold_off_conf = ov2675_groupoff_settings,
	.group_hold_off_conf_size = ARRAY_SIZE(ov2675_groupoff_settings),
#endif //USING_GROUP_HOLD_ON
	.init_settings = &ov2675_init_conf[0],
	.init_size = ARRAY_SIZE(ov2675_init_conf),
	.mode_settings = &ov2675_confs[0],
	.output_settings = &ov2675_dimensions[0],
	.num_conf = ARRAY_SIZE(ov2675_confs),
};

static struct msm_sensor_ctrl_t ov2675_s_ctrl =
{
	.msm_sensor_reg = &ov2675_regs,
	.msm_sensor_v4l2_ctrl_info = ov2675_v4l2_ctrl_info,
	.num_v4l2_ctrl = ARRAY_SIZE(ov2675_v4l2_ctrl_info),
	.sensor_i2c_client = &ov2675_sensor_i2c_client,
	.sensor_i2c_addr = 0x60,

	.vreg_seq_evt0 = ov2675_veg_seq_evt0,
	.num_vreg_seq_evt0 = ARRAY_SIZE(ov2675_veg_seq_evt0),
	.vreg_seq_evt1 = ov2675_veg_seq_evt1,
	.num_vreg_seq_evt1 = ARRAY_SIZE(ov2675_veg_seq_evt1),
	.vreg_seq = ov2675_veg_seq,
	.num_vreg_seq = ARRAY_SIZE(ov2675_veg_seq),

	.sensor_output_reg_addr = &ov2675_reg_addr,
	.sensor_id_info = &ov2675_id_info,
	//.sensor_exp_gain_info = &ov2675_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	//.csi_params = &ov2675_csi_params_array[0],
	//.csic_params = &ov2675_csi_params_array[0],
	.msm_sensor_mutex = &ov2675_mut,
	.sensor_i2c_driver = &ov2675_i2c_driver,
	.sensor_v4l2_subdev_info = ov2675_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov2675_subdev_info),
	.sensor_v4l2_subdev_ops = &ov2675_subdev_ops,
	.func_tbl = &ov2675_func_tbl,
	.wait_num_frames = 3,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(ov2675_sensor_init_module);
MODULE_DESCRIPTION("OV 2M YUV sensor driver");
MODULE_LICENSE("GPL v2");

