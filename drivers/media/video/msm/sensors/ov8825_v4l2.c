/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#include "msm_sensor.h"
#include "msm.h"
#define SENSOR_NAME "ov8825"
#define PLATFORM_DRIVER_NAME "msm_camera_ov8825"
#define ov8825_obj ov8825_##obj

//#define OTP
//#define MIPI_2Lane

#define LSC_REG_NUM 62
#define DETROIT

//#undef CDBG
//#define CDBG printk

/* TO DO - Currently ov5647 typical values are used
 * Need to get the exact values */
#define OV8825_RG_RATIO_TYPICAL_VALUE 64 /* R/G of typical camera module */
#define OV8825_BG_RATIO_TYPICAL_VALUE 105 /* B/G of typical camera module */

int initOtp = false;

DEFINE_MUTEX(ov8825_mut);
static struct msm_sensor_ctrl_t ov8825_s_ctrl;

#if 0
typedef struct otp_struct {
	uint8_t customer_id;
	uint8_t module_integrator_id;
	uint8_t lens_id;
	uint8_t year;
	uint8_t mouth;
	uint8_t date;
	uint8_t hour;
	uint8_t minuite;
	uint8_t second;
	uint8_t rg_ratio_d65;
	uint8_t bg_ratio_d65;
	uint8_t gb_gr_d65;
	uint8_t rg_ratio_cwf;
	uint8_t bg_ratio_cwf;
	uint8_t gb_gr_cwf;
	uint8_t rg_ratio_alight;
	uint8_t bg_ratio_alight;
	uint8_t gb_gr_alight;
//[0]:AF_START_DAC_MSB
//[1]:AF_START_DAC_LSB
//[2]:AF_MACRO_DAC_MSB
//[3]:iAF_MACRO_DAC_LSB
       uint8_t ov8825_af_otp[4];
//	uint8_t user_data[5];
} st_ov8825_otp;

static uint8_t ov8825_af_otp[4] =
{0x00};
#endif

static struct otp_struct st_ov8825_otp;

static uint8_t ov8825_lsc_D65_data[LSC_REG_NUM] =
{0x00};





static struct msm_camera_i2c_reg_conf ov8825_start_settings[] = {
//	{0x0100, 0x01},
//	{0x0100, 0x01},
{0x301a, 0x70}, //2012 07 07, sophia wang use 0x301a[0] to instead 0x0100[0] to avoid when sw reset, af lens will move to ori
};

static struct msm_camera_i2c_reg_conf ov8825_stop_settings[] = {
//	{0x0100, 0x00},
{0x301a, 0x71}, 	//2012 07 07, sophia wang use 0x301a[0] to instead 0x0100[0] to avoid when sw reset, af lens will move to ori	
};

static struct msm_camera_i2c_reg_conf ov8825_groupon_settings[] = {
	{0x3208, 0x00},
};

static struct msm_camera_i2c_reg_conf ov8825_groupoff_settings[] = {
	{0x3208, 0x10},
	{0x3208, 0xA0},
};

#if 0 // copy from QC 8820 4-lane driver, workable
static struct msm_camera_i2c_reg_conf ov8825_prev_settings[] = {
       {0x3005, 0x11},
       {0x3006, 0x11},
       {0x3501, 0x4e},
       {0x370e, 0x08},
       {0x3808, 0x06},
       {0x3809, 0x60},
       {0x380a, 0x04},
       {0x380b, 0xc8},
       {0x380c, 0x06},
       {0x380d, 0xde},
       {0x380e, 0x05},
       {0x380f, 0x05},
       {0x3811, 0x08},
       {0x3813, 0x04},
       {0x3814, 0x31},
       {0x3815, 0x31},
       {0x3f00, 0x00},
};
 #endif

#if 1
//1632x1224 30fps 408Mbps/Lane
//file: 8825_R1.25_20130408
//1632x1224_4lane_30fps_Sysclk=133.3M
static struct msm_camera_i2c_reg_conf ov8825_prev_settings[] = {
	{0x3004, 0xce},
	{0x3006, 0x10},
	{0x3020, 0x01},
	{0x3501, 0x4e},
	{0x3502, 0xa0},
	{0x3700, 0x20},
	{0x3702, 0x50},
	{0x3703, 0xcc},
	{0x3704, 0x19},
	{0x3705, 0x32},
	{0x3706, 0x4b},
	{0x3708, 0x84},
	{0x3709, 0x40},
	{0x370a, 0x33},
	{0x3711, 0x0f},
	{0x3712, 0x9c},
	{0x3724, 0x01},
	{0x3725, 0x92},
	{0x3726, 0x01},
	{0x3727, 0xc7},
	//{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	//{0x3804, 0x00},
	//{0x3805, 0x00},
	{0x3806, 0x09},
	{0x3807, 0x9b},
	{0x3808, 0x06},
	{0x3809, 0x60},
	{0x380a, 0x04},
	{0x380b, 0xc8},
	{0x380c, 0x0d},
	{0x380d, 0xbc},
	{0x380e, 0x04},
	{0x380f, 0xf0},
	{0x3811, 0x08},
	{0x3813, 0x04},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3820, 0x81},
	{0x3821, 0x17},
	{0x3f00, 0x00},
	{0x4005, 0x18},
	{0x4601, 0x00},
	{0x4602, 0x30},
	{0x4837, 0x27},
	{0x5068, 0x00},
	{0x506a, 0x00},

};
#endif

//3264x2448 24fps 528Mbps/Lane
//8825_R1.25_20130408
//3264x2448_4_lane_24fps_Sysclk=200M
static struct msm_camera_i2c_reg_conf ov8825_snap_settings[] = {
	{0x3004, 0xbf},
	{0x3006, 0x00},
	{0x3020, 0x01},
	{0x3501, 0x9a},
	{0x3502, 0xa0},
	{0x3700, 0x20},
	{0x3702, 0x50},
	{0x3703, 0xcc},
	{0x3704, 0x19},
	{0x3705, 0x32},
	{0x3706, 0x4b},
	{0x3708, 0x84},
	{0x3709, 0x40},
	{0x370a, 0x31},
	{0x3711, 0x0f},
	{0x3712, 0x9c},
	{0x3724, 0x01},
	{0x3725, 0x92},
	{0x3726, 0x01},
	{0x3727, 0xc7},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3806, 0x09},
	{0x3807, 0x9b},
	{0x3808, 0x0c},
	{0x3809, 0xc0},
	{0x380a, 0x09},
	{0x380b, 0x90},
	{0x380c, 0x0d},
	{0x380d, 0x20},
	{0x380e, 0x09},
	{0x380f, 0xb0},
	{0x3811, 0x10},
	{0x3813, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x80},
	{0x3821, 0x16},
	{0x3f00, 0x02},
	{0x4005, 0x1a},
	{0x4601, 0x00},
	{0x4602, 0x20},
	{0x4837, 0x1e},
	{0x5068, 0x00},
	{0x506a, 0x00},
};

//Video 1080p Full FOV 30fps 408Mbps/Lane  
//8825_R1.25_20130408
//1920x1080_4lane_30fps_1Sysclk=200M_408MBps/lane 
static struct msm_camera_i2c_reg_conf ov8825_1080p_settings[] = {

      {0x3004, 0xce},
      {0x3006, 0x00},
      {0x3020, 0x01},
      {0x3501, 0x74},
      {0x3502, 0x60},
      {0x3700, 0x20},
      {0x3702, 0x50},
      {0x3703, 0xcc},
      {0x3704, 0x19},
      {0x3705, 0x32},
      {0x3706, 0x4b},
      {0x3708, 0x84},
      {0x3709, 0x40},
      {0x370a, 0x31},
      {0x3711, 0x0f},
      {0x3712, 0x9c},
      {0x3724, 0x01},
      {0x3725, 0x92},
      {0x3726, 0x01},
      {0x3727, 0xc7},
      {0x3802, 0x01},
      {0x3803, 0x30},
      {0x3806, 0x08},
      {0x3807, 0x67},
      {0x3808, 0x07},
      {0x3809, 0x80},
      {0x380a, 0x04},
      {0x380b, 0x38},
      {0x380c, 0x0d},
      {0x380d, 0xf0},
      {0x380e, 0x07},
      {0x380f, 0x4c},
      {0x3811, 0x10},
      {0x3813, 0x06},
      {0x3814, 0x11},
      {0x3815, 0x11},
      {0x3820, 0x80},
      {0x3821, 0x16},
      {0x3f00, 0x02},
      {0x4005, 0x18},
      {0x4601, 0x01},
      {0x4602, 0x00},
      {0x4837, 0x27},
      {0x5068, 0x53},
      {0x506a, 0x53},
};

// Settings for MIPI 4 Lane
// 8825_R1.25_20130408
static struct msm_camera_i2c_reg_conf ov8825_recommend_settings[] = {
      {0x3000, 0x16},
      {0x3001, 0x00},
      {0x3002, 0x6c},
      {0x3003, 0xce},
      {0x3004, 0xce},
      {0x3005, 0x10},
      {0x3006, 0x10},
      {0x3007, 0x3b},
      {0x300d, 0x00},
      {0x301f, 0x09},
      {0x3020, 0x01},
      {0x3010, 0x00},
      {0x3011, 0x02},
      {0x3012, 0x80},
      {0x3013, 0x39},
      {0x3018, 0x00},
      {0x3104, 0x20},
      {0x3106, 0x15},
      {0x3300, 0x00},
      {0x3500, 0x00},
      {0x3501, 0x4e},
      {0x3502, 0xa0},
      {0x3503, 0x07},
      {0x3509, 0x00},//[5] should be 0, we sent the sensor gain
      {0x350b, 0x1f},
      {0x3600, 0x06},
      {0x3601, 0x34},
      {0x3602, 0x42},
      {0x3603, 0x5c},
      {0x3604, 0x98},
      {0x3605, 0xf5},
      {0x3609, 0xb4},
      {0x360a, 0x7c},
      {0x360b, 0xc9},
      {0x360c, 0x0b},
      {0x3612, 0x00},
      {0x3613, 0x02},
      {0x3614, 0x0f},
      {0x3615, 0x00},
      {0x3616, 0x03},
      {0x3617, 0xa1},
      {0x3618, 0x00},
      {0x3619, 0x00},
      {0x361a, 0xb0},// sophia, for VCM use, change from 00->b0 
      {0x361b, 0x04},// sophia, for VCM use, change from 00->04
      {0x361c, 0x04},// sophia, truly suggest related to af
      {0x3700, 0x20},
      {0x3701, 0x44},
      {0x3702, 0x50},
      {0x3703, 0xcc},
      {0x3704, 0x19},
      {0x3705, 0x32},
      {0x3706, 0x4b},
      {0x3707, 0x63},
      {0x3708, 0x84},
      {0x3709, 0x40},
      {0x370a, 0x33},
      {0x370b, 0x01},
      {0x370c, 0x50},
      {0x370d, 0x00},
      {0x370e, 0x00},
      {0x3711, 0x0f},
      {0x3712, 0x9c},
      {0x3724, 0x01},
      {0x3725, 0x92},
      {0x3726, 0x01},
      {0x3727, 0xc7},
      {0x3800, 0x00},
      {0x3801, 0x00},
      {0x3802, 0x00},
      {0x3803, 0x00},
      {0x3804, 0x0c},
      {0x3805, 0xdf},
      {0x3806, 0x09},
      {0x3807, 0x9b},
      {0x3808, 0x06},
      {0x3809, 0x60},
      {0x380a, 0x04},
      {0x380b, 0xc8},
      {0x380c, 0x0d},
      {0x380d, 0xbc},
      {0x380e, 0x04},
      {0x380f, 0xf0},
      {0x3810, 0x00},
      {0x3811, 0x08},
      {0x3812, 0x00},
      {0x3813, 0x04},
      {0x3814, 0x31},
      {0x3815, 0x31},
      {0x3816, 0x02},
      {0x3817, 0x40},
      {0x3818, 0x00},
      {0x3819, 0x40},
      {0x3820, 0x81},
      {0x3821, 0x17},
      {0x3b1f, 0x00},
      {0x3d00, 0x00},
      {0x3d01, 0x00},
      {0x3d02, 0x00},
      {0x3d03, 0x00},
      {0x3d04, 0x00},
      {0x3d05, 0x00},
      {0x3d06, 0x00},
      {0x3d07, 0x00},
      {0x3d08, 0x00},
      {0x3d09, 0x00},
      {0x3d0a, 0x00},
      {0x3d0b, 0x00},
      {0x3d0c, 0x00},
      {0x3d0d, 0x00},
      {0x3d0e, 0x00},
      {0x3d0f, 0x00},
      {0x3d10, 0x00},
      {0x3d11, 0x00},
      {0x3d12, 0x00},
      {0x3d13, 0x00},
      {0x3d14, 0x00},
      {0x3d15, 0x00},
      {0x3d16, 0x00},
      {0x3d17, 0x00},
      {0x3d18, 0x00},
      {0x3d19, 0x00},
      {0x3d1a, 0x00},
      {0x3d1b, 0x00},
      {0x3d1c, 0x00},
      {0x3d1d, 0x00},
      {0x3d1e, 0x00},
      {0x3d1f, 0x00},
      {0x3d80, 0x00},
      {0x3d81, 0x00},
      {0x3d84, 0x00},
      {0x3f00, 0x00},
      {0x3f01, 0xfc},
      {0x3f05, 0x10},
      {0x3f06, 0x00},
      {0x3f07, 0x00},
      {0x4000, 0x29},
      {0x4001, 0x02},
      {0x4002, 0x45},
      {0x4003, 0x08},
      {0x4004, 0x04},
      {0x4005, 0x18},
      {0x404e, 0x37},
      {0x404f, 0x8f},
      {0x4300, 0xff},
      {0x4303, 0x00},
      {0x4304, 0x08},
      {0x4307, 0x00},
      {0x4600, 0x04},
      {0x4601, 0x00},
      {0x4602, 0x30},
      {0x4800, 0x24},// sophia 0x04->0x24 for use 0x301a instead of 0100
      {0x4801, 0x0f},
      {0x4837, 0x27},
      {0x4843, 0x02},
      {0x5000, 0x86},
      {0x5001, 0x00},
      {0x5002, 0x00},
      {0x5068, 0x00},
      {0x506a, 0x00},
      {0x501f, 0x00},
      {0x5780, 0xfc},
      {0x5c00, 0x80},
      {0x5c01, 0x00},
      {0x5c02, 0x00},
      {0x5c03, 0x00},
      {0x5c04, 0x00},
      {0x5c05, 0x00},
      {0x5c06, 0x00},
      {0x5c07, 0x80},
      {0x5c08, 0x10},
      {0x6700, 0x05},
      {0x6701, 0x19},
      {0x6702, 0xfd},
      {0x6703, 0xd7},
      {0x6704, 0xff},
      {0x6705, 0xff},
      {0x6800, 0x10},
      {0x6801, 0x02},
      {0x6802, 0x90},
      {0x6803, 0x10},
      {0x6804, 0x59},
      {0x6900, 0x60},
      {0x6901, 0x05},//modify the binning method 0x4->0x5, sophia
      {0x0100, 0x01},
      {0x5800, 0x0f},
      {0x5801, 0x0d},
      {0x5802, 0x09},
      {0x5803, 0x0a},
      {0x5804, 0x0d},
      {0x5805, 0x14},
      {0x5806, 0x0a},
      {0x5807, 0x04},
      {0x5808, 0x03},
      {0x5809, 0x03},
      {0x580a, 0x05},
      {0x580b, 0x0a},
      {0x580c, 0x05},
      {0x580d, 0x02},
      {0x580e, 0x00},
      {0x580f, 0x00},
      {0x5810, 0x03},
      {0x5811, 0x05},
      {0x5812, 0x09},
      {0x5813, 0x03},
      {0x5814, 0x01},
      {0x5815, 0x01},
      {0x5816, 0x04},
      {0x5817, 0x09},
      {0x5818, 0x09},
      {0x5819, 0x08},
      {0x581a, 0x06},
      {0x581b, 0x06},
      {0x581c, 0x08},
      {0x581d, 0x06},
      {0x581e, 0x33},
      {0x581f, 0x11},
      {0x5820, 0x0e},
      {0x5821, 0x0f},
      {0x5822, 0x11},
      {0x5823, 0x3f},
      {0x5824, 0x08},
      {0x5825, 0x46},
      {0x5826, 0x46},
      {0x5827, 0x46},
      {0x5828, 0x46},
      {0x5829, 0x46},
      {0x582a, 0x42},
      {0x582b, 0x42},
      {0x582c, 0x44},
      {0x582d, 0x46},
      {0x582e, 0x46},
      {0x582f, 0x60},
      {0x5830, 0x62},
      {0x5831, 0x42},
      {0x5832, 0x46},
      {0x5833, 0x46},
      {0x5834, 0x44},
      {0x5835, 0x44},
      {0x5836, 0x44},
      {0x5837, 0x48},
      {0x5838, 0x28},
      {0x5839, 0x46},
      {0x583a, 0x48},
      {0x583b, 0x68},
      {0x583c, 0x28},
      {0x583d, 0xae},
      {0x5842, 0x00},
      {0x5843, 0xef},
      {0x5844, 0x01},
      {0x5845, 0x3f},
      {0x5846, 0x01},
      {0x5847, 0x3f},
      {0x5848, 0x00},
      {0x5849, 0xd5},
};

#if 0 //sophia, 20130422, TA version, ov8825_snap_settings, ov8825_1080p_settings, ov8825_recommend_settings
static struct msm_camera_i2c_reg_conf ov8825_snap_settings[] = {
       {0x3005, 0x10},
       {0x3006, 0x00}, // ori 10
       {0x3501, 0x9a},
       {0x370e, 0x00},
       {0x3800, 0x00},/*HS(HREF start High)*/
       {0x3801, 0x00},
       {0x3802, 0x00},
       {0x3803, 0x00},
       {0x3804, 0x0c},
       {0x3805, 0xdf},
       {0x3806, 0x09},
       {0x3807, 0x9b},       
       {0x3808, 0x0c},
       {0x3809, 0xc0},
       {0x380a, 0x09},
       {0x380b, 0x90},
       {0x380c, 0x0e},
       {0x380d, 0x00},
       {0x380e, 0x09},
       {0x380f, 0xb0},
       {0x3811, 0x10},
       {0x3813, 0x06},
       {0x3814, 0x11},
       {0x3815, 0x11},
       {0x3821, 0x16},
       {0x3f00, 0x02},
      	{0x4601, 0x00}, // add for due to 1080p setting[20130201]
       {0x4602, 0x78}, // add for due to 1080p setting.[20130201]      
       {0x5068, 0x00},
	{0x506a, 0x00},       
};

// workable
static struct msm_camera_i2c_reg_conf ov8825_1080p_settings[] = {
//       {0x3004, 0xd4}, 0xbf
//       {0x3005, 0x00}, 0x10
//       {0x3006, 0x00}, 0x00
       {0x3501, 0x74},// AEC control
       {0x3502, 0x60},// AEC control
       {0x370e, 0x00},
     #if 1
       {0x3800, 0x00}, // horizontal start HS[12:8]
       {0x3801, 0x00}, // horizontal start HS[7:0]
       {0x3802, 0x01}, // VS[11:8]
       {0x3803, 0x30}, // VS[7:0]
       {0x3804, 0x0c}, // horiaontal end HW[12:8]
       {0x3805, 0xdf}, // HW[7:0]
       {0x3806, 0x08}, // VH[11:8]
       {0x3807, 0x67}, // VH[7:0]
       #endif
       {0x3808, 0x07},
       {0x3809, 0x80},
       {0x380a, 0x04},
       {0x380b, 0x40},
       //{0x380b, 0x38},
 	{0x380c, 0x0d},
       {0x380d, 0xf0},
       {0x380e, 0x07},
       {0x380f, 0x4c},
       {0x3811, 0x08},//0x08 H offset
       {0x3813, 0x06},
       {0x3814, 0x11},       
       {0x3815, 0x11},
//      	{0x3820, 0x80},
       {0x3821, 0x16}, //sophia, 20130324, don't set [0] to 1, which it is in crop mode
       {0x3f00, 0x02},// 0x18
      	{0x4601, 0x01},
       {0x4602, 0x00},
//       {0x4602, 0x78},
    //   {0x5000, 0x06},
       {0x5068, 0x53},
	{0x506a, 0x53},
};

static struct msm_camera_i2c_reg_conf ov8825_recommend_settings[] = {
       {0x3000, 0x16}, //ov8825, change from 02->16
       {0x3001, 0x00},
       {0x3002, 0x6c},
       {0x3003, 0xce},
       {0x3004, 0xbf},  //ori d5
       {0x3005, 0x10},// ov8825, change from 0x11->0x10
       {0x3006, 0x00},// ov8825, change from 0x11->0x00
       {0x3007, 0x3b},
       {0x300d, 0x00},
       {0x301f, 0x09},
       {0x3010, 0x00},
       {0x3011, 0x02},
       {0x3012, 0x80},
       {0x3013, 0x39},
       {0x3018, 0x00},
       {0x3104, 0x20},
       {0x3106, 0x15}, //add for 8825
       {0x3300, 0x00},
       {0x3500, 0x00},
       {0x3501, 0x4e},
       {0x3502, 0xa0},
       {0x3503, 0x07},
       {0x3509, 0x00},
       {0x350b, 0x1f},
       {0x3600, 0x06},// ov8825, change from 0x05->0x06
       {0x3601, 0x34},// ov8825, change from 0x32->0x34
       {0x3602, 0x42},// ov8825, change from 0x44->0x42
       {0x3603, 0x5c},
       {0x3604, 0x98},
       {0x3605, 0xf5},// ov8825, change from 0xe9->0xf5
       {0x3609, 0xb4},// ov8825, change from 0xb8->0xb4
       {0x360a, 0x7c},// ov8825, change from 0xbc->0x7c
       {0x360b, 0xc9},// ov8825, change from 0xb4->0xc9
       {0x360c, 0x0b},// ov8825, change from 0x0d->0x0b
       {0x3612, 0x00},// add by 8825
       {0x3613, 0x02},
       {0x3614, 0x0f},
       {0x3615, 0x00},
       {0x3616, 0x03},
       {0x3617, 0xa1},// ov8825, change from 0x01->0xa1
       {0x3618, 0x00},
       {0x3619, 0x00},
       {0x361a, 0x00},
       {0x361b, 0x00},
       {0x361a, 0xb0}, //sophia, wang for VCM use, from 00->b0
       {0x361b, 0x04}, // sophia wang for VCM, from 00->04        
       {0x361c, 0x04}, // sophia, truly suggest, related to af      
       {0x3700, 0x10},
       {0x3701, 0x44},
       {0x3702, 0x28},// ov8825, change from 0x70->0x50->28
       {0x3703, 0x6c},// ov8825, change from 0x4f->0xcc->6c
       {0x3704, 0x8d},// ov8825, change from 0x69->0x19->8d
       {0x3705, 0x0a},// add by 8825
       {0x3706, 0x27},// ov8825, change from 0x7b->0x4b->27
       {0x3707, 0x63},
       {0x3708, 0x40},// ov8825, change from 0x85->0x84->40
       {0x3709, 0x20},
       {0x370a, 0x33},// ov8825, change from 0x12->0x31->33
       {0x370b, 0x01},
       {0x370c, 0x50},
       {0x370d, 0x00},// ov8825, change from 0x0c->0x00
       {0x370e, 0x08},// ov8825, change from 0x08->0x00-<08
       {0x3711, 0x07},// ov8825, change from 0x01->0x0f->07
       {0x3712, 0x4e},// ov8825, change from 0xcc->0x9c->4e
       {0x3724, 0x00},// add by 8825
       {0x3725, 0xd4},// add by 8825
       {0x3726, 0x00},// add by 8825
       {0x3727, 0xe1},// add by 8825
       {0x3800, 0x00},/*HS(HREF start High)*/
       {0x3801, 0x00},
       {0x3802, 0x00},
       {0x3803, 0x00},
       {0x3804, 0x0c},
       {0x3805, 0xdf},
       {0x3806, 0x09},
       {0x3807, 0x9b},
       {0x3808, 0x06},
       {0x3809, 0x60},
       {0x380a, 0x04},
       {0x380b, 0xc8},
       {0x380c, 0x06},
       {0x380d, 0xde},
       {0x380e, 0x05},
       {0x380f, 0x05},
       {0x3810, 0x00},
       {0x3811, 0x08},
       {0x3812, 0x00},
       {0x3813, 0x04},
       {0x3814, 0x31},
       {0x3815, 0x31},
       {0x3816, 0x02},
       {0x3817, 0x40},
       {0x3818, 0x00},
       {0x3819, 0x40},
       {0x3820, 0x80},// ov8825, change from 0x00->0x80
       {0x3821, 0x17},
       {0x3b1f, 0x00},// add by 8825
       {0x3d00, 0x00},
       {0x3d01, 0x00},
       {0x3d02, 0x00},
       {0x3d03, 0x00},
       {0x3d04, 0x00},
       {0x3d05, 0x00},
       {0x3d06, 0x00},
       {0x3d07, 0x00},
       {0x3d08, 0x00},
       {0x3d09, 0x00},
       {0x3d0a, 0x00},
       {0x3d0b, 0x00},
       {0x3d0c, 0x00},
       {0x3d0d, 0x00},
       {0x3d0e, 0x00},
       {0x3d0f, 0x00},
       {0x3d10, 0x00},
       {0x3d11, 0x00},
       {0x3d12, 0x00},
       {0x3d13, 0x00},
       {0x3d14, 0x00},
       {0x3d15, 0x00},
       {0x3d16, 0x00},
       {0x3d17, 0x00},
       {0x3d18, 0x00},
       {0x3d19, 0x00},
       {0x3d1a, 0x00},
       {0x3d1b, 0x00},
       {0x3d1c, 0x00},
       {0x3d1d, 0x00},
       {0x3d1e, 0x00},
       {0x3d1f, 0x00},
       {0x3d80, 0x00},
       {0x3d81, 0x00},
       {0x3d84, 0x00},
       {0x3f00, 0x00},
       {0x3f01, 0xfc},
       {0x3f05, 0x10},
       {0x3f06, 0x00},
       {0x3f07, 0x00},
       {0x4000, 0x29},
       {0x4001, 0x02},
       {0x4002, 0x45},
       {0x4003, 0x08},
       {0x4004, 0x04},
       {0x4005, 0x1a},
       {0x4300, 0xff},
       {0x4303, 0x00},
       {0x4304, 0x08},
       {0x4307, 0x00},
       {0x4600, 0x04},
       {0x4601, 0x00},
       {0x4602, 0x78},
       {0x4800, 0x24},// sophia 0x04->0x24 for use 0x301a instead of 0100
       {0x4801, 0x0f},
       {0x4837, 0x28},
       {0x4843, 0x02},
       {0x5000, 0x06},
       {0x5001, 0x00},
       {0x5002, 0x00},
       {0x5068, 0x00},
       {0x506a, 0x00},
       {0x501f, 0x00},
       {0x5780, 0xfc},
       {0x5c00, 0x80},
       {0x5c01, 0x00},
       {0x5c02, 0x00},
       {0x5c03, 0x00},
       {0x5c04, 0x00},
       {0x5c05, 0x00},
       {0x5c06, 0x00},
       {0x5c07, 0x80},
       {0x5c08, 0x10},
       {0x6700, 0x05},
       {0x6701, 0x19},
       {0x6702, 0xfd},
       {0x6703, 0xd1},
       {0x6704, 0xff},
       {0x6705, 0xff},
       {0x6800, 0x10},
       {0x6801, 0x02},
       {0x6802, 0x90},
       {0x6803, 0x10},
       {0x6804, 0x59},
       {0x6900, 0x61},
       {0x6901, 0x05},//0x04->0x05
#if 0       
       {0x3612, 0x00},
       {0x3617, 0xa1},
       {0x3b1f, 0x00},
       {0x3000, 0x12},
       {0x3000, 0x16},
       {0x3b1f, 0x00},
 #endif      
       {0x0100, 0x01},
       {0x5800, 0x0f},
       {0x5801, 0x0d},
       {0x5802, 0x09},
       {0x5803, 0x0a},
       {0x5804, 0x0d},
       {0x5805, 0x14},
       {0x5806, 0x0a},
       {0x5807, 0x04},
       {0x5808, 0x03},
       {0x5809, 0x03},
       {0x580a, 0x05},
       {0x580b, 0x0a},
       {0x580c, 0x05},
       {0x580d, 0x02},
       {0x580e, 0x00},
       {0x580f, 0x00},
       {0x5810, 0x03},
       {0x5811, 0x05},
       {0x5812, 0x09},
       {0x5813, 0x03},
       {0x5814, 0x01},
       {0x5815, 0x01},
       {0x5816, 0x04},
       {0x5817, 0x09},
       {0x5818, 0x09},
       {0x5819, 0x08},
       {0x581a, 0x06},
       {0x581b, 0x03},
       {0x581c, 0x05},
       {0x581d, 0x06},
       {0x581e, 0x13},
       {0x581f, 0x0b},
       {0x5820, 0x09},
       {0x5821, 0x0f},
       {0x5822, 0x11},
       {0x5823, 0x3f},
       {0x5824, 0x08},
       {0x5825, 0x46},
       {0x5826, 0x46},
       {0x5827, 0x46},
       {0x5828, 0x46},
       {0x5829, 0x46},
       {0x582a, 0x42},
       {0x582b, 0x42},
       {0x582c, 0x44},
       {0x582d, 0x46},
       {0x582e, 0x46},
       {0x582f, 0x60},
       {0x5830, 0x62},
       {0x5831, 0x42},
       {0x5832, 0x46},
       {0x5833, 0x46},
       {0x5834, 0x44},
       {0x5835, 0x44},
       {0x5836, 0x44},
       {0x5837, 0x48},
       {0x5838, 0x28},
       {0x5839, 0x46},
       {0x583a, 0x48},
       {0x583b, 0x68},
       {0x583c, 0x28},
       {0x583d, 0xae},
       {0x5842, 0x00},
       {0x5843, 0xef},
       {0x5844, 0x01},
       {0x5845, 0x3f},
       {0x5846, 0x01},
       {0x5847, 0x3f},
       {0x5848, 0x00},
       {0x5849, 0xd5},
};
#endif

static struct msm_camera_i2c_reg_conf ov8825_reset_settings[] = {
	{0x0103, 0x01},
};

#if 0
#ifdef MIPI_2Lane

#ifdef DETROIT
static struct msm_camera_i2c_reg_conf ov8825_prev_settings[] = {
	{0x3004, 0xd4},
	{0x3005, 0x00},
	{0x3006, 0x10},
	{0x3501, 0x4e},
	{0x3502, 0xa0},	
	{0x3600, 0x06}, //add for 8825
	{0x3601, 0x34}, //add for 8825
	{0x3602, 0x42}, //add for 8825	
       {0x360b, 0xc9}, //add for 8825
       {0x3700, 0x20}, //add for 8825
	{0x3702, 0x50}, //add for 8825
	{0x3703, 0xcc}, //add for 8825
	{0x3704, 0x19}, //add for 8825
	{0x3705, 0x14}, //add for 8825
	{0x3708, 0x84}, //add for 8825
	{0x3709, 0x40}, //add for 8825	
	{0x3711, 0x0f}, //add for 8825
	{0x3724, 0x01}, //add for 8825
	{0x3725, 0x92}, //add for 8825
	{0x3726, 0x01}, //add for 8825
	{0x3727, 0xa9}, //add for 8825
       {0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0c},
	{0x3805, 0xdf},
	{0x3806, 0x09},
	{0x3807, 0x9b},
	{0x3808, 0x06},
	{0x3809, 0x68},
	{0x380a, 0x04},
	{0x380b, 0xce},
	{0x380c, 0x0d},
	{0x380d, 0xbc},
	{0x380e, 0x04},
	{0x380f, 0xf0},
	{0x3811, 0x04},
	{0x3813, 0x00},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3820, 0x81},		
	{0x3821, 0x17},
        {0x3f00, 0x00},
        {0x3f05, 0x10},
        {0x4600, 0x04},
        {0x4601, 0x00},
        {0x4602, 0x30},
        {0x4837, 0x28},
 //       {0x5000, 0x00},
        {0x5068, 0x00},
        {0x506a, 0x00},

	};

#else
static struct msm_camera_i2c_reg_conf ov8825_prev_settings[] = {
	{0x3003, 0xce}, /*PLL_CTRL0*/
	{0x3004, 0xd4}, /*PLL_CTRL1*/
	{0x3005, 0x00}, /*PLL_CTRL2*/
	{0x3006, 0x10}, /*PLL_CTRL3*/
	{0x3007, 0x3b}, /*PLL_CTRL4*/
	{0x3011, 0x01}, /*MIPI_Lane_4_Lane*/
	{0x3012, 0x80}, /*SC_PLL CTRL_S0*/
	{0x3013, 0x39}, /*SC_PLL CTRL_S1*/
	{0x3104, 0x20}, /*SCCB_PLL*/
	{0x3106, 0x15}, /*SRB_CTRL*/
	{0x3501, 0x4e}, /*AEC_HIGH*/
	{0x3502, 0xa0}, /*AEC_LOW*/
	{0x350b, 0x1f}, /*AGC*/
	{0x3600, 0x06}, /*ANACTRL0*/
	{0x3601, 0x34}, /*ANACTRL1*/
	{0x3700, 0x20}, /*SENCTROL0 Sensor control*/
	{0x3702, 0x50}, /*SENCTROL2 Sensor control*/
	{0x3703, 0xcc}, /*SENCTROL3 Sensor control*/
	{0x3704, 0x19}, /*SENCTROL4 Sensor control*/
	{0x3705, 0x14}, /*SENCTROL5 Sensor control*/
	{0x3706, 0x4b}, /*SENCTROL6 Sensor control*/
	{0x3707, 0x63}, /*SENCTROL7 Sensor control*/
	{0x3708, 0x84}, /*SENCTROL8 Sensor control*/
	{0x3709, 0x40}, /*SENCTROL9 Sensor control*/
	{0x370a, 0x12}, /*SENCTROLA Sensor control*/
	{0x370e, 0x00}, /*SENCTROLE Sensor control*/
	{0x3711, 0x0f}, /*SENCTROL11 Sensor control*/
	{0x3712, 0x9c}, /*SENCTROL12 Sensor control*/
	{0x3724, 0x01}, /*Reserved*/
	{0x3725, 0x92}, /*Reserved*/
	{0x3726, 0x01}, /*Reserved*/
	{0x3727, 0xa9}, /*Reserved*/
	{0x3800, 0x00}, /*HS(HREF start High)*/
	{0x3801, 0x00}, /*HS(HREF start Low)*/
	{0x3802, 0x00}, /*VS(Vertical start High)*/
	{0x3803, 0x00}, /*VS(Vertical start Low)*/
	{0x3804, 0x0c}, /*HW = 3295*/
	{0x3805, 0xdf}, /*HW*/
	{0x3806, 0x09}, /*VH = 2459*/
	{0x3807, 0x9b}, /*VH*/
	{0x3808, 0x06}, /*ISPHO = 1632*/
	{0x3809, 0x60}, /*ISPHO*/
	{0x380a, 0x04}, /*ISPVO = 1224*/
	{0x380b, 0xc8}, /*ISPVO*/
	{0x380c, 0x0d}, /*HTS = 3516*/
	{0x380d, 0xbc}, /*HTS*/
	{0x380e, 0x04}, /*VTS = 1264*/
	{0x380f, 0xf0}, /*VTS*/
	{0x3810, 0x00}, /*HOFF = 8*/
	{0x3811, 0x08}, /*HOFF*/
	{0x3812, 0x00}, /*VOFF = 4*/
	{0x3813, 0x04}, /*VOFF*/
	{0x3814, 0x31}, /*X INC*/
	{0x3815, 0x31}, /*Y INC*/
	{0x3820, 0x81}, /*Timing Reg20:Vflip*/
	{0x3821, 0x17}, /*Timing Reg21:Hmirror*/
	{0x3f00, 0x00}, /*PSRAM Ctrl0*/
	{0x3f01, 0xfc}, /*PSRAM Ctrl1*/
	{0x3f05, 0x10}, /*PSRAM Ctrl5*/
	{0x4600, 0x04}, /*VFIFO Ctrl0*/
	{0x4601, 0x00}, /*VFIFO Read ST High*/
	{0x4602, 0x30}, /*VFIFO Read ST Low*/
	{0x4837, 0x28}, /*MIPI PCLK PERIOD*/
	{0x5068, 0x00}, /*HSCALE_CTRL*/
	{0x506a, 0x00}, /*VSCALE_CTRL*/
	{0x5c00, 0x80}, /*PBLC CTRL00*/
	{0x5c01, 0x00}, /*PBLC CTRL01*/
	{0x5c02, 0x00}, /*PBLC CTRL02*/
	{0x5c03, 0x00}, /*PBLC CTRL03*/
	{0x5c04, 0x00}, /*PBLC CTRL04*/
	{0x5c08, 0x10}, /*PBLC CTRL08*/
	{0x6900, 0x61}, /*CADC CTRL00*/
};
#endif //Detroit
#endif //2-lane


#ifdef DETROIT
 static struct msm_camera_i2c_reg_conf ov8825_snap_settings[] = {
 #ifdef MIPI_2Lane
	{0x3004, 0xd4},
	{0x3005, 0x00},
	{0x3006, 0x10},
#else //4-lane
	{0x3004, 0xbf},
	{0x3005, 0x10},
	{0x3006, 0x00},
	{0x3011, 0x02},
#endif
	{0x3501, 0x9a},
	{0x3502, 0xa0},
	{0x350b, 0x1f},
	//{0x370a, 0x12},
	//{0x370e, 0x00},
	{0x3600, 0x06}, //add for 8825
	{0x3601, 0x34}, //add for 8825
	{0x3602, 0x42}, //add for 8825
       {0x360b, 0xc9}, //add for 8825
       {0x3700, 0x20},//dd for 8825
	{0x3702, 0x50}, //add for 8825
	{0x3703, 0xcc}, //add for 8825
	{0x3704, 0x19}, //add for 8825
	{0x3705, 0x14}, //add for 8825
	{0x3706, 0x4b}, //add for 8825
	{0x3708, 0x84}, //add for 8825
	{0x3709, 0x40}, //add for 8825
	{0x3711, 0x0f}, //add for 8825
	{0x3724, 0x01}, //add for 8825
	{0x3725, 0x92},//dd for 88
	{0x3726, 0x01},//for 8825
	{0x3727, 0xa9},//dd for 8825
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0c},
	{0x3805, 0xdf},
	{0x3806, 0x09},
	{0x3807, 0x9b},
	{0x3808, 0x0c},
	{0x3809, 0xd0},
	{0x380a, 0x09},
	{0x380b, 0x9c},
	{0x380c, 0x0e},
	{0x380d, 0x00},
	{0x380e, 0x09},
	{0x380f, 0xb0},
	{0x3811, 0x08},
	{0x3813, 0x00},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x80},
	{0x3821, 0x16},
	{0x3f00, 0x02},
	{0x3f05, 0x10},
	{0x4600, 0x04},
	{0x4601, 0x00},
	{0x4602, 0x78},
	{0x4837, 0x28},
	//{0x5000, 0x00},
	{0x5068, 0x00},
	{0x506a, 0x00},
};

#else
static struct msm_camera_i2c_reg_conf ov8825_snap_settings[] = {
#ifdef MIPI_2Lane
	{0x3003, 0xce}, /*PLL_CTRL0*/
	{0x3004, 0xd8}, /*PLL_CTRL1*/
	{0x3005, 0x00}, /*PLL_CTRL2*/
	{0x3006, 0x10}, /*PLL_CTRL3*/
#else // 4-lane
	{0x3003, 0xcc}, /*PLL_CTRL0*/
	{0x3004, 0xbf}, /*PLL_CTRL1*/
	{0x3005, 0x10}, /*PLL_CTRL2*/
	{0x3006, 0x00}, /*PLL_CTRL3*/
#endif
	{0x3007, 0x3b}, /*PLL_CTRL4*/
#ifdef MIPI_2Lane	
	{0x3011, 0x01}, /*MIPI_Lane_4_Lane*/
	{0x3012, 0x81}, /*SC_PLL CTRL_S0*/
#else //4 lane
	{0x3011, 0x02}, /*MIPI_Lane_4_Lane*/
	{0x3012, 0x80}, /*SC_PLL CTRL_S0*/
#endif
	{0x3013, 0x39}, /*SC_PLL CTRL_S1*/
	{0x3104, 0x20}, /*SCCB_PLL*/
	{0x3106, 0x11}, /*SRB_CTRL*/
	{0x3501, 0x9a}, /*AEC_HIGH*/
	{0x3502, 0xa0}, /*AEC_LOW*/
	{0x350b, 0x1f}, /*AGC*/
	{0x3600, 0x07}, /*ANACTRL0*/
	{0x3601, 0x33}, /*ANACTRL1*/
	{0x3700, 0x10}, /*SENCTROL0 Sensor control*/
	{0x3702, 0x28}, /*SENCTROL2 Sensor control*/
	{0x3703, 0x6c}, /*SENCTROL3 Sensor control*/
	{0x3704, 0x8d}, /*SENCTROL4 Sensor control*/
	{0x3705, 0x0a}, /*SENCTROL5 Sensor control*/
	{0x3706, 0x27}, /*SENCTROL6 Sensor control*/
	{0x3707, 0x63}, /*SENCTROL7 Sensor control*/
	{0x3708, 0x40}, /*SENCTROL8 Sensor control*/
	{0x3709, 0x20}, /*SENCTROL9 Sensor control*/
	{0x370a, 0x12}, /*SENCTROLA Sensor control*/
	{0x370e, 0x00}, /*SENCTROLE Sensor control*/
	{0x3711, 0x07}, /*SENCTROL11 Sensor control*/
	{0x3712, 0x4e}, /*SENCTROL12 Sensor control*/
	{0x3724, 0x00}, /*Reserved*/
	{0x3725, 0xd4}, /*Reserved*/
	{0x3726, 0x00}, /*Reserved*/
	{0x3727, 0xe1}, /*Reserved*/
	{0x3800, 0x00}, /*HS(HREF start High)*/
	{0x3801, 0x00}, /*HS(HREF start Low)*/
	{0x3802, 0x00}, /*VS(Vertical start Hgh)*/
	{0x3803, 0x00}, /*VS(Vertical start Low)*/
	{0x3804, 0x0c}, /*HW = 3295*/
	{0x3805, 0xdf}, /*HW*/
	{0x3806, 0x09}, /*VH = 2459*/
	{0x3807, 0x9b}, /*VH*/
	{0x3808, 0x0c}, /*ISPHO = 1632*/
	{0x3809, 0xc0}, /*ISPHO*/
	{0x380a, 0x09}, /*ISPVO = 1224*/
	{0x380b, 0x90}, /*ISPVO*/
#ifdef MIPI_2Lane	
	{0x380c, 0x0e}, /*HTS = 3516*/
	{0x380d, 0x00}, /*HTS*/
#else
	{0x380c, 0x0d}, /*HTS = 3360*/
	{0x380d, 0x20}, /*HTS*/
#endif
	{0x380e, 0x09}, /*VTS = 1264*/
	{0x380f, 0xb0}, /*VTS*/
	{0x3810, 0x00}, /*HOFF = 8*/
	{0x3811, 0x10}, /*HOFF*/
	{0x3812, 0x00}, /*VOFF = 4*/
	{0x3813, 0x06}, /*VOFF*/
	{0x3814, 0x11}, /*X INC*/
	{0x3815, 0x11}, /*Y INC*/
	{0x3820, 0x80}, /*Timing Reg20:Vflip*/
	{0x3821, 0x16}, /*Timing Reg21:Hmirror*/
	{0x3f00, 0x02}, /*PSRAM Ctrl0*/
	{0x3f01, 0xfc}, /*PSRAM Ctrl1*/
	{0x3f05, 0x10}, /*PSRAM Ctrl5*/
	{0x4600, 0x04}, /*VFIFO Ctrl0*/
	{0x4601, 0x00}, /*VFIFO Read ST High*/
	{0x4602, 0x78}, /*VFIFO Read ST Low*/
	{0x4837, 0x28}, /*MIPI PCLK PERIOD*/
	{0x5068, 0x00}, /*HSCALE_CTRL*/
	{0x506a, 0x00}, /*VSCALE_CTRL*/
	{0x5c00, 0x80}, /*PBLC CTRL00*/
	{0x5c01, 0x00}, /*PBLC CTRL01*/
	{0x5c02, 0x00}, /*PBLC CTRL02*/
	{0x5c03, 0x00}, /*PBLC CTRL03*/
	{0x5c04, 0x00}, /*PBLC CTRL04*/
	{0x5c08, 0x10}, /*PBLC CTRL08*/
	{0x6900, 0x61}, /*CADC CTRL00*/
};
#endif // end of Detroit

static struct msm_camera_i2c_reg_conf ov8825_reset_settings[] = {
	{0x0103, 0x01},
};

#ifdef DETROIT
static struct msm_camera_i2c_reg_conf ov8825_recommend_settings[] = {
	{0x3000, 0x16},
	{0x3001, 0x00},
	{0x3002, 0x6c},
	{0x3003, 0xce},
	{0x3004, 0xd4},
	{0x3005, 0x00},
	{0x3006, 0x10},
	{0x3007, 0x3b},
	{0x300d, 0x00},
	{0x301f, 0x09},
	{0x3010, 0x00},
	{0x3005, 0x00},
	{0x3006, 0x10},
//	{0x3011, 0x01},
	{0x3012, 0x80},
	{0x3013, 0x39},
	{0x3018, 0x00},
	{0x3104, 0x20},
	{0x3300, 0x00},
	{0x3500, 0x00},
	{0x3501, 0x4e},
	{0x3502, 0xa0},
	{0x3503, 0x07},
	{0x3509, 0x00},
	{0x350b, 0x1f},
	{0x3600, 0x07}, // 0x05->0x06 for 8825
	{0x3601, 0x33}, // 0x32->0x34 for 8825
	{0x3602, 0x42}, // 0x44->0x42 for 8825
	{0x3603, 0x5c},
	{0x3604, 0x98},
	{0x3605, 0xf5}, //0xe9-> 0xf5 for 8825
	{0x3609, 0xb4}, //0xb8->0xb4 for8825
	{0x360a, 0x7c}, //0xbc->0x7c for 8825
	{0x360b, 0xc9}, //0xb4->0xc9 for 8825
	{0x360c, 0x0b}, //0x0d-> 0x0b for 8825
       {0x3612, 0x00},//sophia wang 20120708
	{0x3613, 0x02},
	{0x3614, 0x0f},
	{0x3615, 0x00},
	{0x3616, 0x03},
       {0x3617, 0xa1},//sophia wang 20120708
	{0x3618, 0x00},
	{0x3619, 0x00},
       {0x361a, 0xb0}, //sophia, wang for VCM use, from 00->b0
       {0x361b, 0x04}, // sophia wang for VCM, from 00->04
	{0x3700, 0x10}, // sophia wang, for 8825
	{0x3701, 0x44},
	{0x3702, 0x28}, // sophia, for 8825 0x70-> 0x28
	{0x3703, 0x6c}, // sophia, for 8825 0x4f->0x6c
	{0x3704, 0x8d}, // sophia, for 8825 , 0x69-> 0x8d
	{0x3706, 0x27}, // sophia wang, 0x7b->0x27
	{0x3707, 0x63},
	{0x3708, 0x40}, //sophia wang, 0x85->0x40
	{0x3709, 0x20},// sophia wang, 0x40->0x20
	{0x370a, 0x12},
	{0x370b, 0x01},
	{0x370c, 0x50},
	{0x370d, 0x00},// sophia wang, 0x0c->0x00
	{0x370e, 0x00},
	{0x3711, 0x07},// sophia wang, 0x01->0x07, 8825
	{0x3712, 0x4e},// sophia wang, 0xcc->0x4e, 8825
	{0x3724, 0x00}, // add by sophia wang , 8825
	{0x3725, 0xd4}, //sophia wang 8825
	{0x3726, 0x00}, //sophia wang 8825
       {0x3727, 0xe1}, //sophia wang 8825
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0c},
	{0x3805, 0xdf},
	{0x3806, 0x09},
	{0x3807, 0x9b},
#if 0	
	{0x3808, 0x06},
	{0x3809, 0x68},
	{0x380a, 0x04},
	{0x380b, 0xce},	
	{0x380c, 0x0d},
	{0x380d, 0xbc},
	{0x380e, 0x04},
	{0x380f, 0xf0},
#endif
	{0x3810, 0x00},
	{0x3811, 0x04},// sophia wang 0x04->0x07
	{0x3812, 0x00},
	{0x3813, 0x00},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3816, 0x02},
	{0x3817, 0x40},
	{0x3818, 0x00},
	{0x3819, 0x40},
	{0x3820, 0x81},
	{0x3821, 0x17},
	{0x3d00, 0x00},
	{0x3d01, 0x00},
	{0x3d02, 0x00},
	{0x3d03, 0x00},
	{0x3d04, 0x00},
	{0x3d05, 0x00},
	{0x3d06, 0x00},
	{0x3d07, 0x00},
	{0x3d08, 0x00},
	{0x3d09, 0x00},
	{0x3d0a, 0x00},
	{0x3d0b, 0x00},
	{0x3d0c, 0x00},
	{0x3d0d, 0x00},
	{0x3d0e, 0x00},
	{0x3d0f, 0x00},
	{0x3d10, 0x00},
	{0x3d11, 0x00},
	{0x3d12, 0x00},
	{0x3d13, 0x00},
	{0x3d14, 0x00},
	{0x3d15, 0x00},
	{0x3d16, 0x00},
	{0x3d17, 0x00},
	{0x3d18, 0x00},
	{0x3d19, 0x00},
	{0x3d1a, 0x00},
	{0x3d1b, 0x00},
	{0x3d1c, 0x00},
	{0x3d1d, 0x00},
	{0x3d1e, 0x00},
	{0x3d1f, 0x00},
	{0x3d80, 0x00},
	{0x3d81, 0x00},
	{0x3d84, 0x00},
	{0x3f00, 0x00},
	{0x3f01, 0xfc},
	{0x3f05, 0x10},
	{0x3f06, 0x00},
	{0x3f07, 0x00},
	{0x4000, 0x29},
	{0x4001, 0x02},
	{0x4002, 0x45},
	{0x4003, 0x08},
	{0x4004, 0x04},
	{0x4005, 0x18},
	{0x4300, 0xff},
	{0x4303, 0x00},
	{0x4304, 0x08},
	{0x4307, 0x00},
	{0x4600, 0x04},
	{0x4601, 0x00},
	{0x4602, 0x30},
	{0x4800, 0x24}, //sophia wang, Qualcomm's suggestion
	{0x4801, 0x0f},
	{0x4837, 0x28},
	{0x4843, 0x02},
	{0x5000, 0x06},// [7]: enable lens shading
	{0x5001, 0x00},
	{0x5002, 0x00},
	{0x5068, 0x00},
	{0x506a, 0x00},
	{0x501f, 0x00},
	{0x5780, 0xfc}, //sophia wang, add, for 8825
	{0x5c00, 0x80},
	{0x5c01, 0x00},
	{0x5c02, 0x00},
	{0x5c03, 0x00},
	{0x5c04, 0x00},
	{0x5c05, 0x00},
	{0x5c06, 0x00},
	{0x5c07, 0x80},
	{0x5c08, 0x10},
	{0x6700, 0x05},
	{0x6701, 0x19},
	{0x6702, 0xfd},
	{0x6703, 0xd7}, //sophia wang 0xd1->0xd7
	{0x6704, 0xff},
	{0x6705, 0xff},
	{0x6800, 0x10},
	{0x6801, 0x02},
	{0x6802, 0x90},
	{0x6803, 0x10},
	{0x6804, 0x59},
	{0x6900, 0x61},
	{0x6901, 0x05},  //20120707, weichung. modify binning mode from sum(4) to avg.(5) to fix banding issue, 
	//{0x3612, 0x00},
	//{0x3617, 0xa1},
	//{0x3b1f, 0x00},
	//{0x3000, 0x12},
	//{0x3000, 0x16},
	//{0x3b1f, 0x00},
	{0x0100, 0x01},
	{0x5800, 0x16},
	{0x5801, 0x0b},
	{0x5802, 0x09},
	{0x5803, 0x09},
	{0x5804, 0x0b},
	{0x5805, 0x15},
	{0x5806, 0x07},
	{0x5807, 0x05},
	{0x5808, 0x03},
	{0x5809, 0x03},
	{0x580a, 0x05},
	{0x580b, 0x06},
	{0x580c, 0x05},
	{0x580d, 0x02},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x02},
	{0x5811, 0x05},
	{0x5812, 0x06},
	{0x5813, 0x02},
	{0x5814, 0x00},
	{0x5815, 0x00},
	{0x5816, 0x02},
	{0x5817, 0x05},
	{0x5818, 0x07},
	{0x5819, 0x05},
	{0x581a, 0x04},
	{0x581b, 0x03},
	{0x581c, 0x05},
	{0x581d, 0x06},
	{0x581e, 0x13},
	{0x581f, 0x0b},
	{0x5820, 0x09},
	{0x5821, 0x09},
	{0x5822, 0x0b},
	{0x5823, 0x16},
	{0x5824, 0x63},
	{0x5825, 0x23},
	{0x5826, 0x25},
	{0x5827, 0x23},
	{0x5828, 0x45},
	{0x5829, 0x23},
	{0x582a, 0x21},
	{0x582b, 0x41},
	{0x582c, 0x41},
	{0x582d, 0x05},
	{0x582e, 0x23},
	{0x582f, 0x41},
	{0x5830, 0x41},
	{0x5831, 0x41},
	{0x5832, 0x03},
	{0x5833, 0x25},
	{0x5834, 0x23},
	{0x5835, 0x21},
	{0x5836, 0x23},
	{0x5837, 0x05},
	{0x5838, 0x25},
	{0x5839, 0x43},
	{0x583a, 0x25},
	{0x583b, 0x23},
	{0x583c, 0x65},
	{0x583d, 0xcf},
	{0x5842, 0x00},
	{0x5843, 0xef},
	{0x5844, 0x01},
	{0x5845, 0x3f},
	{0x5846, 0x01},
	{0x5847, 0x3f},
	{0x5848, 0x00},
	{0x5849, 0xd5},
};


#else
static struct msm_camera_i2c_reg_conf ov8825_recommend_settings[] = {
	{0x3000, 0x16},
	{0x3001, 0x00},
	{0x3002, 0x6c},
	{0x300d, 0x00},
	{0x301f, 0x09},
	{0x3010, 0x00},
	{0x3018, 0x00},
	{0x3300, 0x00},
	{0x3500, 0x00},
	{0x3503, 0x07},
	{0x3509, 0x00},
	{0x3602, 0x42},
	{0x3603, 0x5c},
	{0x3604, 0x98},
	{0x3605, 0xf5},
	{0x3609, 0xb4},
	{0x360a, 0x7c},
	{0x360b, 0xc9},
	{0x360c, 0x0b},
	{0x3612, 0x00},
	{0x3613, 0x02},
	{0x3614, 0x0f},
	{0x3615, 0x00},
	{0x3616, 0x03},
	{0x3617, 0xa1},
	{0x3618, 0x00},
	{0x3619, 0x00},
	{0x361a, 0xB0},
	{0x361b, 0x04},
	{0x361c, 0x07},
	{0x3701, 0x44},
	{0x370b, 0x01},
	{0x370c, 0x50},
	{0x370d, 0x00},
	{0x3816, 0x02},
	{0x3817, 0x40},
	{0x3818, 0x00},
	{0x3819, 0x40},
	{0x3b1f, 0x00},
	{0x3d00, 0x00},
	{0x3d01, 0x00},
	{0x3d02, 0x00},
	{0x3d03, 0x00},
	{0x3d04, 0x00},
	{0x3d05, 0x00},
	{0x3d06, 0x00},
	{0x3d07, 0x00},
	{0x3d08, 0x00},
	{0x3d09, 0x00},
	{0x3d0a, 0x00},
	{0x3d0b, 0x00},
	{0x3d0c, 0x00},
	{0x3d0d, 0x00},
	{0x3d0e, 0x00},
	{0x3d0f, 0x00},
	{0x3d10, 0x00},
	{0x3d11, 0x00},
	{0x3d12, 0x00},
	{0x3d13, 0x00},
	{0x3d14, 0x00},
	{0x3d15, 0x00},
	{0x3d16, 0x00},
	{0x3d17, 0x00},
	{0x3d18, 0x00},
	{0x3d19, 0x00},
	{0x3d1a, 0x00},
	{0x3d1b, 0x00},
	{0x3d1c, 0x00},
	{0x3d1d, 0x00},
	{0x3d1e, 0x00},
	{0x3d1f, 0x00},
	{0x3d80, 0x00},
	{0x3d81, 0x00},
	{0x3d84, 0x00},
	{0x3f06, 0x00},
	{0x3f07, 0x00},
	{0x4000, 0x29},
	{0x4001, 0x02},
	{0x4002, 0x45},
	{0x4003, 0x08},
	{0x4004, 0x04},
	{0x4005, 0x18},
	{0x4300, 0xff},
	{0x4303, 0x00},
	{0x4304, 0x08},
	{0x4307, 0x00},
	{0x4800, 0x04},
	{0x4801, 0x0f},
	{0x4843, 0x02},
	{0x5000, 0x06},
	{0x5001, 0x00},
	{0x5002, 0x00},
	{0x501f, 0x00},
	{0x5780, 0xfc},
	{0x5c05, 0x00},
	{0x5c06, 0x00},
	{0x5c07, 0x80},
	{0x6700, 0x05},
	{0x6701, 0x19},
	{0x6702, 0xfd},
	{0x6703, 0xd7},
	{0x6704, 0xff},
	{0x6705, 0xff},
	{0x6800, 0x10},
	{0x6801, 0x02},
	{0x6802, 0x90},
	{0x6803, 0x10},
	{0x6804, 0x59},
	{0x6901, 0x04},
	{0x5800, 0x0f},
	{0x5801, 0x0d},
	{0x5802, 0x09},
	{0x5803, 0x0a},
	{0x5804, 0x0d},
	{0x5805, 0x14},
	{0x5806, 0x0a},
	{0x5807, 0x04},
	{0x5808, 0x03},
	{0x5809, 0x03},
	{0x580a, 0x05},
	{0x580b, 0x0a},
	{0x580c, 0x05},
	{0x580d, 0x02},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x05},
	{0x5812, 0x09},
	{0x5813, 0x03},
	{0x5814, 0x01},
	{0x5815, 0x01},
	{0x5816, 0x04},
	{0x5817, 0x09},
	{0x5818, 0x09},
	{0x5819, 0x08},
	{0x581a, 0x06},
	{0x581b, 0x06},
	{0x581c, 0x08},
	{0x581d, 0x06},
	{0x581e, 0x33},
	{0x581f, 0x11},
	{0x5820, 0x0e},
	{0x5821, 0x0f},
	{0x5822, 0x11},
	{0x5823, 0x3f},
	{0x5824, 0x08},
	{0x5825, 0x46},
	{0x5826, 0x46},
	{0x5827, 0x46},
	{0x5828, 0x46},
	{0x5829, 0x46},
	{0x582a, 0x42},
	{0x582b, 0x42},
	{0x582c, 0x44},
	{0x582d, 0x46},
	{0x582e, 0x46},
	{0x582f, 0x60},
	{0x5830, 0x62},
	{0x5831, 0x42},
	{0x5832, 0x46},
	{0x5833, 0x46},
	{0x5834, 0x44},
	{0x5835, 0x44},
	{0x5836, 0x44},
	{0x5837, 0x48},
	{0x5838, 0x28},
	{0x5839, 0x46},
	{0x583a, 0x48},
	{0x583b, 0x68},
	{0x583c, 0x28},
	{0x583d, 0xae},
	{0x5842, 0x00},
	{0x5843, 0xef},
	{0x5844, 0x01},
	{0x5845, 0x3f},
	{0x5846, 0x01},
	{0x5847, 0x3f},
	{0x5848, 0x00},
	{0x5849, 0xd5},
	{0x3503, 0x07},
	{0x3500, 0x00},
	{0x3501, 0x27},
	{0x3502, 0x00},
	{0x350b, 0xff},
	{0x3400, 0x04},
	{0x3401, 0x00},
	{0x3402, 0x04},
	{0x3403, 0x00},
	{0x3404, 0x04},
	{0x3405, 0x00},
	{0x3406, 0x01},
	{0x5001, 0x01},
	{0x5000, 0x86},/* enable lens compensation and dpc */
	/* LENC setting 70% */
	{0x5800, 0x21},
	{0x5801, 0x10},
	{0x5802, 0x09},
	{0x5803, 0x0a},
	{0x5804, 0x0f},
	{0x5805, 0x23},
	{0x5806, 0x08},
	{0x5807, 0x04},
	{0x5808, 0x04},
	{0x5809, 0x04},
	{0x580a, 0x04},
	{0x580b, 0x0a},
	{0x580c, 0x04},
	{0x580d, 0x02},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x06},
	{0x5812, 0x05},
	{0x5813, 0x02},
	{0x5814, 0x00},
	{0x5815, 0x00},
	{0x5816, 0x03},
	{0x5817, 0x06},
	{0x5818, 0x09},
	{0x5819, 0x05},
	{0x581a, 0x04},
	{0x581b, 0x04},
	{0x581c, 0x05},
	{0x581d, 0x0a},
	{0x581e, 0x24},
	{0x581f, 0x11},
	{0x5820, 0x0a},
	{0x5821, 0x0a},
	{0x5822, 0x10},
	{0x5823, 0x27},
	{0x5824, 0x2a},
	{0x5825, 0x58},
	{0x5826, 0x28},
	{0x5827, 0x28},
	{0x5828, 0x28},
	{0x5829, 0x28},
	{0x582a, 0x46},
	{0x582b, 0x44},
	{0x582c, 0x46},
	{0x582d, 0x46},
	{0x582e, 0x28},
	{0x582f, 0x62},
	{0x5830, 0x60},
	{0x5831, 0x42},
	{0x5832, 0x28},
	{0x5833, 0x48},
	{0x5834, 0x46},
	{0x5835, 0x46},
	{0x5836, 0x26},
	{0x5837, 0x46},
	{0x5838, 0x28},
	{0x5839, 0x48},
	{0x583a, 0x28},
	{0x583b, 0x28},
	{0x583c, 0x26},
	{0x583d, 0x9d},
};
#endif
#endif

static struct v4l2_subdev_info ov8825_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array ov8825_init_conf[] = {
	{&ov8825_reset_settings[0],
	ARRAY_SIZE(ov8825_reset_settings), 50, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov8825_recommend_settings[0],
	ARRAY_SIZE(ov8825_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array ov8825_confs[] = {
#ifdef MIPI_2Lane
	{&ov8825_snap_settings[0],
	ARRAY_SIZE(ov8825_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov8825_prev_settings[0],
	ARRAY_SIZE(ov8825_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
#else
	{&ov8825_snap_settings[0],
	ARRAY_SIZE(ov8825_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov8825_prev_settings[0],
	ARRAY_SIZE(ov8825_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
       {&ov8825_1080p_settings[0],
	ARRAY_SIZE(ov8825_1080p_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},	
#endif
};

static struct msm_sensor_output_info_t ov8825_dimensions[] = {

       // 3264*2448 @24 fps, snap        
       {
               .x_output = 0xCC0,
               .y_output = 0x990,
               .line_length_pclk = 0xd20, //   3360
               .frame_length_lines = 0x9B0, // 2480
               .vt_pixel_clk = 200000000,
               .op_pixel_clk = 266000000,
               .binning_factor = 1,
       },

       //1632*1224 @30 fps, prev
       {
               .x_output = 1632,  //0x660
               .y_output = 1224,  //0x4c8
               .line_length_pclk = 0xdbc, // 3516
               .frame_length_lines = 0x4f0, //1264
               .vt_pixel_clk = 133400000,
               .op_pixel_clk = 200000000,
               .binning_factor = 1,
       },
   #if 0            
       // 3264*2448 @24 fps, prev        
       {
               .x_output = 0xCC0,
               .y_output = 0x990,
               .line_length_pclk = 0xd20, //   3360
               .frame_length_lines = 0x9B0, // 2480
               .vt_pixel_clk = 200000000,
               .op_pixel_clk = 266000000,
               .binning_factor = 1,
       },
       #endif
       
	// 1920X1080, 30 fps, camcorder
	{
		.x_output = 1920,/*0x780*/
		.y_output = 1088,/*0x438*/
		.line_length_pclk = 3568,/*0xdf0*/
		.frame_length_lines = 1868,/*0x74c*/
		//.vt_pixel_clk = 133400000,
                .vt_pixel_clk = 200000000,
		.op_pixel_clk = 266000000,
              //.op_pixel_clk = 200000000,
		.binning_factor = 1,
	},       

};


static struct msm_sensor_output_reg_addr_t ov8825_reg_addr = {
	.x_output = 0x3808,
	.y_output = 0x380a,
	.line_length_pclk = 0x380c,
	.frame_length_lines = 0x380e,
};

static struct msm_sensor_id_info_t ov8825_id_info = {
	.sensor_id_reg_addr = 0x300A,
	.sensor_id = 0x8825,
};

static struct msm_sensor_exp_gain_info_t ov8825_exp_gain_info = {
	.coarse_int_time_addr = 0x3501,
	.global_gain_addr = 0x350A,
	.vert_offset = 6,
};

/********************************************
 * index: index of otp group. (0, 1, 2)
 * return value:
 *     0, group index is empty
 *     1, group index has invalid data
 *     2, group index has valid data
 **********************************************/
uint16_t ov8825_check_otp_wb(struct msm_sensor_ctrl_t *s_ctrl, uint16_t index)
{
	uint16_t temp, i;
	uint16_t address;
       uint16_t bank = 0;
	/* clear otp buffer */

      if(index <=1)
      	{
           bank = 0x0;
           
      	}
      else
    	{
	  bank = 0x1;
       }

      
	/* select otp bank 0 */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d84,(0x08|bank),
			MSM_CAMERA_I2C_BYTE_DATA);
       
	/* load otp into buffer */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01,
			MSM_CAMERA_I2C_BYTE_DATA);
	msleep(10);

    	/* disable otp read */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x00,
			MSM_CAMERA_I2C_BYTE_DATA);

       if(index <= 1)
       {
	/* read from group [index] */
	address = 0x3d05 + index * 17;
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, address, &temp,
			MSM_CAMERA_I2C_BYTE_DATA);

       }else
       {
	/* read from group [index] */
	address = 0x3d07;
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, address, &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	
       }

    
	/* clear otp buffer */
	for (i = 0; i < 32; i++) {
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, (0x3d00+i),
				0x00, MSM_CAMERA_I2C_BYTE_DATA);
	}

       printk("%s, index = %d, bank = 0x%x, address:0x%x, temp = 0x%x\n", __func__, index, bank, address, temp);
	if (!temp)
		return 0;
	else if ((!(temp & 0x80)) && (temp & 0x7f))
		return 2;
	else
		return 1;
}

void ov8825_read_otp_wb(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t index, struct otp_struct *potp)
{
	uint16_t temp, i;
	uint16_t address;
       uint16_t bank = 0;

       if(index <=1)
       {
		bank = 0;
		address = 0x3d05 + index * 17;
       }
       else if (index ==2)
       {
		bank = 1;
		address = 0x3d07;
       }
       else
       {
		printk("%s, wrong index, index:%d\n", __func__, index);
		return;
       }

if(index == 0 ||index == 2)
{
	/* select otp bank 0 */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, (0x08|bank),
			MSM_CAMERA_I2C_BYTE_DATA);

	/* load otp data into buffer */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01,
			MSM_CAMERA_I2C_BYTE_DATA);

	msleep(10);
	
	/* disable otp read */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x00,
			MSM_CAMERA_I2C_BYTE_DATA);

	/* read otp data from 0x3d00 - 0x3d1f*/
	
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, address, &temp,
			MSM_CAMERA_I2C_BYTE_DATA);

	potp->module_integrator_id = temp;
	potp->customer_id = temp & 0x7f;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+1), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->lens_id = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+2), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->year = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+3), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->mouth = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+4), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->date= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+5), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->hour= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+6), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->minuite= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+7), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->second = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+8), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->rg_ratio_d65 = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+9), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->bg_ratio_d65= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+10), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->gb_gr_d65= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+11), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->rg_ratio_cwf= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+12), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->bg_ratio_cwf = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+13), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->gb_gr_cwf = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+14), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->rg_ratio_u30= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+15), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->bg_ratio_u30 = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+16), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->gb_gr_u30 = temp;


	/* clear otp buffer */
	for (i = 0; i < 32; i++)
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, (0x3d00+i),
				0x00, MSM_CAMERA_I2C_BYTE_DATA);
}
else
{
	/* select otp bank 0 */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, (0x08|0x0),
			MSM_CAMERA_I2C_BYTE_DATA);

	/* load otp data into buffer */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01,
			MSM_CAMERA_I2C_BYTE_DATA);

	msleep(10);
	
	/* disable otp read */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x00,
			MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_i2c_read(s_ctrl->sensor_i2c_client, address, &temp,
			MSM_CAMERA_I2C_BYTE_DATA);

	potp->module_integrator_id = temp;
	potp->customer_id = temp & 0x7f;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+1), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->lens_id = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+2), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->year = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+3), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->mouth = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+4), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->date= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+5), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->hour= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+6), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->minuite= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+7), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->second = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+8), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->rg_ratio_d65 = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+9), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->bg_ratio_d65= temp;


       // for index 1, the other calibration data is in bank 1
	/* clear otp buffer */
	for (i = 0; i < 32; i++)
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, (0x3d00+i),
				0x00, MSM_CAMERA_I2C_BYTE_DATA);

       bank = bank+1;
       address = 0x3D00; //the rest part of awb_group1, the starting address is 0x3d00, sophia, 2013,01,29
     	/* select otp bank 1 */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0x08|bank,
			MSM_CAMERA_I2C_BYTE_DATA);

	/* load otp data into buffer */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01,
			MSM_CAMERA_I2C_BYTE_DATA);

	msleep(10);
	
	/* disable otp read */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x00,
			MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_i2c_read(s_ctrl->sensor_i2c_client, address, &temp,
			MSM_CAMERA_I2C_BYTE_DATA);

	potp->gb_gr_d65= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+1), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->rg_ratio_cwf= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+2), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->bg_ratio_cwf = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+3), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->gb_gr_cwf = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+4), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->rg_ratio_u30= temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+5), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->bg_ratio_u30 = temp;

	msm_camera_i2c_read(s_ctrl->sensor_i2c_client, (address+6), &temp,
			MSM_CAMERA_I2C_BYTE_DATA);
	potp->gb_gr_u30 = temp;


	/* clear otp buffer */
	for (i = 0; i < 32; i++)
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, (0x3d00+i),
				0x00, MSM_CAMERA_I2C_BYTE_DATA);
	
	printk("%s, index = %d\n", __func__, index);
}
	CDBG("%s customer_id  = 0x%02x\r\n", __func__, potp->customer_id);
	CDBG("%s lens_id      = 0x%02x\r\n", __func__, potp->lens_id);
	CDBG("%s year  = 0x%02x\r\n", __func__, potp->year);
	CDBG("%s mouth  = 0x%02x\r\n", __func__, potp->mouth);
	CDBG("%s date  = 0x%02x\r\n", __func__, potp->date);
	CDBG("%s hour = 0x%02x\r\n", __func__, potp->hour);
	CDBG("%s minuite  = 0x%02x\r\n", __func__, potp->minuite);
	CDBG("%s second  = 0x%02x\r\n", __func__, potp->second);
	CDBG("%s rg_ratio_d65     = 0x%02x\r\n", __func__, potp->rg_ratio_d65);
	CDBG("%s bg_ratio_d65     = 0x%02x\r\n", __func__, potp->bg_ratio_d65);
	CDBG("%s gb_gr_d65     = 0x%02x\r\n", __func__, potp->gb_gr_d65);
	CDBG("%s rg_ratio_cwf     = 0x%02x\r\n", __func__, potp->rg_ratio_cwf);
	CDBG("%s bg_ratio_cwf     = 0x%02x\r\n", __func__, potp->bg_ratio_cwf);
	CDBG("%s gb_gr_cwf     = 0x%02x\r\n", __func__, potp->gb_gr_cwf);
	CDBG("%s rg_ratio_u30     = 0x%02x\r\n", __func__, potp->rg_ratio_u30);
	CDBG("%s bg_ratio_u30     = 0x%02x\r\n", __func__, potp->bg_ratio_u30);
	CDBG("%s gb_gr_u30     = 0x%02x\r\n", __func__, potp->gb_gr_u30);

}


#if 0
/**********************************************
 * r_gain, sensor red gain of AWB, 0x400 =1
 * g_gain, sensor green gain of AWB, 0x400 =1
 * b_gain, sensor blue gain of AWB, 0x400 =1
 ***********************************************/
void ov8825_update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t r_gain, uint16_t g_gain, uint16_t b_gain)
{
	CDBG("%s r_gain = 0x%04x\r\n", __func__, r_gain);
	CDBG("%s g_gain = 0x%04x\r\n", __func__, g_gain);
	CDBG("%s b_gain = 0x%04x\r\n", __func__, b_gain);
	if (r_gain > 0x400) {
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x5186,
				(r_gain>>8), MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x5187,
				(r_gain&0xff), MSM_CAMERA_I2C_BYTE_DATA);
	}
	if (g_gain > 0x400) {
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x5188,
				(g_gain>>8), MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x5189,
				(g_gain&0xff), MSM_CAMERA_I2C_BYTE_DATA);
	}
	if (b_gain > 0x400) {
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x518a,
				(b_gain>>8), MSM_CAMERA_I2C_BYTE_DATA);
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client, 0x518b,
				(b_gain&0xff), MSM_CAMERA_I2C_BYTE_DATA);
	}
}
#endif


static int32_t ov8825_write_shading_data(struct msm_sensor_ctrl_t* s_ctrl)
{
       uint8_t rc = 0;

       printk("%s, enter\n", __func__);
       
	if(initOtp == false)
	{
		printk("%s, opt is not initialized\n", __func__);
	}

	rc = msm_camera_i2c_write_seq(
		s_ctrl->sensor_i2c_client,
		0x5800,
		&ov8825_lsc_D65_data[0], LSC_REG_NUM);
		
	
		
	if( rc == 0)
		 rc = msm_camera_i2c_write(
		  	s_ctrl->sensor_i2c_client,
		  	0x5000,
		  	0x86, MSM_CAMERA_I2C_BYTE_DATA); // enable shading data
       else
       	  printk("%s, there is error while write shading register\n", __func__);
      

       return rc ;
}


static int32_t  ov8825_read_otp(struct msm_sensor_ctrl_t *s_ctrl, int bank, uint16_t addr, uint8_t *buffer, uint8_t buffsize)
{
   uint8_t tempBank, rc, readArray[LSC_REG_NUM];
   uint16_t tempaddr , readValue;
   int i = 0, j=0, k = 0;

    tempaddr = addr;
    
    for(i = 0; i<=(buffsize/32); i++ )
    {
	    j = 0;
	    tempBank = bank | 0x08;

	    printk("%s, tempBank:ox%x, tempaddr:0x%x\n", __func__, tempBank, tempaddr);
	    
	    rc = msm_camera_i2c_write(
		  s_ctrl->sensor_i2c_client,
		  0x3d84,
		  tempBank, MSM_CAMERA_I2C_BYTE_DATA);

    	    if( rc != 0)
    	    {
            	printk("%s, write 0x3d84 0x08 failed\n", __func__);
            	return rc;
    	     }

     	     rc = msm_camera_i2c_write(
		    s_ctrl->sensor_i2c_client,
		    0x3d81,
		    0x01, MSM_CAMERA_I2C_BYTE_DATA); 

    	    if( rc != 0)
    	    {
              printk("%s, write 0x3d81 0x01 failed\n", __func__);
              return rc;
    	    }
   
    	    msleep(10);

    	    rc = msm_camera_i2c_write(
		    s_ctrl->sensor_i2c_client,
		    0x3d81,
		    0x00, MSM_CAMERA_I2C_BYTE_DATA); 

    	    if( rc != 0)
    	    {
              printk("%s, write 0x3d81 0x00 failed\n", __func__);
              return rc;
    	    }

           // each bank include 32 registers

           if(buffsize <32)
           {
           	for(j =0; j < buffsize; j++)
           	{
    	    		rc &= msm_camera_i2c_read(
					s_ctrl->sensor_i2c_client,
					tempaddr, &readValue,
					MSM_CAMERA_I2C_BYTE_DATA);

     //   	       printk("%s, buffsize:%d, readValue:0x%x\n", __func__, buffsize, readValue);
        	    
        			*(buffer+j) = (uint8_t)readValue;

        		          tempaddr++;

           	    } // end of for
           	    
           	    goto end;
           	}
    	       else
    	       {
      			    rc &= msm_camera_i2c_read_seq(
					s_ctrl->sensor_i2c_client,
					tempaddr, &readArray[0+i*31],
					31);
    	         }

           // clear 0x3d00~0x3d1f
	    tempaddr = 0x3d00;
	    for( k =0; k< 32 ; k++)
	    {
    	    	       rc = msm_camera_i2c_write(
		              s_ctrl->sensor_i2c_client,
		              tempaddr,
		               0x00, MSM_CAMERA_I2C_BYTE_DATA); 
                     tempaddr++;
	    }

	    tempaddr = 0x3d00;
	    bank++;
	    msleep(20);	    
    }

    if(buffsize >= 32)
    {
		for(k=0; k< buffsize; k++)
		{
 			buffer[k] = readArray[k];
 //			printk("%s, buffer[%d]: 0x%x\n", __func__, k, buffer[k]);
		}
    }

    
end:
    if(rc)
    {
        printk("%s, there is i2c error while read otp data\n", __func__);
        return -1;
    }
    else
    	return 0;
}

static int32_t  ov8825_read_otp_lsc(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint8_t rc = 0, i,  shadingflag;
    uint16_t bank = 0xFFFF ;

   if(initOtp == true)
   return 0;

    initOtp = true;

   /*sophia, wang reading shading data {*/
   for(i = 0; i<2; i++)
   {
       // check which bank has valid LenC data, bank2, bank 4
   	ov8825_read_otp(s_ctrl, 2+2*i, 0x3d00, &shadingflag, 1);
       
       printk("%s, i = %d, flag:0x%x\n", __func__, i, shadingflag);

   	if((shadingflag >> 6) == 0x1)
   	{
            printk("%s, the shading data is stored in bank %d, flag:0x%x\n ", __func__, (2+2*i), shadingflag);
            bank = 2+2*i;
            break;
   	}
   }

   if( bank == 0xFFFF)
   {
		printk("%s, there is no valide shading data in otp, flag:0x%x\n", __func__, shadingflag);
		return -2;
   }

   // shading data is in bank 1~6, check bit is bank1,3,5
   rc = ov8825_read_otp(s_ctrl, bank, 0x3d01, ov8825_lsc_D65_data, LSC_REG_NUM);
   /*sophia, wang reading shading data }*/


   for( i =0; i<LSC_REG_NUM; i++ )
   {
		CDBG("%s, ov8820_lsc_D65_data_eeprom[%d]:0x%x\n", __func__, i, ov8825_lsc_D65_data[i]);
   }

   
   return rc;
    
}

static int32_t  ov8825_read_otp_af(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint8_t rc = 0, i,  afflag;
    uint16_t  address=0xFFFF;

   for(i = 0; i<3; i++)
   {
       // check which address of bank 6 has valid af calib data // ox3d00, 3d05, 3d0a
   	ov8825_read_otp(s_ctrl, 6, 0x3d00+i*5, &afflag, 1);
       
       printk("%s, i = %d, flag:0x%x\n", __func__, i, afflag);

   	if(afflag == 0x05)
   	{
            address = 0x3d00+5*i;
            printk("%s, the af data is stored in address 0x%x, flag:0x%x\n ", __func__, address,afflag);

            break;
   	}
   }

   if( address == 0xFFFF)
   {
		printk("%s, there is no valide af data in otp, flag:0x%x\n", __func__, afflag);
		return -2;
   }

    rc = ov8825_read_otp(s_ctrl, 6, address+1, &(st_ov8825_otp.ov8825_af_otp[0]), 4);

    for(i = 0; i<4; i++)
    {

		CDBG("%s, ov8825_af_otp[%d]:%x\n", __func__, i, st_ov8825_otp.ov8825_af_otp[i]);		
    }

   
    return rc;


}

/**************************************************
 * call this function after OV8825 initialization
 * return value:
 *     0, update success
 *     1, no OTP
 ***************************************************/
uint8_t ov8825_update_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t i;
	uint16_t otp_index;
	uint16_t temp;
//	uint16_t r_gain, g_gain, b_gain, g_gain_r, g_gain_b;

	/* R/G and B/G of current camera module is read out from sensor OTP */
	/* check first OTP with valid data */
       msm_camera_i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x0100,
		               0x01, MSM_CAMERA_I2C_BYTE_DATA); 

	for (i = 0; i < 3; i++) {
		temp = ov8825_check_otp_wb(s_ctrl, i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i == 3) {
		/* no valid wb OTP data */
		CDBG("no valid wb OTP data\r\n");
		return 1;
	}
	ov8825_read_otp_wb(s_ctrl, otp_index, &st_ov8825_otp);
	ov8825_read_otp_lsc(s_ctrl);
	ov8825_read_otp_af(s_ctrl);

	return st_ov8825_otp.module_integrator_id;
#if 0	
	/* calculate g_gain */
	/* 0x400 = 1x gain */
	if (st_ov8825_otp.bg_ratio < OV8825_BG_RATIO_TYPICAL_VALUE) {
		if (st_ov8825_otp.rg_ratio < OV8825_RG_RATIO_TYPICAL_VALUE) {
			g_gain = 0x400;
			b_gain = 0x400 *
				OV8825_BG_RATIO_TYPICAL_VALUE /
				st_ov8825_otp.bg_ratio;
			r_gain = 0x400 *
				OV8825_RG_RATIO_TYPICAL_VALUE /
				st_ov8825_otp.rg_ratio;
		} else {
			r_gain = 0x400;
			g_gain = 0x400 *
				st_ov8825_otp.rg_ratio /
				OV8825_RG_RATIO_TYPICAL_VALUE;
			b_gain = g_gain *
				OV8825_BG_RATIO_TYPICAL_VALUE /
				st_ov8825_otp.bg_ratio;
		}
	} else {
		if (st_ov8825_otp.rg_ratio < OV8825_RG_RATIO_TYPICAL_VALUE) {
			b_gain = 0x400;
			g_gain = 0x400 *
				st_ov8825_otp.bg_ratio /
				OV8825_BG_RATIO_TYPICAL_VALUE;
			r_gain = g_gain *
				OV8825_RG_RATIO_TYPICAL_VALUE /
				st_ov8825_otp.rg_ratio;
		} else {
			g_gain_b = 0x400 *
				st_ov8825_otp.bg_ratio /
				OV8825_BG_RATIO_TYPICAL_VALUE;
			g_gain_r = 0x400 *
				st_ov8825_otp.rg_ratio /
				OV8825_RG_RATIO_TYPICAL_VALUE;
			if (g_gain_b > g_gain_r) {
				b_gain = 0x400;
				g_gain = g_gain_b;
				r_gain = g_gain *
					OV8825_RG_RATIO_TYPICAL_VALUE /
					st_ov8825_otp.rg_ratio;
			} else {
				r_gain = 0x400;
				g_gain = g_gain_r;
				b_gain = g_gain *
					OV8825_BG_RATIO_TYPICAL_VALUE /
					st_ov8825_otp.bg_ratio;
			}
		}
	}
	
	ov8825_update_awb_gain(s_ctrl, r_gain, g_gain, b_gain);
	#endif
	return 0;
}
//#endif //end of ifdef OTP

static int32_t ov8825_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines, offset;
	uint8_t int_time[3];

	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;

//	pr_err("ov8825_write_exp_gain: %d  frame_lenght_linegs:%d %d\n", fl_lines, s_ctrl->curr_frame_length_lines, s_ctrl->fps_divider);
	
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;
	CDBG("ov8825_write_exp_gain: %d %d %d\n", fl_lines, gain, line);
//       pr_err("ov8825_write_exp_gain: %d %d %d\n", fl_lines, gain, line);
	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);

	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
		MSM_CAMERA_I2C_WORD_DATA);

	int_time[0] = line >> 12;
	int_time[1] = line >> 4;
	int_time[2] = line << 4;
	
	msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr-1,
		&int_time[0], 3);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
		MSM_CAMERA_I2C_WORD_DATA);

	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}

static const struct i2c_device_id ov8825_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&ov8825_s_ctrl},
	{ }
};

int32_t ov8825_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *info = NULL;

	info = s_ctrl->sensordata;
//	gpio_direction_output(info->sensor_pwd, 0);
//	gpio_direction_output(info->sensor_reset, 0);
//	usleep_range(10000, 11000);
	rc = msm_sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s: msm_sensor_power_up failed\n", __func__);
		return rc;
	}
	/* turn on ldo and vreg */
//	gpio_direction_output(info->sensor_pwd, 1);
//	msleep(20); sophia wang ++, remove which power sequence is not in this function
//	gpio_direction_output(info->sensor_reset, 1);
//	msleep(40);
	return rc;
}

static struct i2c_driver ov8825_i2c_driver = {
	.id_table = ov8825_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8825_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static enum msm_camera_vreg_name_t ov8825_veg_seq_evt0[] = {
	CAM_VIO,
	CAM_VANA,	
	CAM_VDIG,
//	CAM_VAF,
};

static enum msm_camera_vreg_name_t ov8825_veg_seq_evt1[] = {
	CAM_VIO,
	CAM_VANA_EXT,// sophia wang, 20130115, from evt1-1 hw wil use gpio 35 instead of l9	
	CAM_VDIG,
	//CAM_VAF,
};

static enum msm_camera_vreg_name_t ov8825_veg_seq[] = {
	CAM_VIO,
	CAM_VANA_EXT,// sophia wang, 20130115, from evt1-1 hw wil use gpio 35 instead of l9	
	//CAM_VDIG, // sophia wang, 20130115, from evt2 hw will use internel DVDD instead of externel DVDD, we don't need to require l12
	//CAM_VAF,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&ov8825_i2c_driver);
}

static struct v4l2_subdev_core_ops ov8825_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ov8825_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ov8825_subdev_ops = {
	.core = &ov8825_subdev_core_ops,
	.video  = &ov8825_subdev_video_ops,
};

int32_t ov8825_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

       pr_err("%s, update_type:%d, res:%d\n", __func__, update_type, res);
	if (update_type == MSM_SENSOR_REG_INIT) {
		CDBG("Register INIT\n");
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
		msm_sensor_enable_debugfs(s_ctrl);
		msm_sensor_write_init_settings(s_ctrl);
		CDBG("Update OTP\n");

		// sophia wang++, 20130218, remove this sleep, previous, the codes follow this sleep is for otp reading
		// now the otp reading is just once when probe.
		// msleep(66); 

              ov8825_write_shading_data(s_ctrl);

              
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		CDBG("PERIODIC : %d\n", res);

		msm_sensor_write_res_settings(
s_ctrl, res);

		msleep(30);

      		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);

	}
	return rc;
}

int32_t ov8825_get_calib_params(struct msm_sensor_ctrl_t *s_ctrl, struct otp_params* otp_params)	
{
	int rc = 0;

	pr_err("%s, enter\n", __func__);
	
	otp_params->size = sizeof(struct otp_struct);
	if (copy_to_user((void *)otp_params->otp_data,
		&st_ov8825_otp,
		sizeof(struct otp_struct)))
		rc = -EFAULT;

	pr_err("%s, rc:%d, size = %d\n", __func__, rc, otp_params->size);
	return rc;

}

static struct msm_sensor_fn_t ov8825_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = ov8825_write_exp_gain,
	.sensor_write_snapshot_exp_gain = ov8825_write_exp_gain,
	//.sensor_csi_setting = ov8825_sensor_setting,
       .sensor_setting = ov8825_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov8825_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params, //20121112, sophia
	.sensor_read_otp_mid = ov8825_update_otp,
	.sensor_get_calib_params = ov8825_get_calib_params,
};

static struct msm_sensor_reg_t ov8825_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = ov8825_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(ov8825_start_settings),
	.stop_stream_conf = ov8825_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(ov8825_stop_settings),
	.group_hold_on_conf = ov8825_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(ov8825_groupon_settings),
	.group_hold_off_conf = ov8825_groupoff_settings,
	.group_hold_off_conf_size =	ARRAY_SIZE(ov8825_groupoff_settings),
	.init_settings = &ov8825_init_conf[0],
	.init_size = ARRAY_SIZE(ov8825_init_conf),
	.mode_settings = &ov8825_confs[0],
	.output_settings = &ov8825_dimensions[0],
	.num_conf = ARRAY_SIZE(ov8825_confs),
};

static struct msm_sensor_ctrl_t ov8825_s_ctrl = {
	.msm_sensor_reg = &ov8825_regs,
	.sensor_i2c_client = &ov8825_sensor_i2c_client,
	.sensor_i2c_addr = 0x6C,
	.vreg_seq_evt0 = ov8825_veg_seq_evt0,
	.num_vreg_seq_evt0 = ARRAY_SIZE(ov8825_veg_seq_evt0),
	.vreg_seq_evt1 = ov8825_veg_seq_evt1,
	.num_vreg_seq_evt1 = ARRAY_SIZE(ov8825_veg_seq_evt1),
	.vreg_seq = ov8825_veg_seq,
	.num_vreg_seq = ARRAY_SIZE(ov8825_veg_seq),
	.sensor_output_reg_addr = &ov8825_reg_addr,
	.sensor_id_info = &ov8825_id_info,
	.sensor_exp_gain_info = &ov8825_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex = &ov8825_mut,
	.sensor_i2c_driver = &ov8825_i2c_driver,
	.sensor_v4l2_subdev_info = ov8825_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8825_subdev_info),
	.sensor_v4l2_subdev_ops = &ov8825_subdev_ops,
	.func_tbl = &ov8825_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
	.otp_info = &st_ov8825_otp,
	
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Omnivison 8MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
