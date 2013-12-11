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
#define IMX091_SENSOR_NAME "imx091"
#define PLATFORM_DRIVER_NAME "msm_camera_imx091"
#define imx091_obj imx091_##obj

#define MSG2(format, arg...)  printk(KERN_INFO "[CAM]" format "\n", ## arg)


DEFINE_MUTEX(imx091_mut);
static struct msm_sensor_ctrl_t imx091_s_ctrl;

static struct otp_struct_imx091 st_imx091_otp;

static struct msm_camera_i2c_reg_conf imx091_start_settings[] = {
  {0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf imx091_stop_settings[] = {
  {0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf imx091_groupon_settings[] = {
  {0x0104, 0x01},
};

static struct msm_camera_i2c_reg_conf imx091_groupoff_settings[] = {
  {0x0104, 0x00},
};

static struct msm_camera_i2c_reg_conf imx091_prev_settings[] = {
  /* 30fps 1/2 * 1/2 */
  /* PLL setting */
  {0x0305, 0x02}, /* pre_pll_clk_div[7:0] */
  {0x0307, 0x2F},/* pll_multiplier[7:0] */
  {0x30A4, 0x02},
  {0x303C, 0x4B},
  /* mode setting */
  {0x0340, 0x06}, /* frame_length_lines[15:8] */
  {0x0341, 0x5A}, /* frame_length_lines[7:0] */
  {0x0342, 0x12},/* line_length_pck[15:8] */
  {0x0343, 0x0C}, /* line_length_pck[7:0] */
  {0x0344, 0x00}, /* x_addr_start[15:8] */
  {0x0345, 0x08}, /* x_addr_start[7:0] */
  {0x0346, 0x00}, /* y_addr_start[15:8] */
  {0x0347, 0x30}, /* y_addr_start[7:0] */
  {0x0348, 0x10}, /* x_addr_end[15:8] */
  {0x0349, 0x77}, /* x_addr_end[7:0] */
  {0x034A, 0x0C}, /* y_addr_end[15:8] */
  {0x034B, 0x5F}, /* y_addr_end[7:0] */
  {0x034C, 0x08},/* x_output_size[15:8] */
  {0x034D, 0x38}, /* x_output_size[7:0] */
  {0x034E, 0x06}, /* y_output_size[15:8] */
  {0x034F, 0x18}, /* y_output_size[7:0] */
  {0x0381, 0x01}, /* x_even_inc[3:0] */
  {0x0383, 0x03}, /* x_odd_inc[3:0] */
  {0x0385, 0x01}, /* y_even_inc[7:0] */
  {0x0387, 0x03}, /* y_odd_inc[7:0] */
  {0x3040, 0x08},
  {0x3041, 0x97},
  {0x3048, 0x01},
  {0x3064, 0x12},
  {0x309B, 0x28},
  {0x309E, 0x00},
  {0x30D5, 0x09},
  {0x30D6, 0x01},
  {0x30D7, 0x01},
  {0x30D8, 0x64},
  {0x30D9, 0x89},
  {0x30DE, 0x02},
  {0x3102, 0x10},
  {0x3103, 0x44},
  {0x3104, 0x40},
  {0x3105, 0x00},
  {0x3106, 0x0D},
  {0x3107, 0x01},
  {0x310A, 0x0A},
  {0x315C, 0x99},
  {0x315D, 0x98},
  {0x316E, 0x9A},
  {0x316F, 0x99},
  {0x3318, 0x73},
};

static struct msm_camera_i2c_reg_conf imx091_snap_settings[] = {
  /* full size */
  /* PLL setting */
  {0x0305, 0x02}, /* pre_pll_clk_div[7:0] */
  {0x0307, 0x2B}, /* pll_multiplier[7:0] */
  {0x30A4, 0x02},
  {0x303C, 0x4B},
  /* mode setting */
  {0x0340, 0x0C}, /* frame_length_lines[15:8] */
  {0x0341, 0x8C}, /* frame_length_lines[7:0] */
  {0x0342, 0x12}, /* line_length_pck[15:8] */
  {0x0343, 0x0C}, /* line_length_pck[7:0] */
  {0x0344, 0x00}, /* x_addr_start[15:8] */
  {0x0345, 0x08}, /* x_addr_start[7:0] */
  {0x0346, 0x00}, /* y_addr_start[15:8] */
  {0x0347, 0x30}, /* y_addr_start[7:0] */
  {0x0348, 0x10}, /* x_addr_end[15:8] */
  {0x0349, 0x77}, /* x_addr_end[7:0] */
  {0x034A, 0x0C}, /* y_addr_end[15:8] */
  {0x034B, 0x5F}, /* y_addr_end[7:0] */
  {0x034C, 0x10}, /* x_output_size[15:8] */
  {0x034D, 0x70}, /* x_output_size[7:0] */
  {0x034E, 0x0C}, /* y_output_size[15:8] */
  {0x034F, 0x30}, /* y_output_size[7:0] */
  {0x0381, 0x01}, /* x_even_inc[3:0] */
  {0x0383, 0x01}, /* x_odd_inc[3:0] */
  {0x0385, 0x01}, /* y_even_inc[7:0] */
  {0x0387, 0x01}, /* y_odd_inc[7:0] */
  {0x3040, 0x08},
  {0x3041, 0x97},
  {0x3048, 0x00},
  {0x3064, 0x12},
  {0x309B, 0x20},
  {0x309E, 0x00},
  {0x30D5, 0x00},
  {0x30D6, 0x85},
  {0x30D7, 0x2A},
  {0x30D8, 0x64},
  {0x30D9, 0x89},
  {0x30DE, 0x00},
  {0x3102, 0x10},
  {0x3103, 0x44},
  {0x3104, 0x40},
  {0x3105, 0x00},
  {0x3106, 0x0D},
  {0x3107, 0x01},
  {0x310A, 0x0A},
  {0x315C, 0x99},
  {0x315D, 0x98},
  {0x316E, 0x9A},
  {0x316F, 0x99},
  {0x3318, 0x64},
};

static struct msm_camera_i2c_reg_conf imx091_recommend_settings[] = {
  /* global setting */
  {0x3087, 0x53},
  {0x309D, 0x94},
  {0x30A1, 0x08},
  {0x30C7, 0x00},
  {0x3115, 0x0E},
  {0x3118, 0x42},
  {0x311D, 0x34},
  {0x3121, 0x0D},
  {0x3212, 0xF2},
  {0x3213, 0x0F},
  {0x3215, 0x0F},
  {0x3217, 0x0B},
  {0x3219, 0x0B},
  {0x321B, 0x0D},
  {0x321D, 0x0D},
  /* black level setting */
  {0x3032, 0x40},
  /* denosise setting*/
  {0x30D0, 0x04},
  {0x3620, 0x0F},
};

static struct v4l2_subdev_info imx091_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array imx091_init_conf[] = {
  {&imx091_recommend_settings[0],
    ARRAY_SIZE(imx091_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_camera_i2c_conf_array imx091_confs[] = {
  {&imx091_snap_settings[0],
  ARRAY_SIZE(imx091_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
  {&imx091_prev_settings[0],
  ARRAY_SIZE(imx091_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t imx091_dimensions[] = {
  {
    /* full size */
    .x_output = 4160,//0x1070, /* 4208 */
    .y_output = 0x0C30, /* 3120 */
    .line_length_pclk = 0x120C, /* 4620 */
    .frame_length_lines = 0x0C8C, /* 3212 */
    .vt_pixel_clk = 206400000,
    .op_pixel_clk = 206400000,
    .binning_factor = 1,
  },
  {
    /* 30 fps 1/2 * 1/2 */
    .x_output = 0x0838, /* 2104 */
    .y_output = 0x0618, /* 1560 */
    .line_length_pclk = 0x120C, /* 4620 */
    .frame_length_lines = 0x065A, /* 1626 */
    .vt_pixel_clk = 225600000,
    .op_pixel_clk = 225600000,
    .binning_factor = 1,
  },
};


static struct msm_sensor_output_reg_addr_t imx091_reg_addr = {
  .x_output = 0x034C,
  .y_output = 0x034E,
  .line_length_pclk = 0x0342,
  .frame_length_lines = 0x0340,
};

static struct msm_sensor_id_info_t imx091_id_info = {
	.sensor_id_reg_addr = 0x0000,
	.sensor_id = 0x0091,
};

static struct msm_sensor_exp_gain_info_t imx091_exp_gain_info = {
  .coarse_int_time_addr = 0x0202,
  .global_gain_addr = 0x0204,
  .vert_offset = 5,
};

//Eric Liu+, for IMX091 read OTP
int32_t imx091_read_otp_bank(struct msm_sensor_ctrl_t* s_ctrl, uint8_t bank, uint16_t addr, void *data){
	int rc = 0;
	msm_camera_i2c_write(
				s_ctrl->sensor_i2c_client,
				0x34C9,
				bank,
				MSM_CAMERA_I2C_BYTE_DATA
				);
	rc = msm_camera_i2c_read_seq(
			s_ctrl->sensor_i2c_client, addr, data, 8);
	
	return rc;
}

uint8_t imx091_read_otp_data(struct msm_sensor_ctrl_t* s_ctrl)
{
	int16_t i = 0;
	uint8_t bank = 0x0B;
	uint8_t is_otp_data_written = 0;
	uint8_t otp_group = 0;
	uint16_t regAddress = 0x355F, data = 0;
	int32_t rc = 0;
	uint8_t buf[8];

	MSG2("%s+",__func__);

  //Hiko: setting the default value for invalid situation
  st_imx091_otp.start_dac = 0xFFFF;
  st_imx091_otp.macro_dac = 0xFFFF;

  //Check which group has otp data from otp_group 2 to otp_group 0.
	for(otp_group = 2; otp_group >= 0; otp_group--)
	{
		msm_camera_i2c_write(
			s_ctrl->sensor_i2c_client,
			0x34C9,
			bank,
			MSM_CAMERA_I2C_BYTE_DATA
			);
		msm_camera_i2c_read(
			s_ctrl->sensor_i2c_client,
			regAddress,
			&data,
			MSM_CAMERA_I2C_BYTE_DATA
			);
		is_otp_data_written = (uint8_t)data;
		
		if(is_otp_data_written)
		{
			break;
		}
		else
		{
			bank -= 4;
			regAddress -= 0x20;
		}
	}

	if(bank < 0)
	{
		MSG2("OTP has no data written !!");
		return 0;
	}
	
	bank = 0x00 + otp_group*4;
	regAddress = 0x3500 + otp_group*0x20;

	// Read OTP data bank by bank from valid otp_group 
	for (i = 0; i < 4; ++i)
	{
		imx091_read_otp_bank(s_ctrl,bank,regAddress,buf);
		regAddress += 8;
		bank++;
		if(i == 0)
		{
			st_imx091_otp.vendor_id = buf[0];
			st_imx091_otp.year      = buf[1];
			st_imx091_otp.month     = buf[2];
			st_imx091_otp.day       = buf[3];
			st_imx091_otp.start_dac = buf[4] << 8 | buf[5];
			st_imx091_otp.macro_dac = buf[6] << 8 | buf[7];
  	}
		else if(i == 1)
		{
			st_imx091_otp.d65_wb_r_over_g   = buf[0] << 8 | buf[1];
			st_imx091_otp.d65_wb_b_over_g   = buf[2] << 8 | buf[3];
			st_imx091_otp.d65_wb_gr_over_gb = buf[4] << 8 | buf[5];
			st_imx091_otp.cwf_wb_r_over_g   = buf[6] << 8 | buf[7];
		}
		else if (i == 2 )
		{
			st_imx091_otp.cwf_wb_b_over_g   = buf[0] << 8 | buf[1];
			st_imx091_otp.cwf_wb_gr_over_gb = buf[2] << 8 | buf[3];
			st_imx091_otp.u30_wb_r_over_g   = buf[4] << 8 | buf[5];
			st_imx091_otp.u30_wb_b_over_g   = buf[6] << 8 | buf[7];
		}
		else if(i == 3)
		{
			st_imx091_otp.u30_wb_gr_over_gb = buf[0] << 8 | buf[1];
		}
	}

	//s_ctrl->msm_sensor_reg->otp_data_imx091 = &st_imx091_otp;

	// Log all OTP data
	MSG2("otp_data_imx091.vendor_id = %d",          st_imx091_otp.vendor_id);
	MSG2("otp_data_imx091.year = %d",               st_imx091_otp.year);
	MSG2("otp_data_imx091.month = %d",              st_imx091_otp.month);
	MSG2("otp_data_imx091.day = %d",                st_imx091_otp.day);
	MSG2("otp_data_imx091.start_dac = %d",          st_imx091_otp.start_dac);
	MSG2("otp_data_imx091.marco_dac = %d",          st_imx091_otp.macro_dac);
	MSG2("otp_data_imx091.d65_wb.r_over_g = %d",    st_imx091_otp.d65_wb_r_over_g);
	MSG2("otp_data_imx091.d65_wb.b_over_g = %d",    st_imx091_otp.d65_wb_b_over_g);
	MSG2("otp_data_imx091.d65_wb.gr_over_gb = %d",  st_imx091_otp.d65_wb_gr_over_gb);
	MSG2("otp_data_imx091.cwf_wb.r_over_g = %d",    st_imx091_otp.cwf_wb_r_over_g);
	MSG2("otp_data_imx091.cwf_wb.b_over_g = %d",    st_imx091_otp.cwf_wb_b_over_g);
	MSG2("otp_data_imx091.cwf_wb.gr_over_gb = %d",  st_imx091_otp.cwf_wb_gr_over_gb);
	MSG2("otp_data_imx091.u30_wb.r_over_g = %d",    st_imx091_otp.u30_wb_r_over_g);
	MSG2("otp_data_imx091.u30_wb.b_over_g = %d",    st_imx091_otp.u30_wb_b_over_g);
	MSG2("otp_data_imx091.u30_wb.gr_over_gb = %d",  st_imx091_otp.u30_wb_gr_over_gb);

	MSG2("%s-",__func__);

	return rc;
}
//Eric Liu-


#if 0 //todo
static int32_t imx091_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines, offset;
	uint8_t int_time[3];

	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;

//	pr_err("imx091_write_exp_gain: %d  frame_lenght_linegs:%d %d\n", fl_lines, s_ctrl->curr_frame_length_lines, s_ctrl->fps_divider);
	
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;
	CDBG("imx091_write_exp_gain: %d %d %d\n", fl_lines, gain, line);
//       pr_err("imx091_write_exp_gain: %d %d %d\n", fl_lines, gain, line);
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
#endif

static const struct i2c_device_id imx091_i2c_id[] = {
	{IMX091_SENSOR_NAME, (kernel_ulong_t)&imx091_s_ctrl},
	{ }
};

static struct i2c_driver imx091_i2c_driver = {
	.id_table = imx091_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = IMX091_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx091_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static enum msm_camera_vreg_name_t imx091_veg_seq_evt0[] = {
	CAM_VANA,
	//CAM_VAF,
	CAM_VDIG,
	CAM_VIO,
};

static enum msm_camera_vreg_name_t imx091_veg_seq_evt1[] = {
	CAM_VANA_EXT,
	//CAM_VAF,
	CAM_VDIG,
	CAM_VIO,
};

static enum msm_camera_vreg_name_t imx091_veg_seq[] = {
	CAM_VANA_EXT,
	//CAM_VAF,
	CAM_VDIG,
	CAM_VIO,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&imx091_i2c_driver);
}

static struct v4l2_subdev_core_ops imx091_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops imx091_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops imx091_subdev_ops = {
	.core = &imx091_subdev_core_ops,
	.video  = &imx091_subdev_video_ops,
};


int32_t imx091_get_calib_params(struct msm_sensor_ctrl_t *s_ctrl, struct otp_params* otp_params)	
{
	int rc = 0;

	pr_err("%s, enter\n", __func__);

	otp_params->size = sizeof(struct otp_struct_imx091);
	if (copy_to_user((void *)otp_params->otp_data_imx091,
		&st_imx091_otp,
		sizeof(struct otp_struct_imx091)))
		rc = -EFAULT;

	pr_err("%s, rc:%d, size = %d\n", __func__, rc, otp_params->size);
	return rc;

}

static struct msm_sensor_fn_t imx091_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,  //imx091_write_exp_gain,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1, //imx091_write_exp_gain,
	.sensor_setting = msm_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_read_otp_mid = imx091_read_otp_data,
	.sensor_get_calib_params = imx091_get_calib_params,
};

static struct msm_sensor_reg_t imx091_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = imx091_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(imx091_start_settings),
	.stop_stream_conf = imx091_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(imx091_stop_settings),
	.group_hold_on_conf = imx091_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(imx091_groupon_settings),
	.group_hold_off_conf = imx091_groupoff_settings,
	.group_hold_off_conf_size =	ARRAY_SIZE(imx091_groupoff_settings),
	.init_settings = &imx091_init_conf[0],
	.init_size = ARRAY_SIZE(imx091_init_conf),
	.mode_settings = &imx091_confs[0],
	.output_settings = &imx091_dimensions[0],
	.num_conf = ARRAY_SIZE(imx091_confs),
};

static struct msm_sensor_ctrl_t imx091_s_ctrl = {
	.msm_sensor_reg = &imx091_regs,
	.sensor_i2c_client = &imx091_sensor_i2c_client,
	.sensor_i2c_addr = 0x34,
	.vreg_seq_evt0 = imx091_veg_seq_evt0,
	.num_vreg_seq_evt0 = ARRAY_SIZE(imx091_veg_seq_evt0),
	.vreg_seq_evt1 = imx091_veg_seq_evt1,
	.num_vreg_seq_evt1 = ARRAY_SIZE(imx091_veg_seq_evt1),
	.vreg_seq = imx091_veg_seq,
	.num_vreg_seq = ARRAY_SIZE(imx091_veg_seq),
	.sensor_output_reg_addr = &imx091_reg_addr,
	.sensor_id_info = &imx091_id_info,
	.sensor_exp_gain_info = &imx091_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex = &imx091_mut,
	.sensor_i2c_driver = &imx091_i2c_driver,
	.sensor_v4l2_subdev_info = imx091_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx091_subdev_info),
	.sensor_v4l2_subdev_ops = &imx091_subdev_ops,
	.func_tbl = &imx091_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,

// TODO: whilte msm_sensor_i2c_probe(), copy the otp data to msm_sensor.c, used in ftd
	//.otp_info = &st_imx091_otp,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Omnivison 13MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
