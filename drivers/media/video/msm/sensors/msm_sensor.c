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
 */

#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include "msm_sensor.h"
#include "msm_sensor_common.h"
#include "msm.h"
#include "msm_ispif.h"
#include "msm_camera_i2c_mux.h"
#include <mach/hwid.h>

//Eric Liu+
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
static struct pmlog_device *pmlog_device_cam_front = NULL;
static struct pmlog_device *pmlog_device_cam_back = NULL;
#endif //CONFIG_PM_LOG

#define MSG2(format, arg...)  printk(KERN_INFO "[CAM]" format "\n", ## arg)
static int cam_id_front = 0;
static int cam_id_back  = 0;
static unsigned long long cam_fuse_id_front = 0;

//whenever the setting is the same, we can skip the real i2c config
static int front_cam_auto_white_balance = -1;
static int front_cam_white_balance_temperature = -1;
static int front_cam_exposure = -1;
static int front_cam_band_stop_filter = -1;
static int front_cam_sharpness = -1; //sophia
//Eric Liu-

//Sophia Wang++, 20120815 for storing calibration data
static uint8_t back_sensor_calied_awb_d65_rg = 0;
static uint8_t back_sensor_calied_awb_d65_bg = 0;
static uint8_t back_sensor_calied_awb_cwf_rg = 0;
static uint8_t back_sensor_calied_awb_cwf_bg = 0;
static uint8_t back_sensor_calied_awb_u30_rg = 0;
static uint8_t back_sensor_calied_awb_u30_bg = 0;


static uint8_t back_sensor_calied_af_ma_msb = 0;
static uint8_t back_sensor_calied_af_ma_lsb = 0;
static uint8_t back_sensor_calied_af_hori_msb = 0; //start dac
static uint8_t back_sensor_calied_af_hori_lsb = 0;// start dac

static uint8_t back_sensor_calied_production_data_year = 0; 
static uint8_t back_sensor_calied_production_data_mouth = 0; 
static uint8_t back_sensor_calied_production_data_date = 0; 
static uint8_t back_sensor_calied_production_data_hour = 0;
static uint8_t back_sensor_calied_production_data_minuite = 0;
static uint8_t back_sensor_calied_production_data_second = 0; 

//0x1--boston with blue glass
//0x2--Dubai without blue glass
static uint8_t back_sensor_calied_production_project_id= 0xff;
//Sophia Wang++, 20120815 for storing calibration data

//Sophia Wang++, 20120606 for auto camera test
 int ov8825_af_done = 0xff;
 int ov8825_af_result = 0xff;
//Sophia Wang--, 20120606 for auto camera test


int  initOpt= false; //add by sophia
//#undef CDBG
//#define CDBG pr_err

/******************************************************************************
 * add OTP information
******************************************************************************/
static ssize_t back_cam_otp_awb_d65_rg_read(struct device *dev, 
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_awb_d65_rg);
}
static DEVICE_ATTR(back_otp_awb_d65_rg, 0444, back_cam_otp_awb_d65_rg_read, NULL);

static ssize_t back_cam_otp_awb_d65_bg_read(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_awb_d65_bg);
}
static DEVICE_ATTR(back_otp_awb_d65_bg, 0444, back_cam_otp_awb_d65_bg_read, NULL);


static ssize_t back_cam_otp_awb_cwf_rg_read(struct device *dev, 
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_awb_cwf_rg);
}
static DEVICE_ATTR(back_otp_awb_cwf_rg, 0444, back_cam_otp_awb_cwf_rg_read, NULL);

static ssize_t back_cam_otp_awb_cwf_bg_read(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_awb_cwf_bg);
}
static DEVICE_ATTR(back_otp_awb_cwf_bg, 0444, back_cam_otp_awb_cwf_bg_read, NULL);

static ssize_t back_cam_otp_awb_u30_rg_read(struct device *dev, 
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_awb_u30_rg);
}
static DEVICE_ATTR(back_otp_awb_u30_rg, 0444, back_cam_otp_awb_u30_rg_read, NULL);

static ssize_t back_cam_otp_awb_u30_bg_read(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_awb_u30_bg);
}
static DEVICE_ATTR(back_otp_awb_u30_bg, 0444, back_cam_otp_awb_u30_bg_read, NULL);

static ssize_t back_cam_otp_af_ma_msb_read(	struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_af_ma_msb);
}
static DEVICE_ATTR(back_otp_af_ma_msb, 0444, back_cam_otp_af_ma_msb_read, NULL);

static ssize_t back_cam_otp_af_ma_lsb_read(	struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_af_ma_lsb);
}
static DEVICE_ATTR(back_otp_af_ma_lsb, 0444, back_cam_otp_af_ma_lsb_read, NULL);

static ssize_t back_cam_otp_af_hori_msb_read(	struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_af_hori_msb);
}
static DEVICE_ATTR(back_otp_af_hori_msb, 0444, back_cam_otp_af_hori_msb_read, NULL);

static ssize_t back_cam_otp_af_hori_lsb_read(	struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 10, "0x%x\n",back_sensor_calied_af_hori_lsb);
}
static DEVICE_ATTR(back_otp_af_hori_lsb, 0444, back_cam_otp_af_hori_lsb_read, NULL);


static ssize_t back_cam_otp_project_id_read(	struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 10, "%d\n",back_sensor_calied_production_project_id);
}
static DEVICE_ATTR(back_sensor_otp_production_project_id, 0444, back_cam_otp_project_id_read, NULL);

static ssize_t back_cam_otp_production_data_read(	struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return snprintf(buf, 14, "%02d%02d%02d%02d%02d%02d\n",
		back_sensor_calied_production_data_year,
		back_sensor_calied_production_data_mouth,
		back_sensor_calied_production_data_date,
		back_sensor_calied_production_data_hour,
		back_sensor_calied_production_data_minuite,
		back_sensor_calied_production_data_second);

#if 0
	return snprintf(buf, 12, "%02d%02d%02d%02d%02d%02d\n",
		back_sensor_calied_production_data_year,
		back_sensor_calied_production_data_mouth,
		back_sensor_calied_production_data_date,
		back_sensor_calied_production_data_hour,
		back_sensor_calied_production_data_minuite,
		back_sensor_calied_production_data_second);
#endif
}
static DEVICE_ATTR(back_otp_production_data, 0444, back_cam_otp_production_data_read, NULL);

static int cam_create_sys_entries(struct i2c_client *client)
{
	int ret=0;

	ret = device_create_file(&client->dev,&dev_attr_back_otp_awb_d65_rg);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_back_otp_awb_d65_bg);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_back_otp_awb_cwf_rg);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_back_otp_awb_cwf_bg);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_back_otp_awb_u30_rg);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_back_otp_awb_u30_bg);
	WARN_ON(ret);
	if (ret)  return ret;
	
	ret = device_create_file(&client->dev,&dev_attr_back_otp_af_ma_msb);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_back_otp_af_ma_lsb);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_back_otp_af_hori_msb);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_back_otp_af_hori_lsb);
	WARN_ON(ret);

	ret = device_create_file(&client->dev,&dev_attr_back_otp_production_data);
	WARN_ON(ret);

	ret = device_create_file(&client->dev,&dev_attr_back_sensor_otp_production_project_id);
	WARN_ON(ret);	
	return ret;
}

/*=============================================================*/
void msm_sensor_adjust_frame_lines1(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t cur_line = 0;
	uint16_t exp_fl_lines = 0;
	if (s_ctrl->sensor_exp_gain_info) {
		msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
			&cur_line,
			MSM_CAMERA_I2C_WORD_DATA);
		exp_fl_lines = cur_line +
			s_ctrl->sensor_exp_gain_info->vert_offset;
		if (exp_fl_lines > s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].frame_length_lines)
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				s_ctrl->sensor_output_reg_addr->
				frame_length_lines,
				exp_fl_lines,
				MSM_CAMERA_I2C_WORD_DATA);
		CDBG("%s cur_fl_lines %d, exp_fl_lines %d\n", __func__,
			s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].frame_length_lines,
			exp_fl_lines);
	}
	return;
}

void msm_sensor_adjust_frame_lines2(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t cur_line = 0;
	uint16_t exp_fl_lines = 0;
	uint8_t int_time[3];
	if (s_ctrl->sensor_exp_gain_info) {
		msm_camera_i2c_read_seq(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->coarse_int_time_addr-1,
			&int_time[0], 3);
		cur_line |= int_time[0] << 12;
		cur_line |= int_time[1] << 4;
		cur_line |= int_time[2] >> 4;
		exp_fl_lines = cur_line +
			s_ctrl->sensor_exp_gain_info->vert_offset;
		if (exp_fl_lines > s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].frame_length_lines)
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				s_ctrl->sensor_output_reg_addr->
				frame_length_lines,
				exp_fl_lines,
				MSM_CAMERA_I2C_WORD_DATA);
		CDBG("%s cur_line %x cur_fl_lines %x, exp_fl_lines %x\n",
			__func__,
			cur_line,
			s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].frame_length_lines,
			exp_fl_lines);
	}
	return;
}

static void msm_sensor_delay_frames(struct msm_sensor_ctrl_t *s_ctrl)
{
	long fps = 0;
	uint32_t delay = 0;

	if (s_ctrl->curr_res < MSM_SENSOR_INVALID_RES &&
		s_ctrl->wait_num_frames > 0) {
		fps = s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].vt_pixel_clk /
			s_ctrl->curr_frame_length_lines /
			s_ctrl->curr_line_length_pclk;
		if (fps == 0)
			delay = s_ctrl->min_delay;
		else
			delay = (1000 * s_ctrl->wait_num_frames) / fps / Q10;
	}
	pr_err("%s fps = %ld, delay = %d, min_delay %d\n", __func__, fps,
		delay, s_ctrl->min_delay);
	if (delay > s_ctrl->min_delay)
		msleep(delay);
	else if (s_ctrl->min_delay)
		msleep(s_ctrl->min_delay);
	return;
}

int32_t msm_sensor_write_init_settings(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc;
	rc = msm_sensor_write_all_conf_array(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->init_settings,
		s_ctrl->msm_sensor_reg->init_size);
	return rc;
}

int32_t msm_sensor_write_res_settings(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
	int32_t rc;
	rc = msm_sensor_write_conf_array(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->mode_settings, res);
	if (rc < 0)
		return rc;

	rc = msm_sensor_write_output_settings(s_ctrl, res);
	if (rc < 0)
		return rc;

	return rc;
}

int32_t msm_sensor_write_output_settings(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t res)
{
	int32_t rc = -EFAULT;
	struct msm_camera_i2c_reg_conf dim_settings[] = {
		{s_ctrl->sensor_output_reg_addr->x_output,
			s_ctrl->msm_sensor_reg->
			output_settings[res].x_output},
		{s_ctrl->sensor_output_reg_addr->y_output,
			s_ctrl->msm_sensor_reg->
			output_settings[res].y_output},
		{s_ctrl->sensor_output_reg_addr->line_length_pclk,
			s_ctrl->msm_sensor_reg->
			output_settings[res].line_length_pclk},
		{s_ctrl->sensor_output_reg_addr->frame_length_lines,
			s_ctrl->msm_sensor_reg->
			output_settings[res].frame_length_lines},
	};

	rc = msm_camera_i2c_write_tbl(s_ctrl->sensor_i2c_client, dim_settings,
		ARRAY_SIZE(dim_settings), MSM_CAMERA_I2C_WORD_DATA);
	return rc;
}

void msm_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
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

	//Sophia Wang++
	//msleep(20);
	msm_sensor_delay_frames(s_ctrl);
	//Sophia Wang--
}

void msm_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->stop_stream_conf,
		s_ctrl->msm_sensor_reg->stop_stream_conf_size,
		s_ctrl->msm_sensor_reg->default_data_type);
	msm_sensor_delay_frames(s_ctrl);
}

void msm_sensor_group_hold_on(struct msm_sensor_ctrl_t *s_ctrl)
{
	msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->group_hold_on_conf,
		s_ctrl->msm_sensor_reg->group_hold_on_conf_size,
		s_ctrl->msm_sensor_reg->default_data_type);
}

void msm_sensor_group_hold_off(struct msm_sensor_ctrl_t *s_ctrl)
{
	msm_camera_i2c_write_tbl(
		s_ctrl->sensor_i2c_client,
		s_ctrl->msm_sensor_reg->group_hold_off_conf,
		s_ctrl->msm_sensor_reg->group_hold_off_conf_size,
		s_ctrl->msm_sensor_reg->default_data_type);
}

int32_t msm_sensor_set_fps(struct msm_sensor_ctrl_t *s_ctrl,
						struct fps_cfg *fps)
{
	s_ctrl->fps_divider = fps->fps_div;

	return 0;
}

int32_t msm_sensor_write_exp_gain1(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines;
	uint8_t offset;
	fl_lines = s_ctrl->curr_frame_length_lines;
	fl_lines = (fl_lines * s_ctrl->fps_divider) / Q10;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;
	fl_lines += (fl_lines & 0x01);

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
		MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr, line,
		MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
		MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}

int32_t msm_sensor_write_exp_gain2(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines, ll_pclk, ll_ratio;
	uint8_t offset;
	fl_lines = s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider / Q10;
	ll_pclk = s_ctrl->curr_line_length_pclk;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset)) {
		ll_ratio = (line * Q10) / (fl_lines - offset);
		ll_pclk = ll_pclk * ll_ratio / Q10;
		line = fl_lines - offset;
	}

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_output_reg_addr->line_length_pclk, ll_pclk,
		MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr, line,
		MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
		MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}

int32_t msm_sensor_setting1(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	if (update_type == MSM_SENSOR_REG_INIT) {
		CDBG("Register INIT\n");
		msm_sensor_enable_debugfs(s_ctrl);
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
		msm_sensor_write_init_settings(s_ctrl);
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		CDBG("PERIODIC : %d\n", res);
		msm_sensor_write_conf_array(
			s_ctrl->sensor_i2c_client,
			s_ctrl->msm_sensor_reg->mode_settings, res);
		msleep(30);
		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE,
			&s_ctrl->sensordata->pdata->ioclk.vfe_clk_rate);
	}
	return rc;
}

int32_t msm_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
			int update_type, int res)
{
	int32_t rc = 0;

	if (update_type == MSM_SENSOR_REG_INIT) {
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
		msm_sensor_write_init_settings(s_ctrl);
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		msm_sensor_write_res_settings(s_ctrl, res);
		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[res].op_pixel_clk);
	}
	return rc;
}

int32_t msm_sensor_set_sensor_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int mode, int res)
{
	int32_t rc = 0;
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

	return rc;
}

int32_t msm_sensor_mode_init(struct msm_sensor_ctrl_t *s_ctrl,
			int mode, struct sensor_init_cfg *init_info)
{
	int32_t rc = 0;
	s_ctrl->fps_divider = Q10;
	s_ctrl->cam_mode = MSM_SENSOR_MODE_INVALID;

	CDBG("%s: %d\n", __func__, __LINE__);
	if (mode != s_ctrl->cam_mode) {
		s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
		s_ctrl->cam_mode = mode;

		if (s_ctrl->is_csic ||
			!s_ctrl->sensordata->csi_if)
			rc = s_ctrl->func_tbl->sensor_csi_setting(s_ctrl,
				MSM_SENSOR_REG_INIT, 0);
		else
			rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
				MSM_SENSOR_REG_INIT, 0);
	}
	return rc;
}

int32_t msm_sensor_get_output_info(struct msm_sensor_ctrl_t *s_ctrl,
		struct sensor_output_info_t *sensor_output_info)
{
	int rc = 0;
	sensor_output_info->num_info = s_ctrl->msm_sensor_reg->num_conf;
	if (copy_to_user((void *)sensor_output_info->output_info,
		s_ctrl->msm_sensor_reg->output_settings,
		sizeof(struct msm_sensor_output_info_t) *
		s_ctrl->msm_sensor_reg->num_conf))
		rc = -EFAULT;

	return rc;
}

static int32_t msm_sensor_release(struct msm_sensor_ctrl_t *s_ctrl)
{
	CDBG("%s called\n", __func__);
	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	return 0;
}

long msm_sensor_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	void __user *argp = (void __user *)arg;
	if (s_ctrl->sensor_state == MSM_SENSOR_POWER_DOWN)
		return -EINVAL;
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_CFG:
		return s_ctrl->func_tbl->sensor_config(s_ctrl, argp);
	case VIDIOC_MSM_SENSOR_RELEASE:
		return msm_sensor_release(s_ctrl);
	case VIDIOC_MSM_SENSOR_CSID_INFO: {
		struct msm_sensor_csi_info *csi_info =
			(struct msm_sensor_csi_info *)arg;
		s_ctrl->is_csic = csi_info->is_csic;
		return 0;
	}
	default:
		return -ENOIOCTLCMD;
	}
}

int32_t msm_sensor_get_csi_params(struct msm_sensor_ctrl_t *s_ctrl,
		struct csi_lane_params_t *sensor_output_info)
{
	uint8_t index;
	struct msm_camera_csi_lane_params *csi_lane_params =
		s_ctrl->sensordata->sensor_platform_info->csi_lane_params;
	if (csi_lane_params) {
		sensor_output_info->csi_lane_assign = csi_lane_params->
			csi_lane_assign;
		sensor_output_info->csi_lane_mask = csi_lane_params->
			csi_lane_mask;
		sensor_output_info->csi_phy_sel = csi_lane_params->csi_phy_sel;
	}
	sensor_output_info->csi_if = s_ctrl->sensordata->csi_if;
	for (index = 0; index < sensor_output_info->csi_if; index++)
		sensor_output_info->csid_core[index] = s_ctrl->sensordata->
			pdata[index].csid_core;

	return 0;
}

int32_t msm_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata.cfgtype);
	switch (cdata.cfgtype) {
		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			if (s_ctrl->func_tbl->
			sensor_set_fps == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_set_fps(
				s_ctrl,
				&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			if (s_ctrl->func_tbl->
			sensor_write_exp_gain == NULL) {
				rc = -EFAULT;
				break;
			}
			rc =
				s_ctrl->func_tbl->
				sensor_write_exp_gain(
					s_ctrl,
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			if (s_ctrl->func_tbl->
			sensor_write_snapshot_exp_gain == NULL) {
				rc = -EFAULT;
				break;
			}
			rc =
				s_ctrl->func_tbl->
				sensor_write_snapshot_exp_gain(
					s_ctrl,
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_sensor_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_set_sensor_mode(
					s_ctrl,
					cdata.mode,
					cdata.rs);
			break;

		case CFG_SET_EFFECT:
			break;

		case CFG_SENSOR_INIT:
			if (s_ctrl->func_tbl->
			sensor_mode_init == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_mode_init(
				s_ctrl,
				cdata.mode,
				&(cdata.cfg.init_info));
			break;

		case CFG_GET_OUTPUT_INFO:
			if (s_ctrl->func_tbl->
			sensor_get_output_info == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_get_output_info(
				s_ctrl,
				&cdata.cfg.output_info);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_START_STREAM:
			if (s_ctrl->func_tbl->sensor_start_stream == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
			break;

		case CFG_STOP_STREAM:
			if (s_ctrl->func_tbl->sensor_stop_stream == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			break;

		case CFG_GET_CSI_PARAMS:
			if (s_ctrl->func_tbl->sensor_get_csi_params == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->sensor_get_csi_params(
				s_ctrl,
				&cdata.cfg.csi_lane_params);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_POWER_UP:
			pr_err("%s calling power up\n", __func__);
			if (s_ctrl->func_tbl->sensor_power_up)
				rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			else
				rc = -EFAULT;
			break;

		case CFG_POWER_DOWN:
			if (s_ctrl->func_tbl->sensor_power_down)
				rc = s_ctrl->func_tbl->sensor_power_down(
					s_ctrl);
			else
				rc = -EFAULT;
			break;

		//Sophia Wang++
		case CFG_GET_CALIB_DATA:
			if (s_ctrl->func_tbl->sensor_get_calib_params == NULL) {
				rc = -EFAULT;
				break;
			}

			rc = s_ctrl->func_tbl->sensor_get_calib_params(
				s_ctrl,
				&cdata.cfg.otp_params);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
		//Sophia Wang--

		default:
			rc = -EFAULT;
			break;
		}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_cam_clk_info cam_8960_clk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
};

static struct msm_cam_clk_info cam_8974_clk_info[] = {
	{"cam_src_clk", 19200000},
	{"cam_clk", -1},
};

int32_t msm_sensor_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

int32_t msm_sensor_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}

static int32_t msm_sensor_init_flash_data(struct device_node *of_node,
	struct  msm_camera_sensor_info *sensordata)
{
	int32_t rc = 0;
	uint32_t val = 0;

	sensordata->flash_data = kzalloc(sizeof(
		struct msm_camera_sensor_flash_data), GFP_KERNEL);
	if (!sensordata->flash_data) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	rc = of_property_read_u32(of_node, "qcom,flash-type", &val);
	CDBG("%s qcom,flash-type %d, rc %d\n", __func__, val, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR;
	}
	sensordata->flash_data->flash_type = val;
	return rc;
ERROR:
	kfree(sensordata->flash_data);
	return rc;
}

static int32_t msm_sensor_init_vreg_data(struct device_node *of_node,
	struct msm_camera_sensor_platform_info *pinfo)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint32_t *val_array = NULL;

	count = of_property_count_strings(of_node, "qcom,cam-vreg-name");
	CDBG("%s qcom,cam-vreg-name count %d\n", __func__, count);

	if (!count)
		return 0;

	pinfo->cam_vreg = kzalloc(sizeof(struct camera_vreg_t) * count,
		GFP_KERNEL);
	if (!pinfo->cam_vreg) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	pinfo->num_vreg = count;
	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"qcom,cam-vreg-name", i, &pinfo->cam_vreg[i].reg_name);
		CDBG("%s reg_name[%d] = %s\n", __func__, i,
			pinfo->cam_vreg[i].reg_name);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR1;
		}
	}

	val_array = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!val_array) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR1;
	}

	rc = of_property_read_u32_array(of_node, "qcom,cam-vreg-type",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		pinfo->cam_vreg[i].type = val_array[i];
		CDBG("%s cam_vreg[%d].type = %d\n", __func__, i,
			pinfo->cam_vreg[i].type);
	}

	rc = of_property_read_u32_array(of_node, "qcom,cam-vreg-min-voltage",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		pinfo->cam_vreg[i].min_voltage = val_array[i];
		CDBG("%s cam_vreg[%d].min_voltage = %d\n", __func__,
			i, pinfo->cam_vreg[i].min_voltage);
	}

	rc = of_property_read_u32_array(of_node, "qcom,cam-vreg-max-voltage",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		pinfo->cam_vreg[i].max_voltage = val_array[i];
		CDBG("%s cam_vreg[%d].max_voltage = %d\n", __func__,
			i, pinfo->cam_vreg[i].max_voltage);
	}

	rc = of_property_read_u32_array(of_node, "qcom,cam-vreg-op-mode",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		pinfo->cam_vreg[i].op_mode = val_array[i];
		CDBG("%s cam_vreg[%d].op_mode = %d\n", __func__, i,
			pinfo->cam_vreg[i].op_mode);
	}

	kfree(val_array);
	return rc;
ERROR2:
	kfree(val_array);
ERROR1:
	kfree(pinfo->cam_vreg);
	pinfo->num_vreg = 0;
	return rc;
}

static int32_t msm_sensor_init_gpio_common_tbl_data(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint32_t *val_array = NULL;

	if (!of_get_property(of_node, "qcom,gpio-common-tbl-num", &count))
		return 0;

	count /= sizeof(uint32_t);
	if (!count) {
		pr_err("%s qcom,gpio-common-tbl-num 0\n", __func__);
		return 0;
	} else if (count > gpio_array_size) {
		pr_err("%s gpio common tbl size exceeds gpio array\n",
			__func__);
		return -EFAULT;
	}

	val_array = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!val_array) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	gconf->cam_gpio_common_tbl = kzalloc(sizeof(struct gpio) * count,
		GFP_KERNEL);
	if (!gconf->cam_gpio_common_tbl) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR1;
	}
	gconf->cam_gpio_common_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "qcom,gpio-common-tbl-num",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			pr_err("%s gpio common tbl index %d invalid\n",
				__func__, val_array[i]);
			return -EINVAL;
		}
		gconf->cam_gpio_common_tbl[i].gpio = gpio_array[val_array[i]];
		CDBG("%s cam_gpio_common_tbl[%d].gpio = %d\n", __func__, i,
			gconf->cam_gpio_common_tbl[i].gpio);
	}

	rc = of_property_read_u32_array(of_node, "qcom,gpio-common-tbl-flags",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		gconf->cam_gpio_common_tbl[i].flags = val_array[i];
		CDBG("%s cam_gpio_common_tbl[%d].flags = %ld\n", __func__, i,
			gconf->cam_gpio_common_tbl[i].flags);
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"qcom,gpio-common-tbl-label", i,
			&gconf->cam_gpio_common_tbl[i].label);
		CDBG("%s cam_gpio_common_tbl[%d].label = %s\n", __func__, i,
			gconf->cam_gpio_common_tbl[i].label);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR2;
		}
	}

	kfree(val_array);
	return rc;

ERROR2:
	kfree(gconf->cam_gpio_common_tbl);
ERROR1:
	kfree(val_array);
	gconf->cam_gpio_common_tbl_size = 0;
	return rc;
}

static int32_t msm_sensor_init_gpio_req_tbl_data(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint32_t *val_array = NULL;

	if (!of_get_property(of_node, "qcom,gpio-req-tbl-num", &count))
		return 0;

	count /= sizeof(uint32_t);
	if (!count) {
		pr_err("%s qcom,gpio-req-tbl-num 0\n", __func__);
		return 0;
	}

	val_array = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!val_array) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	gconf->cam_gpio_req_tbl = kzalloc(sizeof(struct gpio) * count,
		GFP_KERNEL);
	if (!gconf->cam_gpio_req_tbl) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR1;
	}
	gconf->cam_gpio_req_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "qcom,gpio-req-tbl-num",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			pr_err("%s gpio req tbl index %d invalid\n",
				__func__, val_array[i]);
			return -EINVAL;
		}
		gconf->cam_gpio_req_tbl[i].gpio = gpio_array[val_array[i]];
		CDBG("%s cam_gpio_req_tbl[%d].gpio = %d\n", __func__, i,
			gconf->cam_gpio_req_tbl[i].gpio);
	}

	rc = of_property_read_u32_array(of_node, "qcom,gpio-req-tbl-flags",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		gconf->cam_gpio_req_tbl[i].flags = val_array[i];
		CDBG("%s cam_gpio_req_tbl[%d].flags = %ld\n", __func__, i,
			gconf->cam_gpio_req_tbl[i].flags);
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"qcom,gpio-req-tbl-label", i,
			&gconf->cam_gpio_req_tbl[i].label);
		CDBG("%s cam_gpio_req_tbl[%d].label = %s\n", __func__, i,
			gconf->cam_gpio_req_tbl[i].label);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR2;
		}
	}

	kfree(val_array);
	return rc;

ERROR2:
	kfree(gconf->cam_gpio_req_tbl);
ERROR1:
	kfree(val_array);
	gconf->cam_gpio_req_tbl_size = 0;
	return rc;
}

static int32_t msm_sensor_init_gpio_set_tbl_data(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint32_t *val_array = NULL;

	if (!of_get_property(of_node, "qcom,gpio-set-tbl-num", &count))
		return 0;

	count /= sizeof(uint32_t);
	if (!count) {
		pr_err("%s qcom,gpio-set-tbl-num 0\n", __func__);
		return 0;
	}

	val_array = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!val_array) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	gconf->cam_gpio_set_tbl = kzalloc(sizeof(struct msm_gpio_set_tbl) *
		count, GFP_KERNEL);
	if (!gconf->cam_gpio_set_tbl) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR1;
	}
	gconf->cam_gpio_set_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "qcom,gpio-set-tbl-num",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			pr_err("%s gpio set tbl index %d invalid\n",
				__func__, val_array[i]);
			return -EINVAL;
		}
		gconf->cam_gpio_set_tbl[i].gpio = gpio_array[val_array[i]];
		CDBG("%s cam_gpio_set_tbl[%d].gpio = %d\n", __func__, i,
			gconf->cam_gpio_set_tbl[i].gpio);
	}

	rc = of_property_read_u32_array(of_node, "qcom,gpio-set-tbl-flags",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		gconf->cam_gpio_set_tbl[i].flags = val_array[i];
		CDBG("%s cam_gpio_set_tbl[%d].flags = %ld\n", __func__, i,
			gconf->cam_gpio_set_tbl[i].flags);
	}

	rc = of_property_read_u32_array(of_node, "qcom,gpio-set-tbl-delay",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		gconf->cam_gpio_set_tbl[i].delay = val_array[i];
		CDBG("%s cam_gpio_set_tbl[%d].delay = %d\n", __func__, i,
			gconf->cam_gpio_set_tbl[i].delay);
	}

	kfree(val_array);
	return rc;

ERROR2:
	kfree(gconf->cam_gpio_set_tbl);
ERROR1:
	kfree(val_array);
	gconf->cam_gpio_set_tbl_size = 0;
	return rc;
}

static int32_t msm_sensor_init_gpio_tlmm_tbl_data(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint32_t *val_array = NULL;
	struct gpio_tlmm_cfg *tlmm_cfg = NULL;

	if (!of_get_property(of_node, "gpio_tlmm_table_num", &count))
		return 0;

	count /= sizeof(uint32_t);

	if (!count)
		return 0;

	val_array = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!val_array) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	tlmm_cfg = kzalloc(sizeof(struct gpio_tlmm_cfg) * count, GFP_KERNEL);
	if (!tlmm_cfg) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR1;
	}

	gconf->camera_off_table = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!gconf->camera_off_table) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR2;
	}
	gconf->camera_off_table_size = count;

	gconf->camera_on_table = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!gconf->camera_on_table) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR3;
	}
	gconf->camera_on_table_size = count;

	rc = of_property_read_u32_array(of_node, "gpio_tlmm_table_num",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR4;
	}
	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			pr_err("%s gpio set tbl index %d invalid\n",
				__func__, val_array[i]);
			return -EINVAL;
		}
		tlmm_cfg[i].gpio = gpio_array[val_array[i]];
		CDBG("%s tlmm_cfg[%d].gpio = %d\n", __func__, i,
			tlmm_cfg[i].gpio);
	}

	rc = of_property_read_u32_array(of_node, "gpio_tlmm_table_dir",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR4;
	}
	for (i = 0; i < count; i++) {
		tlmm_cfg[i].dir = val_array[i];
		CDBG("%s tlmm_cfg[%d].dir = %d\n", __func__, i,
			tlmm_cfg[i].dir);
	}

	rc = of_property_read_u32_array(of_node, "gpio_tlmm_table_pull",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR4;
	}
	for (i = 0; i < count; i++) {
		tlmm_cfg[i].pull = val_array[i];
		CDBG("%s tlmm_cfg[%d].pull = %d\n", __func__, i,
			tlmm_cfg[i].pull);
	}

	rc = of_property_read_u32_array(of_node, "gpio_tlmm_table_drvstr",
		val_array, count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR4;
	}
	for (i = 0; i < count; i++) {
		tlmm_cfg[i].drvstr = val_array[i];
		CDBG("%s tlmm_cfg[%d].drvstr = %d\n", __func__, i,
			tlmm_cfg[i].drvstr);
	}

	for (i = 0; i < count; i++) {
		gconf->camera_off_table[i] = GPIO_CFG(tlmm_cfg[i].gpio,
			0, tlmm_cfg[i].dir, tlmm_cfg[i].pull,
			tlmm_cfg[i].drvstr);
		gconf->camera_on_table[i] = GPIO_CFG(tlmm_cfg[i].gpio,
			1, tlmm_cfg[i].dir, tlmm_cfg[i].pull,
			tlmm_cfg[i].drvstr);
	}

	kfree(tlmm_cfg);
	kfree(val_array);
	return rc;

ERROR4:
	kfree(gconf->camera_on_table);
ERROR3:
	kfree(gconf->camera_off_table);
ERROR2:
	kfree(tlmm_cfg);
ERROR1:
	kfree(val_array);
	gconf->camera_off_table_size = 0;
	gconf->camera_on_table_size = 0;
	return rc;
}

static int32_t msm_sensor_init_csi_data(struct device_node *of_node,
	struct  msm_camera_sensor_info *sensordata)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0, val = 0;
	uint32_t *val_array = NULL;
	struct msm_camera_sensor_platform_info *pinfo =
		sensordata->sensor_platform_info;

	rc = of_property_read_u32(of_node, "qcom,csi-if", &count);
	CDBG("%s qcom,csi-if %d, rc %d\n", __func__, count, rc);
	if (rc < 0 || !count)
		return rc;
	sensordata->csi_if = count;

	sensordata->pdata = kzalloc(sizeof(
		struct msm_camera_device_platform_data) * count, GFP_KERNEL);
	if (!sensordata->pdata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	val_array = kzalloc(sizeof(uint32_t) * count, GFP_KERNEL);
	if (!val_array) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR1;
	}

	rc = of_property_read_u32_array(of_node, "qcom,csid-core", val_array,
		count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		sensordata->pdata[i].csid_core = val_array[i];
		CDBG("%s csi_data[%d].csid_core = %d\n", __func__, i,
			sensordata->pdata[i].csid_core);
	}

	rc = of_property_read_u32_array(of_node, "qcom,is-vpe", val_array,
		count);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}
	for (i = 0; i < count; i++) {
		sensordata->pdata[i].is_vpe = val_array[i];
		CDBG("%s csi_data[%d].is_vpe = %d\n", __func__, i,
			sensordata->pdata[i].is_vpe);
	}

	pinfo->csi_lane_params = kzalloc(
		sizeof(struct msm_camera_csi_lane_params), GFP_KERNEL);
	if (!pinfo->csi_lane_params) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR2;
	}

	rc = of_property_read_u32(of_node, "qcom,csi-lane-assign", &val);
	CDBG("%s qcom,csi-lane-assign %x, rc %d\n", __func__, val, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR3;
	}
	pinfo->csi_lane_params->csi_lane_assign = val;

	rc = of_property_read_u32(of_node, "qcom,csi-lane-mask", &val);
	CDBG("%s qcom,csi-lane-mask %x, rc %d\n", __func__, val, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR3;
	}
	pinfo->csi_lane_params->csi_lane_mask = val;

	rc = of_property_read_u32(of_node, "qcom,csi-phy-sel", &val);
	CDBG("%s qcom,csi-phy-sel %x, rc %d\n", __func__, val, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR3;
	}
	pinfo->csi_lane_params->csi_phy_sel = val;

	kfree(val_array);
	return rc;
ERROR3:
	kfree(pinfo->csi_lane_params);
ERROR2:
	kfree(val_array);
ERROR1:
	kfree(sensordata->pdata);
	sensordata->csi_if = 0;
	return rc;
}
static int32_t msm_sensor_init_actuator_data(struct device_node *of_node,
	struct  msm_camera_sensor_info *sensordata)
{
	int32_t rc = 0;
	uint32_t val = 0;

	rc = of_property_read_u32(of_node, "qcom,actuator-cam-name", &val);
	CDBG("%s qcom,actuator-cam-name %d, rc %d\n", __func__, val, rc);
	if (rc < 0)
		return 0;

	sensordata->actuator_info = kzalloc(sizeof(struct msm_actuator_info),
		GFP_KERNEL);
	if (!sensordata->actuator_info) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}

	sensordata->actuator_info->cam_name = val;

	rc = of_property_read_u32(of_node, "qcom,actuator-vcm-pwd", &val);
	CDBG("%s qcom,actuator-vcm-pwd %d, rc %d\n", __func__, val, rc);
	if (!rc)
		sensordata->actuator_info->vcm_pwd = val;

	rc = of_property_read_u32(of_node, "qcom,actuator-vcm-enable", &val);
	CDBG("%s qcom,actuator-vcm-enable %d, rc %d\n", __func__, val, rc);
	if (!rc)
		sensordata->actuator_info->vcm_enable = val;

	return 0;
ERROR:
	return rc;
}

static int32_t msm_sensor_init_sensor_data(struct platform_device *pdev,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, i = 0;
	uint32_t val = 0;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_sensor_platform_info *pinfo = NULL;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct msm_camera_sensor_info *sensordata = NULL;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;

	s_ctrl->sensordata = kzalloc(sizeof(struct msm_camera_sensor_info),
		GFP_KERNEL);
	if (!s_ctrl->sensordata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	sensordata = s_ctrl->sensordata;

	rc = of_property_read_string(of_node, "qcom,sensor-name",
		&sensordata->sensor_name);
	CDBG("%s qcom,sensor-name %s, rc %d\n", __func__,
		sensordata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR1;
	}

	rc = of_property_read_u32(of_node, "qcom,camera-type", &val);
	CDBG("%s qcom,camera-type %d, rc %d\n", __func__, val, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR1;
	}
	sensordata->camera_type = val;

	rc = of_property_read_u32(of_node, "qcom,sensor-type", &val);
	CDBG("%s qcom,sensor-type %d, rc %d\n", __func__, val, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR1;
	}
	sensordata->sensor_type = val;

	rc = msm_sensor_init_flash_data(of_node, sensordata);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR1;
	}

	sensordata->sensor_platform_info = kzalloc(sizeof(
		struct msm_camera_sensor_platform_info), GFP_KERNEL);
	if (!sensordata->sensor_platform_info) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR1;
	}

	pinfo = sensordata->sensor_platform_info;

	rc = of_property_read_u32(of_node, "qcom,mount-angle",
		&pinfo->mount_angle);
	CDBG("%s qcom,mount-angle %d, rc %d\n", __func__, pinfo->mount_angle,
		rc);
	if (rc < 0) {
		/* Set default mount angle */
		pinfo->mount_angle = 0;
		rc = 0;
	}

	rc = msm_sensor_init_csi_data(of_node, sensordata);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR2;
	}

	rc = msm_sensor_init_vreg_data(of_node, pinfo);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR3;
	}

	pinfo->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf),
		GFP_KERNEL);
	if (!pinfo->gpio_conf) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR4;
	}
	gconf = pinfo->gpio_conf;
	rc = of_property_read_u32(of_node, "qcom,gpio-no-mux",
		&gconf->gpio_no_mux);
	CDBG("%s gconf->gpio_no_mux %d, rc %d\n", __func__,
		gconf->gpio_no_mux, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR5;
	}

	gpio_array_size = of_gpio_count(of_node);
	CDBG("%s gpio count %d\n", __func__, gpio_array_size);

	if (gpio_array_size) {
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			GFP_KERNEL);
		if (!gpio_array) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR5;
		}
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("%s gpio_array[%d] = %d\n", __func__, i,
				gpio_array[i]);
		}

		rc = msm_sensor_init_gpio_common_tbl_data(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR5;
		}

		rc = msm_sensor_init_gpio_req_tbl_data(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR6;
		}

		rc = msm_sensor_init_gpio_set_tbl_data(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR7;
		}

		rc = msm_sensor_init_gpio_tlmm_tbl_data(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR8;
		}
	}
	rc = msm_sensor_init_actuator_data(of_node, sensordata);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR9;
	}

	kfree(gpio_array);
	return rc;

ERROR9:
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		camera_on_table);
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		camera_off_table);
ERROR8:
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		cam_gpio_set_tbl);
ERROR7:
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		cam_gpio_req_tbl);
ERROR6:
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		cam_gpio_common_tbl);
ERROR5:
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf);
ERROR4:
	kfree(s_ctrl->sensordata->sensor_platform_info->cam_vreg);
ERROR3:
	kfree(s_ctrl->sensordata->sensor_platform_info->csi_lane_params);
	kfree(s_ctrl->sensordata->pdata);
ERROR2:
	kfree(s_ctrl->sensordata->sensor_platform_info);
	kfree(s_ctrl->sensordata->flash_data);
ERROR1:
	kfree(s_ctrl->sensordata);
	kfree(gpio_array);
	return rc;
}

int32_t msm_sensor_free_sensor_data(struct msm_sensor_ctrl_t *s_ctrl)
{
	if (!s_ctrl->pdev)
		return 0;
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		camera_on_table);
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		camera_off_table);
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		cam_gpio_set_tbl);
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		cam_gpio_req_tbl);
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf->
		cam_gpio_common_tbl);
	kfree(s_ctrl->sensordata->sensor_platform_info->gpio_conf);
	kfree(s_ctrl->sensordata->sensor_platform_info->cam_vreg);
	kfree(s_ctrl->sensordata->sensor_platform_info->csi_lane_params);
	kfree(s_ctrl->sensordata->pdata);
	kfree(s_ctrl->sensordata->sensor_platform_info);
	kfree(s_ctrl->sensordata->flash_data);
	kfree(s_ctrl->sensordata);
	return 0;
}

int32_t msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	struct device *dev = NULL;

	//Eric Liu+
	if(s_ctrl->sensordata->camera_type == FRONT_CAMERA_2D)
		MSG2("%s: FRONT_CAMERA",__func__);
	else
		MSG2("%s: BACK_CAMERA",__func__);
	//Eric Liu-

	if (s_ctrl->sensor_device_type == MSM_SENSOR_PLATFORM_DEVICE)
		dev = &s_ctrl->pdev->dev;
	else
		dev = &s_ctrl->sensor_i2c_client->client->dev;
	s_ctrl->reg_ptr = kzalloc(sizeof(struct regulator *)
			* data->sensor_platform_info->num_vreg, GFP_KERNEL);
	if (!s_ctrl->reg_ptr) {
		pr_err("%s: could not allocate mem for regulators\n",
			__func__);
		return -ENOMEM;
	}

	rc = msm_camera_request_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		goto request_gpio_failed;
	}

	rc = msm_camera_config_vreg(dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: regulator on failed\n", __func__);
		goto config_vreg_failed;
	}

	rc = msm_camera_enable_vreg(dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 1);
	if (rc < 0) {
		pr_err("%s: enable regulator failed\n", __func__);
		goto enable_vreg_failed;
	}

	rc = msm_camera_config_gpio_table(data, 1);
	if (rc < 0) {
		pr_err("%s: config gpio failed\n", __func__);
		goto config_gpio_failed;
	}

	if (s_ctrl->sensor_device_type == MSM_SENSOR_I2C_DEVICE) {
		if (s_ctrl->clk_rate != 0)
			cam_8960_clk_info->clk_rate = s_ctrl->clk_rate;

		rc = msm_cam_clk_enable(dev, cam_8960_clk_info,
			s_ctrl->cam_clk, ARRAY_SIZE(cam_8960_clk_info), 1);
		if (rc < 0) {
			pr_err("%s: clk enable failed\n", __func__);
			goto enable_clk_failed;
		}
	} else {
		rc = msm_cam_clk_enable(dev, cam_8974_clk_info,
			s_ctrl->cam_clk, ARRAY_SIZE(cam_8974_clk_info), 1);
		if (rc < 0) {
			pr_err("%s: clk enable failed\n", __func__);
			goto enable_clk_failed;
		}
	}

	if (!s_ctrl->power_seq_delay)
		usleep_range(1000, 2000);
	else if (s_ctrl->power_seq_delay < 20)
		usleep_range((s_ctrl->power_seq_delay * 1000),
			((s_ctrl->power_seq_delay * 1000) + 1000));
	else
		msleep(s_ctrl->power_seq_delay);

	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(1);

	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		msm_sensor_enable_i2c_mux(data->sensor_platform_info->i2c_conf);

	if (s_ctrl->sensor_device_type == MSM_SENSOR_PLATFORM_DEVICE) {
		rc = msm_sensor_cci_util(s_ctrl->sensor_i2c_client,
			MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("%s cci_init failed\n", __func__);
			goto cci_init_failed;
		}
	}
	s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;

  //Eric Liu+, for skip the real i2c config
  if(s_ctrl->sensordata->camera_type == FRONT_CAMERA_2D)
  {
    front_cam_auto_white_balance = -1;
    front_cam_white_balance_temperature = -1;
    front_cam_exposure = -1;
    front_cam_band_stop_filter = -1;
  }
  #ifdef CONFIG_PM_LOG
  {
    int rc, type = s_ctrl->sensordata->camera_type;
    if(type == FRONT_CAMERA_2D && pmlog_device_cam_front != NULL)
    {
      rc = pmlog_device_on(pmlog_device_cam_front);
      if(rc)  MSG2("[PM_LOG]FRONT_CAMERA On, fail = %d",rc);
      //else    MSG2("[PM_LOG]FRONT_CAMERA On");
    }
    else if(type == BACK_CAMERA_2D && pmlog_device_cam_back != NULL)
    {
      rc = pmlog_device_on(pmlog_device_cam_back);
      if(rc)  MSG2("[PM_LOG]BACK_CAMERA On, fail = %d",rc);
      //else    MSG2("[PM_LOG]BACK_CAMERA On");
    }
  }
  #endif //CONFIG_PM_LOG
  //Eric Liu-

	return rc;

cci_init_failed:
	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		msm_sensor_disable_i2c_mux(
			data->sensor_platform_info->i2c_conf);
enable_clk_failed:
		msm_camera_config_gpio_table(data, 0);
config_gpio_failed:
	msm_camera_enable_vreg(dev,
			s_ctrl->sensordata->sensor_platform_info->cam_vreg,
			s_ctrl->sensordata->sensor_platform_info->num_vreg,
			s_ctrl->vreg_seq,
			s_ctrl->num_vreg_seq,
			s_ctrl->reg_ptr, 0);

enable_vreg_failed:
	msm_camera_config_vreg(dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
config_vreg_failed:
	msm_camera_request_gpio_table(data, 0);
request_gpio_failed:
	kfree(s_ctrl->reg_ptr);
	return rc;
}

int32_t msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;
	struct device *dev = NULL;

	//Eric Liu+
	if(s_ctrl->sensordata->camera_type == FRONT_CAMERA_2D)
		MSG2("%s: FRONT_CAMERA",__func__);
	else
		MSG2("%s: BACK_CAMERA",__func__);
	//Eric Liu-

	// sophia.wang++ 20130528
	pr_err("sensor_state:%d", s_ctrl->sensor_state);
	// if the sensor state already power down, we should return back
	if(s_ctrl->sensor_state == MSM_SENSOR_POWER_DOWN)
	{
		pr_err("this sensor has been closed\n");
			return 0;
	}
	// sophia wang--

	if (s_ctrl->sensor_device_type == MSM_SENSOR_PLATFORM_DEVICE)
		dev = &s_ctrl->pdev->dev;
	else
		dev = &s_ctrl->sensor_i2c_client->client->dev;
	if (s_ctrl->sensor_device_type == MSM_SENSOR_PLATFORM_DEVICE) {
		msm_sensor_cci_util(s_ctrl->sensor_i2c_client,
			MSM_CCI_RELEASE);
	}

	if (data->sensor_platform_info->i2c_conf &&
		data->sensor_platform_info->i2c_conf->use_i2c_mux)
		msm_sensor_disable_i2c_mux(
			data->sensor_platform_info->i2c_conf);

	if (data->sensor_platform_info->ext_power_ctrl != NULL)
		data->sensor_platform_info->ext_power_ctrl(0);
	if (s_ctrl->sensor_device_type == MSM_SENSOR_I2C_DEVICE)
		msm_cam_clk_enable(dev, cam_8960_clk_info, s_ctrl->cam_clk,
			ARRAY_SIZE(cam_8960_clk_info), 0);
	else
		msm_cam_clk_enable(dev, cam_8974_clk_info, s_ctrl->cam_clk,
			ARRAY_SIZE(cam_8974_clk_info), 0);
	msm_camera_config_gpio_table(data, 0);
	msm_camera_enable_vreg(dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
	msm_camera_config_vreg(dev,
		s_ctrl->sensordata->sensor_platform_info->cam_vreg,
		s_ctrl->sensordata->sensor_platform_info->num_vreg,
		s_ctrl->vreg_seq,
		s_ctrl->num_vreg_seq,
		s_ctrl->reg_ptr, 0);
	msm_camera_request_gpio_table(data, 0);
	kfree(s_ctrl->reg_ptr);
	s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;

  //Eric Liu+, for skip the real i2c config
  if(s_ctrl->sensordata->camera_type == FRONT_CAMERA_2D)
  {
    front_cam_auto_white_balance = -1;
    front_cam_white_balance_temperature = -1;
    front_cam_exposure = -1;
    front_cam_band_stop_filter = -1;
  }
  #ifdef CONFIG_PM_LOG
  {
    int rc, type = s_ctrl->sensordata->camera_type;
    if(type == FRONT_CAMERA_2D && pmlog_device_cam_front != NULL)
    {
      rc = pmlog_device_off(pmlog_device_cam_front);
      if(rc)  MSG2("[PM_LOG]FRONT_CAMERA Off, fail = %d",rc);
      //else    MSG2("[PM_LOG]FRONT_CAMERA Off");
    }
    else if(type == BACK_CAMERA_2D && pmlog_device_cam_back != NULL)
    {
      rc = pmlog_device_off(pmlog_device_cam_back);
      if(rc)  MSG2("[PM_LOG]BACK_CAMERA Off, fail = %d",rc);
      //else    MSG2("[PM_LOG]BACK_CAMERA Off");
    }
  }
  #endif //CONFIG_PM_LOG
  //Eric Liu-

	return 0;
}

int32_t msm_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = msm_camera_i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_id_info->sensor_id_reg_addr, &chipid,
			MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	pr_err("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensor_id_info->sensor_id);
	if (chipid != s_ctrl->sensor_id_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}

	// sophia wang++ for ftd 
	{
		int type = s_ctrl->sensordata->camera_type;
		if(type == FRONT_CAMERA_2D)
		{
			MSG2("%s, Front = %04X (%s)",__func__,
				chipid,s_ctrl->sensordata->sensor_name);
			cam_id_front = chipid;
			//mt9m114, read Fuse Id
			if(s_ctrl->sensor_id_info->sensor_id == 0x2481)
			{
				int32_t rc1 = 0, rc2 = 0, rc3 = 0, rc4 = 0;
				uint16_t id1 = 0, id2 = 0, id3 = 0, id4 = 0;
				rc1 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x31F4, &id1, MSM_CAMERA_I2C_WORD_DATA); //fuse_id1
				rc2 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x31F6, &id2, MSM_CAMERA_I2C_WORD_DATA); //fuse_id2
				rc3 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x31F8, &id3, MSM_CAMERA_I2C_WORD_DATA); //fuse_id3
				rc4 = msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x31FA, &id4, MSM_CAMERA_I2C_WORD_DATA); //fuse_id4
				if(rc1 < 0 || rc2 < 0 || rc3 < 0 || rc4 < 0)
				{
					MSG2("%s, Front Fuse Id read Fail!",__func__);
				}
				else
				{
					//format = fuse_id4, fuse_id3, fuse_id2, fuse_id1
					cam_fuse_id_front = id4;
					cam_fuse_id_front = (cam_fuse_id_front << 16) + id3;
					cam_fuse_id_front = (cam_fuse_id_front << 16) + id2;
					cam_fuse_id_front = (cam_fuse_id_front << 16) + id1;
					MSG2("%s, Front Fuse Id = 0x%016llX",__func__,cam_fuse_id_front);
				}
			}
			else if(s_ctrl->sensor_id_info->sensor_id == 0x2656) // ov2675
			{

				if(s_ctrl->func_tbl->sensor_get_fuse_id != NULL)
				{
					cam_fuse_id_front = s_ctrl->func_tbl->sensor_get_fuse_id(s_ctrl); 

				}

  				if(s_ctrl->func_tbl->sensor_read_lens_id != NULL)
     					s_ctrl->func_tbl->sensor_read_lens_id(s_ctrl);
			}

		} else if(type == BACK_CAMERA_2D)
		{
			MSG2("%s, Back  = %04X (%s)",__func__,
				chipid,s_ctrl->sensordata->sensor_name);
			cam_id_back = chipid;
		}
		else
		{
			MSG2("%s, Cam Id = %04X (type = %d)",__func__,chipid,type);
		}
	}
	// sophia wang -- for ftd
	
	return rc;
}

int32_t msm_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;
	CDBG("%s %s_i2c_probe called\n", __func__, client->name);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s %s i2c_check_functionality failed\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	s_ctrl->sensor_device_type = MSM_SENSOR_I2C_DEVICE;
	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		if (s_ctrl->sensor_i2c_addr != 0)
			s_ctrl->sensor_i2c_client->client->addr =
				s_ctrl->sensor_i2c_addr;
	} else {
		pr_err("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

  //Eric Liu+
  //base on project and board id, bypass non-usable camera, 20130628
  //Boston EVB0 ~ PVT0:               use ov8825
  //Boston CASPER_EVT1 ~ CASPER_DVT0: use imx091
  MSG2("%s, msm_project_id=%d, system_rev=0x%X",__func__,msm_project_id,system_rev);
  if(msm_project_id  == BOSTON && system_rev >= CASPER_EVT1)
  {
    if(s_ctrl->sensor_id_info->sensor_id == 0x8825)
    {
      MSG2("%s, bypass OV8825",__func__);
      return -ENODEV;
    }
  }
  else
  if(msm_project_id == BOSTON && system_rev < CASPER_EVT1)
  {
    if(s_ctrl->sensor_id_info->sensor_id == 0x0091)
    {
      MSG2("%s, bypass IMX091",__func__);
      return -ENODEV;
    }
  }
  else // sophia, 20130711, 
  {
   if(s_ctrl->sensor_id_info->sensor_id == 0x0091)
    {
      MSG2("%s, bypass IMX091",__func__);
      return -ENODEV;
    }


  }

  //if a back cam already probed success, bypass another back cam probe
  if(msm_project_id >= BOSTON &&
    (s_ctrl->sensor_id_info->sensor_id == 0x8825 || s_ctrl->sensor_id_info->sensor_id == 0x0091) &&
    (pmlog_device_cam_back != NULL) )
  {
    MSG2("%s, pmlog_device_cam_back=%08X, bypass %04X",__func__,
      (int)pmlog_device_cam_back,s_ctrl->sensor_id_info->sensor_id);
    return -ENODEV;
  }

  //base on board id, pick correct vreg_seq, 20130628
  //Boston EVB0 ~ EVT0:               use vreg evt0
  //Boston EVT1 ~ EVT1_2:             use vreg evt1
  //Boston EVT1_3 ~ PVT0:             use vreg evt
  //Boston CASPER_EVT1 ~ CASPER_DVT0: use vreg evt1
  if(msm_project_id == BOSTON && system_rev <= EVT0)  //use vreg_seq_evt0
  {
    if(s_ctrl->vreg_seq_evt0 && s_ctrl->num_vreg_seq_evt0)
    {
      s_ctrl->vreg_seq      = s_ctrl->vreg_seq_evt0;
      s_ctrl->num_vreg_seq  = s_ctrl->num_vreg_seq_evt0;
      MSG2("%s, sensor_id=%04X, use vreg_seq_evt0",__func__,s_ctrl->sensor_id_info->sensor_id);
    }
  }
  else if(msm_project_id == BOSTON && //use vreg_seq_evt1
    (system_rev <= EVT1_2 || system_rev >= CASPER_EVT1))
  {
    if(s_ctrl->vreg_seq_evt1 && s_ctrl->num_vreg_seq_evt1)
    {
      s_ctrl->vreg_seq      = s_ctrl->vreg_seq_evt1;
      s_ctrl->num_vreg_seq  = s_ctrl->num_vreg_seq_evt1;
      MSG2("%s, sensor_id=%04X, use vreg_seq_evt1",__func__,s_ctrl->sensor_id_info->sensor_id);
    }
  }
  else  //use vreg_seq_evt
  {
    MSG2("%s, id %04X, use vreg_seq",__func__,s_ctrl->sensor_id_info->sensor_id);
  }
  //Eric Liu-

	s_ctrl->sensordata = client->dev.platform_data;
	if (s_ctrl->sensordata == NULL) {
		pr_err("%s %s NULL sensor data\n", __func__, client->name);
		return -EFAULT;
	}

	rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s %s power up failed\n", __func__, client->name);
		return rc;
	}

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0)
		goto probe_fail;

  // sophia wang++, 20121127, read OTP releated data
  if(s_ctrl->sensor_id_info->sensor_id == 0x8825 &&
    s_ctrl->func_tbl->sensor_read_otp_mid != NULL &&
    initOpt == false)
	{
#if 0
    uint8_t *awb_calib_data  = (uint8_t *)(s_ctrl->otp_info->data_tbl[0].data);
    uint8_t *af_calib_data  = (uint8_t *)(s_ctrl->otp_info->data_tbl[1].data);
#endif	       
		uint8_t mid_id;
		
    s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);

    mid_id = s_ctrl->func_tbl->sensor_read_otp_mid(s_ctrl);
#if 1    
    back_sensor_calied_awb_d65_rg = s_ctrl->otp_info->rg_ratio_d65;
    back_sensor_calied_awb_d65_bg = s_ctrl->otp_info->bg_ratio_d65;
    back_sensor_calied_awb_cwf_rg = s_ctrl->otp_info->rg_ratio_cwf;
    back_sensor_calied_awb_cwf_bg = s_ctrl->otp_info->bg_ratio_cwf;
    back_sensor_calied_awb_u30_rg = s_ctrl->otp_info->rg_ratio_u30;
    back_sensor_calied_awb_u30_bg = s_ctrl->otp_info->bg_ratio_u30;

    back_sensor_calied_af_hori_msb = s_ctrl->otp_info->ov8825_af_otp[0];
    back_sensor_calied_af_hori_lsb = s_ctrl->otp_info->ov8825_af_otp[1];
    back_sensor_calied_af_ma_msb =s_ctrl->otp_info->ov8825_af_otp[2];
    back_sensor_calied_af_ma_lsb = s_ctrl->otp_info->ov8825_af_otp[3];

    back_sensor_calied_production_data_year = s_ctrl->otp_info->year;
    back_sensor_calied_production_data_mouth = s_ctrl->otp_info->mouth;
    back_sensor_calied_production_data_date = s_ctrl->otp_info->date;
    back_sensor_calied_production_data_hour = s_ctrl->otp_info->hour;
    back_sensor_calied_production_data_minuite = s_ctrl->otp_info->minuite;
    back_sensor_calied_production_data_second = s_ctrl->otp_info->second;

    back_sensor_calied_production_project_id = (s_ctrl->otp_info->lens_id >>6)&0x3;
#endif

    // sophia wang++, 20120727, if there is two module for the same sensor, we need to check mid id       
    if(mid_id != 0x2)
    {
      pr_err("%s, proble failed! mid id is not matched  mid_id:0x%x, \n", __func__,
        mid_id);
      goto probe_fail;
    }
    // sophia wang -- 20120727       

    initOpt = true;
 	}
  // sophia wang++, 20121127, read OTP releated data

  //Eric Liu+, for IMX091
  if(s_ctrl->sensor_id_info->sensor_id == 0x0091 &&
    s_ctrl->func_tbl->sensor_read_otp_mid != NULL)
  {
    s_ctrl->func_tbl->sensor_read_otp_mid(s_ctrl);
  }
  //Eric Liu-
	
	if (!s_ctrl->wait_num_frames)
		s_ctrl->wait_num_frames = 1 * Q10;

	pr_err("%s %s probe succeeded\n", __func__, client->name);
	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s", id->name);
	v4l2_i2c_subdev_init(&s_ctrl->sensor_v4l2_subdev, client,
		s_ctrl->sensor_v4l2_subdev_ops);
	s_ctrl->sensor_v4l2_subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->sensor_v4l2_subdev.entity, 0, NULL, 0);
	s_ctrl->sensor_v4l2_subdev.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->sensor_v4l2_subdev.entity.group_id = SENSOR_DEV;
	s_ctrl->sensor_v4l2_subdev.entity.name =
		s_ctrl->sensor_v4l2_subdev.name;
	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);
	s_ctrl->sensor_v4l2_subdev.entity.revision =
		s_ctrl->sensor_v4l2_subdev.devnode->num;


	msm_sensor_enable_debugfs(s_ctrl);
	
	// sophia wang++
	if(s_ctrl->sensor_id_info->sensor_id == 0x8825)
	{
		rc=cam_create_sys_entries(client);
		if (rc<0) goto probe_fail;
	}
	// sophia wang--
	
	goto power_down;
probe_fail:
	pr_err("%s %s_i2c_probe failed\n", __func__, client->name);
power_down:
	if (rc > 0)
		rc = 0;
	s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;

  //Eric Liu+
  #ifdef CONFIG_PM_LOG
  if(!rc)
  {
    int type = s_ctrl->sensordata->camera_type;
    if(type == FRONT_CAMERA_2D && pmlog_device_cam_front == NULL)
    {
      pmlog_device_cam_front = pmlog_register_device(&client->dev);
      MSG2("[PM_LOG]FRONT_CAMERA, device = %08X",(int)pmlog_device_cam_front);
    }
    else if(type == BACK_CAMERA_2D && pmlog_device_cam_back == NULL)
    {
      pmlog_device_cam_back = pmlog_register_device(&client->dev);
      MSG2("[PM_LOG]BACK_CAMERA, device= %08X",(int)pmlog_device_cam_back);
    }
  }
  #endif //CONFIG_PM_LOG
  //Eric Liu-

	return rc;
}

static int msm_sensor_subdev_match_core(struct device *dev, void *data)
{
	int core_index = (int)data;
	struct platform_device *pdev = to_platform_device(dev);
	CDBG("%s cci pdev %p\n", __func__, pdev);
	if (pdev->id == core_index)
		return 1;
	else
		return 0;
}

int32_t msm_sensor_platform_probe(struct platform_device *pdev, void *data)
{
	int32_t rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = (struct msm_sensor_ctrl_t *)data;
	struct device_driver *driver;
	struct device *dev;
	s_ctrl->pdev = pdev;
	CDBG("%s called data %p\n", __func__, data);
	if (pdev->dev.of_node) {
		rc = msm_sensor_init_sensor_data(pdev, s_ctrl);
		if (rc < 0) {
			pr_err("%s failed line %d\n", __func__, __LINE__);
			return rc;
		}
	}
	s_ctrl->sensor_device_type = MSM_SENSOR_PLATFORM_DEVICE;
	s_ctrl->sensor_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!s_ctrl->sensor_i2c_client->cci_client) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}
	driver = driver_find(MSM_CCI_DRV_NAME, &platform_bus_type);
	if (!driver) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}

	dev = driver_find_device(driver, NULL, 0,
				msm_sensor_subdev_match_core);
	if (!dev) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}
	s_ctrl->sensor_i2c_client->cci_client->cci_subdev =
		dev_get_drvdata(dev);
	CDBG("%s sd %p\n", __func__,
		s_ctrl->sensor_i2c_client->cci_client->cci_subdev);
	s_ctrl->sensor_i2c_client->cci_client->cci_i2c_master = MASTER_0;
	s_ctrl->sensor_i2c_client->cci_client->sid =
		s_ctrl->sensor_i2c_addr >> 1;
	s_ctrl->sensor_i2c_client->cci_client->retries = 3;
	s_ctrl->sensor_i2c_client->cci_client->id_map = 0;

	rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s %s power up failed\n", __func__,
			pdev->id_entry->name);
		return rc;
	}

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0)
		goto probe_fail;

	v4l2_subdev_init(&s_ctrl->sensor_v4l2_subdev,
		s_ctrl->sensor_v4l2_subdev_ops);
	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s",
		s_ctrl->sensordata->sensor_name);
	v4l2_set_subdevdata(&s_ctrl->sensor_v4l2_subdev, pdev);
	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);

	goto power_down;
probe_fail:
	pr_err("%s %s probe failed\n", __func__, pdev->id_entry->name);
power_down:
	s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	return rc;
}

int32_t msm_sensor_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (on) {
		rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		if (rc < 0) {
			pr_err("%s: %s power_up failed rc = %d\n", __func__,
				s_ctrl->sensordata->sensor_name, rc);
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
		} else {
			if (s_ctrl->func_tbl->sensor_match_id)
				rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
			else
				rc = msm_sensor_match_id(s_ctrl);
			if (rc < 0) {
				pr_err("%s: %s match_id failed  rc=%d\n",
					__func__,
					s_ctrl->sensordata->sensor_name, rc);
				if (s_ctrl->func_tbl->sensor_power_down(s_ctrl)
					< 0)
					pr_err("%s: %s power_down failed\n",
					__func__,
					s_ctrl->sensordata->sensor_name);
				s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
		}
	} else {
		rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return rc;
}

int32_t msm_sensor_v4l2_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);

	if ((unsigned int)index >= s_ctrl->sensor_v4l2_subdev_info_size)
		return -EINVAL;

	*code = s_ctrl->sensor_v4l2_subdev_info[index].code;
	return 0;
}

int32_t msm_sensor_v4l2_s_ctrl(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	int rc = -1, i = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	struct msm_sensor_v4l2_ctrl_info_t *v4l2_ctrl =
		s_ctrl->msm_sensor_v4l2_ctrl_info;

	CDBG("%s\n", __func__);
	CDBG("%d\n", ctrl->id);
	if (v4l2_ctrl == NULL)
		return rc;

  //Eric Liu+, for skip the real i2c config
  switch(ctrl->id)
  {
    case V4L2_CID_AUTO_WHITE_BALANCE:
      if(front_cam_auto_white_balance == ctrl->value)
        return 0;
      MSG2("%s, WHITE_BALANCE = %s",__func__,
        ctrl->value == 0 ? "OFF" :
        ctrl->value == 1 ? "AUTO" : "???");
      front_cam_auto_white_balance = ctrl->value;
      front_cam_white_balance_temperature = -1;
      break;
    case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
      if(front_cam_white_balance_temperature == ctrl->value)
        return 0;
      MSG2("%s, WHITE_BALANCE = %s",__func__,
        ctrl->value == 3 ? "INCANDESCENT" :
        ctrl->value == 4 ? "FLUORESCENT" :
        ctrl->value == 5 ? "DAYLIGHT" :
        ctrl->value == 6 ? "CLOUDY_DAYLIGHT" : "???");
      front_cam_white_balance_temperature = ctrl->value;
      front_cam_auto_white_balance = -1;
      break;
    case V4L2_CID_EXPOSURE:
      if(front_cam_exposure == ctrl->value)
        return 0;
      MSG2("%s, EXPOSURE      = %s",__func__,
        ctrl->value == 0 ? "-2" :
        ctrl->value == 1 ? "-1" :
        ctrl->value == 2 ? " 0" :
        ctrl->value == 3 ? " 1" :
        ctrl->value == 4 ? " 2" : "???");
      front_cam_exposure = ctrl->value;
      break;
   // case V4L2_CID_BAND_STOP_FILTER:
    case V4L2_CID_POWER_LINE_FREQUENCY:
      if(front_cam_band_stop_filter == ctrl->value)
        return 0;
      MSG2("%s, BAND_STOP     = %s",__func__,
        ctrl->value == 1 ? "60Hz" :
        ctrl->value == 2 ? "50Hz" : "???");
      front_cam_band_stop_filter = ctrl->value;
      break;

      case V4L2_CID_SHARPNESS:
      	if (front_cam_sharpness == ctrl->value)
      		return 0;
      MSG2("%s, sharpness     = %d",__func__, ctrl->value);
      front_cam_sharpness = ctrl->value;
      break;
      
    default:
      MSG2("%s, id=%08X, = %X",__func__,ctrl->id,ctrl->value);
      break;
  }
  //Eric Liu-
	for (i = 0; i < s_ctrl->num_v4l2_ctrl; i++) {
		CDBG("%s, ctrl->id = 0x%x,  v4l2_ctrl[i].ctrl_id = 0x%x\n", __func__, ctrl->id, v4l2_ctrl[i].ctrl_id);
		if (v4l2_ctrl[i].ctrl_id == ctrl->id) {
			if (v4l2_ctrl[i].s_v4l2_ctrl != NULL) {
				CDBG("\n calling msm_sensor_s_ctrl_by_enum\n");
				rc = v4l2_ctrl[i].s_v4l2_ctrl(
					s_ctrl,
					&s_ctrl->msm_sensor_v4l2_ctrl_info[i],
					ctrl->value);
			}
			break;
		}
	}

	return rc;
}

int32_t msm_sensor_v4l2_query_ctrl(
	struct v4l2_subdev *sd, struct v4l2_queryctrl *qctrl)
{
	int rc = -1, i = 0;
	//Eric Liu+, Get correct sensor control
	//struct msm_sensor_ctrl_t *s_ctrl =
	//	(struct msm_sensor_ctrl_t *) sd->dev_priv;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	//Eric Liu-

	CDBG("%s\n", __func__);
	CDBG("%s id: %d\n", __func__, qctrl->id);

	if (s_ctrl->msm_sensor_v4l2_ctrl_info == NULL)
		return rc;

	for (i = 0; i < s_ctrl->num_v4l2_ctrl; i++) {
		if (s_ctrl->msm_sensor_v4l2_ctrl_info[i].ctrl_id == qctrl->id) {
			qctrl->minimum =
				s_ctrl->msm_sensor_v4l2_ctrl_info[i].min;
			qctrl->maximum =
				s_ctrl->msm_sensor_v4l2_ctrl_info[i].max;
			qctrl->flags = 1;
			rc = 0;
			break;
		}
	}

	return rc;
}

int msm_sensor_s_ctrl_by_enum(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_sensor_v4l2_ctrl_info_t *ctrl_info, int value)
{
	int rc = 0;
	CDBG("%s enter\n", __func__);
	rc = msm_sensor_write_enum_conf_array(
		s_ctrl->sensor_i2c_client,
		ctrl_info->enum_cfg_settings, value);
	return rc;
}


static int msm_sensor_debugfs_read_otp(void *data, u64 val)
{
	struct msm_sensor_ctrl_t *s_ctrl = (struct msm_sensor_ctrl_t *) data;
	

		if( (s_ctrl->func_tbl->sensor_get_fuse_id != NULL) && (s_ctrl->sensordata->camera_type == FRONT_CAMERA_2D))
		{

			s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	       	s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
	   	       
		        cam_fuse_id_front = s_ctrl->func_tbl->sensor_get_fuse_id(s_ctrl); 

			s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	       	s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
		        		
		}


	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(sensor_debugfs_read_otp, NULL,
			msm_sensor_debugfs_read_otp, "%llu\n");

static int msm_sensor_debugfs_write_otp(void *data, u64 val)
{
	struct msm_sensor_ctrl_t *s_ctrl = (struct msm_sensor_ctrl_t *) data;
	
	if (s_ctrl->func_tbl->sensor_writ_otp != NULL)
	{
		s_ctrl->func_tbl->sensor_writ_otp(s_ctrl, val);


#if 0
		if( (s_ctrl->func_tbl->sensor_get_fuse_id != NULL) && (s_ctrl->sensordata->camera_type == FRONT_CAMERA_2D))
		{
		        cam_fuse_id_front = s_ctrl->func_tbl->sensor_get_fuse_id(s_ctrl); 
		
		}
#endif
		
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sensor_debugfs_write_otp, NULL,
			msm_sensor_debugfs_write_otp, "%llu\n");


static int msm_sensor_debugfs_power_s(void *data, u64 val)
{
	struct msm_sensor_ctrl_t *s_ctrl = (struct msm_sensor_ctrl_t *) data;
	if (val)
	{
		s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	       s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
	       pr_err("%s, s_ctrl->sensor_state:%d\n", __func__, s_ctrl->sensor_state);
	}
	else
	{
		s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	       s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sensor_debugfs_power, NULL,
			msm_sensor_debugfs_power_s, "%llu\n");


static int msm_sensor_debugfs_stream_s(void *data, u64 val)
{
	struct msm_sensor_ctrl_t *s_ctrl = (struct msm_sensor_ctrl_t *) data;
	if (val)
	{
		s_ctrl->func_tbl->sensor_start_stream(s_ctrl);

	}
	else
	{
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);


	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sensor_debugfs_stream, NULL,
			msm_sensor_debugfs_stream_s, "%llu\n");

static int msm_sensor_debugfs_test_s(void *data, u64 val)
{
	CDBG("val: %llu\n", val);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sensor_debugfs_test, NULL,
			msm_sensor_debugfs_test_s, "%llu\n");

int msm_sensor_enable_debugfs(struct msm_sensor_ctrl_t *s_ctrl)
{
	static struct dentry *debugfs_base = NULL; 
	struct dentry *sensor_dir;

       if(debugfs_base == NULL)
       {
		debugfs_base = debugfs_create_dir("msm_sensor", NULL);
		if (!debugfs_base)
			return -ENOMEM;
       }

       
	sensor_dir = debugfs_create_dir
		(s_ctrl->sensordata->sensor_name, debugfs_base);
	if (!sensor_dir)
		return -ENOMEM;

	if (!debugfs_create_file("read_otp", S_IRUGO | S_IWUSR, sensor_dir,
			(void *) s_ctrl, &sensor_debugfs_read_otp))
		return -ENOMEM; 

	if (!debugfs_create_file("write_otp", S_IRUGO | S_IWUSR, sensor_dir,
			(void *) s_ctrl, &sensor_debugfs_write_otp))
		return -ENOMEM;      

	if (!debugfs_create_file("power", S_IRUGO | S_IWUSR, sensor_dir,
			(void *) s_ctrl, &sensor_debugfs_power))
		return -ENOMEM;

	if (!debugfs_create_file("stream", S_IRUGO | S_IWUSR, sensor_dir,
			(void *) s_ctrl, &sensor_debugfs_stream))
		return -ENOMEM;

	if (!debugfs_create_file("test", S_IRUGO | S_IWUSR, sensor_dir,
			(void *) s_ctrl, &sensor_debugfs_test))
		return -ENOMEM;

	return 0;
}

//Eric Liu+, for ftd read cam id
static int msm_sensor_ftd_s(void *data, u64 *val)
{
  int type = (int)data;
  switch(type)
  {
    case BACK_CAMERA_2D:
      *val = cam_id_back;
      break;
    case FRONT_CAMERA_2D:
      *val = cam_id_front;
      break;
    case (FRONT_CAMERA_2D + 10):
      *val = cam_fuse_id_front;
      break;
    default:
      MSG2("%s, invalid type = %d",__func__,type);
      *val = 0;
      break;
  }
	return 0;
}

static int ov8825_ftd_read_af_info(void *data, u64 *val)
{
  int type = (int)data;
  switch(type)
  {
    // check whether af is done
    case 0:
      *val = ov8825_af_done;
      break;


     // check af result 
    case 1:
      *val = ov8825_af_result;
      break;


    default:
      MSG2("%s, invalid type = %d",__func__,type);
      *val = 0xff;
      break;
  }
       CDBG("%s: type:%d, value:%d\n", __func__, type, (int)(*val));
	return 0;
}

int ov8825_ftd_write_af_s(void *parm, u64 mode)
{

   int type = (int)parm;

    switch(type)
    {
       case 0:
	ov8825_af_done = mode;
       break;

       case 1:
	ov8825_af_result = mode;
	break;

   }
       CDBG("%s: type:%d, mode:%d\n", __func__, type, (int)mode);
	return 0;
}    


DEFINE_SIMPLE_ATTRIBUTE(msm_sensor_ftd, msm_sensor_ftd_s, NULL, "%llu\n");

//Sophia Wang++, 20120606 for auto camera test
DEFINE_SIMPLE_ATTRIBUTE(ov8825_ftd_read, ov8825_ftd_read_af_info, NULL, "%lld\n"); // ov8820 read af ino function
DEFINE_SIMPLE_ATTRIBUTE(ov8825_ftd_write_af, NULL, ov8825_ftd_write_af_s, "%llu\n"); // ov8820 read af ino function
//DEFINE_SIMPLE_ATTRIBUTE(ov8820_ftd_write_af_result, NULL, ov8820_ftd_write_af_s, "%llu\n");
//Sophia Wang--, 20120606 for auto camera test

static int __init msm_sensor_ftd_init(void)
{
  struct dentry *debugfs_base;
  debugfs_base = debugfs_create_dir("msm_sensor_ftd", 0);
  MSG2("%s+",__func__);
  if (IS_ERR(debugfs_base))
    return -ENOMEM;
  debugfs_create_file("cam_id_front", 0444, debugfs_base, (void *)FRONT_CAMERA_2D, &msm_sensor_ftd);
  debugfs_create_file("cam_id_back",  0444, debugfs_base, (void *)BACK_CAMERA_2D, &msm_sensor_ftd);
  debugfs_create_file("cam_fuse_id_front",  0444, debugfs_base, (void *)FRONT_CAMERA_2D + 10, &msm_sensor_ftd);

  debugfs_base = debugfs_create_dir("cam_back_ftd", 0);
  if (IS_ERR(debugfs_base))
  {
    MSG2("%s, create ov8820 node failed!\n", __func__);
    return -ENOMEM;
  }
  debugfs_create_file("read_af_done", 0777, debugfs_base, (void*)0, &ov8825_ftd_read);
  debugfs_create_file("read_af_result", 0777, debugfs_base, (void*)1, &ov8825_ftd_read);
  debugfs_create_file("write_af_done", 0777, debugfs_base, (void*)0, &ov8825_ftd_write_af);
  debugfs_create_file("write_af_result", 0777, debugfs_base, (void *)1, &ov8825_ftd_write_af);

  
  MSG2("%s-",__func__);

	return 0;
}

module_init(msm_sensor_ftd_init);
//Sophia Wang--, 20120606 for auto camera test
