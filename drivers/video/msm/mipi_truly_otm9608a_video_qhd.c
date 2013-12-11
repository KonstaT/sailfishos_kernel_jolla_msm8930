/* Copyright (c) 2010-2011, The Linux Foundation. All rights reserved.
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_truly_otm9608a.h"

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
/* DSI_BIT_CLK at 500MHz, 2 lane, RGB888 */
		{0x03, 0x0a, 0x04, 0x00, 0x20},	/* regulator */
		/* timing   */
		{0x8B, 0x1F, 0x14, 0x00, 0x45, 0x4A, 0x19,
		0x23, 0x23, 0x03, 0x04},
		{0x5f, 0x00, 0x00, 0x10},	/* phy ctrl */
		{0xff, 0x00, 0x06, 0x00},	/* strength */
		/* pll control */
		{0x01, 0xc5, 0x31, 0xda, 0x00, 0x10, 0x08, 0x62,
		0x71, 0x0f, 0x07,
		0x00, 0x14, 0x03, 0x0, 0x2, 0x0, 0x20, 0x0, 0x01, 0x0},
};

static int __init mipi_video_truly_otm9608a_qhd_init(void)
{
	int ret;

	if (msm_fb_detect_client("mipi_video_truly_otm9608a_qhd"))
		return 0;

	pinfo.xres = 540;
	pinfo.yres = 960;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 20;
	pinfo.lcdc.h_front_porch = 40;
	pinfo.lcdc.h_pulse_width = 10;
	pinfo.lcdc.v_back_porch = 4;
	pinfo.lcdc.v_front_porch = 4;
	pinfo.lcdc.v_pulse_width = 2;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = MIPI_TRULY_OTM9608A_PWM_LEVEL;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	//pinfo.clk_rate = 454000000;
	pinfo.lcd.v_back_porch = 4;
	pinfo.lcd.v_front_porch = 4;
	pinfo.lcd.v_pulse_width = 2;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = FALSE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.esc_byte_ratio = 4;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x04;
	pinfo.mipi.t_clk_pre = 0x1B;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;

	pinfo.mipi.tx_eot_append = 0x01;
	pinfo.mipi.rx_eot_ignore = 0x0;
	pinfo.mipi.dlane_swap = 0x01;
	pinfo.mipi.frame_rate = 57;

	ret = mipi_truly_otm9608a_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_QHD_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_truly_otm9608a_qhd_init);
