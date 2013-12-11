/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#ifndef MIPI_TRUST_NT35516_H
#define MIPI_TRUST_NT35516_H

#if defined(CONFIG_FB_MSM_BACKLIGHT_LCMPWM)
#define MIPI_TRUST_NT35516_PWM_LEVEL 255
#else
// PM8038 support 0~255 levels
#define MIPI_TRUST_NT35516_PWM_LEVEL 255
// 20130321 Jackie, SW workaround for PM8038 WLED output current flick issue.
#define PM8038_WLED_OUTPUT_WA
#if defined(PM8038_WLED_OUTPUT_WA)
#define PM8038_WLED_PWM_MIN_LEVEL 11
#endif // end of PM8038_WLED_OUTPUT_WA
#endif // end of CONFIG_FB_MSM_BACKLIGHT_LCMPWM

int mipi_trust_nt35516_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);

#endif  /* MIPI_TRUST_NT35516_H */
