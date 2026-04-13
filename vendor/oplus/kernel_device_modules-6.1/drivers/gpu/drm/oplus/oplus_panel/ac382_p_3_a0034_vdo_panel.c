// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"

#include "../../oplus/oplus_display_mtk_debug.h"
#define CONFIG_MTK_PANEL_EXT
#include "../../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
//#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#include "../../mediatek/mediatek_v2/mtk_dsi.h"
#include "../../mediatek/mediatek_v2/mtk-cmdq-ext.h"

#ifdef OPLUS_FEATURE_DISPLAY
#include "../../oplus/oplus_drm_disp_panel.h"
#include "../../oplus/oplus_display_temp_compensation.h"
#include "../../oplus/oplus_display_mtk_debug.h"
#endif /* OPLUS_FEATURE_DISPLAY */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "../../oplus/oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/of_address.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include "ac382_p_3_a0034_vdo_panel.h"

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr1p2_enable_gpio;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	bool hbm_en;
	bool hbm_wait;
	int error;
};
//static unsigned int temp_seed_mode = 0;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
//extern unsigned long seed_mode;
static int current_fps = 120;
static bool aod_state = false;
static unsigned int lhbm_last_backlight = 0;

#define MAX_NORMAL_BRIGHTNESS   3050
#define LCM_BRIGHTNESS_TYPE 2

extern void lcdinfo_notify(unsigned long val, void *v);
extern int oplus_serial_number_probe(struct device *dev);

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void lcm_panel_init(struct lcm *ctx)
{
	pr_info("debug for %s+\n", __func__);

	//DSC Setting(2SliceX22_10b_d375_v12)
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x07);
	lcm_dcs_write_seq_static(ctx, 0x8A,0x01);  //pps enable
	lcm_dcs_write_seq_static(ctx, 0x8B,0xA0);  //pps table
	lcm_dcs_write_seq_static(ctx, 0x80,0x00,0x00,0x00,0x12,0x00,0x00,0xab,0x30,0x80,0x0a,0xd4,0x04,0xf8,0x00,0x16,0x02,0x7c,0x02,0x7c,0x02,0x00,0x02,0x59,0x00,0x20,0x02,0x2e,0x00,0x08,0x00,0x0d,0x04,0xf4,0x03,0xed,0x18,0x00,0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,0x3b,0x78,0x3b,0xb6,0x4b,0xb6,0x4b,0xf4,0x4b,0xf4,0x6c,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0x81,0x00,0x00,0x00,0x12,0x00,0x00,0xab,0x30,0x80,0x0a,0xd4,0x04,0xf8,0x00,0x16,0x02,0x7c,0x02,0x7c,0x02,0x00,0x02,0x59,0x00,0x20,0x02,0x2e,0x00,0x08,0x00,0x0d,0x04,0xf4,0x03,0xed,0x18,0x00,0x10,0xf0,0x07,0x10,0x20,0x00,0x06,0x0f,0x0f,0x33,0x0e,0x1c,0x2a,0x38,0x46,0x54,0x62,0x69,0x70,0x77,0x79,0x7b,0x7d,0x7e,0x02,0x02,0x22,0x00,0x2a,0x40,0x2a,0xbe,0x3a,0xfc,0x3a,0xfa,0x3a,0xf8,0x3b,0x38,0x3b,0x78,0x3b,0xb6,0x4b,0xb6,0x4b,0xf4,0x4b,0xf4,0x6c,0x34,0x84,0x74,0x00,0x00,0x00,0x00,0x00,0x00);

	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x07);
	lcm_dcs_write_seq_static(ctx, 0xA0,0x24);
	//From整机调试
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x07);
	lcm_dcs_write_seq_static(ctx, 0xC0,0x14);
	lcm_dcs_write_seq_static(ctx, 0xC2,0x10);
	lcm_dcs_write_seq_static(ctx, 0xC3,0x40);
	lcm_dcs_write_seq_static(ctx, 0xC4,0x01);

	//Sleep out don't reload OTP
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x08);
	lcm_dcs_write_seq_static(ctx, 0xC8,0x62);

	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x00);
	//IRC ON
	lcm_dcs_write_seq_static(ctx, 0xB9,0x01);
	//set fps
	if (current_fps == 60) {
		lcm_dcs_write_seq_static(ctx, 0xD0,0x20);
	} else if (current_fps == 90) {
		lcm_dcs_write_seq_static(ctx, 0xD0,0x10);
	} else if (current_fps == 120) {
		lcm_dcs_write_seq_static(ctx, 0xD0,0x00);
	} else if (current_fps == 144) {
		lcm_dcs_write_seq_static(ctx, 0xD0,0x40);
	}
	lcm_dcs_write_seq_static(ctx, 0xD1,0x00);
	lcm_dcs_write_seq_static(ctx, 0x71,0x01);
	//DBV setting
	lcm_dcs_write_seq_static(ctx, 0x35,0x00);
	lcm_dcs_write_seq_static(ctx, 0x53,0x24);
	lcm_dcs_write_seq_static(ctx, 0x51,0x00,0x00);
	//SPR modify form
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x14);
	lcm_dcs_write_seq_static(ctx, 0x80,0x01);
	lcm_dcs_write_seq_static(ctx, 0x81,0x00);
	lcm_dcs_write_seq_static(ctx, 0x83,0x02);
	lcm_dcs_write_seq_static(ctx, 0x84,0x28);
	lcm_dcs_write_seq_static(ctx, 0x85,0x28);
	lcm_dcs_write_seq_static(ctx, 0x86,0x05);
	lcm_dcs_write_seq_static(ctx, 0x87,0x50);
	lcm_dcs_write_seq_static(ctx, 0x91,0x33,0x4D);
	lcm_dcs_write_seq_static(ctx, 0x92,0x78,0x08);
	lcm_dcs_write_seq_static(ctx, 0x93,0x33,0x4D);
	lcm_dcs_write_seq_static(ctx, 0x94,0x33,0x4D);
	lcm_dcs_write_seq_static(ctx, 0x95,0x78,0x08);
	lcm_dcs_write_seq_static(ctx, 0x96,0x33,0x4D);
	lcm_dcs_write_seq_static(ctx, 0x97,0x33,0x4D);
	lcm_dcs_write_seq_static(ctx, 0x98,0x78,0x08);
	lcm_dcs_write_seq_static(ctx, 0x99,0x33,0x4D);
	lcm_dcs_write_seq_static(ctx, 0x9A,0x33,0x4D);
	lcm_dcs_write_seq_static(ctx, 0x9B,0x78,0x08);
	lcm_dcs_write_seq_static(ctx, 0x9C,0x33,0x4D);
	lcm_dcs_write_seq_static(ctx, 0x9D,0x22);
	lcm_dcs_write_seq_static(ctx, 0x9E,0x22);
	lcm_dcs_write_seq_static(ctx, 0xA0,0x31,0x71,0x31,0x31,0x71,0x31,0x31,0x71,0x31,0x31,0x71,0x31);
	lcm_dcs_write_seq_static(ctx, 0xA1,0x11,0x11,0x00,0x00,0x58,0x80);
	lcm_dcs_write_seq_static(ctx, 0xA2,0x00,0x00,0x26,0x26,0x68,0xA0);
	lcm_dcs_write_seq_static(ctx, 0xA3,0x10,0x00,0x01,0x00,0x70,0x80);
	lcm_dcs_write_seq_static(ctx, 0xA4,0x00,0x26,0x00,0x26,0x74,0xC4);
	lcm_dcs_write_seq_static(ctx, 0xA8,0xA0);
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x15);
	lcm_dcs_write_seq_static(ctx, 0x81,0x01);
	lcm_dcs_write_seq_static(ctx, 0x82,0x10);
	lcm_dcs_write_seq_static(ctx, 0xC8,0x00);
	lcm_dcs_write_seq_static(ctx, 0xC9,0x40);
	lcm_dcs_write_seq_static(ctx, 0xCA,0x00);
	lcm_dcs_write_seq_static(ctx, 0xCB,0x00);
	lcm_dcs_write_seq_static(ctx, 0xD1,0x01);
	lcm_dcs_write_seq_static(ctx, 0xD2,0x94);
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x0E);
	lcm_dcs_write_seq_static(ctx, 0xD1,0x00);
	lcm_dcs_write_seq_static(ctx, 0xD2,0x00);
	lcm_dcs_write_seq_static(ctx, 0xD3,0x00);

	//lhbm size & coordinate
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x1E);
	lcm_dcs_write_seq_static(ctx, 0xB2,0x61,0x63);
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x00);
	lcm_dcs_write_seq_static(ctx, 0x96,0x09);
	lcm_dcs_write_seq_static(ctx, 0x97,0xE6);
	lcm_dcs_write_seq_static(ctx, 0x98,0x02);
	lcm_dcs_write_seq_static(ctx, 0x99,0x7C);

	//GPO输出频率变化,触显一体设定
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x00);
	lcm_dcs_write_seq_static(ctx, 0xAF,0x00,0x00,0xC0,0x01);
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x2D);
	lcm_dcs_write_seq_static(ctx, 0xE3,0x40,0x0B,0x00,0x00,0xD1);
	lcm_dcs_write_seq_static(ctx, 0xEF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x20);
	lcm_dcs_write_seq_static(ctx, 0xBA,0x24);
	lcm_dcs_write_seq_static(ctx, 0xBC,0x54);
	lcm_dcs_write_seq_static(ctx, 0xBD,0x23);

	lcm_dcs_write_seq_static(ctx, 0xFF,0x5A,0xA5,0x00);

	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	usleep_range(120*1000, 121*1000);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);

	pr_info("debug for %s-\n", __func__);
}

static struct regulator *mt6369_vant18;
static int lcm_panel_mt6369_vant18_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	mt6369_vant18 = devm_regulator_get(dev, "vddi");
	if (IS_ERR_OR_NULL(mt6369_vant18)) { /* handle return value */
		ret = PTR_ERR(mt6369_vant18);
		pr_err("get mt6369_vant18 fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */
}

static int lcm_panel_mt6369_vant18_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_mt6369_vant18_regulator_init(dev);

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(mt6369_vant18)) {
		ret = regulator_set_voltage(mt6369_vant18, 1800000, 1800000);
		if (ret < 0)
			pr_err("set voltage mt6369_vant18 fail, ret = %d\n", ret);
		retval |= ret;
	}
	/* enable regulator */
	if (!IS_ERR_OR_NULL(mt6369_vant18)) {
		ret = regulator_enable(mt6369_vant18);
		if (ret < 0)
			pr_err("enable regulator mt6369_vant18 fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("%s, retval = %d\n", __func__, retval);
	return retval;
}

static int lcm_panel_mt6369_vant18_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_mt6369_vant18_regulator_init(dev);

	if (!IS_ERR_OR_NULL(mt6369_vant18)) {
		ret = regulator_disable(mt6369_vant18);
		if (ret < 0)
			pr_err("disable regulator mt6369_vant18 fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("%s, retval = %d\n", __func__, retval);
	return retval;
}

static struct regulator *mt6369_vfp;
static int lcm_panel_mt6369_vfp_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	mt6369_vfp = devm_regulator_get(dev, "vci");
	if (IS_ERR_OR_NULL(mt6369_vfp)) { /* handle return value */
		ret = PTR_ERR(mt6369_vfp);
		pr_err("get mt6369_vfp fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */
}

static int lcm_panel_mt6369_vfp_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_mt6369_vfp_regulator_init(dev);

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(mt6369_vfp)) {
		ret = regulator_set_voltage(mt6369_vfp, 3000000, 3000000);
		if (ret < 0)
			pr_err("set voltage mt6369_vfp fail, ret = %d\n", ret);
		retval |= ret;
	}
	/* enable regulator */
	if (!IS_ERR_OR_NULL(mt6369_vfp)) {
		ret = regulator_enable(mt6369_vfp);
		if (ret < 0)
			pr_err("enable regulator mt6369_vfp fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("%s, retval = %d\n", __func__, retval);
	return retval;
}

static int lcm_panel_mt6369_vfp_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_mt6369_vfp_regulator_init(dev);

	if (!IS_ERR_OR_NULL(mt6369_vfp)) {
		ret = regulator_disable(mt6369_vfp);
		if (ret < 0)
			pr_err("disable regulator mt6369_vfp fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("%s, retval = %d\n", __func__, retval);
	return retval;
}


static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s:prepared=%d\n", __func__, ctx->prepared);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(10000, 11000);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(120*1000, 121*1000);

	ctx->error = 0;
	ctx->prepared = false;
	//ctx->hbm_en = false;
	pr_info("%s:success\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s:prepared=%d\n", __func__, ctx->prepared);
	if (ctx->prepared)
		return 0;

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
	pr_info("%s:success\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define PHYSICAL_WIDTH          71868
#define PHYSICAL_HEIGHT         156618

#define FRAME_WIDTH             (1272)
#define FRAME_HEIGHT            (2772)
#define FRAME_WIDTH_FHD         (1080)
#define FRAME_HEIGHT_FHD        (2354)
#define HFP                     (208)
#define HFP_144HZ               (100)
#define HFP_30HZ                (2126)
#define HBP                     (12)
#define HSA                     (12)
#define VFP_60HZ                (2960)
#define VFP_90HZ                (980)
#define VFP_120HZ               (80)
#define VBP                     (26)
#define VSA                     (2)

static const struct drm_display_mode display_mode[MODE_NUM * RES_NUM] = {
	//fhdplus 120hz
	{
		.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_120HZ + VBP + VSA) * 120) / 1000,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP,
		.hsync_end = FRAME_WIDTH + HFP + HSA,
		.htotal = FRAME_WIDTH + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_120HZ,
		.vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
	},
	//fhdplus 30hz
	{
		.clock = ((FRAME_WIDTH + HFP_30HZ + HBP + HSA) * (FRAME_HEIGHT + VFP_120HZ + VBP + VSA) * 30) / 1000,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP_30HZ,
		.hsync_end = FRAME_WIDTH + HFP_30HZ + HSA,
		.htotal = FRAME_WIDTH + HFP_30HZ + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_120HZ,
		.vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
	},
	//fhdplus 60hz
	{
		.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_60HZ + VBP + VSA) * 60) / 1000,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP,
		.hsync_end = FRAME_WIDTH + HFP + HSA,
		.htotal = FRAME_WIDTH + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_60HZ,
		.vsync_end = FRAME_HEIGHT + VFP_60HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_60HZ + VSA + VBP,
	},
	//fhdplus 90hz
	{
		.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_90HZ + VBP + VSA) * 90) / 1000,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP,
		.hsync_end = FRAME_WIDTH + HFP + HSA,
		.htotal = FRAME_WIDTH + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_90HZ,
		.vsync_end = FRAME_HEIGHT + VFP_90HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_90HZ + VSA + VBP,
	},
	//fhdplus 144hz
	{
		.clock = ((FRAME_WIDTH + HFP_144HZ + HBP + HSA) * (FRAME_HEIGHT + VFP_120HZ + VBP + VSA) * 144) / 1000,
		.hdisplay = FRAME_WIDTH,
		.hsync_start = FRAME_WIDTH + HFP_144HZ,
		.hsync_end = FRAME_WIDTH + HFP_144HZ + HSA,
		.htotal = FRAME_WIDTH + HFP_144HZ + HSA + HBP,
		.vdisplay = FRAME_HEIGHT,
		.vsync_start = FRAME_HEIGHT + VFP_120HZ,
		.vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
		.vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
	},
	//fhd 30hz
	{
		.clock = ((FRAME_WIDTH_FHD + HFP_30HZ + HBP + HSA) * (FRAME_HEIGHT_FHD + VFP_120HZ + VBP + VSA) * 30) / 1000,
		.hdisplay = FRAME_WIDTH_FHD,
		.hsync_start = FRAME_WIDTH_FHD + HFP_30HZ,
		.hsync_end = FRAME_WIDTH_FHD + HFP_30HZ + HSA,
		.htotal = FRAME_WIDTH_FHD + HFP_30HZ + HSA + HBP,
		.vdisplay = FRAME_HEIGHT_FHD,
		.vsync_start = FRAME_HEIGHT_FHD + VFP_120HZ,
		.vsync_end = FRAME_HEIGHT_FHD + VFP_120HZ + VSA,
		.vtotal = FRAME_HEIGHT_FHD + VFP_120HZ + VSA + VBP,
	},
	//fhd 60hz
	{
		.clock = ((FRAME_WIDTH_FHD + HFP + HBP + HSA) * (FRAME_HEIGHT_FHD + VFP_60HZ + VBP + VSA) * 60) / 1000,
		.hdisplay = FRAME_WIDTH_FHD,
		.hsync_start = FRAME_WIDTH_FHD + HFP,
		.hsync_end = FRAME_WIDTH_FHD + HFP + HSA,
		.htotal = FRAME_WIDTH_FHD + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT_FHD,
		.vsync_start = FRAME_HEIGHT_FHD + VFP_60HZ,
		.vsync_end = FRAME_HEIGHT_FHD + VFP_60HZ + VSA,
		.vtotal = FRAME_HEIGHT_FHD + VFP_60HZ + VSA + VBP,
	},
	//fhd 90hz
	{
		.clock = ((FRAME_WIDTH_FHD + HFP + HBP + HSA) * (FRAME_HEIGHT_FHD + VFP_90HZ + VBP + VSA) * 90) / 1000,
		.hdisplay = FRAME_WIDTH_FHD,
		.hsync_start = FRAME_WIDTH_FHD + HFP,
		.hsync_end = FRAME_WIDTH_FHD + HFP + HSA,
		.htotal = FRAME_WIDTH_FHD + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT_FHD,
		.vsync_start = FRAME_HEIGHT_FHD + VFP_90HZ,
		.vsync_end = FRAME_HEIGHT_FHD + VFP_90HZ + VSA,
		.vtotal = FRAME_HEIGHT_FHD + VFP_90HZ + VSA + VBP,
	},
	//fhd 120hz
	{
		.clock = ((FRAME_WIDTH_FHD + HFP + HBP + HSA) * (FRAME_HEIGHT_FHD + VFP_120HZ + VBP + VSA) * 120) / 1000,
		.hdisplay = FRAME_WIDTH_FHD,
		.hsync_start = FRAME_WIDTH_FHD + HFP,
		.hsync_end = FRAME_WIDTH_FHD + HFP + HSA,
		.htotal = FRAME_WIDTH_FHD + HFP + HSA + HBP,
		.vdisplay = FRAME_HEIGHT_FHD,
		.vsync_start = FRAME_HEIGHT_FHD + VFP_120HZ,
		.vsync_end = FRAME_HEIGHT_FHD + VFP_120HZ + VSA,
		.vtotal = FRAME_HEIGHT_FHD + VFP_120HZ + VSA + VBP,
	},
	//fhd 144hz
	{
		.clock = ((FRAME_WIDTH_FHD + HFP_144HZ + HBP + HSA) * (FRAME_HEIGHT_FHD + VFP_120HZ + VBP + VSA) * 144) / 1000,
		.hdisplay = FRAME_WIDTH_FHD,
		.hsync_start = FRAME_WIDTH_FHD + HFP_144HZ,
		.hsync_end = FRAME_WIDTH_FHD + HFP_144HZ + HSA,
		.htotal = FRAME_WIDTH_FHD + HFP_144HZ + HSA + HBP,
		.vdisplay = FRAME_HEIGHT_FHD,
		.vsync_start = FRAME_HEIGHT_FHD + VFP_120HZ,
		.vsync_end = FRAME_HEIGHT_FHD + VFP_120HZ + VSA,
		.vtotal = FRAME_HEIGHT_FHD + VFP_120HZ + VSA + VBP,
	},
};

static struct mtk_panel_params ext_params_60Hz = {
	.pll_clk = 684,
	.data_rate = 1368,
	.vdo_per_frame_lp_enable = 1,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0xD0, 0x20}},
/*		.dfps_cmd_table[0] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[1] = {0, 2 , {0x80, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0x81, 0x03}},
		.dfps_cmd_table[3] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[4] = {0, 2 , {0x88, 0x78}},
		.dfps_cmd_table[5] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
		.dfps_cmd_table[6] = {0, 2 , {0xD0, 0x20}},
		.dfps_cmd_table[7] = {0, 4, {0xFF, 0x5A, 0xA5, 0x20}},
		.dfps_cmd_table[8] = {0, 2 , {0xBA, 0x22}},
		.dfps_cmd_table[9] = {0, 2 , {0xBC, 0x24}},
		.dfps_cmd_table[10] = {0, 2 , {0xBD, 0x53}},
		.dfps_cmd_table[11] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x07}},
		.dfps_cmd_table[12] = {0, 2 , {0xD4, 0x90}},
		.dfps_cmd_table[13] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x2A}},
		.dfps_cmd_table[14] = {0, 2 , {0xA8, 0x8C}},
		.dfps_cmd_table[15] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[16] = {0, 2 , {0x8A, 0x78}},
		.dfps_cmd_table[17] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x2A}},
		.dfps_cmd_table[18] = {0, 2 , {0xA8, 0x8D}},
		.dfps_cmd_table[19] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[20] = {0, 2 , {0x8B, 0x78}},
		.dfps_cmd_table[21] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
*/	},

	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.color_vivid_status = true,
	.vendor = "AC382_A0034",
	.manufacture = "P_3",

/*	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
*/	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.panel_bpp = 10,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2772,
		.pic_width = 1272,
		.slice_height = 22,
		.slice_width = 636,
		.chunk_size = 636,
		.xmit_delay = 512,
		.dec_delay = 601,
		.scale_value = 32,
		.increment_interval = 558,
		.decrement_interval = 8,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 1268,
		.slice_bpg_offset = 1005,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
};

static struct mtk_panel_params ext_params_90Hz = {
	.pll_clk = 684,
	.data_rate = 1368,
	.vdo_per_frame_lp_enable = 1,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0xD0, 0x10}},
/*		.dfps_cmd_table[0] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[1] = {0, 2 , {0x80, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0x81, 0x03}},
		.dfps_cmd_table[3] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[4] = {0, 2 , {0x88, 0x78}},
		.dfps_cmd_table[5] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
		.dfps_cmd_table[6] = {0, 2 , {0xD0, 0x10}},
		.dfps_cmd_table[7] = {0, 4, {0xFF, 0x5A, 0xA5, 0x20}},
		.dfps_cmd_table[8] = {0, 2 , {0xBA, 0x20}},
		.dfps_cmd_table[9] = {0, 2 , {0xBC, 0x50}},
		.dfps_cmd_table[10] = {0, 2 , {0xBD, 0x03}},
		.dfps_cmd_table[11] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x07}},
		.dfps_cmd_table[12] = {0, 2 , {0xD4, 0x90}},
		.dfps_cmd_table[13] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x2A}},
		.dfps_cmd_table[14] = {0, 2 , {0xA8, 0x8E}},
		.dfps_cmd_table[15] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[16] = {0, 2 , {0x8A, 0x78}},
		.dfps_cmd_table[17] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x2A}},
		.dfps_cmd_table[18] = {0, 2 , {0xA8, 0x8F}},
		.dfps_cmd_table[19] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[20] = {0, 2 , {0x8B, 0x78}},
		.dfps_cmd_table[21] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
*/	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.color_vivid_status = true,
	.vendor = "AC382_A0034",
	.manufacture = "P_3",

/*	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
*/	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.panel_bpp = 10,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2772,
		.pic_width = 1272,
		.slice_height = 22,
		.slice_width = 636,
		.chunk_size = 636,
		.xmit_delay = 512,
		.dec_delay = 601,
		.scale_value = 32,
		.increment_interval = 558,
		.decrement_interval = 8,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 1268,
		.slice_bpg_offset = 1005,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
};


static struct mtk_panel_params ext_params_120Hz = {
	.pll_clk = 684,
	.data_rate = 1368,
	.vdo_per_frame_lp_enable = 1,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0xD0, 0x00}},
/*		.dfps_cmd_table[0] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[1] = {0, 2 , {0x80, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0x81, 0x03}},
		.dfps_cmd_table[3] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[4] = {0, 2 , {0x88, 0x78}},
		.dfps_cmd_table[5] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
		.dfps_cmd_table[6] = {0, 2 , {0xD0, 0x00}},
		.dfps_cmd_table[7] = {0, 4, {0xFF, 0x5A, 0xA5, 0x20}},
		.dfps_cmd_table[8] = {0, 2 , {0xBA, 0x24}},
		.dfps_cmd_table[9] = {0, 2 , {0xBC, 0x54}},
		.dfps_cmd_table[10] = {0, 2 , {0xBD, 0x23}},
		.dfps_cmd_table[11] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x07}},
		.dfps_cmd_table[12] = {0, 2 , {0xD4, 0x90}},
		.dfps_cmd_table[13] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x2A}},
		.dfps_cmd_table[14] = {0, 2 , {0xA8, 0x8E}},
		.dfps_cmd_table[15] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[16] = {0, 2 , {0x8A, 0x78}},
		.dfps_cmd_table[17] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x2A}},
		.dfps_cmd_table[18] = {0, 2 , {0xA8, 0x8F}},
		.dfps_cmd_table[19] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[20] = {0, 2 , {0x8B, 0x78}},
		.dfps_cmd_table[21] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
*/	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.color_vivid_status = true,
	.vendor = "AC382_A0034",
	.manufacture = "P_3",

/*	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
*/	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.panel_bpp = 10,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2772,
		.pic_width = 1272,
		.slice_height = 22,
		.slice_width = 636,
		.chunk_size = 636,
		.xmit_delay = 512,
		.dec_delay = 601,
		.scale_value = 32,
		.increment_interval = 558,
		.decrement_interval = 8,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 1268,
		.slice_bpg_offset = 1005,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
};

static struct mtk_panel_params ext_params_144Hz = {
	.pll_clk = 684,
	.data_rate = 1368,
	.vdo_per_frame_lp_enable = 1,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 144,
		.dfps_cmd_table[0] = {0, 2 , {0xD0, 0x40}},
/*		.dfps_cmd_table[0] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[1] = {0, 2 , {0x80, 0x01}},
		.dfps_cmd_table[2] = {0, 2 , {0x81, 0x03}},
		.dfps_cmd_table[3] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[4] = {0, 2 , {0x88, 0x78}},
		.dfps_cmd_table[5] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
		.dfps_cmd_table[6] = {0, 2 , {0xD0, 0x40}},
		.dfps_cmd_table[7] = {0, 4, {0xFF, 0x5A, 0xA5, 0x20}},
		.dfps_cmd_table[8] = {0, 2 , {0xBA, 0x24}},
		.dfps_cmd_table[9] = {0, 2 , {0xBC, 0x54}},
		.dfps_cmd_table[10] = {0, 2 , {0xBD, 0x23}},
		.dfps_cmd_table[11] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x07}},
		.dfps_cmd_table[12] = {0, 2 , {0xD4, 0x90}},
		.dfps_cmd_table[13] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x2A}},
		.dfps_cmd_table[14] = {0, 2 , {0xA8, 0x84}},
		.dfps_cmd_table[15] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[16] = {0, 2 , {0x8A, 0x78}},
		.dfps_cmd_table[17] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x2A}},
		.dfps_cmd_table[18] = {0, 2 , {0xA8, 0x85}},
		.dfps_cmd_table[19] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x4F}},
		.dfps_cmd_table[20] = {0, 2 , {0x8B, 0x78}},
		.dfps_cmd_table[21] = {0, 4 , {0xFF, 0x5A, 0xA5, 0x00}},
*/	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.color_vivid_status = true,
	.vendor = "AC382_A0034",
	.manufacture = "P_3",

	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.panel_bpp = 10,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2772,
		.pic_width = 1272,
		.slice_height = 22,
		.slice_width = 636,
		.chunk_size = 636,
		.xmit_delay = 512,
		.dec_delay = 601,
		.scale_value = 32,
		.increment_interval = 558,
		.decrement_interval = 8,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 1268,
		.slice_bpg_offset = 1005,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
};

static struct mtk_panel_params ext_params_30Hz = {
	.pll_clk = 684,
	.data_rate = 1368,
	.vdo_per_frame_lp_enable = 1,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 30,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.color_vivid_status = true,
	.vendor = "AC382_A0034",
	.manufacture = "P_3",

	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,

	.panel_bpp = 10,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2772,
		.pic_width = 1272,
		.slice_height = 22,
		.slice_width = 636,
		.chunk_size = 636,
		.xmit_delay = 512,
		.dec_delay = 601,
		.scale_value = 32,
		.increment_interval = 558,
		.decrement_interval = 8,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 1268,
		.slice_bpg_offset = 1005,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

//static int bl_demura_mode = -1;
//static bool lhbm_exit = false;
static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};
//	int index = 13;
//	int i = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 0) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	}

	if (level == 1) {
		DISP_ERR("[%s:%d]skip set backlight lvl:%u\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4094) {
		level = 4094;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = 2047;
	}

	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	DISP_ERR("ac382_P_3_a0034 backlight = %d bl_level[1]=%x, bl_level[2]=%x\n", level, bl_level[1], bl_level[2]);
	oplus_display_brightness = level;
//	lhbm_last_backlight = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &level);

	/* demura switch */
/*	if ((level > 1) && (level < 1154) && (lhbm_exit || (!lhbm_exit
		&& (bl_demura_mode != 2) && (bl_demura_mode != 0)))) {
		bl_demura_mode = 0;
		if (lhbm_exit) {
			lhbm_exit = false;
			pr_info("exit lhbm, set bl_demura_mode%d\n", bl_demura_mode);
		}
		index = sizeof(dsi_demura0_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
		}
		pr_info("demura DBV switch to bl_demura_mode%d\n", bl_demura_mode);
	} else if ((level >= 1154) && (lhbm_exit || (!lhbm_exit
		&& (bl_demura_mode != 2) && (bl_demura_mode != 1)))) {
		bl_demura_mode = 1;
		if (lhbm_exit) {
			lhbm_exit = false;
			pr_info("exit lhbm, set bl_demura_mode%d\n", bl_demura_mode);
		}
		index = sizeof(dsi_demura1_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
		}
		pr_info("demura DBV switch to bl_demura_mode%d\n", bl_demura_mode);
	}
*/
	return 0;
}
/*
static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int mode)
{
	unsigned int i = 0;
	pr_info("[DISP][INFO][%s: mode=%d\n", __func__, mode);
	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	temp_seed_mode = mode;

	switch(mode) {
		case NATURAL:
			for(i = 0; i < sizeof(dsi_set_seed_natural)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_natural[i].para_list, dsi_set_seed_natural[i].count);
			}
		break;
		case EXPERT:
			for(i = 0; i < sizeof(dsi_set_seed_expert)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_expert[i].para_list, dsi_set_seed_expert[i].count);
			}
		break;
		default:
		break;
	}
	return 0;
}
*/

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char esd_bl_level[] = {0x51, 0x03, 0xFF};
//	int index = 13;
//	int i = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	esd_bl_level[1] = level >> 8;
	esd_bl_level[2] = level & 0xFF;
	cb(dsi, handle, esd_bl_level, ARRAY_SIZE(esd_bl_level));
	lhbm_last_backlight = level;
	pr_info("esd_bl_level[1]=%x, esd_bl_level[2]=%x\n", esd_bl_level[1], esd_bl_level[2]);

/*	if ((level > 1) && (level < 1154) && (!lhbm_exit
		&& (bl_demura_mode != 2) && (bl_demura_mode != 0))) {
		bl_demura_mode = 0;
		index = sizeof(dsi_demura0_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura0_bl[i].para_list, dsi_demura0_bl[i].count);
		}
	} else if ((level >= 1154) && (!lhbm_exit
		&& (bl_demura_mode != 2) && (bl_demura_mode != 1))) {
		bl_demura_mode = 1;
		index = sizeof(dsi_demura1_bl) / sizeof(struct LCM_setting_table);
		for (i = 0; i < index; i++){
			cb(dsi, handle, dsi_demura1_bl[i].para_list, dsi_demura1_bl[i].count);
		}
	}
	pr_info("level=%d, bl_demura_mode=%d\n", level, bl_demura_mode);
*/
	return 0;
}
/*
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	int i = 0;

	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	pr_info("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, hbm_mode);
	if (hbm_mode == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, hbm_on_cmd[i].para_list, hbm_on_cmd[i].count);
		}
	} else if (hbm_mode == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, hbm_off_cmd[i].para_list, hbm_off_cmd[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	return 0;
}*/

static int oplus_ofp_set_lhbm_pressed_icon_single(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	struct lcm *ctx = NULL;
	int i = 0;
	unsigned int reg_count = 0;
	struct LCM_setting_table *lhbm_pressed_icon_cmd = NULL;

	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid ctx params\n");
	}

	pr_info("%s,oplus_display_brightness=%d, hbm_backlight %d, hbm_mode=%u\n", __func__, oplus_display_brightness, lhbm_last_backlight, en);
	if (en) {
		reg_count = sizeof(lcm_lhbm_on_setting) / sizeof(struct LCM_setting_table);
		lhbm_pressed_icon_cmd = lcm_lhbm_on_setting;
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_cmd[i].para_list, lhbm_pressed_icon_cmd[i].count);
		}
	} else if (en == 0) {
		reg_count = sizeof(lcm_lhbm_off_setting) / sizeof(struct LCM_setting_table);
		lhbm_pressed_icon_cmd = lcm_lhbm_off_setting;
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_cmd[i].para_list, lhbm_pressed_icon_cmd[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	return 0;
}

static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*state = ctx->hbm_en;
}

static void panel_hbm_set_state(struct drm_panel *panel, bool state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->hbm_en = state;
}

static void panel_hbm_get_wait_state(struct drm_panel *panel, bool *wait)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*wait = ctx->hbm_wait;
}

static bool panel_hbm_set_wait_state(struct drm_panel *panel, bool wait)
{
	struct lcm *ctx = panel_to_lcm(panel);
	bool old = ctx->hbm_wait;

	ctx->hbm_wait = wait;
	return old;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;

	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	for (i = 0; i < (sizeof(AOD_off_setting) / sizeof(struct LCM_setting_table)); i++) {

		cmd = AOD_off_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count * 1000, AOD_off_setting[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(AOD_off_setting[i].count, AOD_off_setting[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(AOD_off_setting[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, AOD_off_setting[i].para_list, AOD_off_setting[i].count);
		}
	}
	aod_state = false;
	//if (temp_seed_mode)
		//panel_set_seed(dsi, cb, handle, temp_seed_mode);
	pr_info("%s:success\n", __func__);
	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;
	unsigned int reg_count = 0;
	aod_state = true;
	struct LCM_setting_table *lhbm_pressed_icon_off_cmd = NULL;
	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	if(oplus_ofp_local_hbm_is_enabled() && oplus_ofp_get_hbm_state()) {
		reg_count = sizeof(lcm_lhbm_off_setting) / sizeof(struct LCM_setting_table);
		lhbm_pressed_icon_off_cmd = lcm_lhbm_off_setting;
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_off_cmd[i].para_list, lhbm_pressed_icon_off_cmd[i].count);
		}
		OFP_INFO("should off hbm\n");
	}

	for (i = 0; i < (sizeof(AOD_on_setting)/sizeof(struct LCM_setting_table)); i++) {
		cmd = AOD_on_setting[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(AOD_on_setting[i].count * 1000, AOD_on_setting[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(AOD_on_setting[i].count, AOD_on_setting[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				{
					cb(dsi, handle, AOD_on_setting[i].para_list, AOD_on_setting[i].count);
				}
		}
	}
	pr_info("%s:success\n", __func__);
	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_bl_level[i].para_list, aod_high_bl_level[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_bl_level)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_bl_level[i].para_list, aod_low_bl_level[i].count);
		}
	}
	pr_info("%s:success %d !\n", __func__, level);

	return 0;
}

static struct vdo_aod_params vdo_aod_on = {
	.porch_change_flag = 0x03,
	.dst_hfp = 2126,
	.dst_vfp = 80, //30fps
	.mode_idx = FHD_SDC30,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[1]={1, {0x39}},
	.vdo_aod_cmd_table[2]={2, {0xD1,0x00}},
	.vdo_aod_cmd_table[3]={4, {0xFF, 0x5A, 0xA5, 0x07}},
	.vdo_aod_cmd_table[4]={2, {0xD4,0x93}},
	.vdo_aod_cmd_table[5]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[6]={5, {0x51, 0x00,0x00, 0x0D,0xBB}},
};


static struct vdo_aod_params vdo_aod_to_120hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 114,
	.dst_vfp = 80,
	.mode_idx = 0,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[1]={1, {0x38}},
	.vdo_aod_cmd_table[2]={2, {0xD0,0x00}},
	.vdo_aod_cmd_table[3]={4, {0xFF, 0x5A, 0xA5, 0x07}},
	.vdo_aod_cmd_table[4]={2, {0xD4,0x90}},
	.vdo_aod_cmd_table[5]={4, {0xFF, 0x5A, 0xA5, 0x00}},

};

static struct vdo_aod_params vdo_aod_to_120hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 114,
	.dst_vfp = 80,
	.mode_idx = 0,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[1]={1, {0x38}},
	.vdo_aod_cmd_table[2]={2, {0xD0,0x00}},
	.vdo_aod_cmd_table[3]={4, {0xFF, 0x5A, 0xA5, 0x07}},
	.vdo_aod_cmd_table[4]={2, {0xD4,0x90}},
	.vdo_aod_cmd_table[5]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[6]={5, {0x51, 0x00, 0x00, 0x00, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_90hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 114,
	.dst_vfp = 980,
	.mode_idx = 1,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[1]={1, {0x38}},
	.vdo_aod_cmd_table[2]={2, {0xD0,0x10}},
	.vdo_aod_cmd_table[3]={4, {0xFF, 0x5A, 0xA5, 0x07}},
	.vdo_aod_cmd_table[4]={2, {0xD4,0x90}},
	.vdo_aod_cmd_table[5]={4, {0xFF, 0x5A, 0xA5, 0x00}},

};

static struct vdo_aod_params vdo_aod_to_90hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 114,
	.dst_vfp = 980,
	.mode_idx = 1,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[1]={1, {0x38}},
	.vdo_aod_cmd_table[2]={2, {0xD0,0x10}},
	.vdo_aod_cmd_table[3]={4, {0xFF, 0x5A, 0xA5, 0x07}},
	.vdo_aod_cmd_table[4]={2, {0xD4,0x90}},
	.vdo_aod_cmd_table[5]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[6]={5, {0x51, 0x00, 0x00, 0x00, 0x00}},

};

static struct vdo_aod_params vdo_aod_to_60hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 114,
	.dst_vfp = 2960,
	.mode_idx = 2,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[1]={1, {0x38}},
	.vdo_aod_cmd_table[2]={2, {0xD0,0x20}},
	.vdo_aod_cmd_table[3]={4, {0xFF, 0x5A, 0xA5, 0x07}},
	.vdo_aod_cmd_table[4]={2, {0xD4, 0x90}},
	.vdo_aod_cmd_table[5]={4, {0xFF, 0x5A, 0xA5, 0x00}},
};

static struct vdo_aod_params vdo_aod_to_60hz_unlocking = {
	.porch_change_flag = 0x03,
	.dst_hfp = 114,
	.dst_vfp = 2960,
	.mode_idx = 2,
	.change_mmclk = true,
	.vdo_aod_cmd_table[0]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[1]={1, {0x38}},
	.vdo_aod_cmd_table[2]={2, {0xD0,0x20}},
	.vdo_aod_cmd_table[3]={4, {0xFF, 0x5A, 0xA5, 0x07}},
	.vdo_aod_cmd_table[4]={2, {0xD4, 0x90}},
	.vdo_aod_cmd_table[5]={4, {0xFF, 0x5A, 0xA5, 0x00}},
	.vdo_aod_cmd_table[6]={5, {0x51, 0x00, 0x00, 0x00, 0x00}},
};

static int mtk_get_vdo_aod_param(int aod_en, struct vdo_aod_params **vdo_aod_param)
{
	static int mode_id_before_aod = 120;
	if (aod_en) {
		*vdo_aod_param = &vdo_aod_on;
		mode_id_before_aod = current_fps;
	} else {
		if (mode_id_before_aod == 60) {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_60hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_60hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		} else if (mode_id_before_aod == 90) {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_90hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_90hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		} else {
			if(oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_120hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_120hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		}
		if (oplus_ofp_get_aod_unlocking())
			lhbm_last_backlight = 0;
	}
	OFP_INFO("%s:aod_en %d, mode_id %d, unlocking =%d\n", __func__, aod_en, current_fps, oplus_ofp_get_aod_unlocking());
	return 0;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	DISP_ERR("%s: ac382_P_3_a0034 poweron Start\n", __func__);
	//enable ldo 1p8
	lcm_panel_mt6369_vant18_enable(ctx->dev);
	usleep_range(5000, 5100);

	/* vddr-oled 1p2 enable */
	ctx->vddr1p2_enable_gpio =
			devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);

	/* Wait no limits, actual 3ms */
	usleep_range(5000, 5100);
	//enable ldo 3p0
	lcm_panel_mt6369_vfp_enable(ctx->dev);
	/* Wait > 10ms, actual 12ms */
	usleep_range(22000, 22100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	DISP_ERR("%s: ac382_P_3_a0034 poweron Successful\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_info("%s: ac382_P_3_a0034 lcm ctx->prepared %d\n", __func__, ctx->prepared);

	// set reset 0
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);

	/* vci-oled 3p0 disable */
	lcm_panel_mt6369_vfp_disable(ctx->dev);
	usleep_range(5000, 5100);

	/* vddr-oled 1p2 disable */
	ctx->vddr1p2_enable_gpio =
			devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	usleep_range(5000, 5100);

	/* vddi-oled 1p8 disable */
	lcm_panel_mt6369_vant18_disable(ctx->dev);

	usleep_range(10000, 10100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
//	bl_demura_mode = -1;
	pr_info("%s:ac382_P_3_a0034 Successful\n", __func__);

	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared)
		return 0;

	pr_info("[LCM]debug for lcd reset :%s, ctx->prepared:%d\n", __func__, ctx->prepared);

	if(IS_ERR(ctx->reset_gpio)){
		pr_err("cannot get reset-gpios %ld\n",PTR_ERR(ctx->reset_gpio));
	}

	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(15000, 15100);

	return 0;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);
	pr_info("%s: mode=%d, vrefresh=%d\n", __func__, mode, drm_mode_vrefresh(m));

	if (m_vrefresh == 60) {
		ext->params = &ext_params_60Hz;
		current_fps = 60;
	} else if (m_vrefresh == 90) {
		ext->params = &ext_params_90Hz;
		current_fps = 90;
	} else if (m_vrefresh == 120) {
		ext->params = &ext_params_120Hz;
		current_fps = 120;
	} else if (m_vrefresh == 144) {
		ext->params = &ext_params_144Hz;
		current_fps = 144;
	} else if (m_vrefresh == 30) {
		ext->params = &ext_params_30Hz;
		current_fps = 30;
	} else {
		ret = 1;
	}

	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	struct drm_display_mode *m = get_mode_by_id(connector, id);
	int m_vrefresh = 0;
	int ret = 0;

	if (m == NULL) {
		DISP_ERR("ac382_P_3_a0034 %s display_mode m = NULL!\n", __func__);
		return -EINVAL;
	}

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60) {
		*ext_param = &ext_params_60Hz;
	} else if (m_vrefresh == 90) {
		*ext_param = &ext_params_90Hz;
	} else if (m_vrefresh == 120) {
		*ext_param = &ext_params_120Hz;
	} else if (m_vrefresh == 144) {
		*ext_param = &ext_params_144Hz;
	} else if (m_vrefresh == 30) {
		*ext_param = &ext_params_30Hz;
	} else {
		*ext_param = &ext_params_60Hz;
	}

	if (*ext_param)
		DISP_DEBUG("ac382_P_3_a0034 data_rate:%d\n", (*ext_param)->data_rate);
	else
		DISP_ERR("ac382_P_3_a0034 ext_param is NULL;\n");

	return ret;
}

enum RES_SWITCH_TYPE mtk_get_res_switch_type(void)
{
	pr_info("res_switch_type: %d\n", res_switch_type);
	return res_switch_type;
}

int mtk_scaling_mode_mapping(int mode_idx)
{
	return MODE_MAPPING_RULE(mode_idx);
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.panel_reset = lcm_panel_reset,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.get_res_switch_type = mtk_get_res_switch_type,
	.scaling_mode_mapping = mtk_scaling_mode_mapping,
	//.set_hbm = lcm_set_hbm,
	.oplus_ofp_set_lhbm_pressed_icon_single = oplus_ofp_set_lhbm_pressed_icon_single,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.hbm_get_state = panel_hbm_get_state,
	.hbm_set_state = panel_hbm_set_state,
	.hbm_get_wait_state = panel_hbm_get_wait_state,
	.hbm_set_wait_state = panel_hbm_set_wait_state,
	//.set_seed = panel_set_seed,
	.get_vdo_aod_param = mtk_get_vdo_aod_param,

};

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode[MODE_NUM * RES_NUM];
	int i = 0;

	mode[0] = drm_mode_duplicate(connector->dev, &display_mode[0]);
	if (!mode[0]) {
		pr_err("failed to add mode %ux%ux@%u\n",
				display_mode[0].hdisplay, display_mode[0].vdisplay,
				 drm_mode_vrefresh(&display_mode[0]));
		return -ENOMEM;
	}

	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[0]);

	for (i = 1; i < MODE_NUM * RES_NUM; i++) {
		mode[i] = drm_mode_duplicate(connector->dev, &display_mode[i]);
		if (!mode[i]) {
			pr_err("not enough memory\n");
			return -ENOMEM;
		}

		drm_mode_set_name(mode[i]);
		mode[i]->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode[i]);
	}

	connector->display_info.width_mm = PHYSICAL_WIDTH / 1000;
	connector->display_info.height_mm = PHYSICAL_HEIGHT / 1000;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	unsigned int res_switch;

	pr_info("[LCM] ac382_P_3_a0034 %s START\n", __func__);


	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET;

	ret = of_property_read_u32(dev->of_node, "res-switch", &res_switch);
	if (ret < 0)
		res_switch = 0;
	else
		res_switch_type = (enum RES_SWITCH_TYPE)res_switch;
	pr_info("lcm probe res_switch_type:%d\n", res_switch);

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	lcm_panel_mt6369_vant18_enable(ctx->dev);

	usleep_range(5000, 5100);

	ctx->vddr1p2_enable_gpio =devm_gpiod_get(ctx->dev, "vddr-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);

	usleep_range(5000, 5100);

	lcm_panel_mt6369_vfp_enable(ctx->dev);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_60Hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	oplus_serial_number_probe(dev);
	register_device_proc("lcd", "AC382_A0034", "P_3");

	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	ctx->hbm_en = false;
	oplus_ofp_init(dev);

	pr_info("[LCM] %s- lcm, ac382_P_3_a0034, END\n", __func__);


	return ret;
}

static void lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

}

static const struct of_device_id lcm_of_match[] = {
	{
		.compatible = "ac382,p,3,a0034,vdo,panel",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "ac382_P_3_a0034_vdo_panel",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
