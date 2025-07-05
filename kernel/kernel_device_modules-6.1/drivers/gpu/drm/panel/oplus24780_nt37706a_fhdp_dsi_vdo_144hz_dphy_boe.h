#ifndef __OPLUS24780_NT37706A_FHDP_DSI_VDO_144HZ_DPHY_BOE__
#define __OPLUS24780_NT37706A_FHDP_DSI_VDO_144HZ_DPHY_BOE__

#define REGFLAG_CMD				0xFFFA
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

#define BRIGHTNESS_HALF         3515
#define BRIGHTNESS_MAX          4094

/* Mode Config */
#define MODE_NUM                    (4)
#define RES_NUM                     (2)
#define MODE_MAPPING_RULE(x)        ((x) % (MODE_NUM))
static enum RES_SWITCH_TYPE res_switch_type = RES_SWITCH_NO_USE;

enum MODE_ID {
	FHD_SDC60 = 0,
	FHD_SDC90 = 1,
	FHD_SDC120 = 2,
};

struct ba {
	u32 brightness;
	u32 alpha;
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

/* -------------------------doze mode setting start------------------------- */
static struct LCM_setting_table AOD_off_setting[] = {
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_lhbm_on_setting[] = {
	{REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x09}},
	{REGFLAG_CMD, 2, {0x6F, 0x00}},
	{REGFLAG_CMD, 2, {0xB0, 0x11}},
	{REGFLAG_CMD, 2, {0x6F, 0x25}},
	{REGFLAG_CMD, 2, {0xB0, 0x1F}},
	{REGFLAG_CMD, 3, {0x8B, 0x10, 0x01}},
	{REGFLAG_CMD, 2, {0x87, 0x25}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_lhbm_off_setbrightness_normal[] = {
	{REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x09}},
	{REGFLAG_CMD, 2, {0x6F, 0x00}},
	{REGFLAG_CMD, 2, {0xB0, 0x01}},
	{REGFLAG_CMD, 2, {0x6F, 0x25}},
	{REGFLAG_CMD, 2, {0xB0, 0x00}},
	{REGFLAG_CMD, 3, {0x8B, 0x00, 0x00}},
	{REGFLAG_CMD, 2, {0x87, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table dsi_switch_hbm_apl_on[] = {
	/* Enter APL 1% 6000*/
	{REGFLAG_CMD, 3, {0x51, 0x0F, 0xFF}},
	{REGFLAG_CMD, 2, {0x5F, 0x02}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table dsi_switch_hbm_apl_off[] = {
	{REGFLAG_CMD, 2, {0x5F, 0x00}},
	{REGFLAG_CMD, 3, {0x51, 0x0D, 0xBB}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,3, {0x51, 0x00, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table AOD_on_setting[] = {
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xFF}},

};

static struct LCM_setting_table aod_high_bl_level[] = {
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xFF}},
};

static struct LCM_setting_table aod_low_bl_level[] = {
	{REGFLAG_CMD, 3, {0x51, 0x01, 0xFF}},
};
/* -------------------------doze mode setting end------------------------- */

#endif