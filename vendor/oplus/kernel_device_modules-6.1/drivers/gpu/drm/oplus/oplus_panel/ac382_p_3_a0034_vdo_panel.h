#ifndef PANEL_AC382_P_3_A0034_H
#define PANEL_AC382_P_3_A0034_H

#define REGFLAG_CMD				0xFFFA
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

#define BRIGHTNESS_HALF         3515
#define BRIGHTNESS_MAX          4094

enum MODE_ID {
	FHD_SDC60 = 0,
	FHD_SDC90 = 1,
	FHD_SDC120 = 2,
	FHD_SDC144 = 3,
	FHD_SDC30 = 4,
};

/* Mode Config */
#define MODE_NUM                    5
#define RES_NUM                     (2)
#define MODE_MAPPING_RULE(x)        ((x) % (MODE_NUM))
static enum RES_SWITCH_TYPE res_switch_type = RES_SWITCH_NO_USE;

struct ba {
	u32 brightness;
	u32 alpha;
};
enum SEED_MODE_ID {
	EXPERT = 101,
	NATURAL = 102,
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

/* -------------------------doze mode setting start------------------------- */

struct LCM_setting_table lcm_lhbm_on_setting[] = {
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 2, {0xBA, 0x03}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table lcm_lhbm_off_setting[] = {
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 2, {0xBA, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table AOD_off_setting[] = {
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD,1, {0x38}},
	{REGFLAG_CMD,2, {0xD0,0x00}},
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x07}},
	{REGFLAG_CMD,2, {0xD4,0x90}},
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table AOD_on_setting[] = {
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD,1, {0x39}},
	{REGFLAG_CMD,2, {0xD1,0x00}},
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x07}},
	{REGFLAG_CMD,2, {0xD4,0x93}},
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 5, {0x51, 0x00, 0x00, 0x0D,0xBB}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table aod_high_bl_level[] = {
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD,5, {0x51, 0x00, 0x00, 0x0D,0xBB}},
};

struct LCM_setting_table aod_low_bl_level[] = {
	{REGFLAG_CMD,4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD,5, {0x51, 0x00, 0x00, 0x0B, 0x16}},
};

/* -------------------------doze mode setting end------------------------- */

/* -------------------------demura setting start---------------------- */
static struct LCM_setting_table dsi_demura0_bl[] = {
    /* demura0 level <= 0x0481(70nit_PWM) */
	// brs reload DC(BRS0,1,2)
    {REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x04}},
    {REGFLAG_CMD, 2, {0x6F, 0x01}},
    {REGFLAG_CMD, 4, {0xB5, 0x00, 0x00, 0x07}},
    {REGFLAG_CMD, 2, {0x9D, 0xAA}},
};

static struct LCM_setting_table dsi_demura1_bl[] = {
    /* demura1 0x0482(70.1nit_DC) <= level <= 0x0FFE(1400nit) */
	// brs reload DC(BRS0,1,3)
    {REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x04}},
    {REGFLAG_CMD, 2, {0x6F, 0x01}},
    {REGFLAG_CMD, 4, {0xB5, 0x00, 0x00, 0x0B}},
    {REGFLAG_CMD, 2, {0x9D, 0xAA}},
};
/* -------------------------demura setting end------------------------- */
/* ---------------panel seed setting --------------- */
/* ---------------Loading on 110% --------------- */
struct LCM_setting_table dsi_set_seed_natural[] = {
	{REGFLAG_CMD, 2, {0x5F,0x02}},
};
/* ---------------Loading off 100% --------------- */
struct LCM_setting_table dsi_set_seed_expert[] = {
	{REGFLAG_CMD, 2, {0x5F,0x00}},
};

#endif
