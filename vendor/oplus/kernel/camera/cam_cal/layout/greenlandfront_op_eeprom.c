// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL_GREENLANDFRONT"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include <linux/i2c.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"
#include "oplus_kd_imgsensor.h"
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>

#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

#define READ_4000K 0
#define GROUP1 0x01
#define GROUP2 0x0D
#define GROUP1_FLAG 1
#define GROUP2_FLAG 2
#define GROUP_OFFSET 0xA00

#define OTP_I2C_ADDR 0x6C
#define EEPROM_I2C_MSG_SIZE_READ 2
#define EEPROM_I2C_WRITE_MSG_LENGTH_MAX 32
#define EEPROM_I2C_READ_MSG_LENGTH_MAX 1024

#define GREENLANDFRONT_OTP_PAGE2 2
#define GREENLANDFRONT_OTP_PAGE7 7
#define GREENLANDFRONT_OTP_RET_FAIL -1
#define GREENLANDFRONT_OTP_RET_SUCCESS 0
#define GREENLANDFRONT_OTP_AWB_GROUP1_STARTADDR 0x828E
#define GREENLANDFRONT_OTP_LSC_GROUP1_CHECKSUMADDR 0x82B4

unsigned int addr_map[10] = {0x82B5, 0x83FF, 0x847A, 0x85FF, 0x867A, 0x87FF, 0x887A, 0x89FF, 0x8A7A, 0x8BE8};
int g_groupFlag = 0x0;
bool g_2a_first_read = false;
bool g_lsc_first_read = false;
bool g_lens_id_first_read = false;
int g_2a_buffer[30];
int g_lsc_buffer[5];
unsigned char *g_lsc_pointer;
unsigned char g_LensId[10];
struct STRUCT_CAM_CAL_LSC_MTK_TYPE g_lsc_type;

unsigned int Otp_read_region_greenlandfront(struct i2c_client *client,
	unsigned int addr, unsigned char *data, unsigned int size);
static int iReadData_CAM_CAL(struct i2c_client *client,
 			    unsigned int ui4_offset,
 			    unsigned int ui4_length,
 			    unsigned char *pinputdata);
static int greenlandfront_set_threshold(struct i2c_client *client, u8 threshold);
static unsigned int do_single_lsc_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_2a_gain_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_part_number_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_lens_id_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_group_distinguish(struct EEPROM_DRV_FD_DATA *pdata, unsigned int *pGetSensorCalData);
unsigned int do_lens_id_base_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00003107, 0x000154d1, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000000, 0x0000827F, 0x00000001, do_module_version},
		{0x00000001, 0x000082A0, 0x00000011, do_part_number_greenlandfront},
		{0x00000001, 0x000082B4, 0x0000074C, do_single_lsc_greenlandfront},
		{0x00000001, 0x0000828F, 0x00000012, do_2a_gain_greenlandfront},  //Start address, block size is useless
		{0x00000000, 0x0000827A, 0x0000096F, do_dump_all},
		{0x00000001, 0x00008288, 0x00000001, do_lens_id_greenlandfront}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT greenlandfront_op_eeprom = {
	.base_address = 0x00003000,
	.name = "greenlandfront_op_eeprom",
	.check_layout_function = layout_check,
	.read_function = Otp_read_region_greenlandfront,
	.layout = &cal_layout_table,
	.sensor_id = GREENLANDFRONT_SENSOR_ID,
	.i2c_write_id = 0x6C,
	.max_size = 0x7000,
	.enable_preload = 1,
	.preload_size = 0x1500,
};

static unsigned int do_single_lsc_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

	int read_data_size = 0;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	unsigned short table_size = 1868;
	int i;
	int group_offset = 0;
	int read_size = 0;
	int lsc_startaddr = 0, lsc_endaddr = 0;
	unsigned char *p;

	if (g_lsc_first_read == false) {
		g_lsc_first_read = true;
		if (g_groupFlag != GROUP2_FLAG && g_groupFlag != GROUP1_FLAG) {
			g_groupFlag = do_group_distinguish(pdata, pGetSensorCalData);
		}

		if (g_groupFlag == GROUP2_FLAG){
			group_offset = GROUP_OFFSET;
			LOG_INF("In %s: [greenlandfront_debug] read group flag success, group flag = 0x%x\n", __func__, g_groupFlag);
		} else if (g_groupFlag == GROUP1_FLAG){
			group_offset = 0;
			LOG_INF("In %s: [greenlandfront_debug] read group flag success, group flag = 0x%x\n", __func__, g_groupFlag);
		} else {
			LOG_INF("In %s: [greenlandfront_debug] read group flag fail, group flag = 0x%x\n", __func__, g_groupFlag);
		}

		if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
			err = CAM_CAL_ERR_NO_DEVICE;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
			return err;
		}
		if (block_size != CAM_CAL_SINGLE_LSC_SIZE)
			error_log("block_size(%d) is not match (%d)\n",
					block_size, CAM_CAL_SINGLE_LSC_SIZE);

		pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType = 2;//mtk type
		g_lsc_buffer[0] = pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType;
		pCamCalData->SingleLsc.LscTable.MtkLcsData.PixId = 8;
		g_lsc_buffer[1] = pCamCalData->SingleLsc.LscTable.MtkLcsData.PixId;

		pr_debug("lsc table_size %d\n", table_size);
		pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize = table_size;
		g_lsc_buffer[2] = pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize;
		if (table_size > 0) {
			pCamCalData->SingleLsc.TableRotation = 0;
			g_lsc_buffer[3] = pCamCalData->SingleLsc.TableRotation;
			debug_log("u4Offset=%d u4Length=%d", start_addr, table_size);

			p = (unsigned char *)&pCamCalData->SingleLsc.LscTable.MtkLcsData.SlimLscType;
			g_lsc_pointer = (unsigned char *)&g_lsc_type.SlimLscType;
			for (i = 0; i < sizeof(addr_map) / sizeof (addr_map[0]); i += 2) {
				lsc_startaddr = addr_map[i] + group_offset;
				lsc_endaddr = addr_map[i + 1] + group_offset;
				read_size = lsc_endaddr  - lsc_startaddr + 1;
				LOG_INF("[greenlandfront_debug] before read addr_map start_addr= 0x%x", lsc_startaddr);
				read_data_size += read_data(pdata,
					pCamCalData->sensorID, pCamCalData->deviceID,
					lsc_startaddr, read_size, p);
				read_data(pdata,
					pCamCalData->sensorID, pCamCalData->deviceID,
					lsc_startaddr, read_size, g_lsc_pointer);
				LOG_INF("[greenlandfront_debug] after read addr_map lsc_read_data[%d] = 0x%x, read_data = %d", i, *(int *)p, read_data_size);
				p += read_size;
				g_lsc_pointer += read_size;
			}

			error_log("[greenlandfront_debug] read_data_size = %d, table_size = %d", read_data_size, table_size);
			if (table_size == read_data_size)
				err = CAM_CAL_ERR_NO_ERR;
			else {
				error_log("Read Failed\n");
				err = CamCalReturnErr[pCamCalData->Command];
				show_cmd_error_log(pCamCalData->Command);
			}
		}
	}
	else {
		pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType = g_lsc_buffer[0];
		pCamCalData->SingleLsc.LscTable.MtkLcsData.PixId = g_lsc_buffer[1];
		pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize = g_lsc_buffer[2];
		pCamCalData->SingleLsc.TableRotation = g_lsc_buffer[3];
		pCamCalData->SingleLsc.LscTable.MtkLcsData.SlimLscType = g_lsc_type.SlimLscType;
		pCamCalData->SingleLsc.LscTable.MtkLcsData.PreviewWH = g_lsc_type.PreviewOffSet;
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CaptureWH = g_lsc_type.CaptureWH;
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CaptureOffSet = g_lsc_type.CaptureOffSet;
		pCamCalData->SingleLsc.LscTable.MtkLcsData.PreviewTblSize = g_lsc_type.PreviewTblSize;
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CaptureTblSize = g_lsc_type.CaptureTblSize;
		memcpy(pCamCalData->SingleLsc.LscTable.MtkLcsData.PvIspReg, g_lsc_type.PvIspReg, sizeof(g_lsc_type.PvIspReg));
		memcpy(pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg, g_lsc_type.CapIspReg, sizeof(g_lsc_type.CapIspReg));
		memcpy(pCamCalData->SingleLsc.LscTable.MtkLcsData.CapTable, g_lsc_type.CapTable, sizeof(g_lsc_type.CapTable));

		err = CAM_CAL_ERR_NO_ERR;
	}

	#ifdef DEBUG_CALIBRATION_LOAD
	pr_debug("======================SingleLsc Data==================\n");
	pr_debug("[1st] = %x, %x, %x, %x\n",
		pCamCalData->SingleLsc.LscTable.Data[0],
		pCamCalData->SingleLsc.LscTable.Data[1],
		pCamCalData->SingleLsc.LscTable.Data[2],
		pCamCalData->SingleLsc.LscTable.Data[3]);
	pr_debug("[1st] = SensorLSC(1)?MTKLSC(2)?  %x\n",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType);
	pr_debug("CapIspReg =0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[0],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[1],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[2],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[3],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[4]);
	pr_debug("RETURN = 0x%x\n", err);
	pr_debug("======================SingleLsc Data==================\n");
	#endif

	return err;
}

static unsigned int do_part_number_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	unsigned int size_limit = sizeof(pCamCalData->PartNumber);

	int group_offset = 0;

	if (g_groupFlag != GROUP2_FLAG && g_groupFlag != GROUP1_FLAG) {
		g_groupFlag = do_group_distinguish(pdata, pGetSensorCalData);
	}

	if (g_groupFlag == GROUP2_FLAG){
		group_offset = GROUP_OFFSET;
		LOG_INF("In %s: [greenlandfront_debug] read group flag success, group flag = 0x%x\n", __func__, g_groupFlag);
	} else if (g_groupFlag == GROUP1_FLAG){
		group_offset = 0;
		LOG_INF("In %s: [greenlandfront_debug] read group flag success, group flag = 0x%x\n", __func__, g_groupFlag);
	} else {
		LOG_INF("In %s: [greenlandfront_debug] read group flag fail, group flag = 0x%x\n", __func__, g_groupFlag);
	}

	memset(&pCamCalData->PartNumber[0], 0, size_limit);

	if (block_size > size_limit) {
		error_log("part number size can't larger than %u\n", size_limit);
		return err;
	}

	if (read_data_region(pdata,(unsigned char *)&pCamCalData->PartNumber[0],start_addr+group_offset, block_size) > 0)
		err = CAM_CAL_ERR_NO_ERR;
	else {
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}

	debug_log("======================Part Number==================\n");
	debug_log("[Part Number] = %x %x %x %x\n",
			pCamCalData->PartNumber[0], pCamCalData->PartNumber[1],
			pCamCalData->PartNumber[2], pCamCalData->PartNumber[3]);
	debug_log("[Part Number] = %x %x %x %x\n",
			pCamCalData->PartNumber[4], pCamCalData->PartNumber[5],
			pCamCalData->PartNumber[6], pCamCalData->PartNumber[7]);
	debug_log("[Part Number] = %x %x %x %x\n",
			pCamCalData->PartNumber[8], pCamCalData->PartNumber[9],
			pCamCalData->PartNumber[10], pCamCalData->PartNumber[11]);
	debug_log("======================Part Number==================\n");

	return err;
}

static unsigned int do_2a_gain_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	long long CalGain = 0, FacGain = 0;
	unsigned char AWBAFConfig = 0xf;

	int tempMax = 0;
	int CalR = 1, CalGr = 1, CalGb = 1, CalG = 1, CalB = 1;
	int FacR = 1, FacGr = 1, FacGb = 1, FacG = 1, FacB = 1;
	unsigned int awb_offset;

	(void) start_addr;
	(void) block_size;
	int group_offset = 0;
	if (g_2a_first_read == false){
		do_single_lsc_greenlandfront(pdata, start_addr, block_size, pGetSensorCalData);
		do_lens_id_greenlandfront(pdata, start_addr, block_size, pGetSensorCalData);

		if (g_groupFlag != GROUP2_FLAG && g_groupFlag != GROUP1_FLAG) {
			g_groupFlag = do_group_distinguish(pdata, pGetSensorCalData);
		}

		if (g_groupFlag == GROUP2_FLAG){
			group_offset = GROUP_OFFSET;
			LOG_INF("In %s: [greenlandfront_debug] read group flag success, group flag = 0x%x\n", __func__, g_groupFlag);
		} else if (g_groupFlag == GROUP1_FLAG){
			group_offset = 0;
			LOG_INF("In %s: [greenlandfront_debug] read group flag success, group flag = 0x%x\n", __func__, g_groupFlag);
		} else {
			LOG_INF("In %s: [greenlandfront_debug] read group flag fail, group flag = 0x%x\n", __func__, g_groupFlag);
		}

		pr_debug("In %s: sensor_id=%x\n", __func__, pCamCalData->sensorID);
		memset((void *)&pCamCalData->Single2A, 0, sizeof(struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT));
		/* Check rule */
		if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
			err = CAM_CAL_ERR_NO_DEVICE;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
			return err;
		}
		/* Check AWB & AF enable bit */
		pCamCalData->Single2A.S2aVer = 0x01;
		g_2a_buffer[0] = pCamCalData->Single2A.S2aVer;
		pCamCalData->Single2A.S2aBitEn = (0x03 & AWBAFConfig);
		g_2a_buffer[1] = pCamCalData->Single2A.S2aBitEn;
		pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & AWBAFConfig);
		g_2a_buffer[2] = pCamCalData->Single2A.S2aAfBitflagEn;
		debug_log("S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);
		/* AWB Calibration Data*/
		if (0x1 & AWBAFConfig) {
			pCamCalData->Single2A.S2aAwb.rGainSetNum = 0x02;
			/* AWB Unit Gain (5000K) */
			debug_log("[greenlandfront_debug] 5000K AWB\n");
			awb_offset = 0x828F + group_offset;

			read_data_size = read_data_region(pdata, (unsigned char *)&CalGain, awb_offset, 8);
			LOG_INF("In %s: [greenlandfront_debug] read Unit Gain success, read_data_size = %d\n", __func__, read_data_size);

			if (read_data_size > 0)	{
				debug_log("[greenlandfront_debug] Read CalGain OK %x\n", read_data_size);
				CalR  = CalGain & 0xFFFF;
				CalR  = ((CalR & 0xFF00) >> 8) | ((CalR & 0x00FF) << 8);
				CalGr = (CalGain >> 16) & 0xFFFF;
				CalGr  = ((CalGr & 0xFF00) >> 8) | ((CalGr & 0x00FF) << 8);
				CalGb = (CalGain >> 32) & 0xFFFF;
				CalGb  = ((CalGb & 0xFF00) >> 8) | ((CalGb & 0x00FF) << 8);
				CalG  = ((CalGr + CalGb) + 1) >> 1;
				CalB  = (CalGain >> 48) & 0xFFFF;
				CalB  = ((CalB & 0xFF00) >> 8) | ((CalB & 0x00FF) << 8);
				if (CalR > CalG)
					/* R > G */
					if (CalR > CalB)
						tempMax = CalR;
					else
						tempMax = CalB;
				else
					/* G > R */
					if (CalG > CalB)
						tempMax = CalG;
					else
						tempMax = CalB;
				debug_log("[greenlandfront_debug] UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
						CalR, CalG, CalB, tempMax);
				err = CAM_CAL_ERR_NO_ERR;
			} else {
				pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
				error_log("[greenlandfront_debug] Read CalGain Failed\n");
				show_cmd_error_log(pCamCalData->Command);
			}
			if (CalGain != 0x0000000000000000 &&
				CalGain != 0xFFFFFFFFFFFFFFFF &&
				CalR    != 0x00000000 &&
				CalG    != 0x00000000 &&
				CalB    != 0x00000000) {
				pCamCalData->Single2A.S2aAwb.rGainSetNum = 3;
				g_2a_buffer[3] = pCamCalData->Single2A.S2aAwb.rGainSetNum;
				pCamCalData->Single2A.S2aAwb.rUnitGainu4R =
						(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
				g_2a_buffer[4] = pCamCalData->Single2A.S2aAwb.rUnitGainu4R;
				pCamCalData->Single2A.S2aAwb.rUnitGainu4G =
						(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
				g_2a_buffer[5] = pCamCalData->Single2A.S2aAwb.rUnitGainu4G;
				pCamCalData->Single2A.S2aAwb.rUnitGainu4B =
						(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
				g_2a_buffer[6] = pCamCalData->Single2A.S2aAwb.rUnitGainu4B;

				pCamCalData->Single2A.S2aAwb.rUnitGainu4R_mid =
						(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
				g_2a_buffer[7] = pCamCalData->Single2A.S2aAwb.rUnitGainu4R_mid;
				pCamCalData->Single2A.S2aAwb.rUnitGainu4G_mid =
						(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
				g_2a_buffer[8] = pCamCalData->Single2A.S2aAwb.rUnitGainu4G_mid;
				pCamCalData->Single2A.S2aAwb.rUnitGainu4B_mid =
						(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
				g_2a_buffer[9] = pCamCalData->Single2A.S2aAwb.rUnitGainu4B_mid;

				pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low =
						(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
				g_2a_buffer[10] = pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low;
				pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low =
						(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
				g_2a_buffer[11] = pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low;
				pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low =
						(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
				g_2a_buffer[12] = pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low;
			} else {
				pr_debug("There are something wrong on EEPROM, plz contact module vendor!!\n");
				pr_debug("Unit R=%d G=%d B=%d!!\n", CalR, CalG, CalB);
			}

			/* AWB Golden Gain (5100K) */
			awb_offset = 0x8297 + group_offset;

			read_data_size = read_data_region(pdata, (unsigned char *)&FacGain, awb_offset, 8);
			LOG_INF("In %s: [greenlandfront_debug] read Golden Gain success, read_data_size = %d\n", __func__, read_data_size);

			if (read_data_size > 0)	{
				debug_log("Read FacGain OK\n");
				FacR  = FacGain & 0xFFFF;
				FacR  = ((FacR & 0xFF00) >> 8) | ((FacR & 0x00FF) << 8);
				FacGr = (FacGain >> 16) & 0xFFFF;
				FacGr  = ((FacGr & 0xFF00) >> 8) | ((FacGr & 0x00FF) << 8);
				FacGb = (FacGain >> 32) & 0xFFFF;
				FacGb  = ((FacGb & 0xFF00) >> 8) | ((FacGb & 0x00FF) << 8);
				FacG  = ((FacGr + FacGb) + 1) >> 1;
				FacB  = (FacGain >> 48) & 0xFFFF;
				FacB  = ((FacB & 0xFF00) >> 8) | ((FacB & 0x00FF) << 8);
				if (FacR > FacG)
					if (FacR > FacB)
						tempMax = FacR;
					else
						tempMax = FacB;
				else
					if (FacG > FacB)
						tempMax = FacG;
					else
						tempMax = FacB;
				debug_log("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
						FacR, FacG, FacB, tempMax);
				err = CAM_CAL_ERR_NO_ERR;
			} else {
				pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
				error_log("Read FacGain Failed\n");
				show_cmd_error_log(pCamCalData->Command);
			}

			if (FacGain != 0x0000000000000000 &&
				FacGain != 0xFFFFFFFFFFFFFFFF &&
				FacR    != 0x00000000 &&
				FacG    != 0x00000000 &&
				FacB    != 0x00000000)	{
				pCamCalData->Single2A.S2aAwb.rGoldGainu4R =
						(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
				g_2a_buffer[13] = pCamCalData->Single2A.S2aAwb.rGoldGainu4R;
				pCamCalData->Single2A.S2aAwb.rGoldGainu4G =
						(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
				g_2a_buffer[14] = pCamCalData->Single2A.S2aAwb.rGoldGainu4G;
				pCamCalData->Single2A.S2aAwb.rGoldGainu4B =
						(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
				g_2a_buffer[15] = pCamCalData->Single2A.S2aAwb.rGoldGainu4B;

				pCamCalData->Single2A.S2aAwb.rGoldGainu4R_mid =
						(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
				g_2a_buffer[16] = pCamCalData->Single2A.S2aAwb.rGoldGainu4R_mid;
				pCamCalData->Single2A.S2aAwb.rGoldGainu4G_mid =
						(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
				g_2a_buffer[17] = pCamCalData->Single2A.S2aAwb.rGoldGainu4G_mid;
				pCamCalData->Single2A.S2aAwb.rGoldGainu4B_mid =
						(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
				g_2a_buffer[18] = pCamCalData->Single2A.S2aAwb.rGoldGainu4B_mid;

				pCamCalData->Single2A.S2aAwb.rGoldGainu4R_low =
						(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
				g_2a_buffer[19] = pCamCalData->Single2A.S2aAwb.rGoldGainu4R_low;
				pCamCalData->Single2A.S2aAwb.rGoldGainu4G_low =
						(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
				g_2a_buffer[20] = pCamCalData->Single2A.S2aAwb.rGoldGainu4G_low;
				pCamCalData->Single2A.S2aAwb.rGoldGainu4B_low =
						(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
				g_2a_buffer[21] = pCamCalData->Single2A.S2aAwb.rGoldGainu4B_low;
			} else {
				pr_debug("There are something wrong on EEPROM, plz contact module vendor!!");
				pr_debug("Golden R=%d G=%d B=%d\n", FacR, FacG, FacB);
			}

			/* Set AWB to 3A Layer */
			pCamCalData->Single2A.S2aAwb.rValueR   = CalR;
			g_2a_buffer[22] = pCamCalData->Single2A.S2aAwb.rValueR;
			pCamCalData->Single2A.S2aAwb.rValueGr  = CalGr;
			g_2a_buffer[23] = pCamCalData->Single2A.S2aAwb.rValueGr;
			pCamCalData->Single2A.S2aAwb.rValueGb  = CalGb;
			g_2a_buffer[24] = pCamCalData->Single2A.S2aAwb.rValueGb;
			pCamCalData->Single2A.S2aAwb.rValueB   = CalB;
			g_2a_buffer[25] = pCamCalData->Single2A.S2aAwb.rValueB;
			pCamCalData->Single2A.S2aAwb.rGoldenR  = FacR;
			g_2a_buffer[26] = pCamCalData->Single2A.S2aAwb.rGoldenR;
			pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr;
			g_2a_buffer[27] = pCamCalData->Single2A.S2aAwb.rGoldenGr;
			pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb;
			g_2a_buffer[28] = pCamCalData->Single2A.S2aAwb.rGoldenGb;
			pCamCalData->Single2A.S2aAwb.rGoldenB  = FacB;
			g_2a_buffer[29] = pCamCalData->Single2A.S2aAwb.rGoldenB;
			#ifdef DEBUG_CALIBRATION_LOAD
			pr_debug("======================AWB CAM_CAL==================\n");
			pr_debug("AWB Calibration @5100K\n");
			pr_debug("[CalGain] = 0x%x\n", CalGain);
			pr_debug("[FacGain] = 0x%x\n", FacGain);
			pr_debug("[rCalGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
			pr_debug("[rCalGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
			pr_debug("[rCalGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
			pr_debug("[rFacGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
			pr_debug("[rFacGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
			pr_debug("[rFacGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
			#endif
		}
		g_2a_first_read = true;
	}
	else {
		pCamCalData->Single2A.S2aVer = g_2a_buffer[0];
		pCamCalData->Single2A.S2aBitEn = g_2a_buffer[1];
		pCamCalData->Single2A.S2aAfBitflagEn = g_2a_buffer[2];

		pCamCalData->Single2A.S2aAwb.rGainSetNum = g_2a_buffer[3];
		pCamCalData->Single2A.S2aAwb.rUnitGainu4R = g_2a_buffer[4];
		pCamCalData->Single2A.S2aAwb.rUnitGainu4G = g_2a_buffer[5];
		pCamCalData->Single2A.S2aAwb.rUnitGainu4B = g_2a_buffer[6];
		pCamCalData->Single2A.S2aAwb.rUnitGainu4R_mid = g_2a_buffer[7];
		pCamCalData->Single2A.S2aAwb.rUnitGainu4G_mid = g_2a_buffer[8];
		pCamCalData->Single2A.S2aAwb.rUnitGainu4B_mid = g_2a_buffer[9];
		pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low = g_2a_buffer[10];
		pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low = g_2a_buffer[11];
		pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low = g_2a_buffer[12];

		pCamCalData->Single2A.S2aAwb.rGoldGainu4R = g_2a_buffer[13];
		pCamCalData->Single2A.S2aAwb.rGoldGainu4G = g_2a_buffer[14];
		pCamCalData->Single2A.S2aAwb.rGoldGainu4B = g_2a_buffer[15];
		pCamCalData->Single2A.S2aAwb.rGoldGainu4R_mid = g_2a_buffer[16];
		pCamCalData->Single2A.S2aAwb.rGoldGainu4G_mid = g_2a_buffer[17];
		pCamCalData->Single2A.S2aAwb.rGoldGainu4B_mid = g_2a_buffer[18];
		pCamCalData->Single2A.S2aAwb.rGoldGainu4R_low = g_2a_buffer[19];
		pCamCalData->Single2A.S2aAwb.rGoldGainu4G_low = g_2a_buffer[20];
		pCamCalData->Single2A.S2aAwb.rGoldGainu4B_low = g_2a_buffer[21];

		pCamCalData->Single2A.S2aAwb.rValueR = g_2a_buffer[22];
		pCamCalData->Single2A.S2aAwb.rValueGr = g_2a_buffer[23];
		pCamCalData->Single2A.S2aAwb.rValueGb = g_2a_buffer[24];
		pCamCalData->Single2A.S2aAwb.rValueB = g_2a_buffer[25];
		pCamCalData->Single2A.S2aAwb.rGoldenR = g_2a_buffer[26];
		pCamCalData->Single2A.S2aAwb.rGoldenGr = g_2a_buffer[27];
		pCamCalData->Single2A.S2aAwb.rGoldenGb = g_2a_buffer[28];
		pCamCalData->Single2A.S2aAwb.rGoldenB = g_2a_buffer[29];

		err = CAM_CAL_ERR_NO_ERR;
	}
	return err;
}

static unsigned int do_lens_id_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	return do_lens_id_base_greenlandfront(pdata, start_addr, block_size, pGetSensorCalData);
}

unsigned int do_lens_id_base_greenlandfront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	unsigned int size_limit = sizeof(pCamCalData->LensDrvId);
	int lens_id_offset = 0x8288;
	int group_offset = 0;
	int page = 0;
	int threshold = 0;

	if (g_lens_id_first_read == false) {
		g_lens_id_first_read = true;
		if (g_groupFlag != GROUP2_FLAG && g_groupFlag != GROUP1_FLAG) {
			g_groupFlag = do_group_distinguish(pdata, pGetSensorCalData);
		}

		if (g_groupFlag == GROUP2_FLAG){
			page = GREENLANDFRONT_OTP_PAGE7;
			group_offset = GROUP_OFFSET;
			LOG_INF("In %s: [greenlandfront_debug] read group flag success, group flag = 0x%x\n", __func__, g_groupFlag);
		} else if (g_groupFlag == GROUP1_FLAG){
			page = GREENLANDFRONT_OTP_PAGE2;
			group_offset = 0;
			LOG_INF("In %s: [greenlandfront_debug] read group flag success, group flag = 0x%x\n", __func__, g_groupFlag);
		} else {
			LOG_INF("In %s: [greenlandfront_debug] read group flag fail, group flag = 0x%x\n", __func__, g_groupFlag);
		}
		memset(&pCamCalData->LensDrvId[0], 0, size_limit);

		if (block_size > size_limit) {
			error_log("lens id size can't larger than %u\n", size_limit);
			return err;
		}

		for (threshold = 0; threshold < 3; threshold++) {
			greenlandfront_set_threshold(pdata->pdrv->pi2c_client, threshold);
			read_data_size = read_data_region(pdata, (u8 *)&pCamCalData->LensDrvId[0], lens_id_offset + group_offset, 1);
			read_data_size = read_data_region(pdata, (u8 *)&g_LensId[0], lens_id_offset + group_offset, 1);
			if (read_data_size > 0) {
				break;
				LOG_INF("In %s: [greenlandfront_debug] read data success,read_data_size = 0x%x\n", __func__, read_data_size);
			}
			else {
				LOG_INF("In %s: [greenlandfront_debug] read data fail,read_data_size = 0x%x\n", __func__, read_data_size);
			}
		}
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
	}
	else {
		memcpy(pCamCalData->LensDrvId, g_LensId, sizeof(g_LensId));
		err = CAM_CAL_ERR_NO_ERR;
	}

	debug_log("======================Lens Id==================\n");
	debug_log("[Lens Id] = %x %x %x %x %x\n",
			pCamCalData->LensDrvId[0], pCamCalData->LensDrvId[1],
			pCamCalData->LensDrvId[2], pCamCalData->LensDrvId[3],
			pCamCalData->LensDrvId[4]);
	debug_log("[Lens Id] = %x %x %x %x %x\n",
			pCamCalData->LensDrvId[5], pCamCalData->LensDrvId[6],
			pCamCalData->LensDrvId[7], pCamCalData->LensDrvId[8],
			pCamCalData->LensDrvId[9]);
	debug_log("======================Lens Id==================\n");

	return err;
}

static unsigned int do_group_distinguish(struct EEPROM_DRV_FD_DATA *pdata, unsigned int *pGetSensorCalData){
	unsigned int group_flag;
	int threshold = 0;
	for (threshold = 0; threshold < 3; threshold++) {
		greenlandfront_set_threshold(pdata->pdrv->pi2c_client, threshold);
		read_data_region(pdata, (u8 *)&group_flag, 0x827A, 4);
		group_flag = group_flag & 0x00FF;
		if (group_flag == GROUP1 || group_flag == GROUP2) {
			LOG_INF("[greenlandfront_debug 0] read group group_flag = 0x%x", group_flag);
			break;
		}
		else {
			LOG_INF("[greenlandfront_debug 0] read group fail, group_flag = 0x%x", group_flag);
		}
	}

	if (group_flag == GROUP1){
		LOG_INF("[greenlandfront_debug 1] group1");
		LOG_INF("[greenlandfront_debug 1] read group group_flag = 0x%x", group_flag);
		return GROUP1_FLAG;
	} else if (group_flag == GROUP2) {
		LOG_INF("[greenlandfront_debug 2] group2");
		LOG_INF("[greenlandfront_debug 2] read group group_flag = 0x%x", group_flag);
		return GROUP2_FLAG;
	} else {
		LOG_INF("[greenlandfront_debug 3] read group fail group_flag = 0x%x", group_flag);
	}
	return 0;
}

unsigned char read_otp_8bit_greenlandfront(struct i2c_client *client,
					u16 a_u2Addr,
		  			u32 ui4_length,
		  			u8 *a_puBuff)

{
	int i4RetValue = 0;
  	char puReadCmd[2] = { (char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF) };
  	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

  	if (ui4_length > EEPROM_I2C_READ_MSG_LENGTH_MAX) {
  		must_log("exceed one transition %d bytes limitation\n",
  			 EEPROM_I2C_READ_MSG_LENGTH_MAX);
  		return -1;
  	}

  	msg[0].addr = client->addr;
  	msg[0].flags = client->flags & I2C_M_TEN;
  	msg[0].len = 2;
  	msg[0].buf = puReadCmd;

  	msg[1].addr = client->addr;
  	msg[1].flags = client->flags & I2C_M_TEN;
  	msg[1].flags |= I2C_M_RD;
  	msg[1].len = ui4_length;
  	msg[1].buf = a_puBuff;

  	i4RetValue = i2c_transfer(client->adapter, msg,
  				EEPROM_I2C_MSG_SIZE_READ);
	error_log("[greenlandfront_debug] i4RetValue = %d", i4RetValue);

  	if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
  		must_log("I2C read data failed!!\n");
  		return -1;
  	}

  	return 0;
}

static int iReadData_CAM_CAL(struct i2c_client *client,
 			    unsigned int ui4_offset,
 			    unsigned int ui4_length,
 			    unsigned char *pinputdata)
{
 	int i4ResidueSize;
	u32 u4CurrentOffset, u4Size;
	u8 *pBuff;

	i4ResidueSize = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
	do {
		u4Size = (i4ResidueSize >= EEPROM_I2C_READ_MSG_LENGTH_MAX)
			? EEPROM_I2C_READ_MSG_LENGTH_MAX : i4ResidueSize;

		if (read_otp_8bit_greenlandfront(client, (u16) u4CurrentOffset,
				     u4Size, pBuff) != 0) {
			must_log("I2C iReadData failed!!\n");
			return -1;
		}

		i4ResidueSize -= u4Size;
		u4CurrentOffset += u4Size;
		pBuff += u4Size;
	} while (i4ResidueSize > 0);

	return 0;
}

static void write_otp_8bit_greenlandfront(struct i2c_client *client, u16 reg, u8 val)
{
	int i4RetValue = 0;
	u8 buf[3];
	struct i2c_msg msg;

	buf[0] = (u8)(reg >> 8);
	buf[1] = (u8)(reg & 0xFF);
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.buf = buf;
	msg.len = sizeof(buf);

	i4RetValue = i2c_transfer(client->adapter, &msg, 1);
	error_log("[greenlandfront_debug] i4RetValue = %d", i4RetValue);
	if (i4RetValue != 1) {
		error_log("write data failed!!\n");
	}
}

//set thereshold
static int greenlandfront_set_threshold(struct i2c_client *client, u8 threshold)
{
	int threshold_reg1[3] = { 0x48, 0x48, 0x48 };
	int threshold_reg2[3] = { 0x38, 0x18, 0x58 };
	int threshold_reg3[3] = { 0x41, 0x41, 0x41 };

	if (threshold < 3 && threshold >= 0) {
		write_otp_8bit_greenlandfront(client, 0x36b0, threshold_reg1[threshold]);
		write_otp_8bit_greenlandfront(client, 0x36b1, threshold_reg2[threshold]);
		write_otp_8bit_greenlandfront(client, 0x36b2, threshold_reg3[threshold]);
		LOG_INF("[greenlandfront_debug] set_threshold %d\n", threshold);
	} else {
		LOG_INF("[greenlandfront_debug] set invalid threshold %d\n", threshold);

		return GREENLANDFRONT_OTP_RET_FAIL;
	}

	return GREENLANDFRONT_OTP_RET_SUCCESS;
}

 //set page
static int greenlandfront_set_page_and_load_data(struct i2c_client *client, int page)
{
	uint64_t Startaddress = 0;
	uint64_t EndAddress = 0;
	int delay = 0;
	int pag = 0;
	unsigned int get_byte;

	//set start address in page
	Startaddress = page * 0x200 + 0x7E00;
	//set end address in page
	EndAddress = Startaddress + 0x1ff;
	//change page
	pag = page * 2 - 1;
	write_otp_8bit_greenlandfront(client, 0x4408, (Startaddress >> 8) & 0xff);
	write_otp_8bit_greenlandfront(client, 0x4409, Startaddress & 0xff);
	write_otp_8bit_greenlandfront(client, 0x440a, (EndAddress >> 8) & 0xff);
	write_otp_8bit_greenlandfront(client, 0x440b, EndAddress & 0xff);

	// address set finished
	write_otp_8bit_greenlandfront(client, 0x4401, 0x13);
	// set page
	write_otp_8bit_greenlandfront(client, 0x4412, pag & 0xff);
	// set page finished
	write_otp_8bit_greenlandfront(client, 0x4407, 0x00);
	// manual load begin
	write_otp_8bit_greenlandfront(client, 0x4400, 0x11);
	iReadData_CAM_CAL(client, 0x4420, 4, (u8 *)&get_byte);
	LOG_INF("[greenlandfront_debug] read data: 0x%x\n", get_byte);
	LOG_INF("[greenlandfront_debug] get_byte & 0x01 = 0x%x\n", get_byte & 0x01);
	while ((get_byte & 0x01) == 0x01) {
		delay++;
		iReadData_CAM_CAL(client, 0x4420, 4, (u8 *)&get_byte);
		LOG_INF("[greenlandfront_debug] readsize: 0x%x\n", get_byte);
		LOG_INF("[greenlandfront_debug] get_byte & 0x01 = 0x%x\n", get_byte & 0x01);
		LOG_INF("[greenlandfront_debug] set_page waitting, OTP is still busy for loading %d times\n", delay);
		if (delay == 10) {
			LOG_INF("[greenlandfront_debug] set_page fail, load timeout!!!\n");

			return GREENLANDFRONT_OTP_RET_FAIL;
		}
		mdelay(10);
	}
	LOG_INF("[greenlandfront_debug] set_page success\n");

	return GREENLANDFRONT_OTP_RET_SUCCESS;
}

unsigned int Otp_read_region_greenlandfront(struct i2c_client *client,
	unsigned int addr, unsigned char *data, unsigned int size)
{
	u32 readsize = size;

	error_log("[greenlandfront_debug] readsize: 0x%x\n", size);
	error_log("[greenlandfront_debug] addr: 0x%04x\n", addr);
	if (addr < 0x8000){
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x8200 <= addr && addr <= 0x83FF){
		LOG_INF("[greenlandfront_debug] need set_page2\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE2);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x8400 <= addr && addr <= 0x85FF){
		LOG_INF("[greenlandfront_debug] need set_page3\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE2 + 1);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x8600 <= addr && addr <= 0x87FF){
		LOG_INF("[greenlandfront_debug] need set_page4\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE2 + 2);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x8800 <= addr && addr <= 0x89FF){
		LOG_INF("[greenlandfront_debug] need set_page5\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE2 + 3);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x8A00 <= addr && addr <= 0x8BFF){
		LOG_INF("[greenlandfront_debug] need set_page6\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE2 + 4);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x8C00 <= addr && addr <= 0x8DFF){
		LOG_INF("[greenlandfront_debug] need set_page7\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE7);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x8E00 <= addr && addr <= 0x8FFF){
		LOG_INF("[greenlandfront_debug] need set_page8\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE7 + 1);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x9000 <= addr && addr <= 0x91FF){
		LOG_INF("[greenlandfront_debug] need set_page9\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE7 + 2);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x9200 <= addr && addr <= 0x93FF){
		LOG_INF("[greenlandfront_debug] need set_page10\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE7 + 3);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x9400 <= addr && addr <= 0x95FF){
		LOG_INF("[greenlandfront_debug] need set_page11\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE7 + 4);
		iReadData_CAM_CAL(client, addr, size, data);
	} else if (0x9600 <= addr && addr <= 0x97FF){
		LOG_INF("[greenlandfront_debug] need set_page12\n");
		greenlandfront_set_page_and_load_data(client, GREENLANDFRONT_OTP_PAGE7 + 5);
		iReadData_CAM_CAL(client, addr, size, data);
	}

	return readsize;
}
