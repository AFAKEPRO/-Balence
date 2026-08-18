#ifndef __CALIBRATE_H
#define __CALIBRATE_H

#include "stdint.h"
#include "remote_control.h"

#define CALIBRATE_DATA_ADDR ADDR_FLASH_SECTOR_11
#define RC_CALI_VALUE_HOLE 600        //遥控器外八或者内八 阈值判定， 遥控器摇杆最大是 660 只要大于630 就认为到最大


typedef enum
{
	CAIL_OFF = 0,
	CAIL_READY,
	CAIL_IMU,
	CAIL_JOINT,
}  cail_mods;

typedef enum
{
	NONE_VOLID = 0,
	ONLY_IMU,
	ONLY_JOINT,
	BOTH_VALID,
}cali_status_e;

typedef struct
{
	cali_status_e cali_valid_flag;

	float gyro_offset[3];
	float g_norm;
	float temp_when_cail;

	float joint_zero[4];
	
} cali_data_t;

cali_data_t *CalibrateInit(void);
cali_data_t *GetCalibrateData(void);
uint8_t CalibrateJointRequested(void);
void CalibrateJointCancel(void);
void CalibrateTask(RC_ctrl_t *cail_rc_data);

#endif
