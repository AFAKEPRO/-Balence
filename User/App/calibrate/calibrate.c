#include "calibrate.h"
#include "freeRTOS.h"
#include "task.h"
#include "message_center.h"
#include "robot_def.h"
#include "bsp_flash.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "buzzer.h"
#include "tim.h"
#include "BMI088driver.h"


static Subscriber_t *gimbal_cali_sub;	// 云台反馈信息订阅者
Gimbal_Upload_Data_s gimbal_cali_data; 	// 从云台获取的反馈信息
static BuzzzerInstance *cali_buzzer;
cali_data_t cali_data_save;	//云台校准数据
cail_mods state = CAIL_OFF;
static uint8_t is_init;

static void GimbalCalibrateControl(void);

cali_data_t *CalibrateInit(void)
{
	gimbal_cali_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
	memset(&cali_data_save,0,sizeof(cali_data_t));
	is_init = 1;
	//注册蜂鸣器
	Buzzer_config_s buzzer_config = {
	.alarm_level = ALARM_LEVEL_MEDIUM,
	.loudness = 0.01,
	.octave = OCTAVE_1,
	};
	cali_buzzer=BuzzerRegister(&buzzer_config);
	AlarmSetStatus(cali_buzzer,ALARM_OFF);
	
	//读取flash
	flash_read(CALIBRATE_DATA_ADDR,(uint32_t *)&cali_data_save,sizeof(cali_data_t)/4);
	if(cali_data_save.cali_status != 1)//若无校准数据则先赋值备用数据,备用数据在 [robo_def.h]
	{
		cali_data_save.max_pitch = PITCH_MAX_ANGLE;
		cali_data_save.min_pitch = PITCH_MIN_ANGLE;
		cali_data_save.zero_pitch = (PITCH_MAX_ANGLE + PITCH_MIN_ANGLE)/2;
		cali_data_save.zero_yaw =	YAW_CHASSIS_ALIGN;
		return &cali_data_save;
	}
	else
	{
		BMI088.GyroOffset[0] = cali_data_save.gyro_offset[0];
        BMI088.GyroOffset[1] = cali_data_save.gyro_offset[1];
        BMI088.GyroOffset[2] = cali_data_save.gyro_offset[2];
        BMI088.gNorm = cali_data_save.g_norm;
        BMI088.AccelScale = 9.81f / cali_data_save.g_norm;
        BMI088.TempWhenCali = cali_data_save.temp_when_cail;
		
		return &cali_data_save;//有数据则返回数据
	}
}

cali_data_t *GetCalibrateData(void)
{
	if(is_init != 1)//若无校准数据则先校准
	{
		CalibrateInit();
		return &cali_data_save;
	}
	else
		return &cali_data_save;//有数据则返回数据
}

static void CailbrateOffControl(RC_ctrl_t *cail_rc_data)
{
	static uint32_t cali_cmd_time;
	static float calibrating_time;
	if(cail_rc_data[TEMP].rc.rocker_l_ > RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_l1 < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r_ < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r1 < -RC_CALI_VALUE_HOLE)//下内八开启校准
	{
		if(!cali_cmd_time)
			DWT_GetDeltaT(&cali_cmd_time);
		calibrating_time += DWT_GetDeltaT(&cali_cmd_time);
	}
	else
	{
		cali_cmd_time = 0;
		calibrating_time = 0;
	}
	if(calibrating_time > 2)
	{
		state = CAIL_READY;
	}
}

static void CalibrateReadyControl(RC_ctrl_t *cail_rc_data)
{
	static uint32_t cali_cmd_time;	//校准指令下达时间刻
	static float calibrating_time;	//校准指令下达后持续的时间
	
	static uint32_t run_cmd_time;		//校准进入准备模式的时间戳
	static float run_time = 0;		//这个函数运行的时间
	static float cnt = 0;			//循环控制蜂鸣器用
	
	if(!run_time)
		DWT_GetDeltaT(&run_cmd_time);
	
	//上外八校准云台
	if(cail_rc_data[TEMP].rc.rocker_l_ < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_l1 > RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r_ > RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r1 > RC_CALI_VALUE_HOLE)//上外八校准云台
	{
		if(!cali_cmd_time)
			DWT_GetDeltaT(&cali_cmd_time);
		calibrating_time += DWT_GetDeltaT(&cali_cmd_time);
		if(calibrating_time > 2)//持续时间大于2s
		{
			state = CAIL_GIMBAL;
			run_time = 0;
			cnt = 0;
		}
	}
	//下外八校准陀螺仪
	else if(cail_rc_data[TEMP].rc.rocker_l_ < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_l1 < -RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r_ > RC_CALI_VALUE_HOLE && cail_rc_data[TEMP].rc.rocker_r1 < -RC_CALI_VALUE_HOLE)//下外八校准IMU
	{
		if(!cali_cmd_time)
			DWT_GetDeltaT(&cali_cmd_time);
		calibrating_time += DWT_GetDeltaT(&cali_cmd_time);
		if(calibrating_time > 2)//持续时间大于2s
		{
			state = CAIL_IMU;
			AlarmSetOctave(cali_buzzer,OCTAVE_2);		//校准陀螺仪提前打开蜂鸣器
			AlarmSetStatus(cali_buzzer,ALARM_ON);		//因为关闭任务调度进不去buzzer_task
			run_time = 0;
			cnt = 0;
		}
	}
	//未保持2s重新计时
	else
	{
		cali_cmd_time = 0;
		calibrating_time = 0;
	}
	
	//蜂鸣器间隔0.5s报警
	run_time += DWT_GetDeltaT(&run_cmd_time);
	if(run_time < cnt + 0.3f)
	{
		AlarmSetOctave(cali_buzzer,OCTAVE_1);
		AlarmSetStatus(cali_buzzer,ALARM_ON);
	}
	else if(run_time < cnt + 0.6f)
	{
		AlarmSetStatus(cali_buzzer,ALARM_OFF);
	}
	else
	{
		cnt += 0.6f;
	}
}

static void GimbalCalibrateControl(void)
{
	static float max_pitch_temp = 0,min_pitch_temp = 0;
	static float cali_time = 0;//校准持续时间
	static uint32_t cali_start_time;//校准开始时间戳
	
	SubGetMessage(gimbal_cali_sub,&gimbal_cali_data);
	if(cali_time == 0)
	{
		DWT_GetDeltaT(&cali_start_time);
		//校准期间蜂鸣器持续报警
		AlarmSetOctave(cali_buzzer,OCTAVE_1);
		AlarmSetStatus(cali_buzzer,ALARM_ON);
	}
	if(!max_pitch_temp && !min_pitch_temp)	//赋初始值
	{
		max_pitch_temp = gimbal_cali_data.pitch_ecd_angle;
		min_pitch_temp = gimbal_cali_data.pitch_ecd_angle;
	}
	if(cali_time < 10)//校准10s
	{
		cali_time += DWT_GetDeltaT(&cali_start_time);
		if(max_pitch_temp < gimbal_cali_data.pitch_ecd_angle)	//记录最大pitch角度
		{
			max_pitch_temp = gimbal_cali_data.pitch_ecd_angle;
		}
		if(min_pitch_temp > gimbal_cali_data.pitch_ecd_angle)	//记录最小pitch角度
		{
			min_pitch_temp = gimbal_cali_data.pitch_ecd_angle;
		}
	}
	else
	{
		cali_data_save.max_pitch = max_pitch_temp;
		cali_data_save.min_pitch = min_pitch_temp;
		cali_data_save.zero_pitch = (max_pitch_temp + min_pitch_temp) / 2;
		cali_data_save.zero_yaw = gimbal_cali_data.yaw_motor_single_round_angle;
		cali_data_save.cali_status = 1;
		flash_erase_address(CALIBRATE_DATA_ADDR,1);
		while(flash_write_single_address(CALIBRATE_DATA_ADDR, (uint32_t *)&cali_data_save, sizeof(cali_data_t)/4) != 0);//写入flash
		AlarmSetStatus(cali_buzzer,ALARM_OFF);
		state = CAIL_OFF;
		//校准完成reset
		__disable_irq();
		NVIC_SystemReset();
	}
}


static void IMUCalibrateControl(void)
{
	vTaskSuspendAll();		//关闭任务调度
	while (BMI088_init(1) != BMI088_NO_ERROR){};
	
	for(int i = 0; i < 3; i++)
		cali_data_save.gyro_offset[i] = BMI088.GyroOffset[i];
	cali_data_save.g_norm = BMI088.gNorm;
	cali_data_save.temp_when_cail = BMI088.TempWhenCali;
	flash_erase_address(CALIBRATE_DATA_ADDR,1);
	while(flash_write_single_address(CALIBRATE_DATA_ADDR, (uint32_t *)&cali_data_save, sizeof(cali_data_t)/4) != 0);//写入flash
	state = CAIL_OFF;
	xTaskResumeAll();		//开启任务调度
	//校准完陀螺仪reset读取新参数
	__disable_irq();
	NVIC_SystemReset();
}

void CalibrateTask(RC_ctrl_t *cail_rc_data)
{	
	switch(state)
	{
		case CAIL_OFF:
			CailbrateOffControl(cail_rc_data);
			break;
		case CAIL_READY:
			CalibrateReadyControl(cail_rc_data);
			break;
		case CAIL_GIMBAL:
			GimbalCalibrateControl();
			break;
		case CAIL_IMU:
			IMUCalibrateControl();
			break;
		default:
			break;
	}
//	AlarmSetStatus(cali_buzzer,ALARM_ON);
//	AlarmSetOctave(cali_buzzer,OCTAVE_1);
//	DWT_Delay(300);
//	AlarmSetOctave(cali_buzzer,OCTAVE_2);
//	DWT_Delay(300);
//	AlarmSetOctave(cali_buzzer,OCTAVE_3);
//	DWT_Delay(300);
//	AlarmSetOctave(cali_buzzer,OCTAVE_4);
//	DWT_Delay(300);
//	AlarmSetOctave(cali_buzzer,OCTAVE_5);
//	DWT_Delay(300);
//	AlarmSetOctave(cali_buzzer,OCTAVE_6);
//	DWT_Delay(300);
//	AlarmSetOctave(cali_buzzer,OCTAVE_7);
//	DWT_Delay(300);
//	AlarmSetStatus(cali_buzzer,ALARM_OFF);
}

