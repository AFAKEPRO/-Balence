#include "robot.h"
#include "robot_task.h"
#include "robot_def.h"

#include "bsp_delay.h"
#include "INS_task.h"
#include "bsp_dwt.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "buzzer.h"

#include "robot_cmd.h"
#include "gimbal.h"
#include "shoot.h"
#include "chassis.h"
#include "vision.h"
#include "cmd_send.h"

//static BuzzzerInstance *imu_cail_buzzer;
void RobotInit(void)
{
	HAL_Delay(100);
	DWT_Init(168);
	
	while (BMI088_init(0) != BMI088_NO_ERROR){};
	BuzzerInit();
	RobotCmdInit();
	GimbalInit();
	ChassisInit();
    ShootInit();		
	VisionInit();
//	CmdRevInit();
	
	OSTaskInit();

}

void RobotTask(void)
{
    RobotCMDTask();
    GimbalTask();
    ShootTask();
    ChassisTask();
//	CmdRevTask();
}
