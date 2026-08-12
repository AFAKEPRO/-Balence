#include "motor_task.h"

#include "feed_forward.h"
#include "dji_motor.h"
#include "rmd_motor.h"
#include "dm_motor.h"


void MotorControlTask(void)
{
	static uint8_t cnt = 0; 	//设定不同电机的任务频率
	FeedForwardControl();
	DJIMotorControl();
//	RMDMotorControl();
//	if(cnt%3==0) //500hz
//		DMMotorControl();
//	if(cnt%10==0) //100hz

	
	
	cnt++;
}
