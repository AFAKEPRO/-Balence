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
	if(cnt%1==0) //dmmotor @ task period (2ms = 500hz)
		DMMotorControl();
//	if(cnt%10==0) //100hz

	
	
	cnt++;
}
