#ifndef __RMD_MOTOR_H
#define __RMD_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define RMD_MOTOR_CNT 1

typedef struct
{
	int8_t temperature;
	int16_t iq;				//转矩电流
	int16_t speed;
	float total_angle;
} RMD_Motor_Measure_s;

typedef struct
{
    RMD_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例
	DaemonInstance *daemon;

    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    uint32_t feed_cnt;
    float dt;
} RMDMotorInstance;



RMDMotorInstance *RMDMotorInit(Motor_Init_Config_s *config);

void RMDMotorControl(void);

void RMDMotorZeroCali(void);

void RMDMotorChangeFeed(RMDMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

void RMDMotorStop(RMDMotorInstance *motor);

void RMDMotorEnable(RMDMotorInstance *motor);

void RMDMotorOuterLoop(RMDMotorInstance *motor, Closeloop_Type_e outer_loop);

void RMDMotorSetRef(RMDMotorInstance *motor, float ref);

#endif
