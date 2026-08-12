#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 1

#define DM_P_MIN  (-3.1415926f)
#define DM_P_MAX  3.1415926f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

typedef enum
{
	DM_DISABLE = 0,
	DM_ENABLE 	= 1,
	OVER_VOLTAGE = 8,
	LACK_VOLTAGE = 9,
	OVER_CURRENT = 'A',
	OVER_MOS_TEMPERATURE = 'B',
	OVER_MOTOR_TEMPERATURE = 'C',
	COMMUNICATION_LOST = 'D',
	OVER_LOAD = 'E',
	
} DMMotor_Error_e;

typedef struct 
{
    uint8_t id;
    DMMotor_Error_e state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;

}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;

typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
	Motor_Controller_s motor_controller;
	
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
}DMMotorInstance;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit(void);

void DMMotorControl(void);
#endif // !DMMOTOR
