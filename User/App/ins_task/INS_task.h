/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef INS_Task_H
#define INS_Task_H
#include "stdint.h"
#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"


#define TEMPERATURE_PID_KP 1000.0f //温度控制PID的kp	1600
#define TEMPERATURE_PID_KI 20.0f    //温度控制PID的ki	0.2
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd	0

#define TEMPERATURE_PID_MAX_OUT   2000.0f //温度控制PID的max_out	4500
#define TEMPERATURE_PID_MAX_IOUT 	300.0f  //温度控制PID的max_iout	4400

//#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1


//----------------------
#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2

#define INS_TASK_PERIOD 1


typedef struct
{
    float q[4]; // 四元数估计值

    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
	float Gyro[3];  // 角速度		roll pitch yaw
    float Accel[3]; // 加速度
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

typedef struct
{
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    // 还需要增加角速度数据
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} attitude_t; // 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)

/**
 * @brief 用于修正安装误差的参数,demo中可无视
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t INS;

attitude_t *GetInsData(void);
void INS_Init(void);
extern void INS_task(void);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void IMU_Temperature_Ctrl(void);



#endif
