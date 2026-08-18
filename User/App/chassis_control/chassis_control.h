#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "controller.h"

#pragma pack(1)

/**
 * @brief 一侧腿部机构的数据 0表示左 1表示右
 * 
 */
typedef struct 
{
    float leg_pitch;                //theta
    float leg_pitch_speed;
    float x_set;
    float x;
    float chassis_pitch;            //phi
    float chassis_pitch_speed;
    
}chassis_control_s;

typedef struct 
{
    float time;
    uint8_t active;
    uint8_t done;
    float joint_zero[4];

}joint_cali_s;

typedef enum
{
    ON_GROUND = 0,
    OFF_GROUND
}off_ground_flag_e;

#pragma pack()


/**
 * @brief 底盘初始化任务
 * 
 */
void ChassisInit(void);

void ChassisTask(void);

//临时添加的校准接口，后续优化
void ChassisJointCaliStart(void);
void ChassisJointCaliAbort(void);
uint8_t ChassisJointCaliGetResult(float joint_zero[4]);

#endif
