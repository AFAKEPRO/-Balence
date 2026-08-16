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
    float leg_pitch;
    float leg_pitch_speed;
    float x_set;
    float x;
    float v_fusion;
    float chassis_pitch;
    float chassis_pitch_speed;
    
}chassis_control_s;

#pragma pack()


float arm_sqrt_32(float in);
/**
 * @brief 计算腿部关节的相对角度
 * 
 */
void CalRealtiveAngle(void);

/**
 * @brief 根据平面五连杆机构解算虚拟腿长和角度
 * 
 */
void SolveFiveRod(void);

/**
 * @brief 计算由LQR得出的控制输出矩阵
 * 
 */
void CalLQR(void);

/**
 * @brief 一侧腿的虚拟腿力的重力补偿计算
 * @param 一侧腿的虚拟腿长
 * @return 一侧腿的重力补偿
 * @todo   检查数组可能越界的风险
 * 
 */
static float CalGravityFF(float leg_len);

/**
 * @brief 计算驱动轮力矩，lqr基础驱动轮叠加yaw转向补偿，在倒地模式下直接用vx
 * 
 */
void CalWheelTorque(void);

/**
 * @brief 计算虚拟腿力，腿长pid叠加roll补偿叠加重力补偿
 * 
 */
void CalVirtualForce(void);

/**
 * @brief 计算虚拟髋关节合力矩，lqr基础合力矩叠加防劈叉环
 * 
 */
void CalVirtualTorque(void);

/**
 * @brief VMC解算，计算出关节电机力矩
 * @todo 考虑是否加入限幅与奇异保护
 * 
 */
void SolveVMC(void);

/**
 * @brief 关节力矩到虚拟腿力和虚拟合力矩的反向映射
 * @todo  确定电机安装方向，可能需要改变某些值的符号
 * 
 */
void SolveRealWrench(void);

/**
 * @brief 底盘初始化任务
 * 
 */
void chassis_init(void);

#endif
