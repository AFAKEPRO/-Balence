#ifndef __ROBOT_DEF_H
#define __ROBOT_DEF_H


#include "ins_task.h"
//#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
//#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
#define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN 213.352966f  	// 云台和底盘对齐指向相同方向时的电机角度值,若对云台有机械改动需要修改
#define YAW_ECD_SMALL_THAN_4096 1 	// ALIGN_ECD值是否小于4096 (是否小于180度),是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD -0.560327232f			// 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE -0.16187866f			// 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -0.958775818f			// 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 60    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 100.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 6            // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 300              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0


#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

#define X_MAX 4.476769531f//小轮子
#define JOINT_LF_INIT_ANGLE 0.1f
#define JOINT_RF_INIT_ANGLE 0.1f
#define JOINT_LB_INIT_ANGLE 0.1f
#define JOINT_RB_INIT_ANGLE 0.1f
#define L1 0.1f        //虚拟转轴
#define L2 0.1f        //大腿
#define L3 0.1f        //小腿
//半边机体质量（9.85kg/2）
#define HALF_BODY_M 7.555f  //8.0f
//腿的质量 (kg) 1.027*20
#define LEG_M 2.054f
#define LEG_LEN_MIN 0.065f
#define LEG_LEN_MAX 0.152f
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define RADIUS_WHEEL 0.076f             // 轮子半径
#define INIT_LEG_LEN 0.065f

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
//#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
//    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
//    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
//#error Conflict board definition! You can only define one board type.
//#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_CALI,
    CHASSIS_INIT,
    CHASSIS_DOWN,
    CHASSIS_BALANCE,
    CHASSIS_JUMP,
} chassis_mode_e;

typedef enum
{
    CHASSIS_FOLLOW_GIMBAL_YAW = 0,
    CHASSIS_NO_FOLLOW,
    CHASSIS_ROTATE,
}chassis_yaw_mode_e;


typedef enum
{
	ENEMY_RED 		 	=			0,   //红方         
	ENEMY_BLUE 			=			1,   //蓝方	
} Enemy_Color_e;


typedef enum
{
    IOUT_RESERVE = 0,
    IOUT_REMOVE,
}Iout_remove_flag_e;

typedef enum
{
	OFFLINE = 0,
	ONLINE = 1,
} online_state_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float x_set;
    float vx_set;           // 前进方向速度
    float chassis_yaw_speed_set;
    float leg_len_set;
	
	//底盘状态机
    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;
    chassis_yaw_mode_e chassis_yaw_mode;
	//标志位
    Iout_remove_flag_e leg_pid_Iout_flag;
    Iout_remove_flag_e roll_pid_Iout_flag;

} Chassis_Ctrl_Cmd_s;




/* ----------------chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
	float vx;
	float x;
	float leg_len[2];
    float chassis_yaw;
	
	chassis_mode_e chassis_mode;
	
} Chassis_Upload_Data_s;



#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)



#endif
