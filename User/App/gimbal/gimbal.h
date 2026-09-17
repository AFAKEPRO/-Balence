/**
 * @file gimbal.h
 * @author 天
 * @brief 云台和其他数据反馈接收，以及跟随云台 yaw 的位置环
 * @version 0.2
 * @date 2026-09-15
 *
 * @attention 未完工：跟随环增益是初值，未上机整定
 *
 * @copyright Copyright (c) 2026 PKA all rights reserved
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include "stdint.h"

/* ======================= 板间 CAN 帧一览（CAN2）=======================
 * A板(0standard, 云台板) 和 C板(本工程, 底盘板) 挂同一条 CAN2。
 * 下面是两个工程的代码里确认过的，改动任何一处之前先回看这张表。
 *
 * 【A板 → C板】本工程在收
 *   0x160/0x161/0x162  遥控 DBUS 18B 拆三帧（remote_control.c 拼帧）
 *   0x207              云台 yaw GM6020(ID3) 反馈 —— 本文件（反馈 ID = 0x204 + 电机ID）
 *   0x201/0x204        左右轮电调反馈，电调自己发的，不是 A板 发的
 *
 * 【A板 自己在发 —— C板 绝不能发这些 ID】
 *   0x160/0x161/0x162  遥控转发
 *   0x1FF              云台 yaw 电流（GM6020 ID1-4 组，Data[4..5]）
 *   0x180              云台 yaw 编码器中值 offset（上电一次）
 *   0x202              功率/缓冲能量（Data[0..1]=功率上限，Data[2..3]=缓冲能量，[4..7] 空）
 *   0x700              ⚠ 地雷：全 0 一帧会让总线上所有 M3508 进改 ID 模式
 *
 * 【C板 → A板】目前只有一条
 *   0x200              两轮电流指令（左轮 tx_id=1、右轮 tx_id=4，都落 ID1-4 组）。
 *                      这是控制量，不是底盘状态。腿长/腿角/功率等状态一条都没往回发。
 *
 * 【A板 在等、C板 还没发】
 *   0x330/0x331        左腿五连杆角 → A板 ChassisLegPostureAng[4]（user_UI.c 画腿用）
 *   0x211              超级电容（A板侧清单里的，未逐行核对）
 * ===================================================================== */

typedef struct
{
    uint16_t last_ecd;     // 上一次读取的单圈编码器值
    uint16_t ecd;          // 0-8191
    int16_t speed_rpm;     // 电机转速(rpm)。本模块不参与控制，只用于调试判断云台在不在动
    int32_t total_round;   // 多圈累计圈数
    float relative_angle;  // 云台相对底盘的 yaw 角(rad)，多圈累计、不折叠
    uint32_t frame_cnt;    // 累计收到的帧数（调试用：在涨就说明在收）
} GimbalYawMeasure_s;

typedef struct
{
    GimbalYawMeasure_s yaw; // 来自 0x207
    /* 以后每一路云台下行数据各加一个成员 */
} GimbalData_s;

/**
 * @brief 注册本文件所有云台下行帧的接收，并给每帧注册离线守护进程
 * @note  整个工程只能调用一次：重复注册同一个 (rx_id, can_handle) 会撞
 *        bsp_can.c:71 的重复检查，那里是 while(1) 卡死，不是"多收一份"
 */
void GimbalInit(void);

/**
 * @brief 取云台下行数据总集，给所有 App 用
 * @return 总集指针，始终有效（GimbalInit 之前也在，只是数据全 0）
 */
GimbalData_s *GimbalGetData(void);

/**
 * @brief 0x207（云台 yaw 反馈）是否在正常收
 * @return uint8_t 1:在线 0:离线（未初始化也算离线）
 */
uint8_t GimbalYawIsOnline(void);

/**
 * @brief 跟随云台 yaw 的位置环：把"云台相对底盘的角度"算成底盘 yaw 速度指令
 *
 * @param[in] follow_on 1:本拍跟随  0:不跟随（同时复位锁存，下次跟随重新锁存目标）
 * @return float 底盘 yaw 速度指令(rad/s)，给底盘 yaw 速度环当 ref。
 *               follow_on=0 或云台离线时恒返回 0
 *
 * @note 每拍调用一次，且必须和消费它的 yaw 速度环同频：调用点在
 *       chassis_control.c 的 SetWheelTorque()，不要放进 CAN 中断回调
 */
float GimbalYawFollowLoop(uint8_t follow_on);

#endif