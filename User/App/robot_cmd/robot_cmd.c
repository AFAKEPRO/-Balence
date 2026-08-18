// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "calibrate.h"
// module
#include "message_center.h"
#include "remote_control.h"
#include "INS_task.h"
#include "dji_motor.h"
#include "user_lib.h"
#include "buzzer.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "core_cm4.h"


//#define yaw_align_angle 151.346f

RC_ctrl_t *rc_data;              			// 遥控器数据,初始化时返回
attitude_t *ins_data;
cali_data_t *calibrate_data;

static Publisher_t *chassis_cmd_pub;   		// 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; 		// 底盘反馈信息订阅者
Chassis_Ctrl_Cmd_s chassis_cmd_send;      	// 发送给底盘应用的信息,包括控制信息和UI绘制相关
Chassis_Upload_Data_s chassis_fetch_data; 	// 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等


static Robot_Status_e robot_state; 			// 机器人整体工作状态
//static float yaw_align_angle;				//重新定义变量储存对齐角度，提高可读性，避免对cali_data的直接操作


static void RemoteControlSet(void)
{

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
	chassis_cmd_send.leg_len_set = 0.001f  * (float)rc_data[TEMP].rc.rocker_r_;     //右摇杆水平方向
	chassis_cmd_send.x_set += 0.5f * (float)rc_data[TEMP].rc.rocker_l1;             //左摇杆竖直方向，注意这是叠加的
    chassis_cmd_send.vx_set = 0.05f * (float)rc_data[TEMP].rc.rocker_l1;            //左摇杆竖直方向
    chassis_cmd_send.chassis_yaw_speed_set = 0.5f * (float)rc_data[TEMP].rc.rocker_l_;           //左摇杆水平方向，注意这是叠加的
	
    chassis_cmd_send.last_chassis_mode = chassis_cmd_send.chassis_mode;

/*==============================无力检测==============================*/
	if (switch_is_down(rc_data[TEMP].rc.switch_left)) 				// 左侧开关状态[下],机器人停止
	{
		robot_state = ROBOT_STOP;                                       
	}
    else
	{
        robot_state = ROBOT_READY;
	}
/*==============================右拨杆==============================*/
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) 				    // 右侧开关状态[下],无力模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) 				// 右侧开关状态[上],平衡模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_BALANCE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 			    // 右侧开关状态[中],倒地模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_DOWN;
    }
/*==============================左拨杆==============================*/
    if (switch_is_up(rc_data[TEMP].rc.switch_left)) 				    // 左侧开关状态[上],底盘跟随云台
    {
        chassis_cmd_send.chassis_yaw_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) 			    // 左侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_yaw_mode = CHASSIS_NO_FOLLOW;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_left)) 				// 左侧开关状态为[下],小陀螺模式，现在只是先占个坑位，还没加入这个功能
	{
        
		chassis_cmd_send.chassis_yaw_mode = CHASSIS_ROTATE;
    }
/*==============================校准检测==============================*/
	uint8_t both_switch_down = switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right);
    
    if (both_switch_down || CalibrateJointRequested())
    {
    CalibrateTask(rc_data);
    }
}

static void ControlModeChangeTransit(void)
{   
    //切入零力模式
    if(chassis_cmd_send.last_chassis_mode != CHASSIS_ZERO_FORCE && chassis_cmd_send.chassis_mode ==CHASSIS_ZERO_FORCE)
    {
        chassis_cmd_send.x_set = chassis_fetch_data.x;
        chassis_cmd_send.chassis_yaw_speed_set = chassis_fetch_data.chassis_yaw;
        chassis_cmd_send.leg_len_set = 0.065;
    }
    //切入倒地模式
    if(chassis_cmd_send.last_chassis_mode != CHASSIS_DOWN && chassis_cmd_send.chassis_mode ==CHASSIS_DOWN)
    {
        chassis_cmd_send.x_set = chassis_fetch_data.x;
        chassis_cmd_send.chassis_yaw_speed_set = chassis_fetch_data.chassis_yaw;
        chassis_cmd_send.leg_len_set = 0.065;
    }
    //切入初始化模式
    if(chassis_cmd_send.last_chassis_mode != CHASSIS_DOWN && chassis_cmd_send.chassis_mode ==CHASSIS_DOWN)
    {
        chassis_cmd_send.x_set = chassis_fetch_data.x;
        chassis_cmd_send.chassis_yaw_speed_set = chassis_fetch_data.chassis_yaw;
        chassis_cmd_send.leg_len_set = INIT_LEG_LEN;
        chassis_cmd_send.leg_pid_Iout_flag = IOUT_REMOVE; 
        chassis_cmd_send.roll_pid_Iout_flag = IOUT_REMOVE;
    }
    //切入平衡模式
    if(chassis_cmd_send.last_chassis_mode != CHASSIS_BALANCE && chassis_cmd_send.chassis_mode ==CHASSIS_BALANCE)
    {
        chassis_cmd_send.x_set = chassis_fetch_data.x;
        chassis_cmd_send.chassis_yaw_speed_set = chassis_fetch_data.chassis_yaw;
        chassis_cmd_send.leg_len_set = 0.065;
        chassis_cmd_send.leg_pid_Iout_flag = IOUT_REMOVE; 
        chassis_cmd_send.roll_pid_Iout_flag = IOUT_REMOVE;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler(void)
{
    // 遥控器掉线无力
	if(rc_data->rc_state == 0)
	{
		robot_state = ROBOT_STOP;
        CalibrateJointCancel();
	}
    if(CalibrateJointRequested())
    {
        chassis_cmd_send.chassis_mode = CHASSIS_CALI;
        return;
    }
    if (robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {   
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

void RobotCmdInit(void)
{
	rc_data = RemoteControlInit(&huart3);
	ins_data = GetInsData();
	calibrate_data = CalibrateInit();
	
	chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    chassis_cmd_send.x_set = 0;
    chassis_cmd_send.chassis_yaw_speed_set = 0;
	
//	gimbal_cmd_send.pitch = 0;		作用不明，保留
	robot_state = ROBOT_READY;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);

	
//	if(rc_data[TEMP].key[0].keys == 0x00 && rc_data[TEMP].key[1].keys == 0x00 && rc_data[TEMP].key[2].keys == 0)	//键盘有输入键盘控制
	
	RemoteControlSet();
    ControlModeChangeTransit();
	
//	else	键盘无输入遥控器控制
//    {
//		MouseKeySet();
//	}
	
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
	
    // 设置视觉发送数据,还需增加加速度和角速度数据
//	VisionTask(RadToDeg(ins_data->Roll),-RadToDeg(ins_data->Pitch),RadToDeg(ins_data->Yaw));

    // 推送消息,双板通信,视觉通信等
//	shoot_cmd_send.rest_heat = chassis_fetch_data.rest_heat;
//	shoot_cmd_send.heat_limit = chassis_fetch_data.heat_limit;
//	chassis_cmd_send.loader_mode = shoot_cmd_send.load_mode;
//	chassis_cmd_send.friction_mode = shoot_cmd_send.friction_mode;
//	chassis_cmd_send.gimbal_mode = gimbal_cmd_send.gimbal_mode;
//	chassis_cmd_send.vision_state = vision_fetch_data.vision_state;
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
	
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
}
