#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "INS_task.h"
#include "message_center.h"
#include "feed_forward.h"
#include "calibrate.h"
//#include "bmi088.h"


static attitude_t *gimba_IMU_data;
DJIMotorInstance *pitch_motor;
DJIMotorInstance *yaw_motor;
FeedForwardInstance *yaw_ff;

FeedForwardInstance DEBUG_pitch_ff;
DJIMotorInstance DEBUG_pitch_motor;
DJIMotorInstance DEBUG_yaw_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

void GimbalInit(void)
{
	
    gimba_IMU_data = GetInsData(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
	FeedForward_Init_Config_s yaw_ff_config = {
		.K1 = 0,	//800
		.K2 = 0,
		.RC = 0.03,
		.max_out = 2.0,
	};
	yaw_ff = FeedForwardInit(&yaw_ff_config);
	
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
//			.rx_id = 0,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.0f, // 
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.0,
                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate),
				.CoefA = 0.3,
				.CoefB = 0.5,
                .IntegralLimit = 10.0f,
                .MaxOut = 6,
            },
            .speed_PID = {
                .Kp = 0.0f,  // 
                .Ki = 0, // 
                .Kd = -0.000,
                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter),
                .IntegralLimit = 30,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Yaw,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
			.speed_feedforward_ptr = &yaw_ff->final_out,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
			.feedforward_flag = SPEED_FEEDFORWARD,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
			.angle_rad_format_flag = FORMAT_ENABLE,
        },
        .motor_type = GM6020};
	
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.0f, // 
                .Ki = 0.0,		//
                .Kd = 0.0f,		//
                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate),
				.CoefA = 0.01,
				.CoefB = 0.1,
                .IntegralLimit = 10,
                .MaxOut = 30,
            },
            .speed_PID = {
                .Kp = 0.0f,  // 
                .Ki = 0.0f, // 
                .Kd = 0,   // 
                .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
				.Output_LPF_RC = 0.008,
                .IntegralLimit = 10000,
                .MaxOut = 23333,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[1]),
//			.speed_feedforward_ptr = &pitch_ff->final_out,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
			.feedforward_flag = FEEDFORWARD_NONE,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = (Closeloop_Type_e)(SPEED_LOOP | ANGLE_LOOP),
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
			.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);
	
	FeedForwardSetRef(yaw_ff, &yaw_motor->motor_controller.pid_ref);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
	
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask(void)
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
	DEBUG_pitch_motor = *pitch_motor;
	DEBUG_yaw_motor = *yaw_motor;
	DEBUG_pitch_ff = *yaw_ff;
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
		DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);

        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
	gimbal_feedback_data.pitch_ecd_angle = pitch_motor->measure.total_angle;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
