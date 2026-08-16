
#include "chassis_control.h"
#include "robot_def.h"
#include "INS_task.h"
#include "message_center.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "bsp_dwt.h"
#include "controller.h"
#include "math.h"

#define CalKArray(len,a,b,c)		(a*len*len + b*len + c)
#define LQR(ptr , k0,k1,k2,k3,k4,k5)                 \
(                                                    \
	- (k0)*(ptr)->leg_pitch                            \
	- (k1)*(ptr)->leg_pitch_speed                      \
	- (k2)*(loop_fp32_constrain(((ptr)->x - (ptr)->x_set)  /*cos((ptr)->yaw_feedback->relative_angle )*/,-X_MAX, X_MAX)) \
 	- (k3)* v_fusion/*((ptr)->vx) */                               \
	- (k4)*(ptr)->chassis_pitch                   \
	- (k5)*(ptr)->chassis_pitch_speed                  \
) 

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令                                             // !ONE_BOARD
Chassis_Ctrl_Cmd_s chassis_cmd_recv;         		// 底盘接收到的控制命令
Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static attitude_t *chassis_IMU_data;

//驱动轮电机
DJIMotorInstance *wheel_l, *wheel_r;
DJIMotorInstance DEBUG_WHEEL;

//关节电机
DMMotorInstance *joint_lf, *joint_rf, *joint_lb, *joint_rb;
DMMotorInstance DEBUG_JOINT;

PIDInstance leg_pid[2], roll_pid;      					//虚拟腿力pid控制器
PIDInstance delta_ang_pid, delta_ang_speed_pid;         //虚拟力矩pid控制器
PIDInstance yaw_pid;                					//驱动轮转向yaw补偿

static float leg_len[2], ang[2];
static float leg_len_avg, ang_avg;
static float ang_1[2], ang_2[2], ang_3[2], ang_4[2];    //Five_Rod_solve  规定ang_1为五连杆的前一个角度

static float T[2], Tp[2], T_Wheel[2];
static float Virtual_Force[2], Virtual_Torque[2];       //VMC
static float Torque_f[2], Torque_b[2];
static float Actual_Force[2], Actual_Torque[2];

chassis_control_s state_vector;                			//状态向量
static float dwt_cnt, dt;



void ChassisInit()
{	
	chassis_IMU_data = GetInsData();
	//底盘的六个电机不用配置pid控制器，所以用开环跳过pid计算
	Motor_Init_Config_s wheel_motor_config = {
		.can_init_config.can_handle = &hcan2,
		.controller_setting_init_config = {
			.outer_loop_type = OPEN_LOOP,
			.close_loop_type = OPEN_LOOP,
			.feedforward_flag = FEEDFORWARD_NONE,
		},
		.motor_type = M3508,
	};
	//@todo确定一下电机要不要反转
	wheel_motor_config.can_init_config.tx_id = 1;
	wheel_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
	wheel_l = DJIMotorInit(&wheel_motor_config);

	wheel_motor_config.can_init_config.tx_id = 2;
	wheel_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
	wheel_r = DJIMotorInit(&wheel_motor_config);

	//关节电机初始化
	Motor_Init_Config_s joint_motor_config = {
		.can_init_config.can_handle = &hcan1,
		.controller_setting_init_config = {
			.outer_loop_type = OPEN_LOOP,
			.close_loop_type = OPEN_LOOP,
			.feedforward_flag = FEEDFORWARD_NONE,
		},
		.motor_type = DMJ4310,
	};

	joint_motor_config.can_init_config.tx_id = 1;
	joint_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
	joint_lf = DMMotorInit(&joint_motor_config);

	joint_motor_config.can_init_config.tx_id = 2;
	joint_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
	joint_rf = DMMotorInit(&joint_motor_config);

	joint_motor_config.can_init_config.tx_id = 3;
	joint_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
	joint_lb = DMMotorInit(&joint_motor_config);

	joint_motor_config.can_init_config.tx_id = 4;
	joint_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
	joint_rb = DMMotorInit(&joint_motor_config);


	//PID控制器初始化
	//腿长补偿
	PID_Init_Config_s leg_pid_config =
	{
		.Kp = 50.0f, 
		.Ki = 0,   
		.Kd = 4.0f,
		.Output_LPF_RC = 0.1,
		.IntegralLimit = 0,
		.Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
		.MaxOut = 1000,
	};
	PIDInit(&leg_pid[0],&leg_pid_config);
	PIDInit(&leg_pid[1],&leg_pid_config);
	//roll轴补偿
	PID_Init_Config_s roll_pid_config =
	{
		.Kp = 50.0f, 
		.Ki = 0,   
		.Kd = 4.0f,
		.Output_LPF_RC = 0.1,
		.IntegralLimit = 0,
		.Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
		.MaxOut = 1000,
	};
	PIDInit(&roll_pid,&roll_pid_config);
	//防劈叉角度环
	PID_Init_Config_s delta_ang_pid_config =
	{
		.Kp = 50.0f, 
		.Ki = 0,   
		.Kd = 4.0f,
		.Output_LPF_RC = 0.1,
		.IntegralLimit = 0,
		.Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
		.MaxOut = 1000,
	};
	PIDInit(&delta_ang_pid,&delta_ang_pid_config);
	//防劈叉速度环
	PID_Init_Config_s delta_ang_speed_pid_config =
	{
		.Kp = 50.0f, 
		.Ki = 0,   
		.Kd = 4.0f,
		.Output_LPF_RC = 0.1,
		.IntegralLimit = 0,
		.Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
		.MaxOut = 1000,
	};
	PIDInit(&delta_ang_speed_pid,&delta_ang_speed_pid_config);
	//yaw转向补偿
	PID_Init_Config_s yaw_pid_config =
	{
		.Kp = 50.0f, 
		.Ki = 0,   
		.Kd = 4.0f,
		.Output_LPF_RC = 0.1,
		.IntegralLimit = 0,
		.Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
		.MaxOut = 1000,
	};
	PIDInit(&yaw_pid,&yaw_pid_config);

	//标定电机编码器的零刻度线
	DMMotorCaliEncoder(joint_lf);
	DMMotorCaliEncoder(joint_rf);
	DMMotorCaliEncoder(joint_lb);
	DMMotorCaliEncoder(joint_rb);

	DWT_GetDeltaT(dwt_cnt);

	chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}



/**
 * @brief 计算腿部关节的相对角度
 * 
 */
static void CalRealtiveAngle()
{
	ang_1[0] = joint_lf->measure.position + JOINT_LF_INIT_ANGLE;
	ang_1[1] = joint_rf->measure.position + JOINT_RF_INIT_ANGLE;
	ang_2[0] = joint_lb->measure.position + JOINT_LB_INIT_ANGLE;
	ang_2[1] = joint_rb->measure.position + JOINT_RB_INIT_ANGLE;
}


static float arm_sqrt_32(float in)
{
	float out;
	arm_sqrt_f32(in,&out);
	return out;
}


/*
                            L1 
    ___________________________________
              /ang2 b  |       \  ang_1 a      
             /         |        \   
            /          |         \  L2
           /           |          \
          /            |           \             ----->  正
          \            |           /
           \  ang4 d   |leg_len   /  ang3 c
            \          |         /
             \         |        /
              \        |       /  L3
               \       |      /
                \      |     /
                 \     |    /
                  \    |   /		
                     *``*
                    *    *
                     *..*
*/

/**
 * @brief 根据平面五连杆机构解算虚拟腿长和角度
 * 
 */
static void SolveFiveRod()
{	
	float Xb, Xd, Yb, Yd, Xc, Yc;
	float a, b, c, d, A, B, C;

	//左右两边腿部分别计算 先左后右
	for(int i=0; i<2; i++)
	{
		a = ang_1[i];
		b = ang_2[i];
		Xb = L1+L2*arm_cos_f32(a);
		Yb = L2*arm_sin_f32(a);
		Xd = -L1+L2*arm_cos_f32(b);
		Yd = L2*arm_sin_f32(b);
		A = 2*L3*(Xb-Xd);
		B=2*L3*(Yb-Yd);
		C=(Xb-Xd)*(Xb-Xd)+(Yb-Yd)*(Yb-Yd);
		c=2*atanf((B+arm_sqrt_32(A*A+B*B-C*C))/(A-C));
		Xc=Xb+L3*arm_cos_f32(c);
		Yc=Yb+L3*arm_sin_f32(c);
		d=atanf((Yc-Yd)/(Xc-Xd));
		
		//得到虚拟腿长和虚拟腿角
		leg_len[i] = arm_sqrt_32(Xc*Xc+Yc*Yc);
		ang[i] = atan2f(Yc, Xc);

		ang_3[i] = c;
		ang_4[i] = d;
	}
	
	leg_len_avg = (leg_len[0]+leg_len[1])/2;
	ang_avg = (ang[0]+ang[1])/2;

}

/**
 * @brief 更新空间状态向量的值
 * 
 */
static void UpdataStateVector()
{	
	static float leg_pitch_l, leg_pitch_r, leg_pitch_speed_l, leg_pitch_speed_r;
	static float leg_pitch_last_l, leg_pitch_last_r;

	state_vector.chassis_pitch = chassis_IMU_data->Pitch;
	state_vector.chassis_pitch_speed = chassis_IMU_data->Gyro[1];

	leg_pitch_last_l = leg_pitch_l;
	leg_pitch_last_r = leg_pitch_r;
	leg_pitch_l = ang[0] - state_vector.chassis_pitch - PI/2;
	leg_pitch_r = ang[1] - state_vector.chassis_pitch - PI/2;
	state_vector.leg_pitch = (leg_pitch_l + leg_pitch_r)/2;

	leg_pitch_speed_l = (leg_pitch_l - leg_pitch_last_l)/dt;
	leg_pitch_speed_r = (leg_pitch_r - leg_pitch_last_r)/dt;
}

/**
 * @brief 计算由LQR得出的控制输出矩阵
 * 
 */
static void CalLQR()
{	
	float K[12];

	K[0] = CalKArray(leg_len_avg,  83.492f,-123.701f, -37.039f);
    K[1] = CalKArray(leg_len_avg,  -7.552f, -19.745f,  -3.523f);
    K[2] = CalKArray(leg_len_avg,  30.848f, -26.459f, -23.099f);
    K[3] = CalKArray(leg_len_avg, -22.822f,   9.077f, -21.858f);
    K[4] = CalKArray(leg_len_avg,  65.444f, -61.777f,  22.881f);
    K[5] = CalKArray(leg_len_avg,  19.408f, -16.899f,   6.008f);
    K[6] = CalKArray(leg_len_avg,  54.768f, -67.650f,  41.264f);
    K[7] = CalKArray(leg_len_avg,   0.676f,  -1.203f,   4.183f);
    K[8] = CalKArray(leg_len_avg,  55.097f, -64.510f,  28.681f);
    K[9] = CalKArray(leg_len_avg,  46.604f, -46.241f,  18.080f);
    K[10] = CalKArray(leg_len_avg, -67.267f,  53.441f,  18.030f);
    K[11] = CalKArray(leg_len_avg, -17.167f,  13.674f,   1.482f);

	for(int i = 0;i<2;i++)
	{
		T[i] = 0.5*LQR(&state_vector, K[0], K[1], K[2], K[3], K[4], K[5]);
		Tp[i] = 0.5*LQR(&state_vector, K[6], K[7], K[8], K[9], K[10], K[11]) * 3.0f;
	}
}

/**
 * @brief 一侧腿的虚拟腿力的重力补偿计算
 * @param leg_len 一侧的腿长
 * @return 一侧腿的重力补偿
 * @todo   检查数组可能越界的风险
 * 
 */
static float CalGravityFF(float leg_len)
{

	static int i=0;
	static float Virtual_Force_FF;
	static float leg_centroid[6]={0.1244,0.12567,0.13454,0.14671,0.161,0.17748};//{0.04096,0.04421,0.04925,0.05511,0.06142,0.06840}{0.0369,0.0376,0.0402,0.0443,0.0486,0.054}
	static float leg_body_centroid[6]={0.13835,0.16718,0.19691,0.22703,0.25740 ,0.28801};//{0.11935,0.17083,0.20279,0.21987,0.28279,0.30153}{0.11489,0.15452,0.18668,0.21973,0.25973,0.29306}{0.013775,0.017022,0.020225,0.023467,0.02682,0.030062}
	static float first_leg_len=0.15f;//[6]={0.15,0.19,0.23,0.27,0.31,0.35};
	
	i=(int)((leg_len - first_leg_len)/0.04f);
	Virtual_Force_FF = (HALF_BODY_M+LEG_M*(leg_body_centroid[i]/leg_centroid[i]))*gNORM; /*+ 10.0f*/
	return Virtual_Force_FF;
}

/**
 * @brief 计算驱动轮力矩，lqr基础驱动轮叠加yaw转向补偿，在倒地模式下直接用vx
 * 
 */
static void CalWheelTorque()
{	
	static float ref, measure;

	measure = chassis_IMU_data->Yaw;
	PIDCalculate(&yaw_pid, measure, ref);

	T_Wheel[0] = T[0] + yaw_pid.Output;
	T_Wheel[1] = T[1] - yaw_pid.Output;
}

/**
 * @brief 计算虚拟腿力，腿长pid叠加roll补偿叠加重力补偿
 * 
 */
static void CalVirtualForce()
{	
	static float ref, measure;

	measure = chassis_IMU_data->Roll;
	PIDCalculate(&roll_pid, measure, ref);

	Virtual_Force[0] = leg_pid[0].Output + roll_pid.Output + CalGravityFF(leg_len[0]);
	Virtual_Force[1] = leg_pid[1].Output - roll_pid.Output + CalGravityFF(leg_len[1]);
}

/**
 * @brief 计算虚拟髋关节合力矩，lqr基础合力矩叠加防劈叉环
 * 
 */
static void CalVirtualTorque()
{	
	static float ang_measure, speed_measure;


	PIDCalculate(&delta_ang_pid, ang_measure, 0);
	PIDCalculate(&delta_ang_speed_pid, speed_measure, delta_ang_pid.Output);

	Virtual_Torque[0] = Tp[0] + delta_ang_speed_pid.Output;
	Virtual_Torque[1] = Tp[1] - delta_ang_speed_pid.Output;
}

/**
 * @brief VMC解算，计算出关节电机力矩
 * @todo 考虑是否加入限幅与奇异保护
 * 
 */
static void SolveVMC()
{
	for(int i=0; i<2; i++)
	{
		float a = ang_1[i], b = ang_2[i], c = ang_3[i], d = ang_4[i], angle = ang[i];
		float F = Virtual_Force[i], Ft = Virtual_Torque[i]/leg_len[i];
		//一侧腿的前后两个关节电机解算
		Torque_f[i] = (L2 * arm_cos_f32(a - c + PI/2) * (Ft * arm_cos_f32(angle - d) + F * arm_sin_f32(angle - d)))/(arm_cos_f32(angle - c)*arm_sin_f32(angle - d) - arm_cos_f32(angle - d)*arm_sin_f32(angle - c));
		Torque_b[i] = -(L2*arm_cos_f32(b - d + PI/2) * (Ft * arm_cos_f32(angle - c) + F * arm_sin_f32(angle - c)))/(arm_cos_f32(angle - c)*arm_sin_f32(angle - d) - arm_cos_f32(angle - d)*arm_sin_f32(angle - c));
	}
}

/**
 * @brief VMC逆解，根据关节电机的输出力矩计算实际腿力和实际合力矩
 * 
 */
static void SolveRealWrench()
{	
	float a = ang_1[0], b = ang_2[0], c = ang_3[0], d = ang_4[0], angle = ang[0];
	float g = joint_lb->measure.torque, h = joint_lf->measure.torque;

	Actual_Torque[0] = (-leg_len[0] * ((h * arm_sin_f32(c-angle))/(L2 * arm_sin_f32(c-a)) + (g * arm_sin_f32(d - angle))/(L2 * arm_sin_f32(d - b))));
	Actual_Force[0]  = -((h * arm_cos_f32(c-angle))/(L2 * arm_sin_f32(c-a)) + (g * arm_cos_f32(d - angle))/(L2 * arm_sin_f32(d - b)));

	a = ang_1[1], b = ang_2[1], c = ang_3[1], d = ang_4[1], angle = ang[1];
	g = joint_rb->measure.torque, h = joint_rf->measure.torque;

	Actual_Torque[1] = (-leg_len[1] * ((h * arm_sin_f32(c-angle))/(L2 * arm_sin_f32(c-a)) + (g * arm_sin_f32(d - angle))/(L2 * arm_sin_f32(d - b))));
	Actual_Force[1]  = -((h * arm_cos_f32(c-angle))/(L2 * arm_sin_f32(c-a)) + (g * arm_cos_f32(d - angle))/(L2 * arm_sin_f32(d - b)));

}


/* 机器人底盘控制核心任务 */
void ChassisTask(void)
{
    // 获取新的控制信息
	
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
	DEBUG_JOINT = *joint_lf;
	DEBUG_WHEEL = *wheel_l;

	dt = DWT_GetDeltaT(dwt_cnt);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DMMotorStop(joint_lf);
        DMMotorStop(joint_rf);
        DMMotorStop(joint_lb);
        DMMotorStop(joint_rb);
		DJIMotorStop(wheel_l);
		DJIMotorStop(wheel_r);
    }
    else
    { // 正常工作
        DMMotorEnable(joint_lf);
        DMMotorEnable(joint_rf);
        DMMotorEnable(joint_lb);
        DMMotorEnable(joint_rb);
		DJIMotorEnable(wheel_l);
		DJIMotorEnable(wheel_r);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
	//	chassis_cmd_recv.wz = ChassisFollowControl();
		break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 4000;
        break;
    default:
        break;
    }

	CalRealtiveAngle();
	SolveFiveRod();
	CalLQR();
	CalWheelTorque();
	CalVirtualForce();
	CalVirtualTorque();
	SolveVMC();
	SolveRealWrench();

    // 推送反馈消息
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}
