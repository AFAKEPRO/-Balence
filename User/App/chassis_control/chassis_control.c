
#include "chassis_control.h"
#include "robot_def.h"
#include "message_center.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "controller.h"
#include "math.h"

#define K_array_calc(len,a,b,c)		(a*len*len + b*len + c)
#define LQR(ptr , k0,k1,k2,k3,k4,k5)                 \
(                                                    \
	- (k0)*(ptr)->leg_pitch                            \
	- (k1)*(ptr)->leg_pitch_speed                      \
	- (k2)*(loop_fp32_constrain(((ptr)->x - (ptr)->x_set)  /*cos((ptr)->yaw_feedback->relative_angle )*/,-X_MAX, X_MAX)) \
 	- (k3)* v_fusion/*((ptr)->vx) */                               \
	- (k4)*(ptr)->chassis_pitch                   \
	- (k5)*(ptr)->chassis_pitch_speed                  \
) 

//驱动轮电机
DJIMotorInstance *wheel_l, *wheel_r;
DJIMotorInstance DEBUG_wheel_l, DEBUG_wheel_r;

//关节电机
DMMotorInstance *joint_lf, *joint_rf, *joint_lb, *joint_rb;
DMMotorInstance DEBUG_joint_lf, DEBUG_joint_rf, DEBUG_joint_lb, DEBUG_joint_rb;

PIDInstance leg_pid[2], roll_pid;      					//虚拟腿力pid控制器
PIDInstance delta_ang_pid;          					//虚拟力矩pid控制器
PIDInstance yaw_pid;                					//驱动轮转向yaw补偿

static float leg_len[2], ang[2];
static float leg_len_avg, ang_avg;
static float ang_1[2], ang_2[2], ang_3[2], ang_4[2];    //Five_Rod_solve  规定ang_1为五连杆的前一个角度

static float T[2], Tp[2], T_Wheel[2];
static float Virtual_Force[2], Virtual_Torque[2];       //VMC
static float Torque_f[2], Torque_b[2];
static float Actual_Force[2], Actual_Torque[2];

chassis_control_s state_vector;                			//状态向量

void CalRealtiveAngle()
{
	ang_1[0] = joint_lf->measure.position + JOINT_LF_INIT_ANGLE;
	ang_1[1] = joint_rf->measure.position + JOINT_RF_INIT_ANGLE;
	ang_2[0] = joint_lb->measure.position + JOINT_LB_INIT_ANGLE;
	ang_2[1] = joint_rb->measure.position + JOINT_RB_INIT_ANGLE;
}


float arm_sqrt_32(float in)
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

void SolveFiveRod()
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

void CalLQR()
{	
	float K[12];

	K[0] = K_array_calc(leg_len_avg,  83.492f,-123.701f, -37.039f);
    K[1] = K_array_calc(leg_len_avg,  -7.552f, -19.745f,  -3.523f);
    K[2] = K_array_calc(leg_len_avg,  30.848f, -26.459f, -23.099f);
    K[3] = K_array_calc(leg_len_avg, -22.822f,   9.077f, -21.858f);
    K[4] = K_array_calc(leg_len_avg,  65.444f, -61.777f,  22.881f);
    K[5] = K_array_calc(leg_len_avg,  19.408f, -16.899f,   6.008f);
    K[6] = K_array_calc(leg_len_avg,  54.768f, -67.650f,  41.264f);
    K[7] = K_array_calc(leg_len_avg,   0.676f,  -1.203f,   4.183f);
    K[8] = K_array_calc(leg_len_avg,  55.097f, -64.510f,  28.681f);
    K[9] = K_array_calc(leg_len_avg,  46.604f, -46.241f,  18.080f);
    K[10] = K_array_calc(leg_len_avg, -67.267f,  53.441f,  18.030f);
    K[11] = K_array_calc(leg_len_avg, -17.167f,  13.674f,   1.482f);

	for(int i = 0;i<2;i++)
	{
		T[i] = 0.5*LQR(&state_vector, K[0], K[1], K[2], K[3], K[4], K[5]);
		Tp[i] = 0.5*LQR(&state_vector, K[6], K[7], K[8], K[9], K[10], K[11]);
	}
}

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

void CalWheelTorque()
{
	T_Wheel[0] = T[0] + yaw_pid.Output;
	T_Wheel[1] = T[1] + yaw_pid.Output;
}

void CalVirtualForce()
{
	Virtual_Force[0] = leg_pid[0].Output + roll_pid.Output + CalGravityFF(leg_len[0]);
	Virtual_Force[1] = leg_pid[1].Output - roll_pid.Output; + CalGravityFF(leg_len[1]);
}

void CalVirtualTorque()
{
	Virtual_Torque[0] = Tp[0] + delta_ang_pid.Output;
	Virtual_Torque[0] = Tp[0] - delta_ang_pid.Output;
}

void SolveVMC()
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

void SolveRealWrench()
{	
	float a = ang_1[0], b = ang_2[0], c = ang_3[0], d = ang_4[0], angle = ang[0];
	float g = joint_lb->measure.torque, h = joint_lf->measure.torque;

	Actual_Torque[0] = (-leg_len[0] * (h * arm_sin_f32(c-angle))/(L2 * arm_sin_f32(c-a)) + (g * arm_sin_f32(d - angle))/(L2 * arm_sin_f32(d - b)));
	Actual_Force[0]  = -((h * arm_cos_f32(c-angle))/(L2 * arm_sin_f32(c-a)) + (g * arm_cos_f32(d - angle))/(L2 * arm_sin_f32(d - b)));

	a = ang_1[1], b = ang_2[1], c = ang_3[1], d = ang_4[1], angle = ang[1];
	g = joint_rb->measure.torque, h = joint_rf->measure.torque;

	Actual_Torque[1] = (-leg_len[0] * (h * arm_sin_f32(c-angle))/(L2 * arm_sin_f32(c-a)) + (g * arm_sin_f32(d - angle))/(L2 * arm_sin_f32(d - b)));
	Actual_Force[1]  = -((h * arm_cos_f32(c-angle))/(L2 * arm_sin_f32(c-a)) + (g * arm_cos_f32(d - angle))/(L2 * arm_sin_f32(d - b)));

}