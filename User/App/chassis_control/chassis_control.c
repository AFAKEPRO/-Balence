
#include "chassis_control.h"
#include "robot_def.h"
#include "message_center.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "controller.h"

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

PIDInstance leg_pid, roll_pid;      //虚拟腿力pid控制器
PIDInstance delta_ang_pid;          //虚拟力矩pid控制器
PIDInstance yaw_pid;                //驱动轮转向yaw补偿

static float leg_len[2], ang[2];
static float ang_1[2], ang_2[2];    //Five_Rod_solve
static float Virtual_Force[2], Virtual_Torque[2]        //VMC
static float K[12];

chassis_control_s state_vector;                //状态向量