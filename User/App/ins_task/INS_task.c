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



#include "main.h"
#include "INS_task.h"
#include "bsp_dwt.h"
#include "pid.h"
#include "QuaternionEKF.h"
#include "bsp_imu_pwm.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "MahonyAHRS.h"
//#include "ist8310.h"
#include "user_lib.h"

#include "bsp_usart1.h"

#include "vision.h"

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定


//extern SPI_HandleTypeDef hspi1;

INS_t INS;
IMU_Param_t IMU_Param;

static pid_type_def imu_temp_pid;
static const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
float dt = 0, t = 0;

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);


void INS_Init(void)
{
    IMU_Param.scale[IMU_X] = 1;
    IMU_Param.scale[IMU_Y] = 1;
    IMU_Param.scale[IMU_Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    // imu heat init
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

    INS.AccelLPF = 0.0085;
	q4_Init(INS.q,INS.Accel[IMU_X], INS.Accel[IMU_Y], INS.Accel[IMU_Z]);
//	ist8310_init();
//	IST8310_INIT(&ist);
}

void INS_task(void)
{
	static uint32_t count = 0;
	const float gravity[3] = {0, 0, 9.81f};
	dt = DWT_GetDeltaT(&INS_DWT_Count);
	t += dt;
	
	// ins update
	if ((count % 1) == 0)
	{
		BMI088_Read(&BMI088);
		
		INS.Accel[IMU_X] = BMI088.Accel[IMU_X];
		INS.Accel[IMU_Y] = BMI088.Accel[IMU_Y];
		INS.Accel[IMU_Z] = BMI088.Accel[IMU_Z];
		INS.Gyro[IMU_X] = BMI088.Gyro[IMU_X];
		INS.Gyro[IMU_Y] = BMI088.Gyro[IMU_Y];
		INS.Gyro[IMU_Z] = BMI088.Gyro[IMU_Z];
		
		// demo function,用于修正安装误差,可以不管,本demo暂时没用
		IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);
		
		// 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
		INS.atanxz = -atan2f(INS.Accel[IMU_X], INS.Accel[IMU_Z]) * 180 / PI;
		INS.atanyz = atan2f(INS.Accel[IMU_Y], INS.Accel[IMU_Z]) * 180 / PI;

		// 核心函数,EKF更新四元数
//		IMU_QuaternionEKF_Update(INS.Gyro[IMU_X], INS.Gyro[IMU_Y], INS.Gyro[IMU_Z], INS.Accel[IMU_X], INS.Accel[IMU_Y], INS.Accel[IMU_Z], dt);

//		memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
//		ist8310_read_mag(mag);
		MahonyAHRSupdateIMU(INS.q,INS.Gyro[IMU_X], INS.Gyro[IMU_Y], INS.Gyro[IMU_Z], INS.Accel[IMU_X], INS.Accel[IMU_Y], INS.Accel[IMU_Z]);
//		MahonyAHRSupdate(INS.q,INS.Gyro[IMU_X], INS.Gyro[IMU_Y], INS.Gyro[IMU_Z], INS.Accel[IMU_X], INS.Accel[IMU_Y], INS.Accel[IMU_Z], mag.x, mag.y, mag.z);
		
		// 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
		BodyFrameToEarthFrame(xb, INS.xn, INS.q);
		BodyFrameToEarthFrame(yb, INS.yn, INS.q);
		BodyFrameToEarthFrame(zb, INS.zn, INS.q);
		
		// 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
		float gravity_b[3];
		EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
		
		for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
		{
			INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);//AccelLPF=0.0085
		}
		BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
		
		// 获取最终数据
//		INS.Yaw = QEKF_INS.Yaw*0.01745329252f;	
//		INS.Pitch = QEKF_INS.Pitch*0.01745329252f;	
//		INS.Roll = QEKF_INS.Roll*0.01745329252f;	
//		INS.YawTotalAngle = QEKF_INS.YawTotalAngle;		
		INS.Pitch=(asin(2*(INS.q[0]*INS.q[2]-INS.q[1]*INS.q[3])));
		INS.Yaw=RadFormat((atan2(2*(INS.q[0]*INS.q[3]+INS.q[1]*INS.q[2]),(1-2*(INS.q[2]*INS.q[2]+INS.q[3]*INS.q[3])))));
		INS.Roll=(atan2(2*(INS.q[0]*INS.q[1]+INS.q[2]*INS.q[3]),(1-2*(INS.q[1]*INS.q[1]+INS.q[2]*INS.q[2]))));
		
		VisionTask(RadToDeg(INS.Roll),-RadToDeg(INS.Pitch),RadToDeg(INS.Yaw));
	}
	
	if ((count % 2) == 0)
	{
		// 500hz
		IMU_Temperature_Ctrl();
	}
	if ((count % 1000) == 0)
	{
		// 200hz
	}
}

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[IMU_X] = c_11 * gyro_temp[IMU_X] +
									c_12 * gyro_temp[IMU_Y] +
									c_13 * gyro_temp[IMU_Z];
    gyro[IMU_Y] = c_21 * gyro_temp[IMU_X] +
									c_22 * gyro_temp[IMU_Y] +
									c_23 * gyro_temp[IMU_Z];
    gyro[IMU_Z] = c_31 * gyro_temp[IMU_X] +
									c_32 * gyro_temp[IMU_Y] +
									c_33 * gyro_temp[IMU_Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[IMU_X] = c_11 * accel_temp[IMU_X] +
									 c_12 * accel_temp[IMU_Y] +
									 c_13 * accel_temp[IMU_Z];
    accel[IMU_Y] = c_21 * accel_temp[IMU_X] +
									 c_22 * accel_temp[IMU_Y] +
									 c_23 * accel_temp[IMU_Z];
    accel[IMU_Z] = c_31 * accel_temp[IMU_X] +
									 c_32 * accel_temp[IMU_Y] +
									 c_33 * accel_temp[IMU_Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

void IMU_Temperature_Ctrl(void)
{
		PID_calc(&imu_temp_pid, BMI088.Temperature, BMI088.TempWhenCali);
		if (imu_temp_pid.out < 0.0f)
		{
				imu_temp_pid.out = 0.0f;
		}
		IMU_temp_PWM((uint16_t)imu_temp_pid.out);
}

attitude_t *GetInsData(void)
{
	return (attitude_t *)&INS.Gyro;
}

/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
//static void imu_temp_control(float temp)
//{
//    uint16_t tempPWM;
//    static uint8_t temp_constant_time = 0;
//    if (first_temperate)
//    {
//        PID_calc(&imu_temp_pid, temp, 45.0f);
//        if (imu_temp_pid.out < 0.0f)
//        {
//            imu_temp_pid.out = 0.0f;
//        }
//        tempPWM = (uint16_t)imu_temp_pid.out;
//        IMU_temp_PWM(tempPWM);
//    }
//    else
//    {
//        //在没有达到设置的温度，一直最大功率加热
//        //in beginning, max power
//        if (temp > 45.0f)
//        {
//            temp_constant_time++;
//            if (temp_constant_time > 200)
//            {
//                //达到设置温度，将积分项设置为一半最大功率，加速收敛
//                //
//                first_temperate = 1;
//                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
//            }
//        }

//        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
//    }
//}



