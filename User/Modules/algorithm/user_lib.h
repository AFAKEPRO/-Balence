#ifndef USER_LIB_H
#define USER_LIB_H
#include "stdint.h"
#include "stdlib.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif
typedef __packed struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;
typedef __packed struct
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;
//typedef struct
//{
//    float x;  // 估计的状态
//    float P;  // 估计的误差协方差
//    float Q;  // 系统噪声协方差
//    float R;  // 测量噪声协方差
//} KalmanFilter;// 卡尔曼滤波结构体

//快速开方
extern float invSqrt(float num);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);
//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//一阶滤波计算
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
//卡尔曼滤波初始化
//void Kalman_Filter_Init(KalmanFilter *kf, float initial_x, float initial_P, float initial_Q, float initial_R);
////卡尔曼滤波计算
//void Kalman_Filter_Calc(KalmanFilter *kf, float measurement);
//绝对限制
extern void abs_limit(float *num, float Limit);
//判断符号位
extern float sign(float value);
//浮点死区
extern float fp32_deadline(float Value, float minValue, float maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern float fp32_constrain(float Value, float minValue, float maxValue);
//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
extern float loop_fp32_constrain(float Input, float minValue, float maxValue);
//弧度限幅 -PI ~ PI
float RadFormat(float rad);
//角度 °限幅 180 ~ -180
extern float theta_format(float Ang);
//最小二乘法求微分
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
//最小二乘法初始化
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
//限幅函数
float float_constrain(float Value, float minValue, float maxValue);

int float_rounding(float raw);

float RadToDeg(float rad);

float DegToRad(float deg);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
