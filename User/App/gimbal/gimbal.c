//App
#include "gimbal.h"
//Modules
#include "daemon.h"
#include "controller.h" //PIDInstance / PIDInit / PIDCalculate
#include "user_lib.h"   //RadFormat
//Bsp
#include "bsp_can.h"

/* -------------------- 0x207 云台 yaw 电机反馈 -------------------- */
#define GIMBAL_YAW_CAN_RX_ID 0x207u      // GM6020 反馈 ID = 0x204 + 电机ID(3)（dji_motor.c:19）
#define GIMBAL_YAW_ECD_PER_ROUND 8192.0f // GM6020 单圈编码器分辨率（同 dji_motor.h 的 ECD_ANGLE_COEF_DJI = 360/8192）
#define GIMBAL_YAW_ECD_HALF_RANGE 4096   // 跨圈判定阈值：相邻两帧转过半圈以上才算跨圈（同 DecodeDJIMotor）
#define GIMBAL_YAW_2PI 6.28318530718f    // 2π，用于 ecd → rad。不用 DegToRad 是为了省掉"弧度→角度→弧度"的往返
#define GIMBAL_YAW_DAEMON_RELOAD 5       // 离线门限，单位 10ms/拍（DaemonTask 100Hz）→ 50ms 没帧算离线
//后面还能再加

/* -------------------- 跟随云台 yaw 位置环 -------------------- */
#define GIMBAL_YAW_FOLLOW_KP 2.5f        // 增益是初值，待台架整定
#define GIMBAL_YAW_FOLLOW_MAX_SPEED 2.0f // 输出限幅(rad/s)。手动转向满打约 0.66rad/s（robot_cmd.c:46），
                                         // 跟随留 3 倍余量；给太大时云台偏一点底盘就疯转

//数据
static GimbalData_s gimbal_data;           // 云台下行数据总集
static DaemonInstance *gimbal_yaw_daemon;  // 一帧一个守护进程（各帧频率不同，判离线门限不该互相牵连）
static uint8_t gimbal_init_flag = 0;

//跟随云台 yaw 位置环（外环，输出 yaw 速度指令给底盘 yaw 速度环）
static PIDInstance gimbal_yaw_follow_pid;
static float gimbal_yaw_follow_target;     // 锁存的目标相对角：进跟随时取当前值，之后保持不变
static uint8_t gimbal_yaw_follow_last = 0; // 上一拍是否在跟随，用来取上升沿

/**
 * @brief 0x207 接收回调（注册进 bsp_can，由 CAN2 中断调用）
 *
 * @note 中断上下文，保持短小：只解析 + 喂狗，不做任何控制、绝不打印。
 * @note 反馈帧字节序是 DJI 标准、两端一致：
 *       [0..1]=ecd, [2..3]=转速, [4..5]=电流, [6]=温度，全部大端
 */
static void GimbalYawCANRxCallback(CANInstance *ins)
{
    uint8_t *rxbuff = ins->rx_buff;

    gimbal_data.yaw.last_ecd = gimbal_data.yaw.ecd;
    gimbal_data.yaw.ecd = ((uint16_t)rxbuff[0] << 8) | rxbuff[1];
    gimbal_data.yaw.speed_rpm = (int16_t)((rxbuff[2] << 8) | rxbuff[3]);

    /* 多圈累计：0x207 是 ~1kHz，相邻两帧之间机械上不可能转过半圈，所以差值跨过 ±4096 就只可能是编码器过零。逻辑同 dji_motor.c:145-148 */
    if (gimbal_data.yaw.ecd - gimbal_data.yaw.last_ecd > GIMBAL_YAW_ECD_HALF_RANGE)
        gimbal_data.yaw.total_round--;
    else if (gimbal_data.yaw.ecd - gimbal_data.yaw.last_ecd < -GIMBAL_YAW_ECD_HALF_RANGE)
        gimbal_data.yaw.total_round++;

    /* 弧度换算。原点 = 云台电机自己的 ecd 零点，不是"上电位置"，所以首帧不需要特殊处理：绝对值本来就不需要有意义 —— 跟随环用锁存目标，只看 relative_angle 的变化量 */
    gimbal_data.yaw.relative_angle = GIMBAL_YAW_2PI *
        ((float)gimbal_data.yaw.total_round + (float)gimbal_data.yaw.ecd / GIMBAL_YAW_ECD_PER_ROUND);

    gimbal_data.yaw.frame_cnt++;

    /* 只喂狗，不在中断里碰 DWT：DWT_SysTimeUpdate() 会改 CYCCNT_LAST/SysTime 这些共享静态量，INS 任务也在 1kHz 调它，插进中断最坏会让时间轴整体错一次翻转 */
    DaemonReload(gimbal_yaw_daemon);
}

void GimbalInit(void)
{
    /* ---- 离线守护 ---- */
    /* 必须建在 CANRegister 之前：滤波器一注册就立即生效，而 CAN 回调里会调
       DaemonReload(gimbal_yaw_daemon)。若那时它还是 NULL，窗口内进来一帧 0x207
       就是一次空指针解引用 → HardFault（上电偶发死机）。顺序反过来写没有好处 */
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = GIMBAL_YAW_DAEMON_RELOAD,
        /* 离线时什么都不做（daemon.c:45 对 callback 有判空，传 NULL 是合法的）：
           数据留在原处，谁要用谁先问 GimbalYawIsOnline()。
           如果在这里把 relative_angle 清 0，一旦有人绕过 online 判断，
           读到的是"角突然跳到 0"，比留着旧值更危险 */
        .callback = NULL,
        .owner_id = NULL, // 只有一路云台 yaw 数据，不需要 owner_id
    };
    gimbal_yaw_daemon = DaemonRegister(&daemon_conf);

    /* ---- 0x207 云台 yaw 电机反馈 ---- */
    CAN_Init_Config_s can_conf = {
        .can_handle = &hcan2,
        .tx_id = 0, // 只收不发（本实例永不调用 CANTransmit）。填 0 是为了让滤波器落 FIFO1
                    // （bsp_can 按 tx_id 奇偶分配 FIFO），与遥控三帧保持一致
        .rx_id = GIMBAL_YAW_CAN_RX_ID,
        .can_module_callback = GimbalYawCANRxCallback,
        .id = NULL,
    };
    CANRegister(&can_conf);

    /* ---- 跟随云台 yaw 位置环 ---- */
    PID_Init_Config_s follow_pid_config = {
        .Kp = GIMBAL_YAW_FOLLOW_KP,
        .Ki = 0, // 先不上积分：云台打到底被限位挡住时外环会一直攒，一松手就甩
        .Kd = 0, // 内环 yaw_pid 已经有 Kd=4 作用在陀螺上，外环再来一次是双重阻尼，先留 0
        .Output_LPF_RC = 0.1f,
        .IntegralLimit = 0,
        .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter),
        .MaxOut = GIMBAL_YAW_FOLLOW_MAX_SPEED,
    };
    PIDInit(&gimbal_yaw_follow_pid, &follow_pid_config);

    /* ---- 以后每一路云台下行帧加在这里：一条 CANRegister + 一条 DaemonRegister ---- */

    gimbal_init_flag = 1;
}

GimbalData_s *GimbalGetData(void)
{
    return &gimbal_data;
}

uint8_t GimbalYawIsOnline(void)
{
    if (gimbal_init_flag)
        return DaemonIsOnline(gimbal_yaw_daemon);
    return 0;
}

/**
 * @brief 跟随云台 yaw 的位置环
 *
 * 底盘跟随的本质是让"云台相对底盘的 yaw 角"保持不变，所以目标是锁存出来的，不是写死的常数
 * —— relative_angle 的原点是云台电机自己的 ecd 零点，跟底盘朝向没有固定关系。
 *
 * 这里只算外环，输出的是 yaw 速度指令；内环是 chassis_control.c 的 yaw_pid(陀螺速度环)。
 * 两级都在 ChassisTask 里按同一频率跑，级联才干净 —— 所以调用点在那儿，不在 CAN 回调里。
 */
float GimbalYawFollowLoop(uint8_t follow_on)
{
    static float err;

    /* 不跟随 / 云台离线：不输出，同时复位锁存，下次跟随重新锁存目标。
       离线时返回 0 是安全侧：yaw 速度指令为 0 → 底盘保持当前朝向；
       若硬算，就是拿着一个冻结的角度继续算误差，底盘会匀速一直转下去 */
    if (!follow_on || !GimbalYawIsOnline())
    {
        gimbal_yaw_follow_last = 0;
        return 0;
    }

    /* 上升沿：锁存目标 */
    if (!gimbal_yaw_follow_last)
    {
        gimbal_yaw_follow_target = gimbal_data.yaw.relative_angle;
        gimbal_yaw_follow_last = 1;
    }

    /* 折叠到 ±π：relative_angle 是多圈累计不折叠的，云台转过 360° 后裸误差会长到 2π，
       不折叠的话底盘会跟着转一整圈去追一个物理上早就到位的方向。
       用 RadFormat() 函数而不是 rad_format 宏：宏展开要求调用点能看到 PI，函数不用 */
    err = RadFormat(gimbal_data.yaw.relative_angle - gimbal_yaw_follow_target);

    /* measure 传 -err、ref 传 0，等价于让 Err = err（PIDCalculate 内部算的是 ref - measure）。
       这行的符号还没上机验证：台架上若底盘朝远离云台的方向追，把下面的减号改成加号 */
    PIDCalculate(&gimbal_yaw_follow_pid, -err, 0);

    return gimbal_yaw_follow_pid.Output;
}