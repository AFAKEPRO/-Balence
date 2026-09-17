#include "remote_control.h"
#include "string.h"
#include "bsp_can.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include "referee_task.h"
#include "gimbal_referee.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小

/* 2026-09-12：遥控改由 A板 经 CAN 转发（见 .h 注释），本板在 hcan2 上收三帧。
   A板 侧组帧见 0standard/user/APP/remote_control/remote_control.c 的 RC_ctrl_push_to_chassis_can() */
#define RC_CAN_TAIL_ID 0x162u // 尾部帧: data[0..1]=18B[16..17], data[2]=来源, data[3]=跳跃, data[4]=发弹

// 遥控器数据
RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
gimbal_referee_info_t *rc_referee_recv;
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

static DaemonInstance *rc_daemon_instance;
// 拼帧缓存: 0x160 填 [0..7] → 0x161 填 [8..15] → 0x162 补尾部并触发解析
static uint8_t rc_canbuf[REMOTE_CONTROL_FRAME_SIZE];
static uint8_t rc_can_seen = 0;  // 0=帧头未到, 1=已收0x160, 3=已收0x160+0x161

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 注册到 bsp_can 的回调(0x160/0x161/0x162 共用一个,靠 rx_id 分派)
 *
 * @note 本回调在 CAN 中断上下文执行(bsp_can 的 FIFOx 回调直接调用),保持短小。
 *       三帧的到达顺序由 CAN 外设 FIFO 保证:三个实例的 tx_id 都填 0(偶数) →
 *       bsp_can 按 tx_id 奇偶分配 FIFO,三帧同进 FIFO1,故顺序与 A板 发送顺序一致。
 */
static void RCCANRxCallback(CANInstance *ins)
{
    if (ins->rx_len != 8u) // A板 Send_remote 固定 DLC=8,长度不符直接丢,避免拼进残留字节
        return;

    switch (ins->rx_id)
    {
    case 0x160u: // 18B 的 byte[0..7]
        memcpy(&rc_canbuf[0], ins->rx_buff, 8);
        rc_can_seen = 1u;
        break;

    case 0x161u: // 18B 的 byte[8..15]
        if (rc_can_seen != 1u)
            break; // 乱序或丢帧,本帧丢弃(下一个 0x160 会重置状态机)
        memcpy(&rc_canbuf[8], ins->rx_buff, 8);
        rc_can_seen = 3u;
        break;

    case RC_CAN_TAIL_ID:
        if (rc_can_seen != 3u)
            break;
        rc_canbuf[16] = ins->rx_buff[0];
        rc_canbuf[17] = ins->rx_buff[1];
        /* 扩展字段必须在 sbus_to_rc 之前写进 TEMP:该函数末尾有 TEMP→LAST 的整结构
           memcpy,这样 LAST 也能拿到上一次的值,switch_jump 才做得了上升沿判断 */
        rc_ctrl[TEMP].rc.switch_jump = (uint8_t)(ins->rx_buff[3] & 0x03u);
        rc_ctrl[TEMP].rc.switch_fire = (uint8_t)(ins->rx_buff[4] & 0x03u);
        sbus_to_rc(rc_canbuf);   // 协议解析(18B 布局与接收机直连同构,零改动)
        rc_ctrl->rc_state = 1;   // 遥控正常
        DaemonReload(rc_daemon_instance);
        rc_can_seen = 0u;
        break;

    default:
        break;
    }
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
	if(rc_referee_recv->remote_data.online_state == 1)
	{
		uint8_t temp[sizeof(RC_ctrl_t)] = {0};
		memcpy(&temp[6], &rc_referee_recv->remote_data, 10);
		/* switch_jump/switch_fire 不在 18B 里,sbus_to_rc 不会写它们,裁判兜底时必须显式归 DOWN,
		   否则会残留上一帧 SBUS 的值(掉线瞬间若 SA 停在跳跃位,就会一直请求跳跃) */
		rc_ctrl[TEMP].rc.switch_jump = RC_SW_DOWN;
		rc_ctrl[TEMP].rc.switch_fire = RC_SW_DOWN;
		sbus_to_rc((const uint8_t*)temp);
		rc_ctrl->rc_state = 2;
	}
	else
	{
		memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据(含两个扩展字段)
		// 注:接收已移到 CAN,过滤器常开,不需要像串口那样在这里重启接收
		rc_ctrl->rc_state = 0;
		LOGWARNING("[rc] remote control lost");
	}
}

RC_ctrl_t *RemoteControlInit(void)
{
    /* 三个 CAN 实例共用同一个回调,靠 rx_id 分派。
       tx_id 三个都必须填 0(偶数):bsp_can 按 tx_id 的奇偶决定进 FIFO0 还是 FIFO1,
       三帧同进 FIFO1 才能保证 0x160→0x161→0x162 的到达顺序。 */
    CAN_Init_Config_s can_conf = {
        .can_handle = &hcan2,
        .tx_id = 0, // 只收不发;填 0 是为了让三帧落在同一个 FIFO(见上)
        .rx_id = 0x160,
        .can_module_callback = RCCANRxCallback,
        .id = NULL,
    };
    CANRegister(&can_conf); // 0x160: 18B 的 byte[0..7]

    can_conf.rx_id = 0x161;
    CANRegister(&can_conf); // 0x161: 18B 的 byte[8..15]

    can_conf.rx_id = RC_CAN_TAIL_ID;
    CANRegister(&can_conf); // 0x162: 尾部 + 跳跃/发弹扩展字段

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);
	rc_referee_recv = GimbalRefereeTaskInit(&huart1);
    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}
