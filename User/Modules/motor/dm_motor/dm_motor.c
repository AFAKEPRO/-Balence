#include "dm_motor.h"
//#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"


static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
	
	measure->id = rxbuff[0] & 0x0F;
	measure->state = (DMMotor_Error_e)((rxbuff[0] >> 4 ) & 0x0F);
	
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
//		DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor_ptr);
//    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor_ptr);
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
	
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    // 后续增加电机前馈控制器(速度和电流)

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);
	
//	DMMotorSetMode(DM_CMD_RESET_MODE, motor);
//	 DWT_Delay(0.1);
//    DMMotorCaliEncoder(motor);
//	 DWT_Delay(0.1);
    DMMotorEnable(motor);

    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dji_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
	if(motor->measure.state != DM_ENABLE)
	{
		DMMotorSetMode(DM_CMD_RESET_MODE, motor);
		DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
		DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
	}
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
	DMMotorSetMode(DM_CMD_RESET_MODE, motor);
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void DMMotorControl(void)
{
	float pid_measure, pid_ref;             // 电机PID测量值和设定值
	DM_Motor_Measure_s *measure;
	DMMotorInstance *motor;
	Motor_Control_Setting_s *motor_setting;
	Motor_Controller_s *motor_controller;   // 电机控制器
	DMMotor_Send_s motor_send_mailbox;
	
	for(size_t i=0; i<idx; i++)
	{
		motor = dm_motor_instance[i];
		motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转
		
		// pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->position; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
			if(motor_setting->angle_rad_format_flag == FORMAT_ENABLE)
				pid_ref = PIDCalculate(&motor_controller->angle_PID, RadFormat(pid_measure - pid_ref), 0);
			else
				pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->velocity;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->torque, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
//        set = (int16_t)pid_ref;
		motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
		motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
		motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
		motor_send_mailbox.Kp = 0;
		motor_send_mailbox.Kd = 0;

		if(motor->stop_flag == MOTOR_STOP)
			motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

		motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
		motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
		motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
		motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
		motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
		motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
		motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
		motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

		CANTransmit(motor->motor_can_instace, 1);	
	}
	
}


//@Todo: 目前只实现了力控，更多位控PID等请自行添加
//void DMMotorTask(void const *argument)
//{
//    float  pid_ref, set;
//    DMMotorInstance *motor = (DMMotorInstance *)argument;
//   //DM_Motor_Measure_s *measure = &motor->measure;
//    Motor_Control_Setting_s *setting = &motor->motor_settings;
//    //CANInstance *motor_can = motor->motor_can_instace;
//    //uint16_t tmp;
//    DMMotor_Send_s motor_send_mailbox;
//    while (1)
//    {
//        pid_ref = motor->pid_ref;
//        
//        set = pid_ref;
//        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
//            set *= -1;
//       
//        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
//        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
//        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
//        motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
//        motor_send_mailbox.Kp = 0;
//        motor_send_mailbox.Kd = 0;

//        if(motor->stop_flag == MOTOR_STOP)
//            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

//        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
//        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
//        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
//        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
//        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
//        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
//        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
//        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

//        CANTransmit(motor->motor_can_instace, 1);

//        osDelay(2);
//    }
//}

