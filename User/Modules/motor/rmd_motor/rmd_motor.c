#include "rmd_motor.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static RMDMotorInstance *rmd_motor_instance[RMD_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

static void RMDMotorSetAutoReport(void)
{
	RMDMotorInstance *motor;
	for(size_t i = 0; i<idx; i++)
	{
		motor = rmd_motor_instance[i];
		
		motor->motor_can_instance->tx_buff[0] = 0xB6;
		motor->motor_can_instance->tx_buff[1] = 0x92;	//读取多圈编码器原始位置
		motor->motor_can_instance->tx_buff[2] = 0x01;	//回复使能
		motor->motor_can_instance->tx_buff[3] = 0x00;	//10ms回复一次
		motor->motor_can_instance->tx_buff[4] = 0x00;
		motor->motor_can_instance->tx_buff[5] = 0x00;
		motor->motor_can_instance->tx_buff[6] = 0x00;
		motor->motor_can_instance->tx_buff[7] = 0x00;
		
		CANTransmit(motor->motor_can_instance, 1);
	}
}

void RMDMotorZeroCali(void)
{
	RMDMotorInstance *motor;
	for(size_t i = 0; i<idx; i++)
	{
		motor = rmd_motor_instance[i];
		
		motor->motor_can_instance->tx_buff[0] = 0x64;
		motor->motor_can_instance->tx_buff[1] = 0x00;	//读取多圈编码器原始位置
		motor->motor_can_instance->tx_buff[2] = 0x00;	//回复使能
		motor->motor_can_instance->tx_buff[3] = 0x00;	//10ms回复一次
		motor->motor_can_instance->tx_buff[4] = 0x00;
		motor->motor_can_instance->tx_buff[5] = 0x00;
		motor->motor_can_instance->tx_buff[6] = 0x00;
		motor->motor_can_instance->tx_buff[7] = 0x00;
		CANTransmit(motor->motor_can_instance, 1);
		DWT_Delay(1);
//		motor->motor_can_instance->tx_buff[0] = 0x76;
//		CANTransmit(motor->motor_can_instance, 1);
	}
}

static void DecodeRMDMotor(CANInstance *_instance)
{
	uint8_t *rxbuff = _instance->rx_buff;
    RMDMotorInstance *motor = (RMDMotorInstance *)_instance->id;
    RMD_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销
	motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
	DaemonReload(motor->daemon);
	switch(rxbuff[0])
	{
		case 0x92:
			measure->total_angle = ((float)(rxbuff[4]|rxbuff[5]<<8|rxbuff[6]<<16|rxbuff[7]<<24))/100;
			break;
		default:
			break;
	}

}

uint8_t lost;
static void RMDMotorLostCallback(void *id)
{
	lost++;
}

RMDMotorInstance *RMDMotorInit(Motor_Init_Config_s *config)
{
	RMDMotorInstance *instance = (RMDMotorInstance *)malloc(sizeof(RMDMotorInstance));
	memset(instance, 0, sizeof(RMDMotorInstance));

	// motor basic setting 电机基本设置
	instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
	instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

	// motor controller init 电机控制器初始化
	PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
	PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
	PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
	instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
	instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
	instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
	instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
	// 后续增加电机前馈控制器(速度和电流)

	// 注册电机到CAN总线
	config->can_init_config.can_module_callback = DecodeRMDMotor; // set callback
	config->can_init_config.id = instance;                        // set id,eq to address(it is identity)
	instance->motor_can_instance = CANRegister(&config->can_init_config);

	// 注册守护线程
	Daemon_Init_Config_s daemon_config = {
		.callback = RMDMotorLostCallback,
		.owner_id = instance,
		.reload_count = 2, // 20ms未收到数据则丢失
	};
	instance->daemon = DaemonRegister(&daemon_config);
//	RMDMotorZeroCali();


    RMDMotorEnable(instance);
    rmd_motor_instance[idx++] = instance;
	RMDMotorSetAutoReport();
	return instance;
}

float debug_set;

void RMDMotorControl(void)
{
	RMDMotorInstance *motor;
	Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
	RMD_Motor_Measure_s *measure;           // 电机测量值
	float pid_measure, pid_ref;             // 电机PID测量值和设定值
	
	for(size_t i = 0; i<idx; i++)
	{
		motor = rmd_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
		int16_t set;        // 电机控制CAN发送设定值
		
		if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
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
                pid_measure = measure->speed;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->iq, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        set = (int16_t)pid_ref;
		if (motor->stop_flag == MOTOR_STOP)
			set = 0;
		debug_set = set;
		motor->motor_can_instance->tx_buff[0] = 0xA1;
		motor->motor_can_instance->tx_buff[1] = 0x00;
		motor->motor_can_instance->tx_buff[2] = 0x00;
		motor->motor_can_instance->tx_buff[3] = 0x00;
		motor->motor_can_instance->tx_buff[4] = (uint8_t)set;
		motor->motor_can_instance->tx_buff[5] = (uint8_t)(set>>8);
		motor->motor_can_instance->tx_buff[6] = 0x00;
		motor->motor_can_instance->tx_buff[7] = 0x00;
		
		CANTransmit(motor->motor_can_instance, 1);
		
	}
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void RMDMotorChangeFeed(RMDMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dji_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

void RMDMotorStop(RMDMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void RMDMotorEnable(RMDMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void RMDMotorOuterLoop(RMDMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

// 设置参考值
void RMDMotorSetRef(RMDMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}
