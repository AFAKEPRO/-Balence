#include "cmd_send.h"
#include "string.h"
#include "stdlib.h"
#include "can_comm.h"
#include "message_center.h"
#include "robot_def.h"

CANCommInstance* ins;
static Subscriber_t *cmd_send_sub;	// 云台反馈信息订阅者
static Publisher_t *cmd_send_pub;   // 用于发布底盘的数据
Chassis_Ctrl_Cmd_s cmd_send_data; 	// 从云台获取的反馈信息
chassis_data_rev data_send;
Chassis_Upload_Data_s *chassis_can_fetch_data;

void CmdRevInit(void)
{
	CANComm_Init_Config_s cconfig = {
		.can_config = {
			.can_handle=&hcan2,
			.tx_id=0x22,
			.rx_id=0x33},
		.send_data_len = sizeof(chassis_data_rev),
		.recv_data_len = sizeof(Chassis_Upload_Data_s)
	};
	ins = CANCommInit(&cconfig);
	cmd_send_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
	cmd_send_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

void CmdRevTask(void)
{
	chassis_can_fetch_data = (Chassis_Upload_Data_s*)CANCommGet(ins);
	SubGetMessage(cmd_send_sub, &cmd_send_data);
	data_send.vx = cmd_send_data.vx;
	data_send.vy = cmd_send_data.vy;
	data_send.wz = cmd_send_data.wz;
	data_send.off_set = cmd_send_data.offset_angle;
	data_send.chassis_mode = cmd_send_data.chassis_mode;
	data_send.friction_mode = cmd_send_data.friction_mode;
	data_send.gimbal_mode = cmd_send_data.gimbal_mode;
	data_send.loader_mode = cmd_send_data.loader_mode;
	data_send.vision_state = cmd_send_data.vision_state;
	
	CANCommSend(ins, (uint8_t*)&data_send);
	PubPushMessage(cmd_send_pub, chassis_can_fetch_data);
}
	
