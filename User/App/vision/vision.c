#include "vision.h"
#include "INS_task.h"
#include  "usbd_cdc_if.h"
#include "usb_device.h"
#include "message_center.h"
#include "robot_def.h"
#include "user_lib.h"

#define    COM6_PACKAGE_HEADER       0xFF
#define    COM6_PACKAGE_TAIL_BYTE       0x0D
#define PC_DATA_LEN		16

static Publisher_t *vision_pub;                   // 自瞄应用消息发布者(自瞄反馈给cmd)
Vision_Upload_Data_s vision_feedback_data; // 回传给cmd的云台状态信息

xPC_Data Data_PC;
uint8_t message[PC_DATA_LEN];
uint8_t data_from_pc[PC_DATA_LEN];

void vision_sendmessage(aim_mod_e mode,float roll, float pitch, float yaw) 
{
	// 填充帧头，长度，cmd1和speed
	message[0] = COM6_PACKAGE_HEADER;
	message[1] = mode;
	memcpy(&message[2], &roll, 4);
	memcpy(&message[6], &pitch, 4);
	memcpy(&message[10], &yaw, 4);
	//发送数据给PC
	message[14]= 0;
	message[15]= COM6_PACKAGE_TAIL_BYTE;
	CDC_Transmit_FS(message,PC_DATA_LEN);
}

void PC_read_data(uint8_t* data)
{
		if (data == NULL)
	{		
		return;		
	}

	for(int i=0; i<64; i++,data++)
	{
		if(*data == COM6_PACKAGE_HEADER)
		{
			if(*(data + PC_DATA_LEN - 1) == COM6_PACKAGE_TAIL_BYTE)
			{
				memcpy(&Data_PC, data, PC_DATA_LEN);
				break;
			}
			else
			{
				break;
			}
		}
		
	}
}

void VisionInit(void)
{
	MX_USB_DEVICE_Init();
	vision_pub = PubRegister("vision_feed", sizeof(Vision_Upload_Data_s));
}

void VisionTask(float roll, float pitch, float yaw)
{
	vision_sendmessage(AIM_RED, roll, -pitch, yaw);
	vision_feedback_data.aim_pitch = DegToRad(Data_PC.pitch);
	vision_feedback_data.aim_yaw = DegToRad(Data_PC.yaw);
	vision_feedback_data.fire_advice = Data_PC.fire_advice;
	if(Data_PC.pitch ==0 && Data_PC.yaw ==0)
		vision_feedback_data.vision_state = NO_TARGET;
	else
		vision_feedback_data.vision_state = TARGET_LOCK;
	PubPushMessage(vision_pub, (void *)&vision_feedback_data);
}
