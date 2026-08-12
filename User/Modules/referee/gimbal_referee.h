#ifndef __GIMBAL_REFEREE_H
#define __GIMBAL_REFEREE_H

#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"

typedef struct
{
	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	
	remote_control_t remote_data;
	
	uint8_t init_flag;
	
} gimbal_referee_info_t;


gimbal_referee_info_t *GimbalRefereeInit(UART_HandleTypeDef *referee_usart_handle);

#endif
