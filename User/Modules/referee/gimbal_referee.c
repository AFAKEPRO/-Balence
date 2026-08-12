#include "gimbal_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "daemon.h"

#define GIMBAL_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

USARTInstance *gimbal_referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *gimbal_referee_daemon;		  // 裁判系统守护进程
static gimbal_referee_info_t gimbal_referee_info;            // 接收到的裁判系统数据


static void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&gimbal_referee_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度(byte),用于CR16校验
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				// 2个8位拼成16位int
				gimbal_referee_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				switch (gimbal_referee_info.CmdID)
				{
				case ID_remote_data:
					memcpy(&gimbal_referee_info.remote_data, (buff + DATA_Offset), LEN_remote_data);

				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + gimbal_referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + gimbal_referee_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

static void GimbalRefereeRxCallback(void)
{
	DaemonReload(gimbal_referee_daemon);
	gimbal_referee_info.remote_data.online_state = 1;
	JudgeReadData(gimbal_referee_usart_instance->recv_buff);
}

// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(gimbal_referee_usart_instance);
	gimbal_referee_info.remote_data.online_state = 0;
}

gimbal_referee_info_t *GimbalRefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = GimbalRefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = GIMBAL_RX_BUFFER_SIZE; // mx 255(u8)
	gimbal_referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = gimbal_referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	gimbal_referee_daemon = DaemonRegister(&daemon_conf);

	return &gimbal_referee_info;
}

