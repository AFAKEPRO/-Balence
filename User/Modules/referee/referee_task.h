#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"
#include "robot_def.h"
#include "gimbal_referee.h"

/**
 * @brief 初始化图传链路
 *
 */
gimbal_referee_info_t *GimbalRefereeTaskInit(UART_HandleTypeDef *referee_usart_handle);

#if (defined ONE_BOAD) || (defined CHASS_BOAD)
/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */
void MyUIInit(void);

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask(void);
#endif

#endif // REFEREE_H
