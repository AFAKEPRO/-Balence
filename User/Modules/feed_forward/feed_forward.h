#ifndef __FEED_FORWARD_H
#define __FEED_FORWARD_H

#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"

#define FF_CNT 2

typedef struct
{
	float K1;
	float K2;
	float max_out;
	float RC;
	uint32_t DWT_CNT;
	float dt;
	
	float *set_ptr;
	float set;
	float previous_set;
	float pre_previous_set;

	float temp_out;		//未滤波的out
	float last_out;
	float final_out;	//经过滤波的out
	float output;
} FeedForwardInstance;

typedef struct
{
	float K1;
	float K2;
	float max_out;
	float RC;
	float *set_ptr;
} FeedForward_Init_Config_s;

/**
 * @brief 初始化前馈实例
 * @param config  	前馈配置指针
 * @return float  	前馈实例指针
 */
FeedForwardInstance *FeedForwardInit(FeedForward_Init_Config_s *config);

void FeedForwardSetRef(FeedForwardInstance *FF, float *ref);

void FeedForwardControl(void);

#endif
