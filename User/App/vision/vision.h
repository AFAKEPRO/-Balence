#ifndef __VISION_TASK_H
#define __VISION_TASK_H

#include "stdint.h"
#include "robot_def.h"

typedef enum
{
	AIM_RED 		 	=			0,   //ºì·½         
	AIM_BLUE 			=			1,   //À¶·½	
	SMALL_RUNE_RED  	=			2,
	SMALL_RUNE_BLUE 	=			3,
	BIG_RUNE_RED 		=			4,
	BIG_RUNE_BLUE 		=			5,

} aim_mod_e;

typedef __packed struct
{
	uint8_t header;
	uint8_t fire_advice;
	float pitch;
	float yaw;
	float distance;
	uint8_t check_bit;
	uint8_t tail_byte;		
  
} xPC_Data;

typedef __packed struct 
{
	uint8_t header;
	Enemy_Color_e mode;
	float roll;
	float pitch;
	float yaw;
	uint8_t check_bit;
	uint8_t tail_byte;
	
} data_to_boardA_t;

void PC_read_data(uint8_t* data);
void VisionInit(void);
void VisionTask(float roll, float pitch, float yaw);

#endif
