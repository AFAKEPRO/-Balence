#include "feed_forward.h"
#include "math.h"

static uint8_t idx;
static FeedForwardInstance *feed_forward_instance[FF_CNT];

FeedForwardInstance *FeedForwardInit(FeedForward_Init_Config_s *config)
{
	FeedForwardInstance *instance = (FeedForwardInstance *)malloc(sizeof(FeedForwardInstance));
	memset(instance, 0, sizeof(FeedForwardInstance));
	memcpy(instance, config, sizeof(FeedForward_Init_Config_s));
	instance->set_ptr = config->set_ptr;
	DWT_GetDeltaT(&instance->DWT_CNT);
	feed_forward_instance[idx++] = instance;
	return instance;
}

float rc_temp = 0.005f;
static void FeedForwardCacl(FeedForwardInstance *FF)
{
	if(FF->RC + FF->dt == 0)
	{
		return;
	}
	float temp_set = *(FF->set_ptr);
	FF->set = temp_set * FF->dt / (FF->RC + FF->dt) + 
				FF->previous_set * FF->RC / (FF->RC + FF->dt);
	FF->temp_out = FF->K1*(FF->set-FF->previous_set) + FF->K2*(FF->set - 2*FF->previous_set + FF->pre_previous_set);
	
	FF->final_out = FF->temp_out * FF->dt / (rc_temp + FF->dt) + 
					FF->last_out * rc_temp / (rc_temp + FF->dt);

//!!!!!!!!!!!!!!!!!!!	
//	FF->final_out = FF->final_out * FF->dt / (rc_temp + pid->dt) +
//                  FF->last_out * rc_temp / (rc_temp + FF->dt);
//	FF->final_out = FF->temp_out;

	//�޷�
	if(FF->final_out > FF->max_out)
	{
		FF->final_out = FF->max_out;
	}
	else if(FF->final_out < -(FF->max_out))
	{
		FF->final_out = -(FF->max_out);
	}
	//���˶����Ƹı�ʱǰ�����Ϊ0��ֹӰ����Ӧ�ٶ�

	

	
	FF->pre_previous_set = FF->previous_set;
	FF->previous_set = FF->set;	
	FF->last_out = FF->final_out;
	FF->dt = DWT_GetDeltaT(&FF->DWT_CNT);
}

void FeedForwardSetRef(FeedForwardInstance *FF, float *ref)
{
	FF->set_ptr = ref;
}

void FeedForwardControl(void)
{
	for (size_t i = 0; i < idx; ++i)
    {
		FeedForwardCacl(feed_forward_instance[i]);
	}
	
}
