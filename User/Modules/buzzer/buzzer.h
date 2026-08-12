#ifndef __BUZZER_H
#define __BUZZER_H

#include "bsp_pwm.h"
#define BUZZER_DEVICE_CNT 5

//需要调整，A板蜂鸣器是2700Hz，C板是4000Hz
#define  DoFreq  523
#define  ReFreq  587
#define  MiFreq  659
#define  FaFreq  698
#define  SoFreq  784
#define  LaFreq  880
#define  SiFreq  988

//音高
typedef enum
{
    OCTAVE_1 = 0,
    OCTAVE_2,
    OCTAVE_3,
    OCTAVE_4,
    OCTAVE_5,
    OCTAVE_6,
    OCTAVE_7,
    OCTAVE_8,
}octave_e;

//警报等级
typedef enum
{
    ALARM_LEVEL_HIGH = 0,
    ALARM_LEVEL_ABOVE_MEDIUM,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_BELOW_MEDIUM,
    ALARM_LEVEL_LOW,
}AlarmLevel_e;

//蜂鸣器当前状态
typedef enum
{
    ALARM_OFF = 0,
    ALARM_ON,
}AlarmState_e;

//蜂鸣器配置结构体
typedef struct
{
    AlarmLevel_e alarm_level;
    octave_e octave;
    float loudness;
}Buzzer_config_s;

//蜂鸣器实例结构体
typedef struct
{
	uint8_t state;		//该优先级的蜂鸣器实例是否被注册的标志位
    float loudness;
    octave_e octave;
    AlarmLevel_e alarm_level;
    AlarmState_e alarm_state;
}BuzzzerInstance;


void BuzzerInit(void);
void BuzzerTask(void);
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config);
void AlarmSetStatus(BuzzzerInstance *buzzer,AlarmState_e state);
void AlarmSetOctave(BuzzzerInstance *buzzer, octave_e octave);
	
#endif
