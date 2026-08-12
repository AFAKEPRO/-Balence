#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "buzzer.h"
#include "stdlib.h"
#include "string.h"


PWMInstance *buzzer = NULL;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {NULL};

/**
 * @brief 	蜂鸣器初始化
 * @note  	A板蜂鸣器，需要pwm驱动，额定频率为2700Hz,TIM12_CH1(PH6)
 *			C板蜂鸣器，需要pwm驱动，额定频率为4000Hz,TIM4_CH3
 */
void BuzzerInit() {
    PWM_Init_Config_s buzzer_config = {
        .htim      = &htim4,
        .channel   = TIM_CHANNEL_3,
        .dutyratio = 1,
        .period    = 1,
    };

    buzzer = PWMRegister(&buzzer_config);
    PWMStart(buzzer);
}

/**
 * @brief 注册一个蜂鸣器实例
 *
 * @param config 初始化配置
 * @return BuzzzerInstance*
 */
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config) {

    if (config->alarm_level > BUZZER_DEVICE_CNT) // 瓒呰繃鏈?澶у疄渚嬫暟,鑰冭檻澧炲姞鎴栨煡鐪嬫槸鍧︽湁鍐呭瓨娉勬紳
        while (1);
    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness    = config->loudness;
    buzzer_temp->octave      = config->octave;
    buzzer_temp->alarm_state = ALARM_OFF;
	buzzer_temp->state = 1;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}
void AlarmSetOctave(BuzzzerInstance *buzzer, octave_e octave) { buzzer->octave = octave; }

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state) { buzzer->alarm_state = state; }

/**
 * @brief 蜂鸣器任务
 *
 */

void BuzzerTask() {
    if (buzzer == NULL)
    {
        return;
    }
    BuzzzerInstance *buzz;
    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i) 
	{
        buzz = buzzer_list[i];
		if (buzz->state != 1) {
            continue;
        }
        if (buzz->alarm_level > ALARM_LEVEL_LOW) {
            continue;
        }

        if (buzz->alarm_state == ALARM_OFF) {
            PWMSetDutyRatio(buzzer, 0);
        }
        
        else {
            PWMSetDutyRatio(buzzer, buzz->loudness);
            switch (buzz->octave) {
            case OCTAVE_1:
                PWMSetPeriod(buzzer, (float)1000 / DoFreq);
                break;
            case OCTAVE_2:
                PWMSetPeriod(buzzer, (float)1000 / ReFreq);
                break;
            case OCTAVE_3:
                PWMSetPeriod(buzzer, (float)1000 / MiFreq);
                break;
            case OCTAVE_4:
                PWMSetPeriod(buzzer, (float)1000 / FaFreq);
                break;
            case OCTAVE_5:
                PWMSetPeriod(buzzer, (float)1000 / SoFreq);
                break;
            case OCTAVE_6:
                PWMSetPeriod(buzzer, (float)1000 / LaFreq);
                break;
            case OCTAVE_7:
                PWMSetPeriod(buzzer, (float)1000 / SiFreq);
                break;
			default:
				break;
            }
            break;
        }
        // PWMStart(buzzer);
    }
}

void test_on(float a,float b)
{
	PWMSetDutyRatio(buzzer, a);
	PWMSetPeriod(buzzer, (float)1000 / SiFreq);
}
