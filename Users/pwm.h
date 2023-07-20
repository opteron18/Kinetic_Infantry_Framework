#ifndef PWM_H
#define PWM_H
#include "sys.h"

/*#define PWM12_CH1 121
#define PWM12_CH2 122*/

#define PWM2_CH1 121
#define PWM2_CH2 122
#define PWM2_CH3 123

#define BUZZER			TIM3->CCR1
#define IMU_TEMP_R	TIM3->CCR2	//imuÎÂ¶È²¹³¥µç×è

void TIM2_PWM_Init(void);	//Ä¦²ÁÂÖ
void TIM3_PWM_Init(void);		//imuÎÂ¶È²¹³¥
void PWM_Write(uint8_t PWM_CH,float angle);
extern void fric_PWM_configuration(void);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);

#endif
