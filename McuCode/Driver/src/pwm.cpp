#include "pwm.h"

void PWM::init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef 			TIM_OCInitStructure;
	GPIO_InitTypeDef				GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(PWM_RCC_APB1, ENABLE);
	RCC_APB2PeriphClockCmd(PWM_RCC_APB2, ENABLE);
	
	#if PWM_Pin_isRemap
	GPIO_PinRemapConfig(PWM_TIMx_Remap, ENABLE);
	#endif
	
	GPIO_InitStructure.GPIO_Pin = PWM_FL_Pin | PWM_FR_Pin | PWM_BL_Pin | PWM_BR_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(PWM_GPIOx, &GPIO_InitStructure);
	
	uint16_t PrescalarValue = (uint16_t)(SYS_CLOCK / 1000000) - 1;
	
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalarValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(PWM_TIMx, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(PWM_TIMx, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(PWM_TIMx, TIM_OCPreload_Enable);
	
	TIM_OC2Init(PWM_TIMx, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(PWM_TIMx, TIM_OCPreload_Enable);
	
	TIM_OC3Init(PWM_TIMx, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(PWM_TIMx, TIM_OCPreload_Enable);
	
	TIM_OC4Init(PWM_TIMx, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(PWM_TIMx, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(PWM_TIMx, ENABLE);
	TIM_Cmd(PWM_TIMx, ENABLE);
}

void PWM::setThrottle(uint16_t throttle[Flight_Axis])
{
	/*ÏÞ·ù±£»¤*/
	for(uint8_t i = 0; i < Flight_Axis; i++)
	{
		if(throttle[i] < 1000) throttle[i] = 1000;
		if(throttle[i] > 2000) throttle[i] = 2000;
	}
	
	PWM_TIMx->CCR1 = throttle[0];	//FL
	PWM_TIMx->CCR2 = throttle[1];	//FR
	PWM_TIMx->CCR3 = throttle[2];	//BL
	PWM_TIMx->CCR4 = throttle[3];	//BR
}
