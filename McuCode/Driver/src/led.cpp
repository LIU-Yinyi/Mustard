#include "led.h"
#include "mustard.h"

extern Mustard mustard;

void LED::init(void)
{
	RCC_APB2PeriphClockCmd(LED_RCC_APB2, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = LED_DR_Pin | LED_DG_Pin | LED_DB_Pin | LED_DY_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(LED_GPIOx, &GPIO_InitStructure);
	
	turnOff(LED_DR_Pin | LED_DG_Pin | LED_DB_Pin | LED_DY_Pin);
}

void LED::turnOn(uint16_t Dx)
{
	GPIO_ResetBits(LED_GPIOx, Dx);
}

void LED::turnOff(uint16_t Dx)
{
	GPIO_SetBits(LED_GPIOx, Dx);
}

void LED::loop(uint16_t Dx, uint8_t cycles, uint32_t interval)
{
	for(; cycles > 0; cycles--)
	{
		turnOn(Dx);
		mustard.delay_ms(interval);
		turnOff(Dx);
		mustard.delay_ms(interval);
	}
}

void LED::stream(uint8_t cycles, uint32_t interval)
{
	for(; cycles > 0; cycles--)
	{
		turnOn(LED_DB_Pin);
		mustard.delay_ms(interval);
		turnOn(LED_DG_Pin);
		mustard.delay_ms(interval);
		turnOn(LED_DY_Pin);
		mustard.delay_ms(interval);
		turnOn(LED_DR_Pin);
		mustard.delay_ms(interval);
		turnOff(LED_DB_Pin);
		mustard.delay_ms(interval);
		turnOff(LED_DG_Pin);
		mustard.delay_ms(interval);
		turnOff(LED_DY_Pin);
		mustard.delay_ms(interval);
		turnOff(LED_DR_Pin);
		mustard.delay_ms(interval);
	}
}

bool LED::isOn(uint16_t Dx)
{
	return (GPIO_ReadOutputDataBit(LED_GPIOx, Dx) == RESET);
}
