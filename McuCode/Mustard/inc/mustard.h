#ifndef __MUSTARD_SDK
#define __MUSTARD_SDK

#include "config.h"
#include "led.h"
#include "pwm.h"
#include "debug.h"
#include "database.h"
#include "pid.h"
#include "imu.h"
#include "spi.h"
#include "nrf.h"

class Mustard
{
public:
	LED 				led;
	PWM 				pwm;
	DATABASE 		database;
	PID					pid[5];
	IMU 				imu;
	NRF					nrf;
	
	
	#if CONFIG_USE_DEBUG
	DEBUG debug;
	#endif
	
	void Mustard_Init(void);
	void Mustard_Task(void);
	
	void delay_us(uint32_t us);
	void delay_ms(uint32_t ms);
	
	uint8_t Mustard_GetState(void){return Mustard_Flight_State;}
	

private:
	int8_t
		Mustard_Control_State,
		Mustard_Flight_State;
		
	/*PID 外环输出给内环的期望误差*/
	int rateError[3];
	uint16_t pwmMotor[Flight_Axis];
	
	/*PID 外环和内环计算*/
	void PID_OutLoop(void);
	void PID_InLoop(void);
	
	/*针对自动飞行*/
	void Flight_Mode_TakeOff(void);
	void Flight_Mode_Fly(void);
	
	/*失控保护*/
	void Secure_Lost(void);
};

extern Mustard mustard;

#endif
