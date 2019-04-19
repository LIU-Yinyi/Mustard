#ifndef __IMU_H
#define __IMU_H

#include "config.h"

#if CONFIG_IMU_MODE == IMU_MODE_IIC
#include "iic.h"
#endif

class IMU
{
public:
#if CONFIG_IMU_MODE == IMU_MODE_IIC
	IIC_Soft iic;
#endif

	void init(uint16_t baudRate = 9600);
	
	
#if CONFIG_IMU_MODE == IMU_MODE_IIC
	void receive(void); //仅针对IIC模式有效
#else
	void parse(uint8_t recChar);	//仅针对USART模式有效
#endif
};

#endif
