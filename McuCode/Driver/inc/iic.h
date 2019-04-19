#ifndef __IIC_H
#define __IIC_H

#include "config.h"

/*IIC”¶¥*/
#define IIC_ACK_NO  		0
#define IIC_ACK_YES 		1
/*IIC◊‹œﬂ√¶*/
#define IIC_SBUS_BUSY 	0
#define IIC_SBUS_START 	1
/*IIC∂¡–¥◊¥Ã¨*/
#define IIC_RW_ERROR 		0
#define IIC_RW_SUCCESS 	1


/*
#define SCL_H         IIC_GPIOx->BSRR = IIC_Pin_SCL
#define SCL_L         IIC_GPIOx->BRR  = IIC_Pin_SCL
#define SDA_H         IIC_GPIOx->BSRR = IIC_Pin_SDA
#define SDA_L         IIC_GPIOx->BRR  = IIC_Pin_SDA
*/
#define SCL_READ      IIC_GPIOx->IDR  & IIC_SCL_Pin
#define SDA_READ      IIC_GPIOx->IDR  & IIC_SDA_Pin

class IIC_Soft
{
public:
	void init(GPIO_TypeDef* gpiox, uint16_t scl_pin, uint16_t sda_pin, uint32_t rcc_apb1, uint32_t rcc_apb2, uint8_t config_state);
	
	void SCL_H(void);
	void SCL_L(void);
	void SDA_H(void);
	void SDA_L(void);
	
	uint8_t singleWrite(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t RegData);
	uint8_t singleRead(uint8_t SlaveAddress, uint8_t RegAddress);
	uint8_t multiRead(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t* ptChar, uint8_t size);
		
private:
	GPIO_TypeDef* 		IIC_GPIOx;
	uint16_t					IIC_SCL_Pin, IIC_SDA_Pin;
	uint32_t					IIC_RCC_APB1, IIC_RCC_APB2;
	uint8_t						IIC_Config_State;
	/*
	[IIC_Config_State]
		0:	≈‰÷√APB1
		1:	≈‰÷√APB2
		2:	≈‰÷√APB1 ∫Õ APB2
	*/

	void delay(uint8_t times = 1);
	uint8_t start(void);
	void stop(void);
	void ack(void);
	void noAck(void);
	uint8_t waitAck(void);
	void sendByte(uint8_t buffer);
	uint8_t readByte(void);
};

#endif
