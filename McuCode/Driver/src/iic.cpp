#include "iic.h"

/********************Public********************/
void IIC_Soft::init(GPIO_TypeDef* gpiox, uint16_t scl_pin, uint16_t sda_pin, uint32_t rcc_apb1, uint32_t rcc_apb2, uint8_t config_state)
{
	IIC_GPIOx = gpiox;	IIC_Config_State = config_state;
	IIC_SCL_Pin = scl_pin; IIC_SDA_Pin = sda_pin;
	IIC_RCC_APB1 = rcc_apb1; IIC_RCC_APB2 = rcc_apb2;

	GPIO_InitTypeDef  GPIO_InitStructure; 
	
	switch(IIC_Config_State)
	{
		case 0:
			RCC_APB1PeriphClockCmd(rcc_apb1, ENABLE);
			break;
		case 1:
			RCC_APB2PeriphClockCmd(rcc_apb2, ENABLE);
			break;
		case 2:
			RCC_APB1PeriphClockCmd(rcc_apb1, ENABLE);
			RCC_APB2PeriphClockCmd(rcc_apb2, ENABLE);
			break;
	}

  GPIO_InitStructure.GPIO_Pin =  scl_pin | sda_pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //推挽或开漏(开漏要10k上拉)  
  GPIO_Init(IIC_GPIOx, &GPIO_InitStructure);
}

void IIC_Soft::SCL_H(void)
{
	GPIO_SetBits(IIC_GPIOx, IIC_SCL_Pin);
}

void IIC_Soft::SCL_L(void)
{
	GPIO_ResetBits(IIC_GPIOx, IIC_SCL_Pin);
}

void IIC_Soft::SDA_H(void)
{
	GPIO_SetBits(IIC_GPIOx, IIC_SDA_Pin);
}

void IIC_Soft::SDA_L(void)
{
	GPIO_ResetBits(IIC_GPIOx, IIC_SDA_Pin);
}

uint8_t IIC_Soft::singleWrite(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t RegData)
{
	if(!start())
		return IIC_RW_ERROR;
  sendByte(SlaveAddress);   //发送设备地址+写信号//sendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
  if(!waitAck())
	{
		stop(); 
		return IIC_RW_ERROR;
	}
  sendByte(RegAddress);   //设置低起始地址      
  waitAck();	
  sendByte(RegData);
  waitAck();   
  stop();  
	
	return IIC_RW_SUCCESS;
}

uint8_t IIC_Soft::singleRead(uint8_t SlaveAddress, uint8_t RegAddress)
{
	uint8_t RegData;     	
	if(!start())
		return IIC_RW_ERROR;
  sendByte(SlaveAddress); //sendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
  if(!waitAck())
	{
		stop();
		return IIC_RW_ERROR;
	}
  sendByte(RegAddress);   //设置低起始地址      
  waitAck();
  start();
  sendByte(SlaveAddress + 1);
  waitAck();

	RegData= readByte();
  noAck();
  stop();

	return RegData;
}

uint8_t IIC_Soft::multiRead(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t* ptChar, uint8_t size)
{
	uint8_t i;
    
  if(size < 1)
		return IIC_RW_ERROR;
  if(!start())
		return IIC_RW_ERROR;
  sendByte(SlaveAddress);
  if(!waitAck())
	{
		stop();
		return IIC_RW_ERROR;
	}
  sendByte(RegAddress);    
  waitAck();
    
  start();
  sendByte(SlaveAddress+1);
  waitAck();
    
  for(i = 1; i < size; i++)
  {
		*ptChar++ = readByte();
    ack();
  }
  *ptChar++ = readByte();
  noAck();
  stop();
  
	return IIC_RW_SUCCESS;    
}

/********************Private********************/
void IIC_Soft::delay(uint8_t times)
{
	uint8_t i = 150;
	for(;times > 0; times--)
		while(i)
			i--;
}

uint8_t IIC_Soft::start(void)
{
	SDA_H();
	SCL_H();
	delay();
	if(!SDA_READ)return IIC_SBUS_BUSY;	//SDA线为低电平则总线忙,退出
	SDA_L();
	delay();
	if(SDA_READ) return IIC_SBUS_BUSY;	//SDA线为高电平则总线出错,退出
	SDA_L();
	delay();
	return IIC_SBUS_START;	
}

void IIC_Soft::stop(void)
{
	SCL_L();
	delay();
	SDA_L();
	delay();
	SCL_H();
	delay();
	SDA_H();
	delay();
}

void IIC_Soft::ack(void)
{
	SCL_L();
	delay();
	SDA_L();
	delay();
	SCL_H();
	delay();
	SCL_L();
	delay();
}

void IIC_Soft::noAck(void)
{
	SCL_L();
	delay();
	SDA_H();
	delay();
	SCL_H();
	delay();
	SCL_L();
	delay();
}

uint8_t IIC_Soft::waitAck(void)
{
	SCL_L();
	delay();
	SDA_H();			
	delay();
	SCL_H();
	delay();
	if(SDA_READ)
	{
      SCL_L();
			delay();
      return IIC_ACK_NO;
	}
	SCL_L();
	delay();
	return IIC_ACK_YES;
}

void IIC_Soft::sendByte(uint8_t buffer)
{
	uint8_t i=8;
  while(i--)
  {
     SCL_L();
     delay();
     if(buffer & 0x80)
      SDA_H();  
     else 
			SDA_L();   
		 buffer <<= 1;
     delay();
		 SCL_H();
		 delay();
  }
  SCL_L();
}

uint8_t IIC_Soft::readByte(void)
{
	uint8_t i=8;
  uint8_t ReceiveByte=0;

  SDA_H();				
  while(i--)
  {
      ReceiveByte<<=1;      
      SCL_L();
      delay();
			SCL_H();
      delay();	
      if(SDA_READ)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L();
    return ReceiveByte;
}
