#include "spi.h"

void SPI::init(SPI_TypeDef* spix, GPIO_TypeDef* gpiox, GPIO_TypeDef* aux_gpiox, 
						uint16_t csn_pin, uint16_t sck_pin, uint16_t miso_pin, uint16_t mosi_pin,
						uint16_t ce_pin, uint16_t irq_pin, uint32_t rcc_apb1, uint32_t rcc_apb2, uint8_t config_state)
{
	/*载入数据到实例*/
	SPIx = spix;	SPI_Config_State = config_state;
	SPI_GPIOx = gpiox;	SPI_AUX_GPIOx = aux_gpiox;
	SPI_CSN_Pin = csn_pin;	SPI_SCK_Pin = sck_pin;	SPI_MISO_Pin = miso_pin;	SPI_MOSI_Pin = mosi_pin;
	SPI_CE_Pin = ce_pin;	SPI_IRQ_Pin = irq_pin;	SPI_RCC_APB1 = rcc_apb1;	SPI_RCC_APB2 = rcc_apb2;
	
	
	SPI_InitTypeDef SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	switch(SPI_Config_State)
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
	
	/*配置 gpiox 的 SCK,MISO,MOSI 引脚*/ 
	GPIO_InitStructure.GPIO_Pin = sck_pin | miso_pin | mosi_pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能 
	GPIO_Init(gpiox, &GPIO_InitStructure);
	/*配置 gpiox 的 CSN 引脚 和 aux_gpiox 的 CE 引脚*/
	GPIO_InitStructure.GPIO_Pin = csn_pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(gpiox, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ce_pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(aux_gpiox, &GPIO_InitStructure);	
	
	//GPIO_SetBits(gpiox, sck_pin | miso_pin | mosi_pin);
	GPIO_SetBits(gpiox, csn_pin);
	
	SPI_I2S_DeInit(spix);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//8位数据模式
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//空闲模式下SCK为1
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//数据采样从第2个时间边沿开始
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS软件管理
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//波特率
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//大端模式
  SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC多项式
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//主机模式
  SPI_Init(spix, &SPI_InitStructure);
  SPI_Cmd(spix, ENABLE);
}

uint8_t SPI::RW(uint8_t dat) 
{
	//方案一：
	/* 当 SPI发送缓冲器非空时等待 */ 
	//while (SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_TXE) == RESET); 
	/* 通过 SPIx发送一字节数据 */ 
	//SPI_I2S_SendData(NRF_SPI, dat); 
	/* 当SPI接收缓冲器为空时等待 */ 
	//while (SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	//return SPI_I2S_ReceiveData(NRF_SPI); 
	
	//方案二：
	
	while((SPIx->SR & SPI_I2S_FLAG_TXE) == RESET);
 	SPIx->DR = dat;
 	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == RESET);
 	return(SPIx->DR);
	
	
	//方案三：
	/* 当 SPI发送缓冲器非空时等待 */ 
	//while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET); 
	/* 通过 SPI2发送一字节数据 */ 
	//SPI_I2S_SendData(SPIx, dat); 
	/* 当SPI接收缓冲器为空时等待 */ 
	//while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	//return SPI_I2S_ReceiveData(SPIx); 
}


void SPI::CE_H(void)
{
	GPIO_SetBits(SPI_AUX_GPIOx, SPI_CE_Pin);
}

void SPI::CE_L(void)
{
	GPIO_ResetBits(SPI_AUX_GPIOx, SPI_CE_Pin);
}

void SPI::CSN_H(void)
{
	GPIO_SetBits(SPI_GPIOx, SPI_CSN_Pin);
}

void SPI::CSN_L(void)
{
	GPIO_ResetBits(SPI_GPIOx, SPI_CSN_Pin);
}
