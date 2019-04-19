#include "spi.h"

void SPI::init(SPI_TypeDef* spix, GPIO_TypeDef* gpiox, GPIO_TypeDef* aux_gpiox, 
						uint16_t csn_pin, uint16_t sck_pin, uint16_t miso_pin, uint16_t mosi_pin,
						uint16_t ce_pin, uint16_t irq_pin, uint32_t rcc_apb1, uint32_t rcc_apb2, uint8_t config_state)
{
	/*�������ݵ�ʵ��*/
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
	
	/*���� gpiox �� SCK,MISO,MOSI ����*/ 
	GPIO_InitStructure.GPIO_Pin = sck_pin | miso_pin | mosi_pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���ù��� 
	GPIO_Init(gpiox, &GPIO_InitStructure);
	/*���� gpiox �� CSN ���� �� aux_gpiox �� CE ����*/
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
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//ȫ˫��
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//8λ����ģʽ
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//����ģʽ��SCKΪ1
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//���ݲ����ӵ�2��ʱ����ؿ�ʼ
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS�������
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//������
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//���ģʽ
  SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC����ʽ
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//����ģʽ
  SPI_Init(spix, &SPI_InitStructure);
  SPI_Cmd(spix, ENABLE);
}

uint8_t SPI::RW(uint8_t dat) 
{
	//����һ��
	/* �� SPI���ͻ������ǿ�ʱ�ȴ� */ 
	//while (SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_TXE) == RESET); 
	/* ͨ�� SPIx����һ�ֽ����� */ 
	//SPI_I2S_SendData(NRF_SPI, dat); 
	/* ��SPI���ջ�����Ϊ��ʱ�ȴ� */ 
	//while (SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	//return SPI_I2S_ReceiveData(NRF_SPI); 
	
	//��������
	
	while((SPIx->SR & SPI_I2S_FLAG_TXE) == RESET);
 	SPIx->DR = dat;
 	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == RESET);
 	return(SPIx->DR);
	
	
	//��������
	/* �� SPI���ͻ������ǿ�ʱ�ȴ� */ 
	//while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET); 
	/* ͨ�� SPI2����һ�ֽ����� */ 
	//SPI_I2S_SendData(SPIx, dat); 
	/* ��SPI���ջ�����Ϊ��ʱ�ȴ� */ 
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
