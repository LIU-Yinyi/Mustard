#ifndef __SPI_H
#define __SPI_H

#include "config.h"

class SPI
{
public:	
	void init(SPI_TypeDef* spix, GPIO_TypeDef* gpiox, GPIO_TypeDef* aux_gpiox, 
						uint16_t csn_pin, uint16_t sck_pin, uint16_t miso_pin, uint16_t mosi_pin,
						uint16_t ce_pin, uint16_t irq_pin, uint32_t rcc_apb1, uint32_t rcc_apb2, uint8_t config_state);
	uint8_t RW(uint8_t dat);
	void CE_H(void);
	void CE_L(void);
	void CSN_H(void);
	void CSN_L(void);
private:
	SPI_TypeDef* 			SPIx;
	GPIO_TypeDef* 		SPI_GPIOx;
	GPIO_TypeDef*			SPI_AUX_GPIOx;
	uint16_t					SPI_CSN_Pin, SPI_SCK_Pin, SPI_MISO_Pin, SPI_MOSI_Pin;
	uint16_t					SPI_CE_Pin, SPI_IRQ_Pin;
	uint32_t					SPI_RCC_APB1, SPI_RCC_APB2;
	uint8_t						SPI_Config_State;
	/*
	[SPI_Config_State]
		0:	≈‰÷√APB1
		1:	≈‰÷√APB2
		2:	≈‰÷√APB1 ∫Õ APB2
	*/
};

#endif
