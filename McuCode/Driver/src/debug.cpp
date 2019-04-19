#include "debug.h"

void DEBUG::init(uint16_t baudRate)
{
	USART_InitTypeDef 			USART_InitStructure;
	USART_ClockInitTypeDef 	USART_ClockInitStruct;
	GPIO_InitTypeDef 				GPIO_InitStructure;
	NVIC_InitTypeDef 				NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(DEBUG_RCC_APB2, ENABLE); //开启DEBUG_USARTx时钟
	
	GPIO_InitStructure.GPIO_Pin =  DEBUG_TX_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(DEBUG_GPIOx, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  DEBUG_RX_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_GPIOx, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudRate;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	//配置DEBUG_USARTx时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
	
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	USART_ClockInit(DEBUG_USARTx, &USART_ClockInitStruct);

	//使能DEBUG_USARTx接收中断
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);
	//使能DEBUG_USARTx
	USART_Cmd(DEBUG_USARTx, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DEBUG_NVIC_PROMP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = DEBUG_NVIC_SUB;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void DEBUG::quit(void)
{
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, DISABLE);
	USART_Cmd(DEBUG_USARTx, DISABLE);
	USART_DeInit(DEBUG_USARTx);
	RCC_APB2PeriphClockCmd(DEBUG_RCC_APB2, DISABLE);
}

#if REDIRECT_DEBUG == true
int fputc(int ch, FILE *f)
{
	USART_SendData(DEBUG_USARTx, (uint8_t)ch);
	while(USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TC) == RESET);
	
	return ch;
}

#endif
