#include "debug.h"

void DEBUG::init(uint16_t baudRate)
{
	USART_InitTypeDef 			USART_InitStructure;
	USART_ClockInitTypeDef 	USART_ClockInitStruct;
	GPIO_InitTypeDef 				GPIO_InitStructure;
	NVIC_InitTypeDef 				NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(DEBUG_RCC_APB2, ENABLE); //����DEBUG_USARTxʱ��
	
	GPIO_InitStructure.GPIO_Pin =  DEBUG_TX_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(DEBUG_GPIOx, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  DEBUG_RX_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_GPIOx, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudRate;       //�����ʿ���ͨ������վ����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //��֡��β����1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;    //������żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
	//����DEBUG_USARTxʱ��
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���
	
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	USART_ClockInit(DEBUG_USARTx, &USART_ClockInitStruct);

	//ʹ��DEBUG_USARTx�����ж�
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);
	//ʹ��DEBUG_USARTx
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
