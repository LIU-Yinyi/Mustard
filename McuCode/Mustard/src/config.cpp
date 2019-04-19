#include "config.h"
#include "mustard.h"

extern Mustard mustard;

void USART1_IRQHandler(void)
{
	
}

void USART2_IRQHandler(void)
{
#if TRANSUART_IMU_DEBUG
	//IMU����ת��DEBUG����ģʽ
	if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
	{
		USART_ReceiveData(USART2);
	}
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);	//�Ƿ����������
		USART_SendData(DEBUG_USARTx, USART2->DR);
		while(USART_GetITStatus(DEBUG_USARTx, USART_IT_TXE) == RESET);
	}
#else
//����ģʽ
	if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
	{
		USART_ReceiveData(USART2);
	}
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);	//�Ƿ����������
		mustard.imu.parse(USART2->DR);
	}
#endif
}

void USART3_IRQHandler(void)
{
}

void UART4_IRQHandler(void)
{
}

void UART5_IRQHandler(void)
{
}

void TIM2_IRQHandler(void)
{
	mustard.Mustard_Task();
}
