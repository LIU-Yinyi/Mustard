#ifndef __MUSTARD_CONFIG
#define __MUSTARD_CONFIG

#include <stm32f10x.h>

/*���������*/
#define Flight_Axis 4
enum {PWM_FL = 0, PWM_FR = 1, PWM_BL = 2, PWM_BR = 3};

/*RCң�ؽ���*/
	/*
	[ADCֵң������]
		����/����0		 ~	8191
		΢��/΢��8192	 ~	24575
		��ͣ/����24576 ~	40959
		΢��/΢�ң�40960 ~  57343
		����/���ң�57344 ~  65535
	*/
#define RC_STICK_LEVEL_1 	8192
#define RC_STICK_LEVEL_2 	16384
#define RC_STICK_LEVEL_3	32768
#define RC_STICK_LEVEL_4	49152
#define RC_STICK_LEVEL_5	57344
#define RC_STICK_WIDTH		8192

#define RC_THROTTLE_MIN		1100
#define RC_THROTTLE_MAX		2000

/*ROLL��PITCH����������̬�� 32768 / 180�� * 15�� = 2730.6*/
#define FLIGHT_MAX_ANGLE_INT 2731 

enum CONTROL_STATE
{
	MUSTARD_INIT_FAIL = 0, 
	MUSTARD_INIT_SUCCESS = 1
};
enum FLIGHT_STATE
{	
	MUSTARD_FLIGHT_LOST			=	-3,
	MUSTARD_FLIGHT_ERROR 		= -2, 
	MUSTARD_FLIGHT_LOCKED		=	-1,
	MUSTARD_FLIGHT_FREE 		= 0, 
	MUSTARD_FLIGHT_TAKEOFF 	= 1, 
	MUSTARD_FLIGHT_FLYING 	= 2, 
	MUSTARD_FLIGHT_LANDING 	= 3,
	MUSTARD_FLIGHT_CHASE		=	4
};
enum FLIGHT_MODE
{
	RC_MODE_NORMAL	=	0,
	RC_MODE_HEIGHT	=	1,
	RC_MODE_AUTO		=	2
};

/************��������*************/

/*�Ƿ�ʹ������*/
#define CONFIG_USE_LED			true
#define CONFIG_USE_ADC			false
#define CONFIG_USE_DEBUG		true
#define CONFIG_USE_RAS			false
#define CONFIG_USE_DEFAULT	false

/*�Ƿ���д���ת������*/
#define TRANSUART_IMU_DEBUG	false
#define TRANSUART_RAS_DEBUG	false	

/*�Ƿ���д����ض���*/
#define REDIRECT_DEBUG			true
#define REDIRECT_RAS				false

/*��������ģʽ*/
#define IMU_MODE_USART			0
#define IMU_MODE_IIC				1

#define CONFIG_IMU_MODE			IMU_MODE_USART
#define CONFIG_NRF_MODE

/************����ӿ�*************/

/*ϵͳ�����ж϶�ʱ���ӿ�*/
#define SYS_RCC_APB1				RCC_APB1Periph_TIM2
#define SYS_RCC_APB2				0
#define SYS_TIMx						TIM2
#define SYS_CLOCK						72000000
#define SYS_NVIC_GROUP			NVIC_PriorityGroup_1
#define SYS_NVIC_PROMP			3
#define SYS_NVIC_SUB				1

/*PWM�ӿ�*/
#define PWM_RCC_APB1				RCC_APB1Periph_TIM3
#define PWM_RCC_APB2				RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO
#define PWM_Pin_isRemap			true
#define PWM_TIMx_Remap			GPIO_FullRemap_TIM3
#define PWM_GPIOx						GPIOC
#define PWM_TIMx						TIM3
#define PWM_FL_Pin					GPIO_Pin_6
#define PWM_FR_Pin					GPIO_Pin_7
#define PWM_BL_Pin					GPIO_Pin_8
#define PWM_BR_Pin					GPIO_Pin_9

/*LED�ӿ�*/
#define LED_RCC_APB1				0
#define LED_RCC_APB2				RCC_APB2Periph_GPIOE
#define LED_GPIOx						GPIOE
#define LED_DR_Pin					GPIO_Pin_2
#define LED_DG_Pin					GPIO_Pin_3
#define LED_DB_Pin					GPIO_Pin_5
#define LED_DY_Pin					GPIO_Pin_4

/*Debug��USART�ӿ�*/
#define DEBUG_RCC_APB1			0
#define DEBUG_RCC_APB2			RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO
#define DEBUG_USARTx				USART1
#define DEBUG_GPIOx					GPIOA
#define DEBUG_TX_Pin				GPIO_Pin_9
#define DEBUG_RX_Pin				GPIO_Pin_10
#define DEBUG_NVIC_GROUP		NVIC_PriorityGroup_1
#define DEBUG_NVIC_PROMP		5
#define DEBUG_NVIC_SUB			1

/*IMU��USART�ӿ�*/
#if CONFIG_IMU_MODE == IMU_MODE_USART
//USARTģʽ
#define IMU_RCC_APB1				RCC_APB1Periph_USART2
#define IMU_RCC_APB2				RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO
#define IMU_USARTx					USART2
#define IMU_GPIOx						GPIOD
#define IMU_TX_Pin					GPIO_Pin_5
#define IMU_RX_Pin					GPIO_Pin_6
#define IMU_NVIC_GROUP			NVIC_PriorityGroup_0
#define IMU_NVIC_PROMP			2
#define IMU_NVIC_SUB				1
//����USART�ӿ�
#endif

/*GPS��USART�ӿ�*/
#define GPS_RCC_APB1
#define GPS_RCC_APB2
#define GPS_USARTx
#define GPS_GPIOx
#define GPS_TX_Pin
#define GPS_RX_Pin
#define GPS_NVIC_PROMP
#define GPS_NVIC_SUB

/*Raspi��USART�ӿڣ�ͼ������Э����*/
#define RAS_RCC_APB1				RCC_APB1Periph_UART4
#define RAS_RCC_APB2				RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO
#define RAS_USARTx					UART4
#define RAS_GPIOx						GPIOC
#define RAS_TX_Pin					GPIO_Pin_10
#define RAS_RX_Pin					GPIO_Pin_11
#define RAS_NVIC_PROMP			4
#define RAS_NVIC_SUB				1

/*IMU��IIC�ӿ�*/
#if CONFIG_IMU_MODE == IMU_MODE_IIC
//IICģʽ
#define IMU_RCC_APB1				0
#define IMU_RCC_APB2				RCC_APB2Periph_GPIOB
#define IMU_GPIOx						GPIOB
#define IMU_SCL_Pin					GPIO_Pin_6
#define IMU_SDA_Pin					GPIO_Pin_7
//����IIC�ӿ�
#endif

/*ADC�ӿ�*/

/*MS5611��IIC�ӿ�*/
#define MS5611_RCC_APB1
#define MS5611_RCC_APB2
#define MS5611_IICx
#define MS5611_GPIOx
#define MS5611_SCL_Pin
#define MS5611_SDA_Pin

/*OLED��IIC�ӿڣ����ģ��IIC��*/
#define OLED_RCC_APB1
#define OLED_RCC_APB2
#define OLED_GPIOx
#define OLED_SCL_Pin
#define OLED_SDA_Pin

/*NRF��SPI�ӿ�*/
#define NRF_RCC_APB1				0
#define NRF_RCC_APB2				RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO
#define NRF_SPIx						SPI1
#define NRF_GPIOx						GPIOA
#define NRF_CSN_Pin					GPIO_Pin_4
#define NRF_SCK_Pin					GPIO_Pin_5			
#define NRF_MISO_Pin				GPIO_Pin_6
#define NRF_MOSI_Pin				GPIO_Pin_7
#define NRF_AUX_GPIOx				GPIOC
#define NRF_CE_Pin					GPIO_Pin_4
#define NRF_IRQ_Pin					GPIO_Pin_5
#define NRF_RC_MODE					NRF_MODEL_TX2
#define NRF_RC_CHANNEL			80
#define	NRF_RX_PLOAD_WIDTH	32 
#define NRF_TX_PLOAD_WIDTH	32
#define NRF_RX_ADDR_WIDTH		5
#define NRF_TX_ADDR_WIDTH		5	

/*FLASH��SPI�ӿ�*/
#define FLASH_RCC_APB1			RCC_APB1Periph_SPI2
#define FLASH_RCC_APB2			RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD
#define FLASH_SPIx					SPI2
#define FLASH_GPIOx					GPIOB
#define FLASH_CSN_Pin				GPIO_Pin_12
#define FLASH_SCK_Pin				GPIO_Pin_13
#define FLASH_MISO_Pin			GPIO_Pin_14
#define FLASH_MOSI_Pin			GPIO_Pin_15
#define FLASH_AUX_GPIOx			GPIOD
#define FLASH_WP_Pin				GPIO_Pin_10
#define FLASH_HOLD_Pin			GPIO_Pin_11

/*ȱʡ�ӿ�*/
#define DEFAULT_RCC_APB1		0
#define DEFAULT_RCC_APB2		RCC_APB2Periph_GPIOE
#define DEFAULT_GPIOx				GPIOE
#define DEFAULT_Pin					GPIO_Pin_All

/*�ж�ȫ������*/
extern "C" void USART1_IRQHandler(void);

#if CONFIG_IMU_MODE == IMU_MODE_USART
extern "C" void USART2_IRQHandler(void);
#endif

extern "C" void USART3_IRQHandler(void);
extern "C" void UART4_IRQHandler(void);
extern "C" void UART5_IRQHandler(void);

extern "C" void TIM2_IRQHandler(void);

#endif
