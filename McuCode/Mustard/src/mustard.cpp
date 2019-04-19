#include "mustard.h"

//uint32_t SYS_TIMx_IRQCNT = 0;

/*******************Public*******************/

void Mustard::Mustard_Init(void)
{
	/*STATE ״̬��ʼ��*/
	Mustard_Control_State = MUSTARD_INIT_FAIL;
	Mustard_Flight_State = MUSTARD_FLIGHT_ERROR;
	
	/*PERIPH ��ʼ������*/
	led.init();
	pwm.init();
	database.init();
	imu.init();
	nrf.init(NRF_RC_MODE, NRF_RC_CHANNEL);
	
	led.stream(6, 100);
	
	/*DEBUG �Ƿ�������*/
	#if CONFIG_USE_DEBUG
	debug.init(9600);
	led.turnOn(LED_DY_Pin);
	#endif
	
	/*TASK ϵͳ����ʱ��*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef				NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(SYS_RCC_APB1, ENABLE);
	
	TIM_DeInit(SYS_TIMx);
	
	uint16_t PrescalarValue = (uint16_t)(SYS_CLOCK / 1000000) - 1;
	
	//ʹ���ڲ�����1000 - 1 ����Ч1.6M���� -> 8��Ƶ��ȡ100��
	//ʹ���ⲿ����500 - 1����Ч72M���� -> 72��Ƶ��ȡ500��
	TIM_TimeBaseStructure.TIM_Period = 100 - 1; //ÿ����0.5 ms
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalarValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(SYS_TIMx, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(SYS_TIMx, ENABLE);
	TIM_ClearFlag(SYS_TIMx, TIM_FLAG_Update);
	TIM_ITConfig(SYS_TIMx, TIM_IT_Update, ENABLE);
	
	NVIC_PriorityGroupConfig(SYS_NVIC_GROUP);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SYS_NVIC_PROMP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SYS_NVIC_SUB;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(SYS_TIMx, ENABLE);
	
	/*��ʼ�����*/
	Mustard_Control_State = MUSTARD_INIT_SUCCESS;
	Mustard_Flight_State = MUSTARD_FLIGHT_LOCKED;
	
	led.turnOn(LED_DG_Pin);
}

void Mustard::Mustard_Task(void)
{
	/*������0.5ms*/
	static uint8_t ms1 = 0, ms2 = 0, ms5 = 0, ms10 = 0, ms100 = 0;
	if(TIM_GetITStatus(SYS_TIMx, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(SYS_TIMx, TIM_IT_Update);
		//SYS_TIMx_IRQCNT++;
		
		if(!Mustard_Control_State)
			return;
		
		
		ms1++;
		ms2++;
		ms5++;
		ms10++;
			
		if(ms1 == 2)	//1ms
		{
			ms1 = 0;
			/*NRF ң���źż��*/
			nrf.checkEvent();
			
			
		}
		if(ms2 == 4)	//2ms
		{
			ms2 = 0;
			/*IMU ����ת��(int -> float)*/
			#if CONFIG_IMU_MODE == IMU_MODE_IIC
			imu.receive();
			led.turnOff(LED_DB_Pin);
			#endif
			/*PID ���ȿ��ƣ��ڻ���*/
			mustard.PID_InLoop();
		}
		if(ms5 == 10)	//5ms
		{
			ms5 = 0;
			/*PID ���ȿ��ƣ��⻷��*/
			mustard.PID_OutLoop();
			

		}
		if(ms10 == 20)	//10ms
		{
			ms10 = 0;
			ms100++;
				
			/*NRF ���ݻش�*/
			#if REDIRECT_DEBUG == true
			printf("accel.x : %d\taccel.y : %d\taccel.z : %d\n", database.accelI.x, database.accelI.y, database.accelI.z);
			#endif
			
			if(ms100 == 10)//100ms
			{
				ms100 = 0;
				
				
				/*NRF ʧ�ؼ��*/
				Secure_Lost();
			}
		}	
	}
}

void Mustard::delay_us(uint32_t us)
{
  SysTick->LOAD = (us * SYS_CLOCK / 1000000);
  SysTick->CTRL = 0x00000005;
  while(!(SysTick->CTRL & 0x00010000));
  SysTick->CTRL = 0x00000004;
}

void Mustard::delay_ms(uint32_t ms)
{
  for(; ms>0; ms--)
		Mustard::delay_us(1000);
}



/*******************Private*******************/

void Mustard::PID_OutLoop(void)
{
	int errorAngle[2];
	
	errorAngle[ROLL] 	= constrain_int(1, -FLIGHT_MAX_ANGLE_INT, FLIGHT_MAX_ANGLE_INT) - database.angleI.x;
	errorAngle[PITCH] = constrain_int(1, -FLIGHT_MAX_ANGLE_INT, FLIGHT_MAX_ANGLE_INT) - database.angleI.y;
	
	rateError[ROLL]		=	pid[PID_LEVEL].getPout(errorAngle[ROLL]) - database.palstanceI.x;
	rateError[PITCH]	=	pid[PID_LEVEL].getPout(errorAngle[PITCH]) - database.palstanceI.y;
	rateError[YAW] 		= - database.palstanceI.z;
}

void Mustard::PID_InLoop(void)
{
	int PID_Output[3];
	
	for(uint8_t i = 0; i < 3; i++)
	{
		/*���Ź�С ����������*/
		if(database.rc.expThrottle < RC_THROTTLE_MIN)
			pid[i].resetI();
		
		/*PID �ڻ����*/
		PID_Output[i] = pid[i].getPIDout(rateError[i], 1e-6);
		
	}
	
	/*PWM �����Ӧ*/
	if(database.rc.expFlightMode == RC_MODE_NORMAL)
	{
		pwmMotor[PWM_FL] = database.rc.expThrottle + PID_Output[ROLL] - PID_Output[PITCH] - PID_Output[YAW];
		pwmMotor[PWM_FR] = database.rc.expThrottle - PID_Output[ROLL] - PID_Output[PITCH] + PID_Output[YAW];
		pwmMotor[PWM_BL] = database.rc.expThrottle + PID_Output[ROLL] + PID_Output[PITCH] + PID_Output[YAW];
		pwmMotor[PWM_BR] = database.rc.expThrottle - PID_Output[ROLL] + PID_Output[PITCH] - PID_Output[YAW];
	}
	
	
	/*Flight ״̬���� �� ң�����Ŵ�����С�����ֵ*/
	if((Mustard_Flight_State >= 0))
	{
		pwm.setThrottle(pwmMotor);
	}
	else if(Mustard_Flight_State == MUSTARD_FLIGHT_LOST)
	{
		//���޸ģ�ʧ��ֱ�ӽ��䲻̫�ã���һ��ˮ���Ͼ͹���
		//������ʧ��ʱGPS�Զ�����
		for(uint8_t i = 0; i < 4; i++)
			pwmMotor[i] = 0;
			
		pwm.setThrottle(pwmMotor);
	}
	else
	{
		for(uint8_t i = 0; i < 4; i++)
			pwmMotor[i] = 1000;
			
		pwm.setThrottle(pwmMotor);
	}
}

void Mustard::Flight_Mode_TakeOff(void)
{
}

void Mustard::Flight_Mode_Fly(void)
{
	
}

void Mustard::Secure_Lost(void)
{
	static uint8_t lostSec = 0;
	
	if(lostSec == 25)	//2.5s�ڼ�ⲻ��RCң���ź�
	{
		Mustard_Flight_State = MUSTARD_FLIGHT_LOST;
	}
	if(database.rc.connectFlag)
	{
		lostSec = 0;
		database.rc.connectFlag = false;
	}
	else
		lostSec++;
}
