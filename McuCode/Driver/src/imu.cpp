#include "imu.h"
#include "mustard.h"

extern Mustard mustard;

/*
[IMU数据协议]
0.	帧头： 0x55
1.	功能字：
*/

//IMU协议
#define IMU_DATA_HEADER 0x55
#define IMU_DATA_TIME		0x50
#define IMU_DATA_ACCEL	0x51
#define IMU_DATA_PALST	0x52
#define IMU_DATA_ANGLE	0x53
#define IMU_DATA_MAGNE	0x54
#define IMU_DATA_PORT		0x55
#define IMU_DATA_HEIGH	0x56
#define IMU_DATA_LATLON	0x57
#define IMU_DATA_SPEED	0x58

//IMU读取状态
#define IMU_STATE_FREE	0x00
#define IMU_STATE_REC		0x01
#define IMU_STATE_ANAL	0x02	

#if CONFIG_IMU_MODE == IMU_MODE_IIC
//IMU IIC地址
#define IMU_IIC_ADDRESS		0xA0
//IMU IIC寄存器
#define IMU_IIC_SAVE			0x00
#define IMU_IIC_CALSW			0x01
#define IMU_IIC_RSW				0x02
#define IMU_IIC_RATE			0x03
#define IMU_IIC_BAUD			0x04
#define IMU_IIC_AXOFFSET	0x05
#define IMU_IIC_AYOFFSET	0x06
#define IMU_IIC_AZOFFSET	0x07
#define IMU_IIC_GXOFFSET	0x08
#define IMU_IIC_GYOFFSET	0x09
#define IMU_IIC_GZOFFSET	0x0A
#define IMU_IIC_HXOFFSET	0x0B
#define IMU_IIC_HYOFFSET	0x0C
#define IMU_IIC_HZOFFSET	0x0D
#define IMU_IIC_D0MODE		0x0E
#define IMU_IIC_D1MODE		0x0F
#define IMU_IIC_D2MODE		0x10
#define IMU_IIC_D3MODE		0x11
#define IMU_IIC_D0PWMH		0x12
#define IMU_IIC_D1PWMH		0x13
#define IMU_IIC_D2PWMH		0x14
#define IMU_IIC_D3PWMH		0x15
#define IMU_IIC_D0PWMT		0x16
#define IMU_IIC_D1PWMT		0x17
#define IMU_IIC_D2PWMT		0x18
#define IMU_IIC_D3PWMT		0x19
#define IMU_IIC_IICADDR		0x1A
#define IMU_IIC_LEDOFF		0x1B
#define IMU_IIC_GPSBAUD		0x1C

#define IMU_IIC_YYMM			0x30
#define IMU_IIC_DDHH			0x31
#define IMU_IIC_MMSS			0x32
#define IMU_IIC_MS				0x33
#define IMU_IIC_AX				0x34
#define IMU_IIC_AY				0x35
#define IMU_IIC_AZ				0x36
#define IMU_IIC_GX				0x37
#define IMU_IIC_GY				0x38
#define IMU_IIC_GZ				0x39
#define IMU_IIC_HX				0x3A
#define IMU_IIC_HY				0x3B
#define IMU_IIC_HZ				0x3C
#define IMU_IIC_Roll			0x3D
#define IMU_IIC_Pitch			0x3E
#define IMU_IIC_Yaw				0x3F
#define IMU_IIC_TEMP			0x40
#define IMU_IIC_D0Status	0x41
#define IMU_IIC_D1Status	0x42
#define IMU_IIC_D2Status	0x43
#define IMU_IIC_D3Status	0x44

#define IMU_IIC_LonL			0x49
#define IMU_IIC_LonH			0x4A
#define IMU_IIC_LatL			0x4B
#define IMU_IIC_LatH			0x4C
#define IMU_IIC_GPSHeight	0x4D
#define IMU_IIC_GPSYaw		0x4E
#define IMU_IIC_GPSVL			0x4F
#define IMU_IIC_GPSVH			0x50

#endif	

void IMU::init(uint16_t baudRate)
{
#if CONFIG_IMU_MODE == IMU_MODE_IIC
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	mustard.delay_ms(500);
	
	iic.init(IMU_GPIOx, IMU_SCL_Pin, IMU_SDA_Pin, IMU_RCC_APB1, IMU_RCC_APB2, 1);
#else
	USART_InitTypeDef 			USART_InitStructure;
	USART_ClockInitTypeDef 	USART_ClockInitStruct;
	GPIO_InitTypeDef 				GPIO_InitStructure;
	NVIC_InitTypeDef 				NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(IMU_RCC_APB2, ENABLE); //开启IMU_USARTx时钟

	GPIO_InitStructure.GPIO_Pin =  IMU_TX_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(IMU_GPIOx, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  IMU_RX_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(IMU_GPIOx, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudRate;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	//配置IMU_USARTx时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
	
	USART_Init(IMU_USARTx, &USART_InitStructure);
	USART_ClockInit(IMU_USARTx, &USART_ClockInitStruct);

	//使能IMU_USARTx接收中断
	USART_ITConfig(IMU_USARTx, USART_IT_RXNE, ENABLE);
	//使能IMU_USARTx
	USART_Cmd(IMU_USARTx, ENABLE);
	
	NVIC_PriorityGroupConfig(IMU_NVIC_GROUP);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IMU_NVIC_PROMP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = IMU_NVIC_SUB;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

#if CONFIG_IMU_MODE == IMU_MODE_USART
void IMU::parse(uint8_t recChar)
{
	static uint8_t RxBuf[16];
	static uint8_t i = 0, sum = IMU_DATA_HEADER;
	static uint8_t RxState = 0;
	static uint8_t Protols = 0;
	
	if(RxState == IMU_STATE_FREE)		
	{
		if(recChar == IMU_DATA_HEADER)		
		{
			RxState = IMU_STATE_REC;					
		}
	}
	else if(RxState == IMU_STATE_REC)
	{
		if(recChar == IMU_DATA_TIME)	
		{
			RxState = IMU_STATE_ANAL;					
			Protols = IMU_DATA_TIME;
			sum += IMU_DATA_TIME;
		}
		else if(recChar == IMU_DATA_ACCEL)
		{
			RxState = IMU_STATE_ANAL;
			Protols = IMU_DATA_ACCEL;
			sum += IMU_DATA_ACCEL;
		}
		else if(recChar == IMU_DATA_PALST)
		{
			RxState = IMU_STATE_ANAL;
			Protols = IMU_DATA_PALST;
			sum += IMU_DATA_PALST;
		}
		else if(recChar == IMU_DATA_ANGLE)
		{
			RxState = IMU_STATE_ANAL;
			Protols = IMU_DATA_ANGLE;
			sum += IMU_DATA_ANGLE;
		}
		else if(recChar == IMU_DATA_MAGNE)
		{
			RxState = IMU_STATE_ANAL;
			Protols = IMU_DATA_MAGNE;
			sum += IMU_DATA_MAGNE;
		}
		else if(recChar == IMU_DATA_PORT)
		{
			RxState = IMU_STATE_ANAL;
			Protols = IMU_DATA_PORT;
			sum += IMU_DATA_PORT;
		}
		else if(recChar == IMU_DATA_HEIGH)
		{
			RxState = IMU_STATE_ANAL;
			Protols = IMU_DATA_HEIGH;
			sum += IMU_DATA_HEIGH;
		}
		else if(recChar == IMU_DATA_LATLON)
		{
			RxState = IMU_STATE_ANAL;
			Protols = IMU_DATA_LATLON;
			sum += IMU_DATA_LATLON;
		}
		else if(recChar == IMU_DATA_SPEED)
		{
			RxState = IMU_STATE_ANAL;
			Protols = IMU_DATA_SPEED;
			sum += IMU_DATA_SPEED;
		}
		else
		{
			RxState = IMU_STATE_FREE;
		}
	}
	else if(RxState == IMU_STATE_ANAL)
	{
		if(Protols == IMU_DATA_TIME)
		{
			if(i <= 7)
			{
				RxBuf[i] = recChar;
				sum += RxBuf[i];
				i++;
			}
			else if(i == 8)
			{
				if(sum == recChar)
				{
					//解析数据包
					mustard.database.time.YY = RxBuf[0];
					mustard.database.time.MM = RxBuf[1];
					mustard.database.time.DD = RxBuf[2];
					mustard.database.time.hh = RxBuf[3];
					mustard.database.time.mm = RxBuf[4];
					mustard.database.time.ss = RxBuf[5];
					mustard.database.time.ms = ((uint16_t)RxBuf[7] << 8) | RxBuf[6];
					mustard.database.time.year = (uint16_t)RxBuf[0] + 2000;
				}
				i = 0;
				sum = IMU_DATA_HEADER;
				RxState = IMU_STATE_FREE;
			}
		}
		else if(Protols == IMU_DATA_ACCEL)
		{
			if(i <= 7)
			{
				RxBuf[i] = recChar;
				sum += RxBuf[i];
				i++;
			}
			else if(i == 8)
			{
				if(sum == recChar)
				{
					//解析数据包
					mustard.database.accelI.x = ((int)RxBuf[1] << 8) | RxBuf[0];
					mustard.database.accelI.y = ((int)RxBuf[3] << 8) | RxBuf[2];
					mustard.database.accelI.z = ((int)RxBuf[5] << 8) | RxBuf[4];
					mustard.database.temperatureI = ((int)RxBuf[7] << 8) | RxBuf[6];
				}
				i = 0;
				sum = IMU_DATA_HEADER;
				RxState = IMU_STATE_FREE;
			}
		}
		else if(Protols == IMU_DATA_PALST)
		{
			if(i <= 7)
			{
				RxBuf[i] = recChar;
				sum += RxBuf[i];
				i++;
			}
			else if(i == 8)
			{
				if(sum == recChar)
				{
					//解析数据包
					mustard.database.palstanceI.x = ((int)RxBuf[1] << 8) | RxBuf[0];
					mustard.database.palstanceI.y = ((int)RxBuf[3] << 8) | RxBuf[2];
					mustard.database.palstanceI.z = ((int)RxBuf[5] << 8) | RxBuf[4];
					mustard.database.temperatureI = ((int)RxBuf[7] << 8) | RxBuf[6];
				}
				i = 0;
				sum = IMU_DATA_HEADER;
				RxState = IMU_STATE_FREE;
			}
		}
		else if(Protols == IMU_DATA_ANGLE)
		{
			if(i <= 7)
			{
				RxBuf[i] = recChar;
				sum += RxBuf[i];
				i++;
			}
			else if(i == 8)
			{
				if(sum == recChar)
				{
					//解析数据包
					mustard.database.angleI.x = ((int)RxBuf[1] << 8) | RxBuf[0];
					mustard.database.angleI.y = ((int)RxBuf[3] << 8) | RxBuf[2];
					mustard.database.angleI.z = ((int)RxBuf[5] << 8) | RxBuf[4];
					mustard.database.temperatureI = ((int)RxBuf[7] << 8) | RxBuf[6];
				}
				i = 0;
				sum = IMU_DATA_HEADER;
				RxState = IMU_STATE_FREE;
			}
		}
		else if(Protols == IMU_DATA_MAGNE)
		{
			if(i <= 7)
			{
				RxBuf[i] = recChar;
				sum += RxBuf[i];
				i++;
			}
			else if(i == 8)
			{
				if(sum == recChar)
				{
					//解析数据包
					mustard.database.magnetI.x = ((int)RxBuf[1] << 8) | RxBuf[0];
					mustard.database.magnetI.y = ((int)RxBuf[3] << 8) | RxBuf[2];
					mustard.database.magnetI.z = ((int)RxBuf[5] << 8) | RxBuf[4];
					mustard.database.temperatureI = ((int)RxBuf[7] << 8) | RxBuf[6];
				}
				i = 0;
				sum = IMU_DATA_HEADER;
				RxState = IMU_STATE_FREE;
			}
		}
		else if(Protols == IMU_DATA_PORT)
		{
			if(i <= 7)
			{
				RxBuf[i] = recChar;
				sum += RxBuf[i];
				i++;
			}
			else if(i == 8)
			{
				if(sum == recChar)
				{
					//解析数据包
					//TODO:
				}
				i = 0;
				sum = IMU_DATA_HEADER;
				RxState = IMU_STATE_FREE;
			}
		}
		else if(Protols == IMU_DATA_HEIGH)
		{
			if(i <= 7)
			{
				RxBuf[i] = recChar;
				sum += RxBuf[i];
				i++;
			}
			else if(i == 8)
			{
				if(sum == recChar)
				{
					//解析数据包
					//TODO:
				}
				i = 0;
				sum = IMU_DATA_HEADER;
				RxState = IMU_STATE_FREE;
			}
		}
		else if(Protols == IMU_DATA_LATLON)
		{
			if(i <= 7)
			{
				RxBuf[i] = recChar;
				sum += RxBuf[i];
				i++;
			}
			else if(i == 8)
			{
				if(sum == recChar)
				{
					//解析数据包
					mustard.database.positionI.x = ((long)RxBuf[3] << 24) | ((long)RxBuf[2] << 16) | ((long)RxBuf[1] << 8) | RxBuf[0];
					mustard.database.positionI.y = ((long)RxBuf[7] << 24) | ((long)RxBuf[6] << 16) | ((long)RxBuf[5] << 8) | RxBuf[4];
				}
				i = 0;
				sum = IMU_DATA_HEADER;
				RxState = IMU_STATE_FREE;
			}
		}
		else if(Protols == IMU_DATA_SPEED)
		{
			if(i <= 7)
			{
				RxBuf[i] = recChar;
				sum += RxBuf[i];
				i++;
			}
			else if(i == 8)
			{
				if(sum == recChar)
				{
					//解析数据包
					mustard.database.gpsVelo.GPSHeightI = ((int)RxBuf[1] << 8) | RxBuf[0];
					mustard.database.gpsVelo.GPSYawI = ((int)RxBuf[3] << 8) | RxBuf[2];
					long tmpVelo = ((long)RxBuf[7] << 24) | ((long)RxBuf[6] << 16) | ((long)RxBuf[5] << 8) | RxBuf[4];
					mustard.database.gpsVelo.GPSVeloH = (float)tmpVelo / 1000.0;
					mustard.database.gpsVelo.GPSVeloS = (float)tmpVelo / 3600.0;
				}
				i = 0;
				sum = IMU_DATA_HEADER;
				RxState = IMU_STATE_FREE;
			}
		}
	}
}
#endif

#if CONFIG_IMU_MODE == IMU_MODE_IIC
void IMU::receive(void)
{
	uint8_t RxBuf[4];
	
/*时间部分*/
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_YYMM, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.time.YY = RxBuf[0];
		mustard.database.time.MM = RxBuf[1];
		mustard.database.time.year = (uint16_t)RxBuf[0] + 2000;
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_DDHH, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.time.DD = RxBuf[0];
		mustard.database.time.hh = RxBuf[1];
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_MMSS, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.time.mm = RxBuf[0];
		mustard.database.time.ss = RxBuf[1];
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_MS, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.time.ms = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	
/*姿态部分*/
	//加速度
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_AX, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.accelI.x = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
		printf("\nRxBuf[0] = %d\n", RxBuf[0]);
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_AY, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.accelI.y = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_AZ, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.accelI.z = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	//角速度
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_GX, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.palstanceI.x = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_GY, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.palstanceI.y = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_GZ, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.palstanceI.z = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	//磁场
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_HX, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.magnetI.x = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_HY, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.magnetI.y = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_HZ, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.magnetI.z = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	//角度
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_Roll, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.angleI.x = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_Pitch, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.angleI.y = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
	if(iic.multiRead(IMU_IIC_ADDRESS, IMU_IIC_Yaw, RxBuf, 2) == IIC_RW_SUCCESS)
	{
		mustard.database.angleI.z = ((uint16_t)RxBuf[1] << 8) | RxBuf[0];
	}
}
#endif
