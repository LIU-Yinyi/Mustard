#include "database.h"
#include "mustard.h"

extern Mustard mustard;

/*
[RC协议]
	帧头	：				0xA6
	功能字:
		<控制类>
		1.摇杆信号		0x8A
		2.校准信号		0x8B
		3.PID设置			0x8C
		4.自动飞行		0x8D
		<回传类>
		1.回传姿态		0x61
		2.回传GPS			0x62
		3.回传PID			0x63
		4.回传电量		0x64
		
	Protol[30] = RC_DATA_CONNECTED;

*/
#define RC_DATA_HEADER						0xA6
#define RC_DATA_CONTROL_STICK			0x8A
#define RC_DATA_CONTROL_CALIB			0x8B
#define RC_DATA_CONTROL_SETPID		0x8C
#define RC_DATA_CONTROL_AUTOFLY		0x8D
#define RC_DATA_CALLBACK_ATTITUDE	0x61
#define RC_DATA_CALLBACK_GPS			0x62
#define RC_DATA_CALLBACK_PID			0x63
#define RC_DATA_CALLBACK_BATTERY	0x64
#define RC_DATA_FILL							0xFF
#define RC_DATA_CONNECTED					0x64

/*长位分发*/
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

bool RC::analyze(uint8_t dat[NRF_RX_PLOAD_WIDTH])
{
	uint8_t i, sum = 0;
	
	//判断帧头
	if(dat[0] != RC_DATA_HEADER)
		return false;
	
	//检验校验位
	for(i = 0; i < NRF_RX_PLOAD_WIDTH - 1; i++)
		sum += dat[i];
	if(sum != dat[NRF_RX_PLOAD_WIDTH - 1])
		return false;
	
	if(dat[1] == RC_DATA_CONTROL_STICK)
	{
		mustard.database.rc.expFlightMode = dat[2];
		mustard.database.rc.expFlightState = (int)dat[3];
		mustard.database.rc.expAngleI.x = ((int)dat[4] << 8) | dat[5];
		mustard.database.rc.expAngleI.y = ((int)dat[6] << 8) | dat[7];
		mustard.database.rc.expAngleI.z =	((int)dat[8] << 8) | dat[9];
		mustard.database.rc.expHeightI	=	((int)dat[10] << 8) | dat[11];
		mustard.database.rc.expThrottle	=	((int)dat[12] << 8) | dat[13];
	}
	else if(dat[1] == RC_DATA_CONTROL_CALIB)
	{
		
	}
	else if(dat[1] == RC_DATA_CONTROL_SETPID)
	{
		float tmpP[4], tmpI[4], tmpD[4];
		tmpP[ROLL] 			= (float)((vs16)(dat[4] << 8) | dat[5]) / 1000;
		tmpI[ROLL] 			= (float)((vs16)(dat[6] << 8) | dat[7]) / 1000;
		tmpD[ROLL] 			= (float)((vs16)(dat[8] << 8) | dat[9]) / 1000;
		tmpP[PITCH] 		= (float)((vs16)(dat[10] << 8) | dat[11]) / 1000;
		tmpI[PITCH] 		= (float)((vs16)(dat[12] << 8) | dat[13]) / 1000;
		tmpD[PITCH] 		= (float)((vs16)(dat[14] << 8) | dat[15]) / 1000;
		tmpP[YAW] 			= (float)((vs16)(dat[16] << 8) | dat[17]) / 1000;
		tmpI[YAW] 			= (float)((vs16)(dat[18] << 8) | dat[19]) / 1000;
		tmpD[YAW] 			= (float)((vs16)(dat[20] << 8) | dat[21]) / 1000;
		tmpP[THROTTLE] 	= (float)((vs16)(dat[22] << 8) | dat[23]) / 1000;
		tmpI[THROTTLE] 	= (float)((vs16)(dat[24] << 8) | dat[25]) / 1000;
		tmpD[THROTTLE] 	= (float)((vs16)(dat[26] << 8) | dat[27]) / 1000;
		
		for(uint8_t i = 0; i < 4; i++)
			mustard.pid[i].setPID(tmpP[i], tmpI[i], tmpD[i]);
	}
	else if(dat[1] == RC_DATA_CONTROL_AUTOFLY)
	{
		
	}
	else if(dat[1] == RC_DATA_CALLBACK_ATTITUDE)
	{
		
	}
	else if(dat[1] == RC_DATA_CALLBACK_GPS)
	{
		
	}
	else if(dat[1] == RC_DATA_CALLBACK_PID)
	{
		
	}
	else if(dat[1] == RC_DATA_CALLBACK_BATTERY)
	{
		
	}
	
	if(dat[30] == RC_DATA_CONNECTED)
		connectFlag = true;
	else
		connectFlag = false;
	
	return true;
}

void DATABASE::init(void)
{
	heightF = 0.0f;
	heightI = 0;
}
