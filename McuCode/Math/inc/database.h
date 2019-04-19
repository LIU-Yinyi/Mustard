#ifndef __DATABASE_H
#define __DATABASE_H

#include "config.h"
#include <math.h>

#define constrain_int(res,min,max) ((res < min) ? min : ((res > max) ? max : res))

/*浮点数与字符组互转*/
union float2byte{uint8_t byte[4]; float num;};

/*二维数据类型*/
template <typename Type> class SVector2
{
public:
	Type x, y;
	
	SVector2<Type>(){x = y = 0;}
	SVector2<Type>(const Type a, const Type b){x = a; y = b;}
	
	//向量平方
	Type vectorSquare(void) const {return (Type)(*this * *this);}
	//向量模长
	float modulusLength(void) const {return sqrt(x * x + y * y);}
	
	SVector2<Type> operator +(const SVector2<Type> &v) const {return SVector2(x + v.x , y + v.y);}
	SVector2<Type> operator -(const SVector2<Type> &v) const {return SVector2(x - v.x , y - v.y);}
	SVector2<Type> operator *(const SVector2<Type> &v) const {return SVector2(x * v.x , y * v.y);}
	SVector2<Type> operator *(const Type num) const {return SVector2(x * num , y * num);}
	SVector2<Type> operator /(const Type num) const {if(num == 0)return 0; else return SVector2(x / num , y / num);}
	Type operator %(const SVector2<Type> &v) const {return (x * v.y  - y * v.x);}
};

/*三维数据类型*/
template <typename Type> class SVector3
{
public:
	Type x, y, z;
	
	SVector3<Type>(){x = y = z = 0;}
	SVector3<Type>(const Type a, const Type b, const Type c){x = a; y = b; z = c;}
	
	//向量平方
	Type vectorSquare(void) const {return (Type)(*this * *this);}
	//向量模长
	float modulusLength(void) const {return sqrt(x * x + y * y + z * z);}
	
	SVector3<Type> operator +(const SVector3<Type> &v) const {return SVector3(x + v.x , y + v.y, z + v.z);}
	SVector3<Type> operator -(const SVector3<Type> &v) const {return SVector3(x - v.x , y - v.y, z - v.z);}
	SVector3<Type> operator *(const SVector3<Type> &v) const {return SVector3(x * v.x , y * v.y, z * v.z);}
	SVector3<Type> operator *(const Type num) const {return SVector3(x * num , y * num, z * num);}
	SVector3<Type> operator /(const Type num) const {if(num == 0)return 0; else return SVector3(x / num , y / num, z / num);}
	SVector3<Type> operator %(const SVector3<Type> &v) const {return (y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);}
};

/*时间类型*/
class Time
{
public:
	uint8_t
		YY, MM, DD,
		hh, mm, ss;
	
	uint16_t
		year, ms;
		
	Time(){YY = MM = DD = hh = mm = ss = 0; year = ms = 0;}
	//void analyze(void){year = (uint16_t)(YY) + 2000; ms = (((uint16_t)msH << 8) | msL);}
};

/*GPS地速类型*/
class GPSVectorSpeed
{
public:
	int GPSHeightI;		//cm
	int GPSYawI; 			//deciDegree
	float GPSVeloH, GPSVeloS;		//km/h -> m/s
	
	GPSVectorSpeed(){GPSHeightI = GPSYawI = 0; GPSVeloH = GPSVeloS = 0;}
};

/*RC遥控数据类型*/
class RC
{
public:
	enum{ROLL, PITCH, YAW, THROTTLE, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6};
	
	//RC期望飞行状态 [FLIGHT_STATE]
	int8_t expFlightState;
	//RC期望飞行模式 [FLIGHT_MODE]
	uint8_t expFlightMode;
	//RC期望高度和油门
	int expHeightI, expThrottle;
	
	//RC期望欧拉角角度
	SVector3<int> expAngleI;
	
	//RC连接标志
	bool connectFlag;
		
	RC(){expFlightState = MUSTARD_FLIGHT_LOCKED; expFlightMode = RC_MODE_NORMAL; connectFlag = false;}
	
	/*协议解析 返回值为是否解析成功*/
	bool analyze(uint8_t dat[NRF_RX_PLOAD_WIDTH]); 
	
};

/*数据库类型*/
class DATABASE
{
public:
	//超声波相对高度
	float 							heightF;
	int									heightI;
	//GPS矢量速度（海平面绝对高度+速度方向角+速度大小）
	GPSVectorSpeed			gpsVelo;
	float								temperatureF;
	int									temperatureI;
	//GPS坐标 [x:longitude 经度, y:latitude 纬度]
	SVector2<float> 		positionF;
	SVector2<long> 			positionI;
	//欧拉角->angle [x:roll, y:pitch, z:yaw]
	SVector3<float>			accelF, angleF, palstanceF, magnetF;
	SVector3<int>				accelI, angleI, palstanceI, magnetI;
	Time 								time;
	RC 									rc;
	
	void init(void);
};

#endif
