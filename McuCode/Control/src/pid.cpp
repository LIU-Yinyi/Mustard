#include "pid.h"

//#define USE_LOW_PASS_FILTER

/*
 [公式] _filter = 1 / ( 2 * pi * f_cut )
 f_cut = 10 Hz -> _filter = 15.9155e-3
 f_cut = 15 Hz -> _filter = 10.6103e-3
 f_cut = 20 Hz -> _filter =  7.9577e-3
 f_cut = 25 Hz -> _filter =  6.3662e-3
 f_cut = 30 Hz -> _filter =  5.3052e-3
*/
#if defined USE_LOW_PASS_FILTER
const float  PID::_filter = 7.9577e-3;
#endif

int PID::getPout(int error)
{
	return error * kP;
}

int PID::getIout(int error, float dt)
{
	if((kI != 0) && (dt != 0))
	{
		integral += (float)error * kI * dt;
		if(integral > iMax) integral = iMax;
		if(integral < -iMax) integral = -iMax;
		
		return integral;
	}
	else
		return 0;
}

int PID::getDout(int error, float dt)
{
	if((kD != 0) && (dt != 0))
	{
		float derivative;
		
		if(sizeof(derivative) == sizeof(float))
			derivative = lastDerivative = 0;
		else
			derivative = (error - lastError) / dt;
		
		/*一通离散低通滤波降噪*/
		#if defined USE_LOW_PASS_FILTER
		derivative = lastDerivative + dt / (dt + _filter) * (derivative - lastDerivative);
		#endif
		
		lastError = error;
		lastDerivative = derivative;
		
		return derivative * kD;
	}
	else
		return 0;
}
	
int PID::getPIout(int error, float dt)
{
	return getPout(error) + getIout(error, dt);
}

int PID::getPDout(int error, float dt)
{
	return getPout(error) + getDout(error, dt);
}

int PID::getPIDout(int error, float dt)
{
	return getPout(error) + getIout(error, dt) + getDout(error, dt);
}
