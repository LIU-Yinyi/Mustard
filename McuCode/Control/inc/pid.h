#ifndef __PID_H
#define __PID_H

#include "config.h"

enum {ROLL = 0, PITCH = 1, YAW = 2, PID_LEVEL = 3, PID_MAGNE = 4};

class PID
{
public:
	uint8_t state;
	
	PID(){integral = 0.0; iMax = 100.0; setPID(0.5, 0.1, 0.4);}
	
	/*设置PID参数*/
	void setP(float P){kP = P;}
	void setI(float I){kI = I;}
	void setD(float D){kD = D;}
	
	void setPI(float P, float I){kP = P; kI = I;}
	void setPD(float P, float D){kP = P; kD = D;}
	void setPID(float P, float I, float D){kP = P; kI = I;kD = D;}
	
	/*重置积分项*/
	void resetI(void){integral = 0;}
	
	/*获得PID输出值*/
	int getPout(int error);
	int getIout(int error, float dt);
	int getDout(int error, float dt);
	
	int getPIout(int error, float dt);
	int getPDout(int error, float dt);
	int getPIDout(int error, float dt);
	
private:
	float kP, kI, kD;
	float iMax, integral;
	
	int lastError;
	float lastDerivative;
	
	/*微分项的截止滤波器*/
	static const float _filter;
};

#endif
