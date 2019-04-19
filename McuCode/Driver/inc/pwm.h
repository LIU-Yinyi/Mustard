#ifndef __PWM_H
#define __PWM_H

#include "config.h"

/*************
[PWM˳��]
	CCR1: FL
	CCR2: FR
	CCR3: BL
	CCR4: BR
*/

class PWM
{
public:
	void init(void);
	void setThrottle(uint16_t throttle[Flight_Axis]);
};

#endif
