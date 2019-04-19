#ifndef __LED_H
#define __LED_H

#include "config.h"

class LED
{
public:
	void init(void);
	void turnOn(uint16_t Dx);
	void turnOff(uint16_t Dx);
	void loop(uint16_t Dx, uint8_t cycles, uint32_t interval);
	void stream(uint8_t cycles, uint32_t interval);
	bool isOn(uint16_t Dx);
};

#endif
