#ifndef __DEBUG_H
#define __DEBUG_H

#include "config.h"
#include <string.h>

#if REDIRECT_DEBUG
#include <stdio.h>
extern "C" int fputc(int ch, FILE *f);
#endif

class DEBUG
{
public:
	void init(uint16_t baudRate = 9600);
	void quit(void);

#if REDIRECT_DEBUG == true
	/*MicroLib*/
	//void printf();
#else	
	void putChar(uint8_t ch);
	void putString(uint8_t* str);
	void putInt16(int i);
	void putFloat(float f);
#endif
};

#endif
