#ifndef __NRF_H
#define __NRF_H

#include "config.h"
#include "spi.h"

#define NRF_MODEL_RX		1	//普通接收模式		1
#define	NRF_MODEL_TX		2	//普通发送模式
#define NRF_MODEL_RX2		3	//双向，接收后返回ack包
#define NRF_MODEL_TX2		4	//双向，发送后接收ack包

class NRF	: public SPI
{
public:
	SPI spi;
	
	/*初始化：模式为1~4，信道为0~127*/
	void init(uint8_t model, uint8_t channel);
	/*发送数据包：应用于模式2和4*/
	void TxPacket(uint8_t* tx_buf, uint8_t len);
	/*应答数据包：应用于模式3*/
	void TxPacket_AP(uint8_t* tx_buf, uint8_t len);
	bool check(void);
	void checkEvent(void);
	
private:
	uint8_t NRF_RxData[NRF_RX_PLOAD_WIDTH];
	uint8_t NRF_TxData[NRF_TX_PLOAD_WIDTH];
		
	uint8_t ReadReg(uint8_t reg);
	uint8_t WriteReg(uint8_t reg, uint8_t value);
	uint8_t WriteBuf(uint8_t reg, uint8_t* pBuf, uint8_t len);
	uint8_t ReadBuf(uint8_t reg, uint8_t* pBuf, uint8_t len);
};

#endif
