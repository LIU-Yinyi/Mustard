#ifndef __NRF_H
#define __NRF_H

#include "config.h"
#include "spi.h"

#define NRF_MODEL_RX		1	//��ͨ����ģʽ		1
#define	NRF_MODEL_TX		2	//��ͨ����ģʽ
#define NRF_MODEL_RX2		3	//˫�򣬽��պ󷵻�ack��
#define NRF_MODEL_TX2		4	//˫�򣬷��ͺ����ack��

class NRF	: public SPI
{
public:
	SPI spi;
	
	/*��ʼ����ģʽΪ1~4���ŵ�Ϊ0~127*/
	void init(uint8_t model, uint8_t channel);
	/*�������ݰ���Ӧ����ģʽ2��4*/
	void TxPacket(uint8_t* tx_buf, uint8_t len);
	/*Ӧ�����ݰ���Ӧ����ģʽ3*/
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
