#include "nrf.h"
#include "mustard.h"

extern Mustard mustard;

//***************************************NRF24L01�Ĵ���ָ��*******************************************************
#define NRF_READ_REG        0x00  	// ���Ĵ���ָ��
#define NRF_WRITE_REG       0x20 	// д�Ĵ���ָ��
#define R_RX_PL_WID   	0x60		//��ȡ���ݳ��ȼĴ���
#define RD_RX_PLOAD     0x61  	// ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0  	// д��������ָ��
#define FLUSH_TX        0xE1 	// ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2  	// ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3  	// �����ظ�װ������ָ��
#define NOP             0xFF  	// ����
//*************************************SPI(nRF24L01)�Ĵ�����ַ****************************************************
#define CONFIG          0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define NRFRegSTATUS    0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���           
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��1�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��2�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��3�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��4�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��5�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������
//**************************************************************************************
//*********************************************NRF24L01*************************************
#define RX_DR				6		//�жϱ�־
#define TX_DS				5
#define MAX_RT			4

uint8_t	TX_ADDRESS[NRF_TX_ADDR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01};	//���ص�ַ
uint8_t	RX_ADDRESS[NRF_RX_ADDR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01};	//���յ�ַ	


/********************Public********************/

void NRF::init(uint8_t model, uint8_t channel)
{
	spi.init(NRF_SPIx, NRF_GPIOx, NRF_AUX_GPIOx, 
					NRF_CSN_Pin, NRF_SCK_Pin, NRF_MISO_Pin, NRF_MOSI_Pin, 
					NRF_CE_Pin, NRF_IRQ_Pin, NRF_RCC_APB1, NRF_RCC_APB2, 1);
	
	/*�����ж� ��Ҫ��ʱ*/
	mustard.delay_ms(100);
	
	spi.CE_L();
	WriteBuf(NRF_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, NRF_RX_ADDR_WIDTH);
	WriteBuf(NRF_WRITE_REG + TX_ADDR, TX_ADDRESS, NRF_TX_ADDR_WIDTH);
	WriteReg(NRF_WRITE_REG + EN_AA, 0x01);
	WriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01);
	WriteReg(NRF_WRITE_REG + SETUP_RETR, 0x1A);
	WriteReg(NRF_WRITE_REG + RF_CH, channel);
	WriteReg(NRF_WRITE_REG + RF_SETUP, 0x0F);	//0x0F, 0x07, 0x27
	
	if(model == NRF_MODEL_RX)
	{
		WriteReg(NRF_WRITE_REG + RX_PW_P0, NRF_RX_PLOAD_WIDTH);
		WriteReg(NRF_WRITE_REG + CONFIG, 0x0F);
	}
	else if(model == NRF_MODEL_TX)
	{
		WriteReg(NRF_WRITE_REG + RX_PW_P0, NRF_RX_PLOAD_WIDTH);
		WriteReg(NRF_WRITE_REG + CONFIG, 0x0E);
	}
	else if(model == NRF_MODEL_RX2)
	{
		WriteReg(FLUSH_TX, 0xFF);
		WriteReg(FLUSH_RX, 0xFF);
		WriteReg(NRF_WRITE_REG + CONFIG, 0x0F);
		
		spi.RW(0x50);
		spi.RW(0x73);
		WriteReg(NRF_WRITE_REG + 0x1C, 0x01);
		WriteReg(NRF_WRITE_REG + 0x1D, 0x06);
	}
	else //NRF_MODEL_TX2
	{
		WriteReg(NRF_WRITE_REG + CONFIG, 0x0E);
		WriteReg(FLUSH_TX, 0xFF);
		WriteReg(FLUSH_RX, 0xFF);
		
		spi.RW(0x50);
		spi.RW(0x73);
		WriteReg(NRF_WRITE_REG + 0x1C, 0x01);
		WriteReg(NRF_WRITE_REG + 0x1D, 0x06);
	}
	spi.CE_H();
}

void NRF::TxPacket(uint8_t* tx_buf, uint8_t len)
{
	spi.CE_L();
	WriteBuf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, NRF_TX_ADDR_WIDTH);
	WriteBuf(WR_TX_PLOAD, tx_buf, len);
	spi.CE_H();
}

void NRF::TxPacket_AP(uint8_t* tx_buf, uint8_t len)
{
	spi.CE_L();
	WriteBuf(0x8A, tx_buf, len);
	spi.CE_H();
}

bool NRF::check(void)
{
	uint8_t i, buf[5];
	WriteBuf(NRF_WRITE_REG + TX_ADDR, TX_ADDRESS, 5);
	ReadBuf(TX_ADDR, buf, 5);
	for(i = 0; i < 5; i++)
	{
		if(buf[i] != TX_ADDRESS[i])
			break;
	}
	if(i == 5)
		return true;
	else
		return false;
}

void NRF::checkEvent(void)
{
	uint8_t state = ReadReg(NRF_READ_REG + NRFRegSTATUS);
	
	if(state & (1 << RX_DR))
	{
		uint8_t rx_len = ReadReg(R_RX_PL_WID);
		if(rx_len < 33)
		{
			ReadBuf(RD_RX_PLOAD, NRF_RxData, rx_len);
			//TODO: Data Analyze
			mustard.database.rc.analyze(NRF_RxData);
		}
		else
			WriteReg(FLUSH_RX, 0xFF);
	}
	
	if(state & (1 << TX_DS))
	{
		
	}
	
	if(state & (1 << MAX_RT))
	{
		/*TX FIFO FULL*/
		if(state & 0x01)
		{
			WriteReg(FLUSH_TX, 0xFF);
		}
	}
	
	WriteReg(NRF_WRITE_REG + NRFRegSTATUS, state);
}


/********************Private********************/

uint8_t NRF::ReadReg(uint8_t reg)
{
	uint8_t regVal;
	spi.CSN_L();
	spi.RW(reg);
	regVal = spi.RW(0x00);
	spi.CSN_H();
	
	return regVal;
}

uint8_t NRF::WriteReg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	spi.CSN_L();
	status = spi.RW(reg);
	spi.RW(value);
	spi.CSN_H();
	
	return status;
}

uint8_t NRF::WriteBuf(uint8_t reg, uint8_t* pBuf, uint8_t len)
{
	uint8_t i, status;
	spi.CSN_L();
	status = spi.RW(reg);
	for(i = 0; i < len; i++)
		spi.RW(pBuf[i]);
	spi.CSN_H();
	
	return status;
}

uint8_t NRF::ReadBuf(uint8_t reg, uint8_t* pBuf, uint8_t len)
{
	uint8_t i, status;
	spi.CSN_L();
	status = spi.RW(reg);
	for(i = 0; i < len; i++)
		pBuf[i] = spi.RW(0x00);
	spi.CSN_H();
	
	return status;
}
