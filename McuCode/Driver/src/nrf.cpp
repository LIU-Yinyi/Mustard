#include "nrf.h"
#include "mustard.h"

extern Mustard mustard;

//***************************************NRF24L01寄存器指令*******************************************************
#define NRF_READ_REG        0x00  	// 读寄存器指令
#define NRF_WRITE_REG       0x20 	// 写寄存器指令
#define R_RX_PL_WID   	0x60		//读取数据长度寄存器
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留
//*************************************SPI(nRF24L01)寄存器地址****************************************************
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define NRFRegSTATUS    0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道1接收数据长度
#define RX_PW_P2        0x13  // 接收频道2接收数据长度
#define RX_PW_P3        0x14  // 接收频道3接收数据长度
#define RX_PW_P4        0x15  // 接收频道4接收数据长度
#define RX_PW_P5        0x16  // 接收频道5接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
//**************************************************************************************
//*********************************************NRF24L01*************************************
#define RX_DR				6		//中断标志
#define TX_DS				5
#define MAX_RT			4

uint8_t	TX_ADDRESS[NRF_TX_ADDR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01};	//本地地址
uint8_t	RX_ADDRESS[NRF_RX_ADDR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01};	//接收地址	


/********************Public********************/

void NRF::init(uint8_t model, uint8_t channel)
{
	spi.init(NRF_SPIx, NRF_GPIOx, NRF_AUX_GPIOx, 
					NRF_CSN_Pin, NRF_SCK_Pin, NRF_MISO_Pin, NRF_MOSI_Pin, 
					NRF_CE_Pin, NRF_IRQ_Pin, NRF_RCC_APB1, NRF_RCC_APB2, 1);
	
	/*可能有毒 需要延时*/
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
