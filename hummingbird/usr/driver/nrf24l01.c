/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file nrf24l01.c
 *
 * NRF24L01无线射频模块接收接口
 *
 * @author Zhanghb zhanghaibo@bjfz.cc
 *
 *************************************************************************/
#include "nrf24l01.h"

//***********************************NRF24L01******************************
#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width
#define TX_PLOAD_WIDTH  20  	// 20 uints TX payload
#define RX_PLOAD_WIDTH  20  	// 20 uints TX payload

// 本地地址
uint8_t const TX_ADDRESS[TX_ADR_WIDTH] = { 
	0x34, 0x43, 0x10, 0x10, 0x01
};
// 接收地址
uint8_t const RX_ADDRESS[RX_ADR_WIDTH] = {
	0x34, 0x43, 0x10, 0x10, 0x01
};

/**************************************************************************
 ** NRF24L01寄存器指令
 **************************************************************************/
#define READ_REG        0x00  	// 读寄存器指令
#define WRITE_REG       0x20 	// 写寄存器指令
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留

/**************************************************************************
 ** SPI(nRF24L01)寄存器地址
 **************************************************************************/
#define CONFIG2         0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define STATUS          0x07  // 状态寄存器
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
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置

/**************************************************************************
 ** 状态标志位
 *************************************************************************/
uint16_t 	bdata	sta;			//状态标志
sbit			RX_DR	= sta^6;
sbit			TX_DS	= sta^5;
sbit			MAX_RT	= sta^4;
sbit			TX_FULL = sta^0;

/**************************************************************************
 ** 共有操作方法定义
 *************************************************************************/
/**
 * 函数： nRF24L01_RxPacket()
 * 功能：数据读取后放如rx_buf接收缓冲区中
 * @param unsigned char* rx_buf
 */
uint8_t nRF24L01_RxPacket(uint8_t *rx_buf)
{
	uint8_t revale = 0;
	// 读取状态寄存其来判断数据接收状况
	sta = SPI_Read(STATUS);

	// 判断是否接收到数据
	if (RX_DR) {
		CE	   = 0;											//SPI使能
		// read receive payload from RX_FIFO buffer
		SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);
		revale = 1;											//读取数据完成标志
	}

	// 接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
	SPI_RW_Reg(WRITE_REG + STATUS, sta);
	CE = 1;
	return revale;
}

/**************************************************************************
 *函数：void nRF24L01_TxPacket(uint8_t *tx_buf)
 *功能：发送 tx_buf中数据
 *************************************************************************/
void nRF24L01_TxPacket(uint8_t *tx_buf)
{
	SPI_RW_Reg(WRITE_REG + STATUS, 0xff);
	SPI_RW_Reg(0xE1, 0xff);
	CE = 0;
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG + CONFIG2, 0x0e);
	CE = 1;
	inerDelay_us(10);   //CE高电平大于10us才能进入发射模式
}



//*****************************************长延时**************************
void Delay(uint16_t s)
{
	volatile uint16_t i;
	for (i = 0; i < s; i++);
	for (i = 0; i < s; i++);
}


/**************************************************************************
/*延时函数
/*************************************************************************/
void inerDelay_us(uint8_t n)
{
	for (; n > 0; n--);
}

//*************************************************************************
/*NRF24L01初始化
//************************************************************************/
void init_NRF24L01(void)
{
	CE	= 0;					// chip enable
	CSN = 1;					// Spi  disable
	SCK = 0;					//

	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);		// 写本地地址
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);	// 写接收端地址
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);								//  频道0自动	ACK应答允许
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);							//  允许接收地址只有频道0，如果需要多频道可以参考Page21
	SPI_RW_Reg(WRITE_REG + RF_CH, 0x6e);								//   设置信道工作为2.4GHZ，收发必须一致
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);					//设置接收数据长度，本次设置为32字节
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x27);								//设置发射速率为1MB/S，发射功率为最大值+7dB，由于有X2401L功放，实际+21dbm输出
}

/**
 *函数：uint SPI_RW(uint 功能)
 *uint8_t：NRF24L01的SPI写时序
 */
uint8_t SPI_RW(uint8_t dat)
{
	uint8_t bit_ctr;

	for (bit_ctr = 0; bit_ctr < 8; bit_ctr++) {	// output 8-bit
		MOSI   = (dat & 0x80);				// output 'dat', MSB to MOSI
		dat  = (dat << 1);					// shift next bit into MSB..
		SCK	   = 1;								// Set SCK high..
		inerDelay_us(12);
		dat |= MISO;							// capture current MISO bit
		inerDelay_us(12);
		SCK	   = 0;								// ..then set SCK low again
	}

	return (dat);								// return read dat
}

/**
 *函数：uint8_t SPI_Read(uint8_t reg)
 *功能：NRF24L01的SPI时序
 */
uint8_t SPI_Read(uint8_t reg)
{
	uint8_t reg_val;
	CSN = 0;                // CSN low, initialize SPI communication...
	inerDelay_us(12);
	SPI_RW(reg);            // Select register to read from..
	inerDelay_us(12);
	reg_val = SPI_RW(0);    // ..then read registervalue
	inerDelay_us(12);
	CSN = 1;                // CSN high, terminate SPI communication
	return (reg_val);       // return register value
}

/**
 *功能：NRF24L01读写寄存器函数
 */
uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CSN	   = 0;                 // CSN low, init SPI transaction
	status = SPI_RW(reg);		// select register
	SPI_RW(value);				// ..and write value to it..
	CSN	   = 1;                 // CSN high again
	return (status);			// return nRF24L01 status uchar
}

/**
 *函数：uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
 *功能: 用于读数据，reg：为寄存器地址，pBuf：为待读出数据地址，uchars：读出数据的个数
 */
uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t	status;
	uint8_t		i;

	CSN = 0;                    		// Set CSN low, init SPI tranaction
	status = SPI_RW(reg);       		// Select register to write to and read status uchar

	for (i = 0; i < len; i++) {
		pBuf[i] = SPI_RW(0);
	}

	CSN = 1;
	return (status);                   // return nRF24L01 status uchar
}

/**
 *函数：uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
 *功能: 用于写数据：为寄存器地址，pBuf：为待写入数据地址，uchars：写入数据的个数
 */
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t	status;
	uint8_t		i;

	CSN = 0;            //SPI使能
	status = SPI_RW(reg);

	for (i = 0; i < len; i++) {
		SPI_RW(*pBuf++);
	}

	CSN = 1;			//关闭SPI
	return (status);	//
}

/**
 *函数：void SetRX_Mode(void)
 *功能：数据接收配置
 */
void SetRX_Mode(void)
{
	CE = 0;
	// IRQ收发完成中断响应，16位CRC	，主接收
	SPI_RW_Reg(WRITE_REG + CONFIG2, 0x0F);
	CE = 1;
}

