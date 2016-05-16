/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file nrf24l01.h
 *
 * Abstract
 *
 * Detail
 *
 * @author Author zhaingbo@foxmail.com
 *
 *************************************************************************/
#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include "../inc/stdtype.h"

/**************************************************************************
 **                    nRF24L01引脚定义
 *************************************************************************/
sbit	CE	 = P4^6;
sbit	CSN	 = P4^5;
sbit	SCK	 = P2^7;
sbit 	MOSI = P2^6;
sbit 	MISO = P2^5;
sbit	IRQ	 = P2^4;

/**************************************************************************
 **                    公共接口声明
 *************************************************************************/
void	nRF24L01_RxPacket(uint8_t *rx_buf);
void	nRF24L01_TxPacket(uint8_t *tx_buf);


/**************************************************************************
 **                    私有函数声明
 *************************************************************************/
void		Delay(uint16_t s);
void		inerDelay_us(uint8_t n);
void		init_NRF24L01(void);
void		init_NRF24L012(void);
uint16_t	SPI_RW(uint16_t dat);
uint8_t		SPI_Read(uint8_t reg);
void		SetRX_Mode(void);
uint16_t	SPI_RW_Reg(uint8_t reg, uint8_t value);
uint16_t	SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t count);
uint16_t	SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t count);

#endif /* _NRF24L01_H_ */

