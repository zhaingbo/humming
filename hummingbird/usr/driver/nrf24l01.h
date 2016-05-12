/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file nrf24l01.h
 *
 * NRF24L01无线射频模块接收接口
 *
 * @author ZhangHb zhanghaibo@bjfz.cc
 *
 *************************************************************************/
#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include "stdtype.h"


sbit 	MISO = P1^1;
sbit 	MOSI = P1^0;
sbit	SCK	 = P6^3;
sbit	CE	 = P6^1;
sbit	CSN	 = P6^2;
sbit	IRQ	 = P4^7;

void	nRF24L01_TxPacket(uint8_t *tx_buf);
void	Delay(uint16_t s);
void	inerDelay_us(uint8_t n);
void	init_NRF24L01(void);
uint	SPI_RW(uint16_t uint8_t);
uint8_t	SPI_Read(uint8_t reg);
void	SetRX_Mode(void);
uint	SPI_RW_Reg(uint8_t reg, uint8_t value);
uint	SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);
uint	SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);
uint8_t	nRF24L01_RxPacket(uint8_t *rx_buf);

#endif /* _NRF24L01_H_ */



