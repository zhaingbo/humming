/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file nrf24l01.c
 *
 * NRF24L01������Ƶģ����սӿ�
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

// ���ص�ַ
uint8_t const TX_ADDRESS[TX_ADR_WIDTH] = { 
	0x34, 0x43, 0x10, 0x10, 0x01
};
// ���յ�ַ
uint8_t const RX_ADDRESS[RX_ADR_WIDTH] = {
	0x34, 0x43, 0x10, 0x10, 0x01
};

/**************************************************************************
 ** NRF24L01�Ĵ���ָ��
 **************************************************************************/
#define READ_REG        0x00  	// ���Ĵ���ָ��
#define WRITE_REG       0x20 	// д�Ĵ���ָ��
#define RD_RX_PLOAD     0x61  	// ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0  	// д��������ָ��
#define FLUSH_TX        0xE1 	// ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2  	// ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3  	// �����ظ�װ������ָ��
#define NOP             0xFF  	// ����

/**************************************************************************
 ** SPI(nRF24L01)�Ĵ�����ַ
 **************************************************************************/
#define CONFIG2         0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define STATUS          0x07  // ״̬�Ĵ���
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
#define RX_PW_P1        0x12  // ����Ƶ��0�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��0�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��0�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��0�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������

/**************************************************************************
 ** ״̬��־λ
 *************************************************************************/
uint16_t 	bdata	sta;			//״̬��־
sbit			RX_DR	= sta^6;
sbit			TX_DS	= sta^5;
sbit			MAX_RT	= sta^4;
sbit			TX_FULL = sta^0;

/**************************************************************************
 ** ���в�����������
 *************************************************************************/
/**
 * ������ nRF24L01_RxPacket()
 * ���ܣ����ݶ�ȡ�����rx_buf���ջ�������
 * @param unsigned char* rx_buf
 */
uint8_t nRF24L01_RxPacket(uint8_t *rx_buf)
{
	uint8_t revale = 0;
	// ��ȡ״̬�Ĵ������ж����ݽ���״��
	sta = SPI_Read(STATUS);

	// �ж��Ƿ���յ�����
	if (RX_DR) {
		CE	   = 0;											//SPIʹ��
		// read receive payload from RX_FIFO buffer
		SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);
		revale = 1;											//��ȡ������ɱ�־
	}

	// ���յ����ݺ�RX_DR,TX_DS,MAX_PT���ø�Ϊ1��ͨ��д1������жϱ�־
	SPI_RW_Reg(WRITE_REG + STATUS, sta);
	CE = 1;
	return revale;
}

/**************************************************************************
 *������void nRF24L01_TxPacket(uint8_t *tx_buf)
 *���ܣ����� tx_buf������
 *************************************************************************/
void nRF24L01_TxPacket(uint8_t *tx_buf)
{
	SPI_RW_Reg(WRITE_REG + STATUS, 0xff);
	SPI_RW_Reg(0xE1, 0xff);
	CE = 0;
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG + CONFIG2, 0x0e);
	CE = 1;
	inerDelay_us(10);   //CE�ߵ�ƽ����10us���ܽ��뷢��ģʽ
}



//*****************************************����ʱ**************************
void Delay(uint16_t s)
{
	volatile uint16_t i;
	for (i = 0; i < s; i++);
	for (i = 0; i < s; i++);
}


/**************************************************************************
/*��ʱ����
/*************************************************************************/
void inerDelay_us(uint8_t n)
{
	for (; n > 0; n--);
}

//*************************************************************************
/*NRF24L01��ʼ��
//************************************************************************/
void init_NRF24L01(void)
{
	CE	= 0;					// chip enable
	CSN = 1;					// Spi  disable
	SCK = 0;					//

	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);		// д���ص�ַ
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);	// д���ն˵�ַ
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);								//  Ƶ��0�Զ�	ACKӦ������
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);							//  ������յ�ַֻ��Ƶ��0�������Ҫ��Ƶ�����Բο�Page21
	SPI_RW_Reg(WRITE_REG + RF_CH, 0x6e);								//   �����ŵ�����Ϊ2.4GHZ���շ�����һ��
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);					//���ý������ݳ��ȣ���������Ϊ32�ֽ�
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x27);								//���÷�������Ϊ1MB/S�����书��Ϊ���ֵ+7dB��������X2401L���ţ�ʵ��+21dbm���
}

/**
 *������uint SPI_RW(uint ����)
 *uint8_t��NRF24L01��SPIдʱ��
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
 *������uint8_t SPI_Read(uint8_t reg)
 *���ܣ�NRF24L01��SPIʱ��
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
 *���ܣ�NRF24L01��д�Ĵ�������
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
 *������uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
 *����: ���ڶ����ݣ�reg��Ϊ�Ĵ�����ַ��pBuf��Ϊ���������ݵ�ַ��uchars���������ݵĸ���
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
 *������uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
 *����: ����д���ݣ�Ϊ�Ĵ�����ַ��pBuf��Ϊ��д�����ݵ�ַ��uchars��д�����ݵĸ���
 */
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t	status;
	uint8_t		i;

	CSN = 0;            //SPIʹ��
	status = SPI_RW(reg);

	for (i = 0; i < len; i++) {
		SPI_RW(*pBuf++);
	}

	CSN = 1;			//�ر�SPI
	return (status);	//
}

/**
 *������void SetRX_Mode(void)
 *���ܣ����ݽ�������
 */
void SetRX_Mode(void)
{
	CE = 0;
	// IRQ�շ�����ж���Ӧ��16λCRC	��������
	SPI_RW_Reg(WRITE_REG + CONFIG2, 0x0F);
	CE = 1;
}

