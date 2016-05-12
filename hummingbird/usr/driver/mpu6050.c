/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file mpu6050.c
 *
 * Abstract
 *
 * Detail
 *
 * @author Author email
 *
 *************************************************************************/

#include "mpu6050.h"
#include "nrf24l01.h"

sbit    SCL	  =	P0 ^ 0;			//IICʱ�����Ŷ���    Rev8.0Ӳ��
sbit    SDA	  =	P4 ^ 6;			//IIC�������Ŷ���

//sbit    SCL =	P2^5;			//IICʱ�����Ŷ���      Rev7.0Ӳ��
//sbit    SDA =	P2^6;			//IIC�������Ŷ���

/* // ��ʼ��MPU6050 */
/* void	InitMPU6050(); */
/* void	Delay2us(); */
/* void	I2C_Start(); */
/* void	I2C_Stop(); */
/* bit		I2C_RecvACK(); */
/* void	I2C_SendByte(uint8_t dat); */
/* uint8_t I2C_RecvByte(); */
/* void	I2C_ReadPage(); */
/* void	I2C_WritePage(); */
/* uint8_t Single_ReadI2C(uint8_t REG_Address);	//��ȡI2C���� */
/* void	Single_WriteI2C(uint8_t REG_Address, uint8_t REG_data);	//��I2Cд������ */

// IICʱ������ʱ���ã�����μ���оƬ�������ֲ�
// 6050�Ƽ���С1.3us ���ǻ�����⣬������ʱʵ��1.9us����
void Delay2us()
{
	uint8_t i;
	i = 11;
	while (i--);
}

//**************************************
//I2C��ʼ�ź�
//**************************************
void I2C_Start()
{
	SDA = 1;                    //����������
	SCL = 1;                    //����ʱ����
	Delay2us();                 //��ʱ
	SDA = 0;                    //�����½���
	Delay2us();                 //��ʱ
	SCL = 0;                    //����ʱ����
}

//**************************************
//I2Cֹͣ�ź�
//**************************************
void I2C_Stop()
{
	SDA = 0;                    //����������
	SCL = 1;                    //����ʱ����
	Delay2us();                 //��ʱ
	SDA = 1;                    //����������
	Delay2us();                 //��ʱ
}

//**************************************
//I2C����Ӧ���ź�
//**************************************
bit I2C_RecvACK()
{
	SCL = 1;                    //����ʱ����
	Delay2us();                 //��ʱ
	CY	= SDA;                  //��Ӧ���ź�
	SCL = 0;                    //����ʱ����
	Delay2us();                 //��ʱ
	return CY;
}

//**************************************
//��I2C���߷���һ���ֽ�����
//**************************************
void I2C_SendByte(uint8_t dat)
{
	uint8_t i;

	for (i = 0; i < 8; i++) {   //8λ������
		dat <<= 1;              //�Ƴ����ݵ����λ
		SDA = CY;               //�����ݿ�
		SCL = 1;                //����ʱ����
		Delay2us();             //��ʱ
		SCL = 0;                //����ʱ����
		Delay2us();             //��ʱ
	}

	I2C_RecvACK();
}

//**************************************
//��I2C���߽���һ���ֽ�����
//**************************************
uint8_t I2C_RecvByte()
{
	uint8_t i;
	uint8_t dat = 0;
	SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,

	for (i = 0; i < 8; i++) {   //8λ������
		dat <<= 1;
		SCL = 1;                //����ʱ����
		Delay2us();             //��ʱ
		dat |= SDA;             //������
		SCL = 0;                //����ʱ����
		Delay2us();             //��ʱ
	}

	return dat;
}

//**************************************
//��I2C�豸д��һ���ֽ�����
//**************************************
void Single_WriteI2C(uint8_t REG_Address, uint8_t REG_data)
{
	I2C_Start();                  //��ʼ�ź�
	I2C_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
	I2C_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
	I2C_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
	I2C_Stop();                   //����ֹͣ�ź�
}

//**************************************
//��I2C�豸��ȡһ���ֽ�����
//**************************************
uint8_t Single_ReadI2C(uint8_t REG_Address)
{
	uint8_t REG_data;
	I2C_Start();                   //��ʼ�ź�
	I2C_SendByte(SlaveAddress);    //�����豸��ַ+д�ź�
	I2C_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
	I2C_Start();                   //��ʼ�ź�
	I2C_SendByte(SlaveAddress + 1); //�����豸��ַ+���ź�
	REG_data = I2C_RecvByte();     //�����Ĵ�������
	SDA = 1;                    //дӦ���ź�
	SCL = 1;                    //����ʱ����
	Delay2us();                 //��ʱ
	SCL = 0;                    //����ʱ����
	Delay2us();                 //��ʱ
	I2C_Stop();                    //ֹͣ�ź�
	return REG_data;
}

//**************************************
//��ʼ��MPU6050
//**************************************
void InitMPU6050()
{
	Single_WriteI2C(PWR_MGMT_1, 0x00);	//�������״̬
	Single_WriteI2C(SMPLRT_DIV, 0x07);  //������125hz
	Single_WriteI2C(CONFIG, 0x04);      //21HZ�˲� ��ʱA8.5ms G8.3ms  �˴�ȡֵӦ�൱ע�⣬��ʱ��ϵͳ�������Ϊ��
	Single_WriteI2C(GYRO_CONFIG, 0x08); //������500��/S 65.5LSB/g
	Single_WriteI2C(ACCEL_CONFIG, 0x08);//���ٶ�+-4g  8192LSB/g
}

//**************************************
//�ϳ�����
//**************************************
uint16_t GetData(uint8_t REG_Address)
{
	uint8_t H, L;
	H =	Single_ReadI2C(REG_Address);
	L =	Single_ReadI2C(REG_Address + 1);
	return (H << 8) + L;			//�ϳ�����
}

