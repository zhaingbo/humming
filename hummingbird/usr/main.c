/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file main.c
 *
 * ������
 *
 * @author Author email
 *
 *************************************************************************/


//*************************************2015��2��14�� ��*********************
//Ӳ��������
//���:1S/3.7V��� �Ƽ�300-650mAh����   500mah���ϵĵ���Ƽ���װ�ڱ���
//���/��:720���ı�/59MM��
//   �ر���������������ԵĿ��ı�Ϊ�Ա����� ���ڽ�ʢ��� ��720���ı� �������ҵĵ���������ڱ����򣬳������е���PID
//             ��������Ե�������Ϊ�Ա����� ����ƽ�۵� ��59MMֱ�� 1mm�׾������� �������ҵ��������������ڱ����򣬳������е���PID
//MCU IAP15W4K61S4@28.000MHZ  (B�棡A�浥Ƭ�����Բ��ʺϣ�)
//�ر�˵����������Ŀǰֻ�ʺ�IAPϵ�еĵ�Ƭ������IAP��Ƭ��ʹ�����޸�EEPROM��д��ַ
//�����Ǽ��ٶȼƣ�MPU-6050
//����оƬ:NRF24L01
//�������MOS��:AO3400
//��ѹ����:BL8530
//3.3V��ѹ����:ME6219C-33-M5G
//���ؿڱ���:1K����
//���ܳߴ�:94mm*94mm

//���ʧ��ĵط���
//MOS�ܱ����õ�Ф�ػ��Ŵ���λ�ã�������ȫ��Ӱ��ʹ�ã�����С���᲻��Ф�ػ��������С�

//���������
//��̬���㣺��Ԫ��
//�˲��������˲���From �¹���Դ���ᣩ
//PID������PID �⻷PI,�ڻ�PID

//���ݶ���˵����
//data 51��Ƭ��Ƭ��RAM��ǰ��128�ֽ�RAM ��ACC��д���ٶ����
//idata Ƭ��RAM��ǰ��256�ֽڵ�RAM ����data ������ָ��ģʽ���� �ʺ�����ָ�����
//pdata �ⲿ��չRAM��ǰ256�ֽڵ�RAM ��Ҫ�� ������裡
//xdata �ⲿ��չRAM ��DPTR����

#include "timer.h"

#include "app/imu.h"
#include "app/stc15w4kpwm.h"

#include "driver/eeprom.h"
#include "driver/mpu6050.h"
#include "driver/nrf24l01.h"
#include "driver/usart.h"

#include "inc/common.h"

#include <math.h>

// ************************************************************************
float XE = 0, YE = 0;            //�Ƕ���Ϊ��������������Ư��һ����Ӳ����ɵģ��ʲ�����ֵд��EEPROM�����ֻ��Ӧ��ʹ�ã�����Ư��Ӧ
//������λ����������Ƿ���������������������ʱ����

// ���ű仯�ٶȿ��ƣ����������Ļ����ٱ仯����ʱ�����ʧ�ٷ�ת��GG
float throttle = 0;

// ���ߴ���/�������
uint8_t ich1 = 0, ich2 = 0, ich3 = 0, ich4 = 0, ich5 = 0, ich6 = 0;

// ����ٶȲ���
int speed0 = 0, speed1 = 0, speed2 = 0, speed3 = 0, V = 0;
int PWM0 = 0, PWM1 = 0, PWM2 = 0, PWM3 = 0; //������PWMģ��Ĳ���
int g_x = 0, g_y = 0, g_z = 0;    //�����ǽ�������
char a_x = 0, a_y = 0;            //�ǶȽ�������
/* unsigned char TxBuf[20] = {0}; */
uint8_t RxBuf[20] = {0};

double PID_x = 0, PID_y = 0, PID_z = 0; //PID���������
float FR1 = 0, FR2 = 0, FR3 = 0; //��char����ת��Ϊfloat��

//*****************�ǶȲ���*************************************************
double	Gyro_y	   = 0, Gyro_x = 0, Gyro_z = 0;	//Y�������������ݴ�
double	Accel_x	   = 0, Accel_y = 0, Accel_z = 0;	//X����ٶ�ֵ�ݴ�
double	Angle_ax   = 0, Angle_ay = 0, Angle_az = 0;	//�ɼ��ٶȼ���ļ��ٶ�(������)
double	Angle_gy   = 0, Angle_gx = 0, Angle_gz = 0;	//�ɽ��ٶȼ���Ľ�����(�Ƕ���)
double	AngleAx	   = 0, AngleAy = 0;	//���Ǻ����������ŷ����
double	Angle	   = 0, Angley = 0;	//��Ԫ���������ŷ����
double	Anglezlate = 0;         //Z�����
double	Ax		   = 0, Ay = 0; //����ң������������ĽǶ�

//****************��̬�����PID*********************************************
float Out_PID_X = 0, Last_Angle_gx = 0; //�⻷PI�����  ��һ������������
float Out_XP = 35, Out_XI = 0.01, ERRORX_Out = 0; //�⻷P  �⻷I  �⻷������
//�ڻ�P  �ڻ�I   �ڻ�D  �ڻ�������
float In_XP = 0.4, In_XI = 0.01, In_XD = 9, ERRORX_In = 0; 

float Out_PID_Y = 0, Last_Angle_gy = 0;
float Out_YP = 35, Out_YI = 0.01, ERRORY_Out = 0;
float In_YP = 0.4, In_YI = 0.01, In_YD = 9, ERRORY_In = 0;

float ZP = 5.0, ZD = 4.0; //�������Ƶ�P D

int16_t lastR0 = 0, ZT = 0; //��һ��RxBuf[0]����(RxBuf[0]�����ڲ��ϱ䶯��)   ״̬��ʶ
int16_t i = 0;

void Angle_Calculate() interrupt 1
{
	// ��ֹ���ű仯�����ʧ��
	if (throttle<RxBuf[4]&&(RxBuf[4]-throttle) <= 2) {
		throttle++;
		throttle++;
	} else if (throttle>RxBuf[4]&&(throttle-RxBuf[4]) <= 2) {
		throttle--;
		throttle--;
	} else {
		throttle = RxBuf[4];
	}

	if (RxBuf[1] > FR1) {
		FR1 += 0.2;
	} else if (RxBuf[1] < FR1) {
		FR1 -= 0.2;
	}

	if (RxBuf[2] > FR2) {
		FR2 += 0.2;
	} else if (RxBuf[2] < FR2) {
		FR2 -= 0.2;
	}

	throttle = (float)RxBuf[4] * 3 / 4;

	if (throttle > 100) { //������Ŵ���100 ����ʼ���
		if (RxBuf[0] == lastR0) { //���RxBuf[0]������û���յ� ��ʸ��
			ZT++;  //״̬��ʶ+1

			if (ZT > 128) {
				ZT = 128;   //״̬��ʶ����128��1��û���յ����ݣ�ʧ�ر���
			}
		} else {
			ZT = 0;
		}
	} else {
		ZT = 0;   //�յ��ź��˳�ʧ�ر���
	}

	if (ZT == 128) {
		throttle = 101;    //����ʧ�ر��� ����Ϊ1����һ�㣬�����½������������������
		RxBuf[1] = 128;
		RxBuf[2] = 128;
	}

	lastR0 = RxBuf[0];
	i++;

	if (i == 130) {
		i = 129;
	}

	Accel_y = GetData(ACCEL_YOUT_H);	//��ȡ6050����
	Accel_x = GetData(ACCEL_XOUT_H);
	Accel_z = GetData(ACCEL_ZOUT_H);
	Gyro_x	= GetData(GYRO_XOUT_H) - g_x;
	Gyro_y	= GetData(GYRO_YOUT_H) - g_y;
	Gyro_z	= GetData(GYRO_ZOUT_H) - g_z;

	Last_Angle_gx = Angle_gx; //������һ�ν��ٶ�����
	Last_Angle_gy = Angle_gy;
	Angle_ax = (Accel_x)	/ 8192; //���ٶȴ���
	Angle_az = (Accel_z)	/ 8192; //���ٶ����� +-4g/S
	Angle_ay = (Accel_y)	/ 8192;	//ת����ϵ8192LSB/g
	Angle_gx = (Gyro_x)		/ 65.5; //�����Ǵ���
	Angle_gy = (Gyro_y)		/ 65.5; //���������� +-500��/S
	Angle_gz = (Gyro_z)		/ 65.5; //ת����ϵ65.5LSB/��
	//***********************************��Ԫ������**************************
	// 0.174533ΪPI/180 Ŀ���ǽ��Ƕ�ת����
	IMUupdate(deg2rad(Angle_gx), deg2rad(Angle_gy), deg2rad(Angle_gz),
			  Angle_ax, Angle_ay, Angle_az);

	//********************���Ǻ���ֱ�ӽ����Թ��Ƚ���Ԫ�����㾫׼��*****************
	// �����������180/PI Ŀ���ǻ���ת�Ƕ�
	AngleAx = rad2deg(atan(Angle_ax / sqrt(sq(Angle_ay) + sq(Angle_az))));
	AngleAy = rad2deg(atan(Angle_ay / sqrt(sq(Angle_ax) + sq(Angle_az))));

	//**************X��ָ��*************************************************
	FR1 = ((fp32)RxBuf[1] - 128) / 7;//char����ת��Ϊfloat�Ա��������
	Ax = Angle - FR1 - a_x;			//�Ƕȿ������������Ƕ�

	if (throttle > 20) {
		ERRORX_Out += Ax;			//�⻷����(����С��ĳ��ֵʱ������)
	} else {
		ERRORX_Out = 0;				//����С�ڶ�ֵʱ�������ֵ
	}

	/* �����޷� */
	ERRORX_Out = constraint(ERRORX_Out, -500, 500);
	Out_PID_X = Ax*Out_XP + ERRORX_Out*Out_XI; //�⻷PI

	if (throttle > 20) {
		ERRORX_In += (Angle_gy - Out_PID_X); //�ڻ�����(����С��ĳ��ֵʱ������)
	} else {
		ERRORX_In = 0; //����С�ڶ�ֵʱ�������ֵ
	}

	/* �ڻ������޷� */
	ERRORX_In = constraint(ERRORX_In, -500, 500);
	/* �ڻ�PID���� */
	PID_x = (Angle_gy + Out_PID_X) * In_XP +
		ERRORX_In * In_XI +
		(Angle_gy - Last_Angle_gy) * In_XD;
	/* PID�޷����� */
	PID_x = constraint(PID_x, -1000, 1000);

	speed0 = 0 - PID_x;
	speed2 = 0 + PID_x;

	//**************Y��ָ��*************************************************
	/* ����Y���м�������Χ��113~143�������ȡֵ��ΧΪ0~255 */
	if (RxBuf[2] >= 113 && RxBuf[2] <= 143) {
		RxBuf[2] = 128;
	}

	// char����ת��Ϊfloat�Ա��������
	FR2 = ((fp32)RxBuf[2] - 128) / 7.0;
	Ay = Angley + FR2 - a_y; //�Ƕȿ������������Ƕ�


	if (throttle > 20) {
		ERRORY_Out += Ay; //�⻷����(����С��ĳ��ֵʱ������)
	} else {
		ERRORY_Out = 0; //����С�ڶ�ֵʱ�������ֵ
	}

	/* Y���⻷����޷� */
	ERRORY_Out = constraint(ERRORY_Out, -500, 500);
	Out_PID_Y = Ay * Out_YP + ERRORY_Out * Out_YI; //�⻷PI

	if (throttle > 20) {
		ERRORY_In += (Angle_gx - Out_PID_Y); //�ڻ�����(����С��ĳ��ֵʱ������)
	} else {
		ERRORY_In = 0; //����С�ڶ�ֵʱ�������ֵ
	}

	/* Y���ڻ�����޷� */
	ERRORY_In = constraint(ERRORY_In, -500, 500);
	/* �ڻ�PID */
	PID_y = (Angle_gx + Out_PID_Y) * In_YP +
		ERRORY_In * In_YI +
		(Angle_gx - Last_Angle_gx) * In_YD;

	PID_y = constraint(PID_y, -1000, 1000);

	speed3 = 0 + PID_y;
	speed1 = 0 - PID_y; //���ص��ٶȲ���

	//**************Z��ָ��(Z�����������������û��Ҫ�ϴ���PID)*******************
	FR3 = ((float)RxBuf[3] - 128) * 1.5;
	Angle_gz -= FR3;
	PID_z = (Angle_gz) * ZP + (Angle_gz - Anglezlate) * ZD;
	Anglezlate = Angle_gz;
	speed0 = speed0 + PID_z;
	speed2 = speed2 + PID_z;
	speed1 = speed1 - PID_z;
	speed3 = speed3 - PID_z;

	//*****************���ڼ����ߴ������*************************************
	// �˴��ɷ���6����������λ������Ҫ����ʲô�����ڴ˴��޸ļ���
	ich1 = Ax;
	ich2 = Ay;
	ich3 = AngleAx;
	ich4 = AngleAy;
	ich5 = 0;
	ich6 = 0;

	//**************���ٶȲ���������PWMģ��************************************
	//�ٶȲ������ƣ���ֹ����PWM������Χ0-1000
	PWM0 = (1000 - throttle * 4 + speed0);
	PWM0 = constraint(PWM0, 0, 1000);

	PWM1 = (1000 - throttle * 4 + speed1);
	PWM1 = constraint(PWM1, 0, 1000);

	PWM2 = (1000 - throttle * 4 + speed2);
	PWM2 = constraint(PWM0, 0, 1000);

	PWM3 = (1000 - throttle * 4 + speed3);
	PWM3 = constraint(PWM3, 0, 1000);

	if (throttle >= 10) {
		PWM(PWM1, PWM2, PWM0, PWM3);   //1203
	} else {
		PWM(1000, 1000, 1000, 1000);
	}
}

void main()
{
	PWMGO();//��ʼ��PWM
	IAPRead();//��ȡ�����Ǿ���
	InitMPU6050();//��ʼ��MPU-6050
	Usart_Init();//��ʼ������
	timer0_init();//��ʼ����ʱ��
	RxBuf[1] = 128;
	RxBuf[2] = 128;
	RxBuf[3] = 128;
	RxBuf[4] = 0;

	while (1) {
		Delay(500);
		nRF24L01_RxPacket(RxBuf);

		if (RxBuf[5] == 1 && i > 128) {
			IAP_Gyro();
			RxBuf[5] = 0;
			EA		 = 0;
			PWMCKS	 = 0x10;
			T2L		 = 0xEB;
			T2H		 = 0xFF;
			PWM(960, 960, 960, 960);
			Delay(60000);		//У׼��ϵ�һ��
			PWM(1000, 1000, 1000, 1000);
			PWMCKS	 = 0x00;
			EA		 = 1;
			i		 = 0;
		}

		if (RxBuf[6] == 1 && i > 128) {
			IAP_Angle();
			RxBuf[6] = 0;
			EA		 = 0;
			PWMCKS	 = 0x10;
			T2L		 = 0xEB;
			T2H		 = 0xFF;
			PWM(960, 960, 960, 960);
			Delay(60000);		//У׼��ϵ�һ��
			PWM(1000, 1000, 1000, 1000);
			PWMCKS	 = 0x00;
			EA		 = 1;
			i		 = 0;
		}

		//���ڷ�������  ����������λ������ȡ��ע�ͱ��䣡����ע�ͱ�����Ϊ�˼�Сң����ʱ
		//Send(ich1,ich2,ich3,ich4,ich5,ich6);
	}
}


