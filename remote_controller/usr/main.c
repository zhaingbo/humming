//�׼������ַ                http://shop112966725.taobao.com
//                                        STCunio

//**********************************ң��������Rev1.3��ʽ��********************
//�������������߲���Ա���������κ������ˣ��ó�����Ǹ�Ӳ�������հ汾
//      ������������ж��ο�����������Ҫ��������İ汾�ϴ�����Ī��̳��
//      ����������Դ��������.
//
//      лл!!!
//          ����SWUST �����Զ���13�� LQM

//          ������������STC�����С�ձ���ң�������������ţ�
//��������Ϊ���ţ�����Ϊ���
//��������Ϊ����������Ϊ��ת
//                                 MCU����Ƶ��28MHZ������
#include "stdtype.h"

#include <rtx51tny.h>
#include <STC15F2K60S2.H>
#include <NRF24L01.H>
#include <AD.H>

/* RTX-Tiny������ */
#define TASK_IDLE		0
#define TASK_AD			0		/* AD����Ϊ0#���� */
#define TASK_nRF24L01	1		/* nRF24L01����Ϊ1#���� */
#define TASK_NUM		2		/* ������ */


sbit RLED = P0^5;
sbit GLED = P0^6;
sbit LKEY = P4^7;
sbit RKEY = P3^4;

volatile int16_t idata throttle_gain;
volatile int16_t idata roll_gain;
volatile int16_t idata pitch_gain;
volatile int16_t idata yaw_gain;

volatile int16_t idata throttle;
volatile int16_t idata roll;
volatile int16_t idata pitch;
volatile int16_t idata yaw;

volatile fp32 idata battery;

uint8_t idata TxBuf[20] = {0};
uint8_t idata RxBuf[20] = {0};
void IO_and_Init();

// ��ȡAD��������
void AD() _task_ TASK_AD
{
	IO_and_Init();		//��ʼ��I/O��
	os_create_task (1); //��������1

	while (1) {
		// ��ȡ4��ҡ��ͨ��ÿ��ͨ����8λ���ݣ�ȡֵ��Χ0-255
		throttle = get_adc_result(3);
		Delay(10);
		roll     = get_adc_result(2);
		Delay(10);
		pitch    = get_adc_result(1);
		Delay(10);
		yaw      = get_adc_result(0);
		Delay(10);
		// ��ص�ѹ���ͨ�� ����3.7V�����
		battery  = (get_adc_result(4) * 5.05 * 100) / 256;
		Delay(10);

		if (battery <= 370) {
			RLED = 1;
			GLED = 0;
		} else {
			RLED = 0;
			GLED = 1;
		}

		os_wait(K_IVL, 3, 0); //��ʱ3������
	}
}


//����(Pitch)��TxBuf[1]
//���(Roll)��TxBuf[2]
//ƫ��(Yaw)��TxBuf[3]
//����(Throttle)��TxBuf[4]
void NRF24L01()  _task_ TASK_nRF24L01
{
	while (1)
	{
		TxBuf[0]++;
		TxBuf[1] = 128;
		TxBuf[2] = 128;
		TxBuf[3] = 128;

		if (throttle < 20) {
            goto EXIT;
        } //�������������ʱң��������

		if (RKEY == 0) {
            TxBuf[5] = 1;
        } else {
            TxBuf[5] = 0;
        }

		if (LKEY == 0) {
            TxBuf[6] = 1;
        } else {
            TxBuf[6] = 0;
        }

        // ����TxBuf���������
		nRF24L01_TxPacket(TxBuf);
        // ��һ����ʱ�����ݷ������
		os_wait(K_IVL, 2, 0);
	}

EXIT:

	while (1)
	{
         // ���ϵ��¼�����ݶԲ������ݽ�����������֤ҡ����λʱ����Ϊ128
		if ((pitch - pitch_gain) >= 255) {
            TxBuf[1] = 255;
        } else if ((pitch - pitch_gain) <= 0) {
            TxBuf[1] = 0;
        } else {
            TxBuf[1] = pitch - pitch_gain;
        }

		if ((yaw - yaw_gain) >= 255) {
            TxBuf[3] = 255;
        } else if ((yaw - yaw_gain) <= 0) {
            TxBuf[3] = 0;
        } else {
            TxBuf[3] = yaw - yaw_gain;
        }

		if ((roll - roll_gain) >= 255) {
            TxBuf[2] = 255;
        } else if ((roll - roll_gain) <= 0) {
            TxBuf[2] = 0;
        } else {
            TxBuf[2] = roll - roll_gain;
        }

        // ����ͨ�����账��ֱ�ӷ���AD����8λ���ݼ��ɣ�����AD����ԭ��
        // ��֪��ȡ��AD�������ݲ�����Ϊ��Ҳ�����ܴ���255
		TxBuf[4] = throttle; 

		if (RKEY == 0) {
            TxBuf[5] = 1;
        } else {
            TxBuf[5] = 0;
        }

		if (LKEY == 0) {
            TxBuf[6] = 1;
        } else {
            TxBuf[6] = 0;
        }

		TxBuf[0]++;
		nRF24L01_TxPacket(TxBuf);
        // ��һ����ʱ�����ݷ������
        os_wait(K_IVL, 2, 0);
	}
}

void IO_and_Init()
{
	P1M0 = 0x00; //P1��Ϊ����ģʽ
	P1M1 = 0xFF;
	P0M0 = 0xFF; //����I/O������Ϊ׼˫��������ģʽ
	P0M1 = 0x00;
	P2M0 = 0x00;
	P2M1 = 0x00;
	P3M0 = 0x00;
	P3M1 = 0x00;
	P4M0 = 0x00;
	P4M1 = 0x00;
	P5M0 = 0xFF;
	P5M1 = 0x00;

	LKEY       = 1;             //���߰������I/O�ڵ�ƽ������Ϊ�͵�ƽ����
	RKEY       = 1;
	init_NRF24L01();            //��ʼ������ģ��
	adc_init();                 //��ʼ��AD���ģ��
	roll_gain  = get_adc_result(2) - 128; //��¼�ϵ�ʱҡ�˵�������Ϊ��λ��������Ϊҡ����λҪΪ128��256/2
	Delay(10);
	pitch_gain = get_adc_result(1) - 128;
	Delay(10);
	yaw_gain   = get_adc_result(0) - 128; //��¼���е���ĳ�ʼλ������
	Delay(10);
}
