/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file ad.c
 *
 * ADת�������ӿ�
 *
 * @author Zhanghb zhaingbo@foxmail.com
 *
 *************************************************************************/
#include "ad.h"
#include "nrf24l01.h"

/**************************************************************************
 **                    ������������
 *************************************************************************/
/**
 * ��ʼ��A/Dת��
 */
void adc_init()
{
	P1ASF = 0x1F;
	ADC_RES = 0;
	ADC_CONTR = ADC_POWER | ADC_SPEEDLL;
	Delay(10); //�ʵ���ʱ
}

/**
 * ���A/Dת��������
 */
uint16_t get_adc_result(uint8_t ch)
{
	ADC_CONTR = ADC_POWER | ADC_SPEEDLL | ch | ADC_START;
	Delay(5);

	while (!(ADC_CONTR & ADC_FLAG));//�ȴ�ת�����

	ADC_CONTR &= ~ADC_FLAG; //�ر�adc
	return ADC_RES;
}
