#include <STC15F2K60S2.H>
#include <intrins.h>
#include <NRF24L01.H>
#include <AD.H>

//*********************************��ʼ��A/Dת��*************************************************
void adc_init()
{
	P1ASF = 0x1F;
	ADC_RES = 0;
	ADC_CONTR = ADC_POWER | ADC_SPEEDLL;
	Delay(10); //�ʵ���ʱ
}


//**********************************���A/Dת��������********************************************
uint getADCResult(char ch)
{
	ADC_CONTR = ADC_POWER | ADC_SPEEDLL | ch | ADC_START;
	Delay(5);

	while (!(ADC_CONTR & ADC_FLAG));//�ȴ�ת�����

	ADC_CONTR &= ~ADC_FLAG; //�ر�adc
	return ADC_RES;
}