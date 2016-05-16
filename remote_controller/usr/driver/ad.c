/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file ad.c
 *
 * AD转换驱动接口
 *
 * @author Zhanghb zhaingbo@foxmail.com
 *
 *************************************************************************/
#include "ad.h"
#include "nrf24l01.h"

/**************************************************************************
 **                    公共函数定义
 *************************************************************************/
/**
 * 初始化A/D转换
 */
void adc_init()
{
	P1ASF = 0x1F;
	ADC_RES = 0;
	ADC_CONTR = ADC_POWER | ADC_SPEEDLL;
	Delay(10); //适当延时
}

/**
 * 获得A/D转换的数据
 */
uint16_t get_adc_result(uint8_t ch)
{
	ADC_CONTR = ADC_POWER | ADC_SPEEDLL | ch | ADC_START;
	Delay(5);

	while (!(ADC_CONTR & ADC_FLAG));//等待转换完成

	ADC_CONTR &= ~ADC_FLAG; //关闭adc
	return ADC_RES;
}
