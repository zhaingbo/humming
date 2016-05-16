/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file ad.h
 *
 * AD转换驱动接口
 *
 * @author Zhanghb zhaingbo@foxmail.com
 *
 *************************************************************************/
#ifndef _AD_H_
#define _AD_H_

#include "../inc/stdtype.h"


/**************************************************************************
 **                    AD寄存器定义
 *************************************************************************/


#define ADC_POWER   0x80
#define ADC_FLAG    0x10
#define ADC_START   0x08
#define ADC_SPEEDLL 0x00
#define ADC_SPEEDL  0x20
#define ADC_SPEEDH  0x40
#define ADC_SPEEDHH 0x60


/**************************************************************************
 **                    公共接口定义
 *************************************************************************/
void		adc_init();
uint16_t	get_adc_result(uint8_t ch);

#endif /* _AD_H_ */

