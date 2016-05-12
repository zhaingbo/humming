/**************************************************************************
 *
 * Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file timer.c
 *
 * 定时器初始化
 *
 * @author Zhanghb
 *
 *************************************************************************/
#include "timer.h"
#include <stc/STC15W4K60S4.H>

/**
 * timer0_init()
 *
 * 8ms@28MHz 定时器0 16位12T自动重载
 */
void timer0_init()
{
	AUXR &= 0x7F;
	TMOD &= 0xF0;
	IE	  = 0x82;
	TL0	  = 0x15;
	TH0	  = 0xB7;
	TF0	  = 0;
	TR0	  = 1;
}


/**
 * timer1_init()
 *
 * 8ms@28MHz
 */
void timer1_init()
{
	AUXR |= 0x40;
	TMOD &= 0x0F;
	IE	  = 0x8a;
	TL1	  = 0x54;
	TH1	  = 0xF2;
	TF1	  = 0;
	TR1	  = 1;
}

/**
 * timer0_init()
 *
 * 8ms@28MHz 不用在意这个，装逼用的
 */
void timer2_init()
{
	AUXR |= 0x04;
	T2L	  = 0xEB;
	T2H	  = 0xFF;
	AUXR |= 0x10;
}
