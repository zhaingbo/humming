/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file led.c
 *
 * Status LED
 *
 * 状态指示灯操作
 *
 * @author Zhanghb zhaingbo@foxmail.com
 *
 *************************************************************************/
#define _LED_C_
#include "led.h"

static void sys_normal();
static void sys_error();

static void clear_all_setting();

void set_sys_status(fp32 bat_voltage)
{
	if (bat_voltage > BAT_VOLTAGE_MIN) {
		sys_normal();
	} else {
		sys_error();
	}
}

void select_set_type(uint8_t type)
{
	switch (type) {
	case SETTING_T:
		throttle_led = ON;
		roll_led	 = OFF;
		pitch_led	 = OFF;
		yaw_led		 = OFF;
		break;
	case SETTING_R:
		throttle_led = OFF;
		roll_led	 = ON;
		pitch_led	 = OFF;
		yaw_led		 = OFF;
		break;
	case SETTING_P:
		throttle_led = OFF;
		roll_led	 = OFF;
		pitch_led	 = ON;
		yaw_led		 = OFF;
		break;
	case SETTING_Y:
		throttle_led = OFF;
		roll_led	 = OFF;
		pitch_led	 = OFF;
		yaw_led		 = ON;
		break;
	}
}

void sys_normal()
{
	rLED = OFF;
	gLED = ON;
	bLED = OFF;
}

void sys_error()
{
	rLED = ON;
	gLED = OFF;
	bLED = OFF;
}

void clear_all_setting()
{
	throttle_led = OFF;
	roll_led	 = OFF;
	pitch_led	 = OFF;
	yaw_led		 = OFF;
}

