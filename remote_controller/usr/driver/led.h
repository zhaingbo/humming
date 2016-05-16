/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file led.h
 *
 * Status LED
 *
 * 状态指示灯
 *
 * @author ZhangHb zhaingbo@foxmail.com
 *
 *************************************************************************/
#ifndef _LED_H_
#define _LED_H_

#include "../inc/stdtype.h"

#ifdef _LED_C_
#  define LED_EXTERN
#else
#  define LED_EXTERN extern
#endif /* LED_EXTERN */


#define BAT_VOLTAGE_MIN 370		/* 电源最低电压 */

#define SETTING_T		0		/* 调整throttle */
#define SETTING_R		1		/* 调整roll */
#define SETTING_P		2		/* 调整pitch */
#define SETTING_Y		3		/* 调整yaw */


/* 声明遥控器电源状态指示灯 */
sbit rLED = P0^5;				/* red */
sbit gLED = P0^6;				/* green */
sbit bLED = P0^7;				/* blue */

/* 设置状态指示灯 */
sbit throttle_led	= P1^0;		/* 设置油门 */
sbit roll_led		= P1^1;		/* 设置横滚 */
sbit pitch_led		= P1^2;		/* 设置俯仰 */
sbit yaw_led		= P1^3;		/* 设置偏航 */

LED_EXTERN uint8_t g_setting_type;

void set_sys_status(fp32 bat_voltage);
void select_set_type(uint8_t type);


#endif /* _LED_H_ */

