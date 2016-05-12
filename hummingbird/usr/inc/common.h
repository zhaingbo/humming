/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file common.h
 *
 * 通用接口定义
 *
 * @author 
 *
 *************************************************************************/
#ifndef _COMMON_H_
#define _COMMON_H_

#include "stdtype.h"

#define PI			3.141592654f
#define DEG_TO_RAD	(PI/180)
#define	RAD_TO_DEG	(180/PI)


#define ENABLE_INTEGRAL_MIN_THROTLLE 20

/*  */
#define en_integral(throttle) \
	(throttle > ENABLE_INTEGRAL_MIN_THROTLLE)

/* 计算一个数的平方 */
#define sq(val) \
	((val)*(val))


/* 将角度转化成弧度 */
#define deg2rad(deg) \
	(deg * DEG_TO_RAD)

/* 将弧度转化成角度 */
#define rad2deg(rad) \
	(rad * RAD_TO_DEG)

/* 对参量进行上下限约束 */
float constraint(float val, float min, float max)
{
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}

	return val;
}


#endif /* _COMMON_H_ */


