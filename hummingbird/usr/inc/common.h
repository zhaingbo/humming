/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file common.h
 *
 * ͨ�ýӿڶ���
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

/* ����һ������ƽ�� */
#define sq(val) \
	((val)*(val))


/* ���Ƕ�ת���ɻ��� */
#define deg2rad(deg) \
	(deg * DEG_TO_RAD)

/* ������ת���ɽǶ� */
#define rad2deg(rad) \
	(rad * RAD_TO_DEG)

/* �Բ�������������Լ�� */
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


