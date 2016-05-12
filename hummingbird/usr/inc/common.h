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

#define PI			3.141592654f
#define DEG_TO_RAD	(PI/180)
#define	RAD_TO_DEG	(180/PI)


inline float sq(float val)
{
	return (val*val);
}

/* 将角度转化成弧度 */
inline float deg2rad(float deg)
{
	return deg * DEG_TO_RAD;
}

/* 将弧度转化成角度 */
inline float rad2deg(float rad)
{
	return rad * RAD_TO_DEG;
}

/* 对参量进行上下线约束 */
inline float constraint(float val, float min, float max)
{
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}

	return val;
}


#endif /* _COMMON_H_ */


