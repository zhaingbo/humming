/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file IMU.H
 *
 * IMU 操作接口
 *
 *
 *
 * @author
 *
 **************************************************************************/
#ifndef _IMU_H_
#define _IMU_H_

extern double Angle, Angley;

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);

#endif /* _IMU_H_ */

