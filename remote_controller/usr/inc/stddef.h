/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file stddef.h
 *
 * 通用宏定义
 *
 * @author Zhanghb zhanghaibo@bjfz.cc
 *
 *************************************************************************/
#ifndef _STDDEF_H_
#define _STDDEF_H_

/* 定义bool型数据取值 */
#ifndef FALSE
#define FALSE	0 
#define TRUE	(!FALSE)
#endif /* FALSE */


/* 定义LED灯状态 */
#define OFF 1					/* LED关 */
#define ON	0					/* LED开 */

#endif /* _STDDEF_H_ */

