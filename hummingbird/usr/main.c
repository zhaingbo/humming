/**************************************************************************
 *
 *   Copyright (c) 2016 www.bjfz.cc. All rights reserved.
 *
 * @file main.c
 *
 * 主程序
 *
 * @author Author email
 *
 *************************************************************************/
//套件购买地址：                      http://shop112966725.taobao.com
//                                                 STCunio
//四川 绵阳  西南科技大学 信息工程学院 电气13级 刘其民  技术支持QQ：1203933924
//本人只提供有限技术支持,截止时间：2015年3月1号,过时不候，请勿打扰。
//本人今年大二上期，由于技术有限提问时请不要超过本程序及本硬件的范围，什么捷联惯导技术，扩展卡尔曼，梯度下降滤波
//就不要问了，我也不懂，最近在自学自动控制原理，还是个小渣渣，连STM32都不会，做个四轴都还是只能苦逼地用51单片机
//本程序属于完全开源程序，允许用于进行二次开发，但是不能做细小改动后就声明自己拥有版权，
//对于抄袭后又声明自己有版权是自己开发的我只想说我QNMLGB。

//修改本程序应注意！
//如果将此程序优化，修改，增加功能，须开源至阿莫论坛 STC 1T单片机 版块
//使用本程序应遵循GNUGPL协议！

//本程序属于本硬件的最终版本，不会继续升级，但是会不定期的发布修复Bug后的程序。

// 特别鸣谢在研发过程中给予支持的朋友及团体：
//                                         西南科技大学 电气09级 刘畅
//                                         西南科技大学 嵌入式技术实验室 何科君
//                                         西南科技大学 航空航天学社暨空气动力学实验室

//*************************************2015年2月14日 著**************************************************************
//硬件参数：
//电池:1S/3.7V电池 推荐300-650mAh左右   500mah以上的电池推荐安装在背面
//电机/桨:720空心杯/59MM桨
//   特别声明：本程序测试的空心杯为淘宝店铺 深圳杰盛电机 的720空心杯 其他厂家的电机不适用于本程序，除非自行调整PID
//             本程序测试的螺旋桨为淘宝店铺 虎虎平价店 的59MM直径 1mm孔径螺旋桨 其他厂家的螺旋桨不适用于本程序，除非自行调整PID
//MCU IAP15W4K61S4@28.000MHZ  (B版！A版单片机绝对不适合！)
//特别说明，本程序目前只适合IAP系列的单片机，非IAP单片机使用须修改EEPROM读写地址
//陀螺仪加速度计：MPU-6050
//无线芯片:NRF24L01
//电机驱动MOS管:AO3400
//升压方案:BL8530
//3.3V稳压方案:ME6219C-33-M5G
//下载口保护:1K电阻
//机架尺寸:94mm*94mm

//设计失误的地方：
//MOS管保护用的肖特基放错了位置，不过完全不影响使用，这种小四轴不加肖特基保护都行。

//软件参数：
//姿态解算：四元数
//滤波：互补滤波（From 德国开源四轴）
//PID：串级PID 外环PI,内环PID

//数据定义说明：
//data 51单片机片内RAM最前面128字节RAM 用ACC读写，速度最快
//idata 片内RAM最前面256字节的RAM 包括data 用类似指针模式访问 适合用于指针操作
//pdata 外部扩展RAM的前256字节的RAM 不要用 会大姨妈！
//xdata 外部扩展RAM 用DPTR访问

#include <STC15W4K60S4.H>
#include <intrins.h>
#include <NRF24L01.H>
#include <MPU6050.H>
#include <math.h>
#include <STC15W4KPWM.H>
#include <Timer.h>
#include <EEPROM.h>
#include <USART.h>
#include <IMU.H>
//******************************************************************************************************
float XE = 0, YE = 0;            //角度人为修正，但是四轴漂移一般是硬件造成的，故不将此值写入EEPROM，这个只是应急使用，发现漂移应
//连至上位机检查电机轴是否发生弯曲，发现问题电机及时更换
float YM = 0;            //油门变化速度控制，不这样做的话快速变化油门时四轴会失速翻转并GG
int ich1 = 0, ich2 = 0, ich3 = 0, ich4 = 0, ich5 = 0, ich6 = 0;				 //无线串口/串口相关
int speed0 = 0, speed1 = 0, speed2 = 0, speed3 = 0, V = 0; //电机速度参数
int PWM0 = 0, PWM1 = 0, PWM2 = 0, PWM3 = 0; //加载至PWM模块的参数
int g_x = 0, g_y = 0, g_z = 0;    //陀螺仪矫正参数
char a_x = 0, a_y = 0;            //角度矫正参数
unsigned char TxBuf[20] = {0};
unsigned char RxBuf[20] = {0};
double PID_x = 0, PID_y = 0, PID_z = 0; //PID最终输出量
float FR1 = 0, FR2 = 0, FR3 = 0; //将char数据转存为float型

//*****************角度参数*************************************************
double Gyro_y = 0, Gyro_x = 0, Gyro_z = 0; //Y轴陀螺仪数据暂存
double Accel_x = 0, Accel_y = 0, Accel_z = 0;	 //X轴加速度值暂存
double Angle_ax = 0, Angle_ay = 0, Angle_az = 0; //由加速度计算的加速度(弧度制)
double Angle_gy = 0, Angle_gx = 0, Angle_gz = 0; //由角速度计算的角速率(角度制)
double AngleAx = 0, AngleAy = 0;          //三角函数解算出的欧拉角
double Angle = 0, Angley = 0;             //四元数解算出的欧拉角
double Anglezlate = 0;                    //Z轴相关
double Ax = 0, Ay = 0;                    //加入遥控器控制量后的角度
//****************姿态处理和PID*********************************************
float Out_PID_X = 0, Last_Angle_gx = 0; //外环PI输出量  上一次陀螺仪数据
float Out_XP = 35, Out_XI = 0.01, ERRORX_Out = 0; //外环P  外环I  外环误差积分
float In_XP = 0.4, In_XI = 0.01, In_XD = 9, ERRORX_In = 0; //内环P  内环I   内环D  内环误差积分

float Out_PID_Y = 0, Last_Angle_gy = 0;
float Out_YP = 35, Out_YI = 0.01, ERRORY_Out = 0;
float In_YP = 0.4, In_YI = 0.01, In_YD = 9, ERRORY_In = 0;

float ZP = 5.0, ZD = 4.0; //自旋控制的P D

int lastR0 = 0, ZT = 0; //上一次RxBuf[0]数据(RxBuf[0]数据在不断变动的)   状态标识
int i = 0;

void Angle_Calculate() interrupt 1
{
	/* if (YM<RxBuf[4]&&(RxBuf[4]-YM) <= 2) { */
	/* 	YM++; */
	/* 	YM++; */
	/* } else if (YM>RxBuf[4]&&(YM-RxBuf[4]) <= 2) {  //防止油门变化过快而失速 */
	/* 	YM--; */
	/* 	YM--; */
	/* } else { */
	/* 	YM = RxBuf[4]; */
	/* } */

	if (RxBuf[1] > FR1) {
		FR1 += 0.2;
	} else if (RxBuf[1] < FR1) {
		FR1 -= 0.2;
	}

	if (RxBuf[2] > FR2) {
		FR2 += 0.2;
	} else if (RxBuf[2] < FR2) {
		FR2 -= 0.2;
	}

	YM = (float)RxBuf[4] * 3 / 4;

	if (YM > 100) { //如果油门大于100 即开始起飞
		if (RxBuf[0] == lastR0) { //如果RxBuf[0]的数据没有收到 即矢联
			ZT++;  //状态标识+1

			if (ZT > 128) {
				ZT = 128;   //状态标识大于128即1秒没有收到数据，失控保护
			}
		} else {
			ZT = 0;
		}
	} else {
		ZT = 0;   //收到信号退出失控保护
	}

	if (ZT == 128) {
		YM = 101;    //触发失控保护 油门为1半少一点，缓慢下降，俯仰横滚方向舵归中
		RxBuf[1] = 128;
		RxBuf[2] = 128;
	}

	lastR0 = RxBuf[0];
	i++;

	if (i == 130) {
		i = 129;
	}

	Accel_y = GetData(ACCEL_YOUT_H);	//读取6050数据
	Accel_x = GetData(ACCEL_XOUT_H);
	Accel_z = GetData(ACCEL_ZOUT_H);
	Gyro_x	= GetData(GYRO_XOUT_H) - g_x;
	Gyro_y	= GetData(GYRO_YOUT_H) - g_y;
	Gyro_z	= GetData(GYRO_ZOUT_H) - g_z;

	Last_Angle_gx = Angle_gx; //储存上一次角速度数据
	Last_Angle_gy = Angle_gy;
	Angle_ax = (Accel_x)	/ 8192; //加速度处理
	Angle_az = (Accel_z)	/ 8192; //加速度量程 +-4g/S
	Angle_ay = (Accel_y)	/ 8192;	//转换关系8192LSB/g
	Angle_gx = (Gyro_x)		/ 65.5; //陀螺仪处理
	Angle_gy = (Gyro_y)		/ 65.5; //陀螺仪量程 +-500度/S
	Angle_gz = (Gyro_z)		/ 65.5; //转换关系65.5LSB/度
	//***********************************四元数解算**************************
	// 0.174533为PI/180 目的是将角度转弧度
	IMUupdate(deg2rad(Angle_gx), deg2rad(Angle_gy), deg2rad(Angle_gz),
			  Angle_ax, Angle_ay, Angle_az);

	//********************三角函数直接解算以供比较四元数解算精准度*****************
	// 后面的数字是180/PI 目的是弧度转角度
	AngleAx = rad2deg(atan(Angle_ax / sqrt(sq(Angle_ay) + sq(Angle_az))));
	AngleAy = rad2deg(atan(Angle_ay / sqrt(sq(Angle_ax) + sq(Angle_az))));

	//**************X轴指向*************************************************
	FR1 = ((fp32)RxBuf[1] - 128) / 7;//char类型转存为float以便除法运算
	Ax = Angle - FR1 - a_x; 		//角度控制量加载至角度

	if (YM > 20) {
		ERRORX_Out += Ax;			//外环积分(油门小于某个值时不积分)
	} else {
		ERRORX_Out = 0;				//油门小于定值时清除积分值
	}
	
	/* 积分限幅 */
	ERRORX_Out = constraint(ERRORX_Out, -500, 500);
	Out_PID_X = Ax*Out_XP + ERRORX_Out*Out_XI; //外环PI

	if (YM > 20) {
		ERRORX_In += (Angle_gy - Out_PID_X); //内环积分(油门小于某个值时不积分)
	} else {
		ERRORX_In = 0; //油门小于定值时清除积分值
	}

	/* 内环积分限幅 */
	ERRORX_In = constraint(ERRORX_In, -500, 500);
	/* 内环PID计算 */
	PID_x = (Angle_gy + Out_PID_X) * In_XP + 
		ERRORX_In * In_XI +
		(Angle_gy - Last_Angle_gy) * In_XD;
	/* PID限幅修正 */
	PID_x = constraint(PID_x, -1000, 1000);
	
	speed0 = 0 - PID_x;
	speed2 = 0 + PID_x;
	//**************Y轴指向*************************************************
	if (RxBuf[2] >= 143 || RxBuf[2] <= 113);
	else {
		RxBuf[2] = 128;
	}

	FR2 = ((float)RxBuf[2] - 128) / 7; //char类型转存为float以便除法运算
	Ay = Angley + FR2 - a_y; //角度控制量加载至角度

	if (YM > 20) {
		ERRORY_Out += Ay; //外环积分(油门小于某个值时不积分)
	} else {
		ERRORY_Out = 0; //油门小于定值时清除积分值
	}

	/* Y轴外环误差限幅 */
	ERRORY_Out = constraint(ERRORY_Out, -500, 500);
	Out_PID_Y = Ay * Out_YP + ERRORY_Out * Out_YI; //外环PI

	if (YM > 20) {
		ERRORY_In += (Angle_gx - Out_PID_Y); //内环积分(油门小于某个值时不积分)
	} else {
		ERRORY_In = 0; //油门小于定值时清除积分值
	}

	/* Y轴内环误差限幅 */
	ERRORY_In = constraint(ERRORY_In, -500, 500);
	/* 内环PID */
	PID_y = (Angle_gx + Out_PID_Y) * In_YP + 
		ERRORY_In * In_YI + 
		(Angle_gx - Last_Angle_gx) * In_YD;

	PID_y = constraint(PID_y, -1000, 1000);
	
	speed3 = 0 + PID_y;
	speed1 = 0 - PID_y; //加载到速度参数
	
	//**************Z轴指向(Z轴随便啦，自旋控制没必要上串级PID)*******************
	FR3 = ((float)RxBuf[3] - 128) * 1.5;
	Angle_gz -= FR3;
	PID_z = (Angle_gz) * ZP + (Angle_gz - Anglezlate) * ZD;
	Anglezlate = Angle_gz;
	speed0 = speed0 + PID_z;
	speed2 = speed2 + PID_z;
	speed1 = speed1 - PID_z;
	speed3 = speed3 - PID_z;

	//*****************串口及无线串口相关*************************************
	// 此处可发送6个数据至上位机，需要发送什么数据在此处修改即可
	ich1 = Ax;
	ich2 = Ay;
	ich3 = AngleAx;               
	ich4 = AngleAy;
	ich5 = 0;
	ich6 = 0;

	//**************将速度参数加载至PWM模块************************************
	//速度参数控制，防止超过PWM参数范围0-1000
	PWM0 = (1000 - YM * 4 + speed0);
	PWM0 = constraint(PWM0, 0, 1000);

	PWM1 = (1000 - YM * 4 + speed1);
	PWM1 = constraint(PWM1, 0, 1000);

	PWM2 = (1000 - YM * 4 + speed2);
	PWM2 = constraint(PWM0, 0, 1000);

	PWM3 = (1000 - YM * 4 + speed3);
	PWM3 = constraint(PWM3, 0, 1000);

	if (YM >= 10) {
		PWM(PWM1, PWM2, PWM0, PWM3);   //1203
	} else {
		PWM(1000, 1000, 1000, 1000);
	}
}

void main()
{
	PWMGO();//初始化PWM
	IAPRead();//读取陀螺仪静差
	InitMPU6050();//初始化MPU-6050
	Usart_Init();//初始化串口
	Time0_Init();//初始化定时器
	RxBuf[1] = 128;
	RxBuf[2] = 128;
	RxBuf[3] = 128;
	RxBuf[4] = 0;

	while (1) {
		Delay(500);
		nRF24L01_RxPacket(RxBuf);

		if (RxBuf[5] == 1 && i > 128) {
			IAP_Gyro();
			RxBuf[5] = 0;
			EA = 0;
			PWMCKS = 0x10;
			T2L = 0xEB;
			T2H = 0xFF;
			PWM(960, 960, 960, 960);
			Delay(60000); //校准完毕滴一声
			PWM(1000, 1000, 1000, 1000);
			PWMCKS = 0x00;
			EA = 1;
			i = 0;
		}

		if (RxBuf[6] == 1 && i > 128) {
			IAP_Angle();
			RxBuf[6] = 0;
			EA = 0;
			PWMCKS = 0x10;
			T2L = 0xEB;
			T2H = 0xFF;
			PWM(960, 960, 960, 960);
			Delay(60000);  //校准完毕滴一声
			PWM(1000, 1000, 1000, 1000);
			PWMCKS = 0x00;
			EA = 1;
			i = 0;
		}

		//Send(ich1,ich2,ich3,ich4,ich5,ich6);  //串口发送数据  如需连接上位机，须取消注释本句！！！注释本句是为了减小遥控延时
	}
}




