#include "dmath.h"
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

//弧度转角度
/***********************************************
函数原型：
	float AngleFromPI(float pi)
参数：
	pi:弧度
返回值：
	返回角度值
函数意义：
	把弧度值转化为角度值
************************************************/
float AngleFromPI(float pi)
{
	float alpha;

	alpha = pi * 180 / PI;

	return alpha;
}

//角度转弧度
/***********************************************
函数原型：
	float PIFromAngle(float alpha)
参数：
	alpha:角度
返回值：
	返回弧度值
函数意义：
	把角度值转化为弧度值
************************************************/
float PIFromAngle(float alpha)
{
	float pi;

	pi = alpha * PI / 180;

	return pi;
}

//把rpm转化为角速度
/***********************************************
函数原型：
	float angleSpeedFromRPM(float rpm)
参数：
	rpm:转速
返回值：
	返回角速度
函数意义：
	把转速转化为角速度
************************************************/
float radianSpeedFromRPM(float rpm)
{
	float radianSpeed;

	radianSpeed = rpm * 2 * PI / 60;

	return radianSpeed;
}

/***********************************************
函数原型：
	float rpmFromRadianSpeed(float rad)
参数：
	rad:弧速度
返回值：
	返回rpm
函数意义：
	输入一个弧速度，把这个弧速度转化为rpm输出
************************************************/
float rpmFromRadianSpeed(float rad)
{
	float rpm;

	rpm = rad * 60 / (2 * PI);

	return rpm;
}

/***********************************************
函数原型：
	float angleFromPositionK(unsigned char id, int positionK)
参数：
  id:舵机编号
	positionK:舵机的当前位置刻度
返回值：
	返回对应的角度值;如果返回值是400°,则计算失败
函数意义：
	已知舵机当前刻度转换为关节当前角度
************************************************/
float angleFromPositionK(unsigned char id, int positionK)
{
	float IDAngleY = 60.0, temp = 0.0;

	if(MXID1 == id) {
		temp = (float)(positionK - ID1POSK);
		temp = temp * PositionUnit;
		temp = temp * REDUCTION_RATE;
		IDAngleY = ID1Angle - temp;
  	}
  	else if(MXID2 == id) {
		temp = (float)(positionK - ID2POSK);
		temp = temp * PositionUnit;
		temp = temp * REDUCTION_RATE;
		IDAngleY = ID2Angle - temp;
  	}
  	else if(MXID3 == id) {
		temp = (float)(positionK - ID3POSK);
		temp = temp * PositionUnit;
		temp = temp * REDUCTION_RATE;
		IDAngleY = ID3Angle - temp;
  	}
  	else {
		IDAngleY = -1.0;
  	}

	return IDAngleY;
}

/***********************************************
函数原型：
	float positionKFromangle(unsigned char arm_id, int angle)
参数：
	arm_id:关节编号
	angle:舵机的位置角度
返回值：
	返回相应的位置刻度; 如果返回值是-1, 则返回失败
函数意义：
	已知关节角度转换为与之对应的舵机刻度
************************************************/
int positionKFromAngle(unsigned char arm_id, float angle)
{
	int IDPOSKY, IDPOSK;
	float temp, rate;

	if (MXID1 == arm_id) {
		IDPOSK = ID1POSK;
		temp = ID1Angle - angle;
	}
	else if(MXID2 == arm_id) {
		IDPOSK = ID2POSK;
		temp = ID2Angle - angle;
	}
	else if(MXID3 == arm_id) {
		IDPOSK = ID3POSK;
		temp = ID3Angle - angle;
	}
	else {
		IDPOSKY = -1;
		return IDPOSKY;
	}

	rate = REDUCTION_RATE;
	temp = temp / rate;
	temp = temp / PositionUnit;
	IDPOSKY = (int)(IDPOSK + temp + 0.30);

	return IDPOSKY;
}

//已知转速rpm转换为速度刻度
/***********************************************
函数原型：
	float speedKFromRPM(float rpm)
参数：
	rpm:舵机的转速
返回值：
	返回相应的弧速度
函数意义：
	已知舵机的转速rpm，求出舵机速度刻度
************************************************/
int speedKFromRPM(float rpm)
{
	int speedK;

	speedK = (int)(rpm / SpeedUnit + 0.4);
	if(speedK <= 1) speedK = 5;
	if(speedK >= 1023) speedK = 1020;

	return (int)speedK;
}

//已知速度刻度转换为rpm
/***********************************************
函数原型：
	float rpmFromSpeedK(float speedK)
参数：
	speedK:舵机的速度刻度值
返回值：
	返回相应的舵机转速
函数意义：
	已知舵机速度刻度转化为rpm
************************************************/
float rpmFromSpeedK(int speedK)
{
	float rpm;

	rpm = speedK * SpeedUnit;
	if(rpm <= 0.0) rpm = 0.0;
	if(rpm >= SpeedRange) rpm = SpeedRange;

	return rpm;
}

/***********************************************
函数原型：
	int kFromRadianSpeed(float radianSpeed)
参数：
	radianSpeed:某个关节的转动弧速度
返回值：
	返回指定舵机速度寄存器中的刻度值; 返回-1，则计算失败
函数意义：
	把指定的关节转动时的弧速度转化为相应的舵机速度寄存器
中的刻度值
************************************************/
int kFromRadianSpeed(float radianSpeed)
{
	int kSpeed;
	float vec = 0.0, rate;

	rate = REDUCTION_RATE;
	vec = fabs(radianSpeed / rate); 		//速度绝对值化
	vec = rpmFromRadianSpeed(vec);		 	//把速度转化为rpm
	kSpeed = (int)(vec /SpeedUnit + 0.40);	//把rpm转化为舵机速度寄存器刻度
	if(kSpeed >= 1023) kSpeed = 1020;
	if(kSpeed <= 1) kSpeed = 5;

	return kSpeed;							//返回速度刻度值
}

//把inkscape的基准坐标转化为机械臂的工作坐标
void coor_translate(float *originanal, float *result)
{
	result[0] = originanal[0] - 0.5 * SIZE;
	result[1] = originanal[1] - 0.5 * SIZE;
}


//时间延迟函数
void delay(int sec)
{
	time_t start_time, cur_time;
	time(&start_time);
	do{
		time(&cur_time);
	}while((cur_time - start_time) < sec);
}

//毫秒延迟函数
void delay_ms(float msec)
{
	struct timeval start, end;
	float t1, t2;
	gettimeofday(&start, NULL);
	t1 = (float)(start.tv_sec * 1000 + start.tv_usec / 1000);
	do{
		gettimeofday(&end, NULL);
		t2 = (float)(end.tv_sec * 1000 + end.tv_usec / 1000);
	}while((t2 - t1) <= msec);
}

//微妙延迟函数
void delay_us(unsigned long usec)
{
	struct timeval start, end;
	unsigned long diff;
	gettimeofday(&start, NULL);
	do{
		gettimeofday(&end, NULL);
		diff = 1000000*(end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
	}while(diff <= usec);
}
