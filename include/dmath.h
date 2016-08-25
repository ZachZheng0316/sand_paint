#ifndef _DMATH_HEADER
#define _DMATH_HEADER

#ifdef __cplusplus
extern "C" {
#endif

//MX28AT舵机属性值(12V)
#define AngleRange	  (float)(360.0)       //舵机运动的角度范围
#define PositionRange (float)(4095.0)	   //舵机运动的刻度范围
#define PositionUnit  (float)(0.087912088) //舵机每个刻度对应0.088°
#define SpeedRange    (float)(117.07)      //速度最大为114rpm
#define SpeedUnit     (float)(0.114437927) //每个刻度对应0.111rpm
#define SpeedKRange	  (float)(1023.0)      //舵机速度刻度范围

//常量
#define PI (float)(3.141592653) //常数PI的值

//舵机编号
#define MXID1 (unsigned char)(1)
#define MXID2 (unsigned char)(2)
#define MXID3 (unsigned char)(3)

//基准角度(°)
#define ID1Angle (float)(90.00)
#define ID2Angle (float)(90.00)
#define ID3Angle (float)(90.00)

//基准刻度
/*
#define ID1POSK  (787)
#define ID2POSK  (987)
#define ID3POSK  (1026)
*/
#define ID1POSK  (859)
#define ID2POSK  (973)
#define ID3POSK  (949)

//减速比
#define REDUCTION_RATE (float)(0.416666666)  //1:2.4

//移动极限(°)
#define MINANGLE  (float)(-30.0) //最小运动范围
#define MAXANGLE  (float)(120.0) //最大运动范围

//机械臂工作范围
#define SIZE    (float)(180.0) //机械臂的工作范围为SIZE * SIZE的区间

//工具函数
float AngleFromPI(float pi);         //弧度转角度
float PIFromAngle(float alpha);      //角度转弧度
float radianSpeedFromRPM(float rpm); //把rpm转化为舵机弧速度
float rpmFromRadianSpeed(float rad); //把舵机弧速度转化为rpm
float angleFromPositionK(unsigned char id, int positionK); //已知舵机当前刻度转换为关节当前角度
int positionKFromAngle(unsigned char arm_id, float angle); //已知关节当前角度转换为舵机当前刻度
int speedKFromRPM(float rpm);            			       //已知运动rpm转换为速度刻度
float rpmFromSpeedK(int speedK);         				   //已知速度刻度转换为rpm
int kFromRadianSpeed(float radianSpeed);       //把关节弧速度转化为对应舵机速度寄存器中的刻度

//把inkscape的基准坐标转化为机械臂的工作坐标
void coor_translate(float *originanal, float *result);

void delay(int sec);               //时间延迟函数
void delay_ms(float msec);         //毫秒延迟函数
void delay_us(unsigned long usec); //微妙延迟函数

#ifdef __cplusplus
}
#endif

#endif
