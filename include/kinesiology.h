#ifndef _KINESIOLOGY_HEADER
#define _KINESIOLOGY_HEADER

#ifdef _cplusplus
extern "C" {
#endif

//硬件尺寸
#define RO   (110.0)
#define LB	 (98.00)
#define LA	 (330.0)
#define R_1O (28.00)
//#define H	 (60.10) //执行端的高度
#define H	 (55.70) //执行端的高度

//逆运动学
int set_xyz(float x, float y, float z); //设置坐标
void set_pre_position(float pre_pos[]);	//设置初始位置
void cal_xyz_radian();					//逆运动学求解
void get_angle(float angle[]);			//获取计算弧度

//逆速度求解
//求解条件:1、执行端坐标；2、执行端速度；3、关节角度
void set_Vxyz(float V[]);               //设置执行端的速度
void cal_joint_spe_radian();            //计算关节弧速度
void get_joint_spe(float joint_spe[]);  //获取关节角速度

//综合逆运动学和你速度求解
//输入执行端坐标和线速度
int cal_xyz_jointSpe(float coor[], float V[]);

//正运动学

//test demo
void kinesiology_control_demo();

#ifdef _cplusplus
}
#endif

#endif
