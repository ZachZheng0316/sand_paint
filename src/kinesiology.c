#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "kinesiology.h"
#include "dmath.h"
#include "joint.h"

static float xyz[3];       //执行端坐标
static float radian[3];    //关节弧度
static float Vxyz[3];      //执行端速度
static float joiSpeRad[3]; //关节弧速度
static float K, U, V;	   //必要的系数

static float Pre_Position[3];

//设置坐标
int set_xyz(float x, float y, float z)
{
	
	if((z <= -400.00) || (z >= -334.90)) {
		printf("input (z) error...\n");
		return -1;	
	}
	
	xyz[0] = x;
	xyz[1] = y;
	xyz[2] = z + H;
	
	return 1;
}

//设置初始位置
void set_pre_position(float pre_pos[])
{
	int i;
	
	for(i = 0; i < 3; i++)
		Pre_Position[i] = pre_pos[i];
}

//获取计算的角度
void get_angle(float angle[])
{
	int i;
	//把角度转化为弧度
	for(i = 0; i < 3; i++) 
		angle[i] = AngleFromPI(radian[i]);
}

//逆运动学求解
void cal_xyz_radian()
{
	float KUVtemp, result1, result2;
    float temp;

	//获得第1个系数组
	KUVtemp = pow(LA, 2)-pow(LB, 2);
    KUVtemp = KUVtemp -pow(xyz[0], 2)-pow(xyz[1], 2)-pow(xyz[2], 2);
    KUVtemp = KUVtemp -pow(RO-R_1O, 2);
    KUVtemp = KUVtemp + (RO-R_1O)*(sqrt(3.0)*xyz[0] + xyz[1]);
    temp = KUVtemp / LB;
    K = temp + 2.0 * xyz[2]; //计算K值

    temp = 2.0 * (RO-R_1O);
    temp = temp - sqrt(3.0) * xyz[0];
    temp = temp - xyz[1];
    U = -2.0 * temp; //计算U值

	temp = KUVtemp / LB;
    V = temp - 2.0 * xyz[2]; //计算V值

    temp = pow(U, 2);
    temp = temp - 4.0 * K * V;
    temp = sqrt(temp);
    result1 = -1.0 * U + temp;
    result1 = result1 / (2.0 * K);
    result2 = -1.0 * U - temp;
    result2 = result2 / (2.0 * K);

	radian[0] = atan(result2);
    radian[0] = 2.0 * radian[0];

	//获得第2个系数组
    KUVtemp = pow(LA, 2)-pow(LB, 2);
    KUVtemp = KUVtemp -pow(xyz[0], 2)-pow(xyz[1], 2)-pow(xyz[2], 2);
    KUVtemp = KUVtemp -pow(RO-R_1O, 2);
    KUVtemp = KUVtemp - (RO-R_1O)*(sqrt(3.0)*xyz[0] - xyz[1]);
    temp = KUVtemp / LB;
    K = temp + 2.0 * xyz[2]; //计算K值

    temp = 2.0 * (RO-R_1O);
    temp = temp + sqrt(3.0) * xyz[0];
    temp = temp - xyz[1];
    U = -2.0 * temp; //计算U值

    temp = KUVtemp / LB;
    V = temp - 2.0 * xyz[2]; //计算V值

    temp = pow(U, 2);
    temp = temp - 4.0 * K * V;
    temp = sqrt(temp);
    result1 = -1.0 * U + temp;
    result1 = result1 / (2.0 * K);
    result2 = -1.0 * U - temp;
    result2 = result2 / (2.0 * K);

	radian[1] = atan(result2);
    radian[1] = 2.0 * radian[1];

	//获得第3个系数组
    KUVtemp = pow(LA, 2)-pow(LB, 2);
    KUVtemp = KUVtemp -pow(xyz[0], 2)-pow(xyz[1], 2)-pow(xyz[2], 2);
    KUVtemp = KUVtemp -pow(RO-R_1O, 2);
    KUVtemp = KUVtemp - (RO-R_1O) * (2.0 * xyz[1]);
    temp = KUVtemp / (2 * LB);
    K = temp + xyz[2]; //计算K值

    temp = RO-R_1O;
    temp = temp + xyz[1];
    U = -2.0 * temp; //计算U值

    KUVtemp = pow(LA, 2)-pow(LB, 2);
    KUVtemp = KUVtemp -pow(xyz[0], 2)-pow(xyz[1], 2)-pow(xyz[2], 2);
    KUVtemp = KUVtemp -pow(RO-R_1O, 2);
    KUVtemp = KUVtemp - (RO-R_1O) * (2.0 * xyz[1]);
    temp = KUVtemp / (2 * LB);
    V = temp - xyz[2]; //计算K值

    temp = pow(U, 2);
    temp = temp - 4.0 * K * V;
    temp = sqrt(temp);
    result1 = -1.0 * U + temp;
    result1 = result1 / (2.0 * K);
    result2 = -1.0 * U - temp;
    result2 = result2 / (2.0 * K);

	radian[2] = atan(result2);
    radian[2] = 2.0 * radian[2];
}

void kinesiology_control_demo()
{
	char choice;
	float xyz1[3], angle[3];
	float angle_spe[3] = {5, 5, 5};
	
	printf("welcome to kinesiology control demo...\n");
	while(1){
		printf("welcome chose mode:q,quit; r,run\n");
		if(0 == scanf("%c", &choice)) return ;
        if('q' == choice) {
            printf("be quiting...\n");fflush(stdout);
            relax_joint();
            break;
        }
        if('r' == choice) {
        	printf("welcome to input (xyz)...\n");
        	printf("(1 2 3) pre_position (%f %f %f)\n", Pre_Position[0], Pre_Position[1], Pre_Position[2]);
			if(0 == scanf("%f %f %f", &xyz1[0], &xyz1[1], &xyz1[2])) return;
			if(1 == set_xyz(xyz1[0], xyz1[1], xyz1[2])) {;//设置坐标
				cal_xyz_radian();				   //计算弧度
				get_angle(angle);				   //获得角度
				//运动到目标位置
				move_joint_with_spe3(angle, angle_spe);
				//更新当前位置
				set_pre_position(xyz1);
			}
		}
	}
}

//逆速度求解
//求解条件:1、执行端坐标；2、执行端速度；3、关节角度
//设置执行端的速度
void set_Vxyz(float V[])
{
    Vxyz[0] = V[0];
    Vxyz[1] = V[1];
    Vxyz[2] = V[2];
}

//计算关节弧速度
void cal_joint_spe_radian()
{
    int i;
    float a, b, c, d;
    float eta[3] = {30, 150, 270};
    float Sx, Sy, Sz;
    float dr, temp1, temp2;
    
    dr = RO - R_1O;
    
    //把eta转化为弧度
    for(i = 0; i < 3; i++) 
        eta[i] = PIFromAngle(eta[i]);
        
    //根据当前条件计算当前关节的弧速度
    for(i = 0; i < 3; i++) {
        //计算a, b, c， d的值
        a = cos(eta[i]);
        b = sin(eta[i]);
        c = cos(PI/2 - radian[i]);
        d = sin(PI/2 - radian[i]);
    
        //计算Sx, Sy, Sz
        temp1 = dr + c * LB;
        temp2 = d * LB;
        Sx = xyz[0] - a * temp1;
        Sy = xyz[1] - b * temp1;
        Sz = xyz[2] + temp2;
        
        //计算分子
        temp1 = Sx * Vxyz[0];
        temp1 += Sy * Vxyz[1];
        temp1 += Sz * Vxyz[2];
        
        //计算分母
        temp2 = Sx * a * d * LB;
        temp2 += Sy * b * d * LB;
        temp2 += Sz * c * LB;
        
        //计算关节角度值
        joiSpeRad[i] = temp1 / temp2;
    }
}

//获取关节角速度
void get_joint_spe(float joint_spe[])
{
    int i;
    
    //关节弧速度转化为关节叫速度，并取绝对值
    for(i = 0; i < 3; i++){
        joint_spe[i] = AngleFromPI(joiSpeRad[i]);
        joint_spe[i] = fabs(joint_spe[i]);
    }
    
}

//综合逆运动学和你速度求解
//输入执行端坐标和线速度
int cal_xyz_jointSpe(float coor[], float V[])
{
    //去除笔尖高度
    if(0 == set_xyz(coor[0], coor[1], coor[2])) {
        printf("cal_xyz_jointSpe failed\n");
        return 0;
    }
    
    //设置执行端速度
    set_Vxyz(V);
    
    //计算关节角度
    cal_xyz_radian();
    
    //计算关节速度
    cal_joint_spe_radian();
    
    return 1;
}
