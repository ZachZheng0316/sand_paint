#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "MX28AT.h"
#include "dmath.h"
#include "joint.h"
#include "kinesiology.h"
#include "action.h"

static float CurrentXYZ[3] = {0, 0, -340};
static float precision = 0.5;

//由坐标计算出对应的servoK
void xyz_to_servo_preK(float xyz[], int servoK[])
{
	float angle[3];

	set_xyz(xyz[0], xyz[1], xyz[2]); //设置坐标
	cal_xyz_radian();			 	 //逆运动学求解
	get_angle(angle);			 	 //获取计算的角度
	//关节角度对应的servo刻度
	servoK[0] = positionKFromAngle(1, angle[0]);
	servoK[1] = positionKFromAngle(2, angle[1]);
	servoK[2] = positionKFromAngle(3, angle[2]);
}

//由关节角速度计算出servo_spe
void angle_spe_to_servo_spe(float angle_spe[], int servo_spe[])
{
	servo_spe[0] = kFromRadianSpeed(PIFromAngle(angle_spe[0]));
	servo_spe[1] = kFromRadianSpeed(PIFromAngle(angle_spe[1]));
	servo_spe[2] = kFromRadianSpeed(PIFromAngle(angle_spe[2]));
}

//设置当前坐标
void set_current_xyz(float xyz[])
{
	CurrentXYZ[0] = xyz[0];
	CurrentXYZ[1] = xyz[1];
	CurrentXYZ[2] = xyz[2];
}

//创建数据表
void create_date_sheet(char *dataSheet)
{
	FILE *fp = NULL;
	
	fp = fopen(dataSheet, "w"); //创建新表
	if(NULL == fp) {
		printf("create %s failed\n", dataSheet);
		exit(-1);
	}
	
	fclose(fp);
	fp = NULL;
}

//往数据表中添加点运动数据
//如果t <= 0.0则以定速运动
void add_point_move_data(char *dataSheet, float xyz[], float t)
{
	FILE *fp;
	char flag;
	float angle_spe[3] = {20, 20, 20}, temp[3];
	int servoK[3], speK[3];
	
	fp = fopen(dataSheet, "a+"); //以追加的方式打开运动数据
	
	temp[0] = CurrentXYZ[0];
	temp[1] = CurrentXYZ[1];
	temp[2] = -360.0; 
	
	if(t <= 0.0) {
		//如果t<=0.0, 则按照固定速度运动
		//计算目标角度
		xyz_to_servo_preK(temp, servoK);
		//关节速度对应的servo速度刻度
		angle_spe_to_servo_spe(angle_spe, speK);
		//运动标志
		flag = 'w'; //wait的意思
		fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2],
		speK[0], speK[1], speK[2], flag);
		
		temp[0] = xyz[0];
		temp[1] = xyz[1];
		//计算目标角度
		xyz_to_servo_preK(temp, servoK);
		//关节速度对应的servo速度刻度
		angle_spe_to_servo_spe(angle_spe, speK);
		//运动标志
		flag = 'w'; //wait的意思
		fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2],
		speK[0], speK[1], speK[2], flag);
		
		//计算目标角度
		xyz_to_servo_preK(xyz, servoK);
		//关节速度对应的servo速度刻度
		angle_spe_to_servo_spe(angle_spe, speK);
		//运动标志
		flag = 'w'; //wait的意思
		fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2],
		speK[0], speK[1], speK[2], flag);
	}
	else {
	
	}
	
	//更新当前坐标
	set_current_xyz(xyz);
	//更新kinesiology中的当前位置坐标
	set_pre_position(xyz);
	
	fclose(fp);
}

//往数据表中添加直线运动数据
//如果t <= 0.0，则以定速运动
void add_line_move_data(char *dataSheet, float xyz[], float t)
{
	FILE *fp;
	char flag;
	float k, b;
	float x, y, x1, y1, x2, y2, ktemp;
	float angle_spe[3] = {10, 10, 10}, temp[3];
	int servoK[3], speK[3];
	
	fp = fopen(dataSheet, "a+"); //以追加的方式打开运动数据
	
	x1 = CurrentXYZ[0]; y1 = CurrentXYZ[1];
	x2 = xyz[0]; y2 = xyz[1];

	if(t <= 0.0) {
		//关节速度对应的servo速度刻度
		angle_spe_to_servo_spe(angle_spe, speK);
		//运动标志
		flag = 'r'; //wait的意思
		
		//以定速运动
		//计算直线方程
		if(x1 == x2) {
			//垂直于X轴的直线
			if(y2 <= y1) {//终点小于起始点
				for(y = y1; y >= y2; y -= precision) {
					temp[0] = x1;
					temp[1] = y;
					temp[2] = CurrentXYZ[2];
					
					//计算目标角度
					xyz_to_servo_preK(temp, servoK);
					fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2], speK[0], speK[1], speK[2], flag);
				}
			}
			else{//终点大于起始点
				for(y = y1; y <= y2; y += precision) {
					temp[0] = x1;
					temp[1] = y;
					temp[2] = CurrentXYZ[2];
					
					//计算目标角度
					xyz_to_servo_preK(temp, servoK);
					fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2], speK[0], speK[1], speK[2], flag);
				}
			}
		}
		else if(y1 == y2) {
			//垂直于Y轴的直线
			if(x1 <= x2) {
				for(x = x1; x <= x2; x += precision) {
					temp[0] = x;
					temp[1] = y1;
					temp[2] = CurrentXYZ[2];
					
					//计算目标角度
					xyz_to_servo_preK(temp, servoK);
					fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2], speK[0], speK[1], speK[2], flag);
				}
			}
			else{
				for(x = x1; x >= x2; x -= precision) {
					temp[0] = x;
					temp[1] = y1;
					temp[2] = CurrentXYZ[2];
					
					//计算目标角度
					xyz_to_servo_preK(temp, servoK);
					fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2], speK[0], speK[1], speK[2], flag);
				}
			}
		}
		else{
			//正常的直线
			k = (y2 - y1) / (x2 - x1);
			b = (x2*y1 - x1*y2) / (x2 - x1);
			ktemp = 1 / (pow(k, 2) + 1);
			ktemp = sqrt(ktemp);
			if(x2 > x1) {
				for(x = x1; x <= x2; x += precision * ktemp) {
					temp[0] = x;
					temp[1] = k * x + b;
					temp[2] = CurrentXYZ[2];
					
					//计算目标角度
					xyz_to_servo_preK(temp, servoK);
					fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2], speK[0], speK[1], speK[2], flag);
				}
			}
			else{
				for(x = x1; x >= x2; x -= precision*ktemp) {
					temp[0] = x;
					temp[1] = k * x + b;
					temp[2] = CurrentXYZ[2];
					
					//计算目标角度
					xyz_to_servo_preK(temp, servoK);
					fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2], speK[0], speK[1], speK[2], flag);
				}
			}
		}
					
		//计算目标角度
		xyz_to_servo_preK(xyz, servoK);
		fprintf(fp, "(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2], speK[0], speK[1], speK[2], flag);
	}
	else{
	}
	
	//更新当前坐标
	set_current_xyz(xyz);
	//更新kinesiology中的当前位置坐标
	set_pre_position(xyz);
	
	fclose(fp);
}

//执行数据表
void excute_data_sheet(char *dataSheet)
{
	FILE *fp;
	char flag;
	int servoK[3], speK[3], exten[3] = {5, 5, 5};
	
	fp = fopen(dataSheet, "r");
	while(!feof(fp)) {
		if(EOF == fscanf(fp, "(%d %d %d)(%d %d %d)(%c)\n", &servoK[0], &servoK[1], &servoK[2], &speK[0], &speK[1], &speK[2], &flag)) {
			printf("read data failed...\n");
			return ;
		}
	
		
		if(1 != set_many_servo_word(Moving_Speed, speK)) {
			printf("set Moving Speed failed\n");
			return;
		}
		if(1 != set_many_servo_word(Goal_Position, servoK)) {
			printf("set Goal Position failed\n");
			return;
		}
		
		if('w' == flag) 
			wait_for_many_servo_exten(exten);
		else 
			delay_us(50000); //延迟50ms
			
		printf("(%d %d %d)(%d %d %d)(%c)\n", servoK[0], servoK[1], servoK[2], speK[0], speK[1], speK[2], flag);
	}
}

//action control demo
void action_control_demo(float startPos[], char *datasheet)
{
	char choice;
	float xyz[3];

	printf("welcome to action control demo ...\n");
	set_current_xyz(startPos);	  //设置当前坐标
	create_date_sheet(datasheet); //创建运动数据表
	printf("pre-position (%f %f %f)\n", CurrentXYZ[0], CurrentXYZ[1], CurrentXYZ[2]);
	while(1) {
		printf("please chose mode:q, quit; p, point; L, Line; e, excute; c, clear...\n");
		if(0 == scanf("%c", &choice)) return;
		if('q' == choice) {
			printf("be quiting ...\n");
			relax_joint();
			break;
		}
		else if('p' == choice) {
			printf("welcome to Point ...\n");
			printf("please input point(xyz):\n");
			printf("pre-position (%f %f %f)\n", CurrentXYZ[0], CurrentXYZ[1], CurrentXYZ[2]);
			if(0 == scanf("%f %f %f", &xyz[0], &xyz[1], &xyz[2])) return;
			add_point_move_data(datasheet, xyz, 0.0);
		}
		else if('L' == choice) {
			printf("welcome to Line ...\n");
			printf("please input Line(xyz):\n");
			printf("pre-position (%f %f %f)\n", CurrentXYZ[0], CurrentXYZ[1], CurrentXYZ[2]);
			if(0 == scanf("%f %f %f", &xyz[0], &xyz[1], &xyz[2])) return;
			add_line_move_data(datasheet, xyz, 0.0);
		}
		else if('e' == choice) {
			printf("welcome to excute...\n");
			excute_data_sheet(datasheet);
		}
		else if('c' == choice) {
			printf("welcome to clear datasheet...\n");
			create_date_sheet(datasheet);
		}
	}
}
