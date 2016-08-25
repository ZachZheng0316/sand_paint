#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include "MX28AT.h"
#include "joint.h"
#include "dmath.h"
#include "kinesiology.h"
#include "action.h"
#include "serialCommuni.h"

//全局变量
static char datasheet[] = "datasheet.txt";

//
int add_motion_data(char *src_path, char *data_path); //添加动作数据 
void excute_motion_date(char *data_path);//执行动作数据

//框架
int initial_sys(int device, int baudNum); //系统初始化函数
void receive_instructor(unsigned char instruct[]); //获取指令
int execute_instructor(unsigned char instruct[], unsigned char beedback[]);  //解析并执行指令
void finish_instructor(unsigned char feedback[]);  //发送完成信号

int main()
{
	unsigned char packet[20] = {0, }, feedback[20] = {0, }; //接收指令和发送指令
	
	//初始化
	if(1 != initial_sys(5, 1200))
		return 0;
	else
		printf("sys init success...\n");
		
	do{
		receive_instructor(packet); 		  //接收指令

		execute_instructor(packet, feedback); //解析并执行指令
		
		finish_instructor(feedback); 		  //发送完成信号
	
	}while(1);
	
	
	return 1;
}


//系统初始化函数
int initial_sys(int device, int baudNum)
{
	int value1[3];
	int deviceIndex = 0, baudnum = 207;
	float xyz[3], angle[3], angle_spe[3];
	FILE *fp = NULL;

	//打开通信串口
    if(1 != serial_open(device, baudNum)) {
        printf("double_4-axis::initail_sys:serial_open failed\n");
        return -1;
    }
    
    ///////// Open USB2Dynamixel ////////////
	if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		printf( "Press Enter key to terminate...\n" );
		getchar();
		return -1;
	}
	else
		printf( "Succeed to open USB2Dynamixel!\n" );
		
	//属性初始化
	//PID初始化
	value1[0] = 46; value1[1] = 46; value1[2] = 46;
	if(1 != set_many_servo_byte(28, value1))
		return 0;
		
	value1[0] = 32; value1[1] = 32; value1[2] = 32;
	if(1 != set_many_servo_byte(27, value1))
		return 0;
	
	value1[0] = 30; value1[1] = 30; value1[2] = 30;
	if(1 != set_many_servo_byte(26, value1))
		return 0;
		
	//ENABLE
	value1[0] = 1; value1[1] = 1; value1[2] = 1;
	if(1 != set_many_servo_byte(24, value1))
		return 0;
	
	//position初始化
	xyz[0] = 0.0; xyz[1] = 0.0; xyz[2] = -355.0;
	set_xyz(xyz[0], xyz[1], xyz[2]);
	cal_xyz_radian(); //逆运动学求解
	get_angle(angle); //获取计算的弧度
	//运动到目标位置
	angle_spe[0] = 10; angle_spe[1] = 10; angle_spe[2] = 10;
	move_joint_with_spe3(angle, angle_spe);
	
	//创建数据
	printf("welcome to action control demo ...\n");
	create_date_sheet(datasheet); //创建运动数据表
	set_current_xyz(xyz);	  	  //设置当前坐标
	
	//产生logo数据
	fp = fopen("path.txt", "r");
	if(NULL == fp) {
		printf("read path failed...\n");
		return 0;
	}
	
	if(1 != add_motion_data("path.txt", datasheet))
		return 0;
	
	fclose(fp);
	fp = NULL;
	
	wait_for_many_servo();
	
	return 1;
}

//获取指令
void receive_instructor(unsigned char instruct[])
{
	int flag;

	while(1) {
        //1.获取字符
        flag = receiveMessage(instruct, 20);
		if(-1 == flag) {
			printf("receiving data failed!\r"); fflush(stdout);
		}
		else if(0 == flag) {
			printf("continue receiving data ...\r"); fflush(stdout);
		}
		else if(flag > 0) {
			printf("print data ..."); fflush(stdout);
			if(flag >= 19)
					instruct[19] = '\0';
				else
					instruct[flag] = '\0';
			printf("the receiving data (%s)\n", instruct); fflush(stdout);
			break;
		}
		else {
		}
	}
}

//解析并执行指令
int execute_instructor(unsigned char instruct[], unsigned char feedback[])
{
	int draw_flag = 0;
	char temp[20] = {0, };
	int num, p_num;

	//接受到主题信息
	if(('t' == instruct[1]) && (instruct[2] == 'h')) {
		strcpy((char *)feedback, "ook");
		draw_flag = 0;
	}
	else if(('d' == instruct[1]) && (instruct[2] == 't')) {
		//接受具体绘制的信息
		draw_flag = 1;
		strcpy(temp, (char *)&instruct[3]);
		num = atoi(temp);
		p_num = num % 100; //第几幅画
		sprintf((char *)feedback, "oover%d", p_num);
	}
	else if(('o' == instruct[1]) && (instruct[2] == 'v')) {
		//主题绘制完成
		strcpy((char *)feedback, "oother");
		draw_flag = 0;
	}
	else {
		strcpy((char *)feedback, "oother");
		draw_flag = 0;
	}
	
	//绘制图片
	if(1 == draw_flag) 
		excute_motion_date(datasheet);
		
	return 1;
}

//发送完成信号
void finish_instructor(unsigned char feedback[])
{
	int feedNum, realLen;

	feedNum = strlen((char *)feedback);
	
	while(1) {
        realLen = sendMessage(feedback, feedNum);
		if(realLen != feedNum) {
			printf("failed send data\r"); fflush(stdout);
		}
		else {
			printf("success send data:%s\n", feedback); fflush(stdout);
			break;
		}	
     }
}


//添加动作数据 
int add_motion_data(char *data_path, char *datasheet)
{
	FILE *fp;
	float xyz[3];
	char flag;
	
	
	fp = fopen(data_path, "r");
	if(NULL == fp) {
		printf("read path failed...\n");
		return 0;
	}
	
	while(!feof(fp)) {
		if(EOF == fscanf(fp, "(%f %f %f %c)\n",&xyz[0], &xyz[1], &xyz[2], &flag)) {
			printf("read data failed...\n");
			return 0;
		}
		
		xyz[0] *= 0.9;
		xyz[1] *= 0.9;
		xyz[2] += 5;
		
		if('p' == flag) {
			printf("welcome to Point ...\n");
			printf("please input point(xyz):\n");
			printf("pre-position (%f %f %f)\n", xyz[0], xyz[1], xyz[2]);
			add_point_move_data(datasheet, xyz, 0.0);
		}
		if('L' == flag) {
			printf("welcome to Line ...\n");
			printf("pre-position (%f %f %f)\n", xyz[0], xyz[1], xyz[2]);
			add_line_move_data(datasheet, xyz, 0.0);
		}
	}
	fclose(fp);
	fp = NULL;
	
	return 1;
}

//执行动作数据
void excute_motion_date(char *motiondata)
{
	//执行logo数据
	printf("welcome to excute...\n");
	excute_data_sheet(motiondata);
}
