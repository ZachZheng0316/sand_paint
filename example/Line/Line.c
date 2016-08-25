#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include "MX28AT.h"
#include "joint.h"
#include "dmath.h"
#include "kinesiology.h"
#include <unistd.h>

int main()
{
	int deviceIndex = 0, baudnum = 207;
	int value1[3];
	float xyz[3], angle[3], angle_spe[3], i, j;
	float current_xyz[3] = {0.0, 0.0, -330.0};
	FILE *fp;

	printf( "\n\nRead/Write example for Linux\n\n" );
	///////// Open USB2Dynamixel ////////////
	if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		printf( "Press Enter key to terminate...\n" );
		getchar();
		return 0;
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
	if(1 != set_xyz(current_xyz[0], current_xyz[1], current_xyz[2])) 
		return 0;
	cal_xyz_radian();			 //逆运动学求解
	get_angle(angle);			 //获取计算的弧度
	//运动到目标位置
	angle_spe[0] = 5; angle_spe[1] = 5; angle_spe[2] = 5;
	move_joint_with_spe3(angle, angle_spe);
	//更新当前坐标
	set_pre_position(current_xyz);
	
	//走直线
	fp = fopen("pathPoint.txt", "w");
	xyz[0] = 50.0; xyz[1] = 0.0; xyz[2] = -330.0;
	
	for(i = 0.0; i < 50.0; i += 1.0) {
		for(j = 0.0; j < 1.0; j += 0.1) {
			xyz[0] = i + j;
			xyz[1] = 0.0;
			xyz[2] = -330.0;
			set_xyz(xyz[0], xyz[1], xyz[2]);
			cal_xyz_radian();
			get_angle(angle);
			value1[0] = positionKFromAngle(1, angle[0]);
			value1[1] = positionKFromAngle(2, angle[1]);
			value1[2] = positionKFromAngle(3, angle[2]);
			fprintf(fp, "(%d %d %d)\n", value1[0], value1[1], value1[2]);
			printf("(%d %d %d)\n", value1[0], value1[1], value1[2]);
		}
	}
	fclose(fp);
	
	getchar();
	
	//执行动作
	fp = fopen("pathPoint.txt", "r");
	while(!feof(fp)) {
		if(EOF == fscanf(fp, "(%d %d %d)\n", &value1[0], &value1[1], &value1[2])) {
			printf("error\n");
			return -1;
		}
		printf("--(%d %d %d)\n", value1[0], value1[1], value1[2]);
		set_many_servo_word(Goal_Position, value1);
		usleep(10000);
	}
	
	fclose(fp);
	return 1;
}

