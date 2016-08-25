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
#include "action.h"

int main()
{
	int deviceIndex = 0, baudnum = 207;
	int value1[3];
	float xyz[3], angle[3], angle_spe[3];
	char datasheet[] = "datasheet.txt";

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
	xyz[0] = 0.0; xyz[1] = 0.0; xyz[2] = -360.0;
	set_xyz(xyz[0], xyz[1], xyz[2]);
	cal_xyz_radian(); //逆运动学求解
	get_angle(angle); //获取计算的弧度
	//运动到目标位置
	angle_spe[0] = 10; angle_spe[1] = 10; angle_spe[2] = 10;
	move_joint_with_spe3(angle, angle_spe);
	
	//逆运动学控制
	action_control_demo(xyz, datasheet);
	
	return 1;
}

