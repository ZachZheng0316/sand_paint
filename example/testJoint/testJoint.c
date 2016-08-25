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

int main()
{
	int deviceIndex = 0, baudnum = 207;
	int value1[3], value2[3];

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
	value1[0] = ID1POSK; value1[1] = ID2POSK; value1[2] = ID3POSK;
	value2[0] = 50; value2[1] = 50; value2[2] = 50;
	if(1 != move_to_goal_with_spe3(value1, value2))
		return 0;
	
	joint_control_demo();
	
	return 1;
}

