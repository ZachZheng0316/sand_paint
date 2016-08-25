#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include "MX28AT.h"
#include "dynamixel.h"
#include "dmath.h"
#include "dxl_hal.h"

//全局变量
static int servo_num = 3;             //舵机个数默认为3
static int servo_unit[3] = {1, 2, 3}; //舵机组
static int pre_goal_pos[3];           //舵机组的当前目标刻度值

//设置单个舵机单字节属性
//成功写入，返回1; 写入失败返回-1；
int set_one_servo_byte(int id, int address, int value)
{
    int commStatus, write_num = 2000;

	while(write_num-- >= 0) {
    	//设置属性值
    	dxl_write_byte(id, address, value);
    	//判断是否成功写入
    	commStatus = dxl_get_result();
    	if( commStatus == COMM_RXSUCCESS ) {
        	PrintErrorCode();
        	return 1;
    	}
    	else {
        	PrintCommStatus(commStatus);
        	delay_us(10000);
        	//return -1;
    	}
    }
    
    return -1;
}

//设置单个舵机多字节属性
//写入成功，返回1; 写入失败， 返回-1
int set_one_servo_word(int id, int address, int value)
{
    int commStatus, write_num = 2000;
    
    while(write_num-- >= 0) {
    	dxl_write_word(id, address, value);
    	commStatus = dxl_get_result();
    	if( commStatus == COMM_RXSUCCESS ) {
        	PrintErrorCode();
        	
        	if(Goal_Position == address)
    			pre_goal_pos[id - 1] = value;
    			
        	return 1;
    	}
    	else {
        	PrintCommStatus(commStatus);
        	delay_us(10000);
        	//return -1;
    	}
    }
    
    return -1;
}

//设置多个舵机单字节属性
//写入成功, 返回1; 写入失败, 返回-1。
int set_many_servo_byte(int address, int value[])
{
    int commStatus, i, write_num = 2000;
    
    while(write_num-- >= 0) {
		// Make syncwrite packet
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, address);
		dxl_set_txpacket_parameter(1, 1);
		for( i = 0; i < servo_num; i++ ) {
		    dxl_set_txpacket_parameter(2 + 2 * i, servo_unit[i]);
		    dxl_set_txpacket_parameter(2 + 2 * i + 1, value[i]);
		}
		dxl_set_txpacket_length((1 + 1) * servo_num + 4);
		dxl_txrx_packet();//printf("\n");
		commStatus = dxl_get_result();
		if( commStatus == COMM_RXSUCCESS ) {
		    PrintErrorCode();
		    return 1;
		}
		else{
		    PrintCommStatus(commStatus);
		    delay_us(10000);
		    //return -1;
		}
	}
	
	return -1;
}

//设置多个舵机多自己属性
//写入成功，返回1; 写入失败, 返回-1.
int set_many_servo_word(int address, int value[])
{
    int commStatus, i, write_num = 2000;
    
    while(write_num-- >= 0) {
		// Make syncwrite packet
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, address);
		dxl_set_txpacket_parameter(1, 2);
		for( i = 0; i < servo_num; i++ ) {
		    dxl_set_txpacket_parameter(2 + 3 * i, servo_unit[i]);
		    dxl_set_txpacket_parameter(2 + 3 * i + 1, dxl_get_lowbyte(value[i]));
		    dxl_set_txpacket_parameter(2 + 3 * i + 2, dxl_get_highbyte(value[i]));
		}
		dxl_set_txpacket_length((2 + 1) * servo_num + 4);
		dxl_txrx_packet(); //printf("\n");
		commStatus = dxl_get_result();
		if(commStatus == COMM_RXSUCCESS ) {
		    PrintErrorCode();
		    if(Goal_Position == address) {
		    	for(i = 0; i < servo_num; i++)
		    		pre_goal_pos[i] = value[i];
		    }
		    return 1;
		}
		else{
		    PrintCommStatus(commStatus);
		    delay_us(10000);
		    //dxl_hal_clear();
		}
	}
	
	return -1;
}

//获取单个舵机单字节属性
//读取成功，返回大于0的值;读入失败, 返回-1；
int get_one_servo_byte(int id, int address)
{
    int value, commStatus;
    
    while(1) {
        value = dxl_read_byte(id, address);
        commStatus = dxl_get_result();
        if(commStatus == COMM_RXSUCCESS) {
            PrintErrorCode();
            break;
            //return value;
        }
        else {
            PrintCommStatus(commStatus);
            //dxl_hal_clear();//清空数据包
            //return -1;
        }
    }

    return value;
}

//获取单个舵机多字节属性
//读取成功，返回大于0的值;读入失败, 返回-1；
int get_one_servo_word(int id, int address)
{
    int value, commStatus;
    
    while(1) {
        value = dxl_read_word(id, address);
        commStatus = dxl_get_result();
        if(COMM_RXSUCCESS == commStatus) {
            PrintErrorCode();
            break;
        }
        else {
            PrintCommStatus(commStatus);
            //dxl_hal_clear();//清空数据包
        }
    }

    return value;
}

//等待1个舵机运动停止
//执行成功返回1；执行失败返回-1
void wait_for_one_servo(int id)
{
    int moving;
    int value, read_num = 0;
    
    //读取目标刻度
    value = get_one_servo_word(id, Goal_Position);
    printf("MX28AT::wait_for_one_servo:%d servo's Goal_Position is %d\n", id, value);fflush(stdout);//显示读取的目标刻度
    
    do{
        moving = get_one_servo_byte(id, Moving); //获取运动结果
        if(0 == moving)
        	read_num++;
        else
        	read_num = 0;
    }while(read_num <= 20); //Goal_Position正在执行
    
    //读取舵机的当前刻度
    value = get_one_servo_word(id, Present_Position);
    printf("MX28AT::wait_for_one_servo:%d servo's Present_Position %d\n", id, value);
    
    //return 1;
}

//等待所有舵机停止运动
void wait_for_many_servo()
{
	int i;
	
	for(i = 0; i < servo_num; i++) 
		wait_for_one_servo(servo_unit[i]);
	
}						

//等待1个舵机运动到exten范围内
void wait_for_one_servo_exten(int id, int exten)
{
	int goal_pos, value, diff;
	
	goal_pos = get_one_servo_word(id, Goal_Position);
	
	//获得目标位置
	do{
		value = get_one_servo_word(id, Present_Position);
		diff = goal_pos - value;
	}while(abs(diff) >= exten);
	
	//return 1;
}	

//等待所有舵机运动到exten范围内
void wait_for_many_servo_exten(int exten[])
{
	int i;
	
	for(i = 0; i <servo_num; i++)
		wait_for_one_servo_exten(servo_unit[i], exten[i]);
}

//获得舵机当前刻度值
void get_servo_pre_goal_pos()
{
	int i;
	
	for(i = 0; i < servo_num; i++) 
		pre_goal_pos[i] = get_one_servo_word(servo_unit[i], Goal_Position);
}                      	

//单个舵机按照给定的时间从当前位置运动到目标位置
int move_to_goal_with_t1(int id, int goal_k, float t)
{
	float s, rpm; 
	int speK;
	
	if(t <= 0)
		return -1;
	
	s = (float)(goal_k - pre_goal_pos[id - 1]); //刻度之差
	rpm = (s * PositionUnit) / (6 * t);         //计算rpm
	speK = (int)(rpm / SpeedUnit + 0.3);        //计算速度刻度值
	
	//设置速度
	set_one_servo_word(id, Moving_Speed, speK);
	
	//设置目标值
	set_one_servo_word(id, Goal_Position, goal_k);
	
	//更新当前刻度之
	pre_goal_pos[id - 1] = goal_k;
	
	return 1;
}

//全部舵机按照给定的时间从当前位置运动到目标位置
int move_to_goal_with_t3(int goal_k[], float t[])
{
	float s, rpm;
	int speK, i;
	
	for(i = 0; i < servo_num; i++) {
		if(t[i] <= 0)
			return -1;
		
		s = (float)(goal_k[i] - pre_goal_pos[i]); //刻度之差
		rpm = (s * PositionUnit) / (6 * t[i]);         //计算rpm
		speK = (int)(rpm / SpeedUnit + 0.3);        //计算速度刻度值
	
		//设置速度
		set_one_servo_word(servo_unit[i], Moving_Speed, speK);
	
		//设置目标值
		set_one_servo_word(servo_unit[i], Goal_Position, goal_k[i]);
	
		//更新当前刻度之
		pre_goal_pos[i] = goal_k[i];
	}
	
	return 1;
}

//单个舵机根据给定的速度从当前位置运动到目标位置 
int move_to_goal_with_spe1(int id, int goal_k, int spe)
{
	if((goal_k <= 0) || (goal_k >= (int)PositionRange))
		return -1;
		
	if((spe <= 0) || (spe >= (int)SpeedKRange))	
		return -1;
	
	//设置速度
	set_one_servo_word(id, Moving_Speed, spe);
	
	//设置目标值
	set_one_servo_word(id, Goal_Position, goal_k);
	
	return 1;
}

//全部舵机根据给定的速度从当前位置运动到目标位置
int move_to_goal_with_spe3(int goal_k[], int spe[])
{
	int i;
	
	for(i = 0; i < servo_num; i++) {
		if((goal_k[i] <= 0) || (goal_k[i] >= (int)PositionRange))
			return -1;
		if((spe[i] <= 0) || (spe[i] >= (int)SpeedKRange))
			return -1;
	}
	
	//设置速度
	set_many_servo_word(Moving_Speed, spe);
	
	//设置目标值
	set_many_servo_word(Goal_Position, goal_k);
	
	return 1;
}

//控制舵机的demo
void servo_control_demo()
{
    char choice;
    int id[3], value1[3], value2[3];
    printf("welcome to servo control demo...\n");fflush(stdout);
    while(1) {
        printf("please chose mode:q:quit; s:sigle servo；m:three servo:\n");
        fflush(stdout);
        if(0 == scanf("%c", &choice)) return ;
        if('q' == choice) {
            printf("be quiting...\n");fflush(stdout);
            value1[0] = 0; value1[1] = 0; value1[2] = 0;
            set_many_servo_byte(Torque_Enable, value1);
            break;
        }
        else if('s' == choice) {
            printf("welcome to sigle servo control...\n");
            printf("please input servo ID:");fflush(stdout);
            if(0 == scanf("%d", &id[0])) return;fflush(stdin);
            value1[0] = get_one_servo_word(id[0], Present_Position);
            printf("(%d) servo pre-position (%d)\n", id[0], value1[0]);
            printf("please input goal-position and move speed:");fflush(stdout);
            if(0 == scanf("%d %d", &value1[0], &value2[0])) return;
            move_to_goal_with_spe1(id[0], value1[0], value2[0]);
        }
        else if('m' == choice) {
            printf("welcome to three servos control...\n");
            //获得当前位置
            id[0] = 1; id[1] = 2; id[2] = 3;
            value1[0] = get_one_servo_word(id[0], Present_Position); 
            value1[1] = get_one_servo_word(id[1], Present_Position);
            value1[2] = get_one_servo_word(id[2], Present_Position);
            printf("(1 2 3)servo pre-position (%d %d %d)\n", value1[0], value1[1], value1[2]);
            printf("please input goal-position (gp1, gp2, gp3):");fflush(stdout);
            if(0 == scanf("%d %d %d", &value1[0], &value1[1], &value1[2])) return;fflush(stdin);
            printf("please input move speed (sp1, sp2, sp3):");fflush(stdout);
            if(0 == scanf("%d %d %d", &value2[0], &value2[1], &value2[2])) return;fflush(stdin);
            move_to_goal_with_spe3(value1, value2);
        }
        else {
        }
    }
}
