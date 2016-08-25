#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <math.h>
#include "MX28AT.h"
#include "joint.h"
#include "dmath.h"
#include "kinesiology.h"
#include "action.h"
/*
#define HEIGHT1 (-340.00)
#define HEIGHT2 (-343.00)
*/
#define HEIGHT0 (-370.00)
#define HEIGHT1 (-390.00)
#define HEIGHT2 (-398.00)

#define TIME (0.035) //50ms

float distance(float *ps, float *pd); 	//计算两点之间的距离
int tran_coor_to_k(float *xyz, int *k); //坐标转化为刻度值

int init_sys(); //系统初始化

int translate_ink(char *sourcefile, char *resultfile);		//把inkscape数据进行坐标转换
int get_tra_coor(char *tra1, char *tra2);             		//把粗略的轨迹数据转化为轨迹坐标
int get_tra_vector(char *tra_coor, char *tra_vec);    		//由轨迹坐标生成轨迹速度
int get_tra_k(char *tra_coor, char *tra_vec, char *tra_k); 	//获得轨迹刻度
int excute_tra_k(char *tra_k);         					   	//执行轨迹刻度

int main()
{
	char inkfile[] = "testRange.txt", coorfile[] = "source.txt";
	char tra_coor[] = "tracoor.txt", tra_vec[] = "travec.txt", tra_k[] = "trak.txt";
 	
	//系统初始化
	if(0 == init_sys()) {
		printf("init_sys failed\n");
		return 0;
	}
	
	//转换ink坐标
	if(0 == translate_ink(inkfile, coorfile)){ 
		// 坐标转换
		printf("translate_ink failed out\n");
		return 0;
	}
	
	//获取轨迹坐标
	if(0 == get_tra_coor(coorfile, tra_coor)){
		printf("get tra coor failed\n");
		return 0;
	}
	
	//获取轨迹速度
	if(0 == get_tra_vector(tra_coor, tra_vec)) {
	    printf("get tra vector failed\n");
	    return 0;
	}
	
	//获取轨迹刻度
	if(0 == get_tra_k(tra_coor, tra_vec, tra_k)) {
		printf("get tra k failed\n");
		return 0;
	}
	
	printf("start ...\n");
	getchar();
	
	//执行轨迹刻度
	if(0 == excute_tra_k(tra_k)) {
		printf("excute_tra_k failed out\n");
		return 0;
	}
	
	return 1;
}

//计算两点之间的距离
float distance(float *ps, float *pd)
{
	float x, y, z;
	
	x = pow(pd[0] - ps[0], 2);
	y = pow(pd[1] - ps[1], 2);
	z = pow(pd[2] - ps[2], 2);
	
	return sqrt(x + y + z);
}

//把inkscape数据进行坐标转换
int translate_ink(char *sourcefile, char *resultfile)
{
	char flag;
	float ps[2], pd[3], cur[3] = {0.0, 0.0, HEIGHT1};
	FILE *fp_source, *fp_result;
	
	fp_source = fopen(sourcefile, "r");
	fp_result = fopen(resultfile, "w");
	
	pd[2] = HEIGHT2;
	while(!feof(fp_source)) {
		if(EOF == fscanf(fp_source, "%c (%f %f)\n", &flag, &ps[0], &ps[1]))
			return 0;
		coor_translate(ps, pd);
		
		if('m' == flag) {
			//运动到当前位置-350处
			cur[2] = HEIGHT1;
			fprintf(fp_result, "%c (%f %f %f)\n", flag, cur[0], cur[1], cur[2]);
			//运动到m的-350处
			pd[2] = HEIGHT1;
			fprintf(fp_result, "%c (%f %f %f)\n", flag, pd[0], pd[1], pd[2]);
			//运动到m的-362.0处
			pd[2] = HEIGHT2;
			fprintf(fp_result, "%c (%f %f %f)\n", flag, pd[0], pd[1], pd[2]);
		}
		else
			fprintf(fp_result, "%c (%f %f %f)\n", flag, pd[0], pd[1], pd[2]);
			
		cur[0] = pd[0]; cur[1] = pd[1];
	}
	
	//回到圆点
	cur[2] = HEIGHT1;
	fprintf(fp_result, "%c (%f %f %f)\n", flag, cur[0], cur[1], cur[2]);
	cur[0] = 0.0; cur[1] = 0.0;
	fprintf(fp_result, "%c (%f %f %f)\n", flag, cur[0], cur[1], cur[2]);
	
	fclose(fp_source);
	fclose(fp_result);
	
	return 1;
}

//系统初始化
int init_sys()
{
	int deviceIndex = 0, baudnum = 207;
	int value1[3];
	float xyz[3], angle[3], angle_spe[3];

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
	value1[0] = 40; value1[1] = 40; value1[2] = 40; //46
	if(1 != set_many_servo_byte(28, value1))
		return 0;

	value1[0] = 30; value1[1] = 30; value1[2] = 30; //32
	if(1 != set_many_servo_byte(27, value1))
		return 0;

	value1[0] = 30; value1[1] = 30; value1[2] = 30; //30
	if(1 != set_many_servo_byte(26, value1))
		return 0;

	//ENABLE
	value1[0] = 1; value1[1] = 1; value1[2] = 1;
	if(1 != set_many_servo_byte(24, value1))
		return 0;

	//position初始化
	printf("initial height ...\n");
	xyz[0] = 0.0; xyz[1] = 0.0; xyz[2] = -370.0;
	set_xyz(xyz[0], xyz[1], xyz[2]);
	cal_xyz_radian(); //逆运动学求解
	get_angle(angle); //获取计算的弧度
	//运动到目标位置
	angle_spe[0] = 10; angle_spe[1] = 10; angle_spe[2] = 10;
	move_joint_with_spe3(angle, angle_spe);
	
	printf("start draw ...\n");
	getchar();
	xyz[0] = 0.0; xyz[1] = 0.0; xyz[2] = HEIGHT1;
	set_xyz(xyz[0], xyz[1], xyz[2]);
	cal_xyz_radian(); //逆运动学求解
	get_angle(angle); //获取计算的弧度
	//运动到目标位置
	angle_spe[0] = 10; angle_spe[1] = 10; angle_spe[2] = 10;
	move_joint_with_spe3(angle, angle_spe);

	return 1;
}

//把粗略的轨迹数据转化为轨迹坐标
int get_tra_coor(char *tra1, char *tra2)
{
	float ps[3], pd[3], px[3]; //坐标对
	float dis, unit, t; //线性函数控制变量
	char flag[2];
	FILE  *fp1, *fp2;
	
	fp1 = fopen(tra1, "r");
	fp2 = fopen(tra2, "w");
	
	//搜索到第一个起始点
	while(1) {
		if(EOF == fscanf(fp1, "%c (%f %f %f)\n", &flag[0], &ps[0], &ps[1], &ps[2]))
			return 0;
		if('m' == flag[0]) {
			break;
		}
	}
	
	while(!feof(fp1)) {
		if(EOF == fscanf(fp1, "%c (%f %f %f)\n", &flag[1], &pd[0], &pd[1], &pd[2]))
			return 0;
		
		dis = distance(ps, pd); //两点之间的距离
		if(dis > 0.0) {
			unit = 1 / dis;
		
			for(t = 0; t < 1.0; t += unit) {
				px[0] = t * (pd[0] - ps[0]) + ps[0];
				px[1] = t * (pd[1] - ps[1]) + ps[1];
				px[2] = t * (pd[2] - ps[2]) + ps[2];
			
				fprintf(fp2, "%c (%f %f %f)\n",flag[0], px[0], px[1], px[2]);
			}
		}
		
		//取起始点
		ps[0] = pd[0];
		ps[1] = pd[1];
		ps[2] = pd[2];
		flag[0] = flag[1];
	}

	fprintf(fp2, "%c (%f %f %f)\n", flag[1], pd[0], pd[1], pd[2]);
			
	fclose(fp1);
	fclose(fp2);
	
	return 1;
}

//由轨迹坐标生成轨迹速度
int get_tra_vector(char *tra_coor, char *tra_vec)
{
    float ps[3], pd[3]; //坐标对
    float dis, vec[3];
    char flag[2];
    FILE  *fp1, *fp2;
    
    fp1 = fopen(tra_coor, "r");
	fp2 = fopen(tra_vec, "w");
	
	//搜索到第一个起始点
	if(EOF == fscanf(fp1, "%c (%f %f %f)\n", &flag[0], &ps[0], &ps[1], &ps[2]))
	    return 0;
	while(!feof(fp1)) {
	    if(EOF == fscanf(fp1, "%c (%f %f %f)\n", &flag[1], &pd[0], &pd[1], &pd[2]))
			return 0;
		
		dis = distance(ps, pd); //两点之间的距离
		if(dis <= 0.0)
		    dis = 1.0;
		vec[0] = (pd[0] - ps[0]) / dis;
		vec[1] = (pd[1] - ps[1]) / dis;
		vec[2] = (pd[2] - ps[2]) / dis;
		vec[0] /= TIME;
		vec[1] /= TIME;
		vec[2] /= TIME;
		
		fprintf(fp2, "%c (%f %f %f)\n", flag[0], vec[0], vec[1], vec[2]);
		
		//更新ps
		flag[0] = flag[1];
		ps[0] = pd[0];
		ps[1] = pd[1];
		ps[2] = pd[2];
	}
    
	fclose(fp1);
	fclose(fp2);
	
	return 1;
}

//坐标转化为刻度值
int tran_coor_to_k(float *xyz, int *k)
{
	float angle[3];	
	
	//设置坐标
	if(0 == set_xyz(xyz[0], xyz[1], xyz[2])) {
		printf("tran_coor_to_k failed\n");
		return 0;
	}
	
	//逆运动学求解
	cal_xyz_radian();
	
	//获得计算角度
	get_angle(angle);
	
	//角度转化为舵机刻度
	k[0] = positionKFromAngle(1, angle[0]);
	k[1] = positionKFromAngle(2, angle[1]);
	k[2] = positionKFromAngle(3, angle[2]);
	
	return 1;
}

//获得轨迹刻度(包括速度刻度和目标刻度)
int get_tra_k(char *tra_coor, char *tra_vec, char *tra_k)
{
	FILE *fp1, *fp2, *fp3;
	float xyz1[3], xyz[3], joint_spe[3];
	int vecK[3], posK[3];
	char flag;
	int i;
	
	fp1 = fopen(tra_coor, "r");
	fp2 = fopen(tra_vec, "r");
	fp3 = fopen(tra_k, "w");
	
	//取第一个坐标点
	if(EOF == fscanf(fp1, "%c (%f %f %f)\n", &flag, &xyz1[0], &xyz1[1], &xyz1[2])) {
			printf("get tra k failed 1 in\n");
			return 0;
	}
	if(0 == tran_coor_to_k(xyz1, posK)) {
	    printf("get tra k failed 1.1 in\n");
	    return 0;
	}
	vecK[0] = 10; vecK[1] = 10; vecK[2] = 10;
	fprintf(fp3, "%c (%d %d %d) (%d %d %d)\n",flag, vecK[0], vecK[1], vecK[2], posK[0], posK[1], posK[2]);
	
	while(!feof(fp2)) {
		if(EOF == fscanf(fp1, "%c (%f %f %f)\n", &flag, &xyz[0], &xyz[1], &xyz[2])) {
			printf("get tra k failed 2 in\n");
			return 0;
		}
		
		if(EOF == fscanf(fp2, "%c (%f %f %f)\n", &flag, &joint_spe[0], &joint_spe[1], &joint_spe[2])) {
			printf("get tra k failed 3 in\n");
			return 0;
		}
		
		//获取坐标
		if(0 == tran_coor_to_k(xyz, posK)) {
	        printf("get tra k failed  in\n");
	        return 0;
	    }
		
		//获取关节角速度
		if(0 == cal_xyz_jointSpe(xyz1, joint_spe)) {
		    printf("get tra k failed 4 in \n");
		    return 0;
		}
		get_joint_spe(joint_spe);  //获取关节角速度
		for(i = 0; i < 3; i++) {
		    joint_spe[i] = PIFromAngle(joint_spe[i]);
	        vecK[i] = kFromRadianSpeed(joint_spe[i]);
		}
		fprintf(fp3, "%c (%d %d %d) (%d %d %d)\n", flag, vecK[0], vecK[1], vecK[2], posK[0], posK[1], posK[2]);
		
		//更新
		xyz1[0] = xyz[0];
		xyz1[1] = xyz[1];
		xyz1[2] = xyz[2];
	}
	
	fclose(fp1);
	fclose(fp2);
	fclose(fp3);
	
	return 1;
}


//执行轨迹刻度
int excute_tra_k(char *tra_k)
{
	FILE *fp;
	char flag;
	int posK[3], vecK[3];
	
	fp = fopen(tra_k, "r");
	while(!feof(fp)) {
		if(EOF == fscanf(fp, "%c (%d %d %d) (%d %d %d)\n" ,&flag, &vecK[0], &vecK[1], &vecK[2], &posK[0], &posK[1], &posK[2])) {
			printf("excute_tra_k failed in\n");
			return 0;
		}
		
		//执行数据
		if(1 != set_many_servo_word(Moving_Speed, vecK)) {
			printf("set Moving Speed failed\n");
			return 0;
		}
		if(1 != set_many_servo_word(Goal_Position, posK)) {
			printf("set Goal Position failed\n");
			return 0;
		}
		
		//延时控制
		delay_us(40000); //延迟50ms
	}
	fclose(fp);
	
	return 1;
}
