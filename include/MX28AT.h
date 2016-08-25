#ifndef _MX28AT_HEADER
#define _MX28AT_HEADER

#ifdef __cplusplus
extern "C" {
#endif

//舵机寄存器地址
#define ID (3)      			 //ID地址
#define Baud_Rate (4) 			 //波特率地址
#define MAX_Torque (14) 	     //最大波特率
#define Alarm_Shutdown (18) 	 //出错标识
#define Torque_Enable (24) 		 //扭矩开关
#define D_Gain (26) 			 //D增益
#define I_Gain (27) 			 //I增益
#define P_Gain (28) 			 //P增益
#define Goal_Position (30) 		 //目标刻度
#define Moving_Speed (32) 		 //运动速度
#define Torque_Limit (34) 	     //最大扭矩
#define Present_Position (36)    //当前位置
#define Present_Speed (38) 	     //当前速度
#define Present_Load (40) 	     //当前负载
#define Present_Voltage (42)     //当前电压
#define Present_Temperature (43) //当前温度
#define Moving (46) 			 //判断Goal_Position是否完成
#define Punch (48) 				 //Punch
#define Goal_Acceleration (73)   //目标加速度

int set_one_servo_byte(int id, int address, int value); //设置单个舵机单字节属性
int set_one_servo_word(int id, int address, int value); //设置单个舵机多字节属性
int set_many_servo_byte(int address, int value[]);		//设置多个舵机单字节属性
int set_many_servo_word(int address, int value[]);		//设置多个舵机多自己属性
int get_one_servo_byte(int id, int address);			//获取单个舵机单字节属性
int get_one_servo_word(int id, int address);			//获取单个舵机多字节属性
void wait_for_one_servo(int id); 					    //等待1个舵机运动停止
void wait_for_many_servo();								//等待所有舵机停止运动
void wait_for_one_servo_exten(int id, int exten);		//等待1个舵机运动到exten范围内
void wait_for_many_servo_exten(int exten[]);            //等待所有舵机运动到exten范围内

//获取多个舵机单字节属性
//获取多个舵机多字节属性

//控制舵机运动函数
void get_servo_pre_goal_pos();                       	 	//获得舵机当前刻度值
int move_to_goal_with_t1(int id, int goal_k, float t);  	//单个舵机按照给定的时间从当前位置运动到目标位置
int move_to_goal_with_t3(int goal_k[], float t[]);	        //全部舵机按照给定的时间从当前位置运动到目标位置
int move_to_goal_with_spe1(int id, int goal_k, int spe);  //单个舵机根据给定的速度从当前位置运动到目标位置
int move_to_goal_with_spe3(int goal_k[], int spe[]); 	    //全部舵机根据给定的速度从当前位置运动到目标位置

//控制舵机的测试程序
void servo_control_demo(); //控制舵机的demo

#ifdef __cplusplus
}
#endif

#endif
