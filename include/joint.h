#ifndef _JOINT_HEADER
#define _JOINT_HEADER

#ifdef __cplusplus
extern "C" {
#endif

int set_one_joint_angle_spe(int joint_id, float angle_spe);   //设置单个舵机运动速度
int set_one_joint_goal_angle(int joint_id, float goal_angle); //设置单个舵机目标角度
int set_many_joint_angle_spe(float angle_spe[]);              //设置多个舵机多自己属性
int set_many_joint_goal_angle(float goal_angle[]);            //设置多个舵机多自己属性
float get_one_joint_angle_spe(int joint_id); 				  //获取单个关节目标速度
float get_one_joint_goal_angle(int joint_id); 				  //获取单个关节目标角度
void wait_for_one_joint(int joint_id); 						  //等待单个关节停止
void wait_for_many_joint();									  //等待多个关节停止
void wait_for_one_joint_exten(int joint_id, float exten);	  //等待一个舵机运动到exten角度范围内
void wait_or_many_joint_exten(float exten[]);				  //等待多个舵机运动到exten角度组的范围内
void relax_joint();											  //松掉关节的刚度
			

//控制关节运动函数
void move_joint_with_spe1(int joint_id, float goal_angle, float spe_angle); //以角速度spe_angle运动到目标角度goal_angle
void move_joint_with_spe3(float goal_angle[], float spe_angle[]);           //以角速度spe_angle[]运动到目标角度goal_angle[]
int move_joint_with_t1(int joint_id, float goal_angle, float t1); 			//从当前角度按规定的时间运动到目标角度
int move_joint_with_t3(float goal_angle[], float t3[]);		   				//从当前角度按规定的时间运动到目标角度


//控制关节的demo
void joint_control_demo(); //控制关节的demo

#ifdef __cplusplus
}
#endif

#endif
