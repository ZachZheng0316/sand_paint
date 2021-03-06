#ifndef _ACTION_HEADER
#define _ACTION_HEADER

#ifdef _cplusplus
extern "C" {
#endif

void xyz_to_servo_preK(float xyz[], int servoK[]);//由坐标计算出对应的servoK
void angle_spe_to_servo_spe(float angle_spe[], int servo_spe[]); //由关节角速度计算出servo_spe

void set_current_xyz(float xyz[]);				//设置当前坐标
void create_date_sheet(char *dataSheet);        //创建数据表
void add_point_move_data(char *dataSheet, float xyz[], float t); //往数据表中添加点运动数据
void add_line_move_data(char *dataSheet, float xyz[], float t);  //往数据表中添加直线运动数据

void excute_data_sheet(char *dataSheet);		//执行数据表

void action_control_demo(float startPos[], char *dataSheet); //action control demo

#ifdef _cplusplus
}
#endif

#endif
