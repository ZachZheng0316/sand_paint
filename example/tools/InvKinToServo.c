#include <stdio.h>
#include "kinesiology.h"
#include "joint.h"
#include "MX28AT.h"
#include "dmath.h"

int main(void)
{
	float xyz[3], angle[3];
	
	printf("please input xyz:");
	if(0 == scanf("%f %f %f", &xyz[0], &xyz[1], &xyz[2])) return 0;
	set_xyz(xyz[0], xyz[1], xyz[2]);
	cal_xyz_radian();
	get_angle(angle); //计算出角度
	//由角度计算出舵机刻度
	printf("(%f %f %f) opposite ServoK(%d %d %d)\n", xyz[0], xyz[1], xyz[2], positionKFromAngle(1, angle[0]), positionKFromAngle(2, angle[1]), positionKFromAngle(3, angle[2]));

	return 1;
}
