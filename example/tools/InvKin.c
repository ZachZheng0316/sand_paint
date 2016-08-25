#include <stdio.h>
#include "kinesiology.h"

int main(void)
{
	float xyz[3], angle[3];
	
	printf("please input xyz:");
	if(0 == scanf("%f %f %f", &xyz[0], &xyz[1], &xyz[2])) return 0;
	set_xyz(xyz[0], xyz[1], xyz[2]);
	cal_xyz_radian();
	get_angle(angle);
	printf("(%f %f %f) opposite angle(%f %f %f)\n", xyz[0], xyz[1], xyz[2], angle[0], angle[1], angle[2]);

	return 1;
}
