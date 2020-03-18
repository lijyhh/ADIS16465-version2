#ifndef __ATTITUDE_SOLUTION_H
#define   __ATTITUDE_SOLUTION_H

#include "stm32f4xx.h"

#define Ki 0.008f
#define Kp 0.3f
#define dtt 0.05f   //频率倒数，即积分周期

//定义角度结构体
struct angle 
{
	float pitch,roll,yaw;
};

void AHRS_update();










#endif   /*__ATTITUDE_SOLUTION_H*/
