#ifndef __CONTROL_SYSTEM_H
#define __CONTROL_SYSTEM_H

#include "stm32f10x.h"


extern int L_coder;
extern int R_coder;

extern int Motor_A;
extern int Motor_B;


int Rs_To_CPR(float speed);

int Incremental_PI_A(int now, int target);

int Incremental_PI_B(int now, int target);

void Straight_Control(int forward);


#endif

