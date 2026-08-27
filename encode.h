#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

#define ENCODER_TIM_PERIOD 0xffff

int16_t Encoder_GetLeft(void);
int16_t Encoder_GetRight(void);
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);

int Read_Encoder(u8 TIMX);

#endif
