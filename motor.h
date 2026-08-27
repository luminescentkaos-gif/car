#ifndef __MOTOR_H
#define __MOTOR_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "sys.h"
/* 位带操作（外设）
   使用示例：BITBAND_PERI(&GPIOB->ODR, 14) 当作可读写的位别名 */
#define BITBAND_PERI(addr, bit) (*(volatile u32*)(0x42000000 + (((u32)(addr) - 0x40000000) * 32U) + ((bit) * 4U)))
/* 便利别名：直接操作 GPIOB ODR 指定位 */
#define AIN BITBAND_PERI(&GPIOB->ODR, 14)
#define BIN BITBAND_PERI(&GPIOB->ODR, 13)
/* 将占空比分配到 TIM4 比较寄存器（使用 TIM4 CH1 / CH2，对应 PB6/PB7） */
#define PWMA TIM4->CCR1
#define PWMB TIM4->CCR2
/* 默认 PWM 自动重装载 (与 main 中调用 PWM_Init(7199,9) 对应) */
#define PWM_ARR 7199
/* API */
void Motor_Init(void);
void PWM_Init(u16 arr, u16 psc);
u32 myabs(long int a);
void Set_Pwm(int moto1, int moto2);
#ifdef __cplusplus
}
#endif
#endif /* __MOTOR_H */
