#include "encode.h"
#include "stm32f10x_gpio.h"
#include <stdint.h>

/*********************************************************
函数功能: 把TIM2初始化为编码器接口模式
入口参数: 无
返回  值: 无
*********************************************************/
void Encoder_Init_TIM2(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(
        TIM2,
        TIM_EncoderMode_TI12,
        TIM_ICPolarity_Rising,
        TIM_ICPolarity_Rising
    );

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

/*********************************************************
函数功能: 把TIM3初始化为编码器接口模式
入口参数: 无
返回  值: 无
*********************************************************/
void Encoder_Init_TIM3(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(
        TIM3,
        TIM_EncoderMode_TI12,
        TIM_ICPolarity_Rising,
        TIM_ICPolarity_Rising
    );

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

/*********************************************************
函数功能: 单位时间读取编码器计数
入口参数: 定时器
返回  值: 速度值
*********************************************************/
static long Encoder_TIM_A_now;
static long Encoder_TIM_B_now;

int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;

    switch(TIMX)
    {
        case 2:
            Encoder_TIM_A_now = (short)TIM2->CNT;
            Encoder_TIM = Encoder_TIM_A_now;
            TIM_SetCounter(TIM2, 0);
            break;

        case 3:
            Encoder_TIM_B_now = (short)TIM3->CNT;
            Encoder_TIM = Encoder_TIM_B_now;
            TIM_SetCounter(TIM3, 0);
            break;

        default:
            Encoder_TIM = 0;
            break;
    }

    return Encoder_TIM;
}

/*********************************************************
函数功能: 获取左轮编码器计数
入口参数: 无
返回  值: 左轮编码器计数
*********************************************************/
int16_t Encoder_GetLeft(void)
{
    return (int16_t)Read_Encoder(2);
}

/*********************************************************
函数功能: 获取右轮编码器计数
入口参数: 无
返回  值: 右轮编码器计数
*********************************************************/
int16_t Encoder_GetRight(void)
{
    return (int16_t)Read_Encoder(3);
}

