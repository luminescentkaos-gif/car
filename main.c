#include "stm32f10x.h"
#include "sys.h"
#include "stdio.h"
#include "colorful_led.h"
#include "nfc.h"
#include "encode.h"
#include "motor.h"
#include "control_system.h"


int16_t L_speed = 0;
int16_t R_speed = 0;


extern int L_coder;
extern int R_coder;

extern int Motor_A;
extern int Motor_B;


int main(void)
{
    int i;


    /* 清除复位标志 */
    RCC->CSR |= 1 << 24;


    /* 72MHz */
    Stm32_Clock_Init(9);


    /* NVIC */
    MY_NVIC_PriorityGroupConfig(2);


    /* 延时 */
    delay_init();


    /* USART1 */
    uart_init(115200);


    /* USART2 */
    uart2_init(115200);


    /* SWD */
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);


    /* PWM */
    PWM_Init(7199, 9);


    /* 彩灯 */
    colorful_led_Init();


    /* 编码器 */
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();


    /*
     * 不开SysTick
     */
    //SysTick_Config(72000000 / 1000);


    printf("QST\r\n");
		


    /* ============================
     * 前进5秒
     * ============================ */

    for(i = 0; i < 50; i++)
    {
        Straight_Control(1);
			

        Car_Led_Run();

        NFC_Handler();

        delay_ms(100);
    }


    /* ============================
     * 停车0.5秒
     * ============================ */

    Set_Pwm(0, 0);

    delay_ms(500);


    /* ============================
     * 后退5秒
     * ============================ */

    for(i = 0; i < 50; i++)
    {
        Straight_Control(-1);

        Car_Led_Run();

        NFC_Handler();

        delay_ms(100);
    }


    /* ============================
     * 最终停车
     * ============================ */

    Set_Pwm(0, 0);


    while(1)
    {
        NFC_Handler();
    }
}


