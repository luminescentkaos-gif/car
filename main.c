#include "stm32f10x.h"
#include "sys.h"
#include "stdio.h"
#include "colorful_led.h"
#include "nfc.h"
#include "encode.h"
#include "motor.h"

int16_t L_speed = 0;
int16_t R_speed = 0;


void My_USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uint8_t ch = USART_ReceiveData(USART2);
		printf("RX2:%02X ",ch);  // 直接从串口1打印收到的每一个字节！
		USART2_IRQHandler_Callback();
	}
}


int main(void)
{
    int i;

    RCC->CSR |= 1 << 24; // 清除复位标志
    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);
    delay_init();
    uart_init(115200);
    uart2_init(115200);

    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    PWM_Init(7199, 9);
    colorful_led_Init();

    Encoder_Init_TIM2();
    Encoder_Init_TIM3();

    // 配置SysTick：72M系统时钟，1ms产生一次中断
    SysTick_Config(72000000 / 1000);

    printf("QST\r\n");
		


    /*************** 前进5秒 ***************/
    Set_Pwm(4000, 4000);

    for(i = 0; i < 50; i++)
    {
        L_speed = Encoder_GetLeft();
        R_speed = Encoder_GetRight();
			  Car_Led_Run();

        printf("Forward Left=%d, Right=%d\r\n",
               L_speed, R_speed);

        delay_ms(100);
    }


    /*************** 停车0.5秒 ***************/
    Set_Pwm(0, 0);
    delay_ms(500);


    /*************** 后退5秒 ***************/
    Set_Pwm(-4000, -4000);

    for(i = 0; i < 50; i++)
    {
        L_speed = Encoder_GetLeft();
        R_speed = Encoder_GetRight();
			Car_Led_Run();

        printf("Backward Left=%d, Right=%d\r\n",
               L_speed, R_speed);

        delay_ms(100);
    }


    /*************** 最终停车 ***************/
    Set_Pwm(0, 0);
		
		while(1)
		{
			NFC_Handler();
		}

}

