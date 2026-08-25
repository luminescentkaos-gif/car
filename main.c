#include "stm32f10x.h"
#include "sys.h"
#include "stdio.h"
#include "colorful_led.h"   //引入头文件，识别函数声明

int main(void)
{ 
	Stm32_Clock_Init(9);
	MY_NVIC_PriorityGroupConfig(2);
	uart_init(115200);
	JTAG_Set(JTAG_SWD_DISABLE);
	JTAG_Set(SWD_ENABLE);
	
	colorful_led_Init();	//调用初始化函数
	printf("QST青软\r\n");
	
	    

	      //设置灯色
        //L_ws2812_rgb(1,WS_RED);
        //L_ws2812_refresh(led_num);

        //开启倒车尾灯
        //R_led_mode();
	
	      //前灯炫彩
	      //L_led_mode();

        //关闭尾灯
        //R_led_CLC();
				
    while(1)
    {
      
   //跑马灯
       L1_runingled(); //正 (加了个彩色)
			//L2_runingled();//反  
			//L3_runingled();//回旋
			
        delay_ms(1000);
    }

}
