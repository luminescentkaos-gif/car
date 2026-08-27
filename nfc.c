#include "nfc.h"
#include "usart.h"
#include "delay.h"
#include "led.h"

u8 const NFC_WakeUp[] = {0x55,0x55,0,0,0,0,0,0,0,0,0,0,0,0,0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,0x00}; //唤醒命令
u8 const NFC_SearchCard[] = {0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00};                //寻卡命令

u8 NFC_WakeUp_Ok = 0;	                //NFC唤醒标志
u8 NFC_find_Card = 0;	                //NFC找到一张卡
u8 NFC_sendcmd_find = 1;	            //NFC收到卡帧头等待
u8 NFC_wait_Card = 0;
u8 NFC_read_id_flag=0;
u8 NFC_DataBlock[16];	                //存储一个BLOCK的数据
u8 USART2_RX_BUF[USART2_REC_LEN];	    //接收缓冲,最大USART2_REC_LEN个字节.
u16 USART2_RX_STA=0;	                //接收状态标记
u16 slen;
u8 Sys_Stat;	                        //nfc id卡状态
u8 Sum = 0;	                            //校验和
u8 REC_LEN=0;
u8 led_flag=0;

UART2_FrameTypeDef UART2Frame;

//串口2发送一帧数据
void UART2SendFrame(u8 *buf,u16 len)
{
    HAL_UART_Transmit(&huart2,buf,len,100);
}

//以16进制格式打印缓冲区到串口1
void put_HEX(UART_HandleTypeDef *huart,u8 *buf,u16 len)
{
    u8 i;
    u8 temp[3];
    for(i=0;i<len;i++)
    {
        sprintf((char*)temp,"%02X ",buf[i]);
        HAL_UART_Transmit(huart,temp,3,10);
    }
}

//NFC主处理函数，放在main while循环
void NFC_Handler(void)
{
	if(NFC_WakeUp_Ok) //已唤醒,发指令寻卡
	{
		if(NFC_find_Card==1 ) //是否已寻到卡?
		{
			//找到一张卡
			FoundCard_Handler();
		}
		else if(NFC_find_Card==0 && NFC_sendcmd_find==1)
		{
			UART2Frame.RxCounter=0;
			//未找到卡,发指令
			UART2SendFrame((u8*)NFC_SearchCard, sizeof(NFC_SearchCard));//发送寻卡指令
			NFC_sendcmd_find=0;
			delay_ms(200);
		}
	}
}

//找到卡片回调处理
void FoundCard_Handler(void)
{
	NFC_find_Card=0;	//清除标识
	if(led_flag==0)		//反转车灯
	{
		led_flag =1;
		R_led_mode();
	}
	else
	{
		led_flag =0;
		R_led_CLC();
	}

	NFC_sendcmd_find=1;//发送寻卡指令
	delay_ms(200);
}

//USART2接收中断回调，放在HAL_UART_RxCpltCallback里面调用
void NFC_UART2_RxCallBack(uint8_t data)
{
	UART2Frame.RxBuffer[UART2Frame.RxCounter] = data;
	if(NFC_WakeUp_Ok==0)			//未唤醒
	{
		UART2Frame.RxCounter++;
		if(UART2Frame.RxCounter==15)
		{
			memcpy(USART2_RX_BUF,(uint8_t*)UART2Frame.RxBuffer,15);
			memset((uint8_t*)UART2Frame.RxBuffer,0,20);
			UART2Frame.RxCounter=0;
			NFC_WakeUp_Ok = 1;	//唤醒完成置1
		}
	}
	else							//唤醒成功，进入寻卡流程
	{
		UART2Frame.RxCounter++;
		if(UART2Frame.RxCounter==25)
		{
			memcpy(USART2_RX_BUF,(uint8_t*)UART2Frame.RxBuffer,25);
			put_HEX(&huart1,USART2_RX_BUF,25);
			if( (0xB9==USART2_RX_BUF[19])&&(0x80==USART2_RX_BUF[20])&&(0x06==USART2_RX_BUF[21])&&(0x85==USART2_RX_BUF[22])
			|| (0x50==USART2_RX_BUF[19])&&(0x84==USART2_RX_BUF[20])&&(0xFC==USART2_RX_BUF[21])&&(0x23==USART2_RX_BUF[22])
			|| (0x40==USART2_RX_BUF[19])&&(0x74==USART2_RX_BUF[20])&&(0x80==USART2_RX_BUF[21])&&(0x23==USART2_RX_BUF[22]) )
			{
				NFC_find_Card = 1;
			}
			memset((uint8_t*)UART2Frame.RxBuffer,0,50);
			memset((uint8_t*)USART2_RX_BUF,0,50);
			UART2Frame.RxCounter=0;
		}
	}
}
