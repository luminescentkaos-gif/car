#ifndef __NFC_H
#define __NFC_H

#include "stm32f1xx_hal.h"

#define USART2_REC_LEN  50

typedef struct
{
	uint8_t RxBuffer[50];
	uint16_t RxCounter;
}UART2_FrameTypeDef;

extern UART2_FrameTypeDef UART2Frame;

extern u8 const NFC_WakeUp[];
extern u8 const NFC_SearchCard[];

extern u8 NFC_WakeUp_Ok;
extern u8 NFC_find_Card;
extern u8 NFC_sendcmd_find;
extern u8 led_flag;
extern u8 USART2_RX_BUF[USART2_REC_LEN];

void UART2SendFrame(u8 *buf,u16 len);
void put_HEX(UART_HandleTypeDef *huart,u8 *buf,u16 len);
void NFC_Handler(void);
void FoundCard_Handler(void);
void NFC_UART2_RxCallBack(uint8_t data);

#endif
