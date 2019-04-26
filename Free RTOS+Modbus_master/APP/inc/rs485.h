#ifndef __RS485_H
#define __RS485_H	 


#define RS485_RX_EN  GPIO_SetBits(GPIOD , GPIO_Pin_1)
#define RS485_TX_EN  GPIO_ResetBits(GPIOD , GPIO_Pin_1)

#include "stm32f10x.h"

extern uint8_t Receive_BUFF[8];
extern volatile uint8_t NUM;
void RS485_init(void);
void RS485_Send_Data(u8 *buf,u8 len);

#endif
