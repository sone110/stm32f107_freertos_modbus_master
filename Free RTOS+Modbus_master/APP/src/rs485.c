#include "rs485.h"
#include "delay.h"
#include "usart.h"
#include "mb_poll_port_serial.h"
#include "modbus_poll.h"

void UART5_IRQHandler(void)
{ uint16_t data;  
	if(USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{	
//    data = USART_ReceiveData(UART5);	
//   // printf("0x%x",data);	
//    USART_SendData(USART1,data);		
   	mb_poll_byte_receive();	
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}

	
	if(USART_GetITStatus(UART5, USART_IT_TXE) == SET)
	{
		mb_poll_byte_send();
		USART_ClearFlag(UART5,USART_FLAG_TC);
	}
	
}

void RS485_init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure ;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure ;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC  ,ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD  ,ENABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5  ,ENABLE ); 
	
		//UART4_TX	  GPIOC.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//UART4_RX	  GPIOC.11初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 6 ;//抢占优先级8
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
	USART_InitStructure.USART_BaudRate = 9600;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(UART5, &USART_InitStructure); //初始化串口2
//  USART_ITConfig(UART5, USART_IT_RXNE | USART_IT_TXE, ENABLE);//开启串口接受中断
  USART_Cmd(UART5, ENABLE);   
}


void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN;			//设置为发送模式
  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);	  
		USART_SendData(UART5,buf[t]);
		//delay_ms();
	}	 
 
	while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET) ;		 
	RS485_RX_EN;				//设置为接收模式	
}
