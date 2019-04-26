#include "usart.h"

/**-------------------------------------------------------
  * @函数名 __SZ_STM32_COMInit
  * @功能   面向用户的STM32的USART初始化函数
  * @参数1  COM1  对应STM32的USART1 对应开发板上串口1
  *         COM2  对应STM32的USART2 对应开发板上串口2
  * @参数2  BaudRate 串口的波特率，例如"115200"
  * @返回值 无
***------------------------------------------------------*/
void SZ_STM32_COMInit(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* 使能STM32的USART对应GPIO的Clock时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    
	  GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
    /* 初始化STM32的USART的TX管脚，配置为复用功能推挽输出 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB , &GPIO_InitStructure);

    /* 初始化STM32的USART的RX管脚，配置为复用功能输入 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB , &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate = 115200;              //串口的波特率，例如115200 最高达4.5Mbits/s
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据字长度(8位或9位)
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      //可配置的停止位-支持1或2个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;         //无奇偶校验  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //双工模式，使能发送和接收

    /* 根据传入的参数初始化STM32的USART配置 */
    USART_Init(USART1, &USART_InitStructure);

    /* 使能STM32的USART功能模块 */
    USART_Cmd(USART1, ENABLE);  
}


/* 重定义fputc函数 如果使用MicroLIB只需要重定义fputc函数即可 */  
int fputc(int ch, FILE *f)
{
    /* e.g. write a character to the USART */
    USART_SendData(USART1, (uint8_t) ch);

    /* Loop until the end of transmission */
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    return ch;
}

int fgetc(FILE *fp)
{
	int ch = 0;
	
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
   
    ch = (int)USART1->DR & 0xFF;
	
    putchar(ch); //回显
	
	return ch;
}
