 /********************   (C) COPYRIGHT 2013 www.armjishu.com   ********************
 * 文件名  ：led.c
 * 描述    ：提供STM32F107VC神舟IV号开发板的库函数
 * 实验平台：STM32神舟IV号开发板
 * 作者    ：www.armjishu.com 
 * 修改日期：2014/03
**********************************************************************************/
 #include "led.h"


void LED_Init()	  //端口初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;	//声明一个结构体变量，用来初始化GPIO  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE); /* 开启GPIO时钟 */

	/*  配置GPIO的模式和IO口 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;	 //选择你要设置的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	  //设置推挽输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //设置传输速率
	GPIO_Init(GPIOE,&GPIO_InitStructure); /* 初始化GPIO */	
}

/**-------------------------------------------------------
  * @函数名 SZ_STM32_LED1Toggle
  * @功能   将对应的LED指示灯状态取反
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void LED1Toggle(void)
{
    /* 指定管脚输出异或 1，实现对应的LED指示灯状态取反目的 */
    GPIOE->ODR ^= GPIO_Pin_11;
}

/**-------------------------------------------------------
  * @函数名 SZ_STM32_LED2Toggle
  * @功能   将对应的LED指示灯状态取反
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void LED2Toggle_RED(void)
{
    /* 指定管脚输出异或 1，实现对应的LED指示灯状态取反目的 */
    GPIOE->ODR ^= GPIO_Pin_12;
}
/**-------------------------------------------------------
  * @函数名 SZ_STM32_LED3Toggle
  * @功能   将对应的LED指示灯状态取反
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/
void LED2Toggle_GREEN(void)
{
    /* 指定管脚输出异或 1，实现对应的LED指示灯状态取反目的 */
    GPIOE->ODR ^= GPIO_Pin_13;
} 
/**-------------------------------------------------------
  * @函数名 SZ_STM32_LED4Toggle
  * @功能   将对应的LED指示灯状态取反
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/

