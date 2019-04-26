#include "FreeRTOS.h"
#include "task.h"
#include "modbus_poll.h"
#include "timers.h"
#include "mb_poll_port_timer.h"
#include "usart.h"
#define APP_LOG_MODULE_NAME   "[mb_poll_port_timer]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_ERROR    


static void mb_poll_port_timer_expired(void  );
//TimerHandle_t mb_poll_timer_id;

void TIM2_IRQHandler(void)
{
	mb_poll_port_timer_expired();
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void mb_poll_port_timer_init(void)
{
//  mb_poll_timer_id    = xTimerCreate( "master timer",
//                   MB_POLL_PORT_TIMER_35_TIMEOUT,        //定时时间，
//	                 pdFALSE,                                                             //单次定时
//	                 (void *)0,
//									 mb_poll_port_timer_expired);                                                  //回调函数
// if(mb_poll_timer_id == NULL)
// {
//	 printf("\r\n 3.5字符定时器创建失败 \r\n");
// 
// }
// else
// {
//	 printf("\r\n 3.5字符定时器创建成功 \r\n"); 
// }
	/*** (1+psc )/72M)*(1+arr )***/
	NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
 	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);	
 
  TIM_DeInit(TIM2);
//	
  TIM_TimeBaseStructure.TIM_Period = 8;
  TIM_TimeBaseStructure.TIM_Prescaler = (36000-1);	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
// 	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void mb_poll_port_timer_35_start(void)
{
// printf("timer35 start.\r\n");
//	if( xTimerIsTimerActive(mb_poll_timer_id )  != pdFALSE )
//	{
//	
//	}
//	else
//	{
//		xTimerStart(mb_poll_timer_id,0);	
//	}
// //
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_SetCounter(TIM2, 0);
	//TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void mb_poll_port_timer_stop(void)
{
//	printf("timer35 stop.\r\n");
//  xTimerStop(mb_poll_timer_id,0);
		TIM_SetCounter(TIM2, 0);
	//TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
}

static void mb_poll_port_timer_expired(void)
{
	mb_poll_port_timer_stop();
  mb_poll_timer_35_expired();
}
