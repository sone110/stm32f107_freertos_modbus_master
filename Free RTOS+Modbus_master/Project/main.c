/******************** (C) COPYRIGHT 2013 www.armjishu.com  ***********************
 * 文件名  ：main.c
 * 描述    ：实现STM32F107VC神舟IV号开发板上实现串口输入输出功能
 * 实验平台：STM32神舟开发板
 * 标准库  ：STM32F10x_StdPeriph_Driver V3.5.0
 * 作者    ：www.armjishu.com 
**********************************************************************************/
#include "usart.h"
#include "stm32f10x.h"
#include "led.h"
#include "sht30.h"
#include "delay.h"
/*************FreeRTOS头文件************************/
#include "FreeRTOS.h"
#include "task.h"

#include "modbus_poll.h"
float  Tem_Value_1;
float  RH_Value_1;


#define START_TASK_PRIO  1 //任务优先级
#define START_STK_SIZE   128 //任务堆栈大小
TaskHandle_t  StartTask_Handler ;
void start_task(void *pvParameters);

//任务优先级
#define LED0_TASK_PRIO		3
//任务堆栈大小	
#define LED0_STK_SIZE 		128 
//任务句柄
TaskHandle_t LED0Task_Handler;
//任务函数
void led0_task(void *pvParameters);

//任务优先级
#define sht30_TASK_PRIO		2
//任务堆栈大小	
#define sht30_STK_SIZE 		128  
//任务句柄
TaskHandle_t sht30Task_Handler;
//任务函数
void sht30_task(void *pvParameters);

//任务优先级
#define LED1_TASK_PRIO		3
//任务堆栈大小	
#define LED1_STK_SIZE 		128 
//任务句柄
TaskHandle_t LED1Task_Handler;
//任务函数
void led1_task(void *pvParameters);

void vTaskFunction( void *pvParameters );

//static const char *pcTextForTask1 = "Task 1 is running\r\n";
//static const char *pcTextForTask2 = "Task 2 is running\r\n";

/**-------------------------------------------------------
  * @函数名 main
  * @功能   主函数
  * @参数   无
  * @返回值 无
***------------------------------------------------------*/


int main(void)
{ 
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	  
	SZ_STM32_COMInit();	 /* 串口初始化 */
	delay_init();
  LED_Init();
	SHT_Init();
//	System_Setup(); 
	printf("\n\r WWW.ARMJISHU.COM! \n");

	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄        
	  
    vTaskStartScheduler();          //开启任务调度
	return 0;
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建LED0任务
    xTaskCreate((TaskFunction_t )led0_task,     	
                (const char*    )"led0_task",   	
                (uint16_t       )LED0_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED0_TASK_PRIO,	
                (TaskHandle_t*  )&LED0Task_Handler);   
    //创建LED1任务
    xTaskCreate((TaskFunction_t )led1_task,     
                (const char*    )"led1_task",   
                (uint16_t       )LED1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LED1_TASK_PRIO,
                (TaskHandle_t*  )&LED1Task_Handler);     
								
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//LED0任务函数 
void led0_task(void *pvParameters)
{
	uint8_t i ;
	uint16_t reg[10];
  mb_poll_status_t status;
  modbus_poll_init();
  modbus_poll_pre_transmission(0);
  modbus_poll_post_transmission(0);
  while(1)
  {
		/*功能码0x01 读取线圈状态*/
//    status= modbus_poll_readCoils(1,0,17,reg);
//    if(status!=MODBUS_POLL_STATUS_SUCCESS)
//		{
//      printf("读线圈错误！\r\n"); 
//    }
//    else
//    {
//      printf("读线圈值：%d.%d.\r\n",reg[0],reg[1]);  
//    }
//   vTaskDelay(1);
   /*功能码0x0F 写多个线圈*/
//   reg[0]=0x5555;
//   reg[1]=0xaaaa;
//   status= modbus_poll_write_multiple_coils(1,10,20,reg); 
//    
//   if(status!=MODBUS_POLL_STATUS_SUCCESS)
//   {
//     printf("写多个线圈错误！\r\n"); 
//   }
//     vTaskDelay(1);
//    status=modbus_poll_read_input_registers(1,20,10,reg);
//    
//    if(status!=MODBUS_POLL_STATUS_SUCCESS)
//    {
//      printf("读输入寄存器错误！\r\n"); 
//    }
//    else
//    {
//      for(i=0;i<10;i++)
//      {
//        printf("读取的输入寄存器值:%d.\r\n",reg[i]); 
//      }
//    }
//    vTaskDelay(1);
//    status= modbus_poll_write_single_coil(1,10,0);
//    if(status!=MODBUS_POLL_STATUS_SUCCESS)
//    {
//			printf("写单个线圈错误！\r\n"); 
//    }
//    vTaskDelay(1);
   /*功能码0x10*/
//    status= modbus_poll_write_multiple_registers(1,0,10, reg);
//    if(status!=MODBUS_POLL_STATUS_SUCCESS)
//    {
//			printf("写多个寄存器错误！\r\n"); 
//    }
     vTaskDelay(1);		
		LED1Toggle();
		SHT_GetValue();
		printf("\r\nTemp=%3.1lf  RH=%3.1lf \r\n",Tem_Value_1 ,RH_Value_1);
		vTaskDelay(500);			
  }
}   

//LED1任务函数
void led1_task(void *pvParameters)
{
	  uint16_t reg[4];
  mb_poll_status_t status;
 // vTaskDelay(500);
	while(1)
	{
		LED2Toggle_GREEN();
					/*功能码0x03 读保持寄存器*/
		status= modbus_poll_read_holding_registers(2,0,4,reg);
		if(status!=MODBUS_POLL_STATUS_SUCCESS)
		{
		 printf("读净重错误！\r\n"); 
		}
		else
		{
	//		printf("读净重值：1.%2x 2.%2x 3.%2x 4.%2x\r\n",reg[0],reg[1],reg[2],reg[3]);   
		}
		
	
		vTaskDelay(200);
		LED2Toggle_RED();
	}
}



void vTaskFunction( void *pvParameters )
{
		char *pcTaskName;
		volatile unsigned long ul;
		pcTaskName = ( char * ) pvParameters;
		taskENTER_CRITICAL(); 
/* As per most tasks, this task is implemented in an infinite loop. */
		for( ;; )
		{
/* Print out the name of this task. */
				printf( "%s \n",pcTaskName );
				vTaskDelay(1000);
/* Delay for a period. */
		for( ul = 0; ul < 500000; ul++ )
		{
/* This loop is just a very crude delay implementation. There is
nothing to do in here. Later exercises will replace this crude
loop with a proper delay/sleep function. */
		}
		taskEXIT_CRITICAL(); 
	}
		
}


