#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "rs485.h"
#include "modbus_poll.h"
#include "mb_poll_port_serial.h"

#define APP_LOG_MODULE_NAME   "[mb_poll_serial]"
#define APP_LOG_MODULE_LEVEL   APP_LOG_LEVEL_ERROR   


/*默认115200 8 N 1*/
void mb_poll_port_serial_init(void)
{ 
	RS485_init();
	RS485_TX_EN;

}

void mb_poll_port_serial_enable(mb_poll_bool_t rx_bool,mb_poll_bool_t tx_bool)
{
 if(rx_bool)
  {
 /*使能接收中断*/
		RS485_RX_EN;
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
		
  }
 else
 { 
	  RS485_TX_EN;
		USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
		
 }
 if(tx_bool)
 {
  /*使能发送中断*/
	  RS485_TX_EN;
		USART_ITConfig(UART5, USART_IT_TXE, ENABLE);
   
 }
 else
 {
 /*禁止发送中断*/
	   RS485_RX_EN; 
	   USART_ITConfig(UART5, USART_IT_TXE, DISABLE);
		
 }
}

void mb_poll_port_serial_send_byte(uint8_t send_byte)
{
    USART_SendData(UART5,send_byte);
    while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);	
}

void mb_poll_port_serial_get_byte(uint8_t *ptr_byte)
{
 *ptr_byte = USART_ReceiveData(UART5);
  //APP_LOG_ARRAY("R%d.\r\n",*ptr_byte);
}


//void mb_poll_port_serial_isr(void)
//{
//  uint32_t tmp_flag = 0, tmp_it_source = 0; 
//  
//  tmp_flag = __HAL_UART_GET_FLAG(ptr_mb_poll_port_serial_handle, UART_FLAG_RXNE);
//  tmp_it_source = __HAL_UART_GET_IT_SOURCE(ptr_mb_poll_port_serial_handle, UART_IT_RXNE);
//  /*接收中断*/
//  if((tmp_flag != RESET) && (tmp_it_source != RESET))
//  { 
//   mb_poll_byte_receive();
//  }

//  tmp_flag = __HAL_UART_GET_FLAG(ptr_mb_poll_port_serial_handle, /*UART_FLAG_TXE*/UART_FLAG_TC);
//  tmp_it_source = __HAL_UART_GET_IT_SOURCE(ptr_mb_poll_port_serial_handle, /*UART_IT_TXE*/UART_IT_TC);
//  /*发送中断*/
//  if((tmp_flag != RESET) && (tmp_it_source != RESET))
//  {
//   mb_poll_byte_send();
//  }  
//}
