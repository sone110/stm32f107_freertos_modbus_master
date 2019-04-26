#ifndef  __MB_POLL_PORT_TIMER_H__
#define  __MB_POLL_PORT_TIMER_H__

/*如果波特率大于19200 t3.5 最小等于3ms*/
/*波特率等于9600时为4MS*/
#define  MB_POLL_PORT_TIMER_35_TIMEOUT         4


void mb_poll_port_timer_init(void);
void mb_poll_port_timer_35_start(void);
void mb_poll_port_timer_response_start(void);
void mb_poll_port_timer_stop(void);








#endif
