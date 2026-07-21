#ifndef __POSTION_CONTROL_H
#define __POSTION_CONTROL_H

#include "sys.h"
#include "TIME.h"
#include "tim.h"

#define FINISH_MOVE     1
#define NO_FINISH_MOVE  0
uint8_t uart7WriteBuf(uint8_t *buf, uint8_t len);

extern struct POSTION Z_POSTION;
extern volatile uint8_t u7_debug_buf[8];
extern volatile uint16_t u7_debug_size;
extern volatile uint32_t u7_debug_rx_restart_count;
extern volatile uint32_t u7_debug_rx_restart_fail;
extern volatile uint32_t u7_debug_rx_event_count;
extern volatile uint32_t u7_debug_error_count;
extern volatile uint32_t u7_debug_last_error;
extern volatile uint32_t u7_debug_tx_count;
extern volatile uint32_t u7_debug_tx_retry_count;
extern volatile uint8_t u7_debug_last_tx_status;
extern volatile uint8_t u7_debug_last_tx_len;
extern volatile uint8_t u7_debug_last_tx_buf[13];
extern volatile uint32_t u7_debug_z_finish_count;
extern volatile uint8_t z_debug_last_timeout;
extern volatile uint32_t z_debug_last_wait_ms;
struct POSTION
{
	int NOW;
    int TARGE;
    volatile int CHANGE;
    volatile int BIT;
};

void POSTION_init(void);
void postion_send(uint8_t id,int position);
void Z_SetHeight(int high);
void Y_SetLength(int length);
uint8_t Z_InitMoveSigned(int height);
uint8_t Y_InitMoveSigned(int length);
void Read_Z_position(void);
void u7RXdat_dispose(uint8_t* data);
void u7_speed_send(uint8_t id,int speed);
void Motor_Height_Ss_Stop(char id);
void MY_UART7_IRQHandler(uint16_t size);
void UART7_RxRestart(void);

uint32_t Get_Y_position(void);
void Read_Y_position(void);
void YZ_SetZero(char id);

#endif

