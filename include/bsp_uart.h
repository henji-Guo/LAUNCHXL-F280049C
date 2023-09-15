/*
 * bsp_uart.h
 *
 *  Created on: 2023年8月6日
 *      Author: GHJ
 */

#ifndef USER_DRIVER_INCLUDE_BSP_UART_H_
#define USER_DRIVER_INCLUDE_BSP_UART_H_

extern struct bsp_uart bsp_uart;

void uart_init(void);
void uart_int_transmit(struct bsp_uart *huart,uint16_t *sendMsg, uint16_t len);
int uart_isIDLE(struct bsp_uart *huart);


#endif /* USER_DRIVER_INCLUDE_BSP_UART_H_ */
