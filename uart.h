/*
 * uart.h
 *
 *  Created on: Mar 24, 2017
 *      Author: Atalville
 */

#ifndef _MY_UART_H_
#define _MY_UART_H_

void uartA0_init(const eUSCI_UART_Config *uartConfig,uint8_t *buff,uint16_t rx_buff_len);
void uartA0_tx_str(char *outString);
void uartA0_tx_bytes(char *outString, uint32_t len);

void uartA2_init(const eUSCI_UART_Config *uartConfig,uint8_t *buff,uint16_t rx_buff_len);
void uartA2_tx_str(char *outString);
void uartA2_tx_bytes(char *outString, uint32_t len);

int set_UARTA2_rx_ptr(uint16_t p);

#endif /* UART_H_ */
