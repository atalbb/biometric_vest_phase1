/*
 * uart.c
 *
 *  Created on: Mar 24, 2017
 *      Author: Atalville
 */
#include "driverlib.h"
uint8_t *uartA0_rx_buff;
uint8_t *uartA2_rx_buff;
uint16_t uartA0_rx_buff_len;
uint16_t uartA2_rx_buff_len;
uint16_t uartA0_rx_ptr = 0;
uint16_t uartA2_rx_ptr = 0;
inline void uartA0_tx(char _c)
{
  while((UCA0IFG & UCTXIFG) != UCTXIFG);  // wait until flag is set to indicate a new byte can be sent
  UCA0TXBUF = _c;
}
inline void uartA2_tx(char _c)
{
  while((UCA2IFG & UCTXIFG) != UCTXIFG);  // wait until flag is set to indicate a new byte can be sent
  UCA2TXBUF = _c;
}
void uartA0_tx_str(char *outString)
{
    while(*outString){
        uartA0_tx(*outString);
        outString++;
    }
}
void uartA2_tx_str(char *outString)
{
    while(*outString){
        uartA2_tx(*outString);
        outString++;
    }
}
void uartA0_tx_bytes(char *outString, uint32_t len)
{
    while(len--){
        uartA0_tx(*outString);
    }
}
void uartA2_tx_bytes(char *outString, uint32_t len)
{
    while(len--){
        uartA2_tx(*outString);
    }
}
void uartA0_init(const eUSCI_UART_Config *uartConfig, uint8_t * buff,uint16_t rx_buff_len){
    uartA0_rx_buff = buff;
    uartA0_rx_buff_len = rx_buff_len;
    // set up EUSCI0 in UART mode
    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, uartConfig);
    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);

    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);
}
void uartA2_init(const eUSCI_UART_Config *uartConfig, uint8_t *buff,uint16_t rx_buff_len){
    uartA2_rx_buff = buff;
    uartA2_rx_buff_len = rx_buff_len;
    // set up EUSCI0 in UART mode
    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_BASE, uartConfig);
    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_BASE);

    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA2);
}
uint16_t uartA0_read_rx_buff(){
    if(uartA0_rx_buff[uartA0_rx_ptr]){
        return uartA0_rx_buff[uartA0_rx_ptr--];
    }else{
        return 0xffff;
    }
}
uint16_t uartA2_read_rx_buff(){
    if(uartA2_rx_buff[uartA2_rx_ptr]){
        return uartA2_rx_buff[uartA2_rx_ptr--];
    }else{
        return 0xffff;
    }
}
void EUSCIA0_IRQHandler(){
    char rx0 = UCA0RXBUF;
    if(uartA0_rx_buff_len){
        if(uartA0_rx_ptr == uartA0_rx_buff_len){
            uartA0_rx_ptr  = 0;
        }
        uartA0_rx_buff[uartA0_rx_ptr++] = rx0;
    }
    //uartA2_tx(rx0);
}
void EUSCIA2_IRQHandler(){
    char rx2 = UCA2RXBUF;
    if(uartA2_rx_buff_len){
        if(uartA2_rx_ptr == uartA2_rx_buff_len ){
            uartA2_rx_ptr = 0;
        }
        uartA2_rx_buff[uartA2_rx_ptr++] = rx2;
    }
    //uartA0_tx(rx2);
}
int return_UARTA2_rx_ptr(){
    return uartA2_rx_ptr;
}
int set_UARTA2_rx_ptr(uint16_t p){
    uartA2_rx_ptr  = p;
}
