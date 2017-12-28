/*
 * sensors.c
 *
 *  Created on: Dec 28, 2017
 *      Author: Atalville
 */

/* Standard Includes */
#include <stdio.h>
#include <string.h>

/* My Includes */
#include "driverlib.h"
#include "clk.h"
#include "systick.h"
#include "uart.h"
#include "sensors.h"

/* Variable declarations related to Bluetooth */
//char g_rx_buff[RX_BUFF_SIZE];
//uint8_t g_ms_timeout = 0;
//uint32_t g_check_connection_count = 0;
//uint32_t g_data_send_interval_count = 0;
//uint8_t g_ble_connect_state = 0;
//_E_BLE_STATE g_ble_state = BLE_IDLE;

/* Function to reset buffer for bluetooth */
void reset_rx_buffer(char *buff){
    memset(buff,0,RX_BUFF_SIZE);
    set_UARTA2_rx_ptr(0);
}
/* Function to initialize Bluetooth Pins */
void ble_pins_init(char *buff){
/* Structure for initializing UART related to Bluetooth */
const eUSCI_UART_Config uartAConfig = {
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
            104,                                      // BRDIV = 26
            0,                                       // UCxBRF = 0
            0,                                       // UCxBRS = 0
            EUSCI_A_UART_NO_PARITY,                  // No Parity
            EUSCI_A_UART_LSB_FIRST,                  // MSB First
            EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
            EUSCI_A_UART_MODE,                       // UART mode
            EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION  // Low Frequency Mode
            };
    /* UART2(115200 bps baudrate) BLE UART Module initialization*/
    uartA2_init(&uartAConfig,buff,RX_BUFF_SIZE);
}
/* Function to initialize Debug Uart */
void dummy_uart_init(){
    const eUSCI_UART_Config uartAConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
                104,//1250,//104,                                      // BRDIV = 26
                0,                                       // UCxBRF = 0
                0,                                       // UCxBRS = 0
                EUSCI_A_UART_NO_PARITY,                  // No Parity
                EUSCI_A_UART_LSB_FIRST,                  // MSB First
                EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
                EUSCI_A_UART_MODE,                       // UART mode
                EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION  // Low Frequency Mode
                };

    /* UARTA0(115.2 kbps baudrate) Debug Module initialization*/
    uartA0_init(&uartAConfig,0,0);
}
/* Function to check a string in Bluetooth Response */
uint8_t check_response(char * buff, char *resp){
    char *ret;
    ret = strstr(buff,resp);
    if(!ret){
        return 0;
    }
    return 1;
}
/* Function to Send Bluetooth Data.
 * Returns -1 if sent failed
 * Returns 1 if sent successful
 */
int8_t ble_send_data(char *buff,_S_SENSOR_DATA * data){
    int err = 0;
    uint8_t flag = 0;
    uint8_t i = 0;
    static char ble_buff[200];
    static uint32_t packet_no = 0;
    /* Create the Bluetooth packet */
    sprintf(ble_buff, "{\\r\\n \"Packet No\" : %d,\\r\\n\"HeartRate\" : %d,\\r\\n\"PulseOximetry\" : %d,\\r\\n\"RespiratoryRate\" : %d,\\r\\n\"Temperature\" : %.1f,\\r\\n\"BodyPosition\" : \"%s\" \\r\\n}\\r\\n\r\n",
            ++packet_no,data->hr,data->sp02,data->rr,data->temp_f,data->accel_buf);
    do{
        /* Reset Bluetooth Buffer */
        reset_rx_buffer(buff);
        /* Number of data send retries reaches max limit */
        if(i == BLE_SEND_DATA_RETRIES){
            /* return from function with -1 */
            return -1;
        }
        SEND_CMD("AT+BLEUARTTX=");
        SEND_CMD(ble_buff);
        systick_delay_ms(BLE_DELAY_MS);
        i++;
        /* Check for OK response after Bluetooth data is sent */
        err = check_response(buff,"OK\r\n");
        /* OK not found */
        if(!err){
            if(!flag){
                packet_no--;
                flag = 1;
            }
            printf("Packet Loss for packet number %d\r\n",packet_no);

        }
    }while(err!=1); /* Wait until OK is found */
    /* Return 1 to indicate data sent successfully */
    return 1;
}
/* Function to check Bluetooth Connection
* Returns -1 if Command Error (OK not found)
* Returns 1 if Connection successful
* Returns 0 if no device to connect to
*/
int8_t ble_check_connection(char *buff){
    uint8_t i = 0;
    do{
        /* Reset Bluetooth Buffer */
        reset_rx_buffer(buff);
        /* Number of Connection retries reaches max limit */
        if(i == BLE_CHK_CONN_RETRIES){
            return -1;
        }
        SEND_CMD("AT+GAPGETCONN\r\n");
        systick_delay_ms(BLE_DELAY_MS);
        i++;
    }while(check_response(buff,"OK\r\n")!=1);/* Wait until OK is found */
    /* if 1 found with OK, device is connected */
    if(check_response(buff,"1\r\nOK\r\n")){
        return 1;
    }else{ /* if 0 found with OK, device is not connected */
        return 0;
    }
}



