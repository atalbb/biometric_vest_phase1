
/* Standard Includes */
#include <stdio.h>
#include <string.h>

/* My Includes */
#include "driverlib.h"
#include "clk.h"
#include "systick.h"
#include "uart.h"
#include "sensors.h"

/* Global Variables for common operation */
_S_SENSOR_DATA g_sensor_data={-999,-999,-999,-999.9,"-999.9"};

/* Global Variable Declarations related to Systick */
volatile uint8_t g_ms_timeout = 0;
volatile uint32_t g_ticks = 0;

/* Gloabl Variable declarations related to Bluetooth */
char g_rx_buff[RX_BUFF_SIZE];
uint32_t g_check_connection_count = 0;
uint32_t g_data_send_interval_count = 0;
uint8_t g_ble_connect_state = 0;
_E_BLE_STATE g_ble_state = BLE_IDLE;


/* Systick Handler */
void SysTick_Handler(){
    g_ms_timeout = 1;
    g_ticks++;
    /* If BLE State is Idle */
    if(g_ble_state == BLE_IDLE){
        /* If 1 sec is up */
        if(++g_check_connection_count == 1000){
            /* Change BLE State */
            g_ble_state = BLE_CHECK_CONNECTION;
            g_check_connection_count = 0;
        }
    }else if(g_ble_state == BLE_CONNECTED){ /* If BLE State is BLE_CONNECTED */
        /* If 1 sec is up */
        if(++g_data_send_interval_count == 1000){
            /* Change BLE State */
            g_ble_state = BLE_SEND_DATA;
            g_data_send_interval_count = 0;
        }
    }
}


void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    clk_init();
    dummy_uart_init();
    systick_init();
    ble_pins_init(g_rx_buff);
    MAP_Interrupt_enableMaster();
    uartA0_tx_str(" Program Started!\r\n");
    while(1){
        /* If BLE state is BLE_CHECK_CONNECTION*/
        if(g_ble_state == BLE_CHECK_CONNECTION){
            /* Check to see if our device is connected to other device */
            if(ble_check_connection(g_rx_buff) == -1){/* Device Connection Command failed */
                /* Change BLE State to BLE_IDLE */
                g_ble_state = BLE_IDLE;
                DISPLAY_RESP("OK not found for AT+GAPGETCONN\r\n");
            }else if(ble_check_connection(g_rx_buff) == 0){ /* Our Device is not connected to other device yet */
                DISPLAY_RESP("No BLE Connection Found!\r\n");
                /* Change BLE State to BLE_IDLE */
                g_ble_state = BLE_IDLE;
            }else if(ble_check_connection(g_rx_buff) == 1){ /* Our Device is not connected to another device */
                DISPLAY_RESP("BLE Connected\r\n");
                /* Change BLE State to BLE_CONNECTED */
                g_ble_state = BLE_CONNECTED;
            }else{
                DISPLAY_RESP("INVALID RESPONSE FOR CHECK CONNECTION\r\n");
                g_ble_state = BLE_IDLE;
            }
        /* If BLE state is BLE_SEND_DATA*/
        }else if(g_ble_state == BLE_SEND_DATA){
            if(ble_send_data(g_rx_buff,&g_sensor_data)==1){ /*Bluetooth Data is Sent Successfully */
                static uint32_t success = 0;
                printf("BLE Data Sent Successfully:%d!\r\n",++success);
                /* Change BLE State to BLE_CONNECTED */
                g_ble_state = BLE_CONNECTED;
            }else if(ble_send_data(g_rx_buff,&g_sensor_data)==-1){ /* Bluetooth Data Sent Failed */
                printf("BLE Data Send Fail!\r\n");
                /* Change BLE State to BLE_CHECK_CONNECTION */
                g_ble_state = BLE_CHECK_CONNECTION;
            }else{
                g_ble_state = BLE_IDLE;
            }
        }
    }
}
