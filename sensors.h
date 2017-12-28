/*
 * sensors.h
 *
 *  Created on: Dec 28, 2017
 *      Author: Atalville
 */

#ifndef SENSORS_H_
#define SENSORS_H_

/************************************ COMMON ************************************************/
typedef struct{
    int16_t rr;
    int32_t hr;
    int32_t sp02;
    float temp_f;
    char accel_buf[10];
}_S_SENSOR_DATA;


/************************************ END OF COMMON ************************************************/


/*************************************BLUETOOTH ***********************************************************/

/* Defines related to my UART library */
#define SEND_CMD        uartA2_tx_str
#define DISPLAY_RESP    uartA0_tx_str
#define RX_BUFF_SIZE    256

/* Definitions for BLE parameters */
#define BLE_DELAY_MS            50
#define BLE_CHK_CONN_RETRIES     3
#define BLE_SEND_DATA_RETRIES    3

/* Typdef Enum for Bluetooth States */
typedef enum{
    BLE_IDLE = 0,
    BLE_CHECK_CONNECTION,
    BLE_CONNECTED,
    BLE_SEND_DATA
}_E_BLE_STATE;

void reset_rx_buffer(char *buff);
void ble_pins_init(char *buff);
void dummy_uart_init();
uint8_t check_response(char * buff, char *resp);
int8_t ble_send_data(char *buff,_S_SENSOR_DATA * data);
int8_t ble_check_connection(char *buff);

/************************* END OF BLUETOOTH ***************************************************/

#endif /* SENSORS_H_ */
