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

/* Functions related to Bluetooth */
void reset_rx_buffer(char *buff);
void ble_pins_init(char *buff);
void dummy_uart_init();
uint8_t check_response(char * buff, char *resp);
int8_t ble_send_data(char *buff,_S_SENSOR_DATA * data);
int8_t ble_check_connection(char *buff);

/************************* END OF BLUETOOTH *************************************************************************/

/************************* RESPIRATORY RATE and HEART RATE ***********************************************************/

#define ADC_SAMPLING_1MS     50 // i.e 20 SPS

/* Functions related to Respiratory and Heart Rate  */
uint16_t calculate_RR(double *samples);
uint16_t calculate_HR(double *samples);


/************************* END OF RESPIRATORY RATE and HEART RATE ***********************************************************/

/*************************************************** PULSE OXIMETRY  ***********************************************************/
/* Definitions for MAXREFDES117 */
#define MAX30102_INT_PORT    GPIO_PORT_P4
#define MAX30102_INT_PIN     GPIO_PIN6

void maxrefdes117_init();

/*************************************************** END OF PULSE OXIMETRY  ***********************************************************/

/*************************************************** ACCELEROMETER *****************************************************************/
#define N_ACCEL_SAMPLES     1

//void checkOrientation(float x, float y,float z);
void accel_init();
void update_accel_params(_S_SENSOR_DATA *buff);
/*************************************************** END OF ACCELEROMETER  ***********************************************************/

/*************************************************** TEMPERATURE SENSOR  *****************************************************************/
/* Typedef of structure for BME Sensor */
typedef struct {
    float temperature_c;
    float temperature_f;
    float pressure;
    float humidity;
}_S_BME280_DATA;
void update_temp_value(_S_SENSOR_DATA *buff);
/*************************************************** END OF TEMPERATURE SENSOR  *****************************************************************/


#endif /* SENSORS_H_ */
