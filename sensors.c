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
#include "adc.h"
#include "HR_RR_Algorithm.h"
#include "msp432_spi.h"
#include "adxl345.h"
#include "I2C0.h"
#include "MAX30102.h"
#include "I2C1.h"
#include "bme280_sensor.h"
/******************************************************** BLUETOOTH ***********************************************************/

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
        //__delay_cycles(320000);
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
        //__delay_cycles(320000);
        i++;
    }while(check_response(buff,"OK\r\n")!=1);/* Wait until OK is found */
    /* if 1 found with OK, device is connected */
    if(check_response(buff,"1\r\nOK\r\n")){
        return 1;
    }else{ /* if 0 found with OK, device is not connected */
        return 0;
    }
}
/*************************************************** END OF BLUETOOTH *********************************************************************/

/*************************************************** RESPIRATORY RATE and HEART RATE ***********************************************************/
/* Global Variable declarations for Respiratory rate */
double g_rr_temp_buff[RR_BUF_SIZE]={0.0};
double g_rr_filter_values[RR_BUF_SIZE]={0.0};

/* Global Variable declarations for Heart rate */
double g_hr_temp_buff[RR_BUF_SIZE]={0.0};
double g_hr_filter_values[RR_BUF_SIZE]={0.0};

/* Function to calculate Respiratory Rate from Samples */
uint16_t calculate_RR(double *samples){
    double threshold= 0;
    int16_t peaks = 0;
    /* Find Mean */
    double avg = find_mean(samples);
    /* Find difference of mean to remove DC offset */
    diff_from_mean(samples,g_rr_temp_buff,avg);
    /* 4 point Moving Average to smoothen the signal */
    four_pt_MA(g_rr_temp_buff);
    /* 6th order Butterworth low pass filter with center frequency 2Hz */
    ButterworthLowpassFilter0100SixthOrder(g_rr_temp_buff,g_rr_filter_values,RR_BUF_SIZE-MA4_SIZE+1);
    /* Calculate threshold */
    threshold = threshold_calc(g_rr_filter_values) * 0.7;
    /* Calculate Number of peaks */
    peaks= myPeakCounter(g_rr_filter_values,RR_BUF_SIZE-MA4_SIZE+1,threshold);
    //printf("BR Peaks = %d, ",peaks);
    /* Extrapolate data to get per min value */
    return (60/RR_INITIAL_FRAME_TIME_S) * peaks;
}

/* Function to calculate Heart Rate from Samples */
uint16_t calculate_HR(double *samples){
    double threshold= 0;
    int16_t peaks = 0;
    /* Find Mean */
    double avg = find_mean(samples);
    /* Find Difference from Mean to remove DC offset */
    diff_from_mean(samples,g_hr_temp_buff,avg);
    /* 4 point moving average to smoothen the signal */
    four_pt_MA(g_hr_temp_buff);
    /* 6th order Butterworth low pass filter with center frequency 2Hz */
    ButterworthLowpassFilter0100SixthOrder(g_hr_temp_buff,g_hr_filter_values,RR_BUF_SIZE-MA4_SIZE+1);
    /* Calculate threshold */
    threshold = threshold_calc(g_hr_filter_values) * 0.7;
    /* Calculate Number of peaks */
    peaks= myPeakCounter(g_hr_filter_values,RR_BUF_SIZE-MA4_SIZE+1,threshold);
    //printf(" HR Peaks = %d, ",peaks);
    /* Extrapolate data to get per min value */
    return (60/RR_INITIAL_FRAME_TIME_S) * peaks;

}
/*************************************************** END OF RESPIRATORY RATE and HEART RATE ***********************************************************/

/*************************************************** PULSE OXIMETRY  ***********************************************************/


/* Function to Initialize Pulse Oximetry Sensor */
void maxrefdes117_init(){
    uint8_t id = 0xff;
    uint8_t uch_dummy;

    /* Select Port 6 for I2C - Set Pin 4, 5 to input Primary Module Function,
     *   (UCB1SIMO/UCB1SDA, UCB1SOMI/UCB1SCL).
     */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
            GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsInputPinWithPullUpResistor(MAX30102_INT_PORT, MAX30102_INT_PIN);
    GPIO_interruptEdgeSelect(MAX30102_INT_PORT, MAX30102_INT_PIN,GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(MAX30102_INT_PORT, MAX30102_INT_PIN);
    MAP_GPIO_enableInterrupt(MAX30102_INT_PORT, MAX30102_INT_PIN);
    MAP_Interrupt_enableInterrupt(INT_PORT4);
    /* Initialize I2C0 */
    I2C0_Init();
    printf("Program started!\r\n");
    /* Read Device ID. Device ID is 0x15 */
    I2C0_bus_read(0x57, 0xff, &id, 1);
    printf("id = 0x%x\r\n",id);

    maxim_max30102_reset(); //resets the MAX30102
    //read and clear status register
    maxim_max30102_read_reg(0,&uch_dummy);
    maxim_max30102_init();  //initializes the MAX30102
}
/*************************************************** END OF PULSE OXIMETRY  ***********************************************************/


/*************************************************** ACCELEROMETER *****************************************************************/
/* Function absolute value */
__inline static float absFloat(float x){
    if(x < 0){
        return -x;
    }
    return x;
}
void accel_init(){
    uint8_t readValue = 0;
    /* read DeviceID of the Accelereomter. The DeviceID is 0xE5 */
    readValue = r_reg(0x00);
    printf("device id = 0x%x\r\n",readValue);
    /* Power on the accelerometer */
    ADXL345_powerOn();
    /* Set Acceleromter Range to 16G */
    ADXL345_setRange(ADXL345_RANGE_16_G);
    /* Set Accelerometer Data rate to 400Hz */
    ADXL345_setDataRate(ADXL345_DATARATE_400_HZ);
}
/* Function to check orientation based on x,y,z coordinates */
__inline static void checkOrientation(_S_SENSOR_DATA *buff, float x, float y,float z){
   const float zUpThreshold = 0.7;
   const float xLeftThreshold = 0.5;
   const float xRightThreshold = -0.5;
   const float yUpRightThreshold = 0.7;
   const float zDownThreshold = -0.6;
    if(z > zUpThreshold){
        strcpy(buff->accel_buf,"UP");
    }else if(z < zDownThreshold){
        strcpy(buff->accel_buf,"DOWN");
    }
    else if ((z < zUpThreshold) && (x> xLeftThreshold)){
        strcpy(buff->accel_buf,"LEFT");
    }else if ((z < zUpThreshold) && (x< xRightThreshold)){
        strcpy(buff->accel_buf,"RIGHT");
    }else if(absFloat(y) > yUpRightThreshold  ){
        strcpy(buff->accel_buf,"UPRIGHT");
    }
}
void update_accel_params(_S_SENSOR_DATA *buff){
    float a,b,c;
    //static float x_avg = 0.0,y_avg = 0.0,z_avg = 0.0;
    //static uint8_t n_samples = 0;
    //lis3dh_readNormalizedData(&a,&b,&c);
    ADXL345_readNormalizedAccel(&a,&b,&c);
    checkOrientation(buff,a,b,c);
//    if((a <= 1.0 && a>= -1.0) && (b <= 1.0 && b >= -1.0) && (c <= 1.0 && c >= -1.0)){
//        x_avg += a;
//        y_avg += b;
//        z_avg += c;
//        if(++n_samples == N_ACCEL_SAMPLES){
//            x_avg = x_avg/N_ACCEL_SAMPLES;
//            y_avg = y_avg/N_ACCEL_SAMPLES;
//            z_avg = z_avg/N_ACCEL_SAMPLES;
//            checkOrientation(buff,x_avg,y_avg,z_avg);
//            //printf("Normalized:x = %.2f, y = %.2f, z = %.2f\r\n",g_sensor_data.xpos,g_sensor_data.ypos,g_sensor_data.zpos);
//            n_samples = 0;
//            x_avg = 0;
//            y_avg = 0;
//            z_avg = 0;
//
//        }
//    }
}
/*************************************************** END OF ACCELEROMETER  ***********************************************************/
/*************************************************** TEMPERATURE SENSOR  *****************************************************************/
/* Variable declarations for BME280 */
_S_BME280_DATA bme280_data = {-999.9,-999.9,-999.9,-999.9};
void update_temp_value(_S_SENSOR_DATA *buff){
    get_bme280_values(&bme280_data.temperature_c,&bme280_data.pressure,&bme280_data.humidity);
    bme280_data.temperature_f = bme280_data.temperature_c * 1.8 + 32; /*T(°F) = T(°C) × 1.8 + 32*/
    buff->temp_f =bme280_data.temperature_f;
    //printf("MAIN: tempC=%.2f C,tempF = %.2f F\r\n",bme280_data.temperature_c,bme280_data.temperature_f);
}
/*************************************************** END OF TEMPERATURE SENSOR  *****************************************************************/
