
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
#include "algorithm.h"
#include "HR_RR_Algorithm.h"
#include "MAX30102.h"
#include "bme280_sensor.h"

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

/* Global Variable declaration related to ADC*/
uint8_t g_adc_state = 0;
uint32_t g_adcSamplingPeriod = ADC_SAMPLING_1MS;
uint32_t g_adcSamplingCount = 0;

/* Global Variable declaration related to Respiratory Rate*/
_E_RR_STATE g_rr_state = RR_INITIAL;
double g_rr_buff[RR_BUF_SIZE]={0.0};

uint16_t g_rr_sample_count = 0;
uint8_t g_rr_cal_signal = 0;

/* Global Variable declaration related to Heart Rate*/
_E_HR_STATE g_hr_state = HR_INITIAL;
uint8_t g_hr_valid = 1;
double g_hr_buff[RR_BUF_SIZE]={0.0};
uint16_t g_hr_sample_count = 0;
uint8_t g_hr_cal_signal = 0;

/* Global Variable Declarations for MAXREFDES117 */
uint32_t g_aun_ir_buffer[500]; //IR LED sensor data
uint32_t g_aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
uint8_t g_hr_spo2_state = 0;
uint32_t gIrRedCount = 0;

/* Global Variables for ADXL345 */
volatile uint16_t g_adxl345_interval = 0;
volatile uint8_t g_adxl345_flag = 0;

/* Global Variables for BME280 */
volatile uint32_t g_bme280_get_data_count = 0;
volatile uint8_t g_bme280_flag = 0;

void main(void)
{
    uint32_t i = 0;
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    P2->DIR &= ~(BIT6|BIT7);           // Make LO- and LO+ pins of Heart Rate Sensor as input inputs
    clk_init();
    dummy_uart_init();
    systick_init();
    ble_pins_init(g_rx_buff);
    /* Initialize BME280 sensor */
    bme280_sensor_init();
    accel_init();
    maxrefdes117_init();
    /* Initialize ADC with Channels 0 (for Respiratory Sensor) and 1 (Heart Rate Sensor)*/
    adc_init();
    /* Enable Global Interrupt */
    MAP_Interrupt_enableMaster();
    DISPLAY_RESP(" Program Started!\r\n");
    /* Start ADC conversion for both channels */
    MAP_ADC14_toggleConversionTrigger();

    while(1){

        if(g_adc_state == 2){
            if(g_rr_cal_signal == 1){ // calculate Respiratory Rate
                g_rr_cal_signal = 0;
                /* Have to calculate RR*/
                g_sensor_data.rr = calculate_RR(g_rr_buff);
                //printf("Br is %d\r\n",g_sensor_data.rr);
                //dumping the first X sets of samples and shift the last RR_BUF_SIZE-X sets of samples to the top
                for(i=RR_STABLE_BUF_SIZE;i<RR_BUF_SIZE;i++){
                    g_rr_buff[i-RR_STABLE_BUF_SIZE]=g_rr_buff[i];
                }
            }
            if(g_hr_cal_signal == 1){ // calculate Respiratory Rate
                g_hr_cal_signal = 0;
                if(g_hr_valid){
                    /* Have to calculate RR*/
                    g_sensor_data.hr= calculate_HR(g_hr_buff);
                }else{
                    g_sensor_data.hr = -999;
                }
                //printf("Hr is %d, valid = %d\r\n",g_sensor_data.hr,g_hr_valid);
                g_hr_valid = 1;
                //dumping the first X sets of samples and shift the last RR_BUF_SIZE-X sets of samples to the top
                for(i=RR_STABLE_BUF_SIZE;i<RR_BUF_SIZE;i++){
                    g_hr_buff[i-RR_STABLE_BUF_SIZE]=g_hr_buff[i];
                }
            }

            g_adc_state = 3;
        }
        /* If Sp02 buffer is full , this flag is 1*/
        if(g_hr_spo2_state == 1){
            /* get the calculated Sp02 value and valid flag*/
            maxim_heart_rate_and_oxygen_saturation(g_aun_ir_buffer, HR_SP02_BUF_SIZE, g_aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
            /* if the data is valid*/
            if(ch_spo2_valid){
                /* This was to get a more accurate value */
                n_sp02 -= 4;
            }
            g_sensor_data.sp02 = n_sp02;
            //printf("SpO2=%d, ", n_sp02);
            //printf("SPO2Valid=%d\n\r", ch_spo2_valid);

           /* Remove the oldest 100 samples and shift the next 400 samples to the beginning */
            for(i=HR_SP02_STABLE_BUF_SIZE;i<HR_SP02_BUF_SIZE;i++)
            {
                g_aun_red_buffer[i-HR_SP02_STABLE_BUF_SIZE]=g_aun_red_buffer[i];
                g_aun_ir_buffer[i-HR_SP02_STABLE_BUF_SIZE]=g_aun_ir_buffer[i];
            }
            gIrRedCount = HR_SP02_BUF_SIZE - HR_SP02_STABLE_BUF_SIZE;
            /* Change Sp02 State to 0 */
            g_hr_spo2_state = 0;
        }
        if(g_bme280_flag){
            update_temp_value(&g_sensor_data);
            g_bme280_flag  =0;
        }
        if(g_adxl345_flag){
            update_accel_params(&g_sensor_data);
            g_adxl345_flag = 0;
        }
        /* If BLE state is BLE_CHECK_CONNECTION*/
        if(g_ble_state == BLE_CHECK_CONNECTION){
            /* Check to see if our device is connected to other device */
            if(ble_check_connection(g_rx_buff) == -1){/* Device Connection Command failed */
                /* Change BLE State to BLE_IDLE */
                g_ble_state = BLE_IDLE;
                DISPLAY_RESP("OK not found for AT+GAPGETCONN\r\n");
                SEND_CMD("ATZ\r\n");
                systick_delay_ms(BLE_DELAY_MS);
                SEND_CMD("ATZ\r\n");
                systick_delay_ms(BLE_DELAY_MS);
//                SEND_CMD("ATZ\r\n");
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
                //printf("BLE Data Sent Successfully:%d!\r\n",++success);
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
/* Systick Handler */
void SysTick_Handler(){
    g_ms_timeout = 1;
    g_ticks++;
    if(!g_bme280_flag){
        if(++g_bme280_get_data_count >= 1000){
            g_bme280_flag = 1;
            g_bme280_get_data_count = 0;
        }
    }
    if(!g_adxl345_flag){
        if(++g_adxl345_interval >= 500){
            g_adxl345_flag = 1;
            g_adxl345_interval = 0;
        }
    }
    /* All Respiratory and Heart rate Calculations are done */
    if(g_adc_state == 3){
        /* Sampling Period is up */
        if(++g_adcSamplingCount >= g_adcSamplingPeriod){
            g_adcSamplingCount = 0;
            MAP_ADC14_toggleConversionTrigger();
            P1->OUT ^= 0x01;
            g_adc_state = 0;
        }
    }
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
/* Interrupt Handler for ADC */
void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);
    /* Interrupt for Channel 0 i.e Respiratory Rate */
    if (ADC_INT0 & status)
    {
        /* Store voltage in buffer */
        g_rr_buff[g_rr_sample_count++]= (MAP_ADC14_getResult(ADC_MEM0)* (3300.0/16383.0));
        /* Change ADC state to 1 */
        g_adc_state = 1;
        /* Check RR state */
        switch(g_rr_state){
           /* 1st run i.e 30 sec period */
            case RR_INITIAL:
                /* Check if buffer is full */
                if(g_rr_sample_count == RR_BUF_SIZE){
                    /* Update starting point */
                    g_rr_sample_count = RR_BUF_SIZE - RR_STABLE_BUF_SIZE;
                    /* Change RR state */
                    g_rr_state = RR_STABLE;
                    /* Update flag to perform Respiratory Calculation in Main*/
                    g_rr_cal_signal = 1;
                }
                break;
            /* Other runs i.e 4 sec period */
            case RR_STABLE:
                /* Check if buffer is full */
                if(g_rr_sample_count == RR_BUF_SIZE){
                    /* Update starting point */
                    g_rr_sample_count = RR_BUF_SIZE - RR_STABLE_BUF_SIZE;
                    /* Update flag to perform Respiratory Calculation in Main*/
                    g_rr_cal_signal = 1;
                }
                break;
            /* In case of Default do nothing */
            default:

                break;

        }

    }
    /* Interrupt for Channel 0 i.e Heart Rate */
    if (ADC_INT1 & status)
    {
        /* Checking Data Validity */
        if((P2->IN & 0xc0)!=0){
            g_hr_valid = 0;
        }
        /* Store voltage in buffer */
        g_hr_buff[g_hr_sample_count++]= (MAP_ADC14_getResult(ADC_MEM1)* (3300.0/16383.0));
        /* Change ADC state to 2 */
        g_adc_state = 2;
        /* Check HR state */
        switch(g_hr_state){
        /* 1st run i.e 30 sec period */
            case HR_INITIAL:
                /* Check if buffer is full */
                if(g_hr_sample_count == RR_BUF_SIZE){
                    /* Update starting point */
                    g_hr_sample_count = RR_BUF_SIZE - RR_STABLE_BUF_SIZE;
                    /* Change HR state */
                    g_hr_state = HR_STABLE;
                    /* Update flag to perform Respiratory Calculation in Main*/
                    g_hr_cal_signal = 1;
                }
                break;
             /* Other runs i.e 4 sec period */
            case HR_STABLE:
                /* Check if buffer is full */
                if(g_hr_sample_count == RR_BUF_SIZE){
                    /* Update starting point */
                    g_hr_sample_count = RR_BUF_SIZE - RR_STABLE_BUF_SIZE;
                    /* Update flag to perform Respiratory Calculation in Main*/
                    g_hr_cal_signal = 1;
                }
                break;
                /* In case of Default do nothing */
            default:

                break;

        }
    }
}
/* Port 4 GPIO interrupt Handler */
void PORT4_IRQHandler(){
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(MAX30102_INT_PORT);
    MAP_GPIO_clearInterruptFlag(MAX30102_INT_PORT, status);
    /* If the Pulse Oximetry Sensor sample is ready, this line will give an interrupt */
        if(status & MAX30102_INT_PIN){
            /* read the sample */
            maxim_max30102_read_fifo(&g_aun_red_buffer[gIrRedCount], &g_aun_ir_buffer[gIrRedCount]);  //read from MAX30102 FIFO
            if(g_hr_spo2_state == 0){
                /* If number of samples equals the Buffer size */
                if(++gIrRedCount == HR_SP02_BUF_SIZE){
                    /* Reset Sample count */
                    gIrRedCount = 0;
                    /* Change Sp02 State to 1 */
                    g_hr_spo2_state = 1;
                }
            }
        }
}
