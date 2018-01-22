/*
 * bme289_sensor.c
 *
 *  Created on: Mar 8, 2017
 *      Author: Atalville
 */
#include "driverlib.h"
#include "debug.h"
#include "I2C1.h"
#include "bme280.h"
#include "systick.h"
//extern uint32_t g_ticks = 0;
extern volatile uint32_t g_ticks;

#define BMP280_REG_START__ADDR     (0xF2)
#define BMP280_REG_DATA__LEN       (13)
#define BME280_DATA_INDEX   1
#define BME280_ADDRESS_INDEX    2

/************** I2C/SPI buffer length ******/
#define I2C_BUFFER_LEN 8

volatile uint32_t intervalCnt;
volatile uint32_t com_rslt;
volatile uint32_t MCLKfreq, SMCLKfreq;
volatile uint8_t ms_timeout;
volatile uint32_t uncompTemp;
volatile uint8_t regVal;
volatile uint8_t regData[13];

volatile struct bme280_t bme280;
volatile struct bme280_t *p_bme280; /**< pointer to BME280 */
#define ATITUDE_OFFSET         23.4
#define INCH_HG_CONV_FACTOR    0.02953


#define BME280_delay_msek       systick_delay_ms

float altitude_compensated_pressure_inchHg(float pressure){
    pressure += ATITUDE_OFFSET;
    return pressure * INCH_HG_CONV_FACTOR;
}
//void bme280_1ms_handler()
//{
//    ms_timeout=1;  // set flag for timeout of systick
//    intervalCnt++;  // increment counter for interval timer (reset when needed)
//}
/*-------------------------------------------------------------------*
*   This is a sample code for read and write the data by using I2C/SPI
*   Use either I2C or SPI based on your need
*   The device address defined in the bme280.h file
*-----------------------------------------------------------------------*/
 /* \Brief: The function is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;
    u8 array[I2C_BUFFER_LEN];
    u8 stringpos = BME280_INIT_VALUE;
    array[BME280_INIT_VALUE] = reg_addr;
    uint8_t i;
    uint16_t rtnval, debugdump;
    uint32_t ticks = 0;

    for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
        array[stringpos + BME280_DATA_INDEX] = *(reg_data + stringpos);
    }
    /*
    * Please take the below function as your reference for
    * write the data using I2C communication
    * "IERROR = I2C_WRITE_STRING(DEV_ADDR, array, cnt+1)"
    * add your I2C write function here
    * iError is an return value of I2C read function
    * Please select your valid return value
    * In the driver SUCCESS defined as 0
    * and FAILURE defined as -1
    * Note :
    * This is a full duplex operation,
    * The first read data is discarded, for that extra write operation
    * have to be initiated. For that cnt+1 operation done in the I2C write string function
    * For more information please refer data sheet SPI communication:
    */
    ticks = g_ticks;
    while(UCB0STATW&0x0010){
        if(g_ticks - ticks >= 10){
            debugdump = UCB0IFG;           // snapshot flag register for calling program
            I2C1_Init();                    // reset to known state
            return iError=-1;
        }
    };         // wait for I2C ready
    UCB0CTLW0 |= 0x0001;               // hold the eUSCI module in reset mode
    UCB0TBCNT = cnt+1;                     // generate stop condition after this many bytes
    UCB0CTLW0 &= ~0x0001;              // enable eUSCI module
    UCB0I2CSA = dev_addr;              // I2CCSA[6:0] is slave address
    UCB0CTLW0 = ((UCB0CTLW0&~0x0004)   // clear bit2 (UCTXSTP) for no transmit stop condition
                                       // set bit1 (UCTXSTT) for transmit start condition
                  | 0x0012);           // set bit4 (UCTR) for transmit mode
    ticks = g_ticks;
    while((UCB0IFG&0x0002) == 0){
        if(g_ticks - ticks >= 10){
            debugdump = UCB0IFG;           // snapshot flag register for calling program
            I2C1_Init();                    // reset to known state
            return iError=-1;
        }
    };    // wait for slave address sent

    for(i=0; i<cnt; i++) {
        UCB0TXBUF = array[i]&0xFF;         // TXBUF[7:0] is data
        ticks = g_ticks;
        while((UCB0IFG&0x0002) == 0){      // wait for data sent
            if(UCB0IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
                debugdump = UCB0IFG;           // snapshot flag register for calling program
                I2C1_Init();                    // reset to known state
                return iError=-1;
            }
            if(g_ticks - ticks >= 10){
                debugdump = UCB0IFG;           // snapshot flag register for calling program
                I2C1_Init();                    // reset to known state
                return iError=-1;
            }
        }
    }
    UCB0TXBUF = array[i]&0xFF;         // TXBUF[7:0] is last data
    ticks = g_ticks;
    while(UCB0STATW&0x0010){           // wait for I2C idle
      if(UCB0IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
        debugdump = UCB0IFG;           // snapshot flag register for calling program
        I2C1_Init();                    // reset to known state
        return iError=-1;
      }
      if(g_ticks - ticks >= 10){
          debugdump = UCB0IFG;           // snapshot flag register for calling program
          I2C1_Init();                    // reset to known state
          return iError=-1;
      }
    }
    return iError=0;
}

 /* \Brief: The function is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be read
 *  \param reg_data : This data read from the sensor, which is hold in an array
 *  \param cnt : The no of data byte of to be read
 */
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;
    uint8_t i;
    uint16_t rtnval, debugdump;
    u8 array[I2C_BUFFER_LEN] = {BME280_INIT_VALUE};
    u8 stringpos = BME280_INIT_VALUE;
    array[BME280_INIT_VALUE] = reg_addr;
    uint32_t ticks = 0;
    /* Please take the below function as your reference
     * for read the data using I2C communication
     * add your I2C read function here.
     * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
     * iError is an return value of write function
     * Please select your valid return value
     * In the driver SUCCESS defined as 0
     * and FAILURE defined as -1
     */

    // set pointer to register address
    ticks = g_ticks;
    while(UCB0STATW&0x0010){
        if(g_ticks - ticks >= 10){
            debugdump = UCB0IFG;           // snapshot flag register for calling program
            I2C1_Init();                    // reset to known state
            return iError=-1;
        }
    };         // wait for I2C ready
    UCB0CTLW0 |= 0x0001;               // hold the eUSCI module in reset mode
    UCB0TBCNT = 1;                     // generate stop condition after this many bytes
    UCB0CTLW0 &= ~0x0001;              // enable eUSCI module
    UCB0I2CSA = dev_addr;                 // I2CCSA[6:0] is slave address
    UCB0CTLW0 = ((UCB0CTLW0&~0x0004)   // clear bit2 (UCTXSTP) for no transmit stop condition
                                       // set bit1 (UCTXSTT) for transmit start condition
                  | 0x0012);           // set bit4 (UCTR) for transmit mode
    ticks = g_ticks;
    while(UCB0CTLW0&0x0002){
        if(g_ticks - ticks >= 10){
            debugdump = UCB0IFG;           // snapshot flag register for calling program
            I2C1_Init();                    // reset to known state
            return iError=-1;
        }
    };         // wait for slave address sent
    UCB0TXBUF = reg_addr&0xFF;            // TXBUF[7:0] is data
    ticks = g_ticks;
    while(UCB0STATW&0x0010){           // wait for I2C idle
      if(UCB0IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
        debugdump = UCB0IFG;           // snapshot flag register for calling program
        I2C1_Init();                    // reset to known state
        return iError=-1;
      }
      if(g_ticks - ticks >= 10){
          debugdump = UCB0IFG;           // snapshot flag register for calling program
          I2C1_Init();                    // reset to known state
          return iError=-1;
      }
    }

    // receive bytes from registers on BME280 device
    ticks = g_ticks;
    while(UCB0STATW&0x0010){
        if(g_ticks - ticks >= 10){
            debugdump = UCB0IFG;           // snapshot flag register for calling program
            I2C1_Init();                    // reset to known state
            return iError=-1;
        }
    };         // wait for I2C ready
    UCB0CTLW0 |= 0x0001;               // hold the eUSCI module in reset mode
    UCB0TBCNT = cnt;                     // generate stop condition after this many bytes
    UCB0CTLW0 &= ~0x0001;              // enable eUSCI module
    UCB0I2CSA = dev_addr;                 // I2CCSA[6:0] is slave address
    UCB0CTLW0 = ((UCB0CTLW0&~0x0014)   // clear bit4 (UCTR) for receive mode
                                       // clear bit2 (UCTXSTP) for no transmit stop condition
                  | 0x0002);           // set bit1 (UCTXSTT) for transmit start condition
    for(i=0; i<cnt; i++) {
        ticks = g_ticks;
        while((UCB0IFG&0x0001) == 0){      // wait for complete character received
          if(UCB0IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
            I2C1_Init();                    // reset to known state
            return 0xFFFF;
          }
          if(g_ticks - ticks >= 10){
              debugdump = UCB0IFG;           // snapshot flag register for calling program
              I2C1_Init();                    // reset to known state
              return iError=-1;
          }
        }
        *reg_data++= UCB0RXBUF&0xFF;            // get the reply
    }

    return (s8)iError;
}

/*  Brief : The delay routine
 *  \param : delay in ms
*/
//void BME280_delay_msek(u32 msek)
//{
//    uint32_t i;
//    /*Here you can write your own delay routine*/
//    SysTick -> VAL = 0;  // any write to CVR clears it to start a time interval
//    ms_timeout=0; // clear the flag that is set by the SysTick timer every 1 ms
//    for(i=0; i<msek; i++) {
//        while(ms_timeout==0);  // delay for 1 ms
//        ms_timeout=0;
//    }
//}
/*--------------------------------------------------------------------------*
*   The following function is used to map the I2C bus read, write, delay and
*   device address with global structure bme280
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bme280 the following structure parameter can be accessed
 *  Bus write function pointer: BME280_WR_FUNC_PTR
 *  Bus read function pointer: BME280_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *--------------------------------------------------------------------------*/
    bme280.bus_write = BME280_I2C_bus_write;
    bme280.bus_read = BME280_I2C_bus_read;
    bme280.dev_addr = BME280_I2C_ADDRESS1;
    bme280.delay_msec = BME280_delay_msek;

    return BME280_INIT_VALUE;
}
void bme280_sensor_init(){
    /* Select Port 6 for I2C - Set Pin 4, 5 to input Primary Module Function,
     *   (UCB1SIMO/UCB1SDA, UCB1SOMI/UCB1SCL).
     */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    I2C1_Init();  // initialize eUSCI
    /* The variable used to assign the standby time*/
    u8 v_stand_by_time_u8 = BME280_INIT_VALUE;
    /* The variable used to read uncompensated temperature*/
    s32 v_data_uncomp_temp_s32 = BME280_INIT_VALUE;
    /* The variable used to read uncompensated pressure*/
    s32 v_data_uncomp_pres_s32 = BME280_INIT_VALUE;
    /* The variable used to read uncompensated pressure*/
    s32 v_data_uncomp_hum_s32 = BME280_INIT_VALUE;
    /* The variable used to read compensated temperature*/
    s32 v_comp_temp_s32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    /* The variable used to read compensated pressure*/
    u32 v_comp_press_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    /* The variable used to read compensated humidity*/
    u32 v_comp_humidity_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};

    /* result of communication results*/
    s32 com_rslt = ERROR;

 /*********************** START INITIALIZATION ************************/
  /*    Based on the user need configure I2C or SPI interface.
  * It is example code to explain how to use the bme280 API*/
    I2C_routine();
    /*SPI_routine();*/
/*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *  the following parameters
 *  I2C address
 *  Bus Write
 *  Bus read
 *  Chip id
*-------------------------------------------------------------------------*/
    com_rslt = bme280_init(&bme280);

    /*  For initialization it is required to set the mode of
     *  the sensor as "NORMAL"
     *  data acquisition/read/write is possible in this mode
     *  by using the below API able to set the power mode as NORMAL*/
    /* Set the power mode as NORMAL*/
    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
    /*  For reading the pressure, humidity and temperature data it is required to
     *  set the OSS setting of humidity, pressure and temperature
     * The "BME280_CTRLHUM_REG_OSRSH" register sets the humidity
     * data acquisition options of the device.
     * changes to this registers only become effective after a write operation to
     * "BME280_CTRLMEAS_REG" register.
     * In the code automated reading and writing of "BME280_CTRLHUM_REG_OSRSH"
     * register first set the "BME280_CTRLHUM_REG_OSRSH" and then read and write
     * the "BME280_CTRLMEAS_REG" register in the function*/
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

    /* set the pressure oversampling*/
    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_1X);
    /* set the temperature oversampling*/
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_1X);
/*--------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
    /* This API used to Write the standby time of the sensor input
     *  value have to be given
     *  Normal mode comprises an automated perpetual cycling between an (active)
     *  Measurement period and an (inactive) standby period.
     *  The standby time is determined by the contents of the register t_sb.
     *  Standby time can be set using BME280_STANDBYTIME_125_MS.
     *  Usage Hint : bme280_set_standbydur(BME280_STANDBYTIME_125_MS)*/

  com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
//    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1000_MS);

    /* This API used to read back the written value of standby time*/
    com_rslt += bme280_get_standby_durn(&v_stand_by_time_u8);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*------------------------------------------------------------------*/
    // get registers

    BME280_I2C_bus_read(p_bme280->dev_addr, 0xF2, regData, 12);
/************************* END INITIALIZATION *************************/
    /*-----------------------------------------------------------------------*
    ************************* START DE-INITIALIZATION ***********************
    *-------------------------------------------------------------------------*/
        /*  For de-initialization it is required to set the mode of
         *  the sensor as "SLEEP"
         *  the device reaches the lowest power consumption only
         *  In SLEEP mode no measurements are performed
         *  All registers are accessible
         *  by using the below API able to set the power mode as SLEEP*/
         /* Set the power mode as SLEEP*/
        com_rslt += bme280_set_power_mode(BME280_SLEEP_MODE);
    /*---------------------------------------------------------------------*
    ************************* END DE-INITIALIZATION **********************
    *---------------------------------------------------------------------*/
}
void get_bme280_values(float *temp, float * press, float *hum){
    uint32_t i = 0;
    s32 com_rslt = ERROR;
    /* The variable used to assign the standby time*/
    u8 v_stand_by_time_u8 = BME280_INIT_VALUE;
    /* The variable used to read uncompensated temperature*/
    s32 v_data_uncomp_temp_s32 = BME280_INIT_VALUE;
    /* The variable used to read uncompensated pressure*/
    s32 v_data_uncomp_pres_s32 = BME280_INIT_VALUE;
    /* The variable used to read uncompensated pressure*/
    s32 v_data_uncomp_hum_s32 = BME280_INIT_VALUE;
    /* The variable used to read compensated temperature*/
    s32 v_comp_temp_s32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    /* The variable used to read compensated pressure*/
    u32 v_comp_press_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    /* The variable used to read compensated humidity*/
    u32 v_comp_humidity_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
    com_rslt= bme280_set_power_mode(0x01); // set power mode to forced to generate reading

//    for(i=0; i<10; i++) {
//        ms_timeout=0;  // clear flag for Systick timer
//        while(!ms_timeout); // wait for it to be set again 1 ms later
//    }
    BME280_delay_msek(50);
    // get registers

    BME280_I2C_bus_read(p_bme280->dev_addr, 0xF2, regData, 12);

    /*------------------------------------------------------------------*
    ************ START READ UNCOMPENSATED PRESSURE, TEMPERATURE
    AND HUMIDITY DATA ********
    *---------------------------------------------------------------------*/
        /* API is used to read the uncompensated temperature*/
        com_rslt += bme280_read_uncomp_temperature(&v_data_uncomp_temp_s32);

        /* API is used to read the uncompensated pressure*/
        com_rslt += bme280_read_uncomp_pressure(&v_data_uncomp_pres_s32);

        /* API is used to read the uncompensated humidity*/
        com_rslt += bme280_read_uncomp_humidity(&v_data_uncomp_hum_s32);

        /* API is used to read the uncompensated temperature,pressure
        and humidity data */
//            com_rslt += bme280_read_uncomp_pressure_temperature_humidity(
//            &v_data_uncomp_temp_s32, &v_data_uncomp_pres_s32, &v_data_uncomp_hum_s32);
    /*--------------------------------------------------------------------*
    ************ END READ UNCOMPENSATED PRESSURE AND TEMPERATURE********
    *-------------------------------------------------------------------------*/

    /*------------------------------------------------------------------*

    ************ START READ COMPENSATED PRESSURE, TEMPERATURE
    AND HUMIDITY DATA ********
    *---------------------------------------------------------------------*/
        /* API is used to compute the compensated temperature*/
        v_comp_temp_s32[0] = bme280_compensate_temperature_int32(
                v_data_uncomp_temp_s32);

        /* API is used to compute the compensated pressure*/
        v_comp_press_u32[0] = bme280_compensate_pressure_int32(
                v_data_uncomp_pres_s32);

        /* API is used to compute the compensated humidity*/
        v_comp_humidity_u32[0] = bme280_compensate_humidity_int32(
                v_data_uncomp_hum_s32);

        /* API is used to read the compensated temperature, humidity and pressure*/
//            com_rslt += bme280_read_pressure_temperature_humidity(
//            &v_comp_press_u32[1], &v_comp_temp_s32[1],  &v_comp_humidity_u32[1]);
    /*--------------------------------------------------------------------*
    ************ END READ COMPENSATED PRESSURE, TEMPERATURE AND HUMIDITY ********
    *-------------------------------------------------------------------------*/
        //printf("RAW: temp=%d, pressure=%d, humidity=%d\r\n",v_comp_temp_s32[0],v_comp_press_u32[0],v_comp_humidity_u32[0]);
        *temp  = v_comp_temp_s32[0] * 0.01; //Centigrade
        *press =  v_comp_press_u32[0] * 0.01; //mBar
        *hum = v_comp_humidity_u32[0] / 1024.0;// %rH
        *press = altitude_compensated_pressure_inchHg(*press);
        //printf("Converted: temp=%d C, pressure=%d mBar, humidity=%d%%rh\r\n",v_comp_temp_s32[0],v_comp_press_u32[0],v_comp_humidity_u32[0]);


    /*-----------------------------------------------------------------------*/

    // delay for 5 seconds (5000 ms)
//    for(i=0; i<5000; i++) {
//        ms_timeout=0;  // clear flag for Systick timer
//        while(!ms_timeout); // wait for it to be set again 1 ms later
//    }
}


