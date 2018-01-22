/*
 * lis3dh.h
 *
 *  Created on: Aug 29, 2017
 *      Author: Atalville
 */

#ifndef ADXL345_H_
#define ADXL345_H_
#include "driverlib.h"

#define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb
#define SENSORS_GRAVITY_STANDARD           (1.0)

#define CS_LOW()   GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
#define CS_HIGH()   GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);



/*************************** REGISTER MAP ***************************/
#define ADXL345_DEVID           0x00        // Device ID
#define ADXL345_RESERVED1       0x01        // Reserved. Do Not Access.
#define ADXL345_THRESH_TAP      0x1D        // Tap Threshold.
#define ADXL345_OFSX            0x1E        // X-Axis Offset.
#define ADXL345_OFSY            0x1F        // Y-Axis Offset.
#define ADXL345_OFSZ            0x20        // Z- Axis Offset.
#define ADXL345_DUR             0x21        // Tap Duration.
#define ADXL345_LATENT          0x22        // Tap Latency.
#define ADXL345_WINDOW          0x23        // Tap Window.
#define ADXL345_THRESH_ACT      0x24        // Activity Threshold
#define ADXL345_THRESH_INACT    0x25        // Inactivity Threshold
#define ADXL345_TIME_INACT      0x26        // Inactivity Time
#define ADXL345_ACT_INACT_CTL   0x27        // Axis Enable Control for Activity and Inactivity Detection
#define ADXL345_THRESH_FF       0x28        // Free-Fall Threshold.
#define ADXL345_TIME_FF         0x29        // Free-Fall Time.
#define ADXL345_TAP_AXES        0x2A        // Axis Control for Tap/Double Tap.
#define ADXL345_ACT_TAP_STATUS  0x2B        // Source of Tap/Double Tap
#define ADXL345_BW_RATE         0x2C        // Data Rate and Power mode Control
#define ADXL345_POWER_CTL       0x2D        // Power-Saving Features Control
#define ADXL345_INT_ENABLE      0x2E        // Interrupt Enable Control
#define ADXL345_INT_MAP         0x2F        // Interrupt Mapping Control
#define ADXL345_INT_SOURCE      0x30        // Source of Interrupts
#define ADXL345_DATA_FORMAT     0x31        // Data Format Control
#define ADXL345_DATAX0          0x32        // X-Axis Data 0
#define ADXL345_DATAX1          0x33        // X-Axis Data 1
#define ADXL345_DATAY0          0x34        // Y-Axis Data 0
#define ADXL345_DATAY1          0x35        // Y-Axis Data 1
#define ADXL345_DATAZ0          0x36        // Z-Axis Data 0
#define ADXL345_DATAZ1          0x37        // Z-Axis Data 1
#define ADXL345_FIFO_CTL        0x38        // FIFO Control
#define ADXL345_FIFO_STATUS     0x39        // FIFO Status

#define ADXL345_BW_1600         0xF         // 1111     IDD = 40uA
#define ADXL345_BW_800          0xE         // 1110     IDD = 90uA
#define ADXL345_BW_400          0xD         // 1101     IDD = 140uA
#define ADXL345_BW_200          0xC         // 1100     IDD = 140uA
#define ADXL345_BW_100          0xB         // 1011     IDD = 140uA
#define ADXL345_BW_50           0xA         // 1010     IDD = 140uA
#define ADXL345_BW_25           0x9         // 1001     IDD = 90uA
#define ADXL345_BW_12_5         0x8         // 1000     IDD = 60uA
#define ADXL345_BW_6_25         0x7         // 0111     IDD = 50uA
#define ADXL345_BW_3_13         0x6         // 0110     IDD = 45uA
#define ADXL345_BW_1_56         0x5         // 0101     IDD = 40uA
#define ADXL345_BW_0_78         0x4         // 0100     IDD = 34uA
#define ADXL345_BW_0_39         0x3         // 0011     IDD = 23uA
#define ADXL345_BW_0_20         0x2         // 0010     IDD = 23uA
#define ADXL345_BW_0_10         0x1         // 0001     IDD = 23uA
#define ADXL345_BW_0_05         0x0         // 0000     IDD = 23uA


 /************************** INTERRUPT PINS **************************/
#define ADXL345_INT1_PIN        0x00        //INT1: 0
#define ADXL345_INT2_PIN        0x01        //INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define ADXL345_INT_DATA_READY_BIT      0x07
#define ADXL345_INT_SINGLE_TAP_BIT      0x06
#define ADXL345_INT_DOUBLE_TAP_BIT      0x05
#define ADXL345_INT_ACTIVITY_BIT        0x04
#define ADXL345_INT_INACTIVITY_BIT      0x03
#define ADXL345_INT_FREE_FALL_BIT       0x02
#define ADXL345_INT_WATERMARK_BIT       0x01
#define ADXL345_INT_OVERRUNY_BIT        0x00

#define ADXL345_DATA_READY              0x07
#define ADXL345_SINGLE_TAP              0x06
#define ADXL345_DOUBLE_TAP              0x05
#define ADXL345_ACTIVITY                0x04
#define ADXL345_INACTIVITY              0x03
#define ADXL345_FREE_FALL               0x02
#define ADXL345_WATERMARK               0x01
#define ADXL345_OVERRUNY                0x00


 /****************************** ERRORS ******************************/
#define ADXL345_OK          1       // No Error
#define ADXL345_ERROR       0       // Error Exists

#define ADXL345_NO_ERROR    0       // Initial State
#define ADXL345_READ_ERROR  1       // Accelerometer Reading Error
#define ADXL345_BAD_ARG     2       // Bad Argument


/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G          = 0b11,   // +/- 16g
  ADXL345_RANGE_8_G           = 0b10,   // +/- 8g
  ADXL345_RANGE_4_G           = 0b01,   // +/- 4g
  ADXL345_RANGE_2_G           = 0b00    // +/- 2g (default value)
} range_t;
/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ    = 0b1111, // 1600Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_1600_HZ    = 0b1110, //  800Hz Bandwidth    90µA IDD
  ADXL345_DATARATE_800_HZ     = 0b1101, //  400Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_400_HZ     = 0b1100, //  200Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_200_HZ     = 0b1011, //  100Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_100_HZ     = 0b1010, //   50Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_50_HZ      = 0b1001, //   25Hz Bandwidth    90µA IDD
  ADXL345_DATARATE_25_HZ      = 0b1000, // 12.5Hz Bandwidth    60µA IDD
  ADXL345_DATARATE_12_5_HZ    = 0b0111, // 6.25Hz Bandwidth    50µA IDD
  ADXL345_DATARATE_6_25HZ     = 0b0110, // 3.13Hz Bandwidth    45µA IDD
  ADXL345_DATARATE_3_13_HZ    = 0b0101, // 1.56Hz Bandwidth    40µA IDD
  ADXL345_DATARATE_1_56_HZ    = 0b0100, // 0.78Hz Bandwidth    34µA IDD
  ADXL345_DATARATE_0_78_HZ    = 0b0011, // 0.39Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_39_HZ    = 0b0010, // 0.20Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_20_HZ    = 0b0001, // 0.10Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_10_HZ    = 0b0000  // 0.05Hz Bandwidth    23µA IDD (default value)
} dataRate_t;



uint8_t r_reg(uint8_t addr);
void w_reg(uint8_t addr, uint8_t data);
void ADXL345_powerOn();
void ADXL345_readAccel(int *x, int *y, int *z);
void ADXL345_readNormalizedAccel(float *x, float *y, float *z);
void ADXL345_setRange(range_t range);
range_t ADXL345_getRange(void);
void ADXL345_setDataRate(dataRate_t dataRate);
dataRate_t ADXL345_getDataRate(void);

#endif /* ADXL345_H_ */
