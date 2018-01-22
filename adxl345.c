/*
 * lis3dh.c
 *
 *  Created on: Aug 29, 2017
 *      Author: Atalville
 */

#include "adxl345.h"
#include "driverlib.h"
#include "msp432_spi.h"


uint8_t r_reg(uint8_t addr)
{
    uint16_t i;

    CS_LOW()
    i = spi_transfer16(0xff | ((addr | 0x80) << 8));
    CS_HIGH()
    return (uint8_t) (i & 0x00FF);
}
void w_reg(uint8_t addr, uint8_t data)
{
    CS_LOW()
    spi_transfer16( (data & 0xFF) | (addr << 8) );
    CS_HIGH()
}
void ADXL345_powerOn() {
    w_reg(ADXL345_POWER_CTL, 0);  // Wakeup
    w_reg(ADXL345_POWER_CTL, 16); // Auto_Sleep
    w_reg(ADXL345_POWER_CTL, 8);  // Measure
}
void ADXL345_readAccel(int *x, int *y, int *z) {
    uint8_t _buff[6];
    //readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff);   // Read Accel Data from ADXL345
    _buff[0] =r_reg(ADXL345_DATAX0);
    _buff[1] =r_reg(ADXL345_DATAX1);
    _buff[2] =r_reg(ADXL345_DATAY0);
    _buff[3] =r_reg(ADXL345_DATAY1);
    _buff[4] =r_reg(ADXL345_DATAZ0);
    _buff[5] =r_reg(ADXL345_DATAZ1);

    // Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
    *x = (int16_t)((((int)_buff[1]) << 8) | _buff[0]);
    *y = (int16_t)((((int)_buff[3]) << 8) | _buff[2]);
    *z = (int16_t)((((int)_buff[5]) << 8) | _buff[4]);
}
void ADXL345_readNormalizedAccel(float *x, float *y, float *z){
    int a,b,c;
    ADXL345_readAccel(&a,&b,&c);
    *x = a * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *y = b * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    *z = c * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
}
void ADXL345_setRange(range_t range)
{
  /* Red the data format register to preserve bits */
  uint8_t format = r_reg(ADXL345_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  w_reg(ADXL345_DATA_FORMAT, format);

  /* Keep track of the current range (to avoid readbacks) */
  //_range = range;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
range_t ADXL345_getRange(void)
{
  /* Red the data format register to preserve bits */
  return (range_t)(r_reg(ADXL345_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
void ADXL345_setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
    w_reg(ADXL345_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
dataRate_t ADXL345_getDataRate(void)
{
  return (dataRate_t)(r_reg(ADXL345_BW_RATE) & 0x0F);
}

