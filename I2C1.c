// I2C0.c
// Runs on MSP432
// Provide a function that initializes, sends, and receives
// the eUSCI1B module interfaced with A BMP180 pressure sensor
//
// Daniel Valvano
// October 12, 2015
//
// modified by RWB 2/26/2016

/*
 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include "msp.h"
#include "I2C1.h"

void I2C1_Init(void){
  // initialize eUSCI
  UCB0CTLW0 = 0x0001;                // hold the eUSCI module in reset mode
  // configure UCB1CTLW0 for:
  // bit15      UCA10 = 0; own address is 7-bit address
  // bit14      UCSLA10 = 0; address slave with 7-bit address
  // bit13      UCMM = 0; single master environment
  // bit12      reserved
  // bit11      UCMST = 1; master mode
  // bits10-9   UCMODEx = 3; I2C mode
  // bit8       UCSYNC = 1; synchronous mode
  // bits7-6    UCSSELx = 2; eUSCI clock SMCLK
  // bit5       UCTXACK = X; transmit ACK condition in slave mode
  // bit4       UCTR = X; transmitter/receiver
  // bit3       UCTXNACK = X; transmit negative acknowledge in slave mode
  // bit2       UCTXSTP = X; transmit stop condition in master mode
  // bit1       UCTXSTT = X; transmit start condition in master mode
  // bit0       UCSWRST = 1; reset enabled
  UCB0CTLW0 = 0x0F81;
  // configure UCB1CTLW1 for:
  // bits15-9   reserved
  // bit8       UCETXINT = X; early UCTXIFG0 in slave mode
  // bits7-6    UCCLTO = 3; timeout clock low after 165,000 SYSCLK cycles
  // bit5       UCSTPNACK = 0; send negative acknowledge before stop condition in master receiver mode
  // bit4       UCSWACK = 0; slave address acknowledge controlled by hardware
  // bits3-2    UCASTPx = 2; generate stop condition automatically after UCB1TBCNT bytes
  // bits1-0    UCGLITx = 0 deglitch time of 50 ns
  UCB0CTLW1 = 0x00C8;
  UCB0TBCNT = 2;                     // generate stop condition after this many bytes
  // set the baud rate for the eUSCI which gets its clock from SMCLK
  // Clock_Init48MHz() from ClockSystem.c sets SMCLK = HFXTCLK/4 = 12 MHz
  // if the SMCLK is set to 12 MHz, divide by 120 for 100 kHz baud clock
  UCB0BRW = 120;
// P6SEL0 |= 0xC0;                   //
// P6SEL1 &= ~0xC0;                   // configure P6.6 and P6.7 as primary module function
  P1SEL0 |= (BIT6|BIT7);
  P1SEL1 &= ~(BIT6|BIT7);                   // configure P6.4 and P6.5 as primary module function
  UCB0CTLW0 &= ~0x0001;              // enable eUSCI module
  UCB0IE = 0x0000;                   // disable interrupts
}


/* \Brief: The function is used as I2C bus read
*  \Return : Status of the I2C read
*  \param dev_addr : The device address of the sensor
*  \param reg_addr : Address of the first register, will data is going to be read
*  \param reg_data : This data read from the sensor, which is hold in an array
*  \param cnt : The no of data byte of to be read
*/
//s8 I2C1_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
//{
//   s32 iError = 0;
//   uint8_t i;
//   uint16_t rtnval, debugdump;
//   u8 array[8] = {0};
//   u8 stringpos = 0;
//   array[0] = reg_addr;
//   /* Please take the below function as your reference
//    * for read the data using I2C communication
//    * add your I2C read function here.
//    * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
//    * iError is an return value of write function
//    * Please select your valid return value
//    * In the driver SUCCESS defined as 0
//    * and FAILURE defined as -1
//    */
//
//   // set pointer to register address
//   while(UCB0STATW&0x0010){};         // wait for I2C ready
//   UCB0CTLW0 |= 0x0001;               // hold the eUSCI module in reset mode
//   UCB0TBCNT = 1;                     // generate stop condition after this many bytes
//   UCB0CTLW0 &= ~0x0001;              // enable eUSCI module
//   UCB0I2CSA = dev_addr;                 // I2CCSA[6:0] is slave address
//   UCB0CTLW0 = ((UCB0CTLW0&~0x0004)   // clear bit2 (UCTXSTP) for no transmit stop condition
//                                      // set bit1 (UCTXSTT) for transmit start condition
//                 | 0x0012);           // set bit4 (UCTR) for transmit mode
//   while(UCB0CTLW0&0x0002){};         // wait for slave address sent
//   UCB0TXBUF = reg_addr&0xFF;            // TXBUF[7:0] is data
//   while(UCB0STATW&0x0010){           // wait for I2C idle
//     if(UCB0IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
//       debugdump = UCB0IFG;           // snapshot flag register for calling program
//       I2C_Init();                    // reset to known state
//       return iError=-1;
//     }
//   }
//
//   // receive bytes from registers on BME280 device
//
//   while(UCB0STATW&0x0010){};         // wait for I2C ready
//   UCB0CTLW0 |= 0x0001;               // hold the eUSCI module in reset mode
//   UCB0TBCNT = cnt;                     // generate stop condition after this many bytes
//   UCB0CTLW0 &= ~0x0001;              // enable eUSCI module
//   UCB0I2CSA = dev_addr;                 // I2CCSA[6:0] is slave address
//   UCB0CTLW0 = ((UCB0CTLW0&~0x0014)   // clear bit4 (UCTR) for receive mode
//                                      // clear bit2 (UCTXSTP) for no transmit stop condition
//                 | 0x0002);           // set bit1 (UCTXSTT) for transmit start condition
//   for(i=0; i<cnt; i++) {
//       while((UCB0IFG&0x0001) == 0){      // wait for complete character received
//         if(UCB0IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
//           I2C_Init();                    // reset to known state
//           return 0xFFFF;
//         }
//       }
//       *reg_data++= UCB0RXBUF&0xFF;            // get the reply
//   }
//
//   return (s8)iError;
//}
///*-------------------------------------------------------------------*
//*   This is a sample code for read and write the data by using I2C/SPI
//*   Use either I2C or SPI based on your need
//*   The device address defined in the bme280.h file
//*-----------------------------------------------------------------------*/
// /* \Brief: The function is used as I2C bus write
// *  \Return : Status of the I2C write
// *  \param dev_addr : The device address of the sensor
// *  \param reg_addr : Address of the first register, will data is going to be written
// *  \param reg_data : It is a value hold in the array,
// *      will be used for write the value into the register
// *  \param cnt : The no of byte of data to be write
// */
//s8 I2C1_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
//{
//    s32 iError = 0;
//    u8 array[8];
//    u8 stringpos = 0;
//    array[0] = reg_addr;
//    uint8_t i;
//    uint16_t rtnval, debugdump;
//
//    for (stringpos = 0; stringpos < cnt; stringpos++) {
//        array[stringpos + 1] = *(reg_data + stringpos);
//    }
//    /*
//    * Please take the below function as your reference for
//    * write the data using I2C communication
//    * "IERROR = I2C_WRITE_STRING(DEV_ADDR, array, cnt+1)"
//    * add your I2C write function here
//    * iError is an return value of I2C read function
//    * Please select your valid return value
//    * In the driver SUCCESS defined as 0
//    * and FAILURE defined as -1
//    * Note :
//    * This is a full duplex operation,
//    * The first read data is discarded, for that extra write operation
//    * have to be initiated. For that cnt+1 operation done in the I2C write string function
//    * For more information please refer data sheet SPI communication:
//    */
//
//    while(UCB0STATW&0x0010){};         // wait for I2C ready
//    UCB0CTLW0 |= 0x0001;               // hold the eUSCI module in reset mode
//    UCB0TBCNT = cnt+1;                     // generate stop condition after this many bytes
//    UCB0CTLW0 &= ~0x0001;              // enable eUSCI module
//    UCB0I2CSA = dev_addr;              // I2CCSA[6:0] is slave address
//    UCB0CTLW0 = ((UCB0CTLW0&~0x0004)   // clear bit2 (UCTXSTP) for no transmit stop condition
//                                       // set bit1 (UCTXSTT) for transmit start condition
//                  | 0x0012);           // set bit4 (UCTR) for transmit mode
//    while((UCB0IFG&0x0002) == 0){};    // wait for slave address sent
//
//    for(i=0; i<cnt; i++) {
//        UCB0TXBUF = array[i]&0xFF;         // TXBUF[7:0] is data
//        while((UCB0IFG&0x0002) == 0){      // wait for data sent
//            if(UCB0IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
//                debugdump = UCB0IFG;           // snapshot flag register for calling program
//                I2C_Init();                    // reset to known state
//                return iError=-1;
//            }
//        }
//    }
//    UCB0TXBUF = array[i]&0xFF;         // TXBUF[7:0] is last data
//    while(UCB0STATW&0x0010){           // wait for I2C idle
//      if(UCB0IFG&0x0030){              // bit5 set on not-acknowledge; bit4 set on arbitration lost
//        debugdump = UCB0IFG;           // snapshot flag register for calling program
//        I2C_Init();                    // reset to known state
//        return iError=-1;
//      }
//    }
//    return iError=0;
//}
