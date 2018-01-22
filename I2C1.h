// I2C0.h
// Runs on MSP432
// Provide a function that initializes, sends, and receives
// to the eUSCI module interface
// Daniel Valvano
// August 3, 2015
// Modified by RWB 2/27/16

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
typedef signed char  s8;/**< used for signed 8bit */
typedef signed short int s16;/**< used for signed 16bit */
typedef signed int s32;/**< used for signed 32bit */
typedef signed long long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef unsigned char u8;/**< used for unsigned 8bit */
typedef unsigned short int u16;/**< used for unsigned 16bit */
typedef unsigned int u32;/**< used for unsigned 32bit */
typedef unsigned long long int u64;/**< used for unsigned 64bit */
void I2C1_Init(void);

// receives one byte from specified slave

uint8_t I2C_Recv(int8_t slave);

// receives two bytes from specified slave
// Used to read the contents of a device register
uint16_t I2C_Recv2(int8_t slave);

// sends one byte to specified slave
// Returns 0 if successful, nonzero if error
uint16_t I2C_Send1(int8_t slave, uint8_t data1);

// sends two bytes to specified slave
// Returns 0 if successful, nonzero if error
uint16_t I2C_Send2(int8_t slave, uint8_t data1, uint8_t data2);

// sends three bytes to specified slave
// Returns 0 if successful, nonzero if error
uint16_t I2C_Send3(int8_t slave, uint8_t data1, uint8_t data2, uint8_t data3);

s8 I2C1_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 I2C1_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
