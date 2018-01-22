/* msp430_spi.c
 * Library for performing SPI I/O on a wide range of MSP430 chips.
 *
 * Serial interfaces supported:
 * 1. USI - developed on MSP430G2231
 * 2. USCI_A - developed on MSP430G2553
 * 3. USCI_B - developed on MSP430G2553
 * 4. USCI_A F5xxx - developed on MSP430F5172, added F5529
 * 5. USCI_B F5xxx - developed on MSP430F5172, added F5529
 *
 * Copyright (c) 2013, Eric Brundick <spirilis@linux.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any purpose
 * with or without fee is hereby granted, provided that the above copyright notice
 * and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
 * OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <msp432.h>
#include "msp432_spi.h"
//#include "nrf_userconfig.h"
#include "driverlib.h"

extern volatile uint32_t g_ticks;

/* USCI 16-bit transfer functions rely on the Little-Endian architecture and use
 * an internal uint8_t * pointer to manipulate the individual 8-bit segments of a
 * 16-bit integer.
 */

void spi_init()
{

    /* SPI Master Configuration Parameter */
    const eUSCI_SPI_MasterConfig spiMasterConfig =
    {
            EUSCI_B_SPI_CLOCKSOURCE_SMCLK,             // SMCLK Clock Source
//          3000000,                                   // SMCLK 3Mhz
            12000000,                                  // SMCLK 12Mhz
             4000000,                                    // SPICLK = 500khz
            EUSCI_B_SPI_MSB_FIRST,                     // MSB First
            EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,    // Phase
            //EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,    // Phase
            EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH, // High polarity
            //EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW, // Low polarity
            EUSCI_B_SPI_3PIN                           // 3Wire SPI Mode
    };

//    /* Selecting P1.5 (CLK) P1.6 (MOSI) and P1.7 (MISO) in SPI mode */
//    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
//            GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
//
//    /* Configuring SPI in 3wire master mode */
//    SPI_initMaster(EUSCI_B0_BASE, &spiMasterConfig);
//
//    /* Enable SPI module */
//    SPI_enableModule(EUSCI_B0_BASE);

    /* Selecting P3.5(SCLK) P3.6(MOSI) and P3.7(MISO) in SPI mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
            GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    /* CS setup. */
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);

    /* Configuring SPI in 3wire master mode */
    SPI_initMaster(EUSCI_B2_SPI_BASE, &spiMasterConfig);

    /* Enable SPI module */
    SPI_enableModule(EUSCI_B2_SPI_BASE);
}

uint8_t spi_transfer(uint8_t inb)
{
    uint32_t ticks = 0;
    ticks = g_ticks;
    /* Wait until previous transmission is over */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_TXIFG)){
        if(g_ticks - ticks >= 100){
            spi_init();
            return 0xff;
        }
    }
    // load byte into transmit buffer and send
    EUSCI_B2->TXBUF = inb;
    ticks = g_ticks;
    /* Wait until current transmission is complete */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_RXIFG)){
        if(g_ticks - ticks >= 100){
            spi_init();
            return 0xff;
        }
    }
    // return received data byte
    return EUSCI_B2->RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
    uint16_t retw;
    uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;
    uint32_t ticks = 0;
    ticks = g_ticks;
    /* Wait until previous transmission is over */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_TXIFG)){
        if(g_ticks - ticks >= 100){
            spi_init();
            return 0xff;
        }
    }
    // load 1st byte into transmit buffer and send
    EUSCI_B2->TXBUF = inw8[1];
    ticks = g_ticks;
    /* Wait until current transmission is complete */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_RXIFG)){
        if(g_ticks - ticks >= 100){
            spi_init();
            return 0xff;
        }
    }
    // load received data into return variable upper byte
    retw8[1] = EUSCI_B2->RXBUF;
    // load 2nd byte into transmit buffer and send
    EUSCI_B2->TXBUF = inw8[0];
    ticks = g_ticks;
    /* Wait until current transmission is complete */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_RXIFG)){
        if(g_ticks - ticks >= 100){
            spi_init();
            return 0xff;
        }
    }
    // load received data into return variable lower byte
    retw8[0] = EUSCI_B2->RXBUF;
    return retw;
}

