/*
 * clk.c
 *
 *  Created on: Mar 8, 2017
 *      Author: Atalville
 */
#include "driverlib.h"
#include "debug.h"
uint32_t g_SMCLKfreq;
uint32_t g_MCLKfreq;
static void clockInit48MHzXTL(void) {  // sets the clock module to use the external 48 MHz crystal

    /* Configuring pins for peripheral/crystal usage */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    CS_setExternalClockSourceFrequency(32000,48000000); // enables getMCLK, getSMCLK to know externally set frequencies

    /* Starting HFXT in non-bypass mode without a timeout. Before we start
     * we have to change VCORE to 1 to support the 48MHz frequency */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false);  // false means that there are no timeouts set, will return when stable

    /* Initializing MCLK to HFXT (effectively 48MHz) */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
}
void clk_init(){
    clockInit48MHzXTL();  // set up the clock to use the crystal oscillator on the Launchpad
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);    /* MCLK = 48 Mhz*/
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4); /* SMCLK = 48/4 = 12Mhz*/
//    g_SMCLKfreq=MAP_CS_getSMCLK();  // get SMCLK value to verify it was set correctly
//    g_MCLKfreq=MAP_CS_getMCLK();  // get MCLK value
//    #ifdef __DEBUG__
//        printf("MCLK = %d Hz, SMCLK = %d Hz\r\n",g_MCLKfreq,g_SMCLKfreq);
//    #else
//        #error "__DEBUG__ not defined in debug.h"
//    #endif
}


