/*
 * systick.c
 *
 *  Created on: Mar 8, 2017
 *      Author: Atalville
 */
#include "driverlib.h"

extern volatile uint8_t g_ms_timeout;
void systick_init(){
    SysTick->CTRL = 0;  // disable systick
    SysTick->LOAD = 48000;
    SysTick->VAL = 0; // clear this register
    SysTick->CTRL = 0x00000007;
}
void systick_delay_ms(int32_t ms){
    while(--ms >= 0){
        while(g_ms_timeout == 0);
        g_ms_timeout = 0;
    }
}
