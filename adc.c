/*
 * adc.c
 *
 *  Created on: Mar 9, 2017
 *      Author: Atalville
 */
#include <string.h>
#include "driverlib.h"
#include "adc.h"
//
////void adc_init(){
////    //memset(resultsBuffer, 0x00, ADC_CHANNELS); /* initialize resultsBuffer with 0 value */
////
////     /* Enabling the FPU for floating point operation */
////     MAP_FPU_enableModule();
////     MAP_FPU_enableLazyStacking();
////     /* Initialize ADC module */
////     /* Setting reference voltage to 2.5  and enabling reference */
////     MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
////     MAP_REF_A_enableReferenceVoltage();
////     MAP_ADC14_enableModule();
////     MAP_ADC14_initModule(CS_SMCLK, ADC_PREDIVIDER_64, ADC_DIVIDER_4,
////             0);
////     /* Configuring GPIOs (5.5 A0) */
////     MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
////     GPIO_TERTIARY_MODULE_FUNCTION);
//////     /* Configuring GPIOs for Analog In */
//////     MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
//////             GPIO_PIN5 | GPIO_PIN4 , GPIO_TERTIARY_MODULE_FUNCTION);
//////     MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
//////             GPIO_PIN7 | GPIO_PIN6, GPIO_TERTIARY_MODULE_FUNCTION);
////
////
////     /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A0 - A1)  with no repeat)
////      * with internal 3.3V reference */
////    // MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
////     MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
////     MAP_ADC14_configureConversionMemory(ADC_MEM0,
////             ADC_VREFPOS_INTBUF_VREFNEG_VSS,
////             ADC_INPUT_A0, false);
//////     MAP_ADC14_configureConversionMemory(ADC_MEM1,
//////             ADC_VREFPOS_INTBUF_VREFNEG_VSS,
//////             ADC_INPUT_A1, false);
////
////     /* Enabling the interrupt when a conversion on channel 0 and 1 (end of sequence)
////      *  is complete and enabling conversions */
////     MAP_ADC14_enableInterrupt(ADC_INT0);
////     //MAP_ADC14_enableInterrupt(ADC_INT1);
////     /* Enabling Interrupts */
////     MAP_Interrupt_enableInterrupt(INT_ADC14);
////
////     MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
////
////     /* Triggering the start of the sample */
////     MAP_ADC14_enableConversion();
////     //MAP_ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_192,ADC_PULSE_WIDTH_192);/* sampling time is ADCCLK/192 */
////     //MAP_ADC14_toggleConversionTrigger();
////}
////void adc_start_sample(){
////    MAP_ADC14_toggleConversionTrigger();
////}
////
//
//
//void adc_init()
//{
//    /* Enabling the FPU for floating point operation */
//    MAP_FPU_enableModule();
//    MAP_FPU_enableLazyStacking();
//
//    /* Initializing ADC (MCLK/1/4) */
//    /* Setting reference voltage to 2.5  and enabling reference */
////    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
////    MAP_REF_A_enableReferenceVoltage();
//    MAP_ADC14_enableModule();
//    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_4,
//            0);
//
//    /* Configuring GPIOs (5.5 A0) */
//    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
//    GPIO_TERTIARY_MODULE_FUNCTION);
//
//    /* Configuring ADC Memory */
//    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
//    MAP_ADC14_configureConversionMemory(ADC_MEM0,
//             ADC_VREFPOS_AVCC_VREFNEG_VSS,
//             ADC_INPUT_A0, false);
//
//    /* Configuring Sample Timer */
//    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
//
//    /* Enabling/Toggling Conversion */
//    MAP_ADC14_enableConversion();
//    MAP_ADC14_toggleConversionTrigger();
//
//    /* Enabling interrupts */
//    MAP_ADC14_enableInterrupt(ADC_INT0);
//    MAP_Interrupt_enableInterrupt(INT_ADC14);
//}
//
//void adc_start_sample(){
//    MAP_ADC14_toggleConversionTrigger();
//}
void adc_init()
{
    /* Enabling the FPU for floating point operation */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();

    /* Initializing ADC (MCLK/1/4) */
    /* Setting reference voltage to 2.5  and enabling reference */
//    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
//    MAP_REF_A_enableReferenceVoltage();
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_4,
            0);

    /* Configuring GPIOs (5.5 A0) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5|GPIO_PIN4,
    GPIO_TERTIARY_MODULE_FUNCTION);

    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
        MAP_ADC14_configureConversionMemory(ADC_MEM0,
                 ADC_VREFPOS_AVCC_VREFNEG_VSS,
                 ADC_INPUT_A0, false);
        MAP_ADC14_configureConversionMemory(ADC_MEM1,
                ADC_VREFPOS_AVCC_VREFNEG_VSS,
                ADC_INPUT_A1, false);
//    /* Configuring ADC Memory */
//    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
//    MAP_ADC14_configureConversionMemory(ADC_MEM0,
//             ADC_VREFPOS_AVCC_VREFNEG_VSS,
//             ADC_INPUT_A0, false);
//
//    /* Configuring Sample Timer */
//    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    /* Enabling/Toggling Conversion */
    MAP_ADC14_enableConversion();
    //MAP_ADC14_toggleConversionTrigger();

    /* Enabling interrupts */
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_ADC14_enableInterrupt(ADC_INT1);
    MAP_Interrupt_enableInterrupt(INT_ADC14);
}

void adc_start_sample(){
    MAP_ADC14_toggleConversionTrigger();
}


