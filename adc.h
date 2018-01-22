/*
 * adc.h
 *
 *  Created on: Mar 9, 2017
 *      Author: Atalville
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_VREF        (3.3)
#define ADC_MAX_QUANT   (16383.0)
#define ADC_CHANNELS    2
#define RR_CH          0
#define HR_CH         1

//typedef enum{
//    ADC_IDLE = 0,
//    ADC_CONV_STARTED,
//    ADC_CH0_DONE,
//    ADC_CH1_DONE,
//    ADC_CH0_PROCESS_DONE,
//    ADC_CH1_PROCESS_DONE,
//}_E_ADC_STATE;

void adc_init();
void adc_start_sample();



#endif /* ADC_H_ */
