/*
 * RRAlgorithm.c
 *
 *  Created on: Oct 15, 2017
 *      Author: Atalville
 */
#include <stdint.h>
#include <stdio.h>
#include "HR_RR_Algorithm.h"


#define min(x,y) ((x) < (y) ? (x) : (y))
const uint8_t MIN_DIST  =  5;


double find_mean(double *input){
    double mean = 0;
    uint16_t i = 0;
    for(i=0;i<RR_BUF_SIZE;i++){
        mean += input[i];
    }
    mean /= RR_BUF_SIZE;
    return mean;

}
/* To remove DC Offset of the signal */
void diff_from_mean(double *an_x,double *an_y,double avg){
    int i = 0;
    for(i=0;i<RR_BUF_SIZE;i++){
        an_y[i] = an_x[i] - avg;
    }
}
/* 4 point moving average to smoothen the signal */
void four_pt_MA(double *an_x){
    uint16_t i = 0;
    for(i=0;i<RR_BUF_SIZE-MA4_SIZE+1;i++){
        an_x[i] = an_x[i] + an_x[i+1] + an_x[i+2]+ an_x[i+3];
        an_x[i] /= MA4_SIZE;
    }
}



double threshold_calc(double *an_dx){
    int i=0;
    double n_th1 = 0.0;
    for(i=0;i<RR_BUF_SIZE-MA4_SIZE+1;i++){
        n_th1 += an_dx[i];
    }
    n_th1 /= (RR_BUF_SIZE-MA4_SIZE + 1);
    return n_th1;
}

uint16_t myPeakCounter(double  *pn_x, int32_t n_size, double n_min_height){
    uint32_t i = 0;
    uint8_t flag = 0;
    uint32_t count = 0;
    uint32_t dist = 0;
    for(i=0;i<n_size;i++){
        if(!flag){
            if(pn_x[i] > n_min_height){
                flag = 1;
            }
        }else{
            if(pn_x[i] < n_min_height){
                if(++dist >= MIN_DIST){
                    dist = 0;
                    flag = 0;
                    count++;
                }
            }
        }
    }
    return count;
   // printf("My Number of peaks is %d\n",count);
}
void ButterworthLowpassFilter0040SixthOrder(const double *src, double *dest, int size)
{
    const double GAIN = 4.004448900e+05;
    double xv[7] = {0.0}, yv[7] = {0.0};
    int i = 0;
    for ( i = 0; i < size; i++)
    {
        xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; xv[5] = xv[6];
        xv[6] = src[i] / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; yv[5] = yv[6];
        yv[6] =   (xv[0] + xv[6]) + 6.0 * (xv[1] + xv[5]) + 15.0 * (xv[2] + xv[4])
                     + 20.0 * xv[3]
                     + ( -0.3774523864 * yv[0]) + (  2.6310551285 * yv[1])
                     + ( -7.6754745482 * yv[2]) + ( 11.9993158160 * yv[3])
                     + (-10.6070421840 * yv[4]) + (  5.0294383514 * yv[5]);
        dest[i] = yv[6];
    }
}
void ButterworthLowpassFilter0100SixthOrder(const double src[], double dest[], int size)
{
    const double GAIN = 2.936532839e+03;
    int i = 0;
    double xv[7] = {0.0}, yv[7] = {0.0};

    for ( i = 0; i < size; i++)
    {
        xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; xv[5] = xv[6];
        xv[6] = src[i] / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; yv[5] = yv[6];
        yv[6] =   (xv[0] + xv[6]) + 6.0 * (xv[1] + xv[5]) + 15.0 * (xv[2] + xv[4])
                     + 20.0 * xv[3]
                     + ( -0.0837564796 * yv[0]) + (  0.7052741145 * yv[1])
                     + ( -2.5294949058 * yv[2]) + (  4.9654152288 * yv[3])
                     + ( -5.6586671659 * yv[4]) + (  3.5794347983 * yv[5]);
        dest[i] = yv[6];
    }
}
// Defines a butterworth low pass filter with center freq. 0.05 * sampling frequency
// Note: this processor only works with 'double'
// this filter is used as anti-aliasing filter from 20kHz sampling rate down to 5kHz
void ButterworthLowpassFilter0050SixthOrder(const double src[], double dest[], int size)
{
    const double GAIN = 1.165969038e+05;
    int i =  0;
    double xv[7] = {0.0}, yv[7] = {0.0};

    for ( i = 0; i < size; i++)
    {
        xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; xv[5] = xv[6];
        xv[6] = src[i] / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; yv[5] = yv[6];
        yv[6] =   (xv[0] + xv[6]) + 6.0 * (xv[1] + xv[5]) + 15.0 * (xv[2] + xv[4])
                     + 20.0 * xv[3]
                     + ( -0.2951724313 * yv[0]) + (  2.1290387500 * yv[1])
                     + ( -6.4411118810 * yv[2]) + ( 10.4690788930 * yv[3])
                     + ( -9.6495177287 * yv[4]) + (  4.7871354989 * yv[5]);
        dest[i] = yv[6];
    }
}
// Defines a butterworth lowpass filter with center freq. 0.08 * sampling frequency
// Note: this processor only works with 'double'
// this filter is used as anti-aliasing filter from 10kHz sampling rate down to 4kHz
void ButterworthLowpassFilter0080SixthOrder(const double src[], double dest[], int size)
{
    const double GAIN = 9.339780497e+03;

    double xv[7] = {0.0}, yv[7] = {0.0};
    int i = 0;
    for ( i = 0; i < size; i++)
    {
        xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; xv[5] = xv[6];
        xv[6] = src[i] / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; yv[5] = yv[6];
        yv[6] =   (xv[0] + xv[6]) + 6.0 * (xv[1] + xv[5]) + 15.0 * (xv[2] + xv[4])
                     + 20.0 * xv[3]
                     + ( -0.1396600417 * yv[0]) + (  1.1086708553 * yv[1])
                     + ( -3.7230194289 * yv[2]) + (  6.7850160254 * yv[3])
                     + ( -7.0995038188 * yv[4]) + (  4.0616439992 * yv[5]);
        dest[i] = yv[6];
    }
}

