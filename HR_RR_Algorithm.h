/*
 * RRAlgorithm.h
 *
 *  Created on: Oct 15, 2017
 *      Author: Atalville
 */

#ifndef HR_RR_ALGORITHM_H_
#define HR_RR_ALGORITHM_H_

#define MA4_SIZE    4

typedef enum{
    RR_INITIAL = 0,
    RR_STABLE
}_E_RR_STATE;

typedef enum{
    HR_INITIAL = 0,
    HR_STABLE
}_E_HR_STATE;


/* Configurable defines for Respiratory Rate */
#define RR_SPS                     20
#define RR_INITIAL_FRAME_TIME_S    30
#define RR_STABLE_FRAME_TIME_S     4


/* Configurable defines for Heart Rate */
#define HR_SPS                     20
#define HR_INITIAL_FRAME_TIME_S    30
#define HR_STABLE_FRAME_TIME_S     2

/* Non-Configurable defines for Respiratory Rate */
#define RR_SAMPLE_TIME_MS          (1000/RR_SPS)
#define RR_BUF_SIZE               (RR_INITIAL_FRAME_TIME_S * RR_SPS)
#define RR_STABLE_BUF_SIZE        (RR_STABLE_FRAME_TIME_S * RR_SPS)
#define RR_STABLE_QUEUE_DEPTH      (RR_INITIAL_FRAME_TIME_S/RR_STABLE_FRAME_TIME_S)
#define RR_STABLE_QUEUE_SIZE      (RR_STABLE_QUEUE_DEPTH * RR_STABLE_BUF_SIZE)


/* Non-Configurable defines for Heart Rate */
#define HR_SAMPLE_TIME_MS          (1000/HR_SPS)
#define HR_BUF_SIZE               (HR_INITIAL_FRAME_TIME_S * HR_SPS)
#define HR_STABLE_BUF_SIZE        (HR_STABLE_FRAME_TIME_S * HR_SPS)


/* Function prototypes */
double find_mean(double *input);
void diff_from_mean(double *an_x,double *an_y,double avg);
void four_pt_MA(double *an_x);
uint16_t myPeakCounter(double  *pn_x, int32_t n_size, double n_min_height);
double threshold_calc(double *an_dx);
void ButterworthLowpassFilter0100SixthOrder(const double src[], double dest[], int size);
void ButterworthLowpassFilter0080SixthOrder(const double src[], double dest[], int size);
void ButterworthLowpassFilter0050SixthOrder(const double src[], double dest[], int size);
void ButterworthLowpassFilter0040SixthOrder(const double *src, double *dest, int size);


#endif /* HR_RR_ALGORITHM_H_ */
