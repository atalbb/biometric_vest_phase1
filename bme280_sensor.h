/*
 * bme280_sensor.h
 *
 *  Created on: Mar 8, 2017
 *      Author: Atalville
 */

#ifndef BME280_SENSOR_H_
#define BME280_SENSOR_H_

void get_bme280_values(float *temp, float * press, float *hum);
void  bme280_sensor_init(void);
void bme280_1ms_handler(void);

#endif /* BME280_SENSOR_H_ */
