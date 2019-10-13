#ifndef __BME280_DRIVER_H
#define __BME280_DRIVER_H

#include "bme280.h"

#define	I2C_BUFFER_LEN 			28

int8_t bme280_sensor_init(void);
int8_t bme280_sensor_read(float *temp, float *press, float *hum);

#endif //__BME280_DRIVER_H
