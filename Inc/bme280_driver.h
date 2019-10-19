#ifndef __BME280_DRIVER_H
#define __BME280_DRIVER_H

#include "bme280.h"

#define	I2C_BUFFER_LEN 			28

extern float temp, press, hum;

int8_t bme280_sensor_init(void);
int8_t bme280_sensor_read(void);

#endif //__BME280_DRIVER_H
