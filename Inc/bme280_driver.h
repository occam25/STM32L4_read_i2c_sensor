#ifndef __BME280_DRIVER_H
#define __BME280_DRIVER_H

#include "bme280.h"

extern float temp, press, hum;

s32 bme280_data_readout(void);

#endif //__BME280_DRIVER_H
