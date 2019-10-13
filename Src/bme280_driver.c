
#include "stm32l4xx_hal.h"
#include "bme280_driver.h"

static struct bme280_dev dev;
I2C_HandleTypeDef *hi2c1;

int8_t bme280_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bme280_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
void user_delay_ms(uint32_t period);

int8_t bme280_sensor_init(I2C_HandleTypeDef *handle_i2c1)
{
	hi2c1 = handle_i2c1;
	dev.dev_id = BME280_I2C_ADDR_SEC; //BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = bme280_i2c_read;
	dev.write = bme280_i2c_write;
	dev.delay_ms = user_delay_ms;

	int8_t result = bme280_init(&dev);

	if(result != BME280_OK)
		return -1;

	return 0;
}

int8_t bme280_sensor_read(float *temp, float *press, float *hum)
{
	struct bme280_data comp_data;
	uint8_t settings_sel;

    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    int8_t result = bme280_set_sensor_settings(settings_sel, &dev);

    if(result != BME280_OK)
    	return -1;

    /* configure forced mode */
    result = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    if(result != BME280_OK)
    	return -1;

    /* wait for the measurements to complete */
    dev.delay_ms(40);

    result = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
    if(result != BME280_OK)
    	return -1;

    /* Read was ok, update values */
#ifdef BME280_FLOAT_ENABLE
    *temp = comp_data.temperature;
    *press = 0.01 * comp_data.pressure;
    *hum = comp_data.humidity;
#else
#ifdef BME280_64BIT_ENABLE
    *temp = 0.01f * comp_data.temperature;
    *press = 0.0001f * comp_data.pressure;
    *hum = 1.0f / 1024.0f * comp_data.humidity;
#else
    *temp = 0.01f * comp_data.temperature;
    *press = 0.01f * comp_data.pressure;
    *hum = 1.0f / 1024.0f * comp_data.humidity;
#endif
#endif

    return 0;

}

int8_t bme280_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	if(hi2c1 == NULL)
		return -1;

	int8_t result = 0; /* Return 0 for Success, non-zero for failure */
    HAL_StatusTypeDef status = HAL_OK;
	uint8_t array[I2C_BUFFER_LEN] = {0};
	uint8_t stringpos = 0;
	array[0] = reg_addr;

	while (HAL_I2C_IsDeviceReady(hi2c1, (uint16_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Read(hi2c1,						// i2c handle
    						  (uint16_t)(dev_addr<<1),		// i2c address, left aligned
							  (uint16_t)reg_addr,			// register address
							  I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
							  (uint8_t*)(&array),			// write returned data to this variable
							  len,							// how many bytes to expect returned
							  100);							// timeout

    if (status != HAL_OK)
    {
    	// The BME280 API calls for 0 return value as a success, and -1 returned as failure
    	result = -1;
    }
	for (stringpos = 0; stringpos < len; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}

	return result;
}

int8_t bme280_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	if(hi2c1 == NULL)
		return -1;

    HAL_StatusTypeDef status = HAL_OK;

    while (HAL_I2C_IsDeviceReady(hi2c1, (uint8_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Write(hi2c1,						// i2c handle
    						  (uint16_t)(dev_addr<<1),		// i2c address, left aligned
							  (uint16_t)reg_addr,			// register address
							  I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
							  (uint8_t*)(&reg_data),		// write returned data to reg_data
							  len,							// write how many bytes
							  100);							// timeout

    if (status != HAL_OK)
    	return -1;

    return 0;
}

void user_delay_ms(uint32_t period)
{
	HAL_Delay(period);
}
