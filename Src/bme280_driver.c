
#include "stm32l4xx_hal.h"
#include "bme280_driver.h"
#include "stdio.h"
#include "string.h"

#include "main.h"

static struct bme280_dev dev;

int8_t bme280_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bme280_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
void user_delay_ms(uint32_t period);

int8_t bme280_sensor_init(void)
{

	debugPrint(&huart2, "Initializating I2C\r\n");

	dev.dev_id = BME280_I2C_ADDR_SEC; //BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = bme280_i2c_read;
	dev.write = bme280_i2c_write;
	dev.delay_ms = user_delay_ms;

	int8_t result = bme280_init(&dev);

	if(result != BME280_OK)
		return -1;

    /* Recommended mode of operation: Indoor navigation */
//    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
//    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
//    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
//    dev.settings.filter = BME280_FILTER_COEFF_16;
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_1X;
    dev.settings.osr_t = BME280_OVERSAMPLING_1X;
    dev.settings.filter = BME280_FILTER_COEFF_OFF;

    uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    result = bme280_set_sensor_settings(settings_sel, &dev);

	if(result != BME280_OK)
		return -1;

	return 0;
}

int8_t bme280_sensor_read(float *temp, float *press, float *hum)
{
	struct bme280_data comp_data;
//	uint8_t settings_sel;

//    /* Recommended mode of operation: Indoor navigation */
//    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
//    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
//    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
//    dev.settings.filter = BME280_FILTER_COEFF_16;
//
//    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
//
//    int8_t result = bme280_set_sensor_settings(settings_sel, &dev);
//
//    if(result != BME280_OK)
//    	return -1;

	debugPrint(&huart2, "Setting forced mode\r\n");

    /* configure forced mode */
	uint8_t result = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    if(result != BME280_OK)
    	return -1;

    /* wait for the measurements to complete */
    dev.delay_ms(1000);//40

    debugPrint(&huart2, "Getting sensor data\r\n");
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

void huart_transmit_hex(uint8_t hex)
{
	char digit[16] =
	{'0','1','2','3','4','5','6','7',
	'8','9','A','B','C','D','E','F'};

	uint8_t c;
	c = digit[hex >> 4];
	HAL_UART_Transmit(&huart2, &c, 1, 10);
	c = digit[hex & 0x0F];
	HAL_UART_Transmit(&huart2, &c, 1, 10);
	c = ' ';
	HAL_UART_Transmit(&huart2, &c, 1, 10);
}

#define I2C_DEBUG_WRITE		1
#define I2C_DEBUG_READ		0
#define DEBUG_LINE_LEN		116
void i2c_debug(char type, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, uint8_t *output, uint16_t output_len)
{
	char line[DEBUG_LINE_LEN];

	if(type == I2C_DEBUG_WRITE){
		snprintf(line, 80, "[W] 0x%02X 0x%02X 0x%02X %02d\r\n", dev_addr, reg_addr, *reg_data, len);
		debugPrint(&huart2, line);
	}else{
		snprintf(line, 80, "[R] 0x%02X 0x%02X 0x%02X %02d: ", dev_addr, reg_addr, *reg_data, len);
		debugPrint(&huart2, line);
		for(int i = 0;i < len;i++){
			huart_transmit_hex(output[i]);
		}
		debugPrint(&huart2, "\r\n");
//		int line_len = strlen(line);
//		for(int i = 0;i < len;i++) {
//			snprintf(line + line_len + i, 80 - line_len - i - 2, " %02X", output[i]);
//		}
//		line_len = strlen(line);
//		line[line_len] = '\r';
//		line[line_len+1] = '\n';
	}


}

int8_t bme280_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{

//	char line[80];
//	snprintf(line, 80, "[R] 0x%02X 0x%02X 0x%02X %d: ", dev_addr, reg_addr, *reg_data, len);


	int8_t result = 0; /* Return 0 for Success, non-zero for failure */
    HAL_StatusTypeDef status = HAL_OK;
	uint8_t array[I2C_BUFFER_LEN] = {0};
	uint8_t stringpos = 0;
	array[0] = reg_addr;

	while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Read(&hi2c1,						// i2c handle
    						  (uint16_t)(dev_addr<<1),		// i2c address, left aligned
							  (uint16_t)reg_addr,			// register address
							  I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
							  (uint8_t*)(&array),			// write returned data to this variable
							  len,							// how many bytes to expect returned
							  100);							// timeout


    if (status != HAL_OK)
    {
    	// The BME280 API calls for 0 return value as a success, and -1 returned as failure
    	return -1;
    }
	for (stringpos = 0; stringpos < len; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}

	i2c_debug(I2C_DEBUG_READ, dev_addr, reg_addr, reg_data, len, array, len);

	return result;
}

int8_t bme280_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{

	char line[80];
	snprintf(line, 80, "[W] 0x%02X 0x%02X 0x%02X %02d\r\n", dev_addr, reg_addr, *reg_data, len);
	debugPrint(&huart2, line);

    HAL_StatusTypeDef status = HAL_OK;

    while (HAL_I2C_IsDeviceReady(&hi2c1, (uint8_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Write(&hi2c1,						// i2c handle
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
