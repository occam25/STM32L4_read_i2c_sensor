
#include "stm32l4xx_hal.h"
#include "bme280_driver.h"
#include "stdio.h"
#include "string.h"

#include "main.h"

#define BME280_API


struct bme280_t bme280;

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 I2C_routine(void);
void BME280_delay_msek(u32 msek);

float temp, press, hum;



s32 bme280_data_readout(void)
{
	/* The variable used to assign the standby time*/
	//u8 v_stand_by_time_u8 = BME280_INIT_VALUE;
	/* The variable used to read uncompensated temperature*/
	s32 v_data_uncomp_temp_s32 = BME280_INIT_VALUE;
	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_pres_s32 = BME280_INIT_VALUE;
	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_hum_s32 = BME280_INIT_VALUE;
	/* The variable used to read compensated temperature*/
	s32 v_comp_temp_s32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
	/* The variable used to read compensated pressure*/
	u32 v_comp_press_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
	/* The variable used to read compensated humidity*/
	u32 v_comp_humidity_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};

	/* result of communication results*/
	s32 com_rslt = ERROR;


 /*********************** START INITIALIZATION ************************/
  /*	Based on the user need configure I2C or SPI interface.
  *	It is example code to explain how to use the bme280 API*/
 	#ifdef BME280_API
	I2C_routine();
	/*SPI_routine();*/
	#endif
/*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	Chip id
*-------------------------------------------------------------------------*/
	com_rslt = bme280_init(&bme280);

	/*	For initialization it is required to set the mode of
	 *	the sensor as "NORMAL"
	 *	data acquisition/read/write is possible in this mode
	 *	by using the below API able to set the power mode as NORMAL*/
	/* Set the power mode as NORMAL*/
	com_rslt += bme280_set_power_mode(BME280_FORCED_MODE); //BME280_NORMAL_MODE);
	/*	For reading the pressure, humidity and temperature data it is required to
	 *	set the OSS setting of humidity, pressure and temperature
	 * The "BME280_CTRLHUM_REG_OSRSH" register sets the humidity
	 * data acquisition options of the device.
	 * changes to this registers only become effective after a write operation to
	 * "BME280_CTRLMEAS_REG" register.
	 * In the code automated reading and writing of "BME280_CTRLHUM_REG_OSRSH"
	 * register first set the "BME280_CTRLHUM_REG_OSRSH" and then read and write
	 * the "BME280_CTRLMEAS_REG" register in the function*/
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	/* set the pressure oversampling*/
	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_1X); //BME280_OVERSAMP_2X
	/* set the temperature oversampling*/
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_1X); //BME280_OVERSAMP_4X
/*--------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the standby time of the sensor input
	 *	value have to be given
	 *	Normal mode comprises an automated perpetual cycling between an (active)
	 *	Measurement period and an (inactive) standby period.
	 *	The standby time is determined by the contents of the register t_sb.
	 *	Standby time can be set using BME280_STANDBYTIME_125_MS.
	 *	Usage Hint : bme280_set_standbydur(BME280_STANDBYTIME_125_MS)*/

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);

	/* This API used to read back the written value of standby time*/
	//com_rslt += bme280_get_standby_durn(&v_stand_by_time_u8);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*------------------------------------------------------------------*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------*
************ START READ UNCOMPENSATED PRESSURE, TEMPERATURE
AND HUMIDITY DATA ********
*---------------------------------------------------------------------*/

	/* API is used to read the uncompensated humidity*/
	com_rslt += bme280_read_uncomp_humidity(&v_data_uncomp_hum_s32);

	/* API is used to read the uncompensated temperature*/
	com_rslt += bme280_read_uncomp_temperature(&v_data_uncomp_temp_s32);

	/* API is used to read the uncompensated pressure*/
	com_rslt += bme280_read_uncomp_pressure(&v_data_uncomp_pres_s32);

	HAL_Delay(10); //+++

	/* API is used to read the uncompensated temperature,pressure
	and humidity data */
	com_rslt += bme280_read_uncomp_pressure_temperature_humidity(
	&v_data_uncomp_temp_s32, &v_data_uncomp_pres_s32, &v_data_uncomp_hum_s32);
/*--------------------------------------------------------------------*
************ END READ UNCOMPENSATED PRESSURE AND TEMPERATURE********
*-------------------------------------------------------------------------*/

/*------------------------------------------------------------------*
************ START READ COMPENSATED PRESSURE, TEMPERATURE
AND HUMIDITY DATA ********
*---------------------------------------------------------------------*/
	/* API is used to compute the compensated temperature*/
//	v_comp_temp_s32[0] = bme280_compensate_temperature_int32(
//			v_data_uncomp_temp_s32);
//
//	/* API is used to compute the compensated pressure*/
//	v_comp_press_u32[0] = bme280_compensate_pressure_int32(
//			v_data_uncomp_pres_s32);
//
//	/* API is used to compute the compensated humidity*/
//	v_comp_humidity_u32[0] = bme280_compensate_humidity_int32(
//			v_data_uncomp_hum_s32);

	/* API is used to read the compensated temperature, humidity and pressure*/
	com_rslt += bme280_read_pressure_temperature_humidity(
	&v_comp_press_u32[1], &v_comp_temp_s32[1],  &v_comp_humidity_u32[1]);
/*--------------------------------------------------------------------*
************ END READ COMPENSATED PRESSURE, TEMPERATURE AND HUMIDITY ********
*-------------------------------------------------------------------------*/

	// Update values
	temp = ((float)v_comp_temp_s32[1]/100);
	press = ((float)(v_comp_press_u32[1])/100);
	hum = ((float)(v_comp_humidity_u32[1])/1024);

//	HAL_Delay(100);
//	//float imp_temp = ((float)(v_comp_temp_s32[1])/100)*1.8+32;		// convert to fahrenheit
//	float imp_temp = ((float)v_comp_temp_s32[1]/100);
//	//float imp_press = ((float)(v_comp_press_u32[1])/100)*.0295300; 	// convert to inches of mercury
//	float imp_press = ((float)(v_comp_press_u32[1])/100);
//	float imp_humi = ((float)(v_comp_humidity_u32[1])/1024);		// relative humidity
//
//	char line[80];
//	snprintf(line, 80, "Temp: %.f DegF,  Press: %.2f inHg,  Humi: %.f%% rH\r\n",
//								imp_temp,
//								imp_press,
//								imp_humi);
//	  debugPrint(&huart2, line);

/*-----------------------------------------------------------------------*
************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
	/*	For de-initialization it is required to set the mode of
	 *	the sensor as "SLEEP"
	 *	the device reaches the lowest power consumption only
	 *	In SLEEP mode no measurements are performed
	 *	All registers are accessible
	 *	by using the below API able to set the power mode as SLEEP*/
	 /* Set the power mode as SLEEP*/
	com_rslt += bme280_set_power_mode(BME280_SLEEP_MODE);
/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
	return com_rslt;
}

#ifdef BME280_API
#define BME280_DATA_INDEX	1
#define BME280_ADDRESS_INDEX	2
/*--------------------------------------------------------------------------*
*	The following function is used to map the I2C bus read, write, delay and
*	device address with global structure bme280
*-------------------------------------------------------------------------*/
s8 I2C_routine(void) {
/*--------------------------------------------------------------------------*
 *  By using bme280 the following structure parameter can be accessed
 *	Bus write function pointer: BME280_WR_FUNC_PTR
 *	Bus read function pointer: BME280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
	bme280.bus_write = BME280_I2C_bus_write;
	bme280.bus_read = BME280_I2C_bus_read;
	bme280.dev_addr = BME280_I2C_ADDRESS2;
	bme280.delay_msec = BME280_delay_msek;

	return BME280_INIT_VALUE;
}

/************** I2C/SPI buffer length ******/
//#define	I2C_BUFFER_LEN 28

/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	The device address defined in the bme280.h file
*-----------------------------------------------------------------------*/
 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

    HAL_StatusTypeDef status = HAL_OK;

    while (HAL_I2C_IsDeviceReady(&hi2c1, (uint8_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Write(&hi2c1,								// i2c handle
								(uint16_t)(dev_addr<<1),			// i2c address, left aligned
								(uint16_t)reg_addr,					// register address
								I2C_MEMADD_SIZE_8BIT,				// bme280 uses 8bit register addresses
								(uint8_t*)(&reg_data),				// write returned data to reg_data
								cnt,								// write how many bytes
								100);								// timeout

	if (status != HAL_OK)
		return -1;

	return 0;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of data byte of to be read
 */
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

	HAL_StatusTypeDef status = HAL_OK;

	while(HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

	status = HAL_I2C_Mem_Read(&hi2c1,					// i2c handle
							(uint16_t)(dev_addr<<1),	// i2c address, left aligned
							(uint16_t)reg_addr,			// register address
							I2C_MEMADD_SIZE_8BIT,		// bme280 uses 8bit register addresses
							(uint8_t *)reg_data,		// write returned data to this variable //(uint8_t *)(&array),
							cnt,						// how many bytes to expect returned
							100);						// timeout


    if (status != HAL_OK)
    {
        // The BME280 API calls for 0 return value as a success, and -1 returned as failure
        return -1;
    }

	return 0;
}


/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BME280_delay_msek(u32 msek)
{
	HAL_Delay(msek);
}
#endif

