# MS5611 sensor API
## Author
Name:			Francisco Martín Villegas
Email:			f.martinvillegas00@gmail.com
Colleague:		University of Almería (Spain)

## Introduction
This package contains the MS5611 temperature and pressure sensor.

The sensor driver package includes MS5611.c and MS5611.h files.


## Integration details
* Integrate MS5611.c and MS5611.h file in to the project.
* Include the MS5611.h file in your code like below.
``` c
#include "MS5611.h"
```

## File information
* MS5611.h : This header file has the constants, function declarations, macros and datatype declarations.
* MS5611.c : This source file contains the definitions of the sensor driver APIs.

## Supported sensor interfaces
* SPI 4-wire	:	SDI, SDO and CSB pins
* I2C			:	SDA and SDC pins

SPI 3-wire is currently not supported in the API.
## Usage guide
### Initializing the master
First you need to initialize the I2C master with:
```c
hal_i2c_init();
```
### Initializing the sensor
Then initialize the sensor using:
```c
MS5611_init();
```

#### Example for I2C
Create some FLOAT variables to read pressure and temperature, and the use the functions:
```c
MS5611_read_temperature();
MS5611_read_pressure();
```

### Sensor data units
> The temperature data is read in Celsius. 
> The pressure data is read in mBar.


### Templates for function
``` c
/**
 * @brief	Master initialition
 *
 */
void hal_i2c_init();

/**
 * @brief Data writting in MS5611
 *
 * @param[in]		MS5611_command			: (uint8_t)		command of the register where we want to write
 *
 * @param[out]		ret						: (esp_err_t)	variable that indicates if there was a problem
 *
 */
esp_err_t MS5611_write_byte(uint8_t MS5611_command);

/**
 * @brief Data reading in MS5611
 *
 * @param[in]		seize			: 	(unsigned)		number of bytes that we are going to read
 * @param[in]		buffer_out		: 	(uint8_t)		array where I am going to save the data
 *
 * @param[out]		ret				:	(esp_err_t)		variable that indicates if there was a problem
 *
 */
esp_err_t MS5611_read(uint8_t * buffer_out, unsigned size);

/**
 * @brief Calibration coefficients obtained from the NVM memory
 * The coefficients that I obtained during the development of this driver was:
 * 		C1 = 54535
 * 		C2 = 54134
 * 		C3 = 34040
 * 		C4 = 30924
 * 		C5 = 30191
 * 		C6 = 28416
 *
 *  The registers where we can read the coefficients are
 * 	separated 2 by 2 bytes: 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC
 *
 * 	@param[in]	 	*C1			:	(uint16_t)	pointer to C1, Pressure sensitivity										| SENST1
 * 	@param[in]		*C2			:	(uint16_t)	pointer to C2, Pressure offset											| OFFT1
 * 	@param[in]		*C3			:	(uint16_t)	pointer to C3, Temperature coefficient of pressure sensitivity			| TCS
 * 	@param[in]		*C4			:	(uint16_t)	pointer to C4, Temperature coefficient of pressure offset				| TCO
 * 	@param[in]		*C5			:	(uint16_t)	pointer to C5, Reference temperature									| TREF
 * 	@param[in]		*C6			:	(uint16_t)	pointer to C6, Temperature coefficient of the temperature 				| TEMPSENS
 *
 */
int MS5611_coef(uint16_t *C1, uint16_t *C2, uint16_t *C3, uint16_t *C4, uint16_t *C5, uint16_t *C6);

/**
 * @brief	Slave initialition
 *
 */
void MS5611_init();

/**
 * @brief	Temperature reading
 *
 * @param[in]	 	coef[5]				:	(uint16_t)		calibration coefficients array
 * @param[in]		*t_no_compensate	:	(long)			pointer to raw temperature data from the sensor
 *
 * @param[out]		aux					:	(long)			temperature in Celsius*100
 *
 */
int32_t MS5611_temperature(uint16_t coef[5], uint32_t* t_no_compensate);

/**
 * @brief	Pressure calibration
 *
 * @param[in]	 	temperature	:	(uint32_t)	raw temperature data directly from the sensor
 * @param[in] 		pressure	:	(long)		raw pressure data directly from the sensor
 * @param[in]	 	*C1			:	(uint16_t)	pointer to C1, Pressure sensitivity										| SENST1
 * @param[in]		*C2			:	(uint16_t)	pointer to C2, Pressure offset											| OFFT1
 * @param[in]		*C3			:	(uint16_t)	pointer to C3, Temperature coefficient of pressure sensitivity			| TCS
 * @param[in]		*C4			:	(uint16_t)	pointer to C4, Temperature coefficient of pressure offset				| TCO
 * @param[in]		*C5			:	(uint16_t)	pointer to C5, Reference temperature									| TREF
 * @param[in]		*C6			:	(uint16_t)	pointer to C6, Temperature coefficient of the temperature 				| TEMPSENS
 *
 * @param[out]	 	p			:	(uint32_t)	final pressure valor in mBar
 *
 */
int32_t press_calibrate(uint32_t temperature, int32_t pressure, uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4, uint16_t C5, uint16_t C6);

/**
 * @brief	Pressure calibration for 15 <= Temperature < 20
 *
 * @param[in]	  	temperature	:	(uint32_t)	raw temperature data directly from the sensor
 * @param[in]		pressure	:	(long)		raw pressure data directly from the sensor
 * @param[in]	 	*C1			:	(uint16_t)	pointer to C1, Pressure sensitivity										| SENST1
 * @param[in]		*C2			:	(uint16_t)	pointer to C2, Pressure offset											| OFFT1
 * @param[in]		*C3			:	(uint16_t)	pointer to C3, Temperature coefficient of pressure sensitivity			| TCS
 * @param[in]		*C4			:	(uint16_t)	pointer to C4, Temperature coefficient of pressure offset				| TCO
 * @param[in]		*C5			:	(uint16_t)	pointer to C5, Reference temperature									| TREF
 * @param[in]		*C6			:	(uint16_t)	pointer to C6, Temperature coefficient of the temperature 				| TEMPSENS
 *
 * @param[out]	 	p			:	(uint32_t)	final pressure valor in mBar
 *
 */
int32_t press_calibrate_2(uint32_t temperature, int32_t pressure, uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4, uint16_t C5, uint16_t C6);

/**
 * @brief	Pressure calibration for Temperature < 15
 *
 * @param[in]	 	temperature	:	(uint32_t)	raw temperature data directly from the sensor
 * @param[in]		pressure	:	(long)		raw pressure data directly from the sensor
 * @param[in]	 	*C1			:	(uint16_t)	pointer to C1, Pressure sensitivity										| SENST1
 * @param[in]		*C2			:	(uint16_t)	pointer to C2, Pressure offset											| OFFT1
 * @param[in]		*C3			:	(uint16_t)	pointer to C3, Temperature coefficient of pressure sensitivity			| TCS
 * @param[in]		*C4			:	(uint16_t)	pointer to C4, Temperature coefficient of pressure offset				| TCO
 * @param[in]		*C5			:	(uint16_t)	pointer to C5, Reference temperature									| TREF
 * @param[in]		*C6			:	(uint16_t)	pointer to C6, Temperature coefficient of the temperature 				| TEMPSENS
 *
 * @param[out]	 	p			:	(uint32_t)	final pressure valor in mBar
 *
 */
int32_t press_calibrate_3(uint32_t temperature, int32_t pressure, uint16_t C1, uint16_t C2, uint16_t C3, uint16_t C4, uint16_t C5, uint16_t C6);

/**
 * @brief	Pressure reading
 * [OUT]	- Pressure calibration functions
 * 				* 	press_calibrate 	->	20 Celsius 	<  	temperatura
 * 				*	press_calibrate2  	->	15 Celsius 	<  	temperatura 	<= 	20 Celsius
 * 				*	press_calibrate3	->	temperatura	<	15 Celsius
 *
 * @param[in]	  	coef[5]			:	(uint16_t)		calibration coefficients array
 * @param[in]	  	t_no_compensate	:	(uint32_t)		raw temperature data directly from the sensor
 *
 * @param[out]		pressure		:	(long)			final compensate pressure
 *
 */
int32_t MS5611_pressure(uint16_t coef[5],  uint32_t t_no_compensate);

/**
 * @brief	Final pressure and temperature display function
 *
 * @param[in]	temperature		:	(long)		final temperature
 * @param[in]	pressure		:	(long)		final pressure
 */
void display_temp_press(long temperature, long pressure);

/*
 * @brief	Final temperature valor function
 *
 * @param[out]	aux	:	(float)		temperature reading in [Celsius]
 *
 */
float MS5611_read_temperature();

/**
 * @brief		Final pressure valor function
 *
 * @param[out]	aux	:	(float)		pressure reading in [mBar]
 *
 */
float MS5611_read_pressure();



```