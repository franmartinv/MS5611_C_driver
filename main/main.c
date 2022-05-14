/**
 * MS5611 project
 *
 */


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "MS5611.h"

void hal_i2c_init();


void app_main(void)
{
	float t,p;

	hal_i2c_init();			// Master initialition
	MS5611_init();			// MS5611 slave initialition

	while(1) {
		t = MS5611_read_temperature();					// Temperature reading
		p = MS5611_read_pressure();						// Pressure reading
		printf("Temperature: %f\nPressure: %f\n",t,p);
		vTaskDelay(3000/portTICK_PERIOD_MS);
	}
}
