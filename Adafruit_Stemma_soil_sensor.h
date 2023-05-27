#ifndef ADAFRUIT_STEMMA_SOIL_SENSOR_H
#define ADAFRUIT_STEMMA_SOIL_SENSOR_H

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t adafruit_stemma_soil_sensor_init(i2c_port_t i2c_num, int sda_pin, int scl_pin);
esp_err_t adafruit_stemma_soil_sensor_read_moisture(i2c_port_t i2c_num, uint16_t *moisture_value);
esp_err_t adafruit_stemma_soil_sensor_read_temperature(i2c_port_t i2c_num, float *temperature_value);

#ifdef __cplusplus
}
#endif

#endif // ADAFRUIT_STEMMA_SOIL_SENSOR_H
