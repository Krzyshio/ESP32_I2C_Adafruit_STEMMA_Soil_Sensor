#include "Adafruit_Stemma_soil_sensor.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_system.h"

#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define STEMMA_SENSOR_ADDR 0x36
#define STEMMA_MOISTURE_BASE_REG 0x0F
#define STEMMA_MOISTURE_FUNC_REG 0x10
#define STEMMA_TEMP_BASE_REG 0x00
#define STEMMA_TEMP_FUNC_REG 0x04

#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL I2C_MASTER_ACK
#define NACK_VAL I2C_MASTER_NACK

static const char *TAG = "ADAFRUIT_STEMMA_SOIL_SENSOR_H";

esp_err_t adafruit_stemma_soil_sensor_init(i2c_port_t i2c_num, int sda_pin, int scl_pin)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    i2c_param_config(i2c_num, &conf);
    return i2c_driver_install(i2c_num, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t write_to_sensor(i2c_port_t i2c_num, uint8_t addr, uint8_t base_reg, uint8_t func_reg)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, base_reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, func_reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void delay_ms(int ms)
{
    vTaskDelay((ms) / portTICK_PERIOD_MS);
}

static esp_err_t read_from_sensor(i2c_port_t i2c_num, uint8_t addr, uint8_t *data, int len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, len - 1, ACK_VAL);
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t adafruit_stemma_soil_sensor_read_moisture(i2c_port_t i2c_num, uint16_t *moisture_value)
{
    esp_err_t ret;
    int len = 2;
    uint8_t *moisture_data = (uint8_t *)malloc(len);

    ret = write_to_sensor(i2c_num, STEMMA_SENSOR_ADDR, STEMMA_MOISTURE_BASE_REG, STEMMA_MOISTURE_FUNC_REG);
    if (ret != ESP_OK) 
    {
        ESP_LOGW(TAG, "Write to I2C sensor failed");
        free(moisture_data);
        return ret;
    }

    delay_ms(50);

    ret = read_from_sensor(i2c_num, STEMMA_SENSOR_ADDR, moisture_data, len);
    if (ret == ESP_OK)
    {
        *moisture_value = ((uint16_t)moisture_data[0] << 8) | moisture_data[1];
    }
    else
    {
        ESP_LOGW(TAG, "Read I2C sensor failed");
    }

    free(moisture_data);
    return ret;
}

esp_err_t adafruit_stemma_soil_sensor_read_temperature(i2c_port_t i2c_num, float *temperature_value)
{
    esp_err_t ret;
    int len = 4;
    uint8_t *temp_data = (uint8_t *)malloc(len);

    ret = write_to_sensor(i2c_num, STEMMA_SENSOR_ADDR, STEMMA_TEMP_BASE_REG, STEMMA_TEMP_FUNC_REG);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Write to I2C sensor failed");
        free(temp_data);
        return ret;
    }

    delay_ms(50);

    ret = read_from_sensor(i2c_num, STEMMA_SENSOR_ADDR, temp_data, len);
    if (ret == ESP_OK)
    {
        int32_t raw_temp = ((uint32_t)temp_data[0] << 24) | ((uint32_t)temp_data[1] << 16) | ((uint32_t)temp_data[2] << 8) | temp_data[3];
        *temperature_value = (1.0 / (1UL << 16)) * raw_temp;
    }
    else
    {
        ESP_LOGW(TAG, "Read I2C sensor failed");
    }

    free(temp_data);
    return ret;
}