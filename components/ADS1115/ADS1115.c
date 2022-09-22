#include <stdio.h>
#include "ADS1115.h"

esp_err_t ADS1115_read_bytes(ADS1115 *sensor, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sensor->i2c_addess << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sensor->i2c_bus_number, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGI("read bytes", "Read not ok");
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sensor->i2c_addess << 1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sensor->i2c_bus_number, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    for (int i = 0; i < len; i++)
    {
        ESP_LOGI("read bytes", "Byte #%d: %d", i, data[len]);
    }
    return (ret);
}

esp_err_t ADS1115_read_word(ADS1115 *sensor, uint8_t reg_addr, uint16_t *data)
{
    esp_err_t ret;
    uint16_t read_data = 0;
    uint8_t read_buffer[2];

    ret =
        ADS1115_read_bytes(sensor, reg_addr, read_buffer, 2);

    read_data = (read_buffer[0] << 8) | read_buffer[1];

    *data = read_data;

    return ret;
}

esp_err_t ADS1115_write_bytes(ADS1115 *sensor, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sensor->i2c_addess << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr, 1);
    i2c_master_write(cmd, data, len, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sensor->i2c_bus_number, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t ADS1115_write_word(ADS1115 *sensor, uint8_t reg_addr, uint16_t data)
{
    esp_err_t ret;
    uint8_t write_buffer[2];

    write_buffer[0] = (data >> 8) & 0xFF;
    write_buffer[1] = (data >> 0) & 0xFF;

    ret = ADS1115_write_bytes(sensor, reg_addr, write_buffer, 2);

    return ret;
}

esp_err_t ADS1115_write_bits_word(ADS1115 *sensor, uint8_t reg_addr, uint16_t data, uint16_t mask, uint8_t bit_offset)
{
    esp_err_t ret;

    uint16_t device_data;

    ret = ADS1115_read_word(sensor, reg_addr, &device_data);

    device_data = device_data & ~(ADS1115_WORD_MASK_MODE);

    uint16_t bits_to_write = data << bit_offset;

    device_data = device_data | bits_to_write;

    ret = ADS1115_write_word(sensor, reg_addr, device_data);

    return ret;
}

esp_err_t ADS1115_init(ADS1115 *sensor, uint8_t address, int i2c_bus_number)
{
    sensor->i2c_bus_number = i2c_bus_number;

    sensor->i2c_addess = address;

    return ESP_OK;
}

esp_err_t ADS1115_set_mode(ADS1115 *sensor, uint8_t mode)
{
    esp_err_t ret;

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, mode, ADS1115_WORD_MASK_MODE, ADS1115_CFG_MODE_BIT);

    sensor->mode = mode;

    return ret;
}

esp_err_t ADS1115_set_gain(ADS1115 *sensor, uint8_t gain)
{
    esp_err_t ret;

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, gain, ADS1115_WORD_MASK_GAIN, ADS1115_CFG_GAIN_BIT);

    sensor->pga = gain;

    return ret;
}

esp_err_t ADS1115_set_mux(ADS1115 *sensor, uint8_t mux)
{
    esp_err_t ret;

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, mux, ADS1115_WORD_MASK_MUX, ADS1115_CFG_MUX_BIT);

    sensor->mux = mux;

    return ret;
}

esp_err_t ADS1115_set_comparator_latch(ADS1115 *sensor, uint8_t latch)
{
    esp_err_t ret;

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, latch, ADS1115_WORD_MASK_LATCH, ADS1115_CFG_LATCH_BIT);

    return ret;
}

esp_err_t ADS1115_trigger_conversion(ADS1115 *sensor)
{
    esp_err_t ret;

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, ADS1115_BEGIN_SINGLESHOT, ADS1115_WORD_MASK_OS, ADS1115_CFG_OS_BIT);

    return ret;
}