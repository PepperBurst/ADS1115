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
        ESP_LOGI("read bytes", "Address: %02X Byte #%d: %02X", reg_addr, i, data[i]);
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

    for (int i = 0; i < len; i++)
    {
        ESP_LOGI("write bytes", "Address: %02X Byte #%d: %02X", reg_addr, i, data[i]);
    }
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

    ESP_LOGI("write_bits_word", "Source data: %04x", device_data);

    device_data = device_data & ~(mask);

    uint16_t bits_to_write = data << bit_offset;

    ESP_LOGI("write_bits_word", "Bits to write: %04x", bits_to_write);

    device_data = device_data | bits_to_write;

    ESP_LOGI("write_bits_word", "Result data: %04x", device_data);

    ret = ADS1115_write_word(sensor, reg_addr, device_data);

    return ret;
}

esp_err_t ADS1115_init(ADS1115 *sensor, uint8_t address, int i2c_bus_number)
{
    sensor->i2c_bus_number = i2c_bus_number;

    sensor->i2c_addess = address;

    uint16_t data;

    ADS1115_read_word(sensor, ADS1115_RA_CONFIG, &data);

    ESP_LOGI("init", "Config data: %04X", data);

    ADS1115_read_word(sensor, ADS1115_RA_HI_THRESH, &data);

    ESP_LOGI("init", "Hi-thresh data: %04x", data);

    ADS1115_read_word(sensor, ADS1115_RA_LO_THRESH, &data);

    ESP_LOGI("init", "Lo-thresh data: %04x", data);

    return ESP_OK;
}

esp_err_t ADS1115_reset(ADS1115 *sensor)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x00 << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, 0x06, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sensor->i2c_bus_number, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t ADS1115_set_mode(ADS1115 *sensor, uint8_t mode)
{
    esp_err_t ret;

    ESP_LOGI("set mode", "mode: %d", mode);

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, mode, ADS1115_WORD_MASK_MODE, ADS1115_CFG_MODE_BIT);

    sensor->mode = mode;

    return ret;
}

esp_err_t ADS1115_set_gain(ADS1115 *sensor, uint8_t gain)
{
    esp_err_t ret;

    ESP_LOGI("set gain", "gain: %d", gain);

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, gain, ADS1115_WORD_MASK_GAIN, ADS1115_CFG_GAIN_BIT);

    sensor->pga = gain;

    switch (gain)
    {
    case ADS1115_PGA_6P144:
        sensor->bit_equivalent = ADS1115_MV_6P144;
        break;
    case ADS1115_PGA_4P096:
        sensor->bit_equivalent = ADS1115_MV_4P096;
        break;
    case ADS1115_PGA_2P048:
        sensor->bit_equivalent = ADS1115_MV_2P048;
        break;
    case ADS1115_PGA_1P024:
        sensor->bit_equivalent = ADS1115_MV_1P024;
        break;
    case ADS1115_PGA_0P512:
        sensor->bit_equivalent = ADS1115_MV_0P512;
        break;
    case ADS1115_PGA_0P256:
        sensor->bit_equivalent = ADS1115_MV_0P256;
        break;
    case ADS1115_PGA_0P256B:
        sensor->bit_equivalent = ADS1115_MV_0P256B;
        break;
    case ADS1115_PGA_0P256C:
        sensor->bit_equivalent = ADS1115_MV_0P256C;
        break;

    default:
        break;
    }

    return ret;
}

esp_err_t ADS1115_set_mux(ADS1115 *sensor, uint8_t mux)
{
    esp_err_t ret;

    ESP_LOGI("set mux", "mux: %d", mux);

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, mux, ADS1115_WORD_MASK_MUX, ADS1115_CFG_MUX_BIT);

    sensor->mux = mux;

    return ret;
}

esp_err_t ADS1115_set_comparator_latch(ADS1115 *sensor, uint8_t latch)
{
    esp_err_t ret;

    ESP_LOGI("set comp_latch", "comp_latch: %d", latch);

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, latch, ADS1115_WORD_MASK_LATCH, ADS1115_CFG_LATCH_BIT);

    return ret;
}

esp_err_t ADS1115_set_comparator_queue(ADS1115 *sensor, uint8_t queue)
{
    esp_err_t ret;

    ESP_LOGI("set comp_queue", "comp_queue: %d", queue);

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, queue, ADS1115_WORD_MASK_COMP_QUEUE, ADS1115_CFG_COMP_QUE_BIT);

    return ret;
}

esp_err_t ADS1115_set_ready_mode(ADS1115 *sensor)
{
    esp_err_t ret;

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_HI_THRESH, ADS1115_THRESH_READY_MODE, ADS1115_WORD_MASK_COMP_QUEUE, ADS1115_THRESH_READY_BIT);

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_LO_THRESH, ADS1115_THRESH_READY_MODE, ADS1115_WORD_MASK_COMP_QUEUE, ADS1115_THRESH_READY_BIT);

    return ret;
}

esp_err_t ADS1115_trigger_conversion(ADS1115 *sensor)
{
    esp_err_t ret;

    ret = ADS1115_write_bits_word(sensor, ADS1115_RA_CONFIG, ADS1115_BEGIN_SINGLESHOT, ADS1115_WORD_MASK_OS, ADS1115_CFG_OS_BIT);

    return ret;
}

esp_err_t ADS1115_get_conversion(ADS1115 *sensor)
{
    esp_err_t ret;

    uint16_t conversion;

    ret = ADS1115_read_word(sensor, ADS1115_RA_CONVERSION, &conversion);

    sensor->conversion[sensor->mux] = conversion;

    float volt_conversion = 0.0f;

    uint16_t signal_data = conversion & 0x7FFF;

    float sign = 1.0f;

    if (conversion & 0x8000)
    {
        sign = -1.0f;

        signal_data = ~(signal_data);
    }

    volt_conversion = signal_data * sensor->bit_equivalent;

    sensor->millivolts[sensor->mux] = volt_conversion * sign;

    return ret;
}

float ADS1115_get_millivolts(ADS1115 *sensor)
{
    return (sensor->millivolts[sensor->mux]);
}