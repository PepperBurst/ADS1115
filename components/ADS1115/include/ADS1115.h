#ifndef ADS1115_H
#define ADS1115_H

#include <driver/i2c.h>
#include <esp_log.h>

#define ADS1115_ADDRESS_ADDR_GND 0x48 // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD 0x49 // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA 0x4A // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL 0x4B // address pin tied to SCL pin
#define ADS1115_DEFAULT_ADDRESS ADS1115_ADDRESS_ADDR_GND

#define ADS1115_RA_CONVERSION 0x00
#define ADS1115_RA_CONFIG 0x01
#define ADS1115_RA_LO_THRESH 0x02
#define ADS1115_RA_HI_THRESH 0x03

#define ADS1115_MUX_P0_N1 0x00 // default
#define ADS1115_MUX_P0_N3 0x01
#define ADS1115_MUX_P1_N3 0x02
#define ADS1115_MUX_P2_N3 0x03
#define ADS1115_MUX_P0_NG 0x04
#define ADS1115_MUX_P1_NG 0x05
#define ADS1115_MUX_P2_NG 0x06
#define ADS1115_MUX_P3_NG 0x07

#define ADS1115_PGA_6P144 0x00
#define ADS1115_PGA_4P096 0x01
#define ADS1115_PGA_2P048 0x02 // default
#define ADS1115_PGA_1P024 0x03
#define ADS1115_PGA_0P512 0x04
#define ADS1115_PGA_0P256 0x05
#define ADS1115_PGA_0P256B 0x06
#define ADS1115_PGA_0P256C 0x07

#define ADS1115_MV_6P144 0.187500
#define ADS1115_MV_4P096 0.125000
#define ADS1115_MV_2P048 0.062500 // default
#define ADS1115_MV_1P024 0.031250
#define ADS1115_MV_0P512 0.015625
#define ADS1115_MV_0P256 0.007813
#define ADS1115_MV_0P256B 0.007813
#define ADS1115_MV_0P256C 0.007813

#define ADS1115_MODE_CONTINUOUS 0x00
#define ADS1115_MODE_SINGLESHOT 0x01 // default

#define ADS1115_COMP_LAT_NON_LATCHING 0x00 // default
#define ADS1115_COMP_LAT_LATCHING 0x01

#define ADS1115_BEGIN_SINGLESHOT 0x01

#define ADS1115_CFG_LATCH_BIT 2
#define ADS1115_CFG_MODE_BIT 8
#define ADS1115_CFG_GAIN_BIT 9
#define ADS1115_CFG_MUX_BIT 12
#define ADS1115_CFG_OS_BIT 15

#define ADS1115_WORD_MASK_LATCH 0b00000001 << ADS1115_CFG_LATCH_BIT
#define ADS1115_WORD_MASK_MODE 0b00000001 << ADS1115_CFG_MODE_BIT
#define ADS1115_WORD_MASK_GAIN 0b00000111 << ADS1115_CFG_GAIN_BIT
#define ADS1115_WORD_MASK_MUX 0b00000111 << ADS1115_CFG_MUX_BIT
#define ADS1115_WORD_MASK_OS 0b00000001 << ADS1115_CFG_OS_BIT

typedef struct
{
    int i2c_bus_number;
    uint8_t i2c_addess;
    uint8_t status;
    uint8_t mode;
    uint8_t pga;
    uint8_t mux;
} ADS1115;

esp_err_t ADS1115_read_bytes(ADS1115 *sensor, uint8_t reg_addr,
                             uint8_t *data, uint16_t len);
esp_err_t ADS1115_read_word(ADS1115 *sensor, uint8_t reg_addr, uint16_t *data);
esp_err_t ADS1115_write_bytes(ADS1115 *sensor, uint8_t reg_addr, uint8_t *data, uint16_t len);
esp_err_t ADS1115_write_word(ADS1115 *sensor, uint8_t reg_addr, uint16_t data);

esp_err_t ADS1115_write_bits_word(ADS1115 *sensor, uint8_t reg_addr, uint16_t data, uint16_t mask, uint8_t bit_offset);

esp_err_t ADS1115_init(ADS1115 *sensor, uint8_t address, int i2c_bus_number);

esp_err_t ADS1115_set_mode(ADS1115 *sensor, uint8_t mode);
esp_err_t ADS1115_set_gain(ADS1115 *sensor, uint8_t gain);
esp_err_t ADS1115_set_mux(ADS1115 *sensor, uint8_t mux);
esp_err_t ADS1115_set_comparator_latch(ADS1115 *sensor, uint8_t latch);

esp_err_t ADS1115_trigger_conversion(ADS1115 *sensor);

#endif // !ADS1115_H