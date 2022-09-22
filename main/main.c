#include "ADS1115.h"
#include "driver/i2c.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define PIN_LED GPIO_NUM_19
#define PIN_SDA GPIO_NUM_22
#define PIN_SCL GPIO_NUM_23
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000
#define ADS1115_I2C_ADDRESS 0x48

void vReadAdc(void *pvParameters);

ADS1115 adc_ic_1;

void app_main(void)
{
       printf("Hello world!\n");

       /* Print chip information */
       esp_chip_info_t chip_info;
       esp_chip_info(&chip_info);
       printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
              (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
              (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

       printf("silicon revision %d, ", chip_info.revision);

       printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
              (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                            : "external");

       i2c_config_t i2c_conf = {.mode = I2C_MODE_MASTER,
                                .sda_io_num = PIN_SDA,
                                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                                .scl_io_num = PIN_SCL,
                                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                                .master.clk_speed = 100000};
       i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
       i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);

       uint8_t status = ADS1115_init(&adc_ic_1, ADS1115_I2C_ADDRESS, I2C_MASTER_NUM);

       ESP_LOGI("ADS1115", "Sensor Address %02x", adc_ic_1.i2c_addess);

       xTaskCreate(&vReadAdc, "ADC IC Task", 4096, NULL, 1, NULL);
}

void vReadAdc(void *pvParameters)
{
       ADS1115_set_mode(&adc_ic_1, ADS1115_MODE_SINGLESHOT);
       ADS1115_set_gain(&adc_ic_1, ADS1115_PGA_2P048);
       ADS1115_set_mux(&adc_ic_1, ADS1115_MUX_P0_NG);
       ADS1115_set_comparator_latch(&adc_ic_1, ADS1115_COMP_LAT_LATCHING);
       vTaskDelay(pdMS_TO_TICKS(1000));
       ADS1115_trigger_conversion(&adc_ic_1);
       for (;;)
       {
              ESP_LOGI("Read ADC", "Read 1 sec");
              vTaskDelay(pdMS_TO_TICKS(1000));
       }
}