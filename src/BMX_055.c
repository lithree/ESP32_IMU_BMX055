/*
 * File: bmx055_iic.c
 * Brief:
 *   BMX055 I2C driver for ESP32 / FreeRTOS.
 *   Unchanged from the original version except:
 *     - now includes bmx055_iic.h (own header)
 *     - adds BMX055_ReadAccelRaw/ReadGyroRaw/ReadMagRaw, which return
 *       signed raw counts for use by imu_app.c's Madgwick fusion layer.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "bmx_055.h"

#define I2C_MASTER_SCL_IO 13
#define I2C_MASTER_SDA_IO 14
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_CHIPID_REG_ACC 0x00
#define I2C_CHIPID_REG_GYRO 0x00
#define I2C_CHIPID_REG_MAG 0x40

#define BMX055_ADDR_ACC 0x18
#define BMX055_ADDR_GYRO 0x68
#define BMX055_ADDR_MAG 0x10

#define RANGE_PMU_ACC 0x0f
#define PMU_BW_ACC    0x10
#define PMU_LPW_ACC   0x11

#define RANGE_GYRO        0x0F
#define BW_GYRO              0x10
#define LPM1_GYRO            0x11

#define POWER_CTRL_MAG       0x4B
#define OP_MODE_MAG          0x4C

#define BMX055_D_X_LSB_ACC   0x02
#define BMX055_RATE_X_LSB_GYRO      0x02
#define DATA_X_LSB_MAG       0x42

static const char *TAG = "BMX055_IIC";

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t bmx055_write_register(uint8_t device_address, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, device_address, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t bmx055_read_register(uint8_t device_address, uint8_t reg_addr, uint8_t *data)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        device_address,
        &reg_addr,
        1,
        data,
        1,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t bmx055_read_registers(uint8_t device_address, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        device_address,
        &reg_addr,
        1,
        data,
        len,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/* ------------------------------------------------------------------------ */
/* Raw count reads — added for imu_app.c                                    */
/* ------------------------------------------------------------------------ */

esp_err_t BMX055_ReadAccelRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    esp_err_t err = bmx055_read_registers(BMX055_ADDR_ACC, BMX055_D_X_LSB_ACC, data, 6);
    if (err != ESP_OK) {
        return err;
    }

    /* Accel data is 12-bit, left-justified in the 16-bit register pair. */
    *x = (int16_t)((data[1] << 8) | data[0]) >> 4;
    *y = (int16_t)((data[3] << 8) | data[2]) >> 4;
    *z = (int16_t)((data[5] << 8) | data[4]) >> 4;
    return ESP_OK;
}

esp_err_t BMX055_ReadGyroRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    esp_err_t err = bmx055_read_registers(BMX055_ADDR_GYRO, BMX055_RATE_X_LSB_GYRO, data, 6);
    if (err != ESP_OK) {
        return err;
    }

    /* Gyro data is a full 16-bit signed value. */
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
    return ESP_OK;
}

esp_err_t BMX055_ReadMagRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    esp_err_t err = bmx055_read_registers(BMX055_ADDR_MAG, DATA_X_LSB_MAG, data, 6);
    if (err != ESP_OK) {
        return err;
    }

    /* Mag X/Y are 13-bit, Z is 15-bit, all left-justified in the register pair. */
    *x = (int16_t)((data[1] << 8) | data[0]) >> 3;
    *y = (int16_t)((data[3] << 8) | data[2]) >> 3;
    *z = (int16_t)((data[5] << 8) | data[4]) >> 1;
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */

void bmx055_init(void)
{
    uint8_t accel_id = 0;
    uint8_t gyro_id = 0;
    uint8_t mag_id = 0;
    if (bmx055_read_register(BMX055_ADDR_ACC, I2C_CHIPID_REG_ACC, &accel_id) == ESP_OK)
    {
        ESP_LOGI(TAG, "Accel Chip ID: 0x%02X", accel_id);
    }
    else
    {
        ESP_LOGE(TAG, "Accel ID read fail");
    }

    if (bmx055_read_register(BMX055_ADDR_GYRO, I2C_CHIPID_REG_GYRO, &gyro_id) == ESP_OK)
    {
        ESP_LOGI(TAG, "Gyro Chip ID: 0x%02X", gyro_id);
    }
    else
    {
        ESP_LOGE(TAG, "Gyro ID read fail");
    }
    if (bmx055_read_register(BMX055_ADDR_MAG, I2C_CHIPID_REG_MAG, &mag_id) == ESP_OK)
    {
        ESP_LOGI(TAG, "Magno Chip ID: 0x%02X", mag_id);
    }
    else
    {
        ESP_LOGE(TAG, "Magno ID read fail");
    }
    // Accel
    // +/- 2g
    bmx055_write_register(BMX055_ADDR_ACC, RANGE_PMU_ACC, 0x03);
    // 62.5Hz (ODR 125Hz)
    bmx055_write_register(BMX055_ADDR_ACC, PMU_BW_ACC, 0x0B);
    // NORMAL mode
    bmx055_write_register(BMX055_ADDR_ACC, PMU_LPW_ACC, 0x00);

    // Gyro
    // +/- 500 deg/s
    bmx055_write_register(BMX055_ADDR_GYRO, RANGE_GYRO, 0x00);
    //  100Hz, BW 32Hz (ODR 100Hz)
    bmx055_write_register(BMX055_ADDR_GYRO, BW_GYRO, 0x07);
    // NORMAL mode
    bmx055_write_register(BMX055_ADDR_GYRO, LPM1_GYRO, 0x00);

    //MAG
    // Pwr Ctrl -> 1 (suspend mode to sleep mode)
    bmx055_write_register(BMX055_ADDR_MAG, POWER_CTRL_MAG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Normal Mode
    bmx055_write_register(BMX055_ADDR_MAG, OP_MODE_MAG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    vTaskDelay(pdMS_TO_TICKS(50));
}

void bmx055_accel_read_task(void *pvParameters) {
    int16_t x, y, z;
    float x_g, y_g, z_g;

    while (1) {
        if (BMX055_ReadAccelRaw(&x, &y, &z) == ESP_OK) {
            x_g = (float)x * 0.0009765625f;
            y_g = (float)y * 0.0009765625f;
            z_g = (float)z * 0.0009765625f;

            ESP_LOGI(TAG, "X: %8.3f g  |  Y: %8.3f g  |  Z: %8.3f g", x_g, y_g, z_g);
        } else {
            ESP_LOGE(TAG, "ACCEL Read Error");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void bmx055_gyro_read_task(void *pvParameters) {
    int16_t x, y, z;
    float x_dps, y_dps, z_dps;

    while (1) {
        if (BMX055_ReadGyroRaw(&x, &y, &z) == ESP_OK) {
            x_dps = (float)x * 0.0609756f;
            y_dps = (float)y * 0.0609756f;
            z_dps = (float)z * 0.0609756f;

            ESP_LOGI(TAG, "X: %8.3f dps | Y: %8.3f dps | Z: %8.3f dps", x_dps, y_dps, z_dps);
        } else {
            ESP_LOGE(TAG, "GYRO Read ERROR");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void bmx055_mag_read_task(void *pvParameters) {
    int16_t x, y, z;
    float x_ut, y_ut, z_ut;

    while (1) {
        if (BMX055_ReadMagRaw(&x, &y, &z) == ESP_OK) {
            x_ut = (float)x * 0.3f;
            y_ut = (float)y * 0.3f;
            z_ut = (float)z * 0.3f;

            ESP_LOGI(TAG, "X: %8.2f uT | Y: %8.2f uT | Z: %8.2f uT", x_ut, y_ut, z_ut);
        } else {
            ESP_LOGE(TAG, "MAG Read Error");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}