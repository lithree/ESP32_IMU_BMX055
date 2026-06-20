/*
 * File: bmx055_iic.h
 * Brief:
 *   Public interface for the BMX055 I2C driver (ESP32 / FreeRTOS).
 *   Adds raw-count read functions (BMX055_ReadAccelRaw / ReadGyroRaw /
 *   ReadMagRaw) alongside the original register-level helpers, so that
 *   imu_app.c can apply its own offset + scale instead of consuming
 *   already-converted physical units.
 */

#ifndef BMX_055_H
#define BMX_055_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Bring-up ------------------------------------------------------------ */
esp_err_t i2c_master_init(void);
void bmx055_init(void);

/* --- Low level register access ------------------------------------------ */
esp_err_t bmx055_write_register(uint8_t device_address, uint8_t reg_addr, uint8_t data);
esp_err_t bmx055_read_register(uint8_t device_address, uint8_t reg_addr, uint8_t *data);
esp_err_t bmx055_read_registers(uint8_t device_address, uint8_t reg_addr, uint8_t *data, size_t len);

/* --- Raw count reads -------------------------------- *
 * These mirror the bit-shift/sign-extend logic already used by the
 * logging tasks below, but stop short of multiplying by a physical-unit
 * scale factor, returning the signed LSB counts straight from the
 * sensor. imu_app.c expects exactly this: raw counts in, its own
 * offsets and GYRO_CALIBRATION_COFF-style scaling applied internally. */
esp_err_t BMX055_ReadAccelRaw(int16_t *x, int16_t *y, int16_t *z);
esp_err_t BMX055_ReadGyroRaw(int16_t *x, int16_t *y, int16_t *z);
esp_err_t BMX055_ReadMagRaw(int16_t *x, int16_t *y, int16_t *z);

/* --- Original physical-unit logging tasks --------- */
void bmx055_accel_read_task(void *pvParameters);
void bmx055_gyro_read_task(void *pvParameters);
void bmx055_mag_read_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif