/*
 * File: main.c
 * Brief:
 *   Application entry point. Replaces the old per-sensor logging tasks
 *   with a single call into the ported imu_app, which runs Madgwick
 *   fusion on BMX055 data and logs pitch/roll/yaw.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "imu_app.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "IMU app init...");
    IMU_App_Init();              /* I2C + BMX055 bring-up, gyro calibration, initial attitude */
    ESP_LOGI(TAG, "IMU app init complete");

    /* Runs IMU_App_Update() at ~200 Hz and logs pitch/roll/yaw every 200 ms. */
    IMU_App_StartTask(200);

}