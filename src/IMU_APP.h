/*
 * File: imu_app.h
 * Brief:
 *   Public interface for the Madgwick-based IMU application layer.
 *   Ported from a generic MPU6050+compass backend to the BMX055
 *   driver (bmx055_iic.c/.h) used on ESP32 / FreeRTOS.
 *
 *   This header combines what used to be three files (imu_app.h,
 *   imu_filter.h, imu_math.h) into one, since the filter and math
 *   helpers exist only to support this module and aren't reused
 *   anywhere else in the project.
 *
 *   Call order:
 *     IMU_App_Init()    - once, at startup (initializes I2C + BMX055 + AHRS state)
 *     IMU_App_Update()  - once per loop iteration (~200 Hz recommended)
 *     IMU_App_GetState()/IMU_App_GetEuler() - read the latest pitch/roll/yaw
 */

#ifndef IMU_APP_H
#define IMU_APP_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Math helpers (formerly imu_math.h)
 * ====================================================================== */

#define IMU_DEG2RAD  (0.017453292f)   /* PI / 180 */
#define IMU_RAD2DEG  (57.295780f)     /* 180 / PI */

/* Fast inverse square-root. A plain 1/sqrtf is used here for correctness
 * and portability; swap in a quake-style approximation later if needed
 * for performance on a constrained MCU. */
static inline float IMU_InvSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

/* sqrt(x*x + y*y + z*z), used for magnetometer field-strength health check. */
static inline float IMU_Pythagorous3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

/* ========================================================================
 * Basic vector / quaternion types
 * ====================================================================== */

typedef struct {
    float x;
    float y;
    float z;
} IMU_Vector3f;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} IMU_Quaternion;

/* ========================================================================
 * Filter types and API (formerly imu_filter.h)
 * Simple 2nd-order Butterworth low-pass, one instance per axis;
 * s_*_param is shared per sensor type, s_*_buf is per-axis.
 * ====================================================================== */

typedef struct {
    float a0, a1, a2;   /* feed-forward (b) and feedback (a) coefficients   */
    float b0, b1, b2;
} IMU_ButterParam;

typedef struct {
    float x1, x2;        /* previous two raw inputs  */
    float y1, y2;        /* previous two filtered outputs */
} IMU_ButterBuffer;

/* Compute Butterworth biquad coefficients for the given sample rate and
 * cutoff frequency (both in Hz) and store them in *param. */
void IMU_FilterSetCutoff(float sample_hz, float cutoff_hz, IMU_ButterParam *param);

/* Apply one sample of the filter described by *param, using and updating
 * the per-axis history in *buf. Returns the filtered output sample. */
float IMU_FilterApply(float input, IMU_ButterBuffer *buf, const IMU_ButterParam *param);

/* ========================================================================
 * Full IMU state snapshot
 * ====================================================================== */

typedef struct {
    /* Raw sensor counts (as read from the device, pre-offset) */
    IMU_Vector3f gyro_raw;
    IMU_Vector3f accel_raw;
    IMU_Vector3f mag_raw;
    float        temperature_deg;

    /* Calibration offsets */
    IMU_Vector3f gyro_offset;
    IMU_Vector3f accel_offset;
    IMU_Vector3f mag_offset;

    /* Offset-corrected values */
    IMU_Vector3f gyro;
    IMU_Vector3f accel;
    IMU_Vector3f mag;

    /* Low-pass filtered values used by the fusion step */
    IMU_Vector3f gyro_filtered;
    IMU_Vector3f accel_filtered;
    IMU_Vector3f mag_filtered;

    bool mag_healthy;

    /* Fusion state */
    IMU_Quaternion quat;
    float dt_s;

    /* Output Euler angles, degrees */
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
} IMU_State;

/* ========================================================================
 * Public application API
 * ====================================================================== */

/* Bring up I2C, the BMX055 sensors, calibrate gyro bias and build the
 * initial attitude quaternion from accel + mag. Call once at startup. */
void IMU_App_Init(void);

/* Run one full update cycle: timing, sensor read, filtering, AHRS fusion,
 * Euler conversion. Call this periodically (target ~200 Hz / 5 ms). */
void IMU_App_Update(void);

/* Return a pointer to the latest full state snapshot (read-only). */
const IMU_State *IMU_App_GetState(void);

/* Convenience accessors for just the Euler outputs. */
float IMU_App_GetPitch(void);
float IMU_App_GetRoll(void);
float IMU_App_GetYaw(void);

/* Start a FreeRTOS task that calls IMU_App_Update() in a loop and logs
 * pitch/roll/yaw at the given interval. Call IMU_App_Init() before this. */
void IMU_App_StartTask(uint32_t log_period_ms);

#ifdef __cplusplus
}
#endif

#endif /* IMU_APP_H */