#ifndef MAGNETIC_CALIBRATION_H
#define MAGNETIC_CALIBRATION_H

#include <Arduino.h>

// Number of axes for magnetometer data
#define MFX_NUM_AXES 3
#define MFX_QNUM_AXES 4  // For quaternions

// Fixed-point scaling (Q16.16 format)
#define FX_SHIFT 16
#define FX_SCALE (1 << FX_SHIFT)
#define FX_TO_F(x) ((float)(x) / FX_SCALE)
#define F_TO_FX(x) ((int32_t)((x) * FX_SCALE))

typedef struct {
    int32_t ATime;                            /* merge rate to the accel */
    int32_t MTime;                            /* merge rate to the mag */
    int32_t FTime;                            /* merge rate to the accel when external accelerations occurs */
    unsigned LMode;                         /* gyro bias learn mode: 0: static-learning, 1: dynamic learning */
    int32_t gbias_mag_th_sc;                 /* scaler for the gyro bias mag threshold nominal */
    int32_t gbias_acc_th_sc;                 /* scaler for the gyro bias acc threshold nominal */
    int32_t gbias_gyro_th_sc;                /* scaler for the gyro bias gyro threshold nominal */
    unsigned char modx;                     /* setting to indicate the decimation: set to 1 smartphone/tablet */
    char acc_orientation[MFX_NUM_AXES];     /* accelerometer data orientation */
    char gyro_orientation[MFX_NUM_AXES];    /* gyroscope data orientation */
    char mag_orientation[MFX_NUM_AXES];     /* magnetometer data orientation */
    int start_automatic_gbias_calculation;  /* 0: NED, 1: ENU */
} MFX_knobs_t;

typedef struct {
    int32_t mag[MFX_NUM_AXES];    /* Calibrated mag [uT/50] */
    int32_t acc[MFX_NUM_AXES];    /* Acceleration in [g] */
    int32_t gyro[MFX_NUM_AXES];   /* Angular rate [dps] */
} MFX_input_t;

typedef struct {
    int32_t rotation[MFX_NUM_AXES];              /* yaw, pitch and roll */
    int32_t quaternion[MFX_QNUM_AXES];           /* quaternion */
    int32_t gravity[MFX_NUM_AXES];               /* device frame gravity */
    int32_t linear_acceleration[MFX_NUM_AXES];   /* device frame linear acceleration */
    int32_t heading;                             /* heading */
    int32_t headingErr;                          /* heading error in deg */
} MFX_output_t;

// Magnetic calibration quality levels
typedef enum {
    MFX_MAGCAL_UNKNOWN = 0,
    MFX_MAGCAL_POOR,
    MFX_MAGCAL_OK,
    MFX_MAGCAL_GOOD
} MFX_MagCal_quality_t;

// Fixed-point magnetic calibration data
typedef struct {
    int32_t mag[MFX_NUM_AXES];     // Raw magnetometer data (Q16.16)
    uint32_t timestamp;            // Timestamp in ms
} MFX_MagCal_input_t;

typedef struct {
    int32_t hi_bias[MFX_NUM_AXES];  // Hard iron offset array (Q16.16)
    MFX_MagCal_quality_t quality;   // Calibration quality
    uint16_t sample_count;          // Number of samples collected
} MFX_MagCal_output_t;

// Helper macros for fixed-point math
#define FX_MUL(a, b) (((int64_t)(a) * (b)) >> FX_SHIFT)
#define FX_DIV(a, b) (((int64_t)(a) << FX_SHIFT) / (b))
#define FX_SQRT(x) F_TO_FX(sqrt(FX_TO_F(x)))

#endif // MAGNETIC_CALIBRATION_H
