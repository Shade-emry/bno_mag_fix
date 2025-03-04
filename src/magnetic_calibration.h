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
    uint32_t timestamp;           /* Timestamp in milliseconds */
} MFX_input_t;

typedef struct {
    int32_t rotation[MFX_NUM_AXES];              /* yaw, pitch and roll */
    int32_t quaternion[MFX_QNUM_AXES];           /* quaternion */
    int32_t gravity[MFX_NUM_AXES];               /* device frame gravity */
    int32_t linear_acceleration[MFX_NUM_AXES];   /* device frame linear acceleration */
    int32_t heading;                             /* heading */
    int32_t headingErr;                          /* heading error in deg */
    uint32_t calibration_points;                 /* number of calibration points collected */
    uint8_t quality;                             /* calibration quality */
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
    int32_t hi_bias[MFX_NUM_AXES];            // Hard iron offset array (Q16.16)
    int32_t si_matrix[MFX_NUM_AXES][MFX_NUM_AXES]; // Soft iron transformation matrix (Q16.16)
    bool si_valid;                            // Whether soft iron calibration is valid
    MFX_MagCal_quality_t quality;             // Calibration quality
    uint16_t sample_count;                    // Number of samples collected
    uint16_t calibration_points;              // Number of calibration points collected
    float confidence;                         // Calibration confidence (0.0-1.0)
    int32_t calibration_buffer[100 * MFX_NUM_AXES]; // Buffer for calibration points (100 points max)
} MFX_MagCal_output_t;

// Helper macros for fixed-point math
#define FX_MUL(a, b) (((int64_t)(a) * (b)) >> FX_SHIFT)
#define FX_DIV(a, b) (((int64_t)(a) << FX_SHIFT) / (b))
#define FX_SQRT(x) F_TO_FX(sqrt(FX_TO_F(x)))

// Matrix operations for soft iron compensation
inline void matrix_vector_multiply(const int32_t matrix[MFX_NUM_AXES][MFX_NUM_AXES], const int32_t vector[MFX_NUM_AXES], int32_t result[MFX_NUM_AXES]) {
    for (int i = 0; i < MFX_NUM_AXES; i++) {
        result[i] = 0;
        for (int j = 0; j < MFX_NUM_AXES; j++) {
            result[i] += FX_MUL(matrix[i][j], vector[j]);
        }
    }
}

// Initialize identity matrix for soft iron compensation
inline void init_identity_matrix(int32_t matrix[MFX_NUM_AXES][MFX_NUM_AXES]) {
    for (int i = 0; i < MFX_NUM_AXES; i++) {
        for (int j = 0; j < MFX_NUM_AXES; j++) {
            matrix[i][j] = (i == j) ? F_TO_FX(1.0f) : 0;
        }
    }
}

#endif // MAGNETIC_CALIBRATION_H
