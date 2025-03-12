#pragma once

#include <stdint.h>
#include <stdbool.h>

// SH2 Error Types
typedef enum {
  SH2_OK = 0,
  SH2_ERR = -1,
  SH2_ERR_BAD_PARAM = -2,
  SH2_ERR_OP_IN_PROGRESS = -3,
  SH2_ERR_IO = -4,
  SH2_ERR_HUB = -5,
  SH2_ERR_TIMEOUT = -6,
} sh2_Err_t;

// Sensor IDs
typedef enum {
  SH2_ROTATION_VECTOR = 0x05,
  SH2_GAME_ROTATION_VECTOR = 0x08,
  SH2_GEOMAGNETIC_ROTATION_VECTOR = 0x09,
  SH2_ARVR_STABILIZED_ROTATION_VECTOR = 0x28,
  SH2_ARVR_STABILIZED_GAME_ROTATION_VECTOR = 0x29,
  SH2_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A,
  SH2_ACCELEROMETER = 0x01,
  SH2_GYROSCOPE = 0x02,
  SH2_MAGNETIC_FIELD = 0x03,
  SH2_LINEAR_ACCELERATION = 0x04,
  SH2_GRAVITY = 0x06,
  SH2_PRESSURE = 0x07,
  SH2_AMBIENT_LIGHT = 0x0A,
  SH2_HUMIDITY = 0x0B,
  SH2_PROXIMITY = 0x0C,
  SH2_TEMPERATURE = 0x0D,
  SH2_RESERVED = 0x0E,
  SH2_TAP_DETECTOR = 0x10,
  SH2_STEP_COUNTER = 0x11,
  SH2_STEP_DETECTOR = 0x12,
  SH2_SIGNIFICANT_MOTION = 0x13,
  SH2_STABILITY_CLASSIFIER = 0x14,
  SH2_SHAKE_DETECTOR = 0x15,
  SH2_FLIP_DETECTOR = 0x16,
  SH2_PICKUP_DETECTOR = 0x17,
  SH2_STABILITY_DETECTOR = 0x18,
  SH2_PERSONAL_ACTIVITY_CLASSIFIER = 0x19,
  SH2_SLEEP_DETECTOR = 0x1A,
  SH2_TILT_DETECTOR = 0x1B,
  SH2_POCKET_DETECTOR = 0x1C,
  SH2_CIRCLE_DETECTOR = 0x1D,
  SH2_HEART_RATE_MONITOR = 0x1E,
  SH2_ACTIVITY_CLASSIFIER = 0x1F,
} sh2_SensorId_t;

// Product ID structure
typedef struct {
  uint8_t resetCause;
  uint8_t swVersionMajor;
  uint8_t swVersionMinor;
  uint32_t swPartNumber;
  uint32_t swBuildNumber;
  uint16_t swVersionPatch;
  uint8_t partNumber;
} sh2_ProductIds_t;

// Sensor configuration structure
typedef struct {
  uint8_t changeSensitivityEnabled;
  float changeSensitivity;
  uint8_t wakeupEnabled;
  uint16_t wakeupThreshold;
  uint32_t batchInterval_us;
  uint32_t sensorSpecific;
} sh2_SensorConfig_t;

// Sensor event structure
typedef struct {
  uint8_t reportId;
  uint32_t timestamp;
  uint8_t status;
  uint16_t delay;
  uint8_t data[32];
} sh2_SensorEvent_t;

// Sensor value structure
typedef struct {
  uint64_t timestamp;
  sh2_SensorId_t sensorId;
  union {
    struct {
      float i;
      float j;
      float k;
      float real;
      float accuracy;
    } rotationVector;
    
    struct {
      float x;
      float y;
      float z;
      float accuracy;
    } accelerometer;
    
    struct {
      float x;
      float y;
      float z;
      float accuracy;
    } magnetometer;
    
    struct {
      float x;
      float y;
      float z;
      float accuracy;
    } gyroscope;
    
    struct {
      float x;
      float y;
      float z;
      float accuracy;
    } linearAcceleration;
    
    struct {
      uint8_t classification;
    } stability;
    
    struct {
      uint8_t detection;
    } tap;
    
    struct {
      uint8_t classification;
      uint8_t confidence[9];
    } activity;
  } un;
} sh2_SensorValue_t;

// Callback function types
typedef void (*sh2_EventCallback_t)(void *cookie, sh2_SensorEvent_t *pEvent);
typedef void (*sh2_ErrorCallback_t)(void *cookie, sh2_Err_t error);

// Function prototypes
void sh2_registerEventCallback(sh2_EventCallback_t eventCallback, void *cookie);
void sh2_registerErrorCallback(sh2_ErrorCallback_t errorCallback, void *cookie);

sh2_Err_t sh2_initialize();
sh2_Err_t sh2_getProdIds(sh2_ProductIds_t *pProdIds);
sh2_Err_t sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);
sh2_Err_t sh2_getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *pConfig);
sh2_Err_t sh2_sendReset();

bool sh2_decodeSensorEvent(sh2_SensorValue_t *pValue, const sh2_SensorEvent_t *pEvent);
