#pragma once

#include <Wire.h>
#include <SPI.h>
#include "sh2.h"
#include "sh2_SensorValue.h"

#define BNO08X_DEFAULT_ADDRESS 0x4A
#define BNO08X_ALTERNATE_ADDRESS 0x4B

// Calibration commands
#define SH2_CAL_MAG 0x01
#define SH2_CAL_ON_TABLE 0x10

class BNO08x {
public:
  BNO08x();
  
  bool begin(uint8_t deviceAddress = BNO08X_DEFAULT_ADDRESS, TwoWire &wirePort = Wire, uint8_t intPin = 255);
  bool beginSPI(uint8_t csPin, uint8_t intPin = 255, uint8_t rstPin = 255, SPIClass &spiPort = SPI, uint32_t spiPortSpeed = 3000000);
  
  void enableDebugging(Stream &debugPort = Serial);
  void softReset();
  bool wasReset();
  
  bool receivePacket(void);
  bool waitForI2C();
  bool waitForSPI();
  
  bool enableRotationVector(uint16_t timeBetweenReports);
  bool enableGameRotationVector(uint16_t timeBetweenReports);
  bool enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
  bool enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
  bool enableGyroIntegratedRotationVector(uint16_t timeBetweenReports);
  
  bool enableAccelerometer(uint16_t timeBetweenReports);
  bool enableLinearAccelerometer(uint16_t timeBetweenReports);
  bool enableGyro(uint16_t timeBetweenReports);
  bool enableMagnetometer(uint16_t timeBetweenReports);
  
  bool enableTapDetector(uint16_t timeBetweenReports);
  bool enableStabilityClassifier(uint16_t timeBetweenReports);
  bool enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9]);
  
  bool dataAvailable();
  
  float getQuatI();
  float getQuatJ();
  float getQuatK();
  float getQuatReal();
  float getQuatRadianAccuracy();
  
  // Add this method for convenience
  void getQuat(float &i, float &j, float &k, float &real, float &radianAccuracy, uint8_t &accuracy);
  
  float getAccelX();
  float getAccelY();
  float getAccelZ();
  float getAccelAccuracy();
  
  float getLinAccelX();
  float getLinAccelY();
  float getLinAccelZ();
  float getLinAccelAccuracy();
  
  float getGyroX();
  float getGyroY();
  float getGyroZ();
  float getGyroAccuracy();
  
  // Add this method for convenience
  void getGyro(float &x, float &y, float &z, uint8_t &accuracy);
  
  float getMagX();
  float getMagY();
  float getMagZ();
  float getMagAccuracy();
  
  uint8_t getTapDetector();
  uint8_t getStabilityClassifier();
  
  uint8_t getActivityClassifier();
  float getActivityConfidence(uint8_t activity);
  
  void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
  void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
  
  void sendCommand(uint8_t command);
  void sendCalibrateCommand(uint8_t calibrateMe);
  void requestCalibrationStatus();
  
  bool getSensorConfiguration(sh2_SensorId_t sensorId, sh2_SensorConfig_t *pConfig);
  bool setSensorConfiguration(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);
  
  void getCalibrationStatus();
  uint8_t getCalibratedAccuracy();
  uint8_t getMagneticAccuracy();
  bool calibrateAccelerometer();
  bool calibrateGyro();
  bool calibrateMagnetometer();
  bool calibratePlanarAccelerometer();
  bool endCalibration();
  void saveCalibration();
  
private:
  TwoWire *_i2cPort;
  SPIClass *_spiPort;
  Stream *_debugPort;
  
  uint8_t _deviceAddress;
  
  uint8_t _cs;
  uint8_t _wake;
  uint8_t _int;
  uint8_t _rst;
  
  bool _printDebug = false;
  bool _spiFlag = false;
  bool _isReset = false;
  
  // These are the raw sensor values pulled from the reportID
  float rotationVector_Q1 = 0;
  float rotationVector_Q2 = 0;
  float rotationVector_Q3 = 0;
  float rotationVector_real = 0;
  float rotationVector_accuracy = 0;
  
  float accelerometer_X = 0;
  float accelerometer_Y = 0;
  float accelerometer_Z = 0;
  float accelerometer_accuracy = 0;
  
  float linear_accelerometer_X = 0;
  float linear_accelerometer_Y = 0;
  float linear_accelerometer_Z = 0;
  float linear_accelerometer_accuracy = 0;
  
  float gyro_X = 0;
  float gyro_Y = 0;
  float gyro_Z = 0;
  float gyro_accuracy = 0;
  
  float magnetometer_X = 0;
  float magnetometer_Y = 0;
  float magnetometer_Z = 0;
  float magnetometer_accuracy = 0;
  
  uint8_t tap_detector = 0;
  uint8_t stability_classifier = 0;
  
  uint8_t activity_classifier = 0;
  uint8_t _activityConfidences[9];
  
  // Scale factors
  float rotationVector_Q1_scale = 1.0 / (1 << 14);
  float rotationVector_Q2_scale = 1.0 / (1 << 14);
  float rotationVector_Q3_scale = 1.0 / (1 << 14);
  float rotationVector_real_scale = 1.0 / (1 << 14);
  
  uint8_t calibrationStatus = 0;
  uint8_t calibrationAccuracy = 0;
  uint8_t magneticAccuracy = 0;
  
  // Define the configuration options for the sensors
  sh2_SensorConfig_t sensorConfig;
  sh2_SensorId_t sensorID;
  
  int64_t last_timestamp = 0;
};
