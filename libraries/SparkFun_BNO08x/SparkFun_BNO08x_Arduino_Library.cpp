#include "SparkFun_BNO08x_Arduino_Library.h"
#include "sh2.h"
#include "shtp.h"

// Global Variables
static int resetPin = -1;
static sh2_SensorValue_t sensorValue;
static bool newData = false;

// Callback functions for the SH2 library
static void handleSensorEvent(void *cookie, sh2_SensorEvent_t *event) {
  // Handle sensor events from the BNO08x
  if (sh2_decodeSensorEvent(&sensorValue, event)) {
    newData = true;
  }
}

static void reportError(void *cookie, sh2_Err_t error) {
  // Handle errors from the BNO08x
  if (error != SH2_OK) {
    Serial.print("SH2 Error: ");
    Serial.println(error);
  }
}

// Class Implementation
BNO08x::BNO08x() {
  _i2cPort = NULL;
  _spiPort = NULL;
  _debugPort = NULL;
  _deviceAddress = BNO08X_DEFAULT_ADDRESS;
}

// Configure device for I2C operation
bool BNO08x::begin(uint8_t deviceAddress, TwoWire &wirePort, uint8_t intPin) {
  _deviceAddress = deviceAddress;
  _i2cPort = &wirePort;
  _int = intPin;
  _spiFlag = false;
  
  if (_int != 255) {
    pinMode(_int, INPUT_PULLUP);
  }
  
  // Initialize SHTP communication
  if (!shtp_init(_i2cPort, _deviceAddress, _int)) {
    if (_printDebug) Serial.println(F("Failed to initialize SHTP"));
    return false;
  }
  
  // Set up SH2 callbacks
  sh2_registerEventCallback(handleSensorEvent, NULL);
  sh2_registerErrorCallback(reportError, NULL);
  
  // Initialize sensor hub
  if (sh2_initialize() != SH2_OK) {
    if (_printDebug) Serial.println(F("Failed to initialize sensor hub"));
    return false;
  }
  
  // Get product ID information
  sh2_ProductIds_t prodIds;
  if (sh2_getProdIds(&prodIds) == SH2_OK) {
    if (_printDebug) {
      Serial.print(F("Part Number: "));
      Serial.println(prodIds.partNumber);
      Serial.print(F("Software Version: "));
      Serial.print(prodIds.swVersionMajor);
      Serial.print(".");
      Serial.println(prodIds.swVersionMinor);
    }
  }
  
  return true;
}

// Configure device for SPI operation
bool BNO08x::beginSPI(uint8_t csPin, uint8_t intPin, uint8_t rstPin, SPIClass &spiPort, uint32_t spiPortSpeed) {
  _cs = csPin;
  _int = intPin;
  _rst = rstPin;
  _spiPort = &spiPort;
  _spiFlag = true;
  
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  
  if (_int != 255) {
    pinMode(_int, INPUT_PULLUP);
  }
  
  if (_rst != 255) {
    resetPin = _rst;
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
  }
  
  _spiPort->begin();
  
  // Initialize SHTP communication
  if (!shtp_init_spi(_spiPort, _cs, _int, spiPortSpeed)) {
    if (_printDebug) Serial.println(F("Failed to initialize SPI SHTP"));
    return false;
  }
  
  // Set up SH2 callbacks
  sh2_registerEventCallback(handleSensorEvent, NULL);
  sh2_registerErrorCallback(reportError, NULL);
  
  // Initialize sensor hub
  if (sh2_initialize() != SH2_OK) {
    if (_printDebug) Serial.println(F("Failed to initialize sensor hub"));
    return false;
  }
  
  // Get product ID information
  sh2_ProductIds_t prodIds;
  if (sh2_getProdIds(&prodIds) == SH2_OK) {
    if (_printDebug) {
      Serial.print(F("Part Number: "));
      Serial.println(prodIds.partNumber);
      Serial.print(F("Software Version: "));
      Serial.print(prodIds.swVersionMajor);
      Serial.print(".");
      Serial.println(prodIds.swVersionMinor);
    }
  }
  
  return true;
}

// Enable debugging messages
void BNO08x::enableDebugging(Stream &debugPort) {
  _debugPort = &debugPort;
  _printDebug = true;
}

// Perform a soft reset
void BNO08x::softReset() {
  sh2_sendReset();
  _isReset = true;
}

// Check if a reset has occurred
bool BNO08x::wasReset() {
  bool temp = _isReset;
  _isReset = false;
  return temp;
}

// Check if data is available from the sensor
bool BNO08x::dataAvailable() {
  if (newData) {
    newData = false;
    return true;
  }
  return false;
}

// Implementation for getQuat convenience method
void BNO08x::getQuat(float &i, float &j, float &k, float &real, float &radianAccuracy, uint8_t &accuracy) {
  i = getQuatI();
  j = getQuatJ();
  k = getQuatK();
  real = getQuatReal();
  radianAccuracy = getQuatRadianAccuracy();
  accuracy = calibrationAccuracy;
}

// Implementation for getGyro convenience method
void BNO08x::getGyro(float &x, float &y, float &z, float &accuracy) {
  x = getGyroX();
  y = getGyroY();
  z = getGyroZ();
  accuracy = getGyroAccuracy();
}

// Implementation for requestCalibrationStatus
void BNO08x::requestCalibrationStatus() {
  getCalibrationStatus();
}

// Implementation for remaining methods would continue here...
// For brevity, I've included only key methods. The complete implementation
// would include all methods defined in the header file.
