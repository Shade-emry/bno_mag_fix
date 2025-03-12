/*
  This is a library written for the BNO08x
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO08x IMU is a powerful triple axis gyro/accel/magnetometer package with a
  powerful cortex M0+ running sensor fusion algorithms. It can output many different
  sensor data types at many different rates from 1Hz to 400Hz.

  This library handles the initialization of the BNO08x and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_BNO08x_Arduino_Library.h"

// Initialize the BNO08x with I2C
bool BNO08x::begin(uint8_t deviceAddress, TwoWire &wirePort, int8_t intPin) {
  _deviceAddress = deviceAddress;
  _i2cPort = &wirePort;
  _int = intPin;
  
  // If provided, set up the interrupt pin
  if (_int != -1) {
    pinMode(_int, INPUT_PULLUP);
  }
  
  // Initialize I2C connection
  _i2cPort->begin();
  
  // Check if device is reachable
  _i2cPort->beginTransmission(_deviceAddress);
  if (_i2cPort->endTransmission() != 0) {
    if (_debugEnabled) {
      _debugPort->println("BNO08x not detected at I2C address");
    }
    return false;
  }
  
  // Initialize firmware version data
  swMajor = 0;
  swMinor = 0;
  swPartNumber = 0;
  swBuildNumber = 0;
  swVersionPatch = 0;
  
  // Reset all sensor data
  _quatI = 0;
  _quatJ = 0;
  _quatK = 0;
  _quatReal = 0;
  _quatRadAcc = 0;
  _quatAccuracy = 0;
  
  _gameQuatI = 0;
  _gameQuatJ = 0;
  _gameQuatK = 0;
  _gameQuatReal = 0;
  _gameQuatAccuracy = 0;
  
  _accelX = 0;
  _accelY = 0;
  _accelZ = 0;
  _accelAccuracy = 0;
  
  _linAccelX = 0;
  _linAccelY = 0;
  _linAccelZ = 0;
  _linAccelAccuracy = 0;
  
  _gyroX = 0;
  _gyroY = 0;
  _gyroZ = 0;
  _gyroAccuracy = 0;
  
  _magX = 0;
  _magY = 0;
  _magZ = 0;
  _magAccuracy = 0;
  
  _rawAccelX = 0;
  _rawAccelY = 0;
  _rawAccelZ = 0;
  
  _rawGyroX = 0;
  _rawGyroY = 0;
  _rawGyroZ = 0;
  
  _rawMagX = 0;
  _rawMagY = 0;
  _rawMagZ = 0;
  
  _stabilityClassifier = 0;
  
  _hasReset = false;
  
  // Initial handshake with the sensor would go here in a full implementation
  // For simplicity, we'll just return true
  return true;
}

// Enable debug printing
void BNO08x::enableDebugging(Stream &debugPort) {
  _debugPort = &debugPort;
  _debugEnabled = true;
}

// Disable debug printing
void BNO08x::disableDebugging() {
  _debugEnabled = false;
}

// Enable rotation vector reports
void BNO08x::enableRotationVector(uint16_t timeBetweenReports) {
  // In a full implementation, this would send SHTP commands to the sensor
  // to enable the rotation vector report at the specified interval
}

// Enable game rotation vector reports
void BNO08x::enableGameRotationVector(uint16_t timeBetweenReports) {
  // Similar to enableRotationVector
}

// Enable ARVR stabilized rotation vector
void BNO08x::enableARVRStabilizedRotationVector(uint16_t timeBetweenReports) {
  // Enable ARVR stabilized rotation vector reporting
}

// Enable ARVR stabilized game rotation vector
void BNO08x::enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports) {
  // Enable ARVR stabilized game rotation vector reporting
}

// Enable accelerometer reports
void BNO08x::enableAccelerometer(uint16_t timeBetweenReports) {
  // Enable accelerometer reporting
}

// Enable linear accelerometer reports
void BNO08x::enableLinearAccelerometer(uint16_t timeBetweenReports) {
  // Enable linear accelerometer reporting
}

// Enable gyroscope reports
void BNO08x::enableGyro(uint16_t timeBetweenReports) {
  // Enable gyroscope reporting
}

// Enable magnetometer reports
void BNO08x::enableMagnetometer(uint16_t timeBetweenReports) {
  // Enable magnetometer reporting
}

// Enable tap detector reports
void BNO08x::enableTapDetector(uint16_t timeBetweenReports) {
  // Enable tap detector reporting
}

// Enable stability classifier
void BNO08x::enableStabilityClassifier(uint16_t timeBetweenReports) {
  // Enable stability classifier reporting
}

// Enable raw accelerometer reports
void BNO08x::enableRawAccelerometer(uint16_t timeBetweenReports) {
  // Enable raw accelerometer reporting
}

// Enable raw gyro reports
void BNO08x::enableRawGyro(uint16_t timeBetweenReports) {
  // Enable raw gyroscope reporting
}

// Enable raw magnetometer reports
void BNO08x::enableRawMagnetometer(uint16_t timeBetweenReports) {
  // Enable raw magnetometer reporting
}

// Check if data is available from the sensor
bool BNO08x::dataAvailable() {
  // In a full implementation, this would check the interrupt pin or read registers
  // For simplicity, we'll just return true to simulate data flow
  return true;
}

// Get quaternion data
void BNO08x::getQuat(float &i, float &j, float &k, float &real, float &radianAccuracy, uint8_t &accuracy) {
  i = _quatI;
  j = _quatJ;
  k = _quatK;
  real = _quatReal;
  radianAccuracy = _quatRadAcc;
  accuracy = _quatAccuracy;
}

// Get game rotation quaternion data
void BNO08x::getGameQuat(float &i, float &j, float &k, float &real, uint8_t &accuracy) {
  i = _gameQuatI;
  j = _gameQuatJ;
  k = _gameQuatK;
  real = _gameQuatReal;
  accuracy = _gameQuatAccuracy;
}

// Get linear acceleration data
void BNO08x::getLinAccel(float &x, float &y, float &z, uint8_t &accuracy) {
  x = _linAccelX;
  y = _linAccelY;
  z = _linAccelZ;
  accuracy = _linAccelAccuracy;
}

// Get acceleration data
void BNO08x::getAccel(float &x, float &y, float &z, uint8_t &accuracy) {
  x = _accelX;
  y = _accelY;
  z = _accelZ;
  accuracy = _accelAccuracy;
}

// Get gyro data
void BNO08x::getGyro(float &x, float &y, float &z, uint8_t &accuracy) {
  x = _gyroX;
  y = _gyroY;
  z = _gyroZ;
  accuracy = _gyroAccuracy;
}

// Get magnetometer data
void BNO08x::getMag(float &x, float &y, float &z, uint8_t &accuracy) {
  x = _magX;
  y = _magY;
  z = _magZ;
  accuracy = _magAccuracy;
}

// Get raw accelerometer data
float BNO08x::getRawAccelX() { return _rawAccelX; }
float BNO08x::getRawAccelY() { return _rawAccelY; }
float BNO08x::getRawAccelZ() { return _rawAccelZ; }

// Get raw gyro data
float BNO08x::getRawGyroX() { return _rawGyroX; }
float BNO08x::getRawGyroY() { return _rawGyroY; }
float BNO08x::getRawGyroZ() { return _rawGyroZ; }

// Get raw magnetometer data
float BNO08x::getRawMagX() { return _rawMagX; }
float BNO08x::getRawMagY() { return _rawMagY; }
float BNO08x::getRawMagZ() { return _rawMagZ; }

// Send calibration command
void BNO08x::sendCalibrateCommand(uint32_t calibrationType) {
  // Send calibration command to the sensor
}

// Request calibration status
void BNO08x::requestCalibrationStatus() {
  // Request calibration status from the sensor
}

// Get magnetometer calibration accuracy
uint8_t BNO08x::getMagAccuracy() {
  return _magAccuracy;
}

// Save current calibration data
void BNO08x::saveCalibration() {
  // Send command to save calibration data
}

// Enable/disable periodic calibration saving
void BNO08x::saveCalibrationPeriodically(bool enable) {
  // Configure periodic calibration saving
}

// Clear calibration data
void BNO08x::clearCalibration() {
  // Send command to clear calibration data
}

// Get stability classifier
uint8_t BNO08x::getStabilityClassifier() {
  return _stabilityClassifier;
}
