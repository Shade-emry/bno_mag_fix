/*
  This is a library written for the BNO08x
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO08x IMU is a powerful triple axis gyro/accel/magnetometer package with a
  powerful cortex M0+ running sensor fusion algorithms. It can output many different
  sensor data types at many different rates from 1Hz to 400Hz. The sensor
  package is quite powerful but the datasheet and integration manual are quite complex.

  This library simplifies the interface with the sensor to make it easier to use.

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

#ifndef _SPARKFUN_BNO08x_ARDUINO_LIBRARY_H
#define _SPARKFUN_BNO08x_ARDUINO_LIBRARY_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

//The default I2C address for the BNO08x on the SparkFun breakout is 0x4B
#define BNO08X_DEFAULT_ADDRESS 0x4B

//Command/Report IDs
#define CHANNEL_COMMAND 0
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
#define CHANNEL_WAKE_REPORTS 4
#define CHANNEL_GYRO 5

//All the ways we can configure or talk to the BNO08x
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_ARVR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Report IDs from page 59 of reference manual
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SENSOR_REPORTID_TIMESTAMP_REBASE 0xFB

//Calibration status defines
#define SH2_CAL_IN_PROGRESS 0
#define SH2_CAL_COMPLETE_TIMEOUT 1
#define SH2_CAL_COMPLETE_SUCCESS 2
#define SH2_CAL_COMPLETE_ERROR 3

//Calibration mode defines
#define SH2_CAL_GYRO 0x01
#define SH2_CAL_MAG 0x02
#define SH2_CAL_PLANAR 0x04
#define SH2_CAL_ON_TABLE 0x08
#define SH2_CAL_ACCEL 0x80
#define SH2_CAL_GYRO_IN_HAND 0x101

class BNO08x {
public:
  bool begin(uint8_t deviceAddress = BNO08X_DEFAULT_ADDRESS, TwoWire &wirePort = Wire, int8_t intPin = -1);
  
  // Sensor enable functions
  void enableRotationVector(uint16_t timeBetweenReports);
  void enableGameRotationVector(uint16_t timeBetweenReports);
  void enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
  void enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
  void enableAccelerometer(uint16_t timeBetweenReports);
  void enableLinearAccelerometer(uint16_t timeBetweenReports);
  void enableGyro(uint16_t timeBetweenReports);
  void enableMagnetometer(uint16_t timeBetweenReports);
  void enableTapDetector(uint16_t timeBetweenReports);
  void enableStabilityClassifier(uint16_t timeBetweenReports);
  void enableRawAccelerometer(uint16_t timeBetweenReports);
  void enableRawGyro(uint16_t timeBetweenReports);
  void enableRawMagnetometer(uint16_t timeBetweenReports);
  
  // Data retrieval functions
  bool dataAvailable();
  void getQuat(float &i, float &j, float &k, float &real, float &radianAccuracy, uint8_t &accuracy);
  void getGameQuat(float &i, float &j, float &k, float &real, uint8_t &accuracy);
  void getLinAccel(float &x, float &y, float &z, uint8_t &accuracy);
  void getAccel(float &x, float &y, float &z, uint8_t &accuracy);
  void getGyro(float &x, float &y, float &z, uint8_t &accuracy);
  void getMag(float &x, float &y, float &z, uint8_t &accuracy);
  float getRawAccelX();
  float getRawAccelY();
  float getRawAccelZ();
  float getRawGyroX();
  float getRawGyroY();
  float getRawGyroZ();
  float getRawMagX();
  float getRawMagY();
  float getRawMagZ();
  
  // Calibration functions
  void sendCalibrateCommand(uint32_t calibrationType);
  void requestCalibrationStatus();
  uint8_t getMagAccuracy();
  void saveCalibration();
  void saveCalibrationPeriodically(bool enable);
  void clearCalibration();
  uint8_t getStabilityClassifier();

  // Debugging
  void enableDebugging(Stream &debugPort);
  void disableDebugging();
  
  // Firmware version information
  uint8_t swMajor, swMinor, swPartNumber, swBuildNumber, swVersionPatch;

private:
  TwoWire *_i2cPort;
  Stream *_debugPort;
  uint8_t _deviceAddress;
  int8_t _int;
  
  // Sensor data
  float _quatI, _quatJ, _quatK, _quatReal, _quatRadAcc;
  float _gameQuatI, _gameQuatJ, _gameQuatK, _gameQuatReal;
  float _linAccelX, _linAccelY, _linAccelZ;
  float _accelX, _accelY, _accelZ;
  float _gyroX, _gyroY, _gyroZ;
  float _magX, _magY, _magZ;
  float _rawAccelX, _rawAccelY, _rawAccelZ;
  float _rawGyroX, _rawGyroY, _rawGyroZ;
  float _rawMagX, _rawMagY, _rawMagZ;
  
  uint8_t _quatAccuracy;
  uint8_t _gameQuatAccuracy;
  uint8_t _accelAccuracy;
  uint8_t _linAccelAccuracy;
  uint8_t _gyroAccuracy;
  uint8_t _magAccuracy;
  uint8_t _stabilityClassifier;
  
  bool _hasReset;
  bool _debugEnabled;
};

#endif
