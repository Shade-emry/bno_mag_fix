/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#ifndef SENSORS_BNO080SENSOR_H
#define SENSORS_BNO080SENSOR_H

#include <Arduino.h>
#include <SPI.h>
#include "sensor.h"
#include "mahony.h"
#include "magneto1.4.h"
#include "../configuration/Configuration.h"
#include "../globals.h"
#include "../debug.h"
#include "../magnetic_calibration.h"

#include "bno080.h"

#define FLAG_SENSOR_BNO0XX_MAG_ENABLED 1

// Temperature coefficient for gyro drift (rad/s per °C)
static constexpr float TEMP_COEFF = 0.001f;

// Motion state definitions
enum MotionState {
    MOTION_STATIONARY = 0,
    MOTION_SLOW = 1,
    MOTION_MODERATE = 2,
    MOTION_FAST = 3
};

// History buffer sizes
static constexpr uint8_t GYRO_HISTORY_SIZE = 10;
static constexpr uint8_t MAG_HISTORY_SIZE = 20;
static constexpr uint8_t GRADIENT_HISTORY_SIZE = 10;

class BNO080Sensor : public Sensor {
public:
	static constexpr auto TypeID = SensorTypeID::BNO080;
	static constexpr float DEG_TO_RAD = 0.017453292F;
	static constexpr float RAD_TO_DEG = 57.29577951F;

	BNO080Sensor(uint8_t id, uint8_t address, uint8_t intPin, float rotation, uint8_t sclPin, uint8_t sdaPin);
	~BNO080Sensor();
	void motionSetup() override final;
	void motionLoop() override final;
	void startCalibration(int calibrationType) override final;
	void resetFusion();
	SensorStatus getSensorState();
	bool isMagEnabled() const;
	void setMagEnabled(bool enabled);

private:
	BNO080 imu;
	uint8_t addr;
	uint8_t intPin;
	uint8_t sclPin;
	uint8_t sdaPin;
	uint32_t lastData = 0;
	uint32_t lastReset = 0;

	// Magnetometer calibration
	uint8_t magCalibrationAccuracy = 0;
	uint8_t magneticAccuracyEstimate = 0;
	uint8_t calibrationAccuracy = 0;

	// Magnetometer calibration
	MagnetoCalibration m_MagCal;
	MFX_input_t m_MagCalInput;
	MFX_output_t m_MagCalOutput;

	// Calibration state
	bool m_bMagCalibrationEnabled = false;
	bool m_bMagCalibrationValid = false;
	bool m_bMagCalibrationSaveRequested = false;
	bool m_bMagCalibrationInProgress = false;
	bool m_bMagCalibrationStarted = false;
	uint32_t m_nMagCalibrationStartTime = 0;
	uint32_t m_nMagCalibrationLastSampleTime = 0;
	uint32_t m_nMagCalibrationSampleCount = 0;
	uint32_t m_nMagCalibrationLastStatusUpdateTime = 0;
	uint32_t m_nMagCalibrationLastSaveTime = 0;
	uint32_t m_nMagCalibrationLastLoadTime = 0;
	uint32_t m_nMagCalibrationLastResetTime = 0;
	uint32_t m_nMagCalibrationLastValidTime = 0;
	uint32_t m_nMagCalibrationLastInvalidTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceUpdateTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceValidTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceInvalidTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetRequestTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetDoneTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetFailTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetSuccessTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetSuccessCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetFailCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetRequestCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetDoneCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptSuccessCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptFailCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptRequestCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptDoneCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptSuccessRate = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptFailRate = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptRequestRate = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptDoneRate = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptSuccessRateCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptFailRateCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptRequestRateCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptDoneRateCount = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptSuccessRateTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptFailRateTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptRequestRateTime = 0;
	uint32_t m_nMagCalibrationLastCovarianceResetAttemptDoneRateTime = 0;

	// Calibration flags
	uint8_t m_nMagCalibrationFlags = 0;
	uint8_t m_nMagCalibrationStatus = 0;
	uint8_t m_nMagCalibrationResult = 0;
	uint8_t m_nMagCalibrationAccuracy = 0;
	uint8_t m_nMagCalibrationAccuracyStatus = 0;
	uint8_t m_nMagCalibrationAccuracyResult = 0;
	uint8_t m_nMagCalibrationAccuracyFlags = 0;
	uint8_t m_nMagCalibrationAccuracyLevel = 0;
	uint8_t m_nMagCalibrationAccuracyLevelStatus = 0;
	uint8_t m_nMagCalibrationAccuracyLevelResult = 0;
	uint8_t m_nMagCalibrationAccuracyLevelFlags = 0;
	uint8_t m_nMagCalibrationAccuracyLevelValue = 0;
	uint8_t m_nMagCalibrationAccuracyLevelValueStatus = 0;
	uint8_t m_nMagCalibrationAccuracyLevelValueResult = 0;
	uint8_t m_nMagCalibrationAccuracyLevelValueFlags = 0;
	uint8_t m_nMagCalibrationAccuracyLevelValueValue = 0;
	uint8_t m_nMagCalibrationAccuracyLevelValueValueStatus = 0;
	uint8_t m_nMagCalibrationAccuracyLevelValueValueResult = 0;
	uint8_t m_nMagCalibrationAccuracyLevelValueValueFlags = 0;
	uint8_t m_nMagCalibrationAccuracyLevelValueValueValue = 0;

	// Calibration storage
	uint8_t m_nMagCalibrationStorage = 0;
	uint8_t m_nMagCalibrationStorageStatus = 0;
	uint8_t m_nMagCalibrationStorageResult = 0;
	uint8_t m_nMagCalibrationStorageFlags = 0;
	uint8_t m_nMagCalibrationStorageLevel = 0;
	uint8_t m_nMagCalibrationStorageLevelStatus = 0;
	uint8_t m_nMagCalibrationStorageLevelResult = 0;
	uint8_t m_nMagCalibrationStorageLevelFlags = 0;
	uint8_t m_nMagCalibrationStorageLevelValue = 0;
	uint8_t m_nMagCalibrationStorageLevelValueStatus = 0;
	uint8_t m_nMagCalibrationStorageLevelValueResult = 0;
	uint8_t m_nMagCalibrationStorageLevelValueFlags = 0;
	uint8_t m_nMagCalibrationStorageLevelValueValue = 0;
	uint8_t m_nMagCalibrationStorageLevelValueValueStatus = 0;
	uint8_t m_nMagCalibrationStorageLevelValueValueResult = 0;
	uint8_t m_nMagCalibrationStorageLevelValueValueFlags = 0;
	uint8_t m_nMagCalibrationStorageLevelValueValueValue = 0;

	// Magnetic disturbance tracking
	bool inDisturbance = false;
	int disturbanceLevel = 0;
	int lastDisturbanceLevel = 0;
	uint32_t disturbanceStartTime = 0;
	uint32_t lastDisturbanceTime = 0;
	uint32_t lastDisturbanceCheck = 0;
	
	// Activity level tracking
	float activityLevel = 0.0f;
	uint32_t lastActivityUpdate = 0;
	uint32_t stabilityStartTime = 0;
	bool isStable = false;
	
	// Drift compensation
	float headingDriftRate = 0.0f;        // Estimated heading drift rate in rad/s
	float driftCompensationGain = 0.5f;   // Gain for drift compensation (0.0-1.0)
	uint32_t lastDriftUpdate = 0;
	
	// Gyro heading tracking
	bool usingGyroHeading = false;
	float lastGyroHeading = 0.0f;
	uint32_t lastGyroTime = 0;
	uint32_t lastMagTime = 0;
	
	// Sensor fusion parameters
	float magTrustFactor = 0.8f;
	float gyroTrustFactor = 0.9f;
	float lastTemp = 25.0f;
	uint32_t lastParameterUpdate = 0;
	
	// Magnetic field analysis
	float magFieldHistory[MAG_HISTORY_SIZE] = {0};
	float magGradientHistory[GRADIENT_HISTORY_SIZE] = {0};
	float magFieldStability = 1.0f;
	float gyroMagnitudeHistory[GYRO_HISTORY_SIZE] = {0};

	// Motion state
	MotionState motionState = MOTION_STATIONARY;

	// Methods for sensor data processing
	void processIMUData();
	void processMagData();
	void processGyroData();
	void processMagneticData();
	
	// Methods for magnetic calibration
	void initMagCalibration();
	void updateMagCalibration();
	void updateMagneticCalibration();
	void updateMagneticCalibration(const MFX_input_t& magInput);
	void collectCalibrationPoint();
	void updateHardIronCompensation();
	void softIronCalibration();
	
	// Methods for adaptive calibration
	void updateActivityLevel();
	void updateAdaptiveParameters();
	void applyDriftCompensation(float& gyroZ, float deltaTime);
	float calculateHeadingConfidence();
	
	// Methods for disturbance detection
	void detectMagneticDisturbance();

	Logger m_Logger = Logger("BNO080");
};

#endif
