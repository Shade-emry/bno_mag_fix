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

#include "sensors/bno080sensor.h"

#include "GlobalVars.h"
#include "utils.h"

// Function to calculate the norm of a vector
int32_t norm(const int32_t* vector, int size) {
    int32_t sum = 0;
    for (int i = 0; i < size; i++) {
        sum += FX_MUL(vector[i], vector[i]);
    }
    return (int32_t)sqrtf(FX_TO_F(sum));
}

void BNO080Sensor::motionSetup() {
#ifdef DEBUG_SENSOR
	imu.enableDebugging(Serial);
#endif
	if (!imu.begin(addr, Wire, m_IntPin)) {
		m_Logger.fatal(
			"Can't connect to %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	m_Logger.info(
		"Connected to %s on 0x%02x. "
		"Info: SW Version Major: 0x%02x "
		"SW Version Minor: 0x%02x "
		"SW Part Number: 0x%02x "
		"SW Build Number: 0x%02x "
		"SW Version Patch: 0x%02x",
		getIMUNameByType(sensorType),
		addr,
		imu.swMajor,
		imu.swMinor,
		imu.swPartNumber,
		imu.swBuildNumber,
		imu.swVersionPatch
	);

	SlimeVR::Configuration::SensorConfig sensorConfig
		= configuration.getSensor(sensorId);
	
	// Always enable magnetometer for BNO085
	if (sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086) {
		magStatus = MagnetometerStatus::MAG_ENABLED;
		m_Config.magEnabled = true;
	} else {
		// For other sensors, use config or default
		switch (sensorConfig.type) {
			case SlimeVR::Configuration::SensorConfigType::BNO0XX:
				m_Config = sensorConfig.data.bno0XX;
				magStatus = m_Config.magEnabled ? MagnetometerStatus::MAG_ENABLED
											: MagnetometerStatus::MAG_DISABLED;
				break;
			default:
				magStatus = USE_6_AXIS ? MagnetometerStatus::MAG_DISABLED
									: MagnetometerStatus::MAG_ENABLED;
				break;
		}
	}

	// Enable magnetometer first
	if (isMagEnabled()) {
		// Enable magnetometer at higher rate for better calibration
		imu.enableMagnetometer(50);  // 50Hz updates
		
		// Wait a bit for mag to initialize
		delay(100);
		
		// Configure rotation vector to use magnetometer
		if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
			&& BNO_USE_ARVR_STABILIZATION) {
			imu.enableARVRStabilizedRotationVector(10);
		} else {
			imu.enableRotationVector(10);
		}
		
		// Request initial calibration status
		imu.requestCalibrationStatus();
		
		// Enable periodic calibration saves
		imu.saveCalibrationPeriodically(true);
		
		// Send calibration command specifically for magnetometer
		imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);
		
		// Start initial calibration
		m_Logger.info("=== Initial Magnetometer Calibration ===");
        m_Logger.info("For best results:");
        m_Logger.info("1. Keep sensor away from magnetic interference");
        m_Logger.info("2. Move sensor in smooth figure-8 patterns");
        m_Logger.info("3. Rotate through all orientations slowly");
        m_Logger.info("4. Watch for calibration level updates");
        m_Logger.info("Calibration will be saved automatically when complete");
		
		// Small delay to let settings take effect
		delay(100);
	} else {
		if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
			&& BNO_USE_ARVR_STABILIZATION) {
			imu.enableARVRStabilizedGameRotationVector(10);
		} else {
			imu.enableGameRotationVector(10);
		}
	}

	imu.enableLinearAccelerometer(10);
	initMagneticCalibration();

#if ENABLE_INSPECTION
	imu.enableRawGyro(10);
	imu.enableRawAccelerometer(10);
	if (isMagEnabled()) {
		imu.enableRawMagnetometer(10);
	}
#endif
	// Calibration settings:
	// EXPERIMENTAL Enable periodic calibration save to permanent memory
	imu.saveCalibrationPeriodically(true);
	imu.requestCalibrationStatus();
	// EXPERIMENTAL Disable accelerometer calibration after 1 minute to prevent
	// "stomping" bug WARNING : Executing IMU commands outside of the update loop is not
	// allowed since the address might have changed when the timer is executed!
	if (sensorType == SensorTypeID::BNO085) {
		// For BNO085, disable accel calibration
		globalTimer.in(
			60000,
			[](void* sensor) {
				((BNO080*)sensor)->sendCalibrateCommand(SH2_CAL_MAG | SH2_CAL_ON_TABLE);
				return true;
			},
			&imu
		);
	} else if (sensorType == SensorTypeID::BNO086) {
		// For BNO086, disable accel calibration
		// TODO: Find default flags for BNO086
		globalTimer.in(
			60000,
			[](void* sensor) {
				((BNO080*)sensor)->sendCalibrateCommand(SH2_CAL_MAG | SH2_CAL_ON_TABLE);
				return true;
			},
			&imu
		);
	} else {
		globalTimer.in(
			60000,
			[](void* sensor) {
				((BNO080*)sensor)->requestCalibrationStatus();
				return true;
			},
			&imu
		);
	}
	// imu.sendCalibrateCommand(SH2_CAL_ACCEL | SH2_CAL_GYRO_IN_HAND | SH2_CAL_MAG |
	// SH2_CAL_ON_TABLE | SH2_CAL_PLANAR);

	imu.enableStabilityClassifier(500);

	lastReset = 0;
	lastData = millis();
	working = true;
	configured = true;
	m_tpsCounter.reset();
	m_dataCounter.reset();
}

void BNO080Sensor::motionLoop() {
    // Check if IMU data is available
    if (imu.dataAvailable()) {
        lastData = millis();
        
        Quat nRotation;  // Local quaternion variable
        
        // Process IMU data (quaternion)
        if (isMagEnabled()) {
            static uint32_t lastMagStatusCheck = 0;
            uint32_t currentTime = millis();
            
            // Check mag status every 5 seconds
            if (currentTime - lastMagStatusCheck >= 5000) {
                lastMagStatusCheck = currentTime;
                uint8_t newAccuracy = imu.getMagAccuracy();
                const char* statusText;
                switch(newAccuracy) {
                    case 0:
                        statusText = "Uncalibrated";
                        break;
                    case 1:
                        statusText = "Minimal Calibration";
                        break;
                    case 2:
                        statusText = "More Calibrated";
                        break;
                    case 3:
                        statusText = "Fully Calibrated";
                        // Save calibration when we reach full calibration
                        if (newAccuracy > magCalibrationAccuracy) {
                            m_Logger.info("Reached full calibration - saving calibration data");
                            imu.saveCalibration();
                            delay(100); // Give it time to save
                        }
                        break;
                    default:
                        statusText = "Unknown";
                }
                
                // Only log if accuracy changed
                if (newAccuracy != magCalibrationAccuracy) {
                    magCalibrationAccuracy = newAccuracy;
                    m_Logger.info("Magnetometer Calibration Status: %s (Level %d/3)", statusText, magCalibrationAccuracy);
                } else {
                    m_Logger.info("Magnetometer Status Check - Current Status: %s (Level %d/3)", statusText, magCalibrationAccuracy);
                }
                
                // If accuracy drops below 2, start recalibration
                if (magCalibrationAccuracy < 2) {
                    imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);
                }
            }
            
            if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
                && BNO_USE_ARVR_STABILIZATION) {
                imu.getQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, magneticAccuracyEstimate, calibrationAccuracy);
            } else {
                imu.getQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, magneticAccuracyEstimate, calibrationAccuracy);
            }
            
            networkConnection.sendRotationData(sensorId, &nRotation, DATA_TYPE_NORMAL, calibrationAccuracy);
            
        } else {
            if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
                && BNO_USE_ARVR_STABILIZATION) {
                imu.getGameQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, calibrationAccuracy);
            } else {
                imu.getGameQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, calibrationAccuracy);
            }
            
            networkConnection.sendRotationData(sensorId, &nRotation, DATA_TYPE_NORMAL, calibrationAccuracy);
        }
        
        // Get linear acceleration data
        uint8_t acc;
        Vector3 nAccel;
        imu.getLinAccel(nAccel.x, nAccel.y, nAccel.z, acc);
        networkConnection.sendSensorAcceleration(sensorId, nAccel);
        
        // Process magnetometer data if available
        if (imu.getMagnetometerXYZ()) {
            processMagData();
            
            // Update magnetic calibration if enabled
            if (isMagEnabled()) {
                MFX_input_t magInput;
                magInput.mag[0] = F_TO_FX(imu.getRawMagX());
                magInput.mag[1] = F_TO_FX(imu.getRawMagY());
                magInput.mag[2] = F_TO_FX(imu.getRawMagZ());
                magInput.timestamp = millis();
                updateMagneticCalibration(magInput);
            }
        }
        
        // Process gyroscope data if available
        if (imu.getGyroXYZ()) {
            // Update activity level based on gyro data
            updateActivityLevel();
            
            // Process gyro data for heading tracking
            processGyroData();
        }
        
        // Detect magnetic disturbances
        detectMagneticDisturbance();
        
        // Update adaptive parameters
        updateAdaptiveParameters();
        
        // Check stability classifier
        if (imu.getStabilityClassifier() == 1) {
            markRestCalibrationComplete();
        }
        
    } else {
        // Check for IMU timeout
        if (millis() - lastData > 1000) {
            m_Logger.warn("No data from BNO080 for 1 second!");
            
            // Try to reset if no data for 5 seconds
            if (millis() - lastReset > 5000) {
                m_Logger.warn("Resetting BNO080...");
                resetFusion();
                lastReset = millis();
            }
        }
    }
    
    // Update magnetic calibration
    updateMagneticCalibration();
}

void BNO080Sensor::processIMUData() {
    // This method is for future enhancements
    // Currently, the quaternion data is processed directly in motionLoop
    
    // We can add additional processing here in the future, such as:
    // - Additional filtering
    // - Sensor fusion optimizations
    // - Drift correction
}

SensorStatus BNO080Sensor::getSensorState() {
    return lastReset > 0 ? SensorStatus::SENSOR_ERROR
         : isWorking()   ? SensorStatus::SENSOR_OK
                         : SensorStatus::SENSOR_OFFLINE;
}

void BNO080Sensor::sendData() {
    if (!m_fusion.isUpdated()) {
        return;
    }

    Quat quaternion = m_fusion.getQuaternionQuat();
    Vector3 acceleration = m_fusion.getLinearAccVec();

    networkConnection.sendRotationData(
        sensorId,
        &quaternion,
        DATA_TYPE_NORMAL,
        calibrationAccuracy
    );

    networkConnection.sendSensorAcceleration(sensorId, acceleration);

    m_fusion.clearUpdated();
}

void BNO080Sensor::setFlag(uint16_t flagId, bool state) {
    if (flagId == FLAG_SENSOR_BNO0XX_MAG_ENABLED) {
        m_Config.magEnabled = state;
        magStatus = state ? MagnetometerStatus::MAG_ENABLED
                          : MagnetometerStatus::MAG_DISABLED;

        SlimeVR::Configuration::SensorConfig config;
        config.type = SlimeVR::Configuration::SensorConfigType::BNO0XX;
        config.data.bno0XX = m_Config;
        configuration.setSensor(sensorId, config);

        // Reinitialize the sensor
        motionSetup();
    }
}

void BNO080Sensor::startCalibration(int calibrationType) {
    if (calibrationType == 2) {  // Magnetometer calibration type
        m_Logger.info("Starting magnetometer calibration sequence...");
        m_Logger.info("Current calibration level: %d/3", magCalibrationAccuracy);
        m_Logger.info("=== Calibration Instructions ===");
        m_Logger.info("1. Hold the sensor at least 0.5m away from any large metal objects");
        m_Logger.info("2. Perform the following movements slowly and smoothly:");
        m_Logger.info("   a) Draw figure-8 patterns in different orientations");
        m_Logger.info("   b) Rotate the sensor 360° around each axis");
        m_Logger.info("   c) Keep movements slow - about 3 seconds per rotation");
        m_Logger.info("3. Watch the calibration level:");
        m_Logger.info("   Level 0: Uncalibrated - Keep moving");
        m_Logger.info("   Level 1: Basic calibration - Continue movement");
        m_Logger.info("   Level 2: Good calibration - Fine-tune movements");
        m_Logger.info("   Level 3: Best calibration - Calibration complete");
        m_Logger.info("4. After reaching Level 3:");
        m_Logger.info("   - Keep position stable for 2-3 seconds");
        m_Logger.info("   - Calibration will be automatically saved");
        m_Logger.info("5. If accuracy drops:");
        m_Logger.info("   - Move away from magnetic interference");
        m_Logger.info("   - Repeat calibration if necessary");
        
        // Enable high-rate magnetometer updates during calibration
        imu.enableMagnetometer(50);  // 50Hz updates
        
        // Force recalibration
        imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);
        
        // Request calibration status
        imu.requestCalibrationStatus();
        
        // Small delay to let the commands process
        delay(50);
    }
}

void BNO080Sensor::initMagneticCalibration() {
    if (isMagEnabled()) {
        // Request current calibration status
        imu.requestCalibrationStatus();
        
        // Enable magnetic field reports
        imu.enableMagnetometer(50);  // 50Hz for better initial calibration
        
        // Initialize calibration structures
        memset(&m_MagCalInput, 0, sizeof(m_MagCalInput));
        memset(&m_MagCalOutput, 0, sizeof(m_MagCalOutput));
        m_MagCalOutput.quality = MFX_MAGCAL_UNKNOWN;
        
        // Initialize soft iron matrix to identity
        init_identity_matrix(m_MagCalOutput.si_matrix);
        m_MagCalOutput.si_valid = false;
        m_MagCalOutput.confidence = 0.0f;
        m_MagCalOutput.calibration_points = 0;
        
        // Start calibration
        imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);
    }
}

void BNO080Sensor::updateMagneticCalibration() {
    processMagneticData();
}

void BNO080Sensor::updateMagneticCalibration(const MFX_input_t& magInput) {
    m_MagCalInput = magInput;
    processMagneticData();
}

void BNO080Sensor::processMagneticData() {
    updateTemperatureCompensation();

    // Store current readings in history
    magHistory[historyIndex][0] = m_MagCalInput.mag[0];
    magHistory[historyIndex][1] = m_MagCalInput.mag[1];
    magHistory[historyIndex][2] = m_MagCalInput.mag[2];
    historyIndex = (historyIndex + 1) % 6;

    // Calculate variance and rate of change
    int32_t avgMag[3] = {0, 0, 0};
    int32_t variance = 0;
    int32_t maxRateOfChange = 0;
    
    // Calculate average and max rate of change
    for(int i = 0; i < 6; i++) {
        for(int axis = 0; axis < 3; axis++) {
            avgMag[axis] += magHistory[i][axis];
            
            // Calculate rate of change between consecutive samples
            if(i > 0) {
                int32_t rateOfChange = abs(magHistory[i][axis] - magHistory[i-1][axis]);
                if(rateOfChange > maxRateOfChange) {
                    maxRateOfChange = rateOfChange;
                }
            }
        }
    }
    
    for(int axis = 0; axis < 3; axis++) {
        avgMag[axis] /= 6;
        
        // Calculate variance contribution from this axis
        for(int i = 0; i < 6; i++) {
            int32_t diff = magHistory[i][axis] - avgMag[axis];
            variance += (diff * diff) >> 8;
        }
    }
    
    // Enhanced disturbance detection with multiple criteria
    static const int32_t VARIANCE_THRESHOLD = F_TO_FX(0.8);    // Much more sensitive to variations
    static const int32_t RATE_THRESHOLD = F_TO_FX(0.25);       // More sensitive to sudden changes
    static const int32_t RECOVERY_THRESHOLD = F_TO_FX(0.15);   // Very conservative recovery
    static const int32_t DECAY_RATE = F_TO_FX(0.995);         // Very slow decay
    static const int32_t RECOVERY_RATE = F_TO_FX(0.02);       // Very slow recovery
    static const int32_t STABLE_READING_THRESHOLD = F_TO_FX(0.05); // Threshold for stable readings
    static const uint32_t MIN_DISTURBANCE_TIME = 500;         // Minimum time in disturbance mode (ms)
    static const uint32_t MAX_DISTURBANCE_TIME = 10000;       // Maximum time in disturbance mode (ms)
    static const uint8_t STABLE_READINGS_REQUIRED = 5;        // Required consecutive stable readings
    
    // Add distance-based threshold scaling
    static const int32_t BASE_MAGNETIC_STRENGTH = F_TO_FX(40.0);  // Expected clean magnetic field strength
    
    // Calculate magnetic field strength using fixed point math
    int32_t fieldStrengthSquared = FX_MUL(m_MagCalInput.mag[0], m_MagCalInput.mag[0]) + 
                                   FX_MUL(m_MagCalInput.mag[1], m_MagCalInput.mag[1]) + 
                                   FX_MUL(m_MagCalInput.mag[2], m_MagCalInput.mag[2]);
    int32_t currentStrength = (int32_t)sqrtf(FX_TO_F(fieldStrengthSquared));
    int32_t strengthRatio = FX_DIV(currentStrength, BASE_MAGNETIC_STRENGTH);
    
    // Scale thresholds based on field strength (stronger field = more sensitive detection)
    int32_t scaledVarianceThreshold = strengthRatio > F_TO_FX(1.2) ? 
                                      FX_MUL(VARIANCE_THRESHOLD, F_TO_FX(0.5)) : 
                                      VARIANCE_THRESHOLD;
    
    // Determine disturbance level (0-3) based on severity
    uint8_t newDisturbanceLevel = 0;
    if (variance > scaledVarianceThreshold || maxRateOfChange > RATE_THRESHOLD) {
        newDisturbanceLevel = 1; // Mild disturbance
        
        if (strengthRatio > F_TO_FX(1.3)) {
            newDisturbanceLevel = 2; // Moderate disturbance
        }
        
        if (strengthRatio > F_TO_FX(1.5)) {
            newDisturbanceLevel = 3; // Severe disturbance
        }
    }
    
    bool isDisturbed = newDisturbanceLevel > 0;
    uint32_t currentTime = millis();
    
    if(isDisturbed) {
        if(!inDisturbance) {
            inDisturbance = true;
            disturbanceStartTime = currentTime;
            disturbanceLevel = newDisturbanceLevel;
            memcpy(lastValidMag, m_MagCalInput.mag, sizeof(lastValidMag));
            
            // Store last valid magnetic heading
            lastMagHeading = atan2(FX_TO_F(m_MagCalInput.mag[1]), FX_TO_F(m_MagCalInput.mag[0]));
            
            // Start using gyro for heading immediately
            usingGyroHeading = true;
            lastGyroHeading = lastMagHeading;
            lastGyroTime = currentTime;
            gyroHeadingDrift = 0.0f; // Reset drift estimate
        } else {
            // Update disturbance level if it increases
            if (newDisturbanceLevel > disturbanceLevel) {
                disturbanceLevel = newDisturbanceLevel;
                disturbanceStartTime = currentTime; // Reset timer for severe disturbances
            }
        }
        
        // During disturbance, blend between last valid and current readings
        // with different strategies based on disturbance level
        for(int i = 0; i < 3; i++) {
            int32_t blendFactor;
            
            switch(disturbanceLevel) {
                case 3: // Severe - almost completely ignore current readings
                    blendFactor = F_TO_FX(0.98);
                    break;
                case 2: // Moderate - heavily favor last valid readings
                    blendFactor = F_TO_FX(0.95);
                    break;
                default: // Mild - standard decay
                    blendFactor = DECAY_RATE;
            }
            
            m_MagCalInput.mag[i] = FX_MUL(lastValidMag[i], blendFactor) + 
                                   FX_MUL(avgMag[i], F_TO_FX(1.0) - blendFactor);
        }
        
        // Set quality based on disturbance level
        m_MagCalOutput.quality = (disturbanceLevel >= 3) ? MFX_MAGCAL_UNKNOWN : 
                                (disturbanceLevel >= 2) ? MFX_MAGCAL_POOR : MFX_MAGCAL_OK;
        
        // Reset recovery counter
        consecutiveStableReadings = 0;
        disturbanceRecoveryTime = 0;
        
    } else if(inDisturbance) {
        // Check if we're in recovery phase
        if (disturbanceRecoveryTime == 0 && 
            (currentTime - disturbanceStartTime) >= MIN_DISTURBANCE_TIME) {
            disturbanceRecoveryTime = currentTime;
        }
        
        // Check if readings are stable
        bool isStable = (variance < STABLE_READING_THRESHOLD);
        
        if (isStable) {
            consecutiveStableReadings++;
        } else {
            consecutiveStableReadings = 0;
        }
        
        // Gradual recovery with consistency check
        bool consistentReadings = (consecutiveStableReadings >= STABLE_READINGS_REQUIRED);
        
        // Additional check for field strength normalization during recovery
        bool fieldStrengthNormalized = (strengthRatio >= F_TO_FX(0.8) && 
                                       strengthRatio <= F_TO_FX(1.2));
        
        // Force exit from disturbance mode after maximum time
        bool forceExit = (currentTime - disturbanceStartTime) > MAX_DISTURBANCE_TIME;
        
        if (consistentReadings || forceExit) {
            // Calculate adaptive recovery rate based on disturbance level and duration
            int32_t adaptiveRecoveryRate;
            
            if (forceExit) {
                // Faster recovery when forcing exit
                adaptiveRecoveryRate = F_TO_FX(0.1);
            } else {
                // Base recovery rate on disturbance level (lower = faster recovery)
                float baseRate = 0.02f + (0.01f * (3 - disturbanceLevel));
                
                // Adjust for duration (longer = faster recovery)
                uint32_t disturbanceDuration = currentTime - disturbanceStartTime;
                float durationFactor = min(1.0f, disturbanceDuration / 5000.0f);
                
                adaptiveRecoveryRate = F_TO_FX(baseRate * (1.0f + durationFactor));
            }
            
            for(int i = 0; i < 3; i++) {
                lastValidMag[i] = FX_MUL(lastValidMag[i], F_TO_FX(1.0) - adaptiveRecoveryRate) + 
                                     FX_MUL(avgMag[i], adaptiveRecoveryRate);
                m_MagCalInput.mag[i] = lastValidMag[i];
            }
            
            // Exit disturbance mode if readings are very stable or we've been in disturbance too long
            if(forceExit || (consistentReadings && fieldStrengthNormalized)) {
                inDisturbance = false;
                disturbanceLevel = 0;
                
                // Transition from gyro to mag heading smoothly
                float currentMagHeading = atan2(FX_TO_F(m_MagCalInput.mag[1]), FX_TO_F(m_MagCalInput.mag[0]));
                float headingDiff = currentMagHeading - lastGyroHeading;
                
                // Normalize difference to -PI to PI
                while(headingDiff > PI) headingDiff -= TWO_PI;
                while(headingDiff < -PI) headingDiff += TWO_PI;
                
                // If difference is small, use gyro heading; otherwise, use mag heading
                if (abs(headingDiff) < 0.2f) {
                    // Blend between gyro and mag heading
                    lastGyroHeading = lastGyroHeading * 0.7f + currentMagHeading * 0.3f;
                } else {
                    // Difference is too large, trust mag heading
                    lastGyroHeading = currentMagHeading;
                }
                
                usingGyroHeading = false;
                m_MagCalOutput.quality = MFX_MAGCAL_OK;
                
                // When exiting disturbance, update calibration
                updateHardIronCompensation();
                
                // Calculate and store gyro drift for future reference
                if (currentTime > disturbanceStartTime + 1000) {
                    float disturbanceDuration = (currentTime - disturbanceStartTime) / 1000.0f;
                    gyroHeadingDrift = headingDiff / disturbanceDuration;
                }
            }
        }
    } else if(!inDisturbance) {
        // When not in disturbance, periodically check for calibration
        updateHardIronCompensation();
        
        // Store last valid magnetic heading
        lastMagHeading = atan2(FX_TO_F(m_MagCalInput.mag[1]), FX_TO_F(m_MagCalInput.mag[0]));
    }

    // Apply hard iron bias correction
    int32_t correctedMag[MFX_NUM_AXES];
    for(int i = 0; i < MFX_NUM_AXES; i++) {
        correctedMag[i] = m_MagCalInput.mag[i] - m_MagCalOutput.hi_bias[i];
    }

    // Apply soft iron compensation if valid
    if (m_MagCalOutput.si_valid) {
        int32_t siCorrectedMag[MFX_NUM_AXES];
        matrix_vector_multiply(m_MagCalOutput.si_matrix, correctedMag, siCorrectedMag);
        memcpy(correctedMag, siCorrectedMag, sizeof(correctedMag));
    }

    // Calculate magnitude
    int32_t magSquared = 0;
    for(int i = 0; i < MFX_NUM_AXES; i++) {
        magSquared += FX_MUL(correctedMag[i], correctedMag[i]);
    }
    
    int32_t magStrength = (int32_t)sqrtf(FX_TO_F(magSquared));
    
    // Enhanced field strength compensation
    static const int32_t MAG_COMPRESS = F_TO_FX(1.2);      // More aggressive compression
    static const int32_t MAG_MAX_FIELD = F_TO_FX(2.5);     // Higher max field tolerance
    static const int32_t COMPRESSION_FACTOR = F_TO_FX(0.8); // Stronger compression
    
    if (magStrength > MAG_COMPRESS && magStrength <= MAG_MAX_FIELD) {
        int32_t excess = magStrength - MAG_COMPRESS;
        int32_t compressionRatio = FX_MUL(excess, COMPRESSION_FACTOR);
        
        for(int i = 0; i < MFX_NUM_AXES; i++) {
            // More weight to last valid reading during high disturbance
            int32_t reduction = FX_MUL(correctedMag[i], compressionRatio);
            correctedMag[i] -= reduction;
        }
    }
    
    // Store corrected values
    for(int i = 0; i < MFX_NUM_AXES; i++) {
        m_MagCalInput.mag[i] = correctedMag[i];
    }

    // Check if we should collect calibration point
    collectCalibrationPoint();
    
    processGyroData();
}

void BNO080Sensor::updateHardIronCompensation() {
    static const uint16_t SAMPLES_FOR_CALIBRATION = 25; // Reduced samples for faster response
    static const int32_t LEARNING_RATE = F_TO_FX(0.05); // Increased to 5% learning rate
    
    // Only update when device is stable (low gyro readings)
    if (abs(imu.getGyroX()) < 1.0f && 
        abs(imu.getGyroY()) < 1.0f && 
        abs(imu.getGyroZ()) < 1.0f) {
        
        m_MagCalOutput.sample_count++;
        
        if (m_MagCalOutput.sample_count >= SAMPLES_FOR_CALIBRATION) {
            // Update hard iron bias with exponential moving average
            for (int i = 0; i < MFX_NUM_AXES; i++) {
                int32_t error = m_MagCalInput.mag[i] - m_MagCalOutput.hi_bias[i];
                m_MagCalOutput.hi_bias[i] += FX_MUL(error, LEARNING_RATE);
            }
            
            m_MagCalOutput.sample_count = 0;
            
            // Update calibration quality
            if (m_MagCalOutput.quality < MFX_MAGCAL_GOOD) {
                m_MagCalOutput.quality = MFX_MAGCAL_GOOD;
            }
        }
    }
}

void BNO080Sensor::processGyroData() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastGyroTime) / 1000.0f;
    lastGyroTime = currentTime;
    
    if(usingGyroHeading) {
        // Get raw gyro data
        float gyroZ = imu.getGyroZ();
        
        // Apply temperature compensation to gyro
        float temp = imu.getRawGyroX() * 0.01f;  // Temperature from gyro data
        float tempDiff = temp - 25.0f;  // Deviation from room temperature
        gyroZ *= (1.0f + TEMP_COEFF * tempDiff);  // Apply temperature correction
        
        // Apply enhanced drift compensation
        applyDriftCompensation(gyroZ, deltaTime);
        
        // Update heading with temperature-compensated and drift-corrected gyro
        lastGyroHeading += gyroZ * deltaTime;
        
        // Normalize to -PI to PI
        while(lastGyroHeading > PI) lastGyroHeading -= TWO_PI;
        while(lastGyroHeading < -PI) lastGyroHeading += TWO_PI;
        
        // If we have valid mag data, perform adaptive correction to prevent drift
        if(!inDisturbance && m_MagCalOutput.quality >= MFX_MAGCAL_OK) {
            float magHeading = atan2(FX_TO_F(m_MagCalInput.mag[1]), FX_TO_F(m_MagCalInput.mag[0]));
            float headingDiff = magHeading - lastGyroHeading;
            
            // Normalize difference to -PI to PI
            while(headingDiff > PI) headingDiff -= TWO_PI;
            while(headingDiff < -PI) headingDiff += TWO_PI;
            
            // Calculate confidence in the heading correction
            float confidence = calculateHeadingConfidence();
            
            // Adaptive correction factor based on activity level and confidence
            float correctionFactor = 0.001f; // Base correction rate (0.1% per update)
            
            // Adjust correction factor based on activity level
            if (activityLevel < 0.2f) {
                // More aggressive correction during low activity (up to 0.5% per update)
                correctionFactor = 0.005f * confidence;
            } else if (activityLevel < 0.5f) {
                // Moderate correction during medium activity (up to 0.3% per update)
                correctionFactor = 0.003f * confidence;
            } else {
                // Conservative correction during high activity (up to 0.1% per update)
                correctionFactor = 0.001f * confidence;
            }
            
            // Apply correction with adaptive rate
            lastGyroHeading += headingDiff * correctionFactor;
            
            // Update drift estimate with adaptive learning rate
            float driftLearningRate = 0.001f; // Base learning rate
            
            // Adjust learning rate based on activity and confidence
            if (activityLevel < 0.3f && confidence > 0.7f) {
                // More aggressive learning during stable periods with high confidence
                driftLearningRate = 0.003f;
            }
            
            // Update heading drift rate estimate
            headingDriftRate = headingDriftRate * (1.0f - driftLearningRate) + 
                              (headingDiff / max(deltaTime, 0.01f)) * driftLearningRate;
            
            // Limit drift rate to reasonable values
            headingDriftRate = constrain(headingDriftRate, -0.05f, 0.05f);
            
            // Log significant drift updates (for debugging)
            static uint32_t lastDriftLog = 0;
            if (abs(headingDriftRate) > 0.01f && (currentTime - lastDriftLog > 10000)) {
                lastDriftLog = currentTime;
                m_Logger.debug("Heading drift rate: %.4f rad/s (confidence: %.2f)", 
                              headingDriftRate, confidence);
            }
        }
    }
}

void BNO080Sensor::updateTemperatureCompensation() {
    // Get temperature from gyro data packet
    float currentTemp = imu.getRawGyroX() * 0.01f;
    float tempDiff = currentTemp - lastTemp;
    
    // Exponential moving average for temperature
    static const float TEMP_ALPHA = 0.1f;  // Slower temperature updates
    if(abs(tempDiff) > 0.5f) {  // Only update on significant changes
        lastTemp = lastTemp * (1.0f - TEMP_ALPHA) + currentTemp * TEMP_ALPHA;
        
        // Apply temperature compensation with dynamic coefficient
        float tempCoeff = TEMP_COEFF;
        if(abs(tempDiff) > 5.0f) {
            // Reduce compensation for large temperature changes
            tempCoeff *= 0.5f;
        }
        
        for(int i = 0; i < 3; i++) {
            m_MagCalInput.mag[i] = m_MagCalInput.mag[i] * (1.0f + tempCoeff * tempDiff);
        }
    }
}

void BNO080Sensor::updateActivityLevel() {
    // Get current gyro magnitudes
    float gyroX = imu.getGyroX();
    float gyroY = imu.getGyroY();
    float gyroZ = imu.getGyroZ();
    
    // Calculate total angular velocity magnitude
    float gyroMagnitude = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
    
    // Update gyro history buffer
    for (int i = GYRO_HISTORY_SIZE - 1; i > 0; i--) {
        gyroMagnitudeHistory[i] = gyroMagnitudeHistory[i-1];
    }
    gyroMagnitudeHistory[0] = gyroMagnitude;
    
    // Calculate average and variance of recent gyro magnitudes
    float sum = 0.0f;
    for (int i = 0; i < GYRO_HISTORY_SIZE; i++) {
        sum += gyroMagnitudeHistory[i];
    }
    float avgMagnitude = sum / GYRO_HISTORY_SIZE;
    
    float varianceSum = 0.0f;
    for (int i = 0; i < GYRO_HISTORY_SIZE; i++) {
        float diff = gyroMagnitudeHistory[i] - avgMagnitude;
        varianceSum += diff * diff;
    }
    float variance = varianceSum / GYRO_HISTORY_SIZE;
    
    // Calculate activity level based on average magnitude and variance
    // Map average magnitude from 0-2 rad/s to 0-1 activity level
    float magnitudeComponent = constrain(avgMagnitude / 2.0f, 0.0f, 1.0f);
    
    // Map variance from 0-0.5 (rad/s)² to 0-1 activity level
    float varianceComponent = constrain(variance / 0.5f, 0.0f, 1.0f);
    
    // Combine components with weights
    float newActivityLevel = 0.7f * magnitudeComponent + 0.3f * varianceComponent;
    
    // Smooth activity level changes with exponential moving average
    activityLevel = activityLevel * 0.8f + newActivityLevel * 0.2f;
    
    // Update motion state classification
    if (activityLevel < 0.1f) {
        motionState = MOTION_STATIONARY;
    } else if (activityLevel < 0.4f) {
        motionState = MOTION_SLOW;
    } else if (activityLevel < 0.7f) {
        motionState = MOTION_MODERATE;
    } else {
        motionState = MOTION_FAST;
    }
    
    // Log activity level changes (for debugging)
    static uint32_t lastActivityLog = 0;
    uint32_t currentTime = millis();
    if (currentTime - lastActivityLog > 5000) { // Log every 5 seconds
        lastActivityLog = currentTime;
        m_Logger.debug("Activity level: %.2f (state: %d)", activityLevel, motionState);
    }
}

void BNO080Sensor::collectCalibrationPoint() {
    static uint32_t lastCollectionTime = 0;
    static const uint32_t COLLECTION_INTERVAL = 500; // Collect points every 500ms
    static const int32_t MIN_DISTANCE = F_TO_FX(2.0f); // Minimum distance between points
    uint32_t now = millis();
    
    // Only collect points when not in disturbance and with good quality readings
    if (!inDisturbance && m_MagCalOutput.quality >= MFX_MAGCAL_OK) {
        // Check if we should collect a new point (time-based)
        if (now - lastCollectionTime > COLLECTION_INTERVAL) {
            // Check if we have space in the buffer
            if (m_MagCalOutput.calibration_points < 100) {
                // Check if point is different enough from previous points
                bool uniquePoint = true;
                
                // Calculate average distance to existing points
                if (m_MagCalOutput.calibration_points > 0) {
                    for (int i = 0; i < m_MagCalOutput.calibration_points; i++) {
                        int32_t distSquared = 0;
                        
                        for (int j = 0; j < MFX_NUM_AXES; j++) {
                            int32_t diff = m_MagCalInput.mag[j] - 
                                          m_MagCalOutput.calibration_buffer[i * MFX_NUM_AXES + j];
                            distSquared += FX_MUL(diff, diff);
                        }
                        
                        // If too close to an existing point, skip
                        if (distSquared < FX_MUL(MIN_DISTANCE, MIN_DISTANCE)) {
                            uniquePoint = false;
                            break;
                        }
                    }
                }
                
                if (uniquePoint) {
                    // Store current reading in calibration buffer
                    for (int i = 0; i < MFX_NUM_AXES; i++) {
                        m_MagCalOutput.calibration_buffer[m_MagCalOutput.calibration_points * MFX_NUM_AXES + i] = 
                            m_MagCalInput.mag[i];
                    }
                    
                    m_MagCalOutput.calibration_points++;
                    lastCollectionTime = now;
                    
                    // Update calibration progress
                    calibrationProgress = (float)m_MagCalOutput.calibration_points / 100.0f;
                    
                    // Log progress every 10 points
                    if (m_MagCalOutput.calibration_points % 10 == 0) {
                        m_Logger.info("Magnetic calibration progress: %d%%", 
                                     (int)(calibrationProgress * 100));
                    }
                    
                    // Check if we have enough points for soft iron calibration
                    if (m_MagCalOutput.calibration_points >= 24) {
                        // Perform soft iron calibration
                        softIronCalibration();
                    }
                }
            }
        }
    }
}

void BNO080Sensor::softIronCalibration() {
    // Calculate the ellipsoid parameters
    int32_t center[3] = {0, 0, 0};
    int32_t radii[3] = {0, 0, 0};
    int32_t covariance[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    
    // Calculate center (average of all points)
    for (int i = 0; i < m_MagCalOutput.calibration_points; i++) {
        for (int j = 0; j < MFX_NUM_AXES; j++) {
            center[j] += m_MagCalOutput.calibration_buffer[i * MFX_NUM_AXES + j];
        }
    }
    
    for (int j = 0; j < MFX_NUM_AXES; j++) {
        center[j] = center[j] / m_MagCalOutput.calibration_points;
    }
    
    // Calculate covariance matrix
    for (int i = 0; i < m_MagCalOutput.calibration_points; i++) {
        for (int j = 0; j < MFX_NUM_AXES; j++) {
            int32_t diff = m_MagCalOutput.calibration_buffer[i * MFX_NUM_AXES + j] - center[j];
            radii[j] += FX_MUL(diff, diff);
            
            for (int k = 0; k < MFX_NUM_AXES; k++) {
                int32_t diff2 = m_MagCalOutput.calibration_buffer[i * MFX_NUM_AXES + k] - center[k];
                covariance[j][k] += FX_MUL(diff, diff2);
            }
        }
    }
    
    for (int j = 0; j < MFX_NUM_AXES; j++) {
        radii[j] = radii[j] / m_MagCalOutput.calibration_points;
        
        for (int k = 0; k < MFX_NUM_AXES; k++) {
            covariance[j][k] = covariance[j][k] / m_MagCalOutput.calibration_points;
        }
    }
    
    // Calculate soft iron matrix (simplified approach)
    // We use a simplified approach that works well for most cases
    // A full ellipsoid fitting would be more accurate but more complex
    
    // First, normalize the covariance matrix by the radii
    for (int i = 0; i < MFX_NUM_AXES; i++) {
        for (int j = 0; j < MFX_NUM_AXES; j++) {
            if (radii[i] != 0) {
                // Scale by average radius to keep values in reasonable range
                int32_t avgRadius = (radii[0] + radii[1] + radii[2]) / 3;
                m_MagCalOutput.si_matrix[i][j] = FX_DIV(FX_MUL(covariance[i][j], avgRadius), radii[i]);
            } else {
                // Fallback to identity if we have zero radius (shouldn't happen)
                m_MagCalOutput.si_matrix[i][j] = (i == j) ? F_TO_FX(1.0f) : 0;
            }
        }
    }
    
    // Store hard iron bias (center of ellipsoid)
    for (int i = 0; i < MFX_NUM_AXES; i++) {
        m_MagCalOutput.hi_bias[i] = center[i];
    }
    
    // Mark calibration as valid
    m_MagCalOutput.si_valid = true;
    m_MagCalOutput.quality = MFX_MAGCAL_GOOD;
    m_MagCalOutput.confidence = 1.0f;
    
    // Log calibration results
    m_Logger.info("Soft iron calibration complete");
    m_Logger.info("Hard iron bias: [%f, %f, %f]", 
                 FX_TO_F(m_MagCalOutput.hi_bias[0]),
                 FX_TO_F(m_MagCalOutput.hi_bias[1]),
                 FX_TO_F(m_MagCalOutput.hi_bias[2]));
}

void BNO080Sensor::updateAdaptiveParameters() {
    uint32_t currentTime = millis();
    
    // Only update parameters every 250ms to avoid excessive calculations
    if (currentTime - lastParameterUpdate < 250) {
        return;
    }
    
    lastParameterUpdate = currentTime;
    
    // Base trust factors
    float baseMagTrust = 0.8f;
    float baseGyroTrust = 0.9f;
    
    // Adjust trust factors based on activity level
    switch (motionState) {
        case MOTION_STATIONARY:
            // When stationary, trust magnetometer more and gyro less
            magTrustFactor = baseMagTrust * 1.2f;
            gyroTrustFactor = baseGyroTrust * 0.9f;
            // Increase drift compensation gain when stationary
            driftCompensationGain = 0.8f;
            break;
            
        case MOTION_SLOW:
            // During slow movement, balanced trust
            magTrustFactor = baseMagTrust * 1.0f;
            gyroTrustFactor = baseGyroTrust * 1.0f;
            // Moderate drift compensation
            driftCompensationGain = 0.6f;
            break;
            
        case MOTION_MODERATE:
            // During moderate movement, trust gyro more
            magTrustFactor = baseMagTrust * 0.8f;
            gyroTrustFactor = baseGyroTrust * 1.1f;
            // Lower drift compensation during moderate movement
            driftCompensationGain = 0.4f;
            break;
            
        case MOTION_FAST:
            // During fast movement, trust gyro much more
            magTrustFactor = baseMagTrust * 0.6f;
            gyroTrustFactor = baseGyroTrust * 1.2f;
            // Minimal drift compensation during fast movement
            driftCompensationGain = 0.2f;
            break;
    }
    
    // Adjust for magnetic disturbance
    if (inDisturbance) {
        // Reduce magnetometer trust based on disturbance level
        switch (disturbanceLevel) {
            case 3: // Severe
                magTrustFactor *= 0.2f;
                // Increase gyro trust during severe disturbance
                gyroTrustFactor *= 1.2f;
                break;
                
            case 2: // Moderate
                magTrustFactor *= 0.4f;
                // Slightly increase gyro trust
                gyroTrustFactor *= 1.1f;
                break;
                
            case 1: // Mild
                magTrustFactor *= 0.7f;
                // No change to gyro trust
                break;
        }
        
        // Adjust drift compensation during disturbance
        uint32_t disturbanceTime = currentTime - disturbanceStartTime;
        
        if (disturbanceTime > 10000) { // After 10 seconds in disturbance
            // Gradually increase drift compensation to counteract long-term drift
            driftCompensationGain = min(1.0f, driftCompensationGain + 0.2f);
        }
    } else {
        // If we recently recovered from a disturbance, gradually restore parameters
        if (lastDisturbanceTime > 0) {
            uint32_t timeSinceDisturbance = currentTime - lastDisturbanceTime;
            
            if (timeSinceDisturbance < 5000) { // 5 second recovery period
                float recoveryProgress = timeSinceDisturbance / 5000.0f;
                
                // Gradually restore mag trust
                magTrustFactor = magTrustFactor * (1.0f - recoveryProgress) + 
                                (baseMagTrust * recoveryProgress);
                
                // Gradually restore gyro trust
                gyroTrustFactor = gyroTrustFactor * (1.0f - recoveryProgress) + 
                                 (baseGyroTrust * recoveryProgress);
            }
        }
    }
    
    // Apply calibration quality factor
    if (m_MagCalOutput.quality < MFX_MAGCAL_OK) {
        // Poor calibration reduces magnetometer trust
        magTrustFactor *= 0.7f;
    } else if (m_MagCalOutput.quality == MFX_MAGCAL_GOOD) {
        // Good calibration increases magnetometer trust
        magTrustFactor *= 1.1f;
    }
    
    // Ensure trust factors stay in valid range
    magTrustFactor = constrain(magTrustFactor, 0.1f, 1.0f);
    gyroTrustFactor = constrain(gyroTrustFactor, 0.5f, 1.2f);
    driftCompensationGain = constrain(driftCompensationGain, 0.1f, 1.0f);
    
    // Log parameter updates (for debugging)
    static uint32_t lastParamLog = 0;
    if (currentTime - lastParamLog > 10000) { // Log every 10 seconds
        lastParamLog = currentTime;
        m_Logger.debug("Adaptive params: magTrust=%.2f, gyroTrust=%.2f, driftGain=%.2f", 
                      magTrustFactor, gyroTrustFactor, driftCompensationGain);
    }
}

void BNO080Sensor::detectMagneticDisturbance() {
    uint32_t currentTime = millis();
    
    // Only check for disturbance every 50ms to avoid excessive calculations
    if (currentTime - lastDisturbanceCheck < 50) {
        return;
    }
    
    lastDisturbanceCheck = currentTime;
    
    // Get current magnetometer readings
    float magX = FX_TO_F(m_MagCalInput.mag[0]);
    float magY = FX_TO_F(m_MagCalInput.mag[1]);
    float magZ = FX_TO_F(m_MagCalInput.mag[2]);
    
    // Calculate current field strength
    float currentFieldStrength = sqrt(magX*magX + magY*magY + magZ*magZ);
    
    // Update field strength history
    for (int i = MAG_HISTORY_SIZE - 1; i > 0; i--) {
        magFieldHistory[i] = magFieldHistory[i-1];
    }
    magFieldHistory[0] = currentFieldStrength;
    
    // Calculate average field strength over history
    float avgFieldStrength = 0.0f;
    for (int i = 0; i < MAG_HISTORY_SIZE; i++) {
        avgFieldStrength += magFieldHistory[i];
    }
    avgFieldStrength /= MAG_HISTORY_SIZE;
    
    // Calculate field variance (stability metric)
    float fieldVariance = 0.0f;
    for (int i = 0; i < MAG_HISTORY_SIZE; i++) {
        float diff = magFieldHistory[i] - avgFieldStrength;
        fieldVariance += diff * diff;
    }
    fieldVariance /= MAG_HISTORY_SIZE;
    
    // Calculate field gradient (rate of change)
    float shortTermGradient = 0.0f;
    if (MAG_HISTORY_SIZE >= 5) {
        // Use 5-point stencil for gradient calculation
        shortTermGradient = (magFieldHistory[0] - 8*magFieldHistory[1] + 8*magFieldHistory[3] - magFieldHistory[4]) / 12.0f;
    } else {
        // Fallback to simple difference
        shortTermGradient = magFieldHistory[0] - magFieldHistory[1];
    }
    
    // Update gradient history
    for (int i = GRADIENT_HISTORY_SIZE - 1; i > 0; i--) {
        magGradientHistory[i] = magGradientHistory[i-1];
    }
    magGradientHistory[0] = abs(shortTermGradient);
    
    // Calculate average gradient magnitude
    float avgGradient = 0.0f;
    for (int i = 0; i < GRADIENT_HISTORY_SIZE; i++) {
        avgGradient += magGradientHistory[i];
    }
    avgGradient /= GRADIENT_HISTORY_SIZE;
    
    // Calculate field stability metric (0.0 = unstable, 1.0 = stable)
    // Based on variance and gradient
    const float MAX_ACCEPTABLE_VARIANCE = 5.0f;
    const float MAX_ACCEPTABLE_GRADIENT = 2.0f;
    
    float varianceStability = 1.0f - constrain(fieldVariance / MAX_ACCEPTABLE_VARIANCE, 0.0f, 1.0f);
    float gradientStability = 1.0f - constrain(avgGradient / MAX_ACCEPTABLE_GRADIENT, 0.0f, 1.0f);
    
    // Combine stability metrics with weights
    magFieldStability = 0.7f * varianceStability + 0.3f * gradientStability;
    
    // Smooth stability metric
    static float lastStability = 1.0f;
    magFieldStability = 0.8f * lastStability + 0.2f * magFieldStability;
    lastStability = magFieldStability;
    
    // Check Earth's magnetic field strength range (typically 25-65 μT)
    const float MIN_EARTH_FIELD = 25.0f;
    const float MAX_EARTH_FIELD = 65.0f;
    
    // Determine disturbance level
    int newDisturbanceLevel = 0;
    
    // Factor 1: Field strength outside Earth's normal range
    if (currentFieldStrength < MIN_EARTH_FIELD * 0.7f || currentFieldStrength > MAX_EARTH_FIELD * 1.3f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 3); // Severe
    } else if (currentFieldStrength < MIN_EARTH_FIELD * 0.8f || currentFieldStrength > MAX_EARTH_FIELD * 1.2f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 2); // Moderate
    } else if (currentFieldStrength < MIN_EARTH_FIELD * 0.9f || currentFieldStrength > MAX_EARTH_FIELD * 1.1f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 1); // Mild
    }
    
    // Factor 2: High field variance
    if (fieldVariance > 4.0f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 3); // Severe
    } else if (fieldVariance > 2.0f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 2); // Moderate
    } else if (fieldVariance > 1.0f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 1); // Mild
    }
    
    // Factor 3: High gradient (rapid change)
    if (avgGradient > 1.5f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 3); // Severe
    } else if (avgGradient > 0.8f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 2); // Moderate
    } else if (avgGradient > 0.4f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 1); // Mild
    }
    
    // Factor 4: Low stability
    if (magFieldStability < 0.3f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 3); // Severe
    } else if (magFieldStability < 0.5f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 2); // Moderate
    } else if (magFieldStability < 0.7f) {
        newDisturbanceLevel = max(newDisturbanceLevel, 1); // Mild
    }
    
    // Apply hysteresis to avoid rapid switching
    if (newDisturbanceLevel > disturbanceLevel) {
        // Immediately increase disturbance level
        disturbanceLevel = newDisturbanceLevel;
    } else if (newDisturbanceLevel < disturbanceLevel) {
        // Only decrease after consistent lower readings
        static uint32_t lastLevelDecreaseTime = 0;
        static int lastLowerLevel = 0;
        
        if (newDisturbanceLevel == lastLowerLevel) {
            // Same lower level detected again
            if (currentTime - lastLevelDecreaseTime > 500) {
                // After 500ms of consistent lower level, decrease
                disturbanceLevel = newDisturbanceLevel;
            }
        } else {
            // Reset timer for new lower level
            lastLevelDecreaseTime = currentTime;
            lastLowerLevel = newDisturbanceLevel;
        }
    }
    
    // Update disturbance state
    bool newDisturbanceState = (disturbanceLevel > 0);
    
    if (newDisturbanceState != inDisturbance) {
        if (newDisturbanceState) {
            // Entering disturbance
            inDisturbance = true;
            disturbanceStartTime = currentTime;
            m_Logger.debug("Magnetic disturbance detected (level %d)", disturbanceLevel);
        } else {
            // Exiting disturbance
            inDisturbance = false;
            lastDisturbanceTime = currentTime;
            m_Logger.debug("Magnetic disturbance ended (duration: %d ms)", 
                          currentTime - disturbanceStartTime);
        }
    } else if (inDisturbance && disturbanceLevel != lastDisturbanceLevel) {
        // Log changes in disturbance level
        m_Logger.debug("Disturbance level changed: %d -> %d", 
                      lastDisturbanceLevel, disturbanceLevel);
        lastDisturbanceLevel = disturbanceLevel;
    }
}

void BNO080Sensor::applyDriftCompensation(float& gyroZ, float deltaTime) {
    // Only apply drift compensation if we have a valid estimate
    if (abs(headingDriftRate) > 0.0001f) {
        // Calculate drift correction for this time step
        float driftCorrection = headingDriftRate * deltaTime * driftCompensationGain;
        
        // Apply adaptive correction based on activity level
        if (activityLevel > 0.7f) {
            // During high activity, reduce correction to avoid overcorrection
            driftCorrection *= 0.7f;
        } else if (activityLevel < 0.2f) {
            // During low activity, increase correction for faster convergence
            driftCorrection *= 1.2f;
        }
        
        // Limit correction to avoid overcorrection
        float maxCorrection = 0.01f;  // Max 0.01 rad per update
        if (abs(driftCorrection) > maxCorrection) {
            driftCorrection = (driftCorrection > 0) ? maxCorrection : -maxCorrection;
        }
        
        // Apply correction by subtracting from gyro reading
        gyroZ -= driftCorrection / deltaTime;
        
        // Apply additional correction during magnetic disturbance
        if (inDisturbance) {
            // Calculate time in disturbance
            uint32_t disturbanceTime = millis() - disturbanceStartTime;
            
            // After 5 seconds in disturbance, gradually increase drift compensation
            if (disturbanceTime > 5000) {
                float additionalCorrection = min(0.005f, 0.001f * (disturbanceTime - 5000) / 1000.0f);
                
                // Apply additional correction in the direction of the estimated drift
                if (headingDriftRate > 0) {
                    gyroZ -= additionalCorrection;
                } else if (headingDriftRate < 0) {
                    gyroZ += additionalCorrection;
                }
            }
        }
    }
}

float BNO080Sensor::calculateHeadingConfidence() {
    float confidence = 1.0f;
    
    // Factor 1: Magnetic calibration quality
    // Scale from MFX_MAGCAL_OK (1) to MFX_MAGCAL_GOOD (2)
    float calQuality = constrain(m_MagCalOutput.quality, MFX_MAGCAL_OK, MFX_MAGCAL_GOOD);
    float calFactor = (calQuality - MFX_MAGCAL_OK) / (MFX_MAGCAL_GOOD - MFX_MAGCAL_OK);
    calFactor = 0.7f + 0.3f * calFactor; // Map to 0.7-1.0 range
    confidence *= calFactor;
    
    // Factor 2: Magnetic field stability
    // Use the magFieldStability value (0.0 = unstable, 1.0 = stable)
    confidence *= (0.5f + 0.5f * magFieldStability);
    
    // Factor 3: Activity level
    // High activity reduces confidence in magnetic heading
    float activityFactor = 1.0f - 0.3f * constrain(activityLevel, 0.0f, 1.0f);
    confidence *= activityFactor;
    
    // Factor 4: Magnetic field strength
    // Normalize to expected Earth magnetic field strength (25-65 μT)
    float magX = FX_TO_F(m_MagCalInput.mag[0]);
    float magY = FX_TO_F(m_MagCalInput.mag[1]);
    float magZ = FX_TO_F(m_MagCalInput.mag[2]);
    float magStrength = sqrt(magX*magX + magY*magY + magZ*magZ);
    
    // Check if field strength is within expected range
    // Earth's magnetic field is typically 25-65 μT
    const float MIN_EXPECTED_FIELD = 25.0f;
    const float MAX_EXPECTED_FIELD = 65.0f;
    
    if (magStrength < MIN_EXPECTED_FIELD || magStrength > MAX_EXPECTED_FIELD) {
        // Field strength outside expected range
        float deviation = 0.0f;
        if (magStrength < MIN_EXPECTED_FIELD) {
            deviation = (MIN_EXPECTED_FIELD - magStrength) / MIN_EXPECTED_FIELD;
        } else {
            deviation = (magStrength - MAX_EXPECTED_FIELD) / MAX_EXPECTED_FIELD;
        }
        
        // Reduce confidence based on deviation (max 50% reduction)
        float strengthFactor = 1.0f - constrain(deviation, 0.0f, 0.5f);
        confidence *= strengthFactor;
    }
    
    // Factor 5: Recent disturbance history
    // Gradually restore confidence after disturbance
    if (lastDisturbanceTime > 0) {
        uint32_t timeSinceDisturbance = millis() - lastDisturbanceTime;
        
        if (timeSinceDisturbance < 10000) { // 10 seconds recovery
            float recoveryFactor = constrain(timeSinceDisturbance / 10000.0f, 0.0f, 1.0f);
            confidence *= (0.7f + 0.3f * recoveryFactor);
        }
    }
    
    // Ensure confidence stays in valid range
    return constrain(confidence, 0.1f, 1.0f);
}

void BNO080Sensor::processMagData() {
    uint32_t currentTime = millis();
    
    // Update magnetometer data for our enhanced calibration
    m_MagCalInput.mag[0] = F_TO_FX(imu.getRawMagX());
    m_MagCalInput.mag[1] = F_TO_FX(imu.getRawMagY());
    m_MagCalInput.mag[2] = F_TO_FX(imu.getRawMagZ());
    m_MagCalInput.timestamp = currentTime;
    
    // Only collect calibration points during stable periods
    if (activityLevel < 0.2f && !inDisturbance) {
        collectCalibrationPoint();
    }
    
    // Update last mag time
    lastMagTime = currentTime;
}
