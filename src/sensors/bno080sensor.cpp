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

/*
 * Calculates the Euclidean norm (magnitude) of a vector
 * - Uses fixed-point multiplication for accuracy
 * - Converts back to floating point for sqrt calculation
 */
int32_t norm(const int32_t* vector, int size) {
    int32_t sum = 0;
    for (int i = 0; i < size; i++) {
        sum += FX_MUL(vector[i], vector[i]);  // Fixed-point multiplication for accuracy
    }
    return (int32_t)sqrtf(FX_TO_F(sum));  // Convert back to floating point for sqrt
}

/*
 * Initializes the BNO080 sensor and configures its settings
 * - Sets up I2C communication
 * - Configures sensor fusion modes
 * - Enables required sensor features
 * - Initializes magnetometer if enabled
 * - Sets up calibration parameters
 */
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

/*
 * Main sensor data processing loop
 * - Reads quaternion data for orientation
 * - Handles magnetometer calibration
 * - Processes linear acceleration
 * - Monitors sensor stability
 * - Updates calibration status
 */
void BNO080Sensor::motionLoop() {
    if (imu.dataAvailable()) {
        lastData = millis();
        
        Quat nRotation;  // Local quaternion variable
        
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
            
            // Monitor magnetometer calibration status
            uint8_t newAccuracy = imu.getMagAccuracy();
            if(newAccuracy != magCalibrationAccuracy) {
                magCalibrationAccuracy = newAccuracy;
                const char* statusText;
                switch(magCalibrationAccuracy) {
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
                        break;
                    default:
                        statusText = "Unknown";
                }
                m_Logger.info("Magnetometer Calibration Status: %s (Level %d/3)", statusText, magCalibrationAccuracy);
                
                if(magCalibrationAccuracy == 3) {
                    // Save calibration data when fully calibrated
                    updateHardIronCompensation();
                }
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

        // Update magnetic calibration if enabled
        if (isMagEnabled()) {
            MFX_MagCal_input_t magInput;
            magInput.mag[0] = F_TO_FX(imu.getRawMagX());
            magInput.mag[1] = F_TO_FX(imu.getRawMagY());
            magInput.mag[2] = F_TO_FX(imu.getRawMagZ());
            magInput.timestamp = millis();
            updateMagneticCalibration(magInput);
        }
    }

    if (lastData + 1000 < millis()) {
        m_Logger.warn("Sensor %d: No data from BNO080", sensorId);
        lastData = millis();
    }

    if (imu.getStabilityClassifier() == 1) {
        markRestCalibrationComplete();
    }

    updateMagneticCalibration();
}

/*
 * Returns current sensor operational status
 * - SENSOR_ERROR: If reset occurred
 * - SENSOR_OK: If working normally
 * - SENSOR_OFFLINE: If not working
 */
SensorStatus BNO080Sensor::getSensorState() {
    return lastReset > 0 ? SensorStatus::SENSOR_ERROR
         : isWorking()   ? SensorStatus::SENSOR_OK
                         : SensorStatus::SENSOR_OFFLINE;
}

/*
 * Sends processed sensor data to the network
 * - Sends quaternion orientation data
 * - Sends linear acceleration data
 * - Only sends when fusion data is updated
 */
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

/*
 * Sets sensor configuration flags
 * - Handles magnetometer enable/disable
 * - Updates sensor configuration
 * - Triggers sensor reinitialization when needed
 */
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

/*
 * Initiates sensor calibration sequence
 * - Provides detailed calibration instructions
 * - Configures magnetometer for calibration
 * - Monitors calibration progress
 * - Handles calibration data storage
 */
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

/*
 * Initializes magnetometer calibration
 * - Sets up calibration parameters
 * - Enables magnetometer readings
 * - Prepares calibration structures
 */
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
        
        // Start calibration
        imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);
    }
}

/*
 * Updates magnetic calibration state
 * - Processes new magnetic data
 * - Updates calibration parameters
 */
void BNO080Sensor::updateMagneticCalibration() {
    processMagneticData();
}

/*
 * Updates magnetic calibration state with new input data
 * - Processes new magnetic data
 * - Updates calibration parameters
 */
void BNO080Sensor::updateMagneticCalibration(const MFX_MagCal_input_t& magInput) {
    m_MagCalInput = magInput;
    processMagneticData();
}

/*
 * Processes magnetic sensor data
 * - Handles magnetic interference detection
 * - Applies hard iron compensation
 * - Manages temperature compensation
 * - Updates calibration quality
 * - Implements disturbance recovery
 */
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
    
    bool isDisturbed = (variance > scaledVarianceThreshold) || 
                       (maxRateOfChange > RATE_THRESHOLD) ||
                       (strengthRatio > F_TO_FX(1.5));  // Detect strong fields
    
    if(isDisturbed) {
        if(!inDisturbance) {
            inDisturbance = true;
            memcpy(lastValidMag, m_MagCalInput.mag, sizeof(lastValidMag));
            
            // Start using gyro for heading immediately
            usingGyroHeading = true;
            float magHeading = atan2(m_MagCalInput.mag[1], m_MagCalInput.mag[0]);
            lastGyroHeading = magHeading;
            lastGyroTime = millis();
        }
        
        // During disturbance, blend between last valid and current readings
        for(int i = 0; i < 3; i++) {
            // More aggressive blending during strong disturbances
            int32_t blendFactor = strengthRatio > F_TO_FX(1.5) ? 
                                 F_TO_FX(0.98) :  // Almost entirely ignore current readings
                                 DECAY_RATE;
            m_MagCalInput.mag[i] = FX_MUL(lastValidMag[i], blendFactor) + 
                                  FX_MUL(avgMag[i], F_TO_FX(1.0) - blendFactor);
        }
        
        m_MagCalOutput.quality = MFX_MAGCAL_POOR;
        
    } else if(variance < RECOVERY_THRESHOLD && inDisturbance) {
        // Gradual recovery with consistency check
        bool consistentReadings = true;
        for(int axis = 0; axis < 3; axis++) {
            int32_t maxDeviation = 0;
            for(int i = 0; i < 6; i++) {
                int32_t deviation = abs(magHistory[i][axis] - avgMag[axis]);
                if(deviation > maxDeviation) maxDeviation = deviation;
            }
            if(maxDeviation > F_TO_FX(0.3)) {
                consistentReadings = false;
                break;
            }
        }
        
        if(consistentReadings) {
            for(int axis = 0; axis < 3; axis++) {
                lastValidMag[axis] = FX_MUL(lastValidMag[axis], F_TO_FX(1.0) - RECOVERY_RATE) + 
                                    FX_MUL(avgMag[axis], RECOVERY_RATE);
                m_MagCalInput.mag[axis] = lastValidMag[axis];
            }
            
            if(variance < (RECOVERY_THRESHOLD >> 1)) {
                // Only exit disturbance mode if readings are very stable
                inDisturbance = false;
                usingGyroHeading = false;
                m_MagCalOutput.quality = MFX_MAGCAL_OK;
            }
        }
    }

    // Apply hard iron bias correction
    int32_t correctedMag[MFX_NUM_AXES];
    for(int i = 0; i < MFX_NUM_AXES; i++) {
        correctedMag[i] = m_MagCalInput.mag[i] - m_MagCalOutput.hi_bias[i];
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

    updateHardIronCompensation();
    processGyroData();
}

/*
 * Updates hard iron compensation values
 * - Calculates bias corrections
 * - Updates calibration quality
 * - Manages sample collection
 */
void BNO080Sensor::updateHardIronCompensation() {
    /*
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
    */
}

/*
 * Processes gyroscope data
 * - Handles heading calculations
 * - Applies temperature compensation
 * - Manages drift correction
 * - Integrates with magnetometer data
 */
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
        
        // Update heading with temperature-compensated gyro
        lastGyroHeading += gyroZ * deltaTime;
        
        // Normalize to -PI to PI
        while(lastGyroHeading > PI) lastGyroHeading -= TWO_PI;
        while(lastGyroHeading < -PI) lastGyroHeading += TWO_PI;
        
        // If we have valid mag data, do slow correction to prevent drift
        if(!inDisturbance && m_MagCalOutput.quality >= MFX_MAGCAL_OK) {
            float magHeading = atan2(m_MagCalInput.mag[1], m_MagCalInput.mag[0]);
            float headingDiff = magHeading - lastGyroHeading;
            
            // Normalize difference to -PI to PI
            while(headingDiff > PI) headingDiff -= TWO_PI;
            while(headingDiff < -PI) headingDiff += TWO_PI;
            
            // Very slow correction factor (0.1% per update)
            lastGyroHeading += headingDiff * 0.001f;
        }
    }
}

/*
 * Updates temperature compensation
 * - Monitors temperature changes
 * - Applies dynamic compensation
 * - Adjusts sensor readings based on temperature
 */
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
