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

	imu.enableLinearAccelerometer(10);ding stabilization
	initMagneticCalibration();Enable gyroscope at 50Hz

#if ENABLE_INSPECTIONlerometer(10);
	imu.enableRawGyro(10);n();
	imu.enableRawAccelerometer(10);
	if (isMagEnabled()) {
		imu.enableRawMagnetometer(10);
	}mu.enableRawAccelerometer(10);
#endifsMagEnabled()) {
	// Calibration settings:er(10);
	// EXPERIMENTAL Enable periodic calibration save to permanent memory
	imu.saveCalibrationPeriodically(true);
	imu.requestCalibrationStatus();
	// EXPERIMENTAL Disable accelerometer calibration after 1 minute to prevent
	// "stomping" bug WARNING : Executing IMU commands outside of the update loop is not
	// allowed since the address might have changed when the timer is executed!
	if (sensorType == SensorTypeID::BNO085) {ibration after 1 minute to prevent
		// For BNO085, disable accel calibration commands outside of the update loop is not
		globalTimer.in( the address might have changed when the timer is executed!
			60000,orType == SensorTypeID::BNO085) {
			[](void* sensor) {ble accel calibration
				((BNO080*)sensor)->sendCalibrateCommand(SH2_CAL_MAG | SH2_CAL_ON_TABLE);
				return true;
			},(void* sensor) {
			&imuNO080*)sensor)->sendCalibrateCommand(SH2_CAL_MAG | SH2_CAL_ON_TABLE);
		);return true;
	} else if (sensorType == SensorTypeID::BNO086) {
		// For BNO086, disable accel calibration
		// TODO: Find default flags for BNO086
		globalTimer.in(rType == SensorTypeID::BNO086) {
			60000,BNO086, disable accel calibration
			[](void* sensor) {lt flags for BNO086
				((BNO080*)sensor)->sendCalibrateCommand(SH2_CAL_MAG | SH2_CAL_ON_TABLE);
				return true;
			},(void* sensor) {
			&imuNO080*)sensor)->sendCalibrateCommand(SH2_CAL_MAG | SH2_CAL_ON_TABLE);
		);return true;
	} else {
		globalTimer.in(
			60000,
			[](void* sensor) {
				((BNO080*)sensor)->requestCalibrationStatus();
				return true;
			},(void* sensor) {
			&imuNO080*)sensor)->requestCalibrationStatus();
		);return true;
	}	},
	// imu.sendCalibrateCommand(SH2_CAL_ACCEL | SH2_CAL_GYRO_IN_HAND | SH2_CAL_MAG |
	// SH2_CAL_ON_TABLE | SH2_CAL_PLANAR);
	}
	imu.enableStabilityClassifier(500);_ACCEL | SH2_CAL_GYRO_IN_HAND | SH2_CAL_MAG |
	// SH2_CAL_ON_TABLE | SH2_CAL_PLANAR);
	lastReset = 0;
	lastData = millis();lassifier(500);
	working = true;
	configured = true;
	m_tpsCounter.reset();
	m_dataCounter.reset();
}configured = true;
	m_tpsCounter.reset();
/*_dataCounter.reset();
 * Main sensor data processing loop
 * - Reads quaternion data for orientation
 * - Handles magnetometer calibration
 * - Processes linear accelerationp
 * - Monitors sensor stability orientation
 * - Updates calibration statusration
 */- Processes linear acceleration
void BNO080Sensor::motionLoop() {
    if (imu.dataAvailable()) {s
        lastData = millis();
        080Sensor::motionLoop() {
        // Get raw magnetometer data if enabled
        float magX = 0.0f, magY = 0.0f, magZ = 0.0f;
        if (isMagEnabled()) {
            magX = imu.getMagX();ata if enabled
            magY = imu.getMagY(); 0.0f, magZ = 0.0f;
            magZ = imu.getMagZ();
            magX = imu.getMagX();
            // Store in history buffer (convert to fixed-point for existing implementation)
            magHistory[historyIndex][0] = F_TO_FX(magX);
            magHistory[historyIndex][1] = F_TO_FX(magY);
            magHistory[historyIndex][2] = F_TO_FX(magZ);-point for existing implementation)
            historyIndex = (historyIndex + 1) % HISTORY_SIZE;
            magHistory[historyIndex][1] = F_TO_FX(magY);
            // Calculate average from history buffergZ);
            int32_t sumX = 0, sumY = 0, sumZ = 0;ISTORY_SIZE;
            for (int i = 0; i < HISTORY_SIZE; i++) {
                sumX += magHistory[i][0];tory buffer
                sumY += magHistory[i][1];umZ = 0;
                sumZ += magHistory[i][2];IZE; i++) {
            }   sumX += magHistory[i][0];
            avgMag[0] = FX_TO_F(sumX / HISTORY_SIZE);
            avgMag[1] = FX_TO_F(sumY / HISTORY_SIZE);
            avgMag[2] = FX_TO_F(sumZ / HISTORY_SIZE);
            avgMag[0] = FX_TO_F(sumX / HISTORY_SIZE);
            // When we first get good calibration, save reference position
            uint8_t currentAccuracy = imu.getMagAccuracy();
            if (!hasInitialPosition && currentAccuracy >= 3) {
                initialPosition[0] = magX;bration, save reference position
                initialPosition[1] = magY;getMagAccuracy();
                initialPosition[2] = magZ;rentAccuracy >= 3) {
                hasInitialPosition = true;
                m_Logger.info("Saved initial magnetic position reference");
            }   initialPosition[2] = magZ;
                hasInitialPosition = true;
            // If in disturbance but back near reference positionference");
            if (inDisturbance && hasInitialPosition) {
                float positionDifference = 0.0f;
                for (int i = 0; i < 3; i++) {r reference position
                    float axisDiff = fabsf(avgMag[i] - initialPosition[i]);
                    positionDifference += axisDiff;
                }or (int i = 0; i < 3; i++) {
                    float axisDiff = fabsf(avgMag[i] - initialPosition[i]);
                if (positionDifference < 0.002f) {;
                    m_Logger.info("Back to reference position! Resuming normal tracking");
                    inDisturbance = false;
                    usingGyroHeading = false;2f) {
                }   m_Logger.info("Back to reference position! Resuming normal tracking");
            }       inDisturbance = false;
                    usingGyroHeading = false;
            // Detect sudden magnetic changes (additional check)
            float instantChange = sqrtf(
                powf(magX - avgMag[0], 2) +
                powf(magY - avgMag[1], 2) +es (additional check)
                powf(magZ - avgMag[2], 2)
            );  powf(magX - avgMag[0], 2) +
                powf(magY - avgMag[1], 2) +
            if (instantChange > 0.5f && !inDisturbance) {
                inDisturbance = true;
                usingGyroHeading = true;
                m_Logger.warn("Magnetic disturbance detected! Using gyro for heading");
            }   inDisturbance = true;
        }       usingGyroHeading = true;
                m_Logger.warn("Magnetic disturbance detected! Using gyro for heading");
        // Continue with quaternion processing
        Quat nRotation;
        
        if (isMagEnabled()) {ernion processing
            // If we're using gyro heading due to magnetic disturbance
            if (usingGyroHeading) {
                // Get game rotation (no mag)
                imu.getGameQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, calibrationAccuracy);
                usingGyroHeading) {
                // Verify we can access gyro data
                if (imu.getReadings() & BNO080::SENSOR_GYRO) {;
                    float gyroZ = imu.getGyroZ();
                    // Log first gyro reading when entering gyro heading mode
                    static bool firstGyroHeadingLog = true;his would involve calculating heading delta from gyro and applying to the quaternion
                    if (firstGyroHeadingLog) {
                        m_Logger.info("Initial gyro Z value: %.3f", gyroZ);ibrationAccuracy);
                        firstGyroHeadingLog = false;
                    }
                } else {sorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
                    // If no gyro data available, try to enable it
                    m_Logger.warn("Gyro data not available, re-enabling gyro");   imu.getQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, magneticAccuracyEstimate, calibrationAccuracy);
                    imu.enableGyro(50);} else {
                }librationAccuracy);
                   }
                networkConnection.sendRotationData(sensorId, &nRotation, DATA_TYPE_NORMAL, calibrationAccuracy);    
            } else {ation, DATA_TYPE_NORMAL, calibrationAccuracy);
                // Normal operation with magnetometer
                if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
                    && BNO_USE_ARVR_STABILIZATION) {// Periodic magnetometer status checks (existing code)
                    imu.getQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, magneticAccuracyEstimate, calibrationAccuracy);
                } else {
                    imu.getQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, magneticAccuracyEstimate, calibrationAccuracy);
                }currentTime - lastMagStatusCheck >= 5000) {
                e;
                networkConnection.sendRotationData(sensorId, &nRotation, DATA_TYPE_NORMAL, calibrationAccuracy);
            }
            nged
            // Periodic magnetometer status checks (existing code)uracy) {
            static uint32_t lastMagStatusCheck = 0;
            uint32_t currentTime = millis();
            
            if (currentTime - lastMagStatusCheck >= 5000) {statusText = "Uncalibrated"; break;
                lastMagStatusCheck = currentTime;ation"; break;
                uint8_t newAccuracy = imu.getMagAccuracy();
                
                // Only log if accuracy changed
                if (newAccuracy != magCalibrationAccuracy) {e calibration when we reach full calibration
                    magCalibrationAccuracy = newAccuracy;
                    const char* statusText;       delay(100); // Give it time to save
                    switch(magCalibrationAccuracy) {        break;
                        case 0: statusText = "Uncalibrated"; break;
                        case 1: statusText = "Minimal Calibration"; break;
                        case 2: statusText = "More Calibrated"; break;   
                        case 3:     m_Logger.info("Magnetometer Calibration Status: %s (Level %d/3)", 
                            statusText = "Fully Calibrated";cy);
                            // Save calibration when we reach full calibration
                            imu.saveCalibration();
                            delay(100); // Give it time to save/ If accuracy drops below 2, start recalibration
                            break;   if (magCalibrationAccuracy < 2) {
                        default: statusText = "Unknown";    imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);
                    }
                    
                    m_Logger.info("Magnetometer Calibration Status: %s (Level %d/3)", 
                                 statusText, magCalibrationAccuracy);
                }sorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
                
                // If accuracy drops below 2, start recalibration   imu.getGameQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, calibrationAccuracy);
                if (magCalibrationAccuracy < 2) {} else {
                    imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);
                }   }
            }            
        } else {ionData(sensorId, &nRotation, DATA_TYPE_NORMAL, calibrationAccuracy);
            // No magnetometer enabled - use game rotation vector
            if ((sensorType == SensorTypeID::BNO085 || sensorType == SensorTypeID::BNO086)
                && BNO_USE_ARVR_STABILIZATION) {
                imu.getGameQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, calibrationAccuracy);
            } else {        Vector3 nAccel;
                imu.getGameQuat(nRotation.x, nRotation.y, nRotation.z, nRotation.w, calibrationAccuracy);l.z, acc);
            }dSensorAcceleration(sensorId, nAccel);
            
            networkConnection.sendRotationData(sensorId, &nRotation, DATA_TYPE_NORMAL, calibrationAccuracy);
        }

        // Get linear acceleration datagetRawMagX());
        uint8_t acc;MagY());
        Vector3 nAccel;   magInput.mag[2] = F_TO_FX(imu.getRawMagZ());
        imu.getLinAccel(nAccel.x, nAccel.y, nAccel.z, acc);       magInput.timestamp = millis();
        networkConnection.sendSensorAcceleration(sensorId, nAccel);            updateMagneticCalibration(magInput);

        // Update magnetic calibration if enabled
        if (isMagEnabled()) {
            MFX_MagCal_input_t magInput;f (lastData + 1000 < millis()) {
            magInput.mag[0] = F_TO_FX(imu.getRawMagX());        m_Logger.warn("Sensor %d: No data from BNO080", sensorId);
            magInput.mag[1] = F_TO_FX(imu.getRawMagY());
            magInput.mag[2] = F_TO_FX(imu.getRawMagZ());
            magInput.timestamp = millis();
            updateMagneticCalibration(magInput);    if (imu.getStabilityClassifier() == 1) {
        }ete();
    }   }

    if (lastData + 1000 < millis()) {  updateMagneticCalibration();
        m_Logger.warn("Sensor %d: No data from BNO080", sensorId);
        lastData = millis();
    }
al status
    if (imu.getStabilityClassifier() == 1) {- SENSOR_ERROR: If reset occurred
        markRestCalibrationComplete();
    }

    updateMagneticCalibration();
}   return lastReset > 0 ? SensorStatus::SENSOR_ERROR
         : isWorking()   ? SensorStatus::SENSOR_OK
/*                       : SensorStatus::SENSOR_OFFLINE;
 * Returns current sensor operational status
 * - SENSOR_ERROR: If reset occurred
 * - SENSOR_OK: If working normally
 * - SENSOR_OFFLINE: If not workingrk
 */- Sends quaternion orientation data
SensorStatus BNO080Sensor::getSensorState() {data
    return lastReset > 0 ? SensorStatus::SENSOR_ERROR is updated
         : isWorking()   ? SensorStatus::SENSOR_OK
                         : SensorStatus::SENSOR_OFFLINE;BNO080Sensor::sendData() {
}    if (!m_fusion.isUpdated()) {

/*
 * Sends processed sensor data to the network
 * - Sends quaternion orientation datarnionQuat();
 * - Sends linear acceleration dataeration = m_fusion.getLinearAccVec();
 * - Only sends when fusion data is updated
 */dRotationData(
void BNO080Sensor::sendData() {
    if (!m_fusion.isUpdated()) {  &quaternion,
        return;        DATA_TYPE_NORMAL,
    }
    );
    Quat quaternion = m_fusion.getQuaternionQuat();
    Vector3 acceleration = m_fusion.getLinearAccVec();   networkConnection.sendSensorAcceleration(sensorId, acceleration);

    networkConnection.sendRotationData(  m_fusion.clearUpdated();
        sensorId,
        &quaternion,
        DATA_TYPE_NORMAL,
        calibrationAccuracy
    );- Handles magnetometer enable/disable

    networkConnection.sendSensorAcceleration(sensorId, acceleration);

    m_fusion.clearUpdated();
}
        m_Config.magEnabled = state;
/*ENABLED
 * Sets sensor configuration flags
 * - Handles magnetometer enable/disable
 * - Updates sensor configurationg;
 * - Triggers sensor reinitialization when needed        config.type = SlimeVR::Configuration::SensorConfigType::BNO0XX;
 */fig;
void BNO080Sensor::setFlag(uint16_t flagId, bool state) {setSensor(sensorId, config);
    if (flagId == FLAG_SENSOR_BNO0XX_MAG_ENABLED) {
        m_Config.magEnabled = state;       // Reinitialize the sensor
        magStatus = state ? MagnetometerStatus::MAG_ENABLED        motionSetup();
                          : MagnetometerStatus::MAG_DISABLED;  }

        SlimeVR::Configuration::SensorConfig config;
        config.type = SlimeVR::Configuration::SensorConfigType::BNO0XX;
        config.data.bno0XX = m_Config;quence
        configuration.setSensor(sensorId, config);structions
- Configures magnetometer for calibration
        // Reinitialize the sensor
        motionSetup();
    }
}
on type
/*
 * Initiates sensor calibration sequence
 * - Provides detailed calibration instructions
 * - Configures magnetometer for calibrationarge metal objects");
 * - Monitors calibration progress
 * - Handles calibration data storagefferent orientations");
 */axis");
void BNO080Sensor::startCalibration(int calibrationType) {tion");
    if (calibrationType == 2) {  // Magnetometer calibration type
        m_Logger.info("Starting magnetometer calibration sequence...");
        m_Logger.info("Current calibration level: %d/3", magCalibrationAccuracy); - Continue movement");
        m_Logger.info("=== Calibration Instructions ===");ements");
        m_Logger.info("1. Hold the sensor at least 0.5m away from any large metal objects");lete");
        m_Logger.info("2. Perform the following movements slowly and smoothly:");3:");
        m_Logger.info("   a) Draw figure-8 patterns in different orientations");
        m_Logger.info("   b) Rotate the sensor 360° around each axis");saved");
        m_Logger.info("   c) Keep movements slow - about 3 seconds per rotation");m_Logger.info("5. If accuracy drops:");
        m_Logger.info("3. Watch the calibration level:");
        m_Logger.info("   Level 0: Uncalibrated - Keep moving");cessary");
        m_Logger.info("   Level 1: Basic calibration - Continue movement");
        m_Logger.info("   Level 2: Good calibration - Fine-tune movements");gnetometer updates during calibration
        m_Logger.info("   Level 3: Best calibration - Calibration complete");
        m_Logger.info("4. After reaching Level 3:");
        m_Logger.info("   - Keep position stable for 2-3 seconds");
        m_Logger.info("   - Calibration will be automatically saved");_REPORTID_MAGNETIC_FIELD);
        m_Logger.info("5. If accuracy drops:");
        m_Logger.info("   - Move away from magnetic interference");
        m_Logger.info("   - Repeat calibration if necessary");tCalibrationStatus();
           
        // Enable high-rate magnetometer updates during calibration       // Small delay to let the commands process
        imu.enableMagnetometer(50);  // 50Hz updates        delay(50);
          }
        // Force recalibration
        imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);
        
        // Request calibration statusion
        imu.requestCalibrationStatus();- Sets up calibration parameters
        
        // Small delay to let the commands process structures
        delay(50);
    }ion() {
}isMagEnabled()) {
tatus
/*
 * Initializes magnetometer calibration
 * - Sets up calibration parameters
 * - Enables magnetometer readingsnitial calibration
 * - Prepares calibration structures
 */
void BNO080Sensor::initMagneticCalibration() {memset(&m_MagCalInput, 0, sizeof(m_MagCalInput));
    if (isMagEnabled()) {ut, 0, sizeof(m_MagCalOutput));
        // Request current calibration status
        imu.requestCalibrationStatus();   
               // Start calibration
        // Enable magnetic field reports        imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);
        imu.enableMagnetometer(50);  // 50Hz for better initial calibration  }
        
        // Initialize calibration structures
        memset(&m_MagCalInput, 0, sizeof(m_MagCalInput));
        memset(&m_MagCalOutput, 0, sizeof(m_MagCalOutput));Updates magnetic calibration state
        m_MagCalOutput.quality = MFX_MAGCAL_UNKNOWN;
        arameters
        // Start calibration*/
        imu.sendCalibrateCommand(SENSOR_REPORTID_MAGNETIC_FIELD);void BNO080Sensor::updateMagneticCalibration() {
    }  processMagneticData();
}

/*
 * Updates magnetic calibration stateUpdates magnetic calibration state with new input data
 * - Processes new magnetic data
 * - Updates calibration parametersmeters
 */
void BNO080Sensor::updateMagneticCalibration() {oid BNO080Sensor::updateMagneticCalibration(const MFX_MagCal_input_t& magInput) {
    processMagneticData();    m_MagCalInput = magInput;
}  processMagneticData();

/*
 * Updates magnetic calibration state with new input data
 * - Processes new magnetic data
 * - Updates calibration parametersce detection
 */
void BNO080Sensor::updateMagneticCalibration(const MFX_MagCal_input_t& magInput) {- Manages temperature compensation
    m_MagCalInput = magInput;
    processMagneticData();
} */
 {
/*
 * Processes magnetic sensor data
 * - Handles magnetic interference detection
 * - Applies hard iron compensationInput.mag[0];
 * - Manages temperature compensation    magHistory[historyIndex][1] = m_MagCalInput.mag[1];
 * - Updates calibration qualityput.mag[2];
 * - Implements disturbance recovery 1) % 6;
 */
void BNO080Sensor::processMagneticData() {te of change
    updateTemperatureCompensation();int32_t avgMag[3] = {0, 0, 0};

    // Store current readings in history
    magHistory[historyIndex][0] = m_MagCalInput.mag[0];
    magHistory[historyIndex][1] = m_MagCalInput.mag[1];
    magHistory[historyIndex][2] = m_MagCalInput.mag[2];i = 0; i < 6; i++) {
    historyIndex = (historyIndex + 1) % 6;
] += magHistory[i][axis];
    // Calculate variance and rate of change
    int32_t avgMag[3] = {0, 0, 0};ecutive samples
    int32_t variance = 0;
    int32_t maxRateOfChange = 0;nt32_t rateOfChange = abs(magHistory[i][axis] - magHistory[i-1][axis]);
       if(rateOfChange > maxRateOfChange) {
    // Calculate average and max rate of change           maxRateOfChange = rateOfChange;
    for(int i = 0; i < 6; i++) {           }
        for(int axis = 0; axis < 3; axis++) {        }
            avgMag[axis] += magHistory[i][axis];
            
            // Calculate rate of change between consecutive samples
            if(i > 0) {
                int32_t rateOfChange = abs(magHistory[i][axis] - magHistory[i-1][axis]);
                if(rateOfChange > maxRateOfChange) {
                    maxRateOfChange = rateOfChange;from this axis
                }or(int i = 0; i < 6; i++) {
            }       int32_t diff = magHistory[i][axis] - avgMag[axis];
        }        variance += (diff * diff) >> 8;
    }
    
    for(int axis = 0; axis < 3; axis++) {
        avgMag[axis] /= 6;
        itive to variations
        // Calculate variance contribution from this axis sudden changes
        for(int i = 0; i < 6; i++) {static const int32_t RECOVERY_THRESHOLD = F_TO_FX(0.15);   // Very conservative recovery
            int32_t diff = magHistory[i][axis] - avgMag[axis];FX(0.995);         // Very slow decay
            variance += (diff * diff) >> 8;
        }
    }
    etic field strength
    // Enhanced disturbance detection with multiple criteria
    static const int32_t VARIANCE_THRESHOLD = F_TO_FX(0.8);    // Much more sensitive to variations
    static const int32_t RATE_THRESHOLD = F_TO_FX(0.25);       // More sensitive to sudden changest.mag[0]) + 
    static const int32_t RECOVERY_THRESHOLD = F_TO_FX(0.15);   // Very conservative recovery.mag[1]) + 
    static const int32_t DECAY_RATE = F_TO_FX(0.995);         // Very slow decay                              FX_MUL(m_MagCalInput.mag[2], m_MagCalInput.mag[2]);
    static const int32_t RECOVERY_RATE = F_TO_FX(0.02);       // Very slow recovery
    ENGTH);
    // Add distance-based threshold scaling
    static const int32_t BASE_MAGNETIC_STRENGTH = F_TO_FX(40.0);  // Expected clean magnetic field strengthr field = more sensitive detection)
    int32_t scaledVarianceThreshold = strengthRatio > F_TO_FX(1.2) ? 
    // Calculate magnetic field strength using fixed point math F_TO_FX(0.5)) : 
    int32_t fieldStrengthSquared = FX_MUL(m_MagCalInput.mag[0], m_MagCalInput.mag[0]) + 
                                  FX_MUL(m_MagCalInput.mag[1], m_MagCalInput.mag[1]) + 
                                  FX_MUL(m_MagCalInput.mag[2], m_MagCalInput.mag[2]);bool isDisturbed = (variance > scaledVarianceThreshold) || 
    int32_t currentStrength = (int32_t)sqrtf(FX_TO_F(fieldStrengthSquared));  (maxRateOfChange > RATE_THRESHOLD) ||
    int32_t strengthRatio = FX_DIV(currentStrength, BASE_MAGNETIC_STRENGTH);ngthRatio > F_TO_FX(1.5));  // Detect strong fields
    
    // Scale thresholds based on field strength (stronger field = more sensitive detection)
    int32_t scaledVarianceThreshold = strengthRatio > F_TO_FX(1.2) ? inDisturbance) {
                                     FX_MUL(VARIANCE_THRESHOLD, F_TO_FX(0.5)) : 
                                     VARIANCE_THRESHOLD;agCalInput.mag, sizeof(lastValidMag));
    
    bool isDisturbed = (variance > scaledVarianceThreshold) || ng immediately
                       (maxRateOfChange > RATE_THRESHOLD) ||
                       (strengthRatio > F_TO_FX(1.5));  // Detect strong fields   float magHeading = atan2(m_MagCalInput.mag[1], m_MagCalInput.mag[0]);
        lastGyroHeading = magHeading;
    if(isDisturbed) {
        if(!inDisturbance) {
            inDisturbance = true;
            memcpy(lastValidMag, m_MagCalInput.mag, sizeof(lastValidMag));nt readings
            
            // Start using gyro for heading immediatelyng strong disturbances
            usingGyroHeading = true;
            float magHeading = atan2(m_MagCalInput.mag[1], m_MagCalInput.mag[0]);ent readings
            lastGyroHeading = magHeading;                        DECAY_RATE;
            lastGyroTime = millis();    m_MagCalInput.mag[i] = FX_MUL(lastValidMag[i], blendFactor) + 
        }], F_TO_FX(1.0) - blendFactor);
        }
        // During disturbance, blend between last valid and current readings
        for(int i = 0; i < 3; i++) {
            // More aggressive blending during strong disturbances
            int32_t blendFactor = strengthRatio > F_TO_FX(1.5) ? & inDisturbance) {
                                 F_TO_FX(0.98) :  // Almost entirely ignore current readingsistency check
                                 DECAY_RATE;
            m_MagCalInput.mag[i] = FX_MUL(lastValidMag[i], blendFactor) + 
                                  FX_MUL(avgMag[i], F_TO_FX(1.0) - blendFactor);
        }or(int i = 0; i < 6; i++) {
        story[i][axis] - avgMag[axis]);
        m_MagCalOutput.quality = MFX_MAGCAL_POOR;) maxDeviation = deviation;
        
    } else if(variance < RECOVERY_THRESHOLD && inDisturbance) {f(maxDeviation > F_TO_FX(0.3)) {
        // Gradual recovery with consistency check       consistentReadings = false;
        bool consistentReadings = true;        break;
        for(int axis = 0; axis < 3; axis++) {
            int32_t maxDeviation = 0;
            for(int i = 0; i < 6; i++) {
                int32_t deviation = abs(magHistory[i][axis] - avgMag[axis]);
                if(deviation > maxDeviation) maxDeviation = deviation;
            }   lastValidMag[axis] = FX_MUL(lastValidMag[axis], F_TO_FX(1.0) - RECOVERY_RATE) + 
            if(maxDeviation > F_TO_FX(0.3)) {                        FX_MUL(avgMag[axis], RECOVERY_RATE);
                consistentReadings = false;[axis];
                break;
            }
        }HOLD >> 1)) {
        gs are very stable
        if(consistentReadings) {   inDisturbance = false;
            for(int axis = 0; axis < 3; axis++) {       usingGyroHeading = false;
                lastValidMag[axis] = FX_MUL(lastValidMag[axis], F_TO_FX(1.0) - RECOVERY_RATE) +            m_MagCalOutput.quality = MFX_MAGCAL_OK;
                                    FX_MUL(avgMag[axis], RECOVERY_RATE);            }
                m_MagCalInput.mag[axis] = lastValidMag[axis];
            }
            
            if(variance < (RECOVERY_THRESHOLD >> 1)) {
                // Only exit disturbance mode if readings are very stablent32_t correctedMag[MFX_NUM_AXES];
                inDisturbance = false;    for(int i = 0; i < MFX_NUM_AXES; i++) {
                usingGyroHeading = false;m_MagCalInput.mag[i] - m_MagCalOutput.hi_bias[i];
                m_MagCalOutput.quality = MFX_MAGCAL_OK;
            }
        }
    }nt32_t magSquared = 0;
for(int i = 0; i < MFX_NUM_AXES; i++) {
    // Apply hard iron bias correction;
    int32_t correctedMag[MFX_NUM_AXES];}
    for(int i = 0; i < MFX_NUM_AXES; i++) {
        correctedMag[i] = m_MagCalInput.mag[i] - m_MagCalOutput.hi_bias[i];
    }

    // Calculate magnitudestatic const int32_t MAG_COMPRESS = F_TO_FX(1.2);      // More aggressive compression
    int32_t magSquared = 0;max field tolerance
    for(int i = 0; i < MFX_NUM_AXES; i++) {X(0.8); // Stronger compression
        magSquared += FX_MUL(correctedMag[i], correctedMag[i]);
    }magStrength > MAG_COMPRESS && magStrength <= MAG_MAX_FIELD) {
    RESS;
    int32_t magStrength = (int32_t)sqrtf(FX_TO_F(magSquared));
    
    // Enhanced field strength compensationi++) {
    static const int32_t MAG_COMPRESS = F_TO_FX(1.2);      // More aggressive compression   // More weight to last valid reading during high disturbance
    static const int32_t MAG_MAX_FIELD = F_TO_FX(2.5);     // Higher max field tolerance       int32_t reduction = FX_MUL(correctedMag[i], compressionRatio);
    static const int32_t COMPRESSION_FACTOR = F_TO_FX(0.8); // Stronger compression        correctedMag[i] -= reduction;
    
    if (magStrength > MAG_COMPRESS && magStrength <= MAG_MAX_FIELD) {
        int32_t excess = magStrength - MAG_COMPRESS;
        int32_t compressionRatio = FX_MUL(excess, COMPRESSION_FACTOR);/ Store corrected values
            for(int i = 0; i < MFX_NUM_AXES; i++) {
        for(int i = 0; i < MFX_NUM_AXES; i++) {rrectedMag[i];
            // More weight to last valid reading during high disturbance
            int32_t reduction = FX_MUL(correctedMag[i], compressionRatio);
            correctedMag[i] -= reduction;    updateHardIronCompensation();
        }  processGyroData();
    }
    
    // Store corrected values
    for(int i = 0; i < MFX_NUM_AXES; i++) {ion values
        m_MagCalInput.mag[i] = correctedMag[i];- Calculates bias corrections
    }
anages sample collection
    updateHardIronCompensation();
    processGyroData();
}/*
educed samples for faster response
/* = F_TO_FX(0.05); // Increased to 5% learning rate
 * Updates hard iron compensation values
 * - Calculates bias correctionsble (low gyro readings)
 * - Updates calibration qualityabs(imu.getGyroX()) < 1.0f && 
 * - Manages sample collection
 */abs(imu.getGyroZ()) < 1.0f) {
void BNO080Sensor::updateHardIronCompensation() {
    /*
    static const uint16_t SAMPLES_FOR_CALIBRATION = 25; // Reduced samples for faster response
    static const int32_t LEARNING_RATE = F_TO_FX(0.05); // Increased to 5% learning rate
    
    // Only update when device is stable (low gyro readings)or (int i = 0; i < MFX_NUM_AXES; i++) {
    if (abs(imu.getGyroX()) < 1.0f &&     int32_t error = m_MagCalInput.mag[i] - m_MagCalOutput.hi_bias[i];
        abs(imu.getGyroY()) < 1.0f &&  FX_MUL(error, LEARNING_RATE);
        abs(imu.getGyroZ()) < 1.0f) {}
        
        m_MagCalOutput.sample_count++;
        
        if (m_MagCalOutput.sample_count >= SAMPLES_FOR_CALIBRATION) {/ Update calibration quality
            // Update hard iron bias with exponential moving average   if (m_MagCalOutput.quality < MFX_MAGCAL_GOOD) {
            for (int i = 0; i < MFX_NUM_AXES; i++) {           m_MagCalOutput.quality = MFX_MAGCAL_GOOD;
                int32_t error = m_MagCalInput.mag[i] - m_MagCalOutput.hi_bias[i];      }
                m_MagCalOutput.hi_bias[i] += FX_MUL(error, LEARNING_RATE);       }
            }    }
              */
            m_MagCalOutput.sample_count = 0;
            
            // Update calibration quality
            if (m_MagCalOutput.quality < MFX_MAGCAL_GOOD) {
                m_MagCalOutput.quality = MFX_MAGCAL_GOOD;
            }- Applies temperature compensation
        }
    }
    */
}ata() {
unsigned long currentTime = millis();
/*rentTime - lastGyroTime) / 1000.0f;
 * Processes gyroscope datame;
 * - Handles heading calculations
 * - Applies temperature compensationnsure we have valid gyro data before using it
 * - Manages drift correction
 * - Integrates with magnetometer data
 */
void BNO080Sensor::processGyroData() {
    unsigned long currentTime = millis();singGyroHeading) {
    float deltaTime = (currentTime - lastGyroTime) / 1000.0f;
    lastGyroTime = currentTime;
    if (imu.getReadings() & BNO080::SENSOR_GYRO) {
    if(usingGyroHeading) {);
        // Get raw gyro data
        float gyroZ = imu.getGyroZ();
            static uint32_t lastGyroLog = 0;
        // Apply temperature compensation to gyro
        float temp = imu.getRawGyroX() * 0.01f;  // Temperature from gyro data
        float tempDiff = temp - 25.0f;  // Deviation from room temperature
        gyroZ *= (1.0f + TEMP_COEFF * tempDiff);  // Apply temperature correction
        
        // Update heading with temperature-compensated gyro gyro
        lastGyroHeading += gyroZ * deltaTime;mperature from gyro data
        om room temperature
        // Normalize to -PI to PIgyroZ *= (1.0f + TEMP_COEFF * tempDiff);  // Apply temperature correction
        while(lastGyroHeading > PI) lastGyroHeading -= TWO_PI;
        while(lastGyroHeading < -PI) lastGyroHeading += TWO_PI;nsated gyro
           lastGyroHeading += gyroZ * deltaTime;
        // If we have valid mag data, do slow correction to prevent drift       
        if(!inDisturbance && m_MagCalOutput.quality >= MFX_MAGCAL_OK) {           // Normalize to -PI to PI
            float magHeading = atan2(m_MagCalInput.mag[1], m_MagCalInput.mag[0]);            while(lastGyroHeading > PI) lastGyroHeading -= TWO_PI;
            float headingDiff = magHeading - lastGyroHeading;          while(lastGyroHeading < -PI) lastGyroHeading += TWO_PI;
            
            // Normalize difference to -PI to PIag data, do slow correction to prevent drift
            while(headingDiff > PI) headingDiff -= TWO_PI;m_MagCalOutput.quality >= MFX_MAGCAL_OK) {
            while(headingDiff < -PI) headingDiff += TWO_PI;Input.mag[1], m_MagCalInput.mag[0]);
                         float headingDiff = magHeading - lastGyroHeading;
            // Very slow correction factor (0.1% per update)
            lastGyroHeading += headingDiff * 0.001f;PI to PI
        }f -= TWO_PI;
    }dingDiff += TWO_PI;
}            
.1% per update)
/*
 * Updates temperature compensation
 * - Monitors temperature changes
 * - Applies dynamic compensation    m_Logger.warn("Gyro data not available");
 * - Adjusts sensor readings based on temperature
 */
void BNO080Sensor::updateTemperatureCompensation() {
    // Get temperature from gyro data packet
    float currentTemp = imu.getRawGyroX() * 0.01f;
    float tempDiff = currentTemp - lastTemp;s temperature compensation
    itors temperature changes
    // Exponential moving average for temperature
    static const float TEMP_ALPHA = 0.1f;  // Slower temperature updates
    if(abs(tempDiff) > 0.5f) {  // Only update on significant changes
        lastTemp = lastTemp * (1.0f - TEMP_ALPHA) + currentTemp * TEMP_ALPHA;BNO080Sensor::updateTemperatureCompensation() {
           // Get temperature from gyro data packet
        // Apply temperature compensation with dynamic coefficient    float currentTemp = imu.getRawGyroX() * 0.01f;












}    }        }            m_MagCalInput.mag[i] = m_MagCalInput.mag[i] * (1.0f + tempCoeff * tempDiff);        for(int i = 0; i < 3; i++) {                }            tempCoeff *= 0.5f;            // Reduce compensation for large temperature changes        if(abs(tempDiff) > 5.0f) {        float tempCoeff = TEMP_COEFF;    float tempDiff = currentTemp - lastTemp;
    
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
