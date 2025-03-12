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

// External dependencies
#include <BNO080.h>
#include "SensorFusionRestDetect.h"
#include "sensor.h"
#include "magnetic_calibration.h"

// Flag to enable/disable magnetometer functionality
#define FLAG_SENSOR_BNO0XX_MAG_ENABLED 1

/**
 * @brief Main sensor class for BNO080 IMU
 * Handles sensor initialization, data processing, and magnetic calibration
 */
class BNO080Sensor : public Sensor {
public:
    // Sensor configuration constants
    static constexpr auto TypeID = SensorTypeID::BNO080;
    static constexpr uint8_t Address = 0x4a;  // Default I2C address
    static constexpr float GyrFreq = 400;     // Gyroscope frequency in Hz
    static constexpr float AccFreq = 400;     // Accelerometer frequency in Hz
    static constexpr float MagFreq = 100;     // Magnetometer frequency in Hz

    // Time periods for each sensor (in seconds)
    static constexpr float GyrTs = 1.0f / GyrFreq;
    static constexpr float AccTs = 1.0f / AccFreq;
    static constexpr float MagTs = 1.0f / MagFreq;

    /**
     * @brief Constructor for BNO080 sensor
     * @param id Sensor identifier
     * @param i2cAddress I2C address of the sensor
     * @param rotation Mounting rotation compensation
     * @param sensorInterface Interface for sensor communication
     * @param intPin Interrupt pin interface
     */
    BNO080Sensor(
        uint8_t id,
        uint8_t i2cAddress,
        float rotation,
        SlimeVR::SensorInterface* sensorInterface,
        PinInterface* intPin,
        int
    )
        : Sensor(
            "BNO080Sensor",
            SensorTypeID::BNO080,
            id,
            i2cAddress,
            rotation,
            sensorInterface
        )
        , m_IntPin(intPin)
        , m_fusion(GyrTs, AccTs, MagTs) {
            // Initialize magnetic calibration structures
            initMagneticCalibration();
        };
    ~BNO080Sensor(){};

    // Core sensor interface methods
    void motionSetup() override final;        // Initialize sensor
    void postSetup() override { lastData = millis(); }  // Post-initialization setup
    void motionLoop() override final;         // Main sensor processing loop
    void sendData() override final;           // Transmit sensor data
    void startCalibration(int calibrationType) override final;  // Begin calibration
    SensorStatus getSensorState() override final;  // Get current sensor status
    void setFlag(uint16_t flagId, bool state) override final;  // Set sensor flags

    // Magnetic calibration interface
    /**
     * @brief Update magnetic calibration with current readings
     */
    void updateMagneticCalibration();
    
    /**
     * @brief Update magnetic calibration with specific input
     * @param magInput Magnetic calibration input data
     */
    void updateMagneticCalibration(const MFX_MagCal_input_t& magInput);
    
    /**
     * @brief Get current magnetic calibration quality
     * @return Calibration quality status
     */
    MFX_MagCal_quality_t getMagneticCalibrationQuality() const { return m_MagCalOutput.quality; }
    
    /**
     * @brief Convert and retrieve hard iron bias values
     * @param bias Output array for bias values
     */
    void getHardIronBias(float* bias) const {
        for(int i = 0; i < MFX_NUM_AXES; i++) {
            bias[i] = FX_TO_F(m_MagCalOutput.hi_bias[i]);
        }
    }

protected:
    /**
     * @brief Alternative constructor for derived classes
     */
    BNO080Sensor(
        const char* sensorName,
        SensorTypeID imuId,
        uint8_t id,
        uint8_t i2cAddress,
        float rotation,
        SlimeVR::SensorInterface* sensorInterface,
        PinInterface* intPin,
        int
    )
        : Sensor(sensorName, imuId, id, i2cAddress, rotation, sensorInterface)
        , m_IntPin(intPin)
        , m_fusion(GyrTs, AccTs, MagTs) {
            initMagneticCalibration();
        };

    // Protected utility methods
    void initMagneticCalibration();           // Initialize magnetic calibration
    void processGyroData();                    // Process gyroscope readings
    void processMagneticData();                // Process magnetometer readings
    void updateHardIronCompensation();         // Update hard iron compensation

private:
    BNO080 imu{};                             // IMU instance
    PinInterface* m_IntPin;                    // Interrupt pin
    SlimeVR::Sensors::SensorFusionRestDetect m_fusion;  // Fusion algorithm

    // Sensor state tracking
    uint8_t tap;                              // Tap detection state
    unsigned long lastData = 0;                // Timestamp of last data
    uint8_t lastReset = 0;                    // Last reset counter
    BNO080Error lastError{};                  // Last error state
    SlimeVR::Configuration::BNO0XXSensorConfig m_Config = {};  // Sensor config

    // Magnetometer state
    Quat magQuaternion{};                     // Magnetic orientation
    uint8_t magCalibrationAccuracy = 0;       // Calibration accuracy
    float magneticAccuracyEstimate = 999;     // Accuracy estimate
    bool newMagData = false;                  // New data flag
    bool configured = false;                  // Configuration state

    // Magnetic calibration state
    MFX_knobs_t m_MagKnobs{};                // Calibration parameters
    MFX_input_t m_MagInput{};                // Input data
    MFX_output_t m_MagOutput{};              // Output data
    MFX_MagCal_input_t m_MagCalInput{};      // Calibration input
    MFX_MagCal_output_t m_MagCalOutput{};    // Calibration output

    // Magnetic field history tracking
    static const uint8_t HISTORY_SIZE = 6;    // History buffer size
    int32_t magHistory[6][3];                // Magnetic field history
    uint8_t historyIndex = 0;                // Current history index
    int32_t lastValidMag[3] = {0, 0, 0};    // Last valid reading
    bool inDisturbance = false;              // Disturbance flag

    // Gyro backup tracking
    float lastGyroHeading = 0.0f;            // Last gyro heading
    unsigned long lastGyroTime = 0;          // Last gyro timestamp
    bool usingGyroHeading = false;           // Gyro backup state

    // Position memory
    bool hasInitialPosition = false;
    int32_t initialPosition[3] = {0, 0, 0};
};

/**
 * @brief BNO085 sensor variant
 * Extends BNO080 with specific configurations for BNO085
 */
class BNO085Sensor : public BNO080Sensor {
public:
    static constexpr auto TypeID = SensorTypeID::BNO085;
    BNO085Sensor(
        uint8_t id,
        uint8_t i2cAddress,
        float rotation,
        SlimeVR::SensorInterface* sensorInterface,
        PinInterface* intPin,
        int extraParam
    )
        : BNO080Sensor(
            "BNO085Sensor",
            SensorTypeID::BNO085,
            id,
            i2cAddress,
            rotation,
            sensorInterface,
            intPin,
            extraParam
        ){};
};

/**
 * @brief BNO086 sensor variant
 * Extends BNO080 with specific configurations for BNO086
 */
class BNO086Sensor : public BNO080Sensor {
public:
    static constexpr auto TypeID = SensorTypeID::BNO086;
    BNO086Sensor(
        uint8_t id,
        uint8_t i2cAddress,
        float rotation,
        SlimeVR::SensorInterface* sensorInterface,
        PinInterface* intPin,
        int extraParam
    )
        : BNO080Sensor(
            "BNO086Sensor",
            SensorTypeID::BNO086,
            id,
            i2cAddress,
            rotation,
            sensorInterface,
            intPin,
            extraParam
        ){};
};

#endif
