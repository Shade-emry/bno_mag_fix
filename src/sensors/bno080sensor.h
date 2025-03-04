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

#include <BNO080.h>
#include "SensorFusionRestDetect.h"
#include "sensor.h"
#include "magnetic_calibration.h"

#define FLAG_SENSOR_BNO0XX_MAG_ENABLED 1

class BNO080Sensor : public Sensor {
public:
	static constexpr auto TypeID = SensorTypeID::BNO080;
	static constexpr uint8_t Address = 0x4a;
	static constexpr float GyrFreq = 400;  // 400 Hz
	static constexpr float AccFreq = 400;  // 400 Hz
	static constexpr float MagFreq = 100;  // 100 Hz

	static constexpr float GyrTs = 1.0f / GyrFreq;
	static constexpr float AccTs = 1.0f / AccFreq;
	static constexpr float MagTs = 1.0f / MagFreq;

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
	void motionSetup() override final;
	void postSetup() override { lastData = millis(); }

	void motionLoop() override final;
	void sendData() override final;
	void startCalibration(int calibrationType) override final;
	SensorStatus getSensorState() override final;
	void setFlag(uint16_t flagId, bool state) override final;

	// Magnetic calibration methods
	void updateMagneticCalibration();
	void updateMagneticCalibration(const MFX_MagCal_input_t& magInput);
	MFX_MagCal_quality_t getMagneticCalibrationQuality() const { return m_MagCalOutput.quality; }
	
	// Convert fixed-point bias values to float array when requested
	void getHardIronBias(float* bias) const {
        for(int i = 0; i < MFX_NUM_AXES; i++) {
            bias[i] = FX_TO_F(m_MagCalOutput.hi_bias[i]);
        }
    }

protected:
	void initMagneticCalibration();
    void updateTemperatureCompensation();
    void processGyroData();
    void processMagneticData();
    void updateHardIronCompensation();

    // forwarding constructor
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

private:
	BNO080 imu{};
	PinInterface* m_IntPin;
	SlimeVR::Sensors::SensorFusionRestDetect m_fusion;

	uint8_t tap;
	unsigned long lastData = 0;
	uint8_t lastReset = 0;
	BNO080Error lastError{};
	SlimeVR::Configuration::BNO0XXSensorConfig m_Config = {};

	// Magnetometer specific members
	Quat magQuaternion{};
	uint8_t magCalibrationAccuracy = 0;
	float magneticAccuracyEstimate = 999;
	bool newMagData = false;
	bool configured = false;

	// New magnetic calibration members
	MFX_knobs_t m_MagKnobs{};
	MFX_input_t m_MagInput{};
	MFX_output_t m_MagOutput{};
	MFX_MagCal_input_t m_MagCalInput{};
	MFX_MagCal_output_t m_MagCalOutput{};

	// Magnetic field history (reduced size for ESP8266)
    static const uint8_t HISTORY_SIZE = 6;  // Reduced from 10 to save memory
    int32_t magHistory[6][3];  // Use literal instead of HISTORY_SIZE for array
    uint8_t historyIndex = 0;
    int32_t lastValidMag[3] = {0, 0, 0};
    bool inDisturbance = false;

    // Gyro backup for heading
    float lastGyroHeading = 0.0f;
    unsigned long lastGyroTime = 0;
    bool usingGyroHeading = false;

    // Temperature compensation
    float lastTemp = 25.0f;
    bool tempCalibrated = false;
    static constexpr float TEMP_COEFF = -0.1f;  // Fixed coefficient to save memory

};

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
