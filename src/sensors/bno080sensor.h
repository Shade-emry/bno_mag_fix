#pragma once

#include "sensor.h"
#include "configuration/configuration.h"
#include "SparkFun_BNO08x_Arduino_Library.h"

// Fixed point math definitions
#define F_TO_FX(f) ((int32_t)((f) * 65536.0f))
#define FX_TO_F(fx) ((float)(fx) / 65536.0f)
#define FX_MUL(a, b) (((int64_t)(a) * (b)) >> 16)
#define FX_DIV(a, b) (((int64_t)(a) << 16) / (b))

// Constants - using Arduino's existing definitions
// #define PI 3.14159265358979323846f
// #define TWO_PI 6.28318530717958647693f
#define TEMP_COEFF 0.001f // Temperature coefficient for compensation (0.1% per degree)
#define MFX_NUM_AXES 3
#define FLAG_SENSOR_BNO0XX_MAG_ENABLED 1

// Magnetic field calibration structure
typedef struct {
    int32_t mag[3];
    uint32_t timestamp;
} MFX_MagCal_input_t;

// Magnetic field calibration output structure
typedef struct {
    int32_t hi_bias[3];
    uint16_t sample_count;
    uint8_t quality;
} MFX_MagCal_output_t;

// Magnetic calibration quality definitions
enum MFX_MagCal_quality_t {
    MFX_MAGCAL_UNKNOWN = 0,
    MFX_MAGCAL_POOR,
    MFX_MAGCAL_OK,
    MFX_MAGCAL_GOOD,
    MFX_MAGCAL_EXCELLENT
};

class BNO080Sensor : public Sensor {
public:
    static constexpr auto TypeID = SensorTypeID::BNO085;
    static constexpr uint8_t Address = 0x4a;

    BNO080Sensor(
        uint8_t id,
        uint8_t i2cAddress, 
        float rotation,
        SlimeVR::SensorInterface* sensorInterface,
        PinInterface* intPin,
        int
    ) 
        : Sensor(
            "BNO08x", 
            SensorTypeID::BNO085,
            id,
            i2cAddress,
            rotation,
            sensorInterface
        )
        , addr(i2cAddress), m_IntPin(intPin),
          magStatus(MagnetometerStatus::MAG_DISABLED), 
          calibrationAccuracy(0), magneticAccuracyEstimate(0),
          inDisturbance(false), usingGyroHeading(false),
          hasInitialPosition(false),
          lastReset(0), lastData(0), lastTemp(25.0f), lastGyroHeading(0),
          lastGyroTime(0), magCalibrationAccuracy(0) {
        memset(&m_MagCalInput, 0, sizeof(m_MagCalInput));
        memset(&m_MagCalOutput, 0, sizeof(m_MagCalOutput));
        m_MagCalOutput.quality = MFX_MAGCAL_UNKNOWN;
        memset(lastValidMag, 0, sizeof(lastValidMag));
        memset(initialPosition, 0, sizeof(initialPosition));
        memset(magHistory, 0, sizeof(magHistory));
        historyIndex = 0;
    };
    ~BNO080Sensor(){};

    void motionSetup() override;
    void motionLoop() override;
    void sendData() override;
    SensorStatus getSensorState() override;
    
    void startCalibration(int calibrationType) override;
    void setFlag(uint16_t flagId, bool state) override;

private:
    BNO08x imu;  // SparkFun BNO08x library
    uint8_t addr;
    PinInterface* m_IntPin;
    MagnetometerStatus magStatus;
    // Using SlimeVR::Configuration::BNO0XXSensorConfig directly from the existing project
    bool magEnabled = true;
    bool configured = false;

    uint8_t calibrationAccuracy;
    uint8_t magneticAccuracyEstimate;
    
    // Fusion interface for quaternion data
    class {
    public:
        bool isUpdated() const { return updated; }
        void setUpdated(bool state) { updated = state; }
        Quat getQuaternionQuat() const { return quaternion; }
        void setQuaternion(const Quat& q) { quaternion = q; updated = true; }
        Vector3 getLinearAccVec() const { return linearAcceleration; }
        void setLinearAccVec(const Vector3& acc) { linearAcceleration = acc; }
    private:
        Quat quaternion;
        Vector3 linearAcceleration;
        bool updated = false;
    } m_fusion;
    
    // Magnetic calibration variables
    MFX_MagCal_input_t m_MagCalInput;
    MFX_MagCal_output_t m_MagCalOutput;
    int32_t lastValidMag[3];
    int32_t initialPosition[3];
    int32_t magHistory[6][3];
    uint8_t historyIndex;
    bool inDisturbance;
    bool usingGyroHeading;
    bool hasInitialPosition;
    
    // Timing and state variables
    uint32_t lastReset;
    uint32_t lastData;
    float lastTemp;
    float lastGyroHeading;
    unsigned long lastGyroTime;
    uint8_t magCalibrationAccuracy;

    // Helper functions
    void initMagneticCalibration();
    void updateMagneticCalibration();
    void updateMagneticCalibration(const MFX_MagCal_input_t& magInput);
    void processMagneticData();
    void updateHardIronCompensation();
    void processGyroData();
    void updateTemperatureCompensation();
    void applyHeadingCorrection(Quat& quaternion);
    
    bool isMagEnabled() const {
        return magStatus == MagnetometerStatus::MAG_ENABLED;
    }
};
