#ifndef VQF_H
#define VQF_H

#include <cmath>
#include <algorithm>

#define VQF_SINGLE_PRECISION
#define VQF_NO_MOTION_BIAS_ESTIMATION
#define VQF_VR_MODE

constexpr float VQF_PI = 3.14159265358979323846f;
constexpr float VQF_SQRT2 = 1.41421356237309504880f;
constexpr float VQF_EPS = 1.1920929e-07f;

typedef float vqf_real_t;

struct VQFParams {
    VQFParams();
    
    // Core parameters
    vqf_real_t tauAcc;
    vqf_real_t tauMag;
    bool restBiasEstEnabled;
    bool magDistRejectionEnabled;
    vqf_real_t biasSigmaInit;
    vqf_real_t biasForgettingTime;
    vqf_real_t biasClip;
    vqf_real_t biasSigmaRest;
    vqf_real_t restMinT;
    vqf_real_t restFilterTau;
    vqf_real_t restThGyr;
    vqf_real_t restThAcc;
    vqf_real_t magCurrentTau;
    vqf_real_t magRefTau;
    vqf_real_t magNormTh;
    vqf_real_t magDipTh;
    vqf_real_t magNewTime;
    vqf_real_t magNewFirstTime;
    vqf_real_t magNewMinGyr;
    vqf_real_t magMinUndisturbedTime;
    vqf_real_t magMaxRejectionTime;
    vqf_real_t magRejectionFactor;
    
    // VR parameters
    vqf_real_t maxAngularRate;
    vqf_real_t vrStabilityThreshold;

    // Filter parameters
    vqf_real_t accLpB[3]{0.0f, 0.0f, 0.0f};  // Accelerometer low-pass filter coefficients
    vqf_real_t magLpB[3]{0.0f, 0.0f, 0.0f};  // Magnetometer low-pass filter coefficients
};

struct VQFState {
    // Orientation state
    vqf_real_t gyrQuat[4];
    vqf_real_t accQuat[4];
    vqf_real_t delta;
    
    // System state
    bool restDetected;
    bool magDistDetected;
    bool poseStable;
    vqf_real_t motionEnergy;

    // Filter states
    vqf_real_t lastAccLp[3];
    vqf_real_t accLpState[3*2];
    vqf_real_t lastAccCorrAngularRate;
    vqf_real_t kMagInit;
    vqf_real_t lastMagDisAngle;
    vqf_real_t lastMagCorrAngularRate;
    vqf_real_t bias[3];
    vqf_real_t biasP;
    
    // Rest detection
    vqf_real_t restLastSquaredDeviations[2];
    vqf_real_t restT;
    vqf_real_t restLastGyrLp[3];
    vqf_real_t restGyrLpState[3*2];
    vqf_real_t restLastAccLp[3];
    vqf_real_t restAccLpState[3*2];
    
    // Magnetometer state
    vqf_real_t magRefNorm;
    vqf_real_t magRefDip;
    vqf_real_t magUndisturbedT;
    vqf_real_t magRejectT;
    vqf_real_t magCandidateNorm;
    vqf_real_t magCandidateDip;
    vqf_real_t magCandidateT;
    vqf_real_t magNormDip[2];
    vqf_real_t magNormDipLpState[2*2];
    
    // VR tracking
    vqf_real_t vrCalibrationTimer;
};

class VQF {
public:
    // Default sampling times in seconds
    static constexpr float DEFAULT_GYRO_SAMPLING_TIME = 1.0f/400.0f;  // 400 Hz
    static constexpr float DEFAULT_ACC_SAMPLING_TIME = 1.0f/100.0f;   // 100 Hz
    static constexpr float DEFAULT_MAG_SAMPLING_TIME = 1.0f/100.0f;   // 100 Hz

    VQF(vqf_real_t gyrTs = DEFAULT_GYRO_SAMPLING_TIME, vqf_real_t accTs = -1.0f, vqf_real_t magTs = -1.0f);
    VQF(const VQFParams& params, vqf_real_t gyrTs = DEFAULT_GYRO_SAMPLING_TIME, vqf_real_t accTs = -1.0f, vqf_real_t magTs = -1.0f);

    void updateGyr(const vqf_real_t gyr[3], vqf_real_t gyrTs);
    void updateAcc(const vqf_real_t acc[3]);
    void updateMag(const vqf_real_t mag[3]);

    // VR extensions
    void vrInitSequence(float duration = 5.0f);
    bool getPoseStability() const;
    void getPredictedPose(vqf_real_t out[4], float predictionTime = 0.03f) const;

    // Configuration
    void setTauAcc(vqf_real_t tauAcc);
    void setTauMag(vqf_real_t tauMag);
    void resetState();

    // State access
    void getQuat6D(vqf_real_t out[4]) const;
    void getQuat9D(vqf_real_t out[4]) const;
    vqf_real_t getDelta() const;
    vqf_real_t getBiasEstimate(vqf_real_t out[3]) const;
    bool getRestDetected() const;
    bool getMagDistDetected() const;

private:
    VQFParams params;
    VQFState state;

    struct Coefficients {
        vqf_real_t gyrTs, accTs, magTs;
        vqf_real_t accLpB[3], accLpA[2];
        vqf_real_t kMag;
        vqf_real_t biasP0, biasV, biasRestW;
        vqf_real_t restGyrLpB[3], restGyrLpA[2];
        vqf_real_t restAccLpB[3], restAccLpA[2];
        vqf_real_t kMagRef;
        vqf_real_t magNormDipLpB[3], magNormDipLpA[2];
    } coeffs;

    void setup();
    void handleVrMotionAnomaly();
    void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts,
                   const vqf_real_t b[3], const vqf_real_t a[2], vqf_real_t state[], vqf_real_t out[]);
    static void quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4]);
    static void quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3]);
    static void quatApplyDelta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4]);
    static vqf_real_t norm(const vqf_real_t vec[], size_t N);
    static void normalize(vqf_real_t vec[], size_t N);
    static vqf_real_t gainFromTau(vqf_real_t tau, vqf_real_t Ts);
    static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_real_t outB[3], vqf_real_t outA[2]);
};

#endif // VQF_H