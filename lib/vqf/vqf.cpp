// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT
// Shade Emry - Added changes for optimization for 8266 esp32

#include "vqf.h"

// Fast sin/cos lookup tables
#define VQF_LOOKUP_TABLE_SIZE 1024
vqf_real_t sinTable[VQF_LOOKUP_TABLE_SIZE];
vqf_real_t cosTable[VQF_LOOKUP_TABLE_SIZE];
bool tablesInitialized = false;

// Initialize lookup tables
void initLookupTables() {
    if (tablesInitialized) return;
    for (int i = 0; i < VQF_LOOKUP_TABLE_SIZE; i++) {
        float angle = i * (2.0f * VQF_PI / VQF_LOOKUP_TABLE_SIZE);
        sinTable[i] = sinf(angle);
        cosTable[i] = cosf(angle);
    }
    tablesInitialized = true;
}

// Fast sin/cos lookup
inline vqf_real_t fastSin(vqf_real_t x) {
    // Normalize angle to 0-2π range
    while (x < 0) x += 2.0f * VQF_PI;
    while (x >= 2.0f * VQF_PI) x -= 2.0f * VQF_PI;
    
    // Convert to lookup table index
    int index = (int)(x * VQF_LOOKUP_TABLE_SIZE / (2.0f * VQF_PI));
    index = index % VQF_LOOKUP_TABLE_SIZE;
    
    return sinTable[index];
}

inline vqf_real_t fastCos(vqf_real_t x) {
    // Normalize angle to 0-2π range
    while (x < 0) x += 2.0f * VQF_PI;
    while (x >= 2.0f * VQF_PI) x -= 2.0f * VQF_PI;
    
    // Convert to lookup table index
    int index = (int)(x * VQF_LOOKUP_TABLE_SIZE / (2.0f * VQF_PI));
    index = index % VQF_LOOKUP_TABLE_SIZE;
    
    return cosTable[index];
}

// Fast inverse square root implementation
inline vqf_real_t fastInvSqrt(vqf_real_t x) {
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x); // One Newton-Raphson iteration
    return x;
}

VQFParams::VQFParams() :
    tauAcc(0.02f), tauMag(9.0f),  // Much more conservative mag correction, slightly slower acc
    restBiasEstEnabled(true), magDistRejectionEnabled(true),  // Enable all protection features
    biasSigmaInit(0.1f), biasForgettingTime(0.5f), biasClip(8.0f),
    biasSigmaRest(0.02f), restMinT(0.05f), restFilterTau(0.20f),
    restThGyr(0.3f), restThAcc(0.3f), magCurrentTau(0.1f),  // Much slower magnetic field updates
    magRefTau(6.0f), magNormTh(0.08f), magDipTh(8.0f),  // Much stricter magnetic field validation
    magNewTime(20.0f), magNewFirstTime(5.0f), magNewMinGyr(20.0f),
    magMinUndisturbedTime(2.0f), magMaxRejectionTime(90.0f),  // Much longer undisturbed time requirement
    magRejectionFactor(3.0f), maxAngularRate(360.0f),
    vrStabilityThreshold(0.25f) {
        // Set cutoff filters
        accLpB[0] = 8.0f;  // Filter accelerometer noise
        magLpB[0] = 5.0f;  // Filter magnetometer noise
    }
    
VQF::VQF(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs) {
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = (accTs < 0.0f) ? gyrTs : accTs;
    coeffs.magTs = (magTs < 0.0f) ? gyrTs : magTs;
    params = VQFParams();
    setup();
}

VQF::VQF(const VQFParams& params, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs) : params(params) {
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = (accTs < 0.0f) ? gyrTs : accTs;
    coeffs.magTs = (magTs < 0.0f) ? gyrTs : magTs;
    setup();
}

void VQF::setup() {
    // Initialize lookup tables once
    initLookupTables();
    
    // Rest of the function remains unchanged
    filterCoeffs(params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA);
    coeffs.kMag = gainFromTau(params.tauMag, coeffs.magTs);

    coeffs.biasP0 = params.biasSigmaInit * params.biasSigmaInit * 10000.0f;
    coeffs.biasV = (0.1f * 0.1f) * coeffs.accTs / params.biasForgettingTime;
    coeffs.biasRestW = params.biasSigmaRest * params.biasSigmaRest * 10000.0f + coeffs.biasV;

    filterCoeffs(params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA);
    filterCoeffs(params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA);

    coeffs.kMagRef = gainFromTau(params.magRefTau, coeffs.magTs);
    if(params.magCurrentTau > 0) {
        filterCoeffs(params.magCurrentTau, coeffs.magTs, coeffs.magNormDipLpB, coeffs.magNormDipLpA);
    }
    resetState();
}

void VQF::resetState() {
    // Initialize orientation states
    state.gyrQuat[0] = 1.0f;
    state.gyrQuat[1] = state.gyrQuat[2] = state.gyrQuat[3] = 0.0f;
    state.accQuat[0] = 1.0f;
    state.accQuat[1] = state.accQuat[2] = state.accQuat[3] = 0.0f;
    state.delta = 0.0f;

    // Reset system states
    state.restDetected = false;
    state.magDistDetected = true;
    state.poseStable = false;
    state.motionEnergy = 0.0f;

    // Reset filter states
    std::fill(state.lastAccLp, state.lastAccLp+3, 0.0f);
    std::fill(state.accLpState, state.accLpState+6, 0.0f);
    state.lastAccCorrAngularRate = 0.0f;
    state.kMagInit = 1.0f;
    state.lastMagDisAngle = 0.0f;
    state.lastMagCorrAngularRate = 0.0f;
    std::fill(state.bias, state.bias+3, 0.0f);
    state.biasP = coeffs.biasP0;

    // Reset rest detection
    std::fill(state.restLastSquaredDeviations, state.restLastSquaredDeviations+2, 0.0f);
    state.restT = 0.0f;
    std::fill(state.restLastGyrLp, state.restLastGyrLp+3, 0.0f);
    std::fill(state.restGyrLpState, state.restGyrLpState+6, 0.0f);
    std::fill(state.restLastAccLp, state.restLastAccLp+3, 0.0f);
    std::fill(state.restAccLpState, state.restAccLpState+6, 0.0f);

    // Reset magnetometer state
    state.magRefNorm = 0.0f;
    state.magRefDip = 0.0f;
    state.magUndisturbedT = 0.0f;
    state.magRejectT = params.magMaxRejectionTime;
    state.magCandidateNorm = -1.0f;
    state.magCandidateDip = 0.0f;
    state.magCandidateT = 0.0f;
    std::fill(state.magNormDip, state.magNormDip+2, 0.0f);
    std::fill(state.magNormDipLpState, state.magNormDipLpState+4, 0.0f);

    // VR state
    state.vrCalibrationTimer = 0.0f;
}

void VQF::updateGyr(const vqf_real_t gyr[3], vqf_real_t gyrTs) {
    // Calculate angular rate for VR prediction
    vqf_real_t gyrNorm = norm(gyr, 3) * (180.0f/VQF_PI);
    
    // Remove estimated gyro bias
    vqf_real_t gyrNoBias[3];
    for (size_t i = 0; i < 3; i++) {
        gyrNoBias[i] = gyr[i] - state.bias[i];
    }
    
    // VR stability check
    if (gyrNorm > params.maxAngularRate) {
        handleVrMotionAnomaly();
    }
    
    // Rest detection
    if (params.restBiasEstEnabled) {
        vqf_real_t restGyrLp[3];
        filterVec(gyrNoBias, 3, params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA, state.restGyrLpState, restGyrLp);
        
        vqf_real_t restGyrDiffNorm = 0.0f;
        for (size_t i = 0; i < 3; i++) {
            vqf_real_t diff = gyrNoBias[i] - restGyrLp[i];
            restGyrDiffNorm += diff * diff;
        }
        restGyrDiffNorm = std::sqrt(restGyrDiffNorm);
        
        vqf_real_t biasClip = params.biasClip * (VQF_PI/180.0f);
        if (restGyrDiffNorm >= (params.restThGyr * (VQF_PI/180.0f)) * (params.restThGyr * (VQF_PI/180.0f)) ||
            fabsf(restGyrLp[0]) > biasClip || 
            fabsf(restGyrLp[1]) > biasClip ||
            fabsf(restGyrLp[2]) > biasClip) {
            state.restT = 0.0f;
            state.restDetected = false;
        }
    }

    // Gyro integration
    vqf_real_t angle = norm(gyrNoBias, 3) * gyrTs;
    if (norm(gyrNoBias, 3) > VQF_EPS) {
        vqf_real_t c = fastCos(angle/2);
        vqf_real_t s = fastSin(angle/2)/norm(gyrNoBias, 3);
        vqf_real_t gyrStepQuat[4] = {c, s*gyrNoBias[0], s*gyrNoBias[1], s*gyrNoBias[2]};
        quatMultiply(state.gyrQuat, gyrStepQuat, state.gyrQuat);
        normalize(state.gyrQuat, 4);
    }

    // Update motion energy
    state.motionEnergy = state.motionEnergy * 0.95f + norm(gyrNoBias, 3) * gyrTs;
}

void VQF::updateAcc(const vqf_real_t acc[3]) {
    // Normalize accelerometer reading
    vqf_real_t accNorm = norm(acc, 3);
    if (accNorm < VQF_EPS) {
        return;
    }

    vqf_real_t accNormalized[3];
    for (size_t i = 0; i < 3; i++) {
        accNormalized[i] = acc[i] / accNorm;
    }

    // Low-pass filter accelerometer data
    vqf_real_t accLp[3];
    filterVec(accNormalized, 3, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.accLpState, accLp);
    
    // Calculate rotation that aligns the accelerometer with the z-axis
    vqf_real_t angle = std::acos(accLp[2]);
    if (angle < VQF_EPS) {
        state.accQuat[0] = 1.0f;
        state.accQuat[1] = 0.0f;
        state.accQuat[2] = 0.0f;
        state.accQuat[3] = 0.0f;
    } else {
        vqf_real_t axis[3] = {-accLp[1], accLp[0], 0.0f};
        normalize(axis, 3);

        vqf_real_t s = fastSin(angle/2);  // Using fastSin for optimization
        state.accQuat[0] = fastCos(angle/2);  // Using fastCos for optimization
        state.accQuat[1] = s * axis[0];
        state.accQuat[2] = s * axis[1];
        state.accQuat[3] = s * axis[2];
    }
    
    // Now fuse accelerometer data with gyroscope data
    if (!state.magDistDetected) {
        // Only apply acc correction if magnetometer is stable
        // Apply correction with adaptive gain
        vqf_real_t k = coeffs.kMag * state.kMagInit;
        
        // Calculate difference quaternion and apply partial correction
        vqf_real_t diffQuat[4];
        quatMultiply(state.accQuat, state.gyrQuat, diffQuat);  // diffQuat = accQuat * gyrQuat^-1
        
        // Apply SLERP for smooth transition between gyr and acc orientations
        vqf_real_t corrQuat[4];
        corrQuat[0] = 1.0f - k + k * diffQuat[0];
        corrQuat[1] = k * diffQuat[1];
        corrQuat[2] = k * diffQuat[2];
        corrQuat[3] = k * diffQuat[3];
        normalize(corrQuat, 4);
        
        // Apply correction to gyro quaternion
        quatMultiply(corrQuat, state.gyrQuat, state.gyrQuat);
        normalize(state.gyrQuat, 4);
    }
    
    // Rest detection using accelerometer if enabled
    if (params.restBiasEstEnabled) {
        vqf_real_t restAccLp[3];
        filterVec(accNormalized, 3, params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, 
                 coeffs.restAccLpA, state.restAccLpState, restAccLp);
        
        vqf_real_t restAccDiffNorm = 0.0f;
        for (size_t i = 0; i < 3; i++) {
            vqf_real_t diff = accNormalized[i] - restAccLp[i];
            restAccDiffNorm += diff * diff;
        }
        restAccDiffNorm = std::sqrt(restAccDiffNorm);
        
        if (restAccDiffNorm <= params.restThAcc) {
            state.restT += coeffs.accTs;
            if (state.restT >= params.restMinT) {
                state.restDetected = true;
                
                // Update bias estimation during rest
                vqf_real_t biasLp[3];
                for (size_t i = 0; i < 3; i++) {
                    biasLp[i] = state.restLastGyrLp[i];
                }
                
                // Kalman filter update for bias
                vqf_real_t r = coeffs.biasRestW / (state.biasP + coeffs.biasRestW);
                for (size_t i = 0; i < 3; i++) {
                    state.bias[i] += r * (biasLp[i] - state.bias[i]);
                }
                state.biasP = (1.0f - r) * state.biasP + coeffs.biasV;
            }
        }
    }
}

void VQF::vrInitSequence(float duration) {
    resetState();
    state.vrCalibrationTimer = duration;
}

bool VQF::getPoseStability() const {
    return state.poseStable;
}

void VQF::getPredictedPose(vqf_real_t out[4], float predictionTime) const {
    vqf_real_t currentRate = norm(state.gyrQuat+1, 3);
    vqf_real_t tmpQuat[4] = {state.gyrQuat[0], state.gyrQuat[1], state.gyrQuat[2], state.gyrQuat[3]};
    quatApplyDelta(tmpQuat, currentRate * predictionTime, out);
}

void VQF::getQuat6D(vqf_real_t out[4]) const {
    out[0] = state.accQuat[0];
    out[1] = state.accQuat[1];
    out[2] = state.accQuat[2];
    out[3] = state.accQuat[3];
}

void VQF::getQuat9D(vqf_real_t out[4]) const {
    out[0] = state.gyrQuat[0];
    out[1] = state.gyrQuat[1];
    out[2] = state.gyrQuat[2];
    out[3] = state.gyrQuat[3];
}

void VQF::handleVrMotionAnomaly() {
    state.poseStable = false;
    state.vrCalibrationTimer = 0.0f;
}

void VQF::filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts,
                    const vqf_real_t b[3], const vqf_real_t a[2], vqf_real_t state[], vqf_real_t out[]) {
    for (size_t i = 0; i < N; i++) {
        vqf_real_t tmp = x[i];
        out[i] = b[0]*tmp + state[i*2+0];
        state[i*2+0] = b[1]*tmp - a[0]*out[i] + state[i*2+1];
        state[i*2+1] = b[2]*tmp - a[1]*out[i];
    }
}

vqf_real_t VQF::gainFromTau(vqf_real_t tau, vqf_real_t Ts) {
    if (tau < 0.0f) {
        return 0.0f;
    }
    return 1.0f - std::exp(-Ts/tau);
}

void VQF::filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_real_t outB[3], vqf_real_t outA[2]) {
    if (tau < 0.0f) {
        outB[0] = 1.0f; outB[1] = 0.0f; outB[2] = 0.0f;
        outA[0] = 0.0f; outA[1] = 0.0f;
        return;
    }
    
    vqf_real_t K = gainFromTau(tau, Ts);
    vqf_real_t lambda = std::exp(-Ts/tau);
    
    outB[0] = K;
    outB[1] = 0.0f;
    outB[2] = 0.0f;
    outA[0] = -lambda;
    outA[1] = 0.0f;
}

void VQF::quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4]) {
    out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    normalize(out, 4);
}

// Optimized norm function using fast inverse square root
vqf_real_t VQF::norm(const vqf_real_t vec[], size_t N) {
    vqf_real_t sum = 0.0f;
    for(size_t i=0; i<N; i++) sum += vec[i]*vec[i];
    return 1.0f / fastInvSqrt(sum);
    // Alternative pure approximation: return sum * fastInvSqrt(sum);
}

// Optimize normalize function
void VQF::normalize(vqf_real_t vec[], size_t N) {
    vqf_real_t n = norm(vec, N);
    if(n < VQF_EPS) return;
    
    vqf_real_t invNorm = fastInvSqrt(n*n); // 1/n
    for(size_t i=0; i<N; i++) vec[i] *= invNorm;
}

// Add missing quatApplyDelta function needed for prediction
void VQF::quatApplyDelta(const vqf_real_t quat[4], vqf_real_t delta, vqf_real_t out[4]) {
    if (delta < VQF_EPS) {
        // No significant rotation, copy the input quaternion
        out[0] = quat[0];
        out[1] = quat[1];
        out[2] = quat[2];
        out[3] = quat[3];
        return;
    }
    
    // Create rotation quaternion based on current rotation axis
    vqf_real_t axis[3] = {quat[1], quat[2], quat[3]};
    normalize(axis, 3);
    
    vqf_real_t halfDelta = delta * 0.5f;
    vqf_real_t c = fastCos(halfDelta);
    vqf_real_t s = fastSin(halfDelta);
    
    vqf_real_t deltaQuat[4] = {c, s*axis[0], s*axis[1], s*axis[2]};
    quatMultiply(quat, deltaQuat, out);
}

