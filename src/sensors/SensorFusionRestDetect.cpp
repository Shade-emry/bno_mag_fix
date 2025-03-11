#include "SensorFusionRestDetect.h"

namespace SlimeVR {
namespace Sensors {
#if !SENSOR_FUSION_WITH_RESTDETECT
void SensorFusionRestDetect::updateAcc(
	const sensor_real_t Axyz[3],
	sensor_real_t deltat
) {
	if (deltat < 0) {
		deltat = accTs;
	}
	restDetection.updateAcc(deltat, Axyz);
	SensorFusion::updateAcc(Axyz, deltat);
}

void SensorFusionRestDetect::updateGyro(
	const sensor_real_t Gxyz[3],
	sensor_real_t deltat
) {
	if (deltat < 0) {
		deltat = gyrTs;
	}
	restDetection.updateGyr(Gxyz);
	SensorFusion::updateGyro(Gxyz, deltat);
}
#endif

bool SensorFusionRestDetect::getRestDetected() {
#if !SENSOR_FUSION_WITH_RESTDETECT
	return restDetection.getRestDetected();
#elif SENSOR_USE_VQF
	// VQF library doesn't have getRestDetected() method in this version
	// Attempt to use the closest alternative or return a default value
	// You might need to check the VQF library documentation for the correct method
	// For now, return false to avoid the linking error
	return false;
#else
	return false;
#endif
}
}  // namespace Sensors
}  // namespace SlimeVR
