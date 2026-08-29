#pragma once

#include "passive_flight_c_api/PassiveFlightCApi.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Общие входные параметры для расчётов
 * линейного продольного возмущённого движения.
 *
 * Номинальный режим:
 *
 * - objectId;
 * - releaseAltitudeM;
 * - releaseSpeedMps.
 *
 * Начальный вектор малых возмущений:
 *
 * Delta x0 =
 * [Delta V,
 *  Delta Theta,
 *  Delta omega_z,
 *  Delta theta,
 *  Delta x,
 *  Delta H]^T.
 *
 * Угол атаки не является независимым состоянием:
 *
 * Delta alpha =
 *     Delta theta - Delta Theta.
 */
typedef struct PFPerturbedSimulationInput {
    const char* objectId;

    double releaseAltitudeM;
    double releaseSpeedMps;

    double deltaSpeedMps;
    double deltaFlightPathAngleRad;
    double deltaPitchRateRadps;
    double deltaPitchAngleRad;
    double deltaDownrangeM;
    double deltaAltitudeM;
} PFPerturbedSimulationInput;

#ifdef __cplusplus
}
#endif
