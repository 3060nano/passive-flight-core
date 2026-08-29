#pragma once

#include "passive_flight_c_api/PerturbedFlightTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Одна точка совместной траектории:
 *
 * 1) nominal* — невозмущённое состояние x*(t);
 * 2) delta*   — малое возмущение Delta x(t);
 * 3) total*   — сумма x*(t) + Delta x(t).
 *
 * Угол атаки вычисляется как:
 *
 * alpha = theta - Theta.
 */
typedef struct PFPerturbedTrajectoryPoint {
    double timeS;

    double nominalDownrangeM;
    double nominalAltitudeM;
    double nominalSpeedMps;
    double nominalFlightPathAngleRad;
    double nominalPitchRateRadps;
    double nominalPitchAngleRad;
    double nominalAngleOfAttackRad;

    double deltaDownrangeM;
    double deltaAltitudeM;
    double deltaSpeedMps;
    double deltaFlightPathAngleRad;
    double deltaPitchRateRadps;
    double deltaPitchAngleRad;
    double deltaAngleOfAttackRad;

    double totalDownrangeM;
    double totalAltitudeM;
    double totalSpeedMps;
    double totalFlightPathAngleRad;
    double totalPitchRateRadps;
    double totalPitchAngleRad;
    double totalAngleOfAttackRad;
} PFPerturbedTrajectoryPoint;

/*
 * Возвращает совместную историю невозмущённого
 * и линейного возмущённого движения.
 *
 * Рекомендуемый двухпроходный вызов:
 *
 * 1)
 * points = NULL;
 * pointCapacity = 0.
 *
 * Функция заполнит requiredPointCount
 * и вернёт PF_RESULT_BUFFER_TOO_SMALL.
 *
 * 2)
 * Выделить points[requiredPointCount]
 * и вызвать функцию повторно.
 *
 * Внутренний шаг интегрирования остаётся 0.001 с.
 * Во внешний массив сохраняется точка каждые 0.01 с
 * плюс обязательная конечная точка.
 */
PF_API int32_t PF_CALL
pfCalculatePerturbedTrajectory(
    const PFPerturbedSimulationInput* input,

    PFPerturbedTrajectoryPoint* points,
    uint64_t pointCapacity,

    uint64_t* requiredPointCount,
    uint64_t* writtenPointCount
);

#ifdef __cplusplus
}
#endif
