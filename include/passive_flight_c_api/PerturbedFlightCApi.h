#pragma once

#include "passive_flight_c_api/PerturbedFlightTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Результат первого линейного приближения
 * изменения параметров события падения H = 0.
 *
 * Поля delta* относятся именно к изменению
 * параметров события падения, а не просто
 * к Delta x(t_f*) в момент падения
 * невозмущённой траектории.
 */
typedef struct PFPerturbedImpactOutput {
    double deltaFallTimeS;
    double deltaDownrangeM;

    double deltaImpactSpeedMps;
    double deltaImpactFlightPathAngleRad;
    double deltaImpactPitchRateRadps;
    double deltaImpactPitchAngleRad;
    double deltaImpactAngleOfAttackRad;

    /*
     * Полная оценка параметров возмущённого объекта
     * в его собственный момент достижения H = 0.
     */
    double perturbedImpactTimeS;
    double perturbedImpactDownrangeM;

    double perturbedImpactSpeedMps;
    double perturbedImpactFlightPathAngleRad;
    double perturbedImpactPitchRateRadps;
    double perturbedImpactPitchAngleRad;
    double perturbedImpactAngleOfAttackRad;

    int32_t terminationReason;
} PFPerturbedImpactOutput;

/*
 * Версия расширения C API для возмущённого движения.
 */
PF_API const char* PF_CALL
pfGetPerturbedApiVersion(void);

/*
 * Выполняет расчёт линейного продольного
 * возмущённого движения вдоль номинальной траектории
 * и терминальную поправку первого порядка
 * к событию падения H = 0.
 */
PF_API int32_t PF_CALL
pfCalculatePerturbedImpact(
    const PFPerturbedSimulationInput* input,
    PFPerturbedImpactOutput* output
);

#ifdef __cplusplus
}
#endif
