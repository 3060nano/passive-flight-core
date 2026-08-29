#pragma once

#include "passive_flight/PerturbedTrajectorySimulator.hpp"
#include "passive_flight/Types.hpp"

namespace passive_flight {

/**
 * Изменения параметров падения, вызванные малым возмущением.
 *
 * Это не значения Delta x(t_f*) в момент падения номинальной
 * траектории, а изменения параметров самого события H = H_ground.
 *
 * Например:
 *
 *     fallTimeS = t_f - t_f*;
 *     downrangeM = L_f - L_f*.
 */
struct PerturbedImpactParameterChanges {
    double fallTimeS{0.0};

    double speedMps{0.0};
    double flightPathAngleRad{0.0};
    double pitchRateRadps{0.0};
    double pitchAngleRad{0.0};

    double downrangeM{0.0};

    [[nodiscard]]
    double angleOfAttackRad() const noexcept {
        return
            pitchAngleRad -
            flightPathAngleRad;
    }
};

/**
 * Результат первого линейного приближения параметров падения
 * возмущённого объекта.
 *
 * Номинальная траектория достигает поверхности в момент t_f*.
 * В этот момент малое возмущение по высоте в общем случае
 * не равно нулю:
 *
 *     Delta H_f = Delta H(t_f*).
 *
 * Из условия падения возмущённой траектории
 *
 *     H(t_f) = H_ground
 *
 * в первом порядке получается:
 *
 *     Delta t_f = -Delta H_f / H_dot_f*.
 *
 * Для любого другого параметра состояния y:
 *
 *     Delta y_impact =
 *         Delta y_f + y_dot_f* Delta t_f.
 */
struct PerturbedImpactAnalysis {
    bool available{false};

    /**
     * Вертикальная скорость номинальной
     * траектории при падении.
     */
    double nominalVerticalSpeedMps{0.0};

    /**
     * Изменения параметров самого события падения.
     */
    PerturbedImpactParameterChanges changes;

    /**
     * Оценённое полное состояние возмущённого объекта
     * в его собственный момент достижения поверхности.
     *
     * Высота совпадает с высотой поверхности,
     * то есть с высотой конечного состояния
     * номинальной траектории.
     */
    State estimatedImpactState;
};

/**
 * Вычисляет первое линейное приближение изменения
 * параметров события падения H = H_ground.
 *
 * finalNominalDerivative должна быть правой частью
 * невозмущённой системы, вычисленной
 * в result.finalNominalState.
 *
 * Расчёт доступен только если номинальная траектория
 * завершилась по условию GroundReached и
 * H_dot_f* не близка к нулю.
 */
[[nodiscard]]
PerturbedImpactAnalysis analyzePerturbedImpact(
    const PerturbedSimulationResult& result,
    const StateDerivative& finalNominalDerivative
);

} // namespace passive_flight