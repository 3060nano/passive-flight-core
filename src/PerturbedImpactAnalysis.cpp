#include "passive_flight/PerturbedImpactAnalysis.hpp"

#include <cmath>

namespace passive_flight {
namespace {

constexpr double kMinimumVerticalSpeedMps = 1.0e-12;

bool isFinite(
    const LongitudinalPerturbationState& perturbation
) {
    return
        std::isfinite(perturbation.speedMps) &&
        std::isfinite(
            perturbation.flightPathAngleRad
        ) &&
        std::isfinite(
            perturbation.pitchRateRadps
        ) &&
        std::isfinite(
            perturbation.pitchAngleRad
        ) &&
        std::isfinite(
            perturbation.downrangeM
        ) &&
        std::isfinite(
            perturbation.altitudeM
        );
}

bool isFinite(
    const StateDerivative& derivative
) {
    return
        std::isfinite(derivative.speedMps2) &&
        std::isfinite(
            derivative.flightPathAngleRadps
        ) &&
        std::isfinite(
            derivative.pitchRateRadps2
        ) &&
        std::isfinite(
            derivative.pitchAngleRadps
        ) &&
        std::isfinite(
            derivative.downrangeMps
        ) &&
        std::isfinite(
            derivative.altitudeMps
        );
}

bool isFinite(
    const State& state
) {
    return
        std::isfinite(state.timeS) &&
        std::isfinite(state.speedMps) &&
        std::isfinite(
            state.flightPathAngleRad
        ) &&
        std::isfinite(
            state.pitchRateRadps
        ) &&
        std::isfinite(
            state.pitchAngleRad
        ) &&
        std::isfinite(
            state.downrangeM
        ) &&
        std::isfinite(
            state.altitudeM
        );
}

State makeEstimatedImpactState(
    const State& nominalImpactState,
    const PerturbedImpactParameterChanges& changes
) {
    State state = nominalImpactState;

    state.timeS +=
        changes.fallTimeS;

    state.speedMps +=
        changes.speedMps;

    state.flightPathAngleRad +=
        changes.flightPathAngleRad;

    state.pitchRateRadps +=
        changes.pitchRateRadps;

    state.pitchAngleRad +=
        changes.pitchAngleRad;

    state.downrangeM +=
        changes.downrangeM;

    /*
     * Оба состояния относятся к одному событию:
     * достижению одной и той же поверхности.
     *
     * Поэтому изменение высоты самого события
     * равно нулю.
     */
    state.altitudeM =
        nominalImpactState.altitudeM;

    return state;
}

} // namespace

PerturbedImpactAnalysis analyzePerturbedImpact(
    const PerturbedSimulationResult& result,
    const StateDerivative& finalNominalDerivative
) {
    PerturbedImpactAnalysis analysis;

    if (result.terminationReason !=
        TerminationReason::GroundReached) {
        return analysis;
    }

    if (!isFinite(result.finalNominalState) ||
        !isFinite(result.finalPerturbation) ||
        !isFinite(finalNominalDerivative)) {
        return analysis;
    }

    const double nominalVerticalSpeedMps =
        finalNominalDerivative.altitudeMps;

    if (std::abs(nominalVerticalSpeedMps) <=
        kMinimumVerticalSpeedMps) {
        return analysis;
    }

    /*
     * Условие события:
     *
     *     H*(t_f*) + Delta H_f = H_ground.
     *
     * После сдвига момента падения на Delta t_f:
     *
     *     H_dot_f* Delta t_f + Delta H_f = 0.
     *
     * Член Delta H_dot_f * Delta t_f имеет
     * второй порядок малости и в линейном
     * приближении отбрасывается.
     */
    const double fallTimeChangeS =
        -result.finalPerturbation.altitudeM /
        nominalVerticalSpeedMps;

    if (!std::isfinite(fallTimeChangeS)) {
        return analysis;
    }

    PerturbedImpactParameterChanges changes;

    changes.fallTimeS =
        fallTimeChangeS;

    /*
     * Для каждого параметра y:
     *
     *     Delta y_impact =
     *         Delta y_f + y_dot_f* Delta t_f.
     */
    changes.speedMps =
        result.finalPerturbation.speedMps +
        finalNominalDerivative.speedMps2 *
            fallTimeChangeS;

    changes.flightPathAngleRad =
        result.finalPerturbation
            .flightPathAngleRad +
        finalNominalDerivative
            .flightPathAngleRadps *
            fallTimeChangeS;

    changes.pitchRateRadps =
        result.finalPerturbation.pitchRateRadps +
        finalNominalDerivative.pitchRateRadps2 *
            fallTimeChangeS;

    changes.pitchAngleRad =
        result.finalPerturbation.pitchAngleRad +
        finalNominalDerivative.pitchAngleRadps *
            fallTimeChangeS;

    changes.downrangeM =
        result.finalPerturbation.downrangeM +
        finalNominalDerivative.downrangeMps *
            fallTimeChangeS;

    const State estimatedImpactState =
        makeEstimatedImpactState(
            result.finalNominalState,
            changes
        );

    if (!isFinite(estimatedImpactState)) {
        return analysis;
    }

    analysis.available = true;

    analysis.nominalVerticalSpeedMps =
        nominalVerticalSpeedMps;

    analysis.changes =
        changes;

    analysis.estimatedImpactState =
        estimatedImpactState;

    return analysis;
}

} // namespace passive_flight