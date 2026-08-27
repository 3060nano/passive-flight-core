#include "passive_flight/PerturbedTrajectorySimulator.hpp"

#include <cmath>
#include <cstddef>
#include <utility>

namespace passive_flight {
namespace {

bool isFinite(
    const LongitudinalPerturbationState& perturbation
) {
    return
        std::isfinite(perturbation.speedMps) &&
        std::isfinite(
            perturbation.flightPathAngleRad
        ) &&
        std::isfinite(perturbation.pitchRateRadps) &&
        std::isfinite(perturbation.pitchAngleRad) &&
        std::isfinite(perturbation.downrangeM) &&
        std::isfinite(perturbation.altitudeM);
}

bool isFinite(
    const State& state
) {
    return
        std::isfinite(state.timeS) &&
        std::isfinite(state.speedMps) &&
        std::isfinite(state.flightPathAngleRad) &&
        std::isfinite(state.pitchRateRadps) &&
        std::isfinite(state.pitchAngleRad) &&
        std::isfinite(state.downrangeM) &&
        std::isfinite(state.altitudeM);
}

State combine(
    const State& nominalState,
    const LongitudinalPerturbationState& perturbation
) {
    State totalState = nominalState;

    totalState.speedMps +=
        perturbation.speedMps;

    totalState.flightPathAngleRad +=
        perturbation.flightPathAngleRad;

    totalState.pitchRateRadps +=
        perturbation.pitchRateRadps;

    totalState.pitchAngleRad +=
        perturbation.pitchAngleRad;

    totalState.downrangeM +=
        perturbation.downrangeM;

    totalState.altitudeM +=
        perturbation.altitudeM;

    return totalState;
}

LongitudinalPerturbationState makeEulerStep(
    const LongitudinalPerturbationState& current,
    const LongitudinalPerturbationDerivative& derivative,
    double timeStepS
) {
    LongitudinalPerturbationState next;

    next.speedMps =
        current.speedMps +
        timeStepS * derivative.speedMps2;

    next.flightPathAngleRad =
        current.flightPathAngleRad +
        timeStepS *
            derivative.flightPathAngleRadps;

    next.pitchRateRadps =
        current.pitchRateRadps +
        timeStepS * derivative.pitchRateRadps2;

    next.pitchAngleRad =
        current.pitchAngleRad +
        timeStepS * derivative.pitchAngleRadps;

    next.downrangeM =
        current.downrangeM +
        timeStepS * derivative.downrangeMps;

    next.altitudeM =
        current.altitudeM +
        timeStepS * derivative.altitudeMps;

    return next;
}

void appendSample(
    PerturbedSimulationResult& result,
    const State& nominalState,
    const LongitudinalPerturbationState& perturbation
) {
    PerturbedTrajectorySample sample;

    sample.nominalState = nominalState;
    sample.perturbation = perturbation;
    sample.totalState = combine(
        nominalState,
        perturbation
    );

    if (!result.history.empty() &&
        std::abs(
            result.history.back()
                .nominalState.timeS -
            nominalState.timeS
        ) <= 1.0e-12) {
        result.history.back() = sample;
        return;
    }

    result.history.push_back(sample);
}

} // namespace

PerturbedTrajectorySimulator::
PerturbedTrajectorySimulator(
    ObjectModel object,
    PerturbationLinearizationOptions
        linearizationOptions
)
    : object_(std::move(object)),
      nominalSimulator_(object_),
      perturbationDynamics_(
          object_,
          linearizationOptions
      ) {
}

PerturbedSimulationResult
PerturbedTrajectorySimulator::simulate(
    const SimulationRequest& request,
    const LongitudinalPerturbationState&
        initialPerturbation,
    const SimulationOptions& options
) const {
    PerturbedSimulationResult result;

    if (!isFinite(initialPerturbation) ||
        options.historyStride == 0) {
        result.terminationReason =
            TerminationReason::InvalidInput;

        return result;
    }

    SimulationOptions internalOptions = options;

    /*
     * Для интегрирования Delta x нужны все временные
     * узлы невозмущённого решателя независимо от того,
     * какую частоту сохранения запросил внешний код.
     */
    internalOptions.saveHistory = true;
    internalOptions.historyStride = 1;

    const SimulationResult nominalResult =
        nominalSimulator_.simulate(
            request,
            internalOptions
        );

    result.finalNominalState =
        nominalResult.finalState;

    if (nominalResult.history.empty()) {
        result.terminationReason =
            nominalResult.terminationReason;

        return result;
    }

    LongitudinalPerturbationState current =
        initialPerturbation;

    if (options.saveHistory) {
        appendSample(
            result,
            nominalResult.history.front().state,
            current
        );
    }

    for (std::size_t nextIndex = 1;
         nextIndex < nominalResult.history.size();
         ++nextIndex) {
        const std::size_t currentIndex =
            nextIndex - 1;

        const State& nominalCurrent =
            nominalResult.history[currentIndex].state;

        const State& nominalNext =
            nominalResult.history[nextIndex].state;

        const double timeStepS =
            nominalNext.timeS -
            nominalCurrent.timeS;

        if (!std::isfinite(timeStepS) ||
            timeStepS <= 0.0) {
            result.terminationReason =
                TerminationReason::InvalidState;

            result.finalNominalState =
                nominalCurrent;

            result.finalPerturbation = current;
            result.finalTotalState = combine(
                nominalCurrent,
                current
            );

            return result;
        }

        try {
            const auto evaluation =
                perturbationDynamics_.evaluate(
                    nominalCurrent,
                    current
                );

            current = makeEulerStep(
                current,
                evaluation.derivative,
                timeStepS
            );
        } catch (...) {
            result.terminationReason =
                TerminationReason::InvalidState;

            result.finalNominalState =
                nominalCurrent;

            result.finalPerturbation = current;
            result.finalTotalState = combine(
                nominalCurrent,
                current
            );

            return result;
        }

        const State totalState = combine(
            nominalNext,
            current
        );

        if (!isFinite(current) ||
            !isFinite(totalState)) {
            result.terminationReason =
                TerminationReason::InvalidState;

            result.finalNominalState =
                nominalNext;

            result.finalPerturbation = current;
            result.finalTotalState = totalState;

            return result;
        }

        const bool isLastSample =
            nextIndex + 1 ==
            nominalResult.history.size();

        const bool matchesHistoryStride =
            nextIndex % options.historyStride == 0;

        if (options.saveHistory &&
            (matchesHistoryStride || isLastSample)) {
            appendSample(
                result,
                nominalNext,
                current
            );
        }
    }

    result.terminationReason =
        nominalResult.terminationReason;

    result.finalNominalState =
        nominalResult.finalState;

    result.finalPerturbation = current;

    result.finalTotalState = combine(
        result.finalNominalState,
        current
    );

    return result;
}

const ObjectModel&
PerturbedTrajectorySimulator::object() const noexcept {
    return object_;
}

const ForwardEulerSimulator&
PerturbedTrajectorySimulator::
nominalSimulator() const noexcept {
    return nominalSimulator_;
}

const PerturbedLongitudinalDynamics&
PerturbedTrajectorySimulator::
perturbationDynamics() const noexcept {
    return perturbationDynamics_;
}

} // namespace passive_flight
