#include "passive_flight_c_api/PerturbedFlightCApi.h"

#include "passive_flight/ObjectRegistry.hpp"
#include "passive_flight/PerturbedImpactAnalysis.hpp"
#include "passive_flight/PerturbedTrajectorySimulator.hpp"

#include <cmath>
#include <cstdint>

namespace {

const passive_flight::ObjectRegistry& registry() {
    static const passive_flight::ObjectRegistry instance =
        passive_flight::makeDefaultObjectRegistry();

    return instance;
}

int32_t mapTerminationReason(
    passive_flight::TerminationReason reason
) {
    switch (reason) {
        case passive_flight::TerminationReason::GroundReached:
            return PF_TERMINATION_GROUND_REACHED;

        case passive_flight::TerminationReason::MaximumTimeReached:
            return PF_TERMINATION_MAXIMUM_TIME_REACHED;

        case passive_flight::TerminationReason::MaximumStepsReached:
            return PF_TERMINATION_MAXIMUM_STEPS_REACHED;

        case passive_flight::TerminationReason::InvalidInput:
            return PF_TERMINATION_INVALID_INPUT;

        case passive_flight::TerminationReason::InvalidState:
            return PF_TERMINATION_INVALID_STATE;
    }

    return PF_TERMINATION_INVALID_STATE;
}

void clearOutput(
    PFPerturbedImpactOutput& output
) {
    output.deltaFallTimeS = 0.0;
    output.deltaDownrangeM = 0.0;

    output.deltaImpactSpeedMps = 0.0;
    output.deltaImpactFlightPathAngleRad = 0.0;
    output.deltaImpactPitchRateRadps = 0.0;
    output.deltaImpactPitchAngleRad = 0.0;
    output.deltaImpactAngleOfAttackRad = 0.0;

    output.perturbedImpactTimeS = 0.0;
    output.perturbedImpactDownrangeM = 0.0;

    output.perturbedImpactSpeedMps = 0.0;
    output.perturbedImpactFlightPathAngleRad = 0.0;
    output.perturbedImpactPitchRateRadps = 0.0;
    output.perturbedImpactPitchAngleRad = 0.0;
    output.perturbedImpactAngleOfAttackRad = 0.0;

    output.terminationReason =
        PF_TERMINATION_INVALID_INPUT;
}

bool isFinite(
    const PFPerturbedSimulationInput& input
) {
    return
        std::isfinite(input.releaseAltitudeM) &&
        std::isfinite(input.releaseSpeedMps) &&

        std::isfinite(input.deltaSpeedMps) &&
        std::isfinite(input.deltaFlightPathAngleRad) &&
        std::isfinite(input.deltaPitchRateRadps) &&
        std::isfinite(input.deltaPitchAngleRad) &&
        std::isfinite(input.deltaDownrangeM) &&
        std::isfinite(input.deltaAltitudeM);
}

bool isValidInput(
    const PFPerturbedSimulationInput& input
) {
    return
        input.objectId != nullptr &&
        input.objectId[0] != '\0' &&

        isFinite(input) &&

        input.releaseAltitudeM > 0.0 &&
        input.releaseSpeedMps > 0.0;
}

passive_flight::SimulationRequest makeRequest(
    const PFPerturbedSimulationInput& input
) {
    passive_flight::SimulationRequest request;

    request.objectId =
        input.objectId;

    request.release.altitudeM =
        input.releaseAltitudeM;

    request.release.speedMps =
        input.releaseSpeedMps;

    return request;
}

passive_flight::LongitudinalPerturbationState
makeInitialPerturbation(
    const PFPerturbedSimulationInput& input
) {
    passive_flight::LongitudinalPerturbationState
        perturbation;

    perturbation.speedMps =
        input.deltaSpeedMps;

    perturbation.flightPathAngleRad =
        input.deltaFlightPathAngleRad;

    perturbation.pitchRateRadps =
        input.deltaPitchRateRadps;

    perturbation.pitchAngleRad =
        input.deltaPitchAngleRad;

    perturbation.downrangeM =
        input.deltaDownrangeM;

    perturbation.altitudeM =
        input.deltaAltitudeM;

    return perturbation;
}

passive_flight::SimulationOptions
makeSimulationOptions() {
    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 300.0;
    options.maximumSteps = 2'000'000;

    options.groundAltitudeM = 0.0;

    /*
     * Внешняя история этому API не нужна.
     * PerturbedTrajectorySimulator всё равно
     * внутренне сохраняет каждый номинальный шаг,
     * потому что он нужен для интегрирования Delta x.
     */
    options.saveHistory = false;
    options.historyStride = 1;

    return options;
}

void fillOutput(
    const passive_flight::PerturbedImpactAnalysis& analysis,
    passive_flight::TerminationReason terminationReason,
    PFPerturbedImpactOutput& output
) {
    const auto& changes =
        analysis.changes;

    const auto& impact =
        analysis.estimatedImpactState;

    output.deltaFallTimeS =
        changes.fallTimeS;

    output.deltaDownrangeM =
        changes.downrangeM;

    output.deltaImpactSpeedMps =
        changes.speedMps;

    output.deltaImpactFlightPathAngleRad =
        changes.flightPathAngleRad;

    output.deltaImpactPitchRateRadps =
        changes.pitchRateRadps;

    output.deltaImpactPitchAngleRad =
        changes.pitchAngleRad;

    output.deltaImpactAngleOfAttackRad =
        changes.angleOfAttackRad();

    output.perturbedImpactTimeS =
        impact.timeS;

    output.perturbedImpactDownrangeM =
        impact.downrangeM;

    output.perturbedImpactSpeedMps =
        impact.speedMps;

    output.perturbedImpactFlightPathAngleRad =
        impact.flightPathAngleRad;

    output.perturbedImpactPitchRateRadps =
        impact.pitchRateRadps;

    output.perturbedImpactPitchAngleRad =
        impact.pitchAngleRad;

    output.perturbedImpactAngleOfAttackRad =
        impact.angleOfAttackRad();

    output.terminationReason =
        mapTerminationReason(
            terminationReason
        );
}

} // namespace

extern "C" {

const char* PF_CALL
pfGetPerturbedApiVersion(void) {
    return "1.0.0";
}

int32_t PF_CALL
pfCalculatePerturbedImpact(
    const PFPerturbedSimulationInput* input,
    PFPerturbedImpactOutput* output
) {
    if (input == nullptr ||
        output == nullptr) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    clearOutput(
        *output
    );

    if (!isValidInput(
            *input
        )) {
        return PF_RESULT_INVALID_INPUT;
    }

    try {
        const passive_flight::ObjectModel* object =
            registry().findObject(
                input->objectId
            );

        if (object == nullptr) {
            return PF_RESULT_OBJECT_NOT_FOUND;
        }

        const passive_flight::
            PerturbedTrajectorySimulator simulator(
                *object
            );

        const auto result =
            simulator.simulate(
                makeRequest(*input),
                makeInitialPerturbation(*input),
                makeSimulationOptions()
            );

        output->terminationReason =
            mapTerminationReason(
                result.terminationReason
            );

        if (result.terminationReason !=
            passive_flight::
                TerminationReason::GroundReached) {
            return PF_RESULT_SIMULATION_FAILED;
        }

        const auto finalNominalEvaluation =
            simulator
                .perturbationDynamics()
                .nominalDynamics()
                .evaluate(
                    result.finalNominalState
                );

        const auto analysis =
            passive_flight::
                analyzePerturbedImpact(
                    result,
                    finalNominalEvaluation.derivative
                );

        if (!analysis.available) {
            return PF_RESULT_SIMULATION_FAILED;
        }

        fillOutput(
            analysis,
            result.terminationReason,
            *output
        );

        return PF_RESULT_OK;
    } catch (...) {
        return PF_RESULT_INTERNAL_ERROR;
    }
}

} // extern "C"
