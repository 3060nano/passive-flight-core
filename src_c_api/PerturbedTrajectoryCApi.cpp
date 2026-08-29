#include "passive_flight_c_api/PerturbedTrajectoryCApi.h"

#include "passive_flight/ObjectRegistry.hpp"
#include "passive_flight/PerturbedTrajectorySimulator.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace {

const passive_flight::ObjectRegistry& registry() {
    static const passive_flight::ObjectRegistry instance =
        passive_flight::makeDefaultObjectRegistry();

    return instance;
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
makeTrajectoryOptions() {
    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 300.0;
    options.maximumSteps = 2'000'000;

    options.groundAltitudeM = 0.0;

    options.saveHistory = true;

    /*
     * Решатель интегрирует с 0.001 с.
     *
     * В наружную историю сохраняется каждая
     * десятая интеграционная точка:
     *
     * 10 * 0.001 = 0.01 с.
     */
    options.historyStride = 10;

    return options;
}

PFPerturbedTrajectoryPoint makePoint(
    const passive_flight::PerturbedTrajectorySample& sample
) {
    PFPerturbedTrajectoryPoint point{};

    const auto& nominal =
        sample.nominalState;

    const auto& delta =
        sample.perturbation;

    const auto& total =
        sample.totalState;

    point.timeS =
        nominal.timeS;

    point.nominalDownrangeM =
        nominal.downrangeM;

    point.nominalAltitudeM =
        nominal.altitudeM;

    point.nominalSpeedMps =
        nominal.speedMps;

    point.nominalFlightPathAngleRad =
        nominal.flightPathAngleRad;

    point.nominalPitchRateRadps =
        nominal.pitchRateRadps;

    point.nominalPitchAngleRad =
        nominal.pitchAngleRad;

    point.nominalAngleOfAttackRad =
        nominal.angleOfAttackRad();

    point.deltaDownrangeM =
        delta.downrangeM;

    point.deltaAltitudeM =
        delta.altitudeM;

    point.deltaSpeedMps =
        delta.speedMps;

    point.deltaFlightPathAngleRad =
        delta.flightPathAngleRad;

    point.deltaPitchRateRadps =
        delta.pitchRateRadps;

    point.deltaPitchAngleRad =
        delta.pitchAngleRad;

    point.deltaAngleOfAttackRad =
        delta.pitchAngleRad -
        delta.flightPathAngleRad;

    point.totalDownrangeM =
        total.downrangeM;

    point.totalAltitudeM =
        total.altitudeM;

    point.totalSpeedMps =
        total.speedMps;

    point.totalFlightPathAngleRad =
        total.flightPathAngleRad;

    point.totalPitchRateRadps =
        total.pitchRateRadps;

    point.totalPitchAngleRad =
        total.pitchAngleRad;

    point.totalAngleOfAttackRad =
        total.angleOfAttackRad();

    return point;
}

} // namespace

extern "C" {

int32_t PF_CALL
pfCalculatePerturbedTrajectory(
    const PFPerturbedSimulationInput* input,

    PFPerturbedTrajectoryPoint* points,
    uint64_t pointCapacity,

    uint64_t* requiredPointCount,
    uint64_t* writtenPointCount
) {
    if (input == nullptr ||
        requiredPointCount == nullptr ||
        writtenPointCount == nullptr) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    *requiredPointCount = 0;
    *writtenPointCount = 0;

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
                makeTrajectoryOptions()
            );

        if (result.terminationReason !=
            passive_flight::
                TerminationReason::GroundReached) {
            return PF_RESULT_SIMULATION_FAILED;
        }

        const uint64_t necessaryPointCount =
            static_cast<uint64_t>(
                result.history.size()
            );

        *requiredPointCount =
            necessaryPointCount;

        if (necessaryPointCount == 0) {
            return PF_RESULT_SIMULATION_FAILED;
        }

        if (points == nullptr ||
            pointCapacity <
                necessaryPointCount) {
            return PF_RESULT_BUFFER_TOO_SMALL;
        }

        for (uint64_t index = 0;
             index < necessaryPointCount;
             ++index) {
            points[index] =
                makePoint(
                    result.history[
                        static_cast<std::size_t>(
                            index
                        )
                    ]
                );
        }

        *writtenPointCount =
            necessaryPointCount;

        return PF_RESULT_OK;
    } catch (...) {
        *requiredPointCount = 0;
        *writtenPointCount = 0;

        return PF_RESULT_INTERNAL_ERROR;
    }
}

} // extern "C"
