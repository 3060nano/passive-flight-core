#include "passive_flight_c_api/PassiveFlightCApi.h"

#include "passive_flight/ModelContract.hpp"
#include "passive_flight/ObjectRegistry.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

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
    PFSimulationOutput& output
) {
    output.downrangeM = 0.0;
    output.fallTimeS = 0.0;

    output.impactSpeedMps = 0.0;
    output.impactFlightPathAngleRad = 0.0;
    output.impactPitchAngleRad = 0.0;
    output.impactAngleOfAttackRad = 0.0;

    output.terminationReason =
        PF_TERMINATION_INVALID_INPUT;
}

void fillOutput(
    const passive_flight::SimulationResult& result,
    PFSimulationOutput& output
) {
    const passive_flight::SimulationSummary summary =
        passive_flight::summarize(result);

    output.downrangeM =
        summary.downrangeM;

    output.fallTimeS =
        summary.fallTimeS;

    output.impactSpeedMps =
        summary.impactSpeedMps;

    output.impactFlightPathAngleRad =
        summary.impactFlightPathAngleRad;

    output.impactPitchAngleRad =
        summary.impactPitchAngleRad;

    output.impactAngleOfAttackRad =
        summary.impactAngleOfAttackRad;

    output.terminationReason =
        mapTerminationReason(
            summary.terminationReason
        );
}

passive_flight::SimulationRequest makeRequest(
    const PFSimulationInput& input
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

PFSimulationInput makeCInput(
    const char* objectId,
    double releaseAltitudeM,
    double releaseSpeedMps
) {
    PFSimulationInput input{};

    input.objectId =
        objectId;

    input.releaseAltitudeM =
        releaseAltitudeM;

    input.releaseSpeedMps =
        releaseSpeedMps;

    return input;
}

bool isValidInput(
    const PFSimulationInput& input
) {
    return
        input.objectId != nullptr &&
        input.objectId[0] != '\0' &&

        std::isfinite(input.releaseAltitudeM) &&
        input.releaseAltitudeM > 0.0 &&

        std::isfinite(input.releaseSpeedMps) &&
        input.releaseSpeedMps > 0.0;
}

passive_flight::SimulationOptions
makeSummaryOptions() {
    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 300.0;
    options.maximumSteps = 2'000'000;

    options.groundAltitudeM = 0.0;

    options.saveHistory = false;
    options.historyStride = 100;

    return options;
}

passive_flight::SimulationOptions
makeTrajectoryOptions() {
    passive_flight::SimulationOptions options =
        makeSummaryOptions();

    options.saveHistory = true;

    /*
     * 100 шагов по 0,001 с:
     *
     * интервал сохранения равен 0,1 с.
     */
    options.historyStride = 100;

    return options;
}

int32_t copyStringToBuffer(
    const std::string& value,
    char* buffer,
    uint64_t bufferSize,
    uint64_t* requiredSize
) {
    if (requiredSize == nullptr) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    const uint64_t necessarySize =
        static_cast<uint64_t>(
            value.size() + 1
        );

    *requiredSize =
        necessarySize;

    if (buffer == nullptr ||
        bufferSize < necessarySize) {
        return PF_RESULT_BUFFER_TOO_SMALL;
    }

    std::memcpy(
        buffer,
        value.c_str(),
        static_cast<std::size_t>(
            necessarySize
        )
    );

    return PF_RESULT_OK;
}

PFTrajectoryPoint makeTrajectoryPoint(
    const passive_flight::TrajectorySample& sample
) {
    PFTrajectoryPoint point{};

    point.timeS =
        sample.state.timeS;

    point.downrangeM =
        sample.state.downrangeM;

    point.altitudeM =
        sample.state.altitudeM;

    point.speedMps =
        sample.state.speedMps;

    point.flightPathAngleRad =
        sample.state.flightPathAngleRad;

    point.pitchAngleRad =
        sample.state.pitchAngleRad;

    point.pitchRateRadps =
        sample.state.pitchRateRadps;

    point.angleOfAttackRad =
        sample.state.angleOfAttackRad();

    point.mach =
        sample.diagnostics.mach;

    point.dynamicPressurePa =
        sample.diagnostics.dynamicPressurePa;

    point.cx =
        sample.diagnostics.dragCoefficient;

    point.cy =
        sample.diagnostics.liftCoefficient;

    point.mz =
        sample.diagnostics.pitchingMomentCoefficient;

    return point;
}

int32_t validateObjectExistence(
    const PFSimulationInput& input
) {
    if (!registry().contains(
            input.objectId
        )) {
        return PF_RESULT_OBJECT_NOT_FOUND;
    }

    return PF_RESULT_OK;
}

bool hasAllSimInTechSummaryOutputs(
    double* downrangeM,
    double* fallTimeS,
    double* impactSpeedMps,
    double* impactFlightPathAngleRad,
    double* impactPitchAngleRad,
    double* impactAngleOfAttackRad,
    int32_t* terminationReason
) {
    return
        downrangeM != nullptr &&
        fallTimeS != nullptr &&
        impactSpeedMps != nullptr &&
        impactFlightPathAngleRad != nullptr &&
        impactPitchAngleRad != nullptr &&
        impactAngleOfAttackRad != nullptr &&
        terminationReason != nullptr;
}

void clearSimInTechSummaryOutputs(
    double& downrangeM,
    double& fallTimeS,
    double& impactSpeedMps,
    double& impactFlightPathAngleRad,
    double& impactPitchAngleRad,
    double& impactAngleOfAttackRad,
    int32_t& terminationReason
) {
    downrangeM = 0.0;
    fallTimeS = 0.0;

    impactSpeedMps = 0.0;
    impactFlightPathAngleRad = 0.0;
    impactPitchAngleRad = 0.0;
    impactAngleOfAttackRad = 0.0;

    terminationReason =
        PF_TERMINATION_INVALID_INPUT;
}

void copySimInTechSummaryOutputs(
    const PFSimulationOutput& source,
    double& downrangeM,
    double& fallTimeS,
    double& impactSpeedMps,
    double& impactFlightPathAngleRad,
    double& impactPitchAngleRad,
    double& impactAngleOfAttackRad,
    int32_t& terminationReason
) {
    downrangeM =
        source.downrangeM;

    fallTimeS =
        source.fallTimeS;

    impactSpeedMps =
        source.impactSpeedMps;

    impactFlightPathAngleRad =
        source.impactFlightPathAngleRad;

    impactPitchAngleRad =
        source.impactPitchAngleRad;

    impactAngleOfAttackRad =
        source.impactAngleOfAttackRad;

    terminationReason =
        source.terminationReason;
}

} // namespace

extern "C" {

const char* PF_CALL
pfGetApiVersion(void) {
    return "1.1.0";
}

const char* PF_CALL
pfGetResultCodeName(
    int32_t resultCode
) {
    switch (resultCode) {
        case PF_RESULT_OK:
            return "ok";

        case PF_RESULT_NULL_ARGUMENT:
            return "null_argument";

        case PF_RESULT_INVALID_INPUT:
            return "invalid_input";

        case PF_RESULT_OBJECT_NOT_FOUND:
            return "object_not_found";

        case PF_RESULT_BUFFER_TOO_SMALL:
            return "buffer_too_small";

        case PF_RESULT_INDEX_OUT_OF_RANGE:
            return "index_out_of_range";

        case PF_RESULT_SIMULATION_FAILED:
            return "simulation_failed";

        case PF_RESULT_INTERNAL_ERROR:
            return "internal_error";
    }

    return "unknown_result_code";
}

uint64_t PF_CALL
pfGetObjectCount(void) {
    try {
        return static_cast<uint64_t>(
            registry().size()
        );
    } catch (...) {
        return 0;
    }
}

int32_t PF_CALL
pfGetObjectId(
    uint64_t objectIndex,
    char* buffer,
    uint64_t bufferSize,
    uint64_t* requiredSize
) {
    try {
        const auto descriptors =
            registry().descriptors();

        if (objectIndex >=
            descriptors.size()) {
            return PF_RESULT_INDEX_OUT_OF_RANGE;
        }

        return copyStringToBuffer(
            descriptors[
                static_cast<std::size_t>(
                    objectIndex
                )
            ].id,
            buffer,
            bufferSize,
            requiredSize
        );
    } catch (...) {
        return PF_RESULT_INTERNAL_ERROR;
    }
}

int32_t PF_CALL
pfGetObjectDisplayName(
    uint64_t objectIndex,
    char* buffer,
    uint64_t bufferSize,
    uint64_t* requiredSize
) {
    try {
        const auto descriptors =
            registry().descriptors();

        if (objectIndex >=
            descriptors.size()) {
            return PF_RESULT_INDEX_OUT_OF_RANGE;
        }

        return copyStringToBuffer(
            descriptors[
                static_cast<std::size_t>(
                    objectIndex
                )
            ].displayName,
            buffer,
            bufferSize,
            requiredSize
        );
    } catch (...) {
        return PF_RESULT_INTERNAL_ERROR;
    }
}

int32_t PF_CALL
pfCalculate(
    const PFSimulationInput* input,
    PFSimulationOutput* output
) {
    if (input == nullptr ||
        output == nullptr) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    clearOutput(*output);

    if (!isValidInput(*input)) {
        return PF_RESULT_INVALID_INPUT;
    }

    try {
        const int32_t objectResult =
            validateObjectExistence(
                *input
            );

        if (objectResult != PF_RESULT_OK) {
            return objectResult;
        }

        const passive_flight::SimulationResult result =
            registry().simulate(
                makeRequest(*input),
                makeSummaryOptions()
            );

        fillOutput(
            result,
            *output
        );

        if (result.terminationReason !=
            passive_flight::TerminationReason::GroundReached) {
            return PF_RESULT_SIMULATION_FAILED;
        }

        return PF_RESULT_OK;
    } catch (...) {
        return PF_RESULT_INTERNAL_ERROR;
    }
}

int32_t PF_CALL
pfCalculateTrajectory(
    const PFSimulationInput* input,
    PFSimulationOutput* output,

    PFTrajectoryPoint* points,
    uint64_t pointCapacity,

    uint64_t* requiredPointCount,
    uint64_t* writtenPointCount
) {
    if (input == nullptr ||
        output == nullptr ||
        requiredPointCount == nullptr ||
        writtenPointCount == nullptr) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    clearOutput(*output);

    *requiredPointCount = 0;
    *writtenPointCount = 0;

    if (!isValidInput(*input)) {
        return PF_RESULT_INVALID_INPUT;
    }

    try {
        const int32_t objectResult =
            validateObjectExistence(
                *input
            );

        if (objectResult != PF_RESULT_OK) {
            return objectResult;
        }

        const passive_flight::SimulationResult result =
            registry().simulate(
                makeRequest(*input),
                makeTrajectoryOptions()
            );

        fillOutput(
            result,
            *output
        );

        if (result.terminationReason !=
            passive_flight::TerminationReason::GroundReached) {
            return PF_RESULT_SIMULATION_FAILED;
        }

        const uint64_t necessaryPointCount =
            static_cast<uint64_t>(
                result.history.size()
            );

        *requiredPointCount =
            necessaryPointCount;

        if (points == nullptr ||
            pointCapacity <
                necessaryPointCount) {
            return PF_RESULT_BUFFER_TOO_SMALL;
        }

        for (uint64_t index = 0;
             index < necessaryPointCount;
             ++index) {
            points[index] =
                makeTrajectoryPoint(
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
        return PF_RESULT_INTERNAL_ERROR;
    }
}

int32_t PF_CALL
pfSimInTechCalculate(
    const char* objectId,
    double releaseAltitudeM,
    double releaseSpeedMps,

    double* downrangeM,
    double* fallTimeS,

    double* impactSpeedMps,
    double* impactFlightPathAngleRad,
    double* impactPitchAngleRad,
    double* impactAngleOfAttackRad,

    int32_t* terminationReason
) {
    if (!hasAllSimInTechSummaryOutputs(
            downrangeM,
            fallTimeS,
            impactSpeedMps,
            impactFlightPathAngleRad,
            impactPitchAngleRad,
            impactAngleOfAttackRad,
            terminationReason
        )) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    clearSimInTechSummaryOutputs(
        *downrangeM,
        *fallTimeS,
        *impactSpeedMps,
        *impactFlightPathAngleRad,
        *impactPitchAngleRad,
        *impactAngleOfAttackRad,
        *terminationReason
    );

    const PFSimulationInput input =
        makeCInput(
            objectId,
            releaseAltitudeM,
            releaseSpeedMps
        );

    PFSimulationOutput output{};

    const int32_t result =
        pfCalculate(
            &input,
            &output
        );

    copySimInTechSummaryOutputs(
        output,
        *downrangeM,
        *fallTimeS,
        *impactSpeedMps,
        *impactFlightPathAngleRad,
        *impactPitchAngleRad,
        *impactAngleOfAttackRad,
        *terminationReason
    );

    return result;
}

int32_t PF_CALL
pfSimInTechCalculateTrajectory(
    const char* objectId,
    double releaseAltitudeM,
    double releaseSpeedMps,

    double* timeS,
    double* downrangeM,
    double* altitudeM,

    uint64_t pointCapacity,
    uint64_t* requiredPointCount,
    uint64_t* writtenPointCount
) {
    if (requiredPointCount == nullptr ||
        writtenPointCount == nullptr) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    *requiredPointCount = 0;
    *writtenPointCount = 0;

    const bool allTrajectoryArraysAreNull =
        timeS == nullptr &&
        downrangeM == nullptr &&
        altitudeM == nullptr;

    const bool allTrajectoryArraysArePresent =
        timeS != nullptr &&
        downrangeM != nullptr &&
        altitudeM != nullptr;

    if (!allTrajectoryArraysAreNull &&
        !allTrajectoryArraysArePresent) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    if (pointCapacity > 0 &&
        !allTrajectoryArraysArePresent) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    const PFSimulationInput input =
        makeCInput(
            objectId,
            releaseAltitudeM,
            releaseSpeedMps
        );

    PFSimulationOutput output{};

    if (allTrajectoryArraysAreNull) {
        return pfCalculateTrajectory(
            &input,
            &output,
            nullptr,
            0,
            requiredPointCount,
            writtenPointCount
        );
    }

    try {
        std::vector<PFTrajectoryPoint> points(
            static_cast<std::size_t>(
                pointCapacity
            )
        );

        const int32_t result =
            pfCalculateTrajectory(
                &input,
                &output,
                points.data(),
                pointCapacity,
                requiredPointCount,
                writtenPointCount
            );

        if (result != PF_RESULT_OK) {
            return result;
        }

        for (uint64_t index = 0;
             index < *writtenPointCount;
             ++index) {
            const std::size_t pointIndex =
                static_cast<std::size_t>(
                    index
                );

            timeS[index] =
                points[pointIndex].timeS;

            downrangeM[index] =
                points[pointIndex].downrangeM;

            altitudeM[index] =
                points[pointIndex].altitudeM;
        }

        return PF_RESULT_OK;
    } catch (...) {
        *requiredPointCount = 0;
        *writtenPointCount = 0;

        return PF_RESULT_INTERNAL_ERROR;
    }
}

} // extern "C"