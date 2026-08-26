#include "passive_flight_c_api/PassiveFlightCApi.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message
) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message);
    }
}

std::string getObjectId(
    uint64_t objectIndex
) {
    uint64_t requiredSize = 0;

    const int32_t queryResult =
        pfGetObjectId(
            objectIndex,
            nullptr,
            0,
            &requiredSize
        );

    require(
        queryResult ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "Object ID size query failed"
    );

    require(
        requiredSize > 1,
        "Object ID size is invalid"
    );

    std::vector<char> buffer(
        static_cast<std::size_t>(
            requiredSize
        )
    );

    const int32_t copyResult =
        pfGetObjectId(
            objectIndex,
            buffer.data(),
            requiredSize,
            &requiredSize
        );

    require(
        copyResult == PF_RESULT_OK,
        "Object ID copy failed"
    );

    return std::string(
        buffer.data()
    );
}

std::string getObjectDisplayName(
    uint64_t objectIndex
) {
    uint64_t requiredSize = 0;

    const int32_t queryResult =
        pfGetObjectDisplayName(
            objectIndex,
            nullptr,
            0,
            &requiredSize
        );

    require(
        queryResult ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "Object name size query failed"
    );

    std::vector<char> buffer(
        static_cast<std::size_t>(
            requiredSize
        )
    );

    const int32_t copyResult =
        pfGetObjectDisplayName(
            objectIndex,
            buffer.data(),
            requiredSize,
            &requiredSize
        );

    require(
        copyResult == PF_RESULT_OK,
        "Object name copy failed"
    );

    return std::string(
        buffer.data()
    );
}

PFSimulationInput makeInput() {
    PFSimulationInput input{};

    input.objectId =
        "ABSTRACT_500_UMPK_V1";

    input.releaseAltitudeM =
        100.0;

    input.releaseSpeedMps =
        200.0;

    return input;
}

void testApiVersion() {
    const char* version =
        pfGetApiVersion();

    require(
        version != nullptr,
        "API version is null"
    );

    require(
        std::string(version) ==
            "1.0.0",
        "API version is incorrect"
    );
}

void testResultCodeNames() {
    require(
        std::string(
            pfGetResultCodeName(
                PF_RESULT_OK
            )
        ) == "ok",
        "OK result name is incorrect"
    );

    require(
        std::string(
            pfGetResultCodeName(
                PF_RESULT_OBJECT_NOT_FOUND
            )
        ) == "object_not_found",
        "Object-not-found name is incorrect"
    );
}

void testObjectList() {
    require(
        pfGetObjectCount() == 1,
        "C API must expose one object"
    );

    require(
        getObjectId(0) ==
            "ABSTRACT_500_UMPK_V1",
        "C API object ID is incorrect"
    );

    require(
        !getObjectDisplayName(0).empty(),
        "C API object name is empty"
    );

    uint64_t requiredSize = 0;

    const int32_t result =
        pfGetObjectId(
            100,
            nullptr,
            0,
            &requiredSize
        );

    require(
        result ==
            PF_RESULT_INDEX_OUT_OF_RANGE,
        "Invalid object index must be rejected"
    );
}

void testSummaryCalculation() {
    const PFSimulationInput input =
        makeInput();

    PFSimulationOutput output{};

    const int32_t result =
        pfCalculate(
            &input,
            &output
        );

    require(
        result == PF_RESULT_OK,
        "Summary calculation failed"
    );

    require(
        output.terminationReason ==
            PF_TERMINATION_GROUND_REACHED,
        "Simulation did not reach ground"
    );

    require(
        output.downrangeM > 0.0,
        "Downrange must be positive"
    );

    require(
        output.fallTimeS > 0.0,
        "Fall time must be positive"
    );

    require(
        output.impactSpeedMps > 0.0,
        "Impact speed must be positive"
    );

    require(
        output.impactFlightPathAngleRad < 0.0,
        "Impact trajectory must point downward"
    );
}

void testUnknownObject() {
    PFSimulationInput input =
        makeInput();

    input.objectId =
        "UNKNOWN_OBJECT";

    PFSimulationOutput output{};

    const int32_t result =
        pfCalculate(
            &input,
            &output
        );

    require(
        result ==
            PF_RESULT_OBJECT_NOT_FOUND,
        "Unknown object must be rejected"
    );
}

void testInvalidInput() {
    PFSimulationOutput output{};

    require(
        pfCalculate(
            nullptr,
            &output
        ) == PF_RESULT_NULL_ARGUMENT,
        "Null input must be rejected"
    );

    PFSimulationInput input =
        makeInput();

    input.releaseSpeedMps = 0.0;

    require(
        pfCalculate(
            &input,
            &output
        ) == PF_RESULT_INVALID_INPUT,
        "Zero speed must be rejected"
    );
}

void testTrajectoryCalculation() {
    const PFSimulationInput input =
        makeInput();

    PFSimulationOutput firstOutput{};

    uint64_t requiredPointCount = 0;
    uint64_t writtenPointCount = 0;

    const int32_t queryResult =
        pfCalculateTrajectory(
            &input,
            &firstOutput,
            nullptr,
            0,
            &requiredPointCount,
            &writtenPointCount
        );

    require(
        queryResult ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "Trajectory size query failed"
    );

    require(
        requiredPointCount > 1,
        "Trajectory must contain several points"
    );

    require(
        writtenPointCount == 0,
        "Size query must not write points"
    );

    std::vector<PFTrajectoryPoint> points(
        static_cast<std::size_t>(
            requiredPointCount
        )
    );

    PFSimulationOutput secondOutput{};

    const int32_t calculationResult =
        pfCalculateTrajectory(
            &input,
            &secondOutput,
            points.data(),
            requiredPointCount,
            &requiredPointCount,
            &writtenPointCount
        );

    require(
        calculationResult ==
            PF_RESULT_OK,
        "Trajectory calculation failed"
    );

    require(
        writtenPointCount ==
            requiredPointCount,
        "Incorrect number of trajectory points"
    );

    requireNear(
        points.front().timeS,
        0.0,
        1.0e-12,
        "Trajectory must start at time zero"
    );

    requireNear(
        points.front().altitudeM,
        input.releaseAltitudeM,
        1.0e-12,
        "Trajectory must start at release altitude"
    );

    requireNear(
        points.back().altitudeM,
        0.0,
        1.0e-12,
        "Trajectory must end at ground"
    );

    requireNear(
        points.back().timeS,
        secondOutput.fallTimeS,
        1.0e-12,
        "Last point time must equal fall time"
    );

    requireNear(
        points.back().downrangeM,
        secondOutput.downrangeM,
        1.0e-12,
        "Last point range must equal output range"
    );
}

} // namespace

int main() {
    try {
        testApiVersion();
        testResultCodeNames();
        testObjectList();
        testSummaryCalculation();
        testUnknownObject();
        testInvalidInput();
        testTrajectoryCalculation();

        std::cout
            << "All C API tests passed."
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "Test failure: "
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }
}