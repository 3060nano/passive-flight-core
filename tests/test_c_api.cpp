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
        throw std::runtime_error(
            message
            + ": actual="
            + std::to_string(actual)
            + ", expected="
            + std::to_string(expected)
        );
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

PFSimulationInput makeFab1500TInput() {
    PFSimulationInput input{};

    input.objectId =
        "FAB_1500T_POSTNIKOV_1979";

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
            "1.1.0",
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
        pfGetObjectCount() == 2,
        "C API must expose two objects"
    );

    require(
        getObjectId(0) ==
            "ABSTRACT_500_UMPK_V1",
        "First C API object ID is incorrect"
    );

    require(
        !getObjectDisplayName(0).empty(),
        "First C API object name is empty"
    );

    require(
        getObjectId(1) ==
            "FAB_1500T_POSTNIKOV_1979",
        "Second C API object ID is incorrect"
    );

    require(
        !getObjectDisplayName(1).empty(),
        "Second C API object name is empty"
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

void testFab1500TSummaryCalculation() {
    const PFSimulationInput input =
        makeFab1500TInput();

    PFSimulationOutput output{};

    const int32_t result =
        pfCalculate(
            &input,
            &output
        );

    require(
        result == PF_RESULT_OK,
        "FAB-1500T summary calculation failed"
    );

    require(
        output.terminationReason ==
            PF_TERMINATION_GROUND_REACHED,
        "FAB-1500T simulation did not reach ground"
    );

    /*
     * Эти интервалы являются sanity-check,
     * а не сравнением с баллистической таблицей.
     * Они фиксируют согласованность публичного
     * C API с уже проверенным расчётом ядра.
     */
    require(
        output.downrangeM > 800.0 &&
        output.downrangeM < 1000.0,
        "FAB-1500T downrange is outside sanity range"
    );

    require(
        output.fallTimeS > 4.0 &&
        output.fallTimeS < 5.0,
        "FAB-1500T fall time is outside sanity range"
    );

    require(
        output.impactSpeedMps > 180.0 &&
        output.impactSpeedMps < 220.0,
        "FAB-1500T impact speed is outside sanity range"
    );

    require(
        output.impactFlightPathAngleRad < 0.0,
        "FAB-1500T impact trajectory must point downward"
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

void testFab1500TTrajectoryCalculation() {
    const PFSimulationInput input =
        makeFab1500TInput();

    PFSimulationOutput output{};

    uint64_t requiredPointCount = 0;
    uint64_t writtenPointCount = 0;

    const int32_t queryResult =
        pfCalculateTrajectory(
            &input,
            &output,
            nullptr,
            0,
            &requiredPointCount,
            &writtenPointCount
        );

    require(
        queryResult ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "FAB-1500T trajectory size query failed"
    );

    require(
        requiredPointCount > 1,
        "FAB-1500T trajectory must contain several points"
    );

    std::vector<PFTrajectoryPoint> points(
        static_cast<std::size_t>(
            requiredPointCount
        )
    );

    const int32_t calculationResult =
        pfCalculateTrajectory(
            &input,
            &output,
            points.data(),
            requiredPointCount,
            &requiredPointCount,
            &writtenPointCount
        );

    require(
        calculationResult ==
            PF_RESULT_OK,
        "FAB-1500T trajectory calculation failed"
    );

    require(
        writtenPointCount ==
            requiredPointCount,
        "Incorrect FAB-1500T trajectory point count"
    );

    requireNear(
        points.front().timeS,
        0.0,
        1.0e-12,
        "FAB-1500T trajectory must start at t=0"
    );

    requireNear(
        points.front().altitudeM,
        input.releaseAltitudeM,
        1.0e-12,
        "FAB-1500T initial altitude is incorrect"
    );

    requireNear(
        points.back().altitudeM,
        0.0,
        1.0e-12,
        "FAB-1500T trajectory must end at ground"
    );

    requireNear(
        points.back().downrangeM,
        output.downrangeM,
        1.0e-12,
        "FAB-1500T final trajectory range differs from summary"
    );
}

void testSimInTechSummaryAdapter() {
    const PFSimulationInput input =
        makeInput();

    PFSimulationOutput referenceOutput{};

    const int32_t referenceResult =
        pfCalculate(
            &input,
            &referenceOutput
        );

    require(
        referenceResult == PF_RESULT_OK,
        "Reference calculation failed"
    );

    double downrangeM = 0.0;
    double fallTimeS = 0.0;

    double impactSpeedMps = 0.0;
    double impactFlightPathAngleRad = 0.0;
    double impactPitchAngleRad = 0.0;
    double impactAngleOfAttackRad = 0.0;

    int32_t terminationReason =
        PF_TERMINATION_INVALID_STATE;

    const int32_t adapterResult =
        pfSimInTechCalculate(
            input.objectId,
            input.releaseAltitudeM,
            input.releaseSpeedMps,
            &downrangeM,
            &fallTimeS,
            &impactSpeedMps,
            &impactFlightPathAngleRad,
            &impactPitchAngleRad,
            &impactAngleOfAttackRad,
            &terminationReason
        );

    require(
        adapterResult == PF_RESULT_OK,
        "SimInTech summary calculation failed"
    );

    requireNear(
        downrangeM,
        referenceOutput.downrangeM,
        1.0e-12,
        "SimInTech downrange differs"
    );

    requireNear(
        fallTimeS,
        referenceOutput.fallTimeS,
        1.0e-12,
        "SimInTech fall time differs"
    );

    requireNear(
        impactSpeedMps,
        referenceOutput.impactSpeedMps,
        1.0e-12,
        "SimInTech impact speed differs"
    );

    requireNear(
        impactFlightPathAngleRad,
        referenceOutput
            .impactFlightPathAngleRad,
        1.0e-12,
        "SimInTech trajectory angle differs"
    );

    requireNear(
        impactPitchAngleRad,
        referenceOutput.impactPitchAngleRad,
        1.0e-12,
        "SimInTech pitch angle differs"
    );

    requireNear(
        impactAngleOfAttackRad,
        referenceOutput.impactAngleOfAttackRad,
        1.0e-12,
        "SimInTech angle of attack differs"
    );

    require(
        terminationReason ==
            referenceOutput.terminationReason,
        "SimInTech termination reason differs"
    );
}

void testFab1500TSimInTechSummaryAdapter() {
    const PFSimulationInput input =
        makeFab1500TInput();

    PFSimulationOutput referenceOutput{};

    const int32_t referenceResult =
        pfCalculate(
            &input,
            &referenceOutput
        );

    require(
        referenceResult == PF_RESULT_OK,
        "FAB-1500T reference calculation failed"
    );

    double downrangeM = 0.0;
    double fallTimeS = 0.0;
    double impactSpeedMps = 0.0;
    double impactFlightPathAngleRad = 0.0;
    double impactPitchAngleRad = 0.0;
    double impactAngleOfAttackRad = 0.0;

    int32_t terminationReason =
        PF_TERMINATION_INVALID_STATE;

    const int32_t adapterResult =
        pfSimInTechCalculate(
            input.objectId,
            input.releaseAltitudeM,
            input.releaseSpeedMps,
            &downrangeM,
            &fallTimeS,
            &impactSpeedMps,
            &impactFlightPathAngleRad,
            &impactPitchAngleRad,
            &impactAngleOfAttackRad,
            &terminationReason
        );

    require(
        adapterResult == PF_RESULT_OK,
        "FAB-1500T SimInTech summary calculation failed"
    );

    requireNear(
        downrangeM,
        referenceOutput.downrangeM,
        1.0e-12,
        "FAB-1500T SimInTech downrange differs"
    );

    requireNear(
        fallTimeS,
        referenceOutput.fallTimeS,
        1.0e-12,
        "FAB-1500T SimInTech fall time differs"
    );

    requireNear(
        impactSpeedMps,
        referenceOutput.impactSpeedMps,
        1.0e-12,
        "FAB-1500T SimInTech impact speed differs"
    );

    requireNear(
        impactFlightPathAngleRad,
        referenceOutput
            .impactFlightPathAngleRad,
        1.0e-12,
        "FAB-1500T SimInTech trajectory angle differs"
    );

    requireNear(
        impactPitchAngleRad,
        referenceOutput.impactPitchAngleRad,
        1.0e-12,
        "FAB-1500T SimInTech pitch angle differs"
    );

    requireNear(
        impactAngleOfAttackRad,
        referenceOutput.impactAngleOfAttackRad,
        1.0e-12,
        "FAB-1500T SimInTech angle of attack differs"
    );

    require(
        terminationReason ==
            referenceOutput.terminationReason,
        "FAB-1500T SimInTech termination reason differs"
    );
}

void testSimInTechSummaryValidation() {
    double downrangeM = 0.0;
    double fallTimeS = 0.0;

    double impactSpeedMps = 0.0;
    double impactFlightPathAngleRad = 0.0;
    double impactPitchAngleRad = 0.0;
    double impactAngleOfAttackRad = 0.0;

    int32_t terminationReason =
        PF_TERMINATION_INVALID_STATE;

    const int32_t nullOutputResult =
        pfSimInTechCalculate(
            "ABSTRACT_500_UMPK_V1",
            100.0,
            200.0,
            nullptr,
            &fallTimeS,
            &impactSpeedMps,
            &impactFlightPathAngleRad,
            &impactPitchAngleRad,
            &impactAngleOfAttackRad,
            &terminationReason
        );

    require(
        nullOutputResult ==
            PF_RESULT_NULL_ARGUMENT,
        "Null SimInTech output must be rejected"
    );

    const int32_t unknownObjectResult =
        pfSimInTechCalculate(
            "UNKNOWN_OBJECT",
            100.0,
            200.0,
            &downrangeM,
            &fallTimeS,
            &impactSpeedMps,
            &impactFlightPathAngleRad,
            &impactPitchAngleRad,
            &impactAngleOfAttackRad,
            &terminationReason
        );

    require(
        unknownObjectResult ==
            PF_RESULT_OBJECT_NOT_FOUND,
        "Unknown SimInTech object must be rejected"
    );

    const int32_t invalidSpeedResult =
        pfSimInTechCalculate(
            "ABSTRACT_500_UMPK_V1",
            100.0,
            0.0,
            &downrangeM,
            &fallTimeS,
            &impactSpeedMps,
            &impactFlightPathAngleRad,
            &impactPitchAngleRad,
            &impactAngleOfAttackRad,
            &terminationReason
        );

    require(
        invalidSpeedResult ==
            PF_RESULT_INVALID_INPUT,
        "Invalid SimInTech speed must be rejected"
    );
}

void testSimInTechTrajectoryAdapter() {
    const PFSimulationInput input =
        makeInput();

    uint64_t requiredPointCount = 0;
    uint64_t writtenPointCount = 0;

    const int32_t queryResult =
        pfSimInTechCalculateTrajectory(
            input.objectId,
            input.releaseAltitudeM,
            input.releaseSpeedMps,
            nullptr,
            nullptr,
            nullptr,
            0,
            &requiredPointCount,
            &writtenPointCount
        );

    require(
        queryResult ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "SimInTech trajectory size query failed"
    );

    require(
        requiredPointCount > 1,
        "SimInTech trajectory is empty"
    );

    require(
        writtenPointCount == 0,
        "Trajectory query must not write points"
    );

    std::vector<double> timeS(
        static_cast<std::size_t>(
            requiredPointCount
        )
    );

    std::vector<double> downrangeM(
        static_cast<std::size_t>(
            requiredPointCount
        )
    );

    std::vector<double> altitudeM(
        static_cast<std::size_t>(
            requiredPointCount
        )
    );

    const int32_t calculationResult =
        pfSimInTechCalculateTrajectory(
            input.objectId,
            input.releaseAltitudeM,
            input.releaseSpeedMps,
            timeS.data(),
            downrangeM.data(),
            altitudeM.data(),
            requiredPointCount,
            &requiredPointCount,
            &writtenPointCount
        );

    require(
        calculationResult ==
            PF_RESULT_OK,
        "SimInTech trajectory calculation failed"
    );

    require(
        writtenPointCount ==
            requiredPointCount,
        "Incorrect SimInTech trajectory size"
    );

    requireNear(
        timeS.front(),
        0.0,
        1.0e-12,
        "SimInTech trajectory must start at t=0"
    );

    requireNear(
        downrangeM.front(),
        0.0,
        1.0e-12,
        "SimInTech trajectory must start at x=0"
    );

    requireNear(
        altitudeM.front(),
        input.releaseAltitudeM,
        1.0e-12,
        "Incorrect initial trajectory altitude"
    );

    require(
        timeS.back() > 0.0,
        "Final trajectory time must be positive"
    );

    require(
        downrangeM.back() > 0.0,
        "Final trajectory range must be positive"
    );

    requireNear(
        altitudeM.back(),
        0.0,
        1.0e-12,
        "SimInTech trajectory must end at ground"
    );

    for (std::size_t index = 1;
         index < timeS.size();
         ++index) {
        require(
            timeS[index] > timeS[index - 1],
            "Trajectory time must increase"
        );

        require(
            downrangeM[index] >
                downrangeM[index - 1],
            "Trajectory downrange must increase"
        );
    }
}

void testSimInTechTrajectoryValidation() {
    uint64_t requiredPointCount = 0;
    uint64_t writtenPointCount = 0;

    double timeS = 0.0;

    const int32_t partialArraysResult =
        pfSimInTechCalculateTrajectory(
            "ABSTRACT_500_UMPK_V1",
            100.0,
            200.0,
            &timeS,
            nullptr,
            nullptr,
            1,
            &requiredPointCount,
            &writtenPointCount
        );

    require(
        partialArraysResult ==
            PF_RESULT_NULL_ARGUMENT,
        "Partial trajectory arrays must be rejected"
    );

    const int32_t nullCounterResult =
        pfSimInTechCalculateTrajectory(
            "ABSTRACT_500_UMPK_V1",
            100.0,
            200.0,
            nullptr,
            nullptr,
            nullptr,
            0,
            nullptr,
            &writtenPointCount
        );

    require(
        nullCounterResult ==
            PF_RESULT_NULL_ARGUMENT,
        "Null trajectory counter must be rejected"
    );
}

} // namespace

int main() {
    try {
        testApiVersion();
        testResultCodeNames();
        testObjectList();

        testSummaryCalculation();
        testFab1500TSummaryCalculation();
        testUnknownObject();
        testInvalidInput();

        testTrajectoryCalculation();
        testFab1500TTrajectoryCalculation();

        testSimInTechSummaryAdapter();
        testFab1500TSimInTechSummaryAdapter();
        testSimInTechSummaryValidation();

        testSimInTechTrajectoryAdapter();
        testSimInTechTrajectoryValidation();

        std::cout
            << "All C API and SimInTech adapter tests passed."
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
