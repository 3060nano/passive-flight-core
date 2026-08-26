#include "passive_flight/ModelContract.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using passive_flight::ReleaseConditions;
using passive_flight::SimulationRequest;
using passive_flight::SimulationResult;
using passive_flight::State;
using passive_flight::TerminationReason;
using passive_flight::ValidationIssues;

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
    const std::string& name
) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(
            name
            + ": actual="
            + std::to_string(actual)
            + ", expected="
            + std::to_string(expected)
        );
    }
}

void testValidReleaseConditions() {
    const ReleaseConditions release{
        10'000.0,
        300.0
    };

    const ValidationIssues issues =
        passive_flight::validate(release);

    require(
        passive_flight::isValid(issues),
        "Valid release conditions were rejected"
    );
}

void testInvalidAltitude() {
    const ReleaseConditions release{
        0.0,
        300.0
    };

    const ValidationIssues issues =
        passive_flight::validate(release);

    require(
        !passive_flight::isValid(issues),
        "Zero altitude must be rejected"
    );

    require(
        issues.size() == 1,
        "Exactly one altitude validation issue expected"
    );

    require(
        issues.front().field == "release.altitudeM",
        "Altitude validation field is incorrect"
    );
}

void testInvalidSpeed() {
    const ReleaseConditions release{
        10'000.0,
        -10.0
    };

    const ValidationIssues issues =
        passive_flight::validate(release);

    require(
        !passive_flight::isValid(issues),
        "Negative speed must be rejected"
    );

    require(
        issues.size() == 1,
        "Exactly one speed validation issue expected"
    );

    require(
        issues.front().field == "release.speedMps",
        "Speed validation field is incorrect"
    );
}

void testNonFiniteInput() {
    const double notANumber =
        std::numeric_limits<double>::quiet_NaN();

    const ReleaseConditions release{
        notANumber,
        notANumber
    };

    const ValidationIssues issues =
        passive_flight::validate(release);

    require(
        issues.size() == 2,
        "Two validation issues expected for NaN input"
    );
}

void testEmptyObjectIdentifier() {
    SimulationRequest request;

    request.objectId = "";
    request.release.altitudeM = 10'000.0;
    request.release.speedMps = 300.0;

    const ValidationIssues issues =
        passive_flight::validate(request);

    require(
        !passive_flight::isValid(issues),
        "Empty object identifier must be rejected"
    );

    require(
        issues.size() == 1,
        "Exactly one object identifier issue expected"
    );

    require(
        issues.front().field == "objectId",
        "Object identifier validation field is incorrect"
    );
}

void testHorizontalReleaseState() {
    const ReleaseConditions release{
        8'000.0,
        250.0
    };

    const State state =
        passive_flight::makeHorizontalReleaseState(release);

    requireNear(
        state.timeS,
        0.0,
        0.0,
        "Initial time"
    );

    requireNear(
        state.speedMps,
        250.0,
        0.0,
        "Initial speed"
    );

    requireNear(
        state.altitudeM,
        8'000.0,
        0.0,
        "Initial altitude"
    );

    requireNear(
        state.downrangeM,
        0.0,
        0.0,
        "Initial downrange"
    );

    requireNear(
        state.flightPathAngleRad,
        0.0,
        0.0,
        "Initial flight-path angle"
    );

    requireNear(
        state.pitchAngleRad,
        0.0,
        0.0,
        "Initial pitch angle"
    );

    requireNear(
        state.pitchRateRadps,
        0.0,
        0.0,
        "Initial pitch rate"
    );

    requireNear(
        state.angleOfAttackRad(),
        0.0,
        0.0,
        "Initial angle of attack"
    );
}

void testInvalidReleaseStateThrows() {
    const ReleaseConditions release{
        -100.0,
        200.0
    };

    bool exceptionThrown = false;

    try {
        [[maybe_unused]]
        const State state =
            passive_flight::makeHorizontalReleaseState(release);
    } catch (const std::invalid_argument&) {
        exceptionThrown = true;
    }

    require(
        exceptionThrown,
        "Invalid release must throw std::invalid_argument"
    );
}

void testSimulationSummary() {
    SimulationResult result;

    result.terminationReason =
        TerminationReason::GroundReached;

    result.finalState.timeS = 25.0;
    result.finalState.downrangeM = 4'500.0;
    result.finalState.speedMps = 280.0;
    result.finalState.flightPathAngleRad = -0.75;
    result.finalState.pitchAngleRad = -0.60;

    const passive_flight::SimulationSummary summary =
        passive_flight::summarize(result);

    requireNear(
        summary.downrangeM,
        4'500.0,
        0.0,
        "Summary downrange"
    );

    requireNear(
        summary.fallTimeS,
        25.0,
        0.0,
        "Summary fall time"
    );

    requireNear(
        summary.impactSpeedMps,
        280.0,
        0.0,
        "Summary impact speed"
    );

    requireNear(
        summary.impactAngleOfAttackRad,
        0.15,
        1.0e-12,
        "Summary impact angle of attack"
    );

    require(
        summary.terminationReason
            == TerminationReason::GroundReached,
        "Summary termination reason is incorrect"
    );
}

void testTerminationReasonNames() {
    require(
        std::string(
            passive_flight::terminationReasonName(
                TerminationReason::GroundReached
            )
        ) == "ground_reached",
        "GroundReached name is incorrect"
    );

    require(
        std::string(
            passive_flight::terminationReasonName(
                TerminationReason::InvalidInput
            )
        ) == "invalid_input",
        "InvalidInput name is incorrect"
    );
}

} // namespace

int main() {
    try {
        testValidReleaseConditions();
        testInvalidAltitude();
        testInvalidSpeed();
        testNonFiniteInput();
        testEmptyObjectIdentifier();
        testHorizontalReleaseState();
        testInvalidReleaseStateThrows();
        testSimulationSummary();
        testTerminationReasonNames();

        std::cout
            << "All model contract tests passed."
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