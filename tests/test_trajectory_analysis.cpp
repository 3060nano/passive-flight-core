#include "passive_flight/ForwardEulerSimulator.hpp"
#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/TrajectoryAnalysis.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

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

passive_flight::SimulationResult runTestSimulation(
    const passive_flight::ObjectModel& object
) {
    const passive_flight::ForwardEulerSimulator simulator(
        object
    );

    passive_flight::SimulationRequest request;

    request.objectId = object.id;
    request.release.altitudeM = 100.0;
    request.release.speedMps = 200.0;

    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 30.0;
    options.maximumSteps = 100000;
    options.groundAltitudeM = 0.0;

    options.saveHistory = true;
    options.historyStride = 50;

    return simulator.simulate(
        request,
        options
    );
}

void testEmptyHistory() {
    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const passive_flight::SimulationResult emptyResult;

    const auto analysis =
        passive_flight::analyzeTrajectory(
            emptyResult,
            passport.object
        );

    require(
        !analysis.available,
        "Empty trajectory must not be available"
    );

    require(
        analysis.sampleCount == 0,
        "Empty trajectory must contain zero samples"
    );
}

void testTrajectoryExtrema() {
    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const auto result =
        runTestSimulation(
            passport.object
        );

    require(
        result.terminationReason ==
            passive_flight::TerminationReason::GroundReached,
        "Test trajectory must reach ground"
    );

    const auto analysis =
        passive_flight::analyzeTrajectory(
            result,
            passport.object
        );

    require(
        analysis.available,
        "Trajectory analysis must be available"
    );

    require(
        analysis.sampleCount ==
            result.history.size(),
        "Sample count must match history size"
    );

    require(
        analysis.maximumAltitudeM >=
            analysis.releaseAltitudeM,
        "Maximum altitude must not be below release altitude"
    );

    require(
        analysis.maximumAngleOfAttackRad >=
            analysis.minimumAngleOfAttackRad,
        "Angle-of-attack extrema must be ordered"
    );

    require(
        analysis.maximumEffectiveWingAngleRad >=
            analysis.maximumAngleOfAttackRad,
        "Wing installation angle must be included"
    );

    require(
        analysis.maximumAbsolutePitchRateRadps >= 0.0,
        "Maximum absolute pitch rate must be non-negative"
    );

    require(
        analysis.minimumMach > 0.0,
        "Minimum Mach number must be positive"
    );

    require(
        analysis.maximumMach >=
            analysis.minimumMach,
        "Mach extrema must be ordered"
    );

    require(
        analysis.maximumDynamicPressurePa > 0.0,
        "Maximum dynamic pressure must be positive"
    );

    require(
        analysis.maximumLiftToDragRatio >=
            analysis.minimumLiftToDragRatio,
        "Lift-to-drag extrema must be ordered"
    );

    const auto& finalSample =
        result.history.back();

    const double expectedFinalRatio =
        finalSample.diagnostics.liftCoefficient /
        finalSample.diagnostics.dragCoefficient;

    requireNear(
        analysis.finalLiftToDragRatio,
        expectedFinalRatio,
        1.0e-12,
        "Final lift-to-drag ratio is incorrect"
    );

    require(
        analysis.angleOfAttackSettlingTimeS >= 0.0,
        "Settling time must be non-negative"
    );

    require(
        analysis.angleOfAttackSettlingTimeS <=
            result.finalState.timeS,
        "Settling time must not exceed flight time"
    );
}

void testAerodynamicBalance() {
    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const auto balance =
        passive_flight::analyzeAerodynamicBalance(
            passport.object,
            0.8
        );

    require(
        balance.available,
        "Balance analysis must be available"
    );

    require(
        balance.staticallyStable,
        "Baseline object must be statically stable"
    );

    require(
        balance.mzAlphaPerRad < 0.0,
        "Mz-alpha derivative must be negative"
    );

    require(
        balance.trimAngleOfAttackRad > 0.0,
        "Trim angle of attack must be positive"
    );

    require(
        balance.trimAngleOfAttackRad <
            10.0 *
            std::numbers::pi_v<double> /
            180.0,
        "Trim angle must remain in the small-angle range"
    );

    require(
        balance.trimEffectiveWingAngleRad >
            balance.trimAngleOfAttackRad,
        "Wing installation angle must increase effective angle"
    );

    require(
        balance.trimCx > 0.0,
        "Trim Cx must be positive"
    );

    require(
        balance.trimCy > 0.0,
        "Trim Cy must be positive"
    );

    require(
        balance.trimLiftToDragRatio > 0.0,
        "Trim lift-to-drag ratio must be positive"
    );
}

} // namespace

int main() {
    try {
        testEmptyHistory();
        testTrajectoryExtrema();
        testAerodynamicBalance();

        std::cout
            << "All trajectory analysis tests passed."
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