#include "passive_flight/ModelContract.hpp"
#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/PerturbedTrajectorySimulator.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

int failureCount = 0;

void check(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        ++failureCount;
    }
}

void checkNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message
) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr
            << "FAILED: " << message
            << "; expected " << expected
            << ", actual " << actual
            << ", tolerance " << tolerance
            << '\n';

        ++failureCount;
    }
}

passive_flight::PerturbedTrajectorySimulator
makeSimulator() {
    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    return passive_flight::PerturbedTrajectorySimulator(
        passport.object
    );
}

passive_flight::SimulationRequest makeRequest() {
    return {
        "ABSTRACT_500_UMPK_V1",
        {100.0, 200.0}
    };
}

passive_flight::SimulationOptions makeShortOptions() {
    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 0.01;
    options.maximumSteps = 100;
    options.saveHistory = true;
    options.historyStride = 1;

    return options;
}

void testZeroPerturbationReproducesNominalTrajectory() {
    const auto simulator = makeSimulator();

    passive_flight::SimulationOptions options;
    options.timeStepS = 0.001;
    options.maximumTimeS = 20.0;
    options.maximumSteps = 100'000;
    options.saveHistory = true;
    options.historyStride = 100;

    const auto result = simulator.simulate(
        makeRequest(),
        {},
        options
    );

    check(
        result.terminationReason ==
            passive_flight::TerminationReason::GroundReached,
        "Zero-perturbation run reaches the ground"
    );

    check(
        !result.history.empty(),
        "Zero-perturbation history is saved"
    );

    checkNear(
        result.finalPerturbation.speedMps,
        0.0,
        0.0,
        "Final Delta V remains zero"
    );

    checkNear(
        result.finalPerturbation.altitudeM,
        0.0,
        0.0,
        "Final Delta H remains zero"
    );

    checkNear(
        result.finalTotalState.downrangeM,
        result.finalNominalState.downrangeM,
        0.0,
        "Total and nominal downrange are equal"
    );

    checkNear(
        result.finalTotalState.speedMps,
        result.finalNominalState.speedMps,
        0.0,
        "Total and nominal speed are equal"
    );
}

void testInitialPerturbationIsStoredAndEvolves() {
    const auto simulator = makeSimulator();
    const auto options = makeShortOptions();

    passive_flight::LongitudinalPerturbationState initial;
    initial.speedMps = 1.0;
    initial.pitchAngleRad = 1.0e-3;
    initial.altitudeM = 2.0;

    const auto result = simulator.simulate(
        makeRequest(),
        initial,
        options
    );

    check(
        result.terminationReason ==
            passive_flight::TerminationReason::MaximumTimeReached,
        "Short perturbed run reaches maximum time"
    );

    check(
        result.history.size() == 11,
        "Short run contains initial point and ten steps"
    );

    const auto& first = result.history.front();

    checkNear(
        first.perturbation.speedMps,
        initial.speedMps,
        0.0,
        "Initial Delta V is stored"
    );

    checkNear(
        first.totalState.speedMps,
        first.nominalState.speedMps +
            initial.speedMps,
        0.0,
        "Initial total speed is composed correctly"
    );

    check(
        std::abs(
            result.finalPerturbation.pitchRateRadps -
            initial.pitchRateRadps
        ) > 1.0e-12,
        "Pitch-rate perturbation evolves"
    );

    checkNear(
        result.finalTotalState.altitudeM,
        result.finalNominalState.altitudeM +
            result.finalPerturbation.altitudeM,
        1.0e-12,
        "Final total altitude is composed correctly"
    );
}

void testOneStepMatchesExplicitEulerFormula() {
    const auto simulator = makeSimulator();

    auto options = makeShortOptions();
    options.maximumTimeS = options.timeStepS;

    passive_flight::LongitudinalPerturbationState initial;
    initial.speedMps = 0.5;
    initial.flightPathAngleRad = 1.0e-4;
    initial.pitchAngleRad = 2.0e-4;

    const passive_flight::State nominalInitial =
        passive_flight::makeHorizontalReleaseState(
            makeRequest().release
        );

    const auto derivative =
        simulator.perturbationDynamics()
            .evaluate(
                nominalInitial,
                initial
            )
            .derivative;

    const auto result = simulator.simulate(
        makeRequest(),
        initial,
        options
    );

    checkNear(
        result.finalPerturbation.speedMps,
        initial.speedMps +
            options.timeStepS * derivative.speedMps2,
        1.0e-12,
        "One-step Delta V Euler formula"
    );

    checkNear(
        result.finalPerturbation.pitchRateRadps,
        initial.pitchRateRadps +
            options.timeStepS *
                derivative.pitchRateRadps2,
        1.0e-12,
        "One-step Delta omega-z Euler formula"
    );

    checkNear(
        result.finalPerturbation.downrangeM,
        initial.downrangeM +
            options.timeStepS *
                derivative.downrangeMps,
        1.0e-12,
        "One-step Delta x Euler formula"
    );
}

void testRequestedHistoryStrideIsApplied() {
    const auto simulator = makeSimulator();

    auto options = makeShortOptions();
    options.historyStride = 4;

    const auto result = simulator.simulate(
        makeRequest(),
        {},
        options
    );

    check(
        result.history.size() == 4,
        "History contains initial, stride points and final point"
    );

    checkNear(
        result.history[0].nominalState.timeS,
        0.0,
        1.0e-15,
        "History initial time"
    );

    checkNear(
        result.history[1].nominalState.timeS,
        0.004,
        1.0e-12,
        "First stride time"
    );

    checkNear(
        result.history[2].nominalState.timeS,
        0.008,
        1.0e-12,
        "Second stride time"
    );

    checkNear(
        result.history[3].nominalState.timeS,
        0.01,
        1.0e-12,
        "Final history time"
    );
}

void testInvalidPerturbationIsRejected() {
    const auto simulator = makeSimulator();

    passive_flight::LongitudinalPerturbationState initial;
    initial.speedMps =
        std::numeric_limits<double>::quiet_NaN();

    const auto result = simulator.simulate(
        makeRequest(),
        initial,
        makeShortOptions()
    );

    check(
        result.terminationReason ==
            passive_flight::TerminationReason::InvalidInput,
        "Non-finite initial perturbation is rejected"
    );

    check(
        result.history.empty(),
        "Invalid perturbation does not create history"
    );
}

void testHistoryCanBeDisabled() {
    const auto simulator = makeSimulator();

    auto options = makeShortOptions();
    options.saveHistory = false;

    passive_flight::LongitudinalPerturbationState initial;
    initial.speedMps = 0.5;

    const auto result = simulator.simulate(
        makeRequest(),
        initial,
        options
    );

    check(
        result.history.empty(),
        "Public perturbation history can be disabled"
    );

    check(
        std::isfinite(
            result.finalPerturbation.speedMps
        ),
        "Final perturbation is available without history"
    );
}

void testZeroHistoryStrideIsRejected() {
    const auto simulator = makeSimulator();

    auto options = makeShortOptions();
    options.historyStride = 0;

    const auto result = simulator.simulate(
        makeRequest(),
        {},
        options
    );

    check(
        result.terminationReason ==
            passive_flight::TerminationReason::InvalidInput,
        "Zero history stride is rejected"
    );

    check(
        result.history.empty(),
        "Invalid history stride does not create history"
    );
}

} // namespace

int main() {
    testZeroPerturbationReproducesNominalTrajectory();
    testInitialPerturbationIsStoredAndEvolves();
    testOneStepMatchesExplicitEulerFormula();
    testRequestedHistoryStrideIsApplied();
    testInvalidPerturbationIsRejected();
    testHistoryCanBeDisabled();
    testZeroHistoryStrideIsRejected();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " perturbed trajectory simulator test(s) failed\n";

        return 1;
    }

    std::cout
        << "All perturbed trajectory simulator tests passed\n";

    return 0;
}
