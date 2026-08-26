#include "passive_flight/ForwardEulerSimulator.hpp"
#include "passive_flight/ModelContract.hpp"
#include "passive_flight/ObjectPassport.hpp"

#include <cmath>
#include <iostream>
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

struct TestContext {
    passive_flight::ObjectPassport passport;
    passive_flight::ForwardEulerSimulator simulator;

    TestContext()
        : passport(
              passive_flight::makeAbstract500UmpkPassport()
          ),
          simulator(
              passport.object
          ) {
    }
};

passive_flight::SimulationRequest makeRequest(
    const TestContext& context,
    double altitudeM,
    double speedMps
) {
    passive_flight::SimulationRequest request;

    /*
     * Идентификатор находится непосредственно
     * в ObjectModel.
     */
    request.objectId =
        context.passport.object.id;

    request.release.altitudeM =
        altitudeM;

    request.release.speedMps =
        speedMps;

    return request;
}

void testGroundIsReached() {
    const TestContext context;

    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 30.0;
    options.maximumSteps = 100000;
    options.groundAltitudeM = 0.0;
    options.saveHistory = true;
    options.historyStride = 100;

    const auto result =
        context.simulator.simulate(
            makeRequest(
                context,
                100.0,
                200.0
            ),
            options
        );

    check(
        result.terminationReason ==
            passive_flight::TerminationReason::GroundReached,
        "Simulation reaches the ground"
    );

    checkNear(
        result.finalState.altitudeM,
        0.0,
        1.0e-12,
        "Final altitude equals ground altitude"
    );

    check(
        result.finalState.timeS > 0.0,
        "Fall time is positive"
    );

    check(
        result.finalState.timeS <
            options.maximumTimeS,
        "Ground is reached before maximum time"
    );

    check(
        result.finalState.downrangeM > 0.0,
        "Downrange is positive"
    );

    check(
        result.finalState.speedMps > 0.0,
        "Impact speed is positive"
    );

    check(
        result.finalState.flightPathAngleRad < 0.0,
        "Impact trajectory is directed downward"
    );

    check(
        !result.history.empty(),
        "Trajectory history is saved"
    );

    if (!result.history.empty()) {
        checkNear(
            result.history.front().state.timeS,
            0.0,
            1.0e-12,
            "History starts at release"
        );

        checkNear(
            result.history.front().state.altitudeM,
            100.0,
            1.0e-12,
            "History contains release altitude"
        );

        checkNear(
            result.history.back().state.altitudeM,
            0.0,
            1.0e-12,
            "History ends at ground"
        );

        checkNear(
            result.history.back().state.timeS,
            result.finalState.timeS,
            1.0e-12,
            "Last history time equals fall time"
        );
    }
}

void testSummary() {
    const TestContext context;

    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 30.0;
    options.maximumSteps = 100000;
    options.saveHistory = false;

    const auto result =
        context.simulator.simulate(
            makeRequest(
                context,
                100.0,
                200.0
            ),
            options
        );

    const auto summary =
        passive_flight::summarize(result);

    checkNear(
        summary.downrangeM,
        result.finalState.downrangeM,
        1.0e-12,
        "Summary contains final downrange"
    );

    checkNear(
        summary.fallTimeS,
        result.finalState.timeS,
        1.0e-12,
        "Summary contains fall time"
    );

    checkNear(
        summary.impactSpeedMps,
        result.finalState.speedMps,
        1.0e-12,
        "Summary contains impact speed"
    );

    checkNear(
        summary.impactAngleOfAttackRad,
        result.finalState.angleOfAttackRad(),
        1.0e-12,
        "Summary contains impact angle of attack"
    );

    check(
        summary.terminationReason ==
            result.terminationReason,
        "Summary contains termination reason"
    );
}

void testHistoryCanBeDisabled() {
    const TestContext context;

    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 30.0;
    options.maximumSteps = 100000;
    options.saveHistory = false;

    const auto result =
        context.simulator.simulate(
            makeRequest(
                context,
                100.0,
                200.0
            ),
            options
        );

    check(
        result.history.empty(),
        "History remains empty when disabled"
    );

    check(
        result.terminationReason ==
            passive_flight::TerminationReason::GroundReached,
        "Disabling history does not change termination"
    );
}

void testDeterministicResult() {
    const TestContext context;

    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 30.0;
    options.maximumSteps = 100000;
    options.saveHistory = false;

    const auto request =
        makeRequest(
            context,
            100.0,
            200.0
        );

    const auto first =
        context.simulator.simulate(
            request,
            options
        );

    const auto second =
        context.simulator.simulate(
            request,
            options
        );

    checkNear(
        first.finalState.timeS,
        second.finalState.timeS,
        1.0e-12,
        "Repeated simulations have equal fall time"
    );

    checkNear(
        first.finalState.downrangeM,
        second.finalState.downrangeM,
        1.0e-12,
        "Repeated simulations have equal downrange"
    );

    checkNear(
        first.finalState.speedMps,
        second.finalState.speedMps,
        1.0e-12,
        "Repeated simulations have equal impact speed"
    );
}

void testMaximumTimeTermination() {
    const TestContext context;

    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 0.005;
    options.maximumSteps = 1000;
    options.saveHistory = false;

    const auto result =
        context.simulator.simulate(
            makeRequest(
                context,
                10000.0,
                200.0
            ),
            options
        );

    check(
        result.terminationReason ==
            passive_flight::TerminationReason::MaximumTimeReached,
        "Maximum-time termination works"
    );

    checkNear(
        result.finalState.timeS,
        options.maximumTimeS,
        1.0e-12,
        "Simulation stops at maximum time"
    );

    check(
        result.finalState.altitudeM > 0.0,
        "Object remains above ground at maximum time"
    );
}

void testMaximumStepsTermination() {
    const TestContext context;

    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 300.0;
    options.maximumSteps = 2;
    options.saveHistory = false;

    const auto result =
        context.simulator.simulate(
            makeRequest(
                context,
                10000.0,
                200.0
            ),
            options
        );

    check(
        result.terminationReason ==
            passive_flight::TerminationReason::MaximumStepsReached,
        "Maximum-steps termination works"
    );

    checkNear(
        result.finalState.timeS,
        0.002,
        1.0e-12,
        "Two Euler steps are completed"
    );
}

void testInvalidInput() {
    const TestContext context;

    passive_flight::SimulationOptions options;

    auto wrongObjectRequest =
        makeRequest(
            context,
            1000.0,
            200.0
        );

    wrongObjectRequest.objectId =
        "UNKNOWN_OBJECT";

    const auto wrongObjectResult =
        context.simulator.simulate(
            wrongObjectRequest,
            options
        );

    check(
        wrongObjectResult.terminationReason ==
            passive_flight::TerminationReason::InvalidInput,
        "Unknown object identifier is rejected"
    );

    const auto zeroSpeedResult =
        context.simulator.simulate(
            makeRequest(
                context,
                1000.0,
                0.0
            ),
            options
        );

    check(
        zeroSpeedResult.terminationReason ==
            passive_flight::TerminationReason::InvalidInput,
        "Zero release speed is rejected"
    );

    auto invalidOptions = options;
    invalidOptions.timeStepS = 0.0;

    const auto invalidOptionsResult =
        context.simulator.simulate(
            makeRequest(
                context,
                1000.0,
                200.0
            ),
            invalidOptions
        );

    check(
        invalidOptionsResult.terminationReason ==
            passive_flight::TerminationReason::InvalidInput,
        "Zero Euler time step is rejected"
    );
}

} // namespace

int main() {
    testGroundIsReached();
    testSummary();
    testHistoryCanBeDisabled();
    testDeterministicResult();
    testMaximumTimeTermination();
    testMaximumStepsTermination();
    testInvalidInput();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " forward-Euler simulator test(s) failed"
            << '\n';

        return 1;
    }

    std::cout
        << "All forward-Euler simulator tests passed"
        << '\n';

    return 0;
}