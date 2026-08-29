#include "passive_flight/PerturbedImpactAnalysis.hpp"

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
        std::cerr
            << "FAILED: "
            << message
            << '\n';

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

passive_flight::PerturbedSimulationResult
makeGroundReachedResult() {
    passive_flight::PerturbedSimulationResult result;

    result.terminationReason =
        passive_flight::
            TerminationReason::GroundReached;

    result.finalNominalState.timeS = 10.0;
    result.finalNominalState.speedMps = 150.0;

    result.finalNominalState.flightPathAngleRad =
        -0.5;

    result.finalNominalState.pitchRateRadps =
        0.1;

    result.finalNominalState.pitchAngleRad =
        -0.4;

    result.finalNominalState.downrangeM =
        1000.0;

    result.finalNominalState.altitudeM =
        0.0;

    result.finalPerturbation.speedMps =
        2.0;

    result.finalPerturbation.flightPathAngleRad =
        0.02;

    result.finalPerturbation.pitchRateRadps =
        0.03;

    result.finalPerturbation.pitchAngleRad =
        0.04;

    result.finalPerturbation.downrangeM =
        5.0;

    result.finalPerturbation.altitudeM =
        5.0;

    return result;
}

passive_flight::StateDerivative
makeFinalNominalDerivative() {
    passive_flight::StateDerivative derivative;

    derivative.speedMps2 =
        -4.0;

    derivative.flightPathAngleRadps =
        -0.2;

    derivative.pitchRateRadps2 =
        0.4;

    derivative.pitchAngleRadps =
        0.1;

    derivative.downrangeMps =
        100.0;

    derivative.altitudeMps =
        -20.0;

    return derivative;
}

void testImpactEventCorrection() {
    const auto result =
        makeGroundReachedResult();

    const auto derivative =
        makeFinalNominalDerivative();

    const auto analysis =
        passive_flight::analyzePerturbedImpact(
            result,
            derivative
        );

    check(
        analysis.available,
        "Impact analysis is available"
    );

    /*
     * Delta t_f =
     *     -5 / -20 =
     *     0.25 s.
     */
    checkNear(
        analysis.changes.fallTimeS,
        0.25,
        1.0e-12,
        "Impact-time change"
    );

    checkNear(
        analysis.nominalVerticalSpeedMps,
        -20.0,
        1.0e-12,
        "Nominal vertical speed at impact"
    );

    /*
     * Delta V_impact =
     *     2 + (-4) * 0.25 =
     *     1.
     */
    checkNear(
        analysis.changes.speedMps,
        1.0,
        1.0e-12,
        "Impact-speed change"
    );

    /*
     * Delta Theta_impact =
     *     0.02 + (-0.2) * 0.25 =
     *     -0.03.
     */
    checkNear(
        analysis.changes.flightPathAngleRad,
        -0.03,
        1.0e-12,
        "Impact flight-path-angle change"
    );

    /*
     * Delta omega_z,impact =
     *     0.03 + 0.4 * 0.25 =
     *     0.13.
     */
    checkNear(
        analysis.changes.pitchRateRadps,
        0.13,
        1.0e-12,
        "Impact pitch-rate change"
    );

    /*
     * Delta theta_impact =
     *     0.04 + 0.1 * 0.25 =
     *     0.065.
     */
    checkNear(
        analysis.changes.pitchAngleRad,
        0.065,
        1.0e-12,
        "Impact pitch-angle change"
    );

    /*
     * Delta L =
     *     5 + 100 * 0.25 =
     *     30 m.
     */
    checkNear(
        analysis.changes.downrangeM,
        30.0,
        1.0e-12,
        "Impact downrange change"
    );

    checkNear(
        analysis.changes.angleOfAttackRad(),
        0.095,
        1.0e-12,
        "Impact angle-of-attack change"
    );

    checkNear(
        analysis.estimatedImpactState.timeS,
        10.25,
        1.0e-12,
        "Estimated perturbed impact time"
    );

    checkNear(
        analysis.estimatedImpactState.downrangeM,
        1030.0,
        1.0e-12,
        "Estimated perturbed impact downrange"
    );

    checkNear(
        analysis.estimatedImpactState.speedMps,
        151.0,
        1.0e-12,
        "Estimated perturbed impact speed"
    );

    checkNear(
        analysis.estimatedImpactState.altitudeM,
        0.0,
        0.0,
        "Estimated impact remains on ground"
    );
}

void testEarlierImpactHasNegativeTimeChange() {
    auto result =
        makeGroundReachedResult();

    result.finalPerturbation.altitudeM =
        -2.0;

    const auto analysis =
        passive_flight::analyzePerturbedImpact(
            result,
            makeFinalNominalDerivative()
        );

    check(
        analysis.available,
        "Earlier-impact analysis is available"
    );

    checkNear(
        analysis.changes.fallTimeS,
        -0.1,
        1.0e-12,
        "Negative Delta H produces earlier impact"
    );
}

void testRequiresGroundReachedNominalTrajectory() {
    auto result =
        makeGroundReachedResult();

    result.terminationReason =
        passive_flight::
            TerminationReason::MaximumTimeReached;

    const auto analysis =
        passive_flight::analyzePerturbedImpact(
            result,
            makeFinalNominalDerivative()
        );

    check(
        !analysis.available,
        "Impact analysis requires GroundReached"
    );
}

void testRejectsDegenerateVerticalSpeed() {
    const auto result =
        makeGroundReachedResult();

    auto derivative =
        makeFinalNominalDerivative();

    derivative.altitudeMps =
        0.0;

    const auto analysis =
        passive_flight::analyzePerturbedImpact(
            result,
            derivative
        );

    check(
        !analysis.available,
        "Zero nominal vertical speed disables analysis"
    );
}

void testZeroPerturbationKeepsNominalImpact() {
    auto result =
        makeGroundReachedResult();

    result.finalPerturbation = {};

    const auto analysis =
        passive_flight::analyzePerturbedImpact(
            result,
            makeFinalNominalDerivative()
        );

    check(
        analysis.available,
        "Zero-perturbation analysis is available"
    );

    checkNear(
        analysis.changes.fallTimeS,
        0.0,
        0.0,
        "Zero perturbation has zero time change"
    );

    checkNear(
        analysis.changes.downrangeM,
        0.0,
        0.0,
        "Zero perturbation has zero range change"
    );

    checkNear(
        analysis.estimatedImpactState.timeS,
        result.finalNominalState.timeS,
        0.0,
        "Zero perturbation preserves impact time"
    );
}

} // namespace

int main() {
    testImpactEventCorrection();
    testEarlierImpactHasNegativeTimeChange();
    testRequiresGroundReachedNominalTrajectory();
    testRejectsDegenerateVerticalSpeed();
    testZeroPerturbationKeepsNominalImpact();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " perturbed-impact-analysis "
               "test(s) failed\n";

        return 1;
    }

    std::cout
        << "All perturbed-impact-analysis "
           "tests passed\n";

    return 0;
}