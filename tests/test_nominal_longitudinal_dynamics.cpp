#include "passive_flight/NominalLongitudinalDynamics.hpp"
#include "passive_flight/ObjectPassport.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
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

template <typename ExceptionType, typename Function>
void checkThrows(
    Function function,
    const std::string& message
) {
    try {
        function();

        std::cerr
            << "FAILED: " << message
            << "; exception was not thrown"
            << '\n';

        ++failureCount;
    } catch (const ExceptionType&) {
        // Ожидаемое исключение.
    } catch (...) {
        std::cerr
            << "FAILED: " << message
            << "; unexpected exception type"
            << '\n';

        ++failureCount;
    }
}

passive_flight::NominalLongitudinalDynamics
makeDynamics() {
    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    return passive_flight::NominalLongitudinalDynamics(
        passport.object
    );
}

void testHorizontalReleaseKinematics() {
    const auto dynamics = makeDynamics();

    /*
     * Порядок State:
     *
     * time, V, Theta, omega_z, theta, x, H.
     */
    const passive_flight::State state{
        0.0,
        300.0,
        0.0,
        0.0,
        0.0,
        0.0,
        10000.0
    };

    const auto result =
        dynamics.evaluate(state);

    const auto& [
        speedDerivativeMps2,
        trajectoryAngleDerivativeRadS,
        pitchRateDerivativeRadS2,
        pitchAngleDerivativeRadS,
        downrangeDerivativeMps,
        altitudeDerivativeMps
    ] = result.derivative;

    check(
        speedDerivativeMps2 < 0.0,
        "Drag reduces speed after horizontal release"
    );

    check(
        trajectoryAngleDerivativeRadS < 0.0,
        "Gravity turns trajectory downward"
    );

    checkNear(
        pitchAngleDerivativeRadS,
        0.0,
        1.0e-12,
        "Initial pitch-angle rate equals zero"
    );

    checkNear(
        downrangeDerivativeMps,
        300.0,
        1.0e-10,
        "Initial horizontal velocity equals release speed"
    );

    checkNear(
        altitudeDerivativeMps,
        0.0,
        1.0e-12,
        "Initial vertical velocity equals zero"
    );

    check(
        std::isfinite(pitchRateDerivativeRadS2),
        "Pitch acceleration is finite"
    );

    checkNear(
        result.angleOfAttackRad,
        0.0,
        1.0e-12,
        "Initial angle of attack equals zero"
    );

    check(
        result.angleOfAttackRateRadS > 0.0,
        "Alpha starts increasing when trajectory turns downward"
    );
}

void testKinematicEquations() {
    const auto dynamics = makeDynamics();

    const double trajectoryAngleRad =
        -30.0 *
        std::numbers::pi_v<double> /
        180.0;

    const passive_flight::State state{
        5.0,
        200.0,
        trajectoryAngleRad,
        0.10,
        trajectoryAngleRad,
        1000.0,
        5000.0
    };

    const auto result =
        dynamics.evaluate(state);

    checkNear(
        result.derivative.pitchAngleRadps,
        0.10,
        1.0e-12,
        "Pitch-angle derivative equals pitch rate"
    );

    checkNear(
        result.derivative.downrangeMps,
        200.0 * std::cos(trajectoryAngleRad),
        1.0e-10,
        "Downrange kinematic equation"
    );

    checkNear(
        result.derivative.altitudeMps,
        200.0 * std::sin(trajectoryAngleRad),
        1.0e-10,
        "Altitude kinematic equation"
    );

    check(
        result.derivative.altitudeMps < 0.0,
        "Negative trajectory angle decreases altitude"
    );
}

void testAngleOfAttackDefinition() {
    const auto dynamics = makeDynamics();

    const double trajectoryAngleRad =
        -3.0 *
        std::numbers::pi_v<double> /
        180.0;

    const double pitchAngleRad =
        2.0 *
        std::numbers::pi_v<double> /
        180.0;

    const passive_flight::State state{
        0.0,
        250.0,
        trajectoryAngleRad,
        0.0,
        pitchAngleRad,
        0.0,
        8000.0
    };

    const auto result =
        dynamics.evaluate(state);

    const double expectedAlphaRad =
        5.0 *
        std::numbers::pi_v<double> /
        180.0;

    checkNear(
        result.angleOfAttackRad,
        expectedAlphaRad,
        1.0e-12,
        "Alpha equals pitch angle minus trajectory angle"
    );

    check(
        result.aerodynamics.cy > 0.0,
        "Positive alpha produces positive Cy"
    );

    check(
        result.loads.normalForceN > 0.0,
        "Positive Cy produces positive normal force"
    );
}

void testDimensionedLoads() {
    const auto dynamics = makeDynamics();

    const passive_flight::State state{
        0.0,
        300.0,
        0.0,
        0.0,
        0.0,
        0.0,
        10000.0
    };

    const auto result =
        dynamics.evaluate(state);

    const double expectedDragN =
        result.dynamicPressurePa *
        dynamics.object().reference.areaM2 *
        result.aerodynamics.cx;

    const double expectedNormalForceN =
        result.dynamicPressurePa *
        dynamics.object().reference.areaM2 *
        result.aerodynamics.cy;

    const double expectedMomentNm =
        result.dynamicPressurePa *
        dynamics.object().reference.areaM2 *
        dynamics.object().reference.meanAerodynamicChordM *
        result.aerodynamics.mz;

    checkNear(
        result.loads.dragN,
        expectedDragN,
        1.0e-9,
        "Drag dimensionalization"
    );

    checkNear(
        result.loads.normalForceN,
        expectedNormalForceN,
        1.0e-9,
        "Normal-force dimensionalization"
    );

    checkNear(
        result.loads.pitchingMomentNm,
        expectedMomentNm,
        1.0e-9,
        "Pitching-moment dimensionalization"
    );
}

void testAtmosphereConnection() {
    const auto dynamics = makeDynamics();

    const passive_flight::State lowState{
        0.0,
        250.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1000.0
    };

    const passive_flight::State highState{
        0.0,
        250.0,
        0.0,
        0.0,
        0.0,
        0.0,
        10000.0
    };

    const auto low =
        dynamics.evaluate(lowState);

    const auto high =
        dynamics.evaluate(highState);

    check(
        high.atmosphere.densityKgM3 <
            low.atmosphere.densityKgM3,
        "Density decreases with altitude"
    );

    check(
        high.dynamicPressurePa <
            low.dynamicPressurePa,
        "Dynamic pressure decreases with altitude"
    );

    check(
        high.mach > low.mach,
        "Mach increases when speed of sound decreases"
    );
}

void testRotationalEquation() {
    const auto dynamics = makeDynamics();

    const passive_flight::State state{
        0.0,
        250.0,
        0.0,
        0.20,
        0.05,
        0.0,
        5000.0
    };

    const auto result =
        dynamics.evaluate(state);

    const auto& [
        massKg,
        pitchMomentOfInertiaKgM2,
        centerOfMassXM
    ] = dynamics.object().mass;

    static_cast<void>(massKg);
    static_cast<void>(centerOfMassXM);

    checkNear(
        result.derivative.pitchRateRadps2,
        result.loads.pitchingMomentNm /
            pitchMomentOfInertiaKgM2,
        1.0e-12,
        "Rotational equation Iz * omega_dot = Mz"
    );
}

void testTimeDoesNotChangeInstantaneousDynamics() {
    const auto dynamics = makeDynamics();

    const passive_flight::State firstState{
        0.0,
        250.0,
        -0.05,
        0.01,
        -0.02,
        1000.0,
        5000.0
    };

    passive_flight::State secondState =
        firstState;

    secondState.timeS = 20.0;

    const auto first =
        dynamics.evaluate(firstState);

    const auto second =
        dynamics.evaluate(secondState);

    checkNear(
        first.derivative.speedMps2,
        second.derivative.speedMps2,
        1.0e-12,
        "Time does not directly change speed derivative"
    );

    checkNear(
        first.derivative.flightPathAngleRadps,
        second.derivative.flightPathAngleRadps,
        1.0e-12,
        "Time does not directly change trajectory derivative"
    );

    checkNear(
        first.derivative.pitchRateRadps2,
        second.derivative.pitchRateRadps2,
        1.0e-12,
        "Time does not directly change pitch acceleration"
    );
}

void testInvalidState() {
    const auto dynamics = makeDynamics();

    const passive_flight::State zeroSpeedState{
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        10000.0
    };

    checkThrows<std::invalid_argument>(
        [&dynamics, &zeroSpeedState]() {
            static_cast<void>(
                dynamics.evaluate(
                    zeroSpeedState
                )
            );
        },
        "Zero speed is rejected"
    );

    const passive_flight::State negativeTimeState{
        -1.0,
        300.0,
        0.0,
        0.0,
        0.0,
        0.0,
        10000.0
    };

    checkThrows<std::invalid_argument>(
        [&dynamics, &negativeTimeState]() {
            static_cast<void>(
                dynamics.evaluate(
                    negativeTimeState
                )
            );
        },
        "Negative time is rejected"
    );

    const passive_flight::State excessiveAltitudeState{
        0.0,
        300.0,
        0.0,
        0.0,
        0.0,
        0.0,
        26000.0
    };

    checkThrows<std::out_of_range>(
        [&dynamics, &excessiveAltitudeState]() {
            static_cast<void>(
                dynamics.evaluate(
                    excessiveAltitudeState
                )
            );
        },
        "Altitude above atmosphere range is rejected"
    );
}

} // namespace

int main() {
    testHorizontalReleaseKinematics();
    testKinematicEquations();
    testAngleOfAttackDefinition();
    testDimensionedLoads();
    testAtmosphereConnection();
    testRotationalEquation();
    testTimeDoesNotChangeInstantaneousDynamics();
    testInvalidState();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " nominal-dynamics test(s) failed"
            << '\n';

        return 1;
    }

    std::cout
        << "All nominal longitudinal dynamics tests passed"
        << '\n';

    return 0;
}