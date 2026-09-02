#include "passive_flight/ForwardEulerSimulator.hpp"
#include "passive_flight/KnownObjects.hpp"
#include "passive_flight/NominalLongitudinalDynamics.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
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
        throw std::runtime_error(
            message +
            ": actual=" +
            std::to_string(actual) +
            ", expected=" +
            std::to_string(expected)
        );
    }
}

void testPassportValues() {
    const auto object =
        passive_flight::makeFab1500TPostnikovModel();

    require(
        object.id ==
            "FAB_1500T_POSTNIKOV_1979",
        "FAB-1500T object id"
    );

    requireNear(
        object.mass.massKg,
        1519.0,
        0.0,
        "FAB-1500T mass"
    );

    requireNear(
        object.mass.pitchMomentOfInertiaKgM2,
        1122.3,
        0.0,
        "FAB-1500T pitch inertia"
    );

    requireNear(
        object.mass.centerOfMassXM,
        1.16,
        0.0,
        "FAB-1500T center of mass"
    );

    requireNear(
        object.body.lengthM,
        3.46,
        0.0,
        "FAB-1500T length"
    );

    requireNear(
        object.body.diameterM,
        0.58,
        0.0,
        "FAB-1500T diameter"
    );

    const double expectedArea =
        std::acos(-1.0) *
        0.58 *
        0.58 /
        4.0;

    requireNear(
        object.reference.areaM2,
        expectedArea,
        1.0e-12,
        "FAB-1500T reference area"
    );

    requireNear(
        object.reference.effectiveReferenceLengthM(),
        3.46,
        0.0,
        "FAB-1500T reference length"
    );

    require(
        object.aerodynamicModelType ==
            passive_flight::AerodynamicModelType::Tabulated,
        "FAB-1500T must use tabulated aerodynamics"
    );
}

void testKnownAerodynamicNode() {
    const auto object =
        passive_flight::makeFab1500TPostnikovModel();

    const passive_flight::NominalLongitudinalDynamics
        dynamics(object);

    passive_flight::AerodynamicInput input;

    input.mach = 0.90;
    input.angleOfAttackRad = 0.10;
    input.pitchRateRadS = 0.20;
    input.angleOfAttackRateRadS = 0.0;
    input.speedMps = 250.0;

    const auto coefficients =
        dynamics.aerodynamics().evaluate(input);

    requireNear(
        coefficients.cx0,
        0.280,
        1.0e-12,
        "Cx0 at M=0.90"
    );

    requireNear(
        coefficients.cyAlphaPerRad,
        3.9,
        1.0e-12,
        "CyAlpha at M=0.90"
    );

    requireNear(
        coefficients.mzAlphaPerRad,
        -0.88,
        1.0e-12,
        "MzAlpha at M=0.90"
    );

    requireNear(
        coefficients.mzPitchRateDerivative,
        -0.95,
        1.0e-12,
        "MzOmegaBar at M=0.90"
    );

    const double expectedOmegaBar =
        0.20 *
        3.46 /
        250.0;

    requireNear(
        coefficients.mzPitchDamping,
        -0.95 *
        expectedOmegaBar,
        1.0e-12,
        "Lebedev omegaBar normalization for FAB-1500T"
    );
}

void testHorizontalReleaseTrajectory() {
    const auto object =
        passive_flight::makeFab1500TPostnikovModel();

    const passive_flight::ForwardEulerSimulator
        simulator(object);

    passive_flight::SimulationRequest request;

    request.objectId =
        object.id;

    request.release.altitudeM =
        100.0;

    request.release.speedMps =
        200.0;

    passive_flight::SimulationOptions options;

    options.timeStepS =
        0.001;

    options.maximumTimeS =
        30.0;

    options.maximumSteps =
        100000;

    options.saveHistory =
        true;

    const auto result =
        simulator.simulate(
            request,
            options
        );

    require(
        result.terminationReason ==
            passive_flight::TerminationReason::GroundReached,
        "FAB-1500T trajectory must reach ground"
    );

    require(
        result.finalState.downrangeM > 800.0 &&
        result.finalState.downrangeM < 1000.0,
        "FAB-1500T 100 m downrange sanity range"
    );

    require(
        result.finalState.timeS > 4.0 &&
        result.finalState.timeS < 5.0,
        "FAB-1500T 100 m fall-time sanity range"
    );

    require(
        result.finalState.speedMps > 180.0 &&
        result.finalState.speedMps < 220.0,
        "FAB-1500T impact-speed sanity range"
    );

    require(
        !result.history.empty(),
        "FAB-1500T history must not be empty"
    );

    /*
     * Устойчивый объект после начального переходного
     * процесса должен сохранять малый угол атаки.
     */
    require(
        std::abs(
            result.finalState.angleOfAttackRad()
        ) < 0.02,
        "FAB-1500T final angle of attack must remain small"
    );
}

} // namespace

int main() {
    try {
        testPassportValues();
        testKnownAerodynamicNode();
        testHorizontalReleaseTrajectory();

        std::cout
            << "All FAB-1500T model tests passed."
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
