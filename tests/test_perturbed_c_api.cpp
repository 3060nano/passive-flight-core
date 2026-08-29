#include "passive_flight_c_api/PerturbedFlightCApi.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void require(
    bool condition,
    const char* message
) {
    if (!condition) {
        std::cerr
            << "FAILED: "
            << message
            << '\n';

        std::exit(
            EXIT_FAILURE
        );
    }
}

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const char* message
) {
    if (std::abs(
            actual - expected
        ) > tolerance) {
        std::cerr
            << "FAILED: "
            << message
            << ": actual="
            << actual
            << ", expected="
            << expected
            << '\n';

        std::exit(
            EXIT_FAILURE
        );
    }
}

PFPerturbedSimulationInput
makeBaseInput() {
    PFPerturbedSimulationInput input{};

    input.objectId =
        "ABSTRACT_500_UMPK_V1";

    input.releaseAltitudeM =
        100.0;

    input.releaseSpeedMps =
        200.0;

    return input;
}

void testVersion() {
    const char* version =
        pfGetPerturbedApiVersion();

    require(
        version != nullptr,
        "Perturbed API version is null"
    );
}

void testZeroPerturbation() {
    const auto input =
        makeBaseInput();

    PFPerturbedImpactOutput output{};

    const int32_t result =
        pfCalculatePerturbedImpact(
            &input,
            &output
        );

    require(
        result == PF_RESULT_OK,
        "Zero-perturbation calculation failed"
    );

    require(
        output.terminationReason ==
            PF_TERMINATION_GROUND_REACHED,
        "Zero-perturbation calculation did not reach ground"
    );

    requireNear(
        output.deltaFallTimeS,
        0.0,
        1.0e-12,
        "Zero perturbation changed fall time"
    );

    requireNear(
        output.deltaDownrangeM,
        0.0,
        1.0e-12,
        "Zero perturbation changed downrange"
    );

    requireNear(
        output.deltaImpactSpeedMps,
        0.0,
        1.0e-12,
        "Zero perturbation changed impact speed"
    );

    requireNear(
        output.deltaImpactFlightPathAngleRad,
        0.0,
        1.0e-12,
        "Zero perturbation changed impact path angle"
    );

    requireNear(
        output.deltaImpactPitchRateRadps,
        0.0,
        1.0e-12,
        "Zero perturbation changed impact pitch rate"
    );

    requireNear(
        output.deltaImpactPitchAngleRad,
        0.0,
        1.0e-12,
        "Zero perturbation changed impact pitch angle"
    );

    requireNear(
        output.deltaImpactAngleOfAttackRad,
        0.0,
        1.0e-12,
        "Zero perturbation changed impact attack angle"
    );

    require(
        output.perturbedImpactTimeS > 0.0,
        "Perturbed impact time must be positive"
    );

    require(
        output.perturbedImpactDownrangeM > 0.0,
        "Perturbed impact downrange must be positive"
    );

    require(
        output.perturbedImpactSpeedMps > 0.0,
        "Perturbed impact speed must be positive"
    );
}

void testNonZeroPerturbation() {
    auto input =
        makeBaseInput();

    input.deltaSpeedMps =
        1.0;

    input.deltaPitchAngleRad =
        0.1 *
        3.14159265358979323846 /
        180.0;

    PFPerturbedImpactOutput output{};

    const int32_t result =
        pfCalculatePerturbedImpact(
            &input,
            &output
        );

    require(
        result == PF_RESULT_OK,
        "Nonzero-perturbation calculation failed"
    );

    require(
        std::isfinite(
            output.deltaFallTimeS
        ),
        "Delta fall time is not finite"
    );

    require(
        std::isfinite(
            output.deltaDownrangeM
        ),
        "Delta downrange is not finite"
    );

    require(
        std::isfinite(
            output.deltaImpactSpeedMps
        ),
        "Delta impact speed is not finite"
    );

    require(
        std::isfinite(
            output.deltaImpactFlightPathAngleRad
        ),
        "Delta impact path angle is not finite"
    );

    require(
        std::isfinite(
            output.deltaImpactPitchRateRadps
        ),
        "Delta impact pitch rate is not finite"
    );

    require(
        std::isfinite(
            output.deltaImpactPitchAngleRad
        ),
        "Delta impact pitch angle is not finite"
    );

    require(
        std::isfinite(
            output.deltaImpactAngleOfAttackRad
        ),
        "Delta impact attack angle is not finite"
    );

    require(
        std::abs(
            output.deltaDownrangeM
        ) > 1.0e-9,
        "Nonzero perturbation did not change impact downrange"
    );
}

void testInvalidObject() {
    auto input =
        makeBaseInput();

    input.objectId =
        "UNKNOWN_OBJECT";

    PFPerturbedImpactOutput output{};

    const int32_t result =
        pfCalculatePerturbedImpact(
            &input,
            &output
        );

    require(
        result ==
            PF_RESULT_OBJECT_NOT_FOUND,
        "Unknown object was not rejected"
    );
}

} // namespace

int main() {
    testVersion();
    testZeroPerturbation();
    testNonZeroPerturbation();
    testInvalidObject();

    std::cout
        << "All perturbed C API tests passed.\n";

    return EXIT_SUCCESS;
}
