#include "passive_flight_c_api/PerturbedTrajectoryCApi.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

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

PFPerturbedSimulationInput makeInput() {
    PFPerturbedSimulationInput input{};

    input.objectId =
        "ABSTRACT_500_UMPK_V1";

    input.releaseAltitudeM =
        100.0;

    input.releaseSpeedMps =
        200.0;

    input.deltaSpeedMps =
        1.0;

    input.deltaPitchAngleRad =
        0.1 *
        3.14159265358979323846 /
        180.0;

    return input;
}

std::vector<PFPerturbedTrajectoryPoint>
calculateTrajectory(
    const PFPerturbedSimulationInput& input
) {
    uint64_t required = 0;
    uint64_t written = 0;

    int32_t result =
        pfCalculatePerturbedTrajectory(
            &input,
            nullptr,
            0,
            &required,
            &written
        );

    require(
        result ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "First trajectory call must request a buffer"
    );

    require(
        required > 2,
        "Trajectory must contain several points"
    );

    require(
        written == 0,
        "First trajectory call must not write points"
    );

    std::vector<PFPerturbedTrajectoryPoint>
        points(
            static_cast<std::size_t>(
                required
            )
        );

    result =
        pfCalculatePerturbedTrajectory(
            &input,
            points.data(),
            required,
            &required,
            &written
        );

    require(
        result == PF_RESULT_OK,
        "Second trajectory call failed"
    );

    require(
        written == required,
        "Written trajectory size is incorrect"
    );

    points.resize(
        static_cast<std::size_t>(
            written
        )
    );

    return points;
}

void testTrajectory() {
    const auto input =
        makeInput();

    const auto points =
        calculateTrajectory(
            input
        );

    const auto& first =
        points.front();

    const auto& last =
        points.back();

    requireNear(
        first.timeS,
        0.0,
        1.0e-12,
        "Initial time is incorrect"
    );

    requireNear(
        first.nominalDownrangeM,
        0.0,
        1.0e-12,
        "Initial nominal downrange is incorrect"
    );

    requireNear(
        first.nominalAltitudeM,
        100.0,
        1.0e-12,
        "Initial nominal altitude is incorrect"
    );

    requireNear(
        first.nominalSpeedMps,
        200.0,
        1.0e-12,
        "Initial nominal speed is incorrect"
    );

    requireNear(
        first.deltaSpeedMps,
        1.0,
        1.0e-12,
        "Initial Delta V is incorrect"
    );

    requireNear(
        first.deltaPitchAngleRad,
        input.deltaPitchAngleRad,
        1.0e-12,
        "Initial Delta theta is incorrect"
    );

    requireNear(
        first.deltaAngleOfAttackRad,
        input.deltaPitchAngleRad,
        1.0e-12,
        "Initial Delta alpha is incorrect"
    );

    requireNear(
        first.totalSpeedMps,
        first.nominalSpeedMps +
            first.deltaSpeedMps,
        1.0e-12,
        "Initial total speed is inconsistent"
    );

    requireNear(
        first.totalAltitudeM,
        first.nominalAltitudeM +
            first.deltaAltitudeM,
        1.0e-12,
        "Initial total altitude is inconsistent"
    );

    require(
        last.timeS > 0.0,
        "Final time must be positive"
    );

    requireNear(
        last.totalAltitudeM,
        0.0,
        1.0e-9,
        "Final perturbed altitude must be ground"
    );

    requireNear(
        last.totalDownrangeM,
        last.nominalDownrangeM +
            last.deltaDownrangeM,
        1.0e-10,
        "Final total downrange is inconsistent"
    );

    requireNear(
        last.totalAltitudeM,
        last.nominalAltitudeM +
            last.deltaAltitudeM,
        1.0e-10,
        "Final total altitude is inconsistent"
    );

    requireNear(
        last.totalSpeedMps,
        last.nominalSpeedMps +
            last.deltaSpeedMps,
        1.0e-10,
        "Final total speed is inconsistent"
    );

    requireNear(
        last.totalFlightPathAngleRad,
        last.nominalFlightPathAngleRad +
            last.deltaFlightPathAngleRad,
        1.0e-10,
        "Final total Theta is inconsistent"
    );

    requireNear(
        last.totalPitchAngleRad,
        last.nominalPitchAngleRad +
            last.deltaPitchAngleRad,
        1.0e-10,
        "Final total theta is inconsistent"
    );

    requireNear(
        last.totalAngleOfAttackRad,
        last.totalPitchAngleRad -
            last.totalFlightPathAngleRad,
        1.0e-10,
        "Final total alpha is inconsistent"
    );

    require(
        points.size() >= 2,
        "Trajectory must have a point before impact"
    );

    require(
        points[points.size() - 2].timeS <
            last.timeS,
        "Terminal impact point must preserve increasing time"
    );
}

void testZeroPerturbation() {
    auto input =
        makeInput();

    input.deltaSpeedMps = 0.0;
    input.deltaPitchAngleRad = 0.0;

    const auto points =
        calculateTrajectory(
            input
        );

    for (const auto& point : points) {
        requireNear(
            point.deltaSpeedMps,
            0.0,
            1.0e-12,
            "Zero perturbation produced Delta V"
        );

        requireNear(
            point.deltaAltitudeM,
            0.0,
            1.0e-12,
            "Zero perturbation produced Delta H"
        );

        requireNear(
            point.deltaAngleOfAttackRad,
            0.0,
            1.0e-12,
            "Zero perturbation produced Delta alpha"
        );
    }

    requireNear(
        points.back().nominalAltitudeM,
        0.0,
        1.0e-9,
        "Zero-perturbation nominal altitude must end at ground"
    );

    requireNear(
        points.back().totalAltitudeM,
        0.0,
        1.0e-9,
        "Zero-perturbation total altitude must end at ground"
    );
}

} // namespace

int main() {
    testTrajectory();
    testZeroPerturbation();

    std::cout
        << "All perturbed trajectory C API tests passed.\n";

    return EXIT_SUCCESS;
}
