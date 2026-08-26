#include "passive_flight/Aerodynamics.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

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

passive_flight::AerodynamicInput makeInput(
    double mach,
    double alphaDeg
) {
    passive_flight::AerodynamicInput input;

    input.mach = mach;
    input.angleOfAttackRad =
        alphaDeg *
        std::numbers::pi_v<double> /
        180.0;

    input.pitchRateRadS = 0.0;
    input.angleOfAttackRateRadS = 0.0;
    input.speedMps = 300.0;

    return input;
}

void testBaselineGeometry() {
    const auto geometry =
        passive_flight::makeAbstract500AerodynamicGeometry();

    checkNear(
        geometry.referenceAreaM2,
        0.475,
        1.0e-12,
        "Reference area"
    );

    checkNear(
        geometry.wing.aspectRatio,
        6.521263157894737,
        1.0e-12,
        "Wing aspect ratio"
    );

    checkNear(
        geometry.tail.aspectRatio,
        0.9821412639405205,
        1.0e-12,
        "Tail aspect ratio"
    );

    check(
        geometry.tail.aerodynamicCenterXM >
            geometry.centerOfMassXM,
        "Tail aerodynamic center is behind center of mass"
    );
}

void testDragTableNodes() {
    const passive_flight::PreliminaryAerodynamicModel model;

    const auto subsonic =
        model.evaluate(makeInput(0.40, 0.0));

    const auto sonic =
        model.evaluate(makeInput(1.00, 0.0));

    const auto supersonic =
        model.evaluate(makeInput(2.00, 0.0));

    checkNear(
        subsonic.cx0,
        0.190,
        1.0e-12,
        "Cx0 at Mach 0.4"
    );

    checkNear(
        sonic.cx0,
        0.446,
        1.0e-12,
        "Cx0 at Mach 1.0"
    );

    checkNear(
        supersonic.cx0,
        0.480,
        1.0e-12,
        "Cx0 at Mach 2.0"
    );
}

void testDragInterpolation() {
    const passive_flight::PreliminaryAerodynamicModel model;

    const auto result =
        model.evaluate(makeInput(0.775, 0.0));

    checkNear(
        result.cx0,
        0.205,
        1.0e-12,
        "Linear interpolation of Cx0"
    );
}

void testZeroAngleOfAttack() {
    const passive_flight::PreliminaryAerodynamicModel model;

    const auto result =
        model.evaluate(makeInput(0.8, 0.0));

    checkNear(
        result.cyBody,
        0.0,
        1.0e-12,
        "Body Cy is zero at zero alpha"
    );

    checkNear(
        result.cyWing,
        0.0,
        1.0e-12,
        "Wing Cy is zero at zero alpha"
    );

    checkNear(
        result.cyTail,
        0.0,
        1.0e-12,
        "Tail Cy is zero at zero alpha"
    );

    checkNear(
        result.cy,
        0.0,
        1.0e-12,
        "Total Cy is zero at zero alpha"
    );

    checkNear(
        result.cxInduced,
        0.0,
        1.0e-12,
        "Induced drag is zero at zero alpha"
    );

    checkNear(
        result.mzStatic,
        0.0,
        1.0e-12,
        "Static Mz is zero at zero alpha"
    );
}

void testAngleOfAttackSigns() {
    const passive_flight::PreliminaryAerodynamicModel model;

    const auto positive =
        model.evaluate(makeInput(0.8, 5.0));

    const auto negative =
        model.evaluate(makeInput(0.8, -5.0));

    check(
        positive.cy > 0.0,
        "Positive alpha produces positive Cy"
    );

    check(
        negative.cy < 0.0,
        "Negative alpha produces negative Cy"
    );

    checkNear(
        positive.cy,
        -negative.cy,
        1.0e-12,
        "Cy is antisymmetric by alpha"
    );

    checkNear(
        positive.cxInduced,
        negative.cxInduced,
        1.0e-12,
        "Induced drag is symmetric by alpha"
    );

    check(
        positive.cx > positive.cx0,
        "Angle of attack increases drag"
    );

    check(
        positive.cyAlphaPerRad > 0.0,
        "Cy-alpha derivative is positive"
    );
}

void testPitchDamping() {
    const passive_flight::PreliminaryAerodynamicModel model;

    auto input = makeInput(0.8, 0.0);
    input.pitchRateRadS = 1.0;

    const auto result = model.evaluate(input);

    check(
        result.mzPitchRateDerivative < 0.0,
        "Pitch-rate derivative is damping"
    );

    check(
        result.mzPitchDamping < 0.0,
        "Positive pitch rate produces negative damping moment"
    );

    checkNear(
        result.mz,
        result.mzPitchDamping,
        1.0e-12,
        "At zero alpha Mz consists of pitch damping"
    );
}

void testAlphaDotContribution() {
    const passive_flight::PreliminaryAerodynamicModel model;

    auto input = makeInput(0.8, 0.0);
    input.angleOfAttackRateRadS = 1.0;

    const auto result = model.evaluate(input);

    check(
        result.mzAlphaDotDerivative < 0.0,
        "Alpha-dot derivative is damping"
    );

    check(
        result.mzAlphaDot < 0.0,
        "Positive alpha-dot produces negative moment"
    );

    checkNear(
        result.mz,
        result.mzAlphaDot,
        1.0e-12,
        "At zero alpha Mz consists of alpha-dot contribution"
    );
}

void testGeometryInfluence() {
    auto baselineGeometry =
        passive_flight::makeAbstract500AerodynamicGeometry();

    auto modifiedGeometry = baselineGeometry;

    modifiedGeometry.wing.aspectRatio =
        baselineGeometry.wing.aspectRatio * 1.5;

    const passive_flight::PreliminaryAerodynamicModel baselineModel(
        baselineGeometry,
        passive_flight::makeAbstract500ZeroLiftDragTable()
    );

    const passive_flight::PreliminaryAerodynamicModel modifiedModel(
        modifiedGeometry,
        passive_flight::makeAbstract500ZeroLiftDragTable()
    );

    const auto input = makeInput(0.8, 5.0);

    const auto baseline =
        baselineModel.evaluate(input);

    const auto modified =
        modifiedModel.evaluate(input);

    check(
        modified.cyWing > baseline.cyWing,
        "Increasing wing aspect ratio increases wing Cy"
    );

    check(
        std::abs(modified.cy - baseline.cy) > 1.0e-6,
        "Changing geometry changes total Cy"
    );

    check(
        std::abs(modified.mz - baseline.mz) > 1.0e-6,
        "Changing geometry changes Mz"
    );
}

void testValidation() {
    auto geometry =
        passive_flight::makeAbstract500AerodynamicGeometry();

    geometry.referenceAreaM2 = 0.0;

    checkThrows<std::invalid_argument>(
        [&geometry]() {
            const passive_flight::PreliminaryAerodynamicModel model(
                geometry,
                passive_flight::makeAbstract500ZeroLiftDragTable()
            );

            static_cast<void>(model);
        },
        "Zero reference area is rejected"
    );

    checkThrows<std::invalid_argument>(
        []() {
            const passive_flight::PreliminaryAerodynamicModel model;

            auto input = makeInput(0.8, 0.0);
            input.mach =
                std::numeric_limits<double>::quiet_NaN();

            static_cast<void>(model.evaluate(input));
        },
        "NaN Mach number is rejected"
    );

    checkThrows<std::invalid_argument>(
        []() {
            const auto geometry =
                passive_flight::makeAbstract500AerodynamicGeometry();

            const std::vector<
                passive_flight::DragCoefficientPoint
            > invalidTable{
                {0.8, 0.20},
                {0.7, 0.19}
            };

            const passive_flight::PreliminaryAerodynamicModel model(
                geometry,
                invalidTable
            );

            static_cast<void>(model);
        },
        "Unsorted drag table is rejected"
    );
}

} // namespace

int main() {
    testBaselineGeometry();
    testDragTableNodes();
    testDragInterpolation();
    testZeroAngleOfAttack();
    testAngleOfAttackSigns();
    testPitchDamping();
    testAlphaDotContribution();
    testGeometryInfluence();
    testValidation();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " aerodynamic test(s) failed"
            << '\n';

        return 1;
    }

    std::cout
        << "All aerodynamic tests passed"
        << '\n';

    return 0;
}