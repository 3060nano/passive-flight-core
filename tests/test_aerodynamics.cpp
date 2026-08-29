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
    if (
        std::abs(
            actual -
            expected
        ) >
        tolerance
    ) {
        std::cerr
            << "FAILED: "
            << message
            << "; expected "
            << expected
            << ", actual "
            << actual
            << ", tolerance "
            << tolerance
            << '\n';

        ++failureCount;
    }
}

template <
    typename ExceptionType,
    typename Function
>
void checkThrows(
    Function function,
    const std::string& message
) {
    try {
        function();

        std::cerr
            << "FAILED: "
            << message
            << "; exception was not thrown"
            << '\n';

        ++failureCount;
    } catch (
        const ExceptionType&
    ) {
        // Ожидаемое исключение.
    } catch (...) {
        std::cerr
            << "FAILED: "
            << message
            << "; unexpected exception type"
            << '\n';

        ++failureCount;
    }
}

passive_flight::AerodynamicInput
makeInput(
    double mach,
    double alphaDeg
) {
    passive_flight::AerodynamicInput input;

    input.mach =
        mach;

    input.angleOfAttackRad =
        alphaDeg *
        std::numbers::pi_v<double> /
        180.0;

    input.pitchRateRadS =
        0.0;

    input.angleOfAttackRateRadS =
        0.0;

    input.speedMps =
        300.0;

    return input;
}

void testBaselineGeometry() {
    const auto geometry =
        passive_flight::
            makeAbstract500AerodynamicGeometry();

    checkNear(
        geometry.referenceAreaM2,
        0.475,
        1.0e-12,
        "Reference area"
    );

    checkNear(
        geometry.centerOfMassXM,
        1.17,
        1.0e-12,
        "Baseline center of mass"
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

    checkNear(
        geometry.wing.installationAngleRad,
        3.0 *
            std::numbers::pi_v<double> /
            180.0,
        1.0e-12,
        "Wing installation angle is +3 degrees"
    );

    checkNear(
        geometry.tail.installationAngleRad,
        0.0,
        1.0e-12,
        "Tail installation angle is zero"
    );

    checkNear(
        geometry.downwashGradient,
        0.57,
        1.0e-12,
        "Fallback downwash gradient is 0.57"
    );

    check(
        geometry.tail.aerodynamicCenterXM >
            geometry.centerOfMassXM,
        "Tail aerodynamic center is behind center of mass"
    );
}

void testDragTableNodes() {
    const passive_flight::
        PreliminaryAerodynamicModel
            model;

    const auto subsonic =
        model.evaluate(
            makeInput(
                0.40,
                0.0
            )
        );

    const auto sonic =
        model.evaluate(
            makeInput(
                1.00,
                0.0
            )
        );

    const auto supersonic =
        model.evaluate(
            makeInput(
                2.00,
                0.0
            )
        );

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
    const passive_flight::
        PreliminaryAerodynamicModel
            model;

    const auto result =
        model.evaluate(
            makeInput(
                0.775,
                0.0
            )
        );

    checkNear(
        result.cx0,
        0.205,
        1.0e-12,
        "Linear interpolation of Cx0"
    );
}

/*
 * Здесь alpha = 0 означает нулевой угол атаки
 * всего объекта, а не крыла.
 *
 * При iWing = +3 deg:
 *
 *     alphaWing = alpha + iWing = 3 deg.
 */
void testZeroObjectAngleOfAttack() {
    const passive_flight::
        PreliminaryAerodynamicModel
            model;

    const auto result =
        model.evaluate(
            makeInput(
                0.8,
                0.0
            )
        );

    checkNear(
        result.cyBody,
        0.0,
        1.0e-12,
        "Body Cy is zero at zero object alpha"
    );

    check(
        result.cyWing > 0.0,
        "Positive wing installation angle produces positive wing Cy"
    );

    checkNear(
        result.cyTail,
        0.0,
        1.0e-12,
        "Tail Cy is zero at zero object alpha and zero tail incidence"
    );

    check(
        result.cy > 0.0,
        "Total Cy is positive because wing incidence is positive"
    );

    check(
        result.cxInduced > 0.0,
        "Wing incidence creates induced drag"
    );

    check(
        std::isfinite(
            result.mzStatic
        ),
        "Static Mz is finite at zero object alpha"
    );
}

void testAngleOfAttackInfluence() {
    const passive_flight::
        PreliminaryAerodynamicModel
            model;

    const auto zero =
        model.evaluate(
            makeInput(
                0.8,
                0.0
            )
        );

    const auto positive =
        model.evaluate(
            makeInput(
                0.8,
                5.0
            )
        );

    const auto negative =
        model.evaluate(
            makeInput(
                0.8,
                -5.0
            )
        );

    check(
        positive.cy >
            zero.cy,
        "Increasing object alpha increases Cy"
    );

    check(
        negative.cy <
            zero.cy,
        "Decreasing object alpha decreases Cy"
    );

    check(
        positive.cyAlphaPerRad >
            0.0,
        "Cy-alpha derivative is positive"
    );

    /*
     * При ненулевом угле установки крыла
     * зависимость больше не антисимметрична
     * относительно alpha = 0.
     */
    check(
        std::abs(
            positive.cy +
            negative.cy
        ) >
        1.0e-8,
        "Wing incidence shifts Cy-alpha relation"
    );
}

void testBaselineDownwashTable() {
    const passive_flight::
        PreliminaryAerodynamicModel model;

    const auto atMach04 =
        model.evaluate(makeInput(0.4, 5.0));

    const auto atMach07 =
        model.evaluate(makeInput(0.7, 5.0));

    const auto supersonicFallback =
        model.evaluate(makeInput(1.5, 5.0));

    checkNear(
        atMach04.downwashGradient,
        0.576,
        1.0e-12,
        "Baseline downwash gradient at Mach 0.4"
    );

    checkNear(
        atMach07.downwashGradient,
        0.5725,
        1.0e-12,
        "Downwash gradient is linearly interpolated by Mach"
    );

    checkNear(
        supersonicFallback.downwashGradient,
        0.569,
        1.0e-12,
        "Supersonic downwash uses last subsonic value as provisional fallback"
    );
}

void testCustomDownwashTable() {
    auto geometry =
        passive_flight::makeAbstract500AerodynamicGeometry();

    geometry.downwashGradient = 0.30;

    const std::vector<passive_flight::DownwashGradientPoint>
        downwashTable{
            {0.5, 0.40},
            {1.0, 0.60}
        };

    const passive_flight::PreliminaryAerodynamicModel model(
        geometry,
        passive_flight::makeAbstract500ZeroLiftDragTable(),
        downwashTable,
        {}
    );

    const auto result =
        model.evaluate(makeInput(0.75, 5.0));

    checkNear(
        result.downwashGradient,
        0.50,
        1.0e-12,
        "Custom downwash table is interpolated"
    );

    geometry.downwashGradient = 0.30;

    const passive_flight::PreliminaryAerodynamicModel fallbackModel(
        geometry,
        passive_flight::makeAbstract500ZeroLiftDragTable()
    );

    const auto fallbackResult =
        fallbackModel.evaluate(makeInput(0.75, 5.0));

    check(
        result.cyTail < fallbackResult.cyTail,
        "Larger downwash gradient reduces tail normal-force response"
    );
}

void testDownwashFallbackWithoutTable() {
    auto geometry =
        passive_flight::makeAbstract500AerodynamicGeometry();

    geometry.downwashGradient = 0.42;

    const passive_flight::PreliminaryAerodynamicModel model(
        geometry,
        passive_flight::makeAbstract500ZeroLiftDragTable()
    );

    const auto result =
        model.evaluate(makeInput(0.8, 5.0));

    checkNear(
        result.downwashGradient,
        0.42,
        1.0e-12,
        "Geometry fallback is used when downwash table is absent"
    );
}

void testPitchDamping() {
    const passive_flight::
        PreliminaryAerodynamicModel
            model;

    auto input =
        makeInput(
            0.8,
            0.0
        );

    input.pitchRateRadS =
        1.0;

    const auto result =
        model.evaluate(
            input
        );

    check(
        result.mzPitchRateBodyDerivative <
            0.0,
        "Body pitch-rate derivative is damping"
    );

    checkNear(
        result.mzPitchRateWingDerivative,
        0.0,
        1.0e-12,
        "Wing pitch-rate derivative is explicitly not modelled yet"
    );

    check(
        result.mzPitchRateTailDerivative <
            0.0,
        "Tail pitch-rate derivative is damping"
    );

    checkNear(
        result.mzPitchRateDerivative,
        result.mzPitchRateBodyDerivative +
            result.mzPitchRateWingDerivative +
            result.mzPitchRateTailDerivative,
        1.0e-12,
        "Total pitch-rate derivative is sum of body, wing and tail contributions"
    );

    check(
        result.mzPitchRateDerivative <
            result.mzPitchRateTailDerivative,
        "Body contribution makes total damping stronger than tail-only damping"
    );

    check(
        result.mzPitchDamping <
            0.0,
        "Positive pitch rate produces negative damping moment"
    );

    checkNear(
        result.mz,
        result.mzStatic +
            result.mzPitchDamping,
        1.0e-12,
        "Mz consists of static and pitch-rate terms when alpha-dot table is absent"
    );
}

/*
 * Пока таблица mzAlphaDotDerivative(M)
 * отсутствует, alphaDot не должен создавать
 * искусственный момент.
 */
void testAlphaDotDisabledWithoutTable() {
    const passive_flight::
        PreliminaryAerodynamicModel
            model;

    auto input =
        makeInput(
            0.8,
            0.0
        );

    input.angleOfAttackRateRadS =
        1.0;

    const auto result =
        model.evaluate(
            input
        );

    checkNear(
        result.mzAlphaDotDerivative,
        0.0,
        1.0e-12,
        "Alpha-dot derivative is zero without source table"
    );

    checkNear(
        result.mzAlphaDot,
        0.0,
        1.0e-12,
        "Alpha-dot contribution is zero without source table"
    );

    checkNear(
        result.mz,
        result.mzStatic,
        1.0e-12,
        "Alpha-dot does not change Mz while derivative table is absent"
    );
}

/*
 * Проверяем готовность архитектуры к будущей
 * таблице mzAlphaDotDerivative(M).
 */
void testAlphaDotTableInterpolation() {
    const auto geometry =
        passive_flight::
            makeAbstract500AerodynamicGeometry();

    const std::vector<
        passive_flight::
            PitchMomentAlphaDotDerivativePoint
    > alphaDotTable{
        {0.5, -4.0},
        {1.0, -2.0}
    };

    const passive_flight::
        PreliminaryAerodynamicModel
            model(
                geometry,
                passive_flight::
                    makeAbstract500ZeroLiftDragTable(),
                alphaDotTable
            );

    auto input =
        makeInput(
            0.75,
            0.0
        );

    input.angleOfAttackRateRadS =
        1.0;

    const auto result =
        model.evaluate(
            input
        );

    checkNear(
        result.mzAlphaDotDerivative,
        -3.0,
        1.0e-12,
        "Alpha-dot derivative is linearly interpolated by Mach"
    );

    check(
        result.mzAlphaDot <
            0.0,
        "Negative derivative produces negative moment for positive alpha-dot"
    );

    checkNear(
        result.mz,
        result.mzStatic +
            result.mzAlphaDot,
        1.0e-12,
        "Tabular alpha-dot contribution enters total Mz"
    );
}

void testAlphaDotTableClamping() {
    const auto geometry =
        passive_flight::
            makeAbstract500AerodynamicGeometry();

    const std::vector<
        passive_flight::
            PitchMomentAlphaDotDerivativePoint
    > alphaDotTable{
        {0.5, -4.0},
        {1.0, -2.0}
    };

    const passive_flight::
        PreliminaryAerodynamicModel
            model(
                geometry,
                passive_flight::
                    makeAbstract500ZeroLiftDragTable(),
                alphaDotTable
            );

    const auto below =
        model.evaluate(
            makeInput(
                0.2,
                0.0
            )
        );

    const auto above =
        model.evaluate(
            makeInput(
                1.5,
                0.0
            )
        );

    checkNear(
        below.mzAlphaDotDerivative,
        -4.0,
        1.0e-12,
        "Alpha-dot table clamps below minimum Mach"
    );

    checkNear(
        above.mzAlphaDotDerivative,
        -2.0,
        1.0e-12,
        "Alpha-dot table clamps above maximum Mach"
    );
}

void testGeometryInfluence() {
    auto baselineGeometry =
        passive_flight::
            makeAbstract500AerodynamicGeometry();

    auto modifiedGeometry =
        baselineGeometry;

    modifiedGeometry.wing.aspectRatio =
        baselineGeometry.wing.aspectRatio *
        1.5;

    const passive_flight::
        PreliminaryAerodynamicModel
            baselineModel(
                baselineGeometry,
                passive_flight::
                    makeAbstract500ZeroLiftDragTable()
            );

    const passive_flight::
        PreliminaryAerodynamicModel
            modifiedModel(
                modifiedGeometry,
                passive_flight::
                    makeAbstract500ZeroLiftDragTable()
            );

    const auto input =
        makeInput(
            0.8,
            5.0
        );

    const auto baseline =
        baselineModel.evaluate(
            input
        );

    const auto modified =
        modifiedModel.evaluate(
            input
        );

    check(
        modified.cyWing >
            baseline.cyWing,
        "Increasing wing aspect ratio increases wing Cy"
    );

    check(
        std::abs(
            modified.cy -
            baseline.cy
        ) >
        1.0e-6,
        "Changing geometry changes total Cy"
    );

    check(
        std::abs(
            modified.mz -
            baseline.mz
        ) >
        1.0e-6,
        "Changing geometry changes Mz"
    );
}

void testValidation() {
    auto geometry =
        passive_flight::
            makeAbstract500AerodynamicGeometry();

    geometry.referenceAreaM2 =
        0.0;

    checkThrows<
        std::invalid_argument
    >(
        [&geometry]() {
            const passive_flight::
                PreliminaryAerodynamicModel
                    model(
                        geometry,
                        passive_flight::
                            makeAbstract500ZeroLiftDragTable()
                    );

            static_cast<void>(
                model
            );
        },
        "Zero reference area is rejected"
    );

    checkThrows<
        std::invalid_argument
    >(
        []() {
            const passive_flight::
                PreliminaryAerodynamicModel
                    model;

            auto input =
                makeInput(
                    0.8,
                    0.0
                );

            input.mach =
                std::numeric_limits<double>::
                    quiet_NaN();

            static_cast<void>(
                model.evaluate(
                    input
                )
            );
        },
        "NaN Mach number is rejected"
    );

    checkThrows<
        std::invalid_argument
    >(
        []() {
            const auto geometry =
                passive_flight::
                    makeAbstract500AerodynamicGeometry();

            const std::vector<
                passive_flight::
                    DragCoefficientPoint
            > invalidTable{
                {0.8, 0.20},
                {0.7, 0.19}
            };

            const passive_flight::
                PreliminaryAerodynamicModel
                    model(
                        geometry,
                        invalidTable
                    );

            static_cast<void>(
                model
            );
        },
        "Unsorted drag table is rejected"
    );

    checkThrows<
        std::invalid_argument
    >(
        []() {
            const auto geometry =
                passive_flight::
                    makeAbstract500AerodynamicGeometry();

            const std::vector<
                passive_flight::
                    PitchMomentAlphaDotDerivativePoint
            > invalidTable{
                {0.8, -3.0}
            };

            const passive_flight::
                PreliminaryAerodynamicModel
                    model(
                        geometry,
                        passive_flight::
                            makeAbstract500ZeroLiftDragTable(),
                        invalidTable
                    );

            static_cast<void>(
                model
            );
        },
        "Single-point alpha-dot table is rejected"
    );

    checkThrows<
        std::invalid_argument
    >(
        []() {
            const auto geometry =
                passive_flight::
                    makeAbstract500AerodynamicGeometry();

            const std::vector<
                passive_flight::
                    PitchMomentAlphaDotDerivativePoint
            > invalidTable{
                {0.8, -3.0},
                {0.7, -2.0}
            };

            const passive_flight::
                PreliminaryAerodynamicModel
                    model(
                        geometry,
                        passive_flight::
                            makeAbstract500ZeroLiftDragTable(),
                        invalidTable
                    );

            static_cast<void>(
                model
            );
        },
        "Unsorted alpha-dot table is rejected"
    );

    checkThrows<
        std::invalid_argument
    >(
        []() {
            const auto geometry =
                passive_flight::makeAbstract500AerodynamicGeometry();

            const std::vector<passive_flight::DownwashGradientPoint>
                invalidTable{
                    {0.8, 0.50}
                };

            const passive_flight::PreliminaryAerodynamicModel model(
                geometry,
                passive_flight::makeAbstract500ZeroLiftDragTable(),
                invalidTable,
                {}
            );

            static_cast<void>(model);
        },
        "Single-point downwash table is rejected"
    );

    checkThrows<
        std::invalid_argument
    >(
        []() {
            const auto geometry =
                passive_flight::makeAbstract500AerodynamicGeometry();

            const std::vector<passive_flight::DownwashGradientPoint>
                invalidTable{
                    {0.8, 0.50},
                    {0.7, 0.55}
                };

            const passive_flight::PreliminaryAerodynamicModel model(
                geometry,
                passive_flight::makeAbstract500ZeroLiftDragTable(),
                invalidTable,
                {}
            );

            static_cast<void>(model);
        },
        "Unsorted downwash table is rejected"
    );
}

} // namespace

int main() {
    testBaselineGeometry();
    testDragTableNodes();
    testDragInterpolation();
    testZeroObjectAngleOfAttack();
    testAngleOfAttackInfluence();
    testBaselineDownwashTable();
    testCustomDownwashTable();
    testDownwashFallbackWithoutTable();
    testPitchDamping();
    testAlphaDotDisabledWithoutTable();
    testAlphaDotTableInterpolation();
    testAlphaDotTableClamping();
    testGeometryInfluence();
    testValidation();

    if (
        failureCount !=
        0
    ) {
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
