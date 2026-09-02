#include "passive_flight/TabulatedAerodynamicModel.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
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
    if (std::abs(actual - expected) >
        tolerance) {
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
    } catch (const ExceptionType&) {
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

passive_flight::TabulatedAerodynamicData
makeTestData() {
    using Point =
        passive_flight::MachCoefficientPoint;

    passive_flight::TabulatedAerodynamicData data;

    data.cx0 = {
        Point{0.5, 0.20},
        Point{1.0, 0.40}
    };

    data.cxAlphaSquared = {
        Point{0.5, 4.0},
        Point{1.0, 6.0}
    };

    data.cyAlpha = {
        Point{0.5, 3.0},
        Point{1.0, 5.0}
    };

    data.mzAlpha = {
        Point{0.5, -0.6},
        Point{1.0, -1.0}
    };

    data.mzPitchRate = {
        Point{0.5, -1.0},
        Point{1.0, -1.4}
    };

    return data;
}

void testNodeValues() {
    const passive_flight::TabulatedAerodynamicModel model(
        3.0,
        makeTestData()
    );

    passive_flight::AerodynamicInput input;

    input.mach = 0.5;
    input.angleOfAttackRad = 0.1;
    input.pitchRateRadS = 0.2;
    input.angleOfAttackRateRadS = 0.0;
    input.speedMps = 200.0;

    const auto result =
        model.evaluate(input);

    checkNear(
        result.cx0,
        0.20,
        1.0e-12,
        "Cx0 at table node"
    );

    checkNear(
        result.cxInduced,
        4.0 * 0.1 * 0.1,
        1.0e-12,
        "Cx alpha-squared contribution"
    );

    checkNear(
        result.cy,
        3.0 * 0.1,
        1.0e-12,
        "Cy at table node"
    );

    checkNear(
        result.mzStatic,
        -0.6 * 0.1,
        1.0e-12,
        "Static pitching moment"
    );

    /*
     * Проверяем именно нормировку Лебедева:
     *
     * omegaBar =
     *     omega_z * L / V =
     *     0.2 * 3 / 200 = 0.003.
     *
     * Никакого деления на 2 нет.
     */
    const double expectedOmegaBar =
        0.2 *
        3.0 /
        200.0;

    checkNear(
        result.mzPitchDamping,
        -1.0 *
        expectedOmegaBar,
        1.0e-12,
        "Lebedev pitch-rate normalization"
    );

    checkNear(
        result.mz,
        result.mzStatic +
        result.mzPitchDamping,
        1.0e-12,
        "Total pitching moment coefficient"
    );
}

void testLinearInterpolation() {
    const passive_flight::TabulatedAerodynamicModel model(
        3.0,
        makeTestData()
    );

    passive_flight::AerodynamicInput input;

    input.mach = 0.75;
    input.angleOfAttackRad = 0.2;
    input.pitchRateRadS = 0.0;
    input.speedMps = 250.0;

    const auto result =
        model.evaluate(input);

    checkNear(
        result.cx0,
        0.30,
        1.0e-12,
        "Cx0 linear interpolation"
    );

    checkNear(
        result.cyAlphaPerRad,
        4.0,
        1.0e-12,
        "CyAlpha linear interpolation"
    );

    checkNear(
        result.mzAlphaPerRad,
        -0.8,
        1.0e-12,
        "MzAlpha linear interpolation"
    );

    checkNear(
        result.mzPitchRateDerivative,
        -1.2,
        1.0e-12,
        "Pitch-rate derivative interpolation"
    );
}

void testZeroAlphaAndZeroPitchRate() {
    const passive_flight::TabulatedAerodynamicModel model(
        3.0,
        makeTestData()
    );

    passive_flight::AerodynamicInput input;

    input.mach = 0.75;
    input.angleOfAttackRad = 0.0;
    input.pitchRateRadS = 0.0;
    input.speedMps = 250.0;

    const auto result =
        model.evaluate(input);

    checkNear(
        result.cy,
        0.0,
        1.0e-12,
        "Zero alpha gives zero Cy"
    );

    checkNear(
        result.mz,
        0.0,
        1.0e-12,
        "Zero alpha and pitch rate give zero mz"
    );

    checkNear(
        result.cx,
        result.cx0,
        1.0e-12,
        "Zero alpha gives Cx equal to Cx0"
    );
}

void testStabilitySigns() {
    const passive_flight::TabulatedAerodynamicModel model(
        3.0,
        makeTestData()
    );

    passive_flight::AerodynamicInput positiveAlpha;

    positiveAlpha.mach = 0.75;
    positiveAlpha.angleOfAttackRad = 0.05;
    positiveAlpha.pitchRateRadS = 0.0;
    positiveAlpha.speedMps = 250.0;

    const auto alphaResult =
        model.evaluate(
            positiveAlpha
        );

    check(
        alphaResult.cy > 0.0,
        "Positive alpha gives positive Cy"
    );

    check(
        alphaResult.mzStatic < 0.0,
        "Negative mzAlpha gives restoring moment"
    );

    passive_flight::AerodynamicInput positivePitchRate =
        positiveAlpha;

    positivePitchRate.angleOfAttackRad =
        0.0;

    positivePitchRate.pitchRateRadS =
        0.2;

    const auto pitchRateResult =
        model.evaluate(
            positivePitchRate
        );

    check(
        pitchRateResult.mzPitchDamping < 0.0,
        "Negative pitch-rate derivative damps positive pitch rate"
    );
}

void testInvalidTablesAndRange() {
    auto data =
        makeTestData();

    data.cyAlpha.clear();

    checkThrows<std::invalid_argument>(
        [&data]() {
            const passive_flight::
                TabulatedAerodynamicModel model(
                    3.0,
                    data
                );

            static_cast<void>(model);
        },
        "Empty table is rejected"
    );

    const passive_flight::TabulatedAerodynamicModel model(
        3.0,
        makeTestData()
    );

    passive_flight::AerodynamicInput input;

    input.mach = 0.4;
    input.angleOfAttackRad = 0.0;
    input.pitchRateRadS = 0.0;
    input.speedMps = 200.0;

    checkThrows<std::out_of_range>(
        [&model, &input]() {
            static_cast<void>(
                model.evaluate(input)
            );
        },
        "Mach outside table range is rejected"
    );
}

} // namespace

int main() {
    testNodeValues();
    testLinearInterpolation();
    testZeroAlphaAndZeroPitchRate();
    testStabilitySigns();
    testInvalidTablesAndRange();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " tabulated-aerodynamic test(s) failed"
            << '\n';

        return 1;
    }

    std::cout
        << "All tabulated aerodynamic model tests passed"
        << '\n';

    return 0;
}
