#include "passive_flight/Atmosphere.hpp"

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

void testSeaLevel() {
    const passive_flight::StandardAtmosphere atmosphere;
    const auto state = atmosphere.evaluate(0.0);

    checkNear(
        state.geometricAltitudeM,
        0.0,
        1.0e-12,
        "Sea-level geometric altitude"
    );

    checkNear(
        state.geopotentialAltitudeM,
        0.0,
        1.0e-12,
        "Sea-level geopotential altitude"
    );

    checkNear(
        state.temperatureK,
        288.15,
        1.0e-10,
        "Sea-level temperature"
    );

    checkNear(
        state.pressurePa,
        101325.0,
        1.0e-6,
        "Sea-level pressure"
    );

    checkNear(
        state.densityKgM3,
        1.225,
        5.0e-4,
        "Sea-level density"
    );

    checkNear(
        state.speedOfSoundMps,
        340.294,
        0.01,
        "Sea-level speed of sound"
    );

    checkNear(
        state.dynamicViscosityPaS,
        1.789e-5,
        5.0e-8,
        "Sea-level dynamic viscosity"
    );
}

void testNegativeAltitudeIsClampedToGround() {
    const passive_flight::StandardAtmosphere atmosphere;

    const auto belowGround =
        atmosphere.evaluate(-0.25);

    const auto atGround =
        atmosphere.evaluate(0.0);

    checkNear(
        belowGround.geometricAltitudeM,
        0.0,
        1.0e-12,
        "Negative altitude is clamped to zero"
    );

    checkNear(
        belowGround.densityKgM3,
        atGround.densityKgM3,
        1.0e-12,
        "Below-ground density equals ground density"
    );

    checkNear(
        belowGround.pressurePa,
        atGround.pressurePa,
        1.0e-9,
        "Below-ground pressure equals ground pressure"
    );
}

void testAtmosphereChangesWithAltitude() {
    const passive_flight::StandardAtmosphere atmosphere;

    const auto seaLevel =
        atmosphere.evaluate(0.0);

    const auto fiveKilometres =
        atmosphere.evaluate(5000.0);

    const auto tenKilometres =
        atmosphere.evaluate(10000.0);

    const auto twentyKilometres =
        atmosphere.evaluate(20000.0);

    check(
        fiveKilometres.temperatureK <
            seaLevel.temperatureK,
        "Temperature decreases between 0 and 5 km"
    );

    check(
        tenKilometres.temperatureK <
            fiveKilometres.temperatureK,
        "Temperature decreases between 5 and 10 km"
    );

    checkNear(
        twentyKilometres.temperatureK,
        atmosphere.evaluate(11000.0).temperatureK,
        1.0e-10,
        "Lower stratosphere is isothermal"
    );

    check(
        fiveKilometres.pressurePa <
            seaLevel.pressurePa,
        "Pressure decreases between 0 and 5 km"
    );

    check(
        tenKilometres.pressurePa <
            fiveKilometres.pressurePa,
        "Pressure decreases between 5 and 10 km"
    );

    check(
        twentyKilometres.pressurePa <
            tenKilometres.pressurePa,
        "Pressure decreases between 10 and 20 km"
    );

    check(
        fiveKilometres.densityKgM3 <
            seaLevel.densityKgM3,
        "Density decreases between 0 and 5 km"
    );

    check(
        tenKilometres.densityKgM3 <
            fiveKilometres.densityKgM3,
        "Density decreases between 5 and 10 km"
    );

    check(
        twentyKilometres.densityKgM3 <
            tenKilometres.densityKgM3,
        "Density decreases between 10 and 20 km"
    );

    check(
        twentyKilometres.speedOfSoundMps <
            seaLevel.speedOfSoundMps,
        "Speed of sound at 20 km is below sea-level value"
    );

    check(
        twentyKilometres.dynamicViscosityPaS <
            seaLevel.dynamicViscosityPaS,
        "Dynamic viscosity decreases with temperature"
    );
}

void testGeopotentialAltitude() {
    const passive_flight::StandardAtmosphere atmosphere;
    const auto state = atmosphere.evaluate(10000.0);

    check(
        state.geopotentialAltitudeM > 0.0,
        "Geopotential altitude is positive"
    );

    check(
        state.geopotentialAltitudeM <
            state.geometricAltitudeM,
        "Geopotential altitude is below geometric altitude"
    );

    checkNear(
        state.geopotentialAltitudeM,
        9984.3,
        1.0,
        "Geopotential altitude at 10 km"
    );
}

void testTropopauseContinuity() {
    const passive_flight::StandardAtmosphere atmosphere;

    const auto below =
        atmosphere.evaluate(10999.999);

    const auto above =
        atmosphere.evaluate(11000.001);

    checkNear(
        below.temperatureK,
        above.temperatureK,
        1.0e-4,
        "Temperature is continuous at the tropopause"
    );

    checkNear(
        below.pressurePa,
        above.pressurePa,
        0.1,
        "Pressure is continuous at the tropopause"
    );

    checkNear(
        below.densityKgM3,
        above.densityKgM3,
        1.0e-5,
        "Density is continuous at the tropopause"
    );
}

void testRangeValidation() {
    const passive_flight::StandardAtmosphere atmosphere;

    const auto upperBoundary =
        atmosphere.evaluate(25000.0);

    check(
        upperBoundary.densityKgM3 > 0.0,
        "Maximum supported altitude can be evaluated"
    );

    checkThrows<std::out_of_range>(
        [&atmosphere]() {
            atmosphere.evaluate(25000.001);
        },
        "Altitude above model range is rejected"
    );

    checkThrows<std::invalid_argument>(
        [&atmosphere]() {
            atmosphere.evaluate(
                std::numeric_limits<double>::quiet_NaN()
            );
        },
        "NaN altitude is rejected"
    );

    checkThrows<std::invalid_argument>(
        [&atmosphere]() {
            atmosphere.evaluate(
                std::numeric_limits<double>::infinity()
            );
        },
        "Infinite altitude is rejected"
    );
}

void testParameterValidation() {
    auto parameters =
        passive_flight::StandardAtmosphereParameters{};

    parameters.seaLevelTemperatureK = 0.0;

    checkThrows<std::invalid_argument>(
        [&parameters]() {
            const passive_flight::StandardAtmosphere atmosphere(
                parameters
            );

            static_cast<void>(atmosphere);
        },
        "Non-positive sea-level temperature is rejected"
    );

    parameters =
        passive_flight::StandardAtmosphereParameters{};

    parameters.troposphericLapseRateKPerM = 0.0065;

    checkThrows<std::invalid_argument>(
        [&parameters]() {
            const passive_flight::StandardAtmosphere atmosphere(
                parameters
            );

            static_cast<void>(atmosphere);
        },
        "Positive tropospheric lapse rate is rejected"
    );

    parameters =
        passive_flight::StandardAtmosphereParameters{};

    parameters.maximumGeometricAltitudeM = 10000.0;

    checkThrows<std::invalid_argument>(
        [&parameters]() {
            const passive_flight::StandardAtmosphere atmosphere(
                parameters
            );

            static_cast<void>(atmosphere);
        },
        "Maximum altitude below tropopause is rejected"
    );
}

} // namespace

int main() {
    testSeaLevel();
    testNegativeAltitudeIsClampedToGround();
    testAtmosphereChangesWithAltitude();
    testGeopotentialAltitude();
    testTropopauseContinuity();
    testRangeValidation();
    testParameterValidation();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " atmosphere test(s) failed"
            << '\n';

        return 1;
    }

    std::cout
        << "All atmosphere tests passed"
        << '\n';

    return 0;
}