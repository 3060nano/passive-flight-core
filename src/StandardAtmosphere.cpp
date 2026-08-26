#include "passive_flight/Atmosphere.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace passive_flight {
namespace {

void validateParameters(
    const StandardAtmosphereParameters& parameters
) {
    if (!std::isfinite(parameters.seaLevelTemperatureK) ||
        parameters.seaLevelTemperatureK <= 0.0) {
        throw std::invalid_argument(
            "Sea-level temperature must be finite and positive"
        );
    }

    if (!std::isfinite(parameters.seaLevelPressurePa) ||
        parameters.seaLevelPressurePa <= 0.0) {
        throw std::invalid_argument(
            "Sea-level pressure must be finite and positive"
        );
    }

    if (!std::isfinite(parameters.gravityMps2) ||
        parameters.gravityMps2 <= 0.0) {
        throw std::invalid_argument(
            "Gravity must be finite and positive"
        );
    }

    if (!std::isfinite(parameters.specificGasConstantJkgK) ||
        parameters.specificGasConstantJkgK <= 0.0) {
        throw std::invalid_argument(
            "Specific gas constant must be finite and positive"
        );
    }

    if (!std::isfinite(parameters.heatCapacityRatio) ||
        parameters.heatCapacityRatio <= 1.0) {
        throw std::invalid_argument(
            "Heat capacity ratio must be finite and greater than one"
        );
    }

    if (!std::isfinite(parameters.troposphericLapseRateKPerM) ||
        parameters.troposphericLapseRateKPerM >= 0.0) {
        throw std::invalid_argument(
            "Tropospheric lapse rate must be finite and negative"
        );
    }

    if (!std::isfinite(
            parameters.troposphereTopGeometricAltitudeM
        ) ||
        parameters.troposphereTopGeometricAltitudeM <= 0.0) {
        throw std::invalid_argument(
            "Troposphere top altitude must be finite and positive"
        );
    }

    if (!std::isfinite(parameters.maximumGeometricAltitudeM) ||
        parameters.maximumGeometricAltitudeM <=
            parameters.troposphereTopGeometricAltitudeM) {
        throw std::invalid_argument(
            "Maximum altitude must be above the troposphere top"
        );
    }

    if (!std::isfinite(parameters.earthRadiusM) ||
        parameters.earthRadiusM <=
            parameters.maximumGeometricAltitudeM) {
        throw std::invalid_argument(
            "Earth radius must be greater than the maximum altitude"
        );
    }

    if (!std::isfinite(
            parameters.sutherlandReferenceTemperatureK
        ) ||
        parameters.sutherlandReferenceTemperatureK <= 0.0) {
        throw std::invalid_argument(
            "Sutherland reference temperature must be positive"
        );
    }

    if (!std::isfinite(
            parameters.sutherlandReferenceViscosityPaS
        ) ||
        parameters.sutherlandReferenceViscosityPaS <= 0.0) {
        throw std::invalid_argument(
            "Sutherland reference viscosity must be positive"
        );
    }

    if (!std::isfinite(parameters.sutherlandConstantK) ||
        parameters.sutherlandConstantK <= 0.0) {
        throw std::invalid_argument(
            "Sutherland constant must be finite and positive"
        );
    }

    const double tropopauseGeopotentialAltitudeM =
        parameters.earthRadiusM *
        parameters.troposphereTopGeometricAltitudeM /
        (
            parameters.earthRadiusM +
            parameters.troposphereTopGeometricAltitudeM
        );

    const double tropopauseTemperatureK =
        parameters.seaLevelTemperatureK +
        parameters.troposphericLapseRateKPerM *
        tropopauseGeopotentialAltitudeM;

    if (tropopauseTemperatureK <= 0.0) {
        throw std::invalid_argument(
            "Atmosphere parameters produce a non-positive "
            "tropopause temperature"
        );
    }
}

double geometricToGeopotentialAltitude(
    double geometricAltitudeM,
    double earthRadiusM
) {
    return earthRadiusM * geometricAltitudeM /
           (earthRadiusM + geometricAltitudeM);
}

double calculateDynamicViscosity(
    double temperatureK,
    const StandardAtmosphereParameters& parameters
) {
    const double temperatureRatio =
        temperatureK /
        parameters.sutherlandReferenceTemperatureK;

    return parameters.sutherlandReferenceViscosityPaS *
           std::pow(temperatureRatio, 1.5) *
           (
               parameters.sutherlandReferenceTemperatureK +
               parameters.sutherlandConstantK
           ) /
           (
               temperatureK +
               parameters.sutherlandConstantK
           );
}

} // namespace

StandardAtmosphere::StandardAtmosphere()
    : StandardAtmosphere(StandardAtmosphereParameters{}) {
}

StandardAtmosphere::StandardAtmosphere(
    const StandardAtmosphereParameters& parameters
)
    : parameters_(parameters) {
    validateParameters(parameters_);
}

AtmosphereState StandardAtmosphere::evaluate(
    double geometricAltitudeM
) const {
    if (!std::isfinite(geometricAltitudeM)) {
        throw std::invalid_argument(
            "Geometric altitude must be a finite number"
        );
    }

    if (geometricAltitudeM >
        parameters_.maximumGeometricAltitudeM) {
        throw std::out_of_range(
            "Geometric altitude exceeds the atmosphere model range"
        );
    }

    /*
     * При интегрировании траектории последний шаг метода Эйлера
     * может дать небольшую отрицательную высоту. Для атмосферы
     * такая высота приравнивается к уровню земли.
     */
    const double usedGeometricAltitudeM =
        std::max(0.0, geometricAltitudeM);

    const double geopotentialAltitudeM =
        geometricToGeopotentialAltitude(
            usedGeometricAltitudeM,
            parameters_.earthRadiusM
        );

    const double tropopauseGeopotentialAltitudeM =
        geometricToGeopotentialAltitude(
            parameters_.troposphereTopGeometricAltitudeM,
            parameters_.earthRadiusM
        );

    const double temperatureAtTropopauseK =
        parameters_.seaLevelTemperatureK +
        parameters_.troposphericLapseRateKPerM *
        tropopauseGeopotentialAltitudeM;

    /*
     * Давление на верхней границе тропосферы.
     *
     * p = p0 * (T / T0)^[-g / (L * R)]
     */
    const double pressureExponent =
        -parameters_.gravityMps2 /
        (
            parameters_.troposphericLapseRateKPerM *
            parameters_.specificGasConstantJkgK
        );

    const double pressureAtTropopausePa =
        parameters_.seaLevelPressurePa *
        std::pow(
            temperatureAtTropopauseK /
                parameters_.seaLevelTemperatureK,
            pressureExponent
        );

    double temperatureK = 0.0;
    double pressurePa = 0.0;

    if (usedGeometricAltitudeM <=
        parameters_.troposphereTopGeometricAltitudeM) {
        /*
         * Тропосфера.
         *
         * Температура линейно уменьшается:
         *
         * T = T0 + L * H.
         */
        temperatureK =
            parameters_.seaLevelTemperatureK +
            parameters_.troposphericLapseRateKPerM *
            geopotentialAltitudeM;

        /*
         * Давление в слое с постоянным температурным
         * градиентом:
         *
         * p = p0 * (T / T0)^[-g / (L * R)].
         */
        pressurePa =
            parameters_.seaLevelPressurePa *
            std::pow(
                temperatureK /
                    parameters_.seaLevelTemperatureK,
                pressureExponent
            );
    } else {
        /*
         * Нижняя стратосфера в пределах текущей модели
         * принимается изотермической:
         *
         * T = const.
         */
        temperatureK = temperatureAtTropopauseK;

        /*
         * Давление в изотермическом слое:
         *
         * p = p11 * exp[
         *     -g * (H - H11) / (R * T11)
         * ].
         */
        pressurePa =
            pressureAtTropopausePa *
            std::exp(
                -parameters_.gravityMps2 *
                (
                    geopotentialAltitudeM -
                    tropopauseGeopotentialAltitudeM
                ) /
                (
                    parameters_.specificGasConstantJkgK *
                    temperatureAtTropopauseK
                )
            );
    }

    /*
     * Уравнение состояния идеального газа:
     *
     * rho = p / (R * T).
     */
    const double densityKgM3 =
        pressurePa /
        (
            parameters_.specificGasConstantJkgK *
            temperatureK
        );

    /*
     * Скорость звука:
     *
     * a = sqrt(k * R * T).
     */
    const double speedOfSoundMps =
        std::sqrt(
            parameters_.heatCapacityRatio *
            parameters_.specificGasConstantJkgK *
            temperatureK
        );

    /*
     * Динамическая вязкость вычисляется
     * по формуле Сазерленда.
     */
    const double dynamicViscosityPaS =
        calculateDynamicViscosity(
            temperatureK,
            parameters_
        );

    AtmosphereState result;

    result.geometricAltitudeM =
        usedGeometricAltitudeM;

    result.geopotentialAltitudeM =
        geopotentialAltitudeM;

    result.temperatureK =
        temperatureK;

    result.pressurePa =
        pressurePa;

    result.densityKgM3 =
        densityKgM3;

    result.speedOfSoundMps =
        speedOfSoundMps;

    result.dynamicViscosityPaS =
        dynamicViscosityPaS;

    return result;
}

const StandardAtmosphereParameters&
StandardAtmosphere::parameters() const noexcept {
    return parameters_;
}

} // namespace passive_flight