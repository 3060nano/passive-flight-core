#include "passive_flight/NominalLongitudinalDynamics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace passive_flight {
namespace {

void requireFinite(
    double value,
    const char* parameterName
) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(
            std::string(parameterName) +
            " must be finite"
        );
    }
}

void validateState(
    const State& state
) {
    requireFinite(
        state.timeS,
        "Time"
    );

    requireFinite(
        state.speedMps,
        "Speed"
    );

    requireFinite(
        state.flightPathAngleRad,
        "Flight-path angle"
    );

    requireFinite(
        state.pitchRateRadps,
        "Pitch rate"
    );

    requireFinite(
        state.pitchAngleRad,
        "Pitch angle"
    );

    requireFinite(
        state.downrangeM,
        "Downrange"
    );

    requireFinite(
        state.altitudeM,
        "Altitude"
    );

    if (state.timeS < 0.0) {
        throw std::invalid_argument(
            "Time must be non-negative"
        );
    }

    if (state.speedMps <= 0.0) {
        throw std::invalid_argument(
            "Speed must be positive"
        );
    }
}

} // namespace

NominalLongitudinalDynamics::
NominalLongitudinalDynamics(
    ObjectModel object
)
    : NominalLongitudinalDynamics(
          std::move(object),
          StandardAtmosphere{}
      ) {
}

NominalLongitudinalDynamics::
NominalLongitudinalDynamics(
    ObjectModel object,
    StandardAtmosphere atmosphere
)
    : object_(std::move(object)),
      atmosphere_(std::move(atmosphere)),
      aerodynamics_(
          makeAerodynamicModel(object_)
      ) {
    /*
     * Структура MassProperties содержит:
     *
     * massKg;
     * pitchMomentOfInertiaKgM2;
     * centerOfMassXM.
     */
    const auto& [
        massKg,
        pitchMomentOfInertiaKgM2,
        centerOfMassXM
    ] = object_.mass;

    static_cast<void>(centerOfMassXM);

    if (!std::isfinite(massKg) ||
        massKg <= 0.0) {
        throw std::invalid_argument(
            "Object mass must be finite and positive"
        );
    }

    if (!std::isfinite(
            pitchMomentOfInertiaKgM2
        ) ||
        pitchMomentOfInertiaKgM2 <= 0.0) {
        throw std::invalid_argument(
            "Pitch moment of inertia must be finite and positive"
        );
    }
}

LongitudinalDynamicsEvaluation
NominalLongitudinalDynamics::evaluate(
    const State& state
) const {
    validateState(state);

    const double speedMps =
        state.speedMps;

    const double trajectoryAngleRad =
        state.flightPathAngleRad;

    const double pitchRateRadS =
        state.pitchRateRadps;

    const double altitudeM =
        state.altitudeM;

    const auto& [
        massKg,
        pitchMomentOfInertiaKgM2,
        centerOfMassXM
    ] = object_.mass;

    static_cast<void>(centerOfMassXM);

    /*
     * Параметры атмосферы вычисляются
     * по текущей геометрической высоте.
     */
    const AtmosphereState atmosphereState =
        atmosphere_.evaluate(
            altitudeM
        );

    /*
     * Число Маха:
     *
     * M = V / a.
     */
    const double mach =
        speedMps /
        atmosphereState.speedOfSoundMps;

    /*
     * Скоростной напор:
     *
     * q = rho * V^2 / 2.
     */
    const double dynamicPressurePa =
        0.5 *
        atmosphereState.densityKgM3 *
        speedMps *
        speedMps;

    /*
     * Угол атаки:
     *
     * alpha = theta - Theta.
     */
    const double angleOfAttackRad =
        state.angleOfAttackRad();

    /*
     * Предварительный аэродинамический расчёт
     * без производной угла атаки.
     *
     * Он необходим для определения Y
     * и производной угла траектории.
     */
    AerodynamicInput preliminaryInput;

    preliminaryInput.mach =
        mach;

    preliminaryInput.angleOfAttackRad =
        angleOfAttackRad;

    preliminaryInput.pitchRateRadS =
        pitchRateRadS;

    preliminaryInput.angleOfAttackRateRadS =
        0.0;

    preliminaryInput.speedMps =
        speedMps;

    const AerodynamicCoefficients
        preliminaryAerodynamics =
            aerodynamics_.evaluate(
                preliminaryInput
            );

    const double preliminaryNormalForceN =
        dynamicPressurePa *
        object_.reference.areaM2 *
        preliminaryAerodynamics.cy;

    const double gravityMps2 =
        atmosphere_.parameters().gravityMps2;

    /*
     * Уравнение движения по нормали
     * к вектору скорости:
     *
     * m * V * dTheta/dt =
     *     Y - m * g * cos(Theta).
     */
    const double trajectoryAngleDerivativeRadS =
        (
            preliminaryNormalForceN -
            massKg *
            gravityMps2 *
            std::cos(trajectoryAngleRad)
        ) /
        (
            massKg *
            speedMps
        );

    /*
     * alpha = theta - Theta,
     *
     * поэтому:
     *
     * alpha_dot = omega_z - Theta_dot.
     */
    const double angleOfAttackRateRadS =
        pitchRateRadS -
        trajectoryAngleDerivativeRadS;

    /*
     * Окончательный аэродинамический расчёт.
     *
     * Теперь в коэффициент момента входят:
     *
     * - статический момент;
     * - момент по omega_z;
     * - момент по alpha_dot.
     */
    AerodynamicInput finalInput;

    finalInput.mach =
        mach;

    finalInput.angleOfAttackRad =
        angleOfAttackRad;

    finalInput.pitchRateRadS =
        pitchRateRadS;

    finalInput.angleOfAttackRateRadS =
        angleOfAttackRateRadS;

    finalInput.speedMps =
        speedMps;

    const AerodynamicCoefficients
        aerodynamicCoefficients =
            aerodynamics_.evaluate(
                finalInput
            );

    /*
     * Размерные аэродинамические нагрузки:
     *
     * X  = q * S * cx;
     * Y  = q * S * cy;
     * Mz = q * S * b_A * mz.
     */
    LongitudinalAerodynamicLoads loads;

    loads.dragN =
        dynamicPressurePa *
        object_.reference.areaM2 *
        aerodynamicCoefficients.cx;

    loads.normalForceN =
        dynamicPressurePa *
        object_.reference.areaM2 *
        aerodynamicCoefficients.cy;

    loads.pitchingMomentNm =
        dynamicPressurePa *
        object_.reference.areaM2 *
        object_.reference.meanAerodynamicChordM *
        aerodynamicCoefficients.mz;

    /*
     * Уравнение движения вдоль вектора скорости:
     *
     * m * dV/dt =
     *     -X - m * g * sin(Theta).
     */
    const double speedDerivativeMps2 =
        -loads.dragN / massKg -
        gravityMps2 *
        std::sin(trajectoryAngleRad);

    /*
     * Уравнение вращательного движения:
     *
     * Iz * domega_z/dt = Mz.
     */
    const double pitchRateDerivativeRadS2 =
        loads.pitchingMomentNm /
        pitchMomentOfInertiaKgM2;

    /*
     * Кинематическое уравнение тангажа:
     *
     * dtheta/dt = omega_z.
     */
    const double pitchAngleDerivativeRadS =
        pitchRateRadS;

    /*
     * Горизонтальная составляющая скорости:
     *
     * dx/dt = V * cos(Theta).
     */
    const double downrangeDerivativeMps =
        speedMps *
        std::cos(trajectoryAngleRad);

    /*
     * Вертикальная составляющая скорости:
     *
     * dH/dt = V * sin(Theta).
     */
    const double altitudeDerivativeMps =
        speedMps *
        std::sin(trajectoryAngleRad);

    LongitudinalDynamicsEvaluation result;

    /*
     * StateDerivative не содержит времени.
     *
     * Его компоненты:
     *
     * [
     *     dV/dt,
     *     dTheta/dt,
     *     domega_z/dt,
     *     dtheta/dt,
     *     dx/dt,
     *     dH/dt
     * ].
     */
    result.derivative = StateDerivative{
        speedDerivativeMps2,
        trajectoryAngleDerivativeRadS,
        pitchRateDerivativeRadS2,
        pitchAngleDerivativeRadS,
        downrangeDerivativeMps,
        altitudeDerivativeMps
    };

    result.atmosphere =
        atmosphereState;

    result.aerodynamics =
        aerodynamicCoefficients;

    result.loads =
        loads;

    result.mach =
        mach;

    result.dynamicPressurePa =
        dynamicPressurePa;

    result.angleOfAttackRad =
        angleOfAttackRad;

    result.angleOfAttackRateRadS =
        angleOfAttackRateRadS;

    return result;
}

const ObjectModel&
NominalLongitudinalDynamics::object() const noexcept {
    return object_;
}

const StandardAtmosphere&
NominalLongitudinalDynamics::atmosphere() const noexcept {
    return atmosphere_;
}

const PreliminaryAerodynamicModel&
NominalLongitudinalDynamics::aerodynamics() const noexcept {
    return aerodynamics_;
}

} // namespace passive_flight