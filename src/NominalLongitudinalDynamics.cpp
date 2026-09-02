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

    const double referenceLengthM =
        object_.reference
            .effectiveReferenceLengthM();

    if (!std::isfinite(referenceLengthM) ||
        referenceLengthM <= 0.0) {
        throw std::invalid_argument(
            "Reference length must be finite and positive"
        );
    }

    if (!aerodynamics_) {
        throw std::invalid_argument(
            "Aerodynamic model must not be null"
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

    const AtmosphereState atmosphereState =
        atmosphere_.evaluate(
            altitudeM
        );

    const double mach =
        speedMps /
        atmosphereState.speedOfSoundMps;

    const double dynamicPressurePa =
        0.5 *
        atmosphereState.densityKgM3 *
        speedMps *
        speedMps;

    const double angleOfAttackRad =
        state.angleOfAttackRad();

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
            aerodynamics_->evaluate(
                preliminaryInput
            );

    const double preliminaryNormalForceN =
        dynamicPressurePa *
        object_.reference.areaM2 *
        preliminaryAerodynamics.cy;

    const double gravityMps2 =
        atmosphere_.parameters().gravityMps2;

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

    const double angleOfAttackRateRadS =
        pitchRateRadS -
        trajectoryAngleDerivativeRadS;

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
            aerodynamics_->evaluate(
                finalInput
            );

    LongitudinalAerodynamicLoads loads;

    loads.dragN =
        dynamicPressurePa *
        object_.reference.areaM2 *
        aerodynamicCoefficients.cx;

    loads.normalForceN =
        dynamicPressurePa *
        object_.reference.areaM2 *
        aerodynamicCoefficients.cy;

    /*
     * Универсальная размеризация момента:
     *
     *     Mz = q * S_ref * L_ref * mz.
     *
     * Для текущего ABSTRACT_500 L_ref временно
     * автоматически равна САХ крыла.
     *
     * Для ФАБ-1500Т на следующем этапе
     * L_ref будет равна длине бомбы.
     */
    loads.pitchingMomentNm =
        dynamicPressurePa *
        object_.reference.areaM2 *
        object_.reference
            .effectiveReferenceLengthM() *
        aerodynamicCoefficients.mz;

    const double speedDerivativeMps2 =
        -loads.dragN / massKg -
        gravityMps2 *
        std::sin(trajectoryAngleRad);

    const double pitchRateDerivativeRadS2 =
        loads.pitchingMomentNm /
        pitchMomentOfInertiaKgM2;

    const double pitchAngleDerivativeRadS =
        pitchRateRadS;

    const double downrangeDerivativeMps =
        speedMps *
        std::cos(trajectoryAngleRad);

    const double altitudeDerivativeMps =
        speedMps *
        std::sin(trajectoryAngleRad);

    LongitudinalDynamicsEvaluation result;

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

const AerodynamicModel&
NominalLongitudinalDynamics::aerodynamics() const noexcept {
    return *aerodynamics_;
}

} // namespace passive_flight
