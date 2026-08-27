#pragma once

#include "passive_flight/NominalLongitudinalDynamics.hpp"
#include "passive_flight/ObjectModel.hpp"
#include "passive_flight/Types.hpp"

#include <array>
#include <cstddef>

namespace passive_flight {

inline constexpr std::size_t
    kLongitudinalStateDimension = 6;

enum class LongitudinalStateIndex : std::size_t {
    Speed = 0,
    FlightPathAngle = 1,
    PitchRate = 2,
    PitchAngle = 3,
    Downrange = 4,
    Altitude = 5
};

using LongitudinalStateVector =
    std::array<double, kLongitudinalStateDimension>;

using LongitudinalSystemMatrix =
    std::array<
        LongitudinalStateVector,
        kLongitudinalStateDimension
    >;

/**
 * Малые отклонения от невозмущённой траектории.
 *
 * Порядок компонент совпадает с State:
 *
 * [Delta V, Delta Theta, Delta omega_z,
 *  Delta theta, Delta x, Delta H].
 */
struct LongitudinalPerturbationState {
    double speedMps{0.0};
    double flightPathAngleRad{0.0};
    double pitchRateRadps{0.0};
    double pitchAngleRad{0.0};
    double downrangeM{0.0};
    double altitudeM{0.0};
};

/**
 * Производные малых отклонений.
 */
struct LongitudinalPerturbationDerivative {
    double speedMps2{0.0};
    double flightPathAngleRadps{0.0};
    double pitchRateRadps2{0.0};
    double pitchAngleRadps{0.0};
    double downrangeMps{0.0};
    double altitudeMps{0.0};
};

/**
 * Шаги численного дифференцирования правой части.
 *
 * Шаги являются размерными и задаются в СИ.
 */
struct PerturbationLinearizationOptions {
    double speedStepMps{1.0e-3};
    double angleStepRad{1.0e-7};
    double pitchRateStepRadps{1.0e-7};
    double positionStepM{1.0e-3};
    double altitudeStepM{1.0e-3};
};

struct PerturbedLongitudinalEvaluation {
    LongitudinalSystemMatrix systemMatrix{};
    LongitudinalPerturbationDerivative derivative{};
};

/**
 * Линеаризованная система продольного возмущённого движения.
 *
 * Для текущего состояния невозмущённой траектории x*(t)
 * вычисляется матрица Якоби:
 *
 *     A(t) = df/dx | x*(t),
 *
 * после чего малое возмущение определяется системой:
 *
 *     Delta x dot = A(t) * Delta x.
 */
class PerturbedLongitudinalDynamics {
public:
    explicit PerturbedLongitudinalDynamics(
        ObjectModel object,
        PerturbationLinearizationOptions options = {}
    );

    [[nodiscard]]
    LongitudinalSystemMatrix linearize(
        const State& nominalState
    ) const;

    [[nodiscard]]
    PerturbedLongitudinalEvaluation evaluate(
        const State& nominalState,
        const LongitudinalPerturbationState& perturbation
    ) const;

    [[nodiscard]]
    const NominalLongitudinalDynamics&
    nominalDynamics() const noexcept;

    [[nodiscard]]
    const PerturbationLinearizationOptions&
    options() const noexcept;

private:
    NominalLongitudinalDynamics nominalDynamics_;
    PerturbationLinearizationOptions options_;
};

} // namespace passive_flight
