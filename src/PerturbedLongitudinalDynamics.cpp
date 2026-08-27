#include "passive_flight/PerturbedLongitudinalDynamics.hpp"

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace passive_flight {
namespace {

using DerivativeVector = LongitudinalStateVector;

constexpr std::size_t indexOf(
    LongitudinalStateIndex index
) noexcept {
    return static_cast<std::size_t>(index);
}

void requirePositiveFinite(
    double value,
    const char* name
) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(
            std::string(name) +
            " must be finite and positive"
        );
    }
}

void validateOptions(
    const PerturbationLinearizationOptions& options
) {
    requirePositiveFinite(
        options.speedStepMps,
        "Speed differentiation step"
    );

    requirePositiveFinite(
        options.angleStepRad,
        "Angle differentiation step"
    );

    requirePositiveFinite(
        options.pitchRateStepRadps,
        "Pitch-rate differentiation step"
    );

    requirePositiveFinite(
        options.positionStepM,
        "Position differentiation step"
    );

    requirePositiveFinite(
        options.altitudeStepM,
        "Altitude differentiation step"
    );
}

void validatePerturbation(
    const LongitudinalPerturbationState& perturbation
) {
    const std::array<double, kLongitudinalStateDimension>
        values{
            perturbation.speedMps,
            perturbation.flightPathAngleRad,
            perturbation.pitchRateRadps,
            perturbation.pitchAngleRad,
            perturbation.downrangeM,
            perturbation.altitudeM
        };

    for (const double value : values) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument(
                "Perturbation components must be finite"
            );
        }
    }
}

DerivativeVector toVector(
    const StateDerivative& derivative
) {
    return {
        derivative.speedMps2,
        derivative.flightPathAngleRadps,
        derivative.pitchRateRadps2,
        derivative.pitchAngleRadps,
        derivative.downrangeMps,
        derivative.altitudeMps
    };
}

LongitudinalStateVector toVector(
    const LongitudinalPerturbationState& perturbation
) {
    return {
        perturbation.speedMps,
        perturbation.flightPathAngleRad,
        perturbation.pitchRateRadps,
        perturbation.pitchAngleRad,
        perturbation.downrangeM,
        perturbation.altitudeM
    };
}

LongitudinalPerturbationDerivative toDerivative(
    const LongitudinalStateVector& values
) {
    return {
        values[indexOf(LongitudinalStateIndex::Speed)],
        values[indexOf(
            LongitudinalStateIndex::FlightPathAngle
        )],
        values[indexOf(LongitudinalStateIndex::PitchRate)],
        values[indexOf(LongitudinalStateIndex::PitchAngle)],
        values[indexOf(LongitudinalStateIndex::Downrange)],
        values[indexOf(LongitudinalStateIndex::Altitude)]
    };
}

void addStateIncrement(
    State& state,
    LongitudinalStateIndex index,
    double increment
) {
    switch (index) {
    case LongitudinalStateIndex::Speed:
        state.speedMps += increment;
        break;
    case LongitudinalStateIndex::FlightPathAngle:
        state.flightPathAngleRad += increment;
        break;
    case LongitudinalStateIndex::PitchRate:
        state.pitchRateRadps += increment;
        break;
    case LongitudinalStateIndex::PitchAngle:
        state.pitchAngleRad += increment;
        break;
    case LongitudinalStateIndex::Downrange:
        state.downrangeM += increment;
        break;
    case LongitudinalStateIndex::Altitude:
        state.altitudeM += increment;
        break;
    }
}

double differentiationStep(
    const PerturbationLinearizationOptions& options,
    LongitudinalStateIndex index
) {
    switch (index) {
    case LongitudinalStateIndex::Speed:
        return options.speedStepMps;
    case LongitudinalStateIndex::FlightPathAngle:
    case LongitudinalStateIndex::PitchAngle:
        return options.angleStepRad;
    case LongitudinalStateIndex::PitchRate:
        return options.pitchRateStepRadps;
    case LongitudinalStateIndex::Downrange:
        return options.positionStepM;
    case LongitudinalStateIndex::Altitude:
        return options.altitudeStepM;
    }

    throw std::logic_error(
        "Unknown longitudinal state index"
    );
}

std::optional<DerivativeVector> tryEvaluate(
    const NominalLongitudinalDynamics& dynamics,
    const State& state
) {
    try {
        return toVector(
            dynamics.evaluate(state).derivative
        );
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

void fillMatrixColumn(
    LongitudinalSystemMatrix& matrix,
    std::size_t column,
    const DerivativeVector& numerator,
    double denominator
) {
    for (std::size_t row = 0;
         row < kLongitudinalStateDimension;
         ++row) {
        matrix[row][column] =
            numerator[row] / denominator;
    }
}

DerivativeVector subtract(
    const DerivativeVector& first,
    const DerivativeVector& second
) {
    DerivativeVector result{};

    for (std::size_t index = 0;
         index < kLongitudinalStateDimension;
         ++index) {
        result[index] = first[index] - second[index];
    }

    return result;
}

} // namespace

PerturbedLongitudinalDynamics::
PerturbedLongitudinalDynamics(
    ObjectModel object,
    PerturbationLinearizationOptions options
)
    : nominalDynamics_(std::move(object)),
      options_(options) {
    validateOptions(options_);
}

LongitudinalSystemMatrix
PerturbedLongitudinalDynamics::linearize(
    const State& nominalState
) const {
    const DerivativeVector nominalDerivative =
        toVector(
            nominalDynamics_
                .evaluate(nominalState)
                .derivative
        );

    LongitudinalSystemMatrix matrix{};

    for (std::size_t column = 0;
         column < kLongitudinalStateDimension;
         ++column) {
        const auto stateIndex =
            static_cast<LongitudinalStateIndex>(column);

        const double step =
            differentiationStep(
                options_,
                stateIndex
            );

        State plusState = nominalState;
        State minusState = nominalState;

        addStateIncrement(
            plusState,
            stateIndex,
            step
        );

        addStateIncrement(
            minusState,
            stateIndex,
            -step
        );

        const auto plusDerivative =
            tryEvaluate(
                nominalDynamics_,
                plusState
            );

        const auto minusDerivative =
            tryEvaluate(
                nominalDynamics_,
                minusState
            );

        if (plusDerivative && minusDerivative) {
            fillMatrixColumn(
                matrix,
                column,
                subtract(
                    *plusDerivative,
                    *minusDerivative
                ),
                2.0 * step
            );
            continue;
        }

        if (plusDerivative) {
            fillMatrixColumn(
                matrix,
                column,
                subtract(
                    *plusDerivative,
                    nominalDerivative
                ),
                step
            );
            continue;
        }

        if (minusDerivative) {
            fillMatrixColumn(
                matrix,
                column,
                subtract(
                    nominalDerivative,
                    *minusDerivative
                ),
                step
            );
            continue;
        }

        throw std::runtime_error(
            "Cannot evaluate a longitudinal "
            "linearization column"
        );
    }

    return matrix;
}

PerturbedLongitudinalEvaluation
PerturbedLongitudinalDynamics::evaluate(
    const State& nominalState,
    const LongitudinalPerturbationState& perturbation
) const {
    validatePerturbation(perturbation);

    PerturbedLongitudinalEvaluation result;

    result.systemMatrix =
        linearize(nominalState);

    const LongitudinalStateVector perturbationVector =
        toVector(perturbation);

    LongitudinalStateVector derivativeVector{};

    for (std::size_t row = 0;
         row < kLongitudinalStateDimension;
         ++row) {
        for (std::size_t column = 0;
             column < kLongitudinalStateDimension;
             ++column) {
            derivativeVector[row] +=
                result.systemMatrix[row][column] *
                perturbationVector[column];
        }
    }

    result.derivative =
        toDerivative(derivativeVector);

    return result;
}

const NominalLongitudinalDynamics&
PerturbedLongitudinalDynamics::
nominalDynamics() const noexcept {
    return nominalDynamics_;
}

const PerturbationLinearizationOptions&
PerturbedLongitudinalDynamics::options() const noexcept {
    return options_;
}

} // namespace passive_flight
