#pragma once

#include "passive_flight/ForwardEulerSimulator.hpp"
#include "passive_flight/ObjectModel.hpp"
#include "passive_flight/PerturbedLongitudinalDynamics.hpp"
#include "passive_flight/Types.hpp"

#include <vector>

namespace passive_flight {

/**
 * Одна точка совместной истории невозмущённого
 * и возмущённого движения.
 */
struct PerturbedTrajectorySample {
    State nominalState;
    LongitudinalPerturbationState perturbation;
    State totalState;
};

/**
 * Результат интегрирования малых возмущений
 * вдоль невозмущённой траектории.
 */
struct PerturbedSimulationResult {
    TerminationReason terminationReason{
        TerminationReason::InvalidState
    };

    State finalNominalState;
    LongitudinalPerturbationState finalPerturbation;
    State finalTotalState;

    std::vector<PerturbedTrajectorySample> history;
};

/**
 * Решатель линейного продольного возмущённого движения.
 *
 * Сначала рассчитывается невозмущённая траектория x*(t).
 * Затем по тем же временным узлам явным методом Эйлера
 * интегрируется система:
 *
 *     Delta x dot = A(t) * Delta x.
 *
 * Полное состояние для анализа формируется как:
 *
 *     x(t) = x*(t) + Delta x(t).
 */
class PerturbedTrajectorySimulator {
public:
    explicit PerturbedTrajectorySimulator(
        ObjectModel object,
        PerturbationLinearizationOptions
            linearizationOptions = {}
    );

    [[nodiscard]]
    PerturbedSimulationResult simulate(
        const SimulationRequest& request,
        const LongitudinalPerturbationState&
            initialPerturbation,
        const SimulationOptions& options = {}
    ) const;

    [[nodiscard]]
    const ObjectModel& object() const noexcept;

    [[nodiscard]]
    const ForwardEulerSimulator&
    nominalSimulator() const noexcept;

    [[nodiscard]]
    const PerturbedLongitudinalDynamics&
    perturbationDynamics() const noexcept;

private:
    ObjectModel object_;
    ForwardEulerSimulator nominalSimulator_;
    PerturbedLongitudinalDynamics perturbationDynamics_;
};

} // namespace passive_flight
