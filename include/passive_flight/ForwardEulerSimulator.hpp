#pragma once

#include "passive_flight/NominalLongitudinalDynamics.hpp"
#include "passive_flight/ObjectModel.hpp"
#include "passive_flight/Types.hpp"

namespace passive_flight {

    /**
     * Решатель системы невозмущённого продольного движения
     * явным методом Эйлера.
     *
     * Внешними параметрами являются:
     *
     * - идентификатор объекта;
     * - высота сброса;
     * - скорость сброса.
     *
     * Начальные углы и угловая скорость принимаются равными нулю.
     */
    class ForwardEulerSimulator {
    public:
        explicit ForwardEulerSimulator(
            ObjectModel object
        );

        /**
         * Выполняет полный расчёт траектории.
         *
         * Метод не выбрасывает исключения из-за некорректного
         * запроса или состояния во время интегрирования.
         * Причина остановки возвращается в SimulationResult.
         */
        [[nodiscard]]
        SimulationResult simulate(
            const SimulationRequest& request,
            const SimulationOptions& options = {}
        ) const;

        [[nodiscard]]
        const ObjectModel& object() const noexcept;

        [[nodiscard]]
        const NominalLongitudinalDynamics&
        dynamics() const noexcept;

    private:
        ObjectModel object_;
        NominalLongitudinalDynamics dynamics_;
    };

} // namespace passive_flight