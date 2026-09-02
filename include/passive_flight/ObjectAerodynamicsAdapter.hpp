#pragma once

#include "passive_flight/Aerodynamics.hpp"
#include "passive_flight/ObjectModel.hpp"

#include <memory>

namespace passive_flight {

    /**
     * Преобразует полный паспорт геометрии объекта
     * в набор параметров текущей предварительной
     * аэродинамической модели.
     */
    [[nodiscard]]
    AerodynamicGeometry makeAerodynamicGeometry(
        const ObjectModel& object
    );

    /**
     * Создаёт аэродинамическую модель объекта.
     *
     * Пока существующий ABSTRACT_500_UMPK_V1 продолжает
     * использовать PreliminaryAerodynamicModel.
     *
     * Возврат через общий интерфейс нужен, чтобы динамика
     * не зависела от способа получения коэффициентов.
     */
    [[nodiscard]]
    std::shared_ptr<const AerodynamicModel>
    makeAerodynamicModel(
        const ObjectModel& object
    );

} // namespace passive_flight
