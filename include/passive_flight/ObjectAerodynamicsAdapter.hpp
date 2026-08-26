#pragma once

#include "passive_flight/Aerodynamics.hpp"
#include "passive_flight/ObjectModel.hpp"

namespace passive_flight {

    /**
     * Преобразует полный паспорт геометрии объекта
     * в набор параметров аэродинамической модели.
     *
     * Адаптер позволяет не хранить одни и те же размеры
     * отдельно в ObjectModel и AerodynamicGeometry.
     */
    [[nodiscard]]
    AerodynamicGeometry makeAerodynamicGeometry(
        const ObjectModel& object
    );

    /**
     * Создаёт готовую аэродинамическую модель
     * непосредственно из паспорта объекта.
     */
    [[nodiscard]]
    PreliminaryAerodynamicModel makeAerodynamicModel(
        const ObjectModel& object
    );

} // namespace passive_flight