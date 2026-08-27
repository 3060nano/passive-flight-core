#pragma once

#include "passive_flight/PerturbedTrajectorySimulator.hpp"

#include <iosfwd>

namespace passive_flight {

    /**
     * Записывает историю возмущённого движения в CSV.
     *
     * Используется разделитель ';', чтобы файл без ручной
     * настройки открывался в русской локали Excel.
     * Все размерные значения записываются в единицах СИ,
     * углы - в радианах.
     */
    void writePerturbedTrajectoryCsv(
        const PerturbedSimulationResult& result,
        std::ostream& output
    );

} // namespace passive_flight
