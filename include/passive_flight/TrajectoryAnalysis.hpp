#pragma once

#include "passive_flight/ObjectModel.hpp"
#include "passive_flight/Types.hpp"

#include <cstddef>

namespace passive_flight {

/**
 * Результат анализа полной траектории.
 */
struct TrajectoryAnalysis {
    bool available{false};
    std::size_t sampleCount{0};

    double releaseAltitudeM{0.0};
    double maximumAltitudeM{0.0};
    double altitudeGainM{0.0};
    double timeAtMaximumAltitudeS{0.0};

    double minimumAngleOfAttackRad{0.0};
    double maximumAngleOfAttackRad{0.0};

    double minimumEffectiveWingAngleRad{0.0};
    double maximumEffectiveWingAngleRad{0.0};

    double maximumAbsolutePitchRateRadps{0.0};

    double minimumMach{0.0};
    double maximumMach{0.0};

    double maximumDynamicPressurePa{0.0};

    double minimumCx{0.0};
    double maximumCx{0.0};

    double minimumCy{0.0};
    double maximumCy{0.0};

    double minimumMz{0.0};
    double maximumMz{0.0};

    double minimumLiftToDragRatio{0.0};
    double maximumLiftToDragRatio{0.0};
    double finalLiftToDragRatio{0.0};

    /*
     * Первый момент времени, после которого угол атаки
     * больше не выходит из заданной полосы относительно
     * конечного значения.
     */
    double angleOfAttackSettlingTimeS{0.0};
};

/**
 * Результат расчёта статической балансировки
 * при заданном числе Маха.
 */
struct AerodynamicBalanceAnalysis {
    bool available{false};
    bool staticallyStable{false};

    double mach{0.0};

    // Производная статического момента по alpha, 1/рад.
    double mzAlphaPerRad{0.0};

    // Угол атаки, при котором mzStatic = 0.
    double trimAngleOfAttackRad{0.0};

    // Эффективный угол атаки крыла в балансировочном режиме.
    double trimEffectiveWingAngleRad{0.0};

    double trimCx{0.0};
    double trimCy{0.0};
    double trimLiftToDragRatio{0.0};
};

/**
 * Анализирует историю траектории.
 *
 * По умолчанию угол атаки считается установившимся,
 * если он остаётся в пределах ±0,1 градуса
 * относительно конечного значения.
 */
[[nodiscard]]
TrajectoryAnalysis analyzeTrajectory(
    const SimulationResult& result,
    const ObjectModel& object,
    double angleOfAttackSettlingBandRad
);

/**
 * Перегрузка с полосой установления ±0,1 градуса.
 */
[[nodiscard]]
TrajectoryAnalysis analyzeTrajectory(
    const SimulationResult& result,
    const ObjectModel& object
);

/**
 * Вычисляет балансировочный угол и производную
 * статического момента при заданном числе Маха.
 */
[[nodiscard]]
AerodynamicBalanceAnalysis analyzeAerodynamicBalance(
    const ObjectModel& object,
    double mach
);

} // namespace passive_flight