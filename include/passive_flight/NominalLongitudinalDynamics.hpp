#pragma once

#include "passive_flight/Aerodynamics.hpp"
#include "passive_flight/Atmosphere.hpp"
#include "passive_flight/ObjectAerodynamicsAdapter.hpp"
#include "passive_flight/ObjectModel.hpp"
#include "passive_flight/Types.hpp"

namespace passive_flight {

/**
 * Размерные аэродинамические силы и момент.
 *
 * Знаки определяются принятой системой уравнений:
 *
 * dragN — положительный модуль силы сопротивления,
 * направленной против вектора скорости;
 *
 * normalForceN — нормальная сила Y, положительная
 * в сторону увеличения угла наклона траектории;
 *
 * pitchingMomentNm — продольный момент Mz.
 */
struct LongitudinalAerodynamicLoads {
    double dragN{0.0};
    double normalForceN{0.0};
    double pitchingMomentNm{0.0};
};

/**
 * Полный результат вычисления правых частей системы.
 *
 * Помимо производных состояния здесь сохраняются
 * промежуточные значения. Они потребуются:
 *
 * - для тестирования;
 * - для построения графиков;
 * - для подробного вывода в приложении;
 * - для передачи диагностических данных в SimInTech.
 */
struct LongitudinalDynamicsEvaluation {
    StateDerivative derivative;

    AtmosphereState atmosphere;
    AerodynamicCoefficients aerodynamics;
    LongitudinalAerodynamicLoads loads;

    double mach{0.0};
    double dynamicPressurePa{0.0};

    double angleOfAttackRad{0.0};
    double angleOfAttackRateRadS{0.0};
};

/**
 * Правая часть нелинейной системы уравнений
 * невозмущённого продольного движения.
 *
 * Двигательная установка отсутствует.
 * Тяга принимается равной нулю.
 */
class NominalLongitudinalDynamics {
public:
    explicit NominalLongitudinalDynamics(
        ObjectModel object
    );

    NominalLongitudinalDynamics(
        ObjectModel object,
        StandardAtmosphere atmosphere
    );

    /**
     * Вычисляет производные состояния в заданной точке.
     *
     * Порядок компонент State:
     *
     * [V, Theta, omega_z, vartheta, x, H].
     */
    [[nodiscard]]
    LongitudinalDynamicsEvaluation evaluate(
        const State& state
    ) const;

    [[nodiscard]]
    const ObjectModel& object() const noexcept;

    [[nodiscard]]
    const StandardAtmosphere& atmosphere() const noexcept;

    [[nodiscard]]
    const PreliminaryAerodynamicModel&
    aerodynamics() const noexcept;

private:
    ObjectModel object_;
    StandardAtmosphere atmosphere_;
    PreliminaryAerodynamicModel aerodynamics_;
};

} // namespace passive_flight