#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace passive_flight {

/*
 * СИСТЕМА КООРДИНАТ И ЗНАКОВ
 *
 * x:
 *   Горизонтальная координата.
 *   Положительное направление совпадает с направлением движения
 *   носителя в момент сброса.
 *
 * H:
 *   Геометрическая высота.
 *   Положительное направление — вверх.
 *
 * Theta:
 *   Угол наклона траектории к местному горизонту.
 *   Положительный угол соответствует набору высоты.
 *   Отрицательный угол соответствует снижению.
 *
 * theta:
 *   Угол тангажа продольной оси объекта.
 *   Положительный угол соответствует поднятию носа.
 *
 * omegaZ:
 *   Угловая скорость тангажа.
 *   Положительна при увеличении theta.
 *
 * alpha:
 *   Угол атаки:
 *
 *       alpha = theta - Theta.
 *
 * Все размерные значения передаются в единицах СИ.
 * Все углы внутри ядра задаются в радианах.
 */

/*
 * Параметры горизонтального сброса.
 *
 * На внешнем интерфейсе пользователь задаёт только высоту
 * и скорость сброса.
 */
struct ReleaseConditions {
    double altitudeM{};
    double speedMps{};
};

/*
 * Запрос на моделирование.
 *
 * objectId позволяет в будущем выбирать один объект
 * из реестра нескольких пассивных летательных аппаратов.
 */
struct SimulationRequest {
    std::string objectId;
    ReleaseConditions release;
};

/*
 * Вектор состояния невозмущённого продольного движения:
 *
 *     [V, Theta, omegaZ, theta, x, H].
 *
 * Время хранится в той же структуре для удобства,
 * но не является отдельной неизвестной дифференциальной системы.
 */
struct State {
    double timeS{};

    double speedMps{};
    double flightPathAngleRad{};
    double pitchRateRadps{};
    double pitchAngleRad{};

    double downrangeM{};
    double altitudeM{};

    /*
     * Угол атаки вычисляется из угла тангажа
     * и угла наклона траектории.
     */
    [[nodiscard]] double angleOfAttackRad() const noexcept {
        return pitchAngleRad - flightPathAngleRad;
    }
};

/*
 * Производные параметров состояния.
 *
 * Позднее эта структура будет заполняться правой частью
 * системы дифференциальных уравнений.
 */
struct StateDerivative {
    double speedMps2{};
    double flightPathAngleRadps{};
    double pitchRateRadps2{};
    double pitchAngleRadps{};

    double downrangeMps{};
    double altitudeMps{};
};

/*
 * Состояние атмосферы на заданной высоте.
 */
struct Environment {
    double densityKgM3{};
    double speedOfSoundMps{};
    double dynamicViscosityPaS{};
};

/*
 * Безразмерные аэродинамические коэффициенты сил.
 */
struct ForceCoefficients {
    double drag{};
    double lift{};
};

/*
 * Диагностические данные одного шага.
 *
 * Они не являются независимыми параметрами состояния,
 * а вычисляются по текущему состоянию объекта.
 */
struct StepDiagnostics {
    double angleOfAttackRad{};
    double angleOfAttackRateRadps{};

    double mach{};
    double reynolds{};
    double dynamicPressurePa{};

    double dragCoefficient{};
    double liftCoefficient{};
    double pitchingMomentCoefficient{};

    double dragN{};
    double liftN{};
    double pitchingMomentNm{};
};

/*
 * Одна точка истории движения.
 *
 * Используется для построения графиков в SimInTech
 * и будущем исследовательском приложении.
 */
struct TrajectorySample {
    State state;
    StepDiagnostics diagnostics;
};

/*
 * Причина завершения моделирования.
 */
enum class TerminationReason {
    GroundReached,
    MaximumTimeReached,
    MaximumStepsReached,
    InvalidInput,
    InvalidState
};

/*
 * Настройки численного решения.
 *
 * Пользователь SimInTech не обязан передавать эти значения.
 * Они являются внутренними настройками ядра.
 */
struct SimulationOptions {
    double timeStepS{0.001};
    double maximumTimeS{300.0};

    std::size_t maximumSteps{2'000'000};

    double groundAltitudeM{0.0};

    bool saveHistory{true};
    std::size_t historyStride{1};
};

/*
 * Полный результат моделирования.
 */
struct SimulationResult {
    TerminationReason terminationReason{
        TerminationReason::InvalidState
    };

    State finalState{};

    std::vector<TrajectorySample> history;
};

/*
 * Краткий результат моделирования.
 *
 * Именно эти значения впоследствии будут основными
 * выходами блока SimInTech.
 */
struct SimulationSummary {
    double downrangeM{};
    double fallTimeS{};

    double impactSpeedMps{};
    double impactFlightPathAngleRad{};
    double impactPitchAngleRad{};
    double impactAngleOfAttackRad{};

    TerminationReason terminationReason{
        TerminationReason::InvalidState
    };
};

} // namespace passive_flight