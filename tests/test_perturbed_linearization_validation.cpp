#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/PerturbedTrajectorySimulator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <string>

namespace {

int failureCount = 0;

constexpr double kDegreesToRadians =
    std::numbers::pi_v<double> / 180.0;

void check(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        std::cerr
            << "FAILED: "
            << message
            << '\n';

        ++failureCount;
    }
}

/*
 * Один шаг той же нелинейной системы явным методом Эйлера.
 *
 * Этот локальный интегратор нужен только для верификации:
 * мы намеренно не добавляем в production API отдельный режим
 * запуска ForwardEulerSimulator из произвольного состояния.
 */
passive_flight::State makeNonlinearEulerStep(
    const passive_flight::State& current,
    const passive_flight::StateDerivative& derivative,
    double timeStepS
) {
    passive_flight::State next = current;

    next.timeS +=
        timeStepS;

    next.speedMps +=
        timeStepS *
        derivative.speedMps2;

    next.flightPathAngleRad +=
        timeStepS *
        derivative.flightPathAngleRadps;

    next.pitchRateRadps +=
        timeStepS *
        derivative.pitchRateRadps2;

    next.pitchAngleRad +=
        timeStepS *
        derivative.pitchAngleRadps;

    next.downrangeM +=
        timeStepS *
        derivative.downrangeMps;

    next.altitudeM +=
        timeStepS *
        derivative.altitudeMps;

    return next;
}

/*
 * Разность двух нелинейных состояний:
 *
 *     Delta u_NL =
 *         u_perturbed_NL - u_nominal.
 */
passive_flight::LongitudinalPerturbationState
makeStateDifference(
    const passive_flight::State& perturbed,
    const passive_flight::State& nominal
) {
    passive_flight::LongitudinalPerturbationState difference;

    difference.speedMps =
        perturbed.speedMps -
        nominal.speedMps;

    difference.flightPathAngleRad =
        perturbed.flightPathAngleRad -
        nominal.flightPathAngleRad;

    difference.pitchRateRadps =
        perturbed.pitchRateRadps -
        nominal.pitchRateRadps;

    difference.pitchAngleRad =
        perturbed.pitchAngleRad -
        nominal.pitchAngleRad;

    difference.downrangeM =
        perturbed.downrangeM -
        nominal.downrangeM;

    difference.altitudeM =
        perturbed.altitudeM -
        nominal.altitudeM;

    return difference;
}

/*
 * Ошибка линейного приближения:
 *
 *     e =
 *         Delta u_NL - Delta u_linear.
 */
passive_flight::LongitudinalPerturbationState
makeLinearizationError(
    const passive_flight::LongitudinalPerturbationState&
        nonlinearDifference,
    const passive_flight::LongitudinalPerturbationState&
        linearPerturbation
) {
    passive_flight::LongitudinalPerturbationState error;

    error.speedMps =
        nonlinearDifference.speedMps -
        linearPerturbation.speedMps;

    error.flightPathAngleRad =
        nonlinearDifference.flightPathAngleRad -
        linearPerturbation.flightPathAngleRad;

    error.pitchRateRadps =
        nonlinearDifference.pitchRateRadps -
        linearPerturbation.pitchRateRadps;

    error.pitchAngleRad =
        nonlinearDifference.pitchAngleRad -
        linearPerturbation.pitchAngleRad;

    error.downrangeM =
        nonlinearDifference.downrangeM -
        linearPerturbation.downrangeM;

    error.altitudeM =
        nonlinearDifference.altitudeM -
        linearPerturbation.altitudeM;

    return error;
}

/*
 * Нормированная безразмерная норма.
 *
 * Масштабы нужны только для того, чтобы координаты x и H
 * в метрах не подавляли в норме угловые величины.
 *
 * Это не физический критерий качества объекта.
 * Это техническая мера ошибки для теста сходимости.
 */
double normalizedErrorNorm(
    const passive_flight::LongitudinalPerturbationState& value
) {
    const double normalizedSpeed =
        value.speedMps / 200.0;

    const double normalizedFlightPathAngle =
        value.flightPathAngleRad;

    const double normalizedPitchRate =
        value.pitchRateRadps;

    const double normalizedPitchAngle =
        value.pitchAngleRad;

    const double normalizedDownrange =
        value.downrangeM / 1000.0;

    const double normalizedAltitude =
        value.altitudeM / 100.0;

    return std::sqrt(
        normalizedSpeed *
            normalizedSpeed +

        normalizedFlightPathAngle *
            normalizedFlightPathAngle +

        normalizedPitchRate *
            normalizedPitchRate +

        normalizedPitchAngle *
            normalizedPitchAngle +

        normalizedDownrange *
            normalizedDownrange +

        normalizedAltitude *
            normalizedAltitude
    );
}

struct ValidationResult {
    double finalErrorNorm{};
    double maximumErrorNorm{};
};

/*
 * Выполняет один эксперимент.
 *
 * 1. Строится номинальная нелинейная траектория u*(t).
 *
 * 2. Одновременно PerturbedTrajectorySimulator интегрирует:
 *
 *        Delta u dot = A(t) Delta u.
 *
 * 3. Из начального состояния
 *
 *        u_p(0) = u*(0) + Delta u(0)
 *
 *    отдельно интегрируется ПОЛНАЯ нелинейная система.
 *
 * 4. В одинаковые моменты времени сравниваются:
 *
 *        u_p(t) - u*(t)
 *
 *    и
 *
 *        Delta u_linear(t).
 */
ValidationResult runValidation(
    double perturbationScale
) {
    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const passive_flight::PerturbedTrajectorySimulator simulator(
        passport.object
    );

    passive_flight::SimulationRequest request;

    request.objectId =
        passport.object.id;

    request.release.altitudeM =
        100.0;

    request.release.speedMps =
        200.0;

    /*
     * Базовое возмущение совпадает с используемым
     * сейчас в perturbed demo:
     *
     * Delta V0 = 1 m/s;
     * Delta theta0 = 0.1 deg.
     *
     * Далее всё возмущение умножается на scale.
     */
    passive_flight::LongitudinalPerturbationState
        initialPerturbation;

    initialPerturbation.speedMps =
        1.0 *
        perturbationScale;

    initialPerturbation.pitchAngleRad =
        0.1 *
        kDegreesToRadians *
        perturbationScale;

    passive_flight::SimulationOptions options;

    options.timeStepS =
        0.001;

    /*
     * Сравниваем траектории до падения.
     *
     * Это принципиально:
     * здесь проверяется система в вариациях
     * на одном и том же интервале времени,
     * а не чувствительность события H = 0.
     */
    options.maximumTimeS =
        4.0;

    options.maximumSteps =
        10'000;

    options.groundAltitudeM =
        0.0;

    options.saveHistory =
        true;

    /*
     * Нужен каждый временной узел, чтобы вторая
     * нелинейная траектория шла точно с теми же dt.
     */
    options.historyStride =
        1;

    const auto linearResult =
        simulator.simulate(
            request,
            initialPerturbation,
            options
        );

    check(
        linearResult.terminationReason ==
            passive_flight::
                TerminationReason::MaximumTimeReached,
        "Validation trajectory reaches common "
        "maximum time before ground"
    );

    check(
        linearResult.history.size() > 1,
        "Validation trajectory contains history"
    );

    if (linearResult.history.size() <= 1) {
        return {};
    }

    /*
     * Начальное состояние второй нелинейной
     * траектории:
     *
     *     u_p(0) = u*(0) + Delta u0.
     */
    passive_flight::State nonlinearPerturbedState =
        linearResult.history.front().nominalState;

    nonlinearPerturbedState.speedMps +=
        initialPerturbation.speedMps;

    nonlinearPerturbedState.flightPathAngleRad +=
        initialPerturbation.flightPathAngleRad;

    nonlinearPerturbedState.pitchRateRadps +=
        initialPerturbation.pitchRateRadps;

    nonlinearPerturbedState.pitchAngleRad +=
        initialPerturbation.pitchAngleRad;

    nonlinearPerturbedState.downrangeM +=
        initialPerturbation.downrangeM;

    nonlinearPerturbedState.altitudeM +=
        initialPerturbation.altitudeM;

    double maximumErrorNorm = 0.0;
    double finalErrorNorm = 0.0;

    for (
        std::size_t index = 1;
        index < linearResult.history.size();
        ++index
    ) {
        const auto& previousSample =
            linearResult.history[index - 1];

        const auto& currentSample =
            linearResult.history[index];

        const double timeStepS =
            currentSample.nominalState.timeS -
            previousSample.nominalState.timeS;

        check(
            std::isfinite(timeStepS) &&
                timeStepS > 0.0,
            "Validation time step is positive"
        );

        if (!std::isfinite(timeStepS) ||
            timeStepS <= 0.0) {
            return {};
        }

        /*
         * Правая часть полной НЕЛИНЕЙНОЙ системы
         * вычисляется не в номинальном состоянии,
         * а в independently perturbed state.
         */
        const auto nonlinearEvaluation =
            simulator
                .perturbationDynamics()
                .nominalDynamics()
                .evaluate(
                    nonlinearPerturbedState
                );

        nonlinearPerturbedState =
            makeNonlinearEulerStep(
                nonlinearPerturbedState,
                nonlinearEvaluation.derivative,
                timeStepS
            );

        /*
         * Фактическая разность двух решений
         * полной нелинейной системы.
         */
        const auto nonlinearDifference =
            makeStateDifference(
                nonlinearPerturbedState,
                currentSample.nominalState
            );

        /*
         * Отклонение фактической нелинейной разности
         * от решения системы в вариациях.
         */
        const auto error =
            makeLinearizationError(
                nonlinearDifference,
                currentSample.perturbation
            );

        const double errorNorm =
            normalizedErrorNorm(error);

        maximumErrorNorm =
            std::max(
                maximumErrorNorm,
                errorNorm
            );

        finalErrorNorm =
            errorNorm;
    }

    check(
        std::isfinite(finalErrorNorm),
        "Final linearization error is finite"
    );

    check(
        std::isfinite(maximumErrorNorm),
        "Maximum linearization error is finite"
    );

    ValidationResult result;

    result.finalErrorNorm =
        finalErrorNorm;

    result.maximumErrorNorm =
        maximumErrorNorm;

    return result;
}

/*
 * Основная физико-математическая проверка.
 *
 * Если:
 *
 *     Delta u dot = A(t) Delta u
 *
 * действительно является первым приближением
 * исходной нелинейной системы, то ошибка должна
 * иметь второй порядок:
 *
 *     E(epsilon) = O(epsilon^2).
 *
 * Поэтому при:
 *
 *     epsilon -> epsilon / 2
 *
 * ожидаем приблизительно:
 *
 *     E -> E / 4.
 */
void testLinearizationHasSecondOrderError() {
    const auto scale1 =
        runValidation(1.0);

    const auto scaleHalf =
        runValidation(0.5);

    const auto scaleQuarter =
        runValidation(0.25);

    const auto scaleEighth =
        runValidation(0.125);

    check(
        scale1.finalErrorNorm > 0.0,
        "Full-scale linearization error is non-zero"
    );

    check(
        scaleHalf.finalErrorNorm > 0.0,
        "Half-scale linearization error is non-zero"
    );

    check(
        scaleQuarter.finalErrorNorm > 0.0,
        "Quarter-scale linearization error is non-zero"
    );

    check(
        scaleEighth.finalErrorNorm > 0.0,
        "Eighth-scale linearization error is non-zero"
    );

    /*
     * Идеальное отношение для квадратичной ошибки:
     *
     *     E(eps / 2) / E(eps) = 0.25.
     *
     * Допускаем значение до 0.35, чтобы тест не
     * был привязан к последним цифрам конкретной
     * аэродинамической параметризации.
     */
    check(
        scaleHalf.finalErrorNorm <
            0.35 *
            scale1.finalErrorNorm,
        "Halving perturbation reduces final "
        "error approximately quadratically"
    );

    check(
        scaleQuarter.finalErrorNorm <
            0.35 *
            scaleHalf.finalErrorNorm,
        "Second halving preserves "
        "quadratic error reduction"
    );

    check(
        scaleEighth.finalErrorNorm <
            0.35 *
            scaleQuarter.finalErrorNorm,
        "Third halving preserves "
        "quadratic error reduction"
    );

    /*
     * Аналогично проверяем не только конечную точку,
     * но и максимальную ошибку на всём интервале.
     */
    check(
        scaleHalf.maximumErrorNorm <
            0.35 *
            scale1.maximumErrorNorm,
        "Maximum trajectory error decreases "
        "quadratically after first halving"
    );

    check(
        scaleQuarter.maximumErrorNorm <
            0.35 *
            scaleHalf.maximumErrorNorm,
        "Maximum trajectory error decreases "
        "quadratically after second halving"
    );

    check(
        scaleEighth.maximumErrorNorm <
            0.35 *
            scaleQuarter.maximumErrorNorm,
        "Maximum trajectory error decreases "
        "quadratically after third halving"
    );

    std::cout
        << "Linearization validation errors:\n"

        << "scale 1.000: final = "
        << scale1.finalErrorNorm
        << ", max = "
        << scale1.maximumErrorNorm
        << '\n'

        << "scale 0.500: final = "
        << scaleHalf.finalErrorNorm
        << ", max = "
        << scaleHalf.maximumErrorNorm
        << '\n'

        << "scale 0.250: final = "
        << scaleQuarter.finalErrorNorm
        << ", max = "
        << scaleQuarter.maximumErrorNorm
        << '\n'

        << "scale 0.125: final = "
        << scaleEighth.finalErrorNorm
        << ", max = "
        << scaleEighth.maximumErrorNorm
        << '\n';
}

} // namespace

int main() {
    testLinearizationHasSecondOrderError();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " perturbed-linearization-validation "
               "test(s) failed\n";

        return 1;
    }

    std::cout
        << "All perturbed-linearization-validation "
           "tests passed\n";

    return 0;
}