#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/PerturbedImpactAnalysis.hpp"
#include "passive_flight/PerturbedTrajectorySimulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
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

bool isFinite(
    const passive_flight::State& state
) {
    return
        std::isfinite(state.timeS) &&
        std::isfinite(state.speedMps) &&
        std::isfinite(state.flightPathAngleRad) &&
        std::isfinite(state.pitchRateRadps) &&
        std::isfinite(state.pitchAngleRad) &&
        std::isfinite(state.downrangeM) &&
        std::isfinite(state.altitudeM);
}

/*
 * Один шаг полной нелинейной системы
 * тем же явным методом Эйлера, который используется
 * в основном ForwardEulerSimulator.
 *
 * Этот код находится только в тесте.
 * Production API ради верификации не расширяем.
 */
passive_flight::State makeEulerStep(
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

double interpolateValue(
    double first,
    double second,
    double interpolationParameter
) {
    return
        first +
        interpolationParameter *
        (second - first);
}

/*
 * Линейная интерполяция события H = H_ground.
 *
 * Она повторяет принцип, используемый
 * ForwardEulerSimulator:
 *
 * если один узел находится выше поверхности,
 * а следующий ниже, определяем точку пересечения
 * поверхности между ними.
 */
passive_flight::State interpolateGroundImpact(
    const passive_flight::State& aboveGround,
    const passive_flight::State& belowGround,
    double groundAltitudeM
) {
    const double altitudeDifference =
        aboveGround.altitudeM -
        belowGround.altitudeM;

    double interpolationParameter = 1.0;

    if (std::abs(altitudeDifference) >
        1.0e-12) {
        interpolationParameter =
            (
                aboveGround.altitudeM -
                groundAltitudeM
            ) /
            altitudeDifference;
    }

    interpolationParameter =
        std::clamp(
            interpolationParameter,
            0.0,
            1.0
        );

    passive_flight::State impact;

    impact.timeS =
        interpolateValue(
            aboveGround.timeS,
            belowGround.timeS,
            interpolationParameter
        );

    impact.speedMps =
        interpolateValue(
            aboveGround.speedMps,
            belowGround.speedMps,
            interpolationParameter
        );

    impact.flightPathAngleRad =
        interpolateValue(
            aboveGround.flightPathAngleRad,
            belowGround.flightPathAngleRad,
            interpolationParameter
        );

    impact.pitchRateRadps =
        interpolateValue(
            aboveGround.pitchRateRadps,
            belowGround.pitchRateRadps,
            interpolationParameter
        );

    impact.pitchAngleRad =
        interpolateValue(
            aboveGround.pitchAngleRad,
            belowGround.pitchAngleRad,
            interpolationParameter
        );

    impact.downrangeM =
        interpolateValue(
            aboveGround.downrangeM,
            belowGround.downrangeM,
            interpolationParameter
        );

    impact.altitudeM =
        groundAltitudeM;

    return impact;
}

struct NonlinearImpactResult {
    bool available{false};
    passive_flight::State impactState;
};

/*
 * Интегрирует ПОЛНУЮ нелинейную модель
 * из произвольного начального состояния
 * непосредственно до достижения поверхности.
 *
 * Это эталон для текущего теста.
 *
 * Важно:
 * здесь не используется система малых возмущений.
 */
NonlinearImpactResult simulateNonlinearImpact(
    const passive_flight::NominalLongitudinalDynamics& dynamics,
    const passive_flight::State& initialState,
    const passive_flight::SimulationOptions& options
) {
    NonlinearImpactResult result;

    if (!isFinite(initialState) ||
        !std::isfinite(options.timeStepS) ||
        options.timeStepS <= 0.0 ||
        !std::isfinite(options.maximumTimeS) ||
        options.maximumTimeS <= 0.0 ||
        options.maximumSteps == 0 ||
        !std::isfinite(options.groundAltitudeM)) {
        return result;
    }

    passive_flight::State current =
        initialState;

    if (current.altitudeM <=
        options.groundAltitudeM) {
        return result;
    }

    std::size_t completedSteps = 0;

    while (true) {
        if (current.timeS >=
            options.maximumTimeS) {
            return result;
        }

        if (completedSteps >=
            options.maximumSteps) {
            return result;
        }

        const double remainingTimeS =
            options.maximumTimeS -
            current.timeS;

        const double actualTimeStepS =
            std::min(
                options.timeStepS,
                remainingTimeS
            );

        if (actualTimeStepS <= 0.0) {
            return result;
        }

        passive_flight::LongitudinalDynamicsEvaluation
            evaluation;

        try {
            evaluation =
                dynamics.evaluate(current);
        } catch (...) {
            return result;
        }

        const passive_flight::State next =
            makeEulerStep(
                current,
                evaluation.derivative,
                actualTimeStepS
            );

        ++completedSteps;

        if (!isFinite(next) ||
            next.speedMps <= 0.0) {
            return result;
        }

        if (next.altitudeM <=
            options.groundAltitudeM) {
            const passive_flight::State impact =
                interpolateGroundImpact(
                    current,
                    next,
                    options.groundAltitudeM
                );

            if (!isFinite(impact) ||
                impact.speedMps <= 0.0) {
                return result;
            }

            result.available = true;
            result.impactState = impact;

            return result;
        }

        current = next;
    }
}

struct ActualImpactChanges {
    double fallTimeS{};

    double speedMps{};
    double flightPathAngleRad{};
    double pitchRateRadps{};
    double pitchAngleRad{};

    double downrangeM{};

    [[nodiscard]]
    double angleOfAttackRad() const noexcept {
        return
            pitchAngleRad -
            flightPathAngleRad;
    }
};

ActualImpactChanges makeActualImpactChanges(
    const passive_flight::State& nominalImpact,
    const passive_flight::State& perturbedImpact
) {
    ActualImpactChanges changes;

    changes.fallTimeS =
        perturbedImpact.timeS -
        nominalImpact.timeS;

    changes.speedMps =
        perturbedImpact.speedMps -
        nominalImpact.speedMps;

    changes.flightPathAngleRad =
        perturbedImpact.flightPathAngleRad -
        nominalImpact.flightPathAngleRad;

    changes.pitchRateRadps =
        perturbedImpact.pitchRateRadps -
        nominalImpact.pitchRateRadps;

    changes.pitchAngleRad =
        perturbedImpact.pitchAngleRad -
        nominalImpact.pitchAngleRad;

    changes.downrangeM =
        perturbedImpact.downrangeM -
        nominalImpact.downrangeM;

    return changes;
}

/*
 * Безразмерная норма ошибки параметров падения.
 *
 * Нормировка нужна только для объединения величин
 * разных размерностей в один показатель.
 *
 * Она не является частью физической модели.
 */
double normalizedImpactError(
    const ActualImpactChanges& actual,
    const passive_flight::
        PerturbedImpactParameterChanges& predicted
) {
    const double timeError =
        (
            predicted.fallTimeS -
            actual.fallTimeS
        ) /
        10.0;

    const double speedError =
        (
            predicted.speedMps -
            actual.speedMps
        ) /
        200.0;

    const double flightPathAngleError =
        predicted.flightPathAngleRad -
        actual.flightPathAngleRad;

    const double pitchRateError =
        predicted.pitchRateRadps -
        actual.pitchRateRadps;

    const double pitchAngleError =
        predicted.pitchAngleRad -
        actual.pitchAngleRad;

    const double downrangeError =
        (
            predicted.downrangeM -
            actual.downrangeM
        ) /
        1000.0;

    return std::sqrt(
        timeError *
            timeError +

        speedError *
            speedError +

        flightPathAngleError *
            flightPathAngleError +

        pitchRateError *
            pitchRateError +

        pitchAngleError *
            pitchAngleError +

        downrangeError *
            downrangeError
    );
}

struct ImpactValidationResult {
    bool available{false};

    double perturbationScale{};

    ActualImpactChanges actualChanges;

    passive_flight::
        PerturbedImpactParameterChanges
            predictedChanges;

    passive_flight::State
        actualPerturbedImpactState;

    passive_flight::State
        predictedPerturbedImpactState;

    double normalizedError{};
};

ImpactValidationResult runImpactValidation(
    double perturbationScale
) {
    ImpactValidationResult validation;

    validation.perturbationScale =
        perturbationScale;

    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const passive_flight::
        PerturbedTrajectorySimulator simulator(
            passport.object
        );

    passive_flight::SimulationRequest request;

    request.objectId =
        passport.object.id;

    request.release.altitudeM =
        100.0;

    request.release.speedMps =
        200.0;

    passive_flight::
        LongitudinalPerturbationState
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

    options.maximumTimeS =
        30.0;

    options.maximumSteps =
        100'000;

    options.groundAltitudeM =
        0.0;

    options.saveHistory =
        false;

    options.historyStride =
        1;

    /*
     * Сначала получаем номинальную траекторию
     * и решение системы малых возмущений.
     */
    const auto linearResult =
        simulator.simulate(
            request,
            initialPerturbation,
            options
        );

    check(
        linearResult.terminationReason ==
            passive_flight::
                TerminationReason::GroundReached,
        "Linearized calculation reaches "
        "nominal ground event"
    );

    if (linearResult.terminationReason !=
        passive_flight::
            TerminationReason::GroundReached) {
        return validation;
    }

    /*
     * Производная номинального состояния
     * непосредственно в точке падения.
     *
     * Она используется формулой:
     *
     *     Delta t_f =
     *       -Delta H_f / H_dot_f*.
     */
    passive_flight::
        LongitudinalDynamicsEvaluation
            finalNominalEvaluation;

    try {
        finalNominalEvaluation =
            simulator
                .perturbationDynamics()
                .nominalDynamics()
                .evaluate(
                    linearResult.finalNominalState
                );
    } catch (...) {
        check(
            false,
            "Nominal impact derivative can be evaluated"
        );

        return validation;
    }

    const auto impactAnalysis =
        passive_flight::analyzePerturbedImpact(
            linearResult,
            finalNominalEvaluation.derivative
        );

    check(
        impactAnalysis.available,
        "First-order impact analysis is available"
    );

    if (!impactAnalysis.available) {
        return validation;
    }

    /*
     * Теперь формируем настоящее возмущённое
     * начальное состояние полной нелинейной модели:
     *
     *     u_p(0) = u*(0) + Delta u_0.
     */
    passive_flight::State
        perturbedInitialState;

    perturbedInitialState.timeS =
        0.0;

    perturbedInitialState.speedMps =
        request.release.speedMps +
        initialPerturbation.speedMps;

    perturbedInitialState.flightPathAngleRad =
        initialPerturbation.flightPathAngleRad;

    perturbedInitialState.pitchRateRadps =
        initialPerturbation.pitchRateRadps;

    perturbedInitialState.pitchAngleRad =
        initialPerturbation.pitchAngleRad;

    perturbedInitialState.downrangeM =
        initialPerturbation.downrangeM;

    perturbedInitialState.altitudeM =
        request.release.altitudeM +
        initialPerturbation.altitudeM;

    /*
     * Независимо интегрируем ПОЛНУЮ нелинейную
     * траекторию до её собственного H = 0.
     */
    const auto nonlinearImpact =
        simulateNonlinearImpact(
            simulator
                .perturbationDynamics()
                .nominalDynamics(),
            perturbedInitialState,
            options
        );

    check(
        nonlinearImpact.available,
        "Nonlinear perturbed trajectory "
        "reaches its own ground event"
    );

    if (!nonlinearImpact.available) {
        return validation;
    }

    const ActualImpactChanges actualChanges =
        makeActualImpactChanges(
            linearResult.finalNominalState,
            nonlinearImpact.impactState
        );

    validation.available = true;

    validation.actualChanges =
        actualChanges;

    validation.predictedChanges =
        impactAnalysis.changes;

    validation.actualPerturbedImpactState =
        nonlinearImpact.impactState;

    validation.predictedPerturbedImpactState =
        impactAnalysis.estimatedImpactState;

    validation.normalizedError =
        normalizedImpactError(
            actualChanges,
            impactAnalysis.changes
        );

    return validation;
}

void printValidation(
    const ImpactValidationResult& result
) {
    std::cout
        << std::setprecision(12)

        << "scale "
        << result.perturbationScale
        << ":\n"

        << "  normalized error = "
        << result.normalizedError
        << '\n'

        << "  Delta t predicted / actual = "
        << result.predictedChanges.fallTimeS
        << " / "
        << result.actualChanges.fallTimeS
        << " s\n"

        << "  Delta L predicted / actual = "
        << result.predictedChanges.downrangeM
        << " / "
        << result.actualChanges.downrangeM
        << " m\n"

        << "  Delta V predicted / actual = "
        << result.predictedChanges.speedMps
        << " / "
        << result.actualChanges.speedMps
        << " m/s\n"

        << "  Delta Theta predicted / actual = "
        << result.predictedChanges.flightPathAngleRad
        << " / "
        << result.actualChanges.flightPathAngleRad
        << " rad\n"

        << "  Delta omega_z predicted / actual = "
        << result.predictedChanges.pitchRateRadps
        << " / "
        << result.actualChanges.pitchRateRadps
        << " rad/s\n"

        << "  Delta theta predicted / actual = "
        << result.predictedChanges.pitchAngleRad
        << " / "
        << result.actualChanges.pitchAngleRad
        << " rad\n"

        << "  Delta alpha predicted / actual = "
        << result.predictedChanges.angleOfAttackRad()
        << " / "
        << result.actualChanges.angleOfAttackRad()
        << " rad\n";
}

/*
 * Главная проверка:
 *
 * PerturbedImpactAnalysis является первым
 * приближением отображения:
 *
 *     начальное состояние
 *          ->
 *     собственное событие H = 0.
 *
 * Поэтому его ошибка также должна стремиться
 * ко второму порядку по величине начального
 * возмущения:
 *
 *     E(epsilon) = O(epsilon^2).
 */
void testImpactPredictionConvergesToNonlinearImpact() {
    const auto scale1 =
        runImpactValidation(1.0);

    const auto scaleHalf =
        runImpactValidation(0.5);

    const auto scaleQuarter =
        runImpactValidation(0.25);

    const auto scaleEighth =
        runImpactValidation(0.125);

    check(
        scale1.available,
        "Full-scale impact validation is available"
    );

    check(
        scaleHalf.available,
        "Half-scale impact validation is available"
    );

    check(
        scaleQuarter.available,
        "Quarter-scale impact validation is available"
    );

    check(
        scaleEighth.available,
        "Eighth-scale impact validation is available"
    );

    if (!scale1.available ||
        !scaleHalf.available ||
        !scaleQuarter.available ||
        !scaleEighth.available) {
        return;
    }

    check(
        scale1.normalizedError > 0.0 &&
        std::isfinite(
            scale1.normalizedError
        ),
        "Full-scale impact error is finite "
        "and non-zero"
    );

    check(
        scaleHalf.normalizedError > 0.0 &&
        std::isfinite(
            scaleHalf.normalizedError
        ),
        "Half-scale impact error is finite "
        "and non-zero"
    );

    check(
        scaleQuarter.normalizedError > 0.0 &&
        std::isfinite(
            scaleQuarter.normalizedError
        ),
        "Quarter-scale impact error is finite "
        "and non-zero"
    );

    check(
        scaleEighth.normalizedError > 0.0 &&
        std::isfinite(
            scaleEighth.normalizedError
        ),
        "Eighth-scale impact error is finite "
        "and non-zero"
    );

    /*
     * Для идеального второго порядка:
     *
     *     E(eps/2) / E(eps) = 0.25.
     *
     * Здесь допускаем 0.45.
     *
     * Допуск намеренно шире, чем у проверки
     * самой системы в вариациях, потому что
     * здесь дополнительно присутствуют:
     *
     * - численное определение события H = 0;
     * - интерполяция точки пересечения земли;
     * - first-order event-time correction.
     */
    check(
        scaleHalf.normalizedError <
            0.45 *
            scale1.normalizedError,
        "Halving perturbation strongly reduces "
        "impact prediction error"
    );

    check(
        scaleQuarter.normalizedError <
            0.45 *
            scaleHalf.normalizedError,
        "Second halving preserves "
        "impact error convergence"
    );

    check(
        scaleEighth.normalizedError <
            0.45 *
            scaleQuarter.normalizedError,
        "Third halving preserves "
        "impact error convergence"
    );

    /*
     * Дополнительная физическая проверка
     * конкретного базового возмущения.
     *
     * Для Delta H_f > 0 при снижении номинального
     * объекта момент падения должен смещаться вперёд.
     */
    check(
        scale1.predictedChanges.fallTimeS > 0.0,
        "Base perturbation predicts later impact"
    );

    check(
        scale1.actualChanges.fallTimeS > 0.0,
        "Nonlinear trajectory actually impacts later"
    );

    /*
     * Аналогично дальность должна увеличиваться
     * и в линейной оценке, и в прямом
     * нелинейном расчёте.
     */
    check(
        scale1.predictedChanges.downrangeM > 0.0,
        "Base perturbation predicts "
        "positive impact-range change"
    );

    check(
        scale1.actualChanges.downrangeM > 0.0,
        "Nonlinear trajectory has "
        "positive impact-range change"
    );

    std::cout
        << "Perturbed impact validation:\n";

    printValidation(scale1);
    printValidation(scaleHalf);
    printValidation(scaleQuarter);
    printValidation(scaleEighth);

    std::cout
        << "Error ratios:\n"

        << "  E(0.5) / E(1.0) = "
        << scaleHalf.normalizedError /
            scale1.normalizedError
        << '\n'

        << "  E(0.25) / E(0.5) = "
        << scaleQuarter.normalizedError /
            scaleHalf.normalizedError
        << '\n'

        << "  E(0.125) / E(0.25) = "
        << scaleEighth.normalizedError /
            scaleQuarter.normalizedError
        << '\n';
}

} // namespace

int main() {
    testImpactPredictionConvergesToNonlinearImpact();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " perturbed-impact-validation "
               "test(s) failed\n";

        return 1;
    }

    std::cout
        << "All perturbed-impact-validation "
           "tests passed\n";

    return 0;
}