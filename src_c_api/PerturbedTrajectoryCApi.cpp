#include "passive_flight_c_api/PerturbedTrajectoryCApi.h"

#include "passive_flight/ObjectRegistry.hpp"
#include "passive_flight/PerturbedImpactAnalysis.hpp"
#include "passive_flight/PerturbedTrajectorySimulator.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

constexpr double kTimeComparisonToleranceS = 1.0e-12;

const passive_flight::ObjectRegistry& registry() {
    static const passive_flight::ObjectRegistry instance =
        passive_flight::makeDefaultObjectRegistry();

    return instance;
}

bool isFinite(
    const PFPerturbedSimulationInput& input
) {
    return
        std::isfinite(input.releaseAltitudeM) &&
        std::isfinite(input.releaseSpeedMps) &&

        std::isfinite(input.deltaSpeedMps) &&
        std::isfinite(input.deltaFlightPathAngleRad) &&
        std::isfinite(input.deltaPitchRateRadps) &&
        std::isfinite(input.deltaPitchAngleRad) &&
        std::isfinite(input.deltaDownrangeM) &&
        std::isfinite(input.deltaAltitudeM);
}

bool isValidInput(
    const PFPerturbedSimulationInput& input
) {
    return
        input.objectId != nullptr &&
        input.objectId[0] != '\0' &&

        isFinite(input) &&

        input.releaseAltitudeM > 0.0 &&
        input.releaseSpeedMps > 0.0;
}

passive_flight::SimulationRequest makeRequest(
    const PFPerturbedSimulationInput& input
) {
    passive_flight::SimulationRequest request;

    request.objectId =
        input.objectId;

    request.release.altitudeM =
        input.releaseAltitudeM;

    request.release.speedMps =
        input.releaseSpeedMps;

    return request;
}

passive_flight::LongitudinalPerturbationState
makeInitialPerturbation(
    const PFPerturbedSimulationInput& input
) {
    passive_flight::LongitudinalPerturbationState
        perturbation;

    perturbation.speedMps =
        input.deltaSpeedMps;

    perturbation.flightPathAngleRad =
        input.deltaFlightPathAngleRad;

    perturbation.pitchRateRadps =
        input.deltaPitchRateRadps;

    perturbation.pitchAngleRad =
        input.deltaPitchAngleRad;

    perturbation.downrangeM =
        input.deltaDownrangeM;

    perturbation.altitudeM =
        input.deltaAltitudeM;

    return perturbation;
}

passive_flight::SimulationOptions
makeTrajectoryOptions() {
    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 300.0;
    options.maximumSteps = 2'000'000;

    options.groundAltitudeM = 0.0;

    options.saveHistory = true;

    /*
     * Решатель интегрирует с 0.001 с.
     *
     * Во внешнюю историю сохраняется каждая
     * десятая интеграционная точка:
     *
     *     10 * 0.001 = 0.01 с.
     *
     * Конечная точка собственного падения
     * возмущённого объекта добавляется отдельно
     * и поэтому не обязана лежать на сетке 0.01 с.
     */
    options.historyStride = 10;

    return options;
}

PFPerturbedTrajectoryPoint makePoint(
    const passive_flight::PerturbedTrajectorySample& sample
) {
    PFPerturbedTrajectoryPoint point{};

    const auto& nominal =
        sample.nominalState;

    const auto& delta =
        sample.perturbation;

    const auto& total =
        sample.totalState;

    point.timeS =
        nominal.timeS;

    point.nominalDownrangeM =
        nominal.downrangeM;

    point.nominalAltitudeM =
        nominal.altitudeM;

    point.nominalSpeedMps =
        nominal.speedMps;

    point.nominalFlightPathAngleRad =
        nominal.flightPathAngleRad;

    point.nominalPitchRateRadps =
        nominal.pitchRateRadps;

    point.nominalPitchAngleRad =
        nominal.pitchAngleRad;

    point.nominalAngleOfAttackRad =
        nominal.angleOfAttackRad();

    point.deltaDownrangeM =
        delta.downrangeM;

    point.deltaAltitudeM =
        delta.altitudeM;

    point.deltaSpeedMps =
        delta.speedMps;

    point.deltaFlightPathAngleRad =
        delta.flightPathAngleRad;

    point.deltaPitchRateRadps =
        delta.pitchRateRadps;

    point.deltaPitchAngleRad =
        delta.pitchAngleRad;

    point.deltaAngleOfAttackRad =
        delta.pitchAngleRad -
        delta.flightPathAngleRad;

    point.totalDownrangeM =
        total.downrangeM;

    point.totalAltitudeM =
        total.altitudeM;

    point.totalSpeedMps =
        total.speedMps;

    point.totalFlightPathAngleRad =
        total.flightPathAngleRad;

    point.totalPitchRateRadps =
        total.pitchRateRadps;

    point.totalPitchAngleRad =
        total.pitchAngleRad;

    point.totalAngleOfAttackRad =
        total.angleOfAttackRad();

    return point;
}

passive_flight::State makeNominalStateAtPerturbedImpact(
    const passive_flight::State& nominalImpactState,
    const passive_flight::StateDerivative& nominalImpactDerivative,
    double deltaTimeS
) {
    /*
     * Для терминальной поправки продолжаем
     * невозмущённую опорную траекторию линейно
     * от её собственного момента падения t_f*:
     *
     *     x*(t_f) ~= x*(t_f*) + x_dot*(t_f*) Delta t_f.
     *
     * Это не дополнительное физическое движение
     * объекта под землёй, а локальное продолжение
     * опорного решения, необходимое для согласованного
     * первого линейного приближения события H = 0.
     */
    passive_flight::State state =
        nominalImpactState;

    state.timeS +=
        deltaTimeS;

    state.speedMps +=
        nominalImpactDerivative.speedMps2 *
        deltaTimeS;

    state.flightPathAngleRad +=
        nominalImpactDerivative.flightPathAngleRadps *
        deltaTimeS;

    state.pitchRateRadps +=
        nominalImpactDerivative.pitchRateRadps2 *
        deltaTimeS;

    state.pitchAngleRad +=
        nominalImpactDerivative.pitchAngleRadps *
        deltaTimeS;

    state.downrangeM +=
        nominalImpactDerivative.downrangeMps *
        deltaTimeS;

    state.altitudeM +=
        nominalImpactDerivative.altitudeMps *
        deltaTimeS;

    return state;
}

PFPerturbedTrajectoryPoint makeTerminalImpactPoint(
    const passive_flight::PerturbedSimulationResult& result,
    const passive_flight::StateDerivative& finalNominalDerivative,
    const passive_flight::PerturbedImpactAnalysis& analysis
) {
    PFPerturbedTrajectoryPoint point{};

    const double deltaTimeS =
        analysis.changes.fallTimeS;

    const passive_flight::State nominal =
        makeNominalStateAtPerturbedImpact(
            result.finalNominalState,
            finalNominalDerivative,
            deltaTimeS
        );

    const passive_flight::State& total =
        analysis.estimatedImpactState;

    point.timeS =
        total.timeS;

    point.nominalDownrangeM =
        nominal.downrangeM;

    point.nominalAltitudeM =
        nominal.altitudeM;

    point.nominalSpeedMps =
        nominal.speedMps;

    point.nominalFlightPathAngleRad =
        nominal.flightPathAngleRad;

    point.nominalPitchRateRadps =
        nominal.pitchRateRadps;

    point.nominalPitchAngleRad =
        nominal.pitchAngleRad;

    point.nominalAngleOfAttackRad =
        nominal.angleOfAttackRad();

    /*
     * Delta в терминальной точке задаём как
     * разность полного оценённого состояния и
     * линейно продолженного номинального состояния
     * в тот же момент времени.
     *
     * В первом порядке это сохраняет физический
     * смысл Delta x(t), а полная высота точно
     * удовлетворяет условию поверхности H = 0.
     */
    point.deltaDownrangeM =
        total.downrangeM -
        nominal.downrangeM;

    point.deltaAltitudeM =
        total.altitudeM -
        nominal.altitudeM;

    point.deltaSpeedMps =
        total.speedMps -
        nominal.speedMps;

    point.deltaFlightPathAngleRad =
        total.flightPathAngleRad -
        nominal.flightPathAngleRad;

    point.deltaPitchRateRadps =
        total.pitchRateRadps -
        nominal.pitchRateRadps;

    point.deltaPitchAngleRad =
        total.pitchAngleRad -
        nominal.pitchAngleRad;

    point.deltaAngleOfAttackRad =
        point.deltaPitchAngleRad -
        point.deltaFlightPathAngleRad;

    point.totalDownrangeM =
        total.downrangeM;

    point.totalAltitudeM =
        total.altitudeM;

    point.totalSpeedMps =
        total.speedMps;

    point.totalFlightPathAngleRad =
        total.flightPathAngleRad;

    point.totalPitchRateRadps =
        total.pitchRateRadps;

    point.totalPitchAngleRad =
        total.pitchAngleRad;

    point.totalAngleOfAttackRad =
        total.angleOfAttackRad();

    return point;
}

std::vector<PFPerturbedTrajectoryPoint>
buildOutputTrajectory(
    const passive_flight::PerturbedSimulationResult& result,
    const passive_flight::StateDerivative& finalNominalDerivative,
    const passive_flight::PerturbedImpactAnalysis& analysis
) {
    std::vector<PFPerturbedTrajectoryPoint> points;

    const double nominalImpactTimeS =
        result.finalNominalState.timeS;

    const double perturbedImpactTimeS =
        analysis.estimatedImpactState.timeS;

    const double deltaTimeS =
        analysis.changes.fallTimeS;

    if (!std::isfinite(perturbedImpactTimeS) ||
        perturbedImpactTimeS < 0.0) {
        return points;
    }

    const bool impactTimesCoincide =
        std::abs(deltaTimeS) <=
        kTimeComparisonToleranceS;

    points.reserve(
        result.history.size() + 1
    );

    if (impactTimesCoincide) {
        for (const auto& sample : result.history) {
            points.push_back(
                makePoint(sample)
            );
        }

        return points;
    }

    /*
     * Если возмущённый объект падает раньше
     * номинального, историю обрезаем на его
     * собственном моменте падения.
     *
     * Если позже — сохраняем всю номинальную
     * историю до t_f* и затем добавляем
     * терминальную точку при t_f.
     */
    for (const auto& sample : result.history) {
        const double timeS =
            sample.nominalState.timeS;

        if (timeS <
            perturbedImpactTimeS -
                kTimeComparisonToleranceS) {
            points.push_back(
                makePoint(sample)
            );
            continue;
        }

        if (perturbedImpactTimeS >=
            nominalImpactTimeS -
                kTimeComparisonToleranceS) {
            points.push_back(
                makePoint(sample)
            );
            continue;
        }

        break;
    }

    const auto terminalPoint =
        makeTerminalImpactPoint(
            result,
            finalNominalDerivative,
            analysis
        );

    if (!points.empty() &&
        std::abs(
            points.back().timeS -
            terminalPoint.timeS
        ) <= kTimeComparisonToleranceS) {
        points.back() =
            terminalPoint;
    } else {
        points.push_back(
            terminalPoint
        );
    }

    return points;
}

} // namespace

extern "C" {

int32_t PF_CALL
pfCalculatePerturbedTrajectory(
    const PFPerturbedSimulationInput* input,

    PFPerturbedTrajectoryPoint* points,
    uint64_t pointCapacity,

    uint64_t* requiredPointCount,
    uint64_t* writtenPointCount
) {
    if (input == nullptr ||
        requiredPointCount == nullptr ||
        writtenPointCount == nullptr) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    *requiredPointCount = 0;
    *writtenPointCount = 0;

    if (!isValidInput(
            *input
        )) {
        return PF_RESULT_INVALID_INPUT;
    }

    try {
        const passive_flight::ObjectModel* object =
            registry().findObject(
                input->objectId
            );

        if (object == nullptr) {
            return PF_RESULT_OBJECT_NOT_FOUND;
        }

        const passive_flight::
            PerturbedTrajectorySimulator simulator(
                *object
            );

        const auto result =
            simulator.simulate(
                makeRequest(*input),
                makeInitialPerturbation(*input),
                makeTrajectoryOptions()
            );

        if (result.terminationReason !=
            passive_flight::
                TerminationReason::GroundReached) {
            return PF_RESULT_SIMULATION_FAILED;
        }

        const auto finalNominalEvaluation =
            simulator
                .perturbationDynamics()
                .nominalDynamics()
                .evaluate(
                    result.finalNominalState
                );

        const auto impactAnalysis =
            passive_flight::
                analyzePerturbedImpact(
                    result,
                    finalNominalEvaluation.derivative
                );

        if (!impactAnalysis.available) {
            return PF_RESULT_SIMULATION_FAILED;
        }

        const auto outputTrajectory =
            buildOutputTrajectory(
                result,
                finalNominalEvaluation.derivative,
                impactAnalysis
            );

        const uint64_t necessaryPointCount =
            static_cast<uint64_t>(
                outputTrajectory.size()
            );

        *requiredPointCount =
            necessaryPointCount;

        if (necessaryPointCount == 0) {
            return PF_RESULT_SIMULATION_FAILED;
        }

        if (points == nullptr ||
            pointCapacity <
                necessaryPointCount) {
            return PF_RESULT_BUFFER_TOO_SMALL;
        }

        for (uint64_t index = 0;
             index < necessaryPointCount;
             ++index) {
            points[index] =
                outputTrajectory[
                    static_cast<std::size_t>(
                        index
                    )
                ];
        }

        *writtenPointCount =
            necessaryPointCount;

        return PF_RESULT_OK;
    } catch (...) {
        *requiredPointCount = 0;
        *writtenPointCount = 0;

        return PF_RESULT_INTERNAL_ERROR;
    }
}

} // extern "C"
