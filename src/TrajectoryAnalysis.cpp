#include "passive_flight/TrajectoryAnalysis.hpp"

#include "passive_flight/ObjectAerodynamicsAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace passive_flight {
namespace {

constexpr double kCoefficientTolerance = 1.0e-12;
constexpr double kDerivativeStepRad = 1.0e-5;

double liftToDragRatio(
    double cy,
    double cx
) {
    if (std::abs(cx) <= kCoefficientTolerance) {
        return 0.0;
    }

    return cy / cx;
}

AerodynamicInput makeStaticAerodynamicInput(
    double mach,
    double angleOfAttackRad
) {
    AerodynamicInput input;

    input.mach = mach;
    input.angleOfAttackRad = angleOfAttackRad;

    input.pitchRateRadS = 0.0;
    input.angleOfAttackRateRadS = 0.0;

    /*
     * При нулевых угловых скоростях значение скорости
     * не влияет на статические коэффициенты.
     */
    input.speedMps = 300.0;

    return input;
}

} // namespace

TrajectoryAnalysis analyzeTrajectory(
    const SimulationResult& result,
    const ObjectModel& object,
    double angleOfAttackSettlingBandRad
) {
    if (!std::isfinite(
            angleOfAttackSettlingBandRad
        ) ||
        angleOfAttackSettlingBandRad <= 0.0) {
        throw std::invalid_argument(
            "Angle-of-attack settling band must be positive"
        );
    }

    TrajectoryAnalysis analysis;

    if (result.history.empty()) {
        return analysis;
    }

    analysis.available = true;
    analysis.sampleCount = result.history.size();

    analysis.releaseAltitudeM =
        result.history.front().state.altitudeM;

    analysis.maximumAltitudeM =
        -std::numeric_limits<double>::infinity();

    analysis.minimumAngleOfAttackRad =
        std::numeric_limits<double>::infinity();

    analysis.maximumAngleOfAttackRad =
        -std::numeric_limits<double>::infinity();

    analysis.minimumEffectiveWingAngleRad =
        std::numeric_limits<double>::infinity();

    analysis.maximumEffectiveWingAngleRad =
        -std::numeric_limits<double>::infinity();

    analysis.minimumMach =
        std::numeric_limits<double>::infinity();

    analysis.maximumMach =
        -std::numeric_limits<double>::infinity();

    analysis.minimumCx =
        std::numeric_limits<double>::infinity();

    analysis.maximumCx =
        -std::numeric_limits<double>::infinity();

    analysis.minimumCy =
        std::numeric_limits<double>::infinity();

    analysis.maximumCy =
        -std::numeric_limits<double>::infinity();

    analysis.minimumMz =
        std::numeric_limits<double>::infinity();

    analysis.maximumMz =
        -std::numeric_limits<double>::infinity();

    analysis.minimumLiftToDragRatio =
        std::numeric_limits<double>::infinity();

    analysis.maximumLiftToDragRatio =
        -std::numeric_limits<double>::infinity();

    for (const TrajectorySample& sample :
         result.history) {
        const double angleOfAttackRad =
            sample.state.angleOfAttackRad();

        const double effectiveWingAngleRad =
            angleOfAttackRad +
            object.wing.installationAngleRad;

        const double cx =
            sample.diagnostics.dragCoefficient;

        const double cy =
            sample.diagnostics.liftCoefficient;

        const double mz =
            sample.diagnostics.pitchingMomentCoefficient;

        const double currentLiftToDragRatio =
            liftToDragRatio(cy, cx);

        if (sample.state.altitudeM >
            analysis.maximumAltitudeM) {
            analysis.maximumAltitudeM =
                sample.state.altitudeM;

            analysis.timeAtMaximumAltitudeS =
                sample.state.timeS;
        }

        analysis.minimumAngleOfAttackRad =
            std::min(
                analysis.minimumAngleOfAttackRad,
                angleOfAttackRad
            );

        analysis.maximumAngleOfAttackRad =
            std::max(
                analysis.maximumAngleOfAttackRad,
                angleOfAttackRad
            );

        analysis.minimumEffectiveWingAngleRad =
            std::min(
                analysis.minimumEffectiveWingAngleRad,
                effectiveWingAngleRad
            );

        analysis.maximumEffectiveWingAngleRad =
            std::max(
                analysis.maximumEffectiveWingAngleRad,
                effectiveWingAngleRad
            );

        analysis.maximumAbsolutePitchRateRadps =
            std::max(
                analysis.maximumAbsolutePitchRateRadps,
                std::abs(sample.state.pitchRateRadps)
            );

        analysis.minimumMach =
            std::min(
                analysis.minimumMach,
                sample.diagnostics.mach
            );

        analysis.maximumMach =
            std::max(
                analysis.maximumMach,
                sample.diagnostics.mach
            );

        analysis.maximumDynamicPressurePa =
            std::max(
                analysis.maximumDynamicPressurePa,
                sample.diagnostics.dynamicPressurePa
            );

        analysis.minimumCx =
            std::min(
                analysis.minimumCx,
                cx
            );

        analysis.maximumCx =
            std::max(
                analysis.maximumCx,
                cx
            );

        analysis.minimumCy =
            std::min(
                analysis.minimumCy,
                cy
            );

        analysis.maximumCy =
            std::max(
                analysis.maximumCy,
                cy
            );

        analysis.minimumMz =
            std::min(
                analysis.minimumMz,
                mz
            );

        analysis.maximumMz =
            std::max(
                analysis.maximumMz,
                mz
            );

        analysis.minimumLiftToDragRatio =
            std::min(
                analysis.minimumLiftToDragRatio,
                currentLiftToDragRatio
            );

        analysis.maximumLiftToDragRatio =
            std::max(
                analysis.maximumLiftToDragRatio,
                currentLiftToDragRatio
            );
    }

    analysis.altitudeGainM =
        analysis.maximumAltitudeM -
        analysis.releaseAltitudeM;

    const TrajectorySample& finalSample =
        result.history.back();

    analysis.finalLiftToDragRatio =
        liftToDragRatio(
            finalSample.diagnostics.liftCoefficient,
            finalSample.diagnostics.dragCoefficient
        );

    /*
     * Поиск времени установления угла атаки.
     */
    const double finalAngleOfAttackRad =
        finalSample.state.angleOfAttackRad();

    analysis.angleOfAttackSettlingTimeS =
        finalSample.state.timeS;

    for (std::size_t firstIndex = 0;
         firstIndex < result.history.size();
         ++firstIndex) {
        bool remainsInsideBand = true;

        for (std::size_t currentIndex = firstIndex;
             currentIndex < result.history.size();
             ++currentIndex) {
            const double currentAngleOfAttackRad =
                result.history[currentIndex]
                    .state
                    .angleOfAttackRad();

            if (std::abs(
                    currentAngleOfAttackRad -
                    finalAngleOfAttackRad
                ) >
                angleOfAttackSettlingBandRad) {
                remainsInsideBand = false;
                break;
            }
        }

        if (remainsInsideBand) {
            analysis.angleOfAttackSettlingTimeS =
                result.history[firstIndex]
                    .state
                    .timeS;

            break;
        }
    }

    return analysis;
}

TrajectoryAnalysis analyzeTrajectory(
    const SimulationResult& result,
    const ObjectModel& object
) {
    constexpr double defaultSettlingBandRad =
        0.1 *
        std::numbers::pi_v<double> /
        180.0;

    return analyzeTrajectory(
        result,
        object,
        defaultSettlingBandRad
    );
}

AerodynamicBalanceAnalysis analyzeAerodynamicBalance(
    const ObjectModel& object,
    double mach
) {
    if (!std::isfinite(mach) || mach < 0.0) {
        throw std::invalid_argument(
            "Mach number must be finite and non-negative"
        );
    }

    /*
     * После введения общего интерфейса makeAerodynamicModel()
     * возвращает shared_ptr<const AerodynamicModel>.
     *
     * Анализ балансировки не должен зависеть от конкретной
     * реализации аэродинамики, поэтому здесь также работаем
     * только через общий контракт AerodynamicModel.
     */
    const auto model =
        makeAerodynamicModel(object);

    if (!model) {
        throw std::runtime_error(
            "Aerodynamic model must not be null"
        );
    }

    const AerodynamicCoefficients atZero =
        model->evaluate(
            makeStaticAerodynamicInput(
                mach,
                0.0
            )
        );

    const AerodynamicCoefficients atPositiveStep =
        model->evaluate(
            makeStaticAerodynamicInput(
                mach,
                kDerivativeStepRad
            )
        );

    const double mzAlphaPerRad =
        (
            atPositiveStep.mzStatic -
            atZero.mzStatic
        ) /
        kDerivativeStepRad;

    AerodynamicBalanceAnalysis analysis;

    analysis.mach = mach;
    analysis.mzAlphaPerRad = mzAlphaPerRad;

    analysis.staticallyStable =
        mzAlphaPerRad < 0.0;

    if (!std::isfinite(mzAlphaPerRad) ||
        std::abs(mzAlphaPerRad) <=
            kCoefficientTolerance) {
        return analysis;
    }

    analysis.trimAngleOfAttackRad =
        -atZero.mzStatic /
        mzAlphaPerRad;

    analysis.trimEffectiveWingAngleRad =
        analysis.trimAngleOfAttackRad +
        object.wing.installationAngleRad;

    const AerodynamicCoefficients atTrim =
        model->evaluate(
            makeStaticAerodynamicInput(
                mach,
                analysis.trimAngleOfAttackRad
            )
        );

    analysis.trimCx =
        atTrim.cx;

    analysis.trimCy =
        atTrim.cy;

    analysis.trimLiftToDragRatio =
        liftToDragRatio(
            atTrim.cy,
            atTrim.cx
        );

    analysis.available =
        std::isfinite(
            analysis.trimAngleOfAttackRad
        ) &&
        std::isfinite(
            analysis.trimLiftToDragRatio
        );

    return analysis;
}

} // namespace passive_flight
