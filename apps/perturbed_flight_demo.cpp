#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/PerturbedImpactAnalysis.hpp"
#include "passive_flight/PerturbedTrajectoryCsv.hpp"
#include "passive_flight/PerturbedTrajectorySimulator.hpp"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>

namespace {

constexpr double kRadiansToDegrees =
    180.0 / std::numbers::pi_v<double>;

constexpr double kDegreesToRadians =
    std::numbers::pi_v<double> / 180.0;

double deltaAngleOfAttackRad(
    const passive_flight::
        LongitudinalPerturbationState& perturbation
) {
    return
        perturbation.pitchAngleRad -
        perturbation.flightPathAngleRad;
}

} // namespace

int main(
    int argumentCount,
    char* argumentValues[]
) {
    const std::string outputPath =
        argumentCount >= 2
            ? argumentValues[1]
            : "perturbed_trajectory.csv";

    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const passive_flight::
        PerturbedTrajectorySimulator simulator(
            passport.object
        );

    const passive_flight::SimulationRequest request{
        passport.object.id,
        {
            100.0,
            200.0
        }
    };

    passive_flight::LongitudinalPerturbationState
        initialPerturbation;

    initialPerturbation.speedMps =
        1.0;

    initialPerturbation.pitchAngleRad =
        0.1 * kDegreesToRadians;

    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 30.0;
    options.maximumSteps = 100'000;
    options.saveHistory = true;
    options.historyStride = 10;

    const auto result =
        simulator.simulate(
            request,
            initialPerturbation,
            options
        );

    if (result.terminationReason !=
        passive_flight::
            TerminationReason::GroundReached) {

        std::cerr
            << "Perturbed simulation failed: "
            << passive_flight::terminationReasonName(
                result.terminationReason
            )
            << '\n';

        return EXIT_FAILURE;
    }

    std::ofstream output(outputPath);

    if (!output) {
        std::cerr
            << "Cannot open CSV file: "
            << outputPath
            << '\n';

        return EXIT_FAILURE;
    }

    try {
        passive_flight::writePerturbedTrajectoryCsv(
            result,
            output
        );
    } catch (const std::exception& error) {
        std::cerr
            << "Cannot write CSV: "
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }

    /*
     * Правая часть невозмущённой системы
     * непосредственно в точке номинального падения.
     *
     * Она нужна для чувствительности параметров
     * события падения к сдвигу времени.
     */
    const auto finalNominalEvaluation =
        simulator
            .perturbationDynamics()
            .nominalDynamics()
            .evaluate(
                result.finalNominalState
            );

    const auto impactAnalysis =
        passive_flight::analyzePerturbedImpact(
            result,
            finalNominalEvaluation.derivative
        );

    std::cout << std::setprecision(10);

    std::cout
        << "Perturbed longitudinal flight demo\n"
        << "Object: "
        << passport.object.id
        << '\n'

        << "Release altitude: "
        << request.release.altitudeM
        << " m\n"

        << "Release speed: "
        << request.release.speedMps
        << " m/s\n"

        << "Initial Delta V: "
        << initialPerturbation.speedMps
        << " m/s\n"

        << "Initial Delta theta: "
        << initialPerturbation.pitchAngleRad *
            kRadiansToDegrees
        << " deg\n"

        << "Initial Delta alpha: "
        << deltaAngleOfAttackRad(
            initialPerturbation
        ) *
            kRadiansToDegrees
        << " deg\n\n";

    std::cout
        << "Nominal impact:\n"

        << "Time: "
        << result.finalNominalState.timeS
        << " s\n"

        << "Downrange: "
        << result.finalNominalState.downrangeM
        << " m\n"

        << "Speed: "
        << result.finalNominalState.speedMps
        << " m/s\n\n";

    /*
     * Эти Delta относятся к одному и тому же
     * моменту времени t_f*.
     */
    std::cout
        << "Perturbation at nominal impact time:\n"

        << "Delta V: "
        << result.finalPerturbation.speedMps
        << " m/s\n"

        << "Delta Theta: "
        << result.finalPerturbation
               .flightPathAngleRad *
            kRadiansToDegrees
        << " deg\n"

        << "Delta omega_z: "
        << result.finalPerturbation
               .pitchRateRadps *
            kRadiansToDegrees
        << " deg/s\n"

        << "Delta theta: "
        << result.finalPerturbation
               .pitchAngleRad *
            kRadiansToDegrees
        << " deg\n"

        << "Delta alpha: "
        << deltaAngleOfAttackRad(
            result.finalPerturbation
        ) *
            kRadiansToDegrees
        << " deg\n"

        << "Delta x: "
        << result.finalPerturbation.downrangeM
        << " m\n"

        << "Delta H: "
        << result.finalPerturbation.altitudeM
        << " m\n\n";

    if (!impactAnalysis.available) {
        std::cerr
            << "Perturbed impact analysis "
               "is unavailable\n";

        return EXIT_FAILURE;
    }

    const auto& changes =
        impactAnalysis.changes;

    const auto& perturbedImpact =
        impactAnalysis.estimatedImpactState;

    /*
     * Здесь уже выводятся не Delta(t_f*),
     * а изменения параметров самого события H = 0.
     */
    std::cout
        << "Perturbed impact event, "
           "first-order estimate:\n"

        << "Nominal vertical speed at impact: "
        << impactAnalysis.nominalVerticalSpeedMps
        << " m/s\n"

        << "Delta impact time: "
        << changes.fallTimeS
        << " s\n"

        << "Perturbed impact time: "
        << perturbedImpact.timeS
        << " s\n"

        << "Delta impact range: "
        << changes.downrangeM
        << " m\n"

        << "Perturbed impact range: "
        << perturbedImpact.downrangeM
        << " m\n"

        << "Delta impact speed: "
        << changes.speedMps
        << " m/s\n"

        << "Perturbed impact speed: "
        << perturbedImpact.speedMps
        << " m/s\n"

        << "Delta impact Theta: "
        << changes.flightPathAngleRad *
            kRadiansToDegrees
        << " deg\n"

        << "Delta impact omega_z: "
        << changes.pitchRateRadps *
            kRadiansToDegrees
        << " deg/s\n"

        << "Delta impact theta: "
        << changes.pitchAngleRad *
            kRadiansToDegrees
        << " deg\n"

        << "Delta impact alpha: "
        << changes.angleOfAttackRad() *
            kRadiansToDegrees
        << " deg\n"

        << "Impact altitude: "
        << perturbedImpact.altitudeM
        << " m\n\n";

    std::cout
        << "CSV points: "
        << result.history.size()
        << '\n'

        << "CSV file: "
        << outputPath
        << '\n';

    return EXIT_SUCCESS;
}