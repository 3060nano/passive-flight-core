#include "passive_flight/ForwardEulerSimulator.hpp"
#include "passive_flight/ModelContract.hpp"
#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/TrajectoryAnalysis.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

struct CalculationCase {
    double altitudeM{};
    double speedMps{};
};

void configureConsoleEncoding() {
#ifdef _WIN32
    SetConsoleCP(CP_UTF8);
    SetConsoleOutputCP(CP_UTF8);
#endif
}

double radiansToDegrees(
    double angleRad
) {
    return
        angleRad *
        180.0 /
        std::numbers::pi_v<double>;
}

passive_flight::SimulationRequest makeRequest(
    const passive_flight::ObjectModel& object,
    double altitudeM,
    double speedMps
) {
    passive_flight::SimulationRequest request;

    request.objectId = object.id;
    request.release.altitudeM = altitudeM;
    request.release.speedMps = speedMps;

    return request;
}

passive_flight::SimulationOptions
makeCalculationOptions(
    bool saveHistory
) {
    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 300.0;
    options.maximumSteps = 2'000'000;
    options.groundAltitudeM = 0.0;

    options.saveHistory = saveHistory;
    options.historyStride = 100;

    return options;
}

void printResultHeader() {
    std::cout
        << std::left
        << std::setw(12) << "H0, m"
        << std::setw(12) << "V0, m/s"
        << std::setw(22) << "Termination"
        << std::setw(14) << "Time, s"
        << std::setw(16) << "Range, m"
        << std::setw(16) << "Impact V"
        << std::setw(16) << "Theta, deg"
        << std::setw(16) << "Pitch, deg"
        << std::setw(16) << "Alpha, deg"
        << '\n';

    std::cout
        << std::string(140, '-')
        << '\n';
}

void printResult(
    const CalculationCase& calculationCase,
    const passive_flight::SimulationResult& result
) {
    const auto summary =
        passive_flight::summarize(result);

    std::cout
        << std::left
        << std::fixed
        << std::setprecision(3)

        << std::setw(12)
        << calculationCase.altitudeM

        << std::setw(12)
        << calculationCase.speedMps

        << std::setw(22)
        << passive_flight::terminationReasonName(
               result.terminationReason
           )

        << std::setw(14)
        << summary.fallTimeS

        << std::setw(16)
        << summary.downrangeM

        << std::setw(16)
        << summary.impactSpeedMps

        << std::setw(16)
        << radiansToDegrees(
               summary.impactFlightPathAngleRad
           )

        << std::setw(16)
        << radiansToDegrees(
               summary.impactPitchAngleRad
           )

        << std::setw(16)
        << radiansToDegrees(
               summary.impactAngleOfAttackRad
           )

        << '\n';
}

bool writeTrajectoryCsv(
    const std::string& fileName,
    const passive_flight::SimulationResult& result
) {
    std::ofstream output(fileName);

    if (!output.is_open()) {
        return false;
    }

    output
        << "time_s,"
        << "downrange_m,"
        << "altitude_m,"
        << "speed_mps,"
        << "flight_path_angle_deg,"
        << "pitch_angle_deg,"
        << "angle_of_attack_deg,"
        << "pitch_rate_radps,"
        << "angle_of_attack_rate_radps,"
        << "mach,"
        << "reynolds,"
        << "dynamic_pressure_pa,"
        << "cx,"
        << "cy,"
        << "mz,"
        << "drag_n,"
        << "normal_force_n,"
        << "pitching_moment_nm"
        << '\n';

    output
        << std::fixed
        << std::setprecision(10);

    for (const auto& sample : result.history) {
        output
            << sample.state.timeS << ','
            << sample.state.downrangeM << ','
            << sample.state.altitudeM << ','
            << sample.state.speedMps << ','

            << radiansToDegrees(
                   sample.state.flightPathAngleRad
               )
            << ','

            << radiansToDegrees(
                   sample.state.pitchAngleRad
               )
            << ','

            << radiansToDegrees(
                   sample.state.angleOfAttackRad()
               )
            << ','

            << sample.state.pitchRateRadps << ','

            << sample.diagnostics
                   .angleOfAttackRateRadps
            << ','

            << sample.diagnostics.mach << ','
            << sample.diagnostics.reynolds << ','
            << sample.diagnostics.dynamicPressurePa << ','
            << sample.diagnostics.dragCoefficient << ','
            << sample.diagnostics.liftCoefficient << ','
            << sample.diagnostics.pitchingMomentCoefficient << ','
            << sample.diagnostics.dragN << ','
            << sample.diagnostics.liftN << ','
            << sample.diagnostics.pitchingMomentNm
            << '\n';
    }

    return output.good();
}

void printTrajectoryAnalysis(
    const passive_flight::TrajectoryAnalysis& analysis,
    const passive_flight::AerodynamicBalanceAnalysis& balance
) {
    if (!analysis.available) {
        std::cout
            << "Trajectory analysis is unavailable."
            << '\n';

        return;
    }

    std::cout
        << std::fixed
        << std::setprecision(4)
        << "\nTrajectory analysis:"
        << '\n'

        << "  samples: "
        << analysis.sampleCount
        << '\n'

        << "  maximum altitude: "
        << analysis.maximumAltitudeM
        << " m"
        << '\n'

        << "  altitude gain: "
        << analysis.altitudeGainM
        << " m"
        << '\n'

        << "  time at maximum altitude: "
        << analysis.timeAtMaximumAltitudeS
        << " s"
        << '\n'

        << "  alpha min/max: "
        << radiansToDegrees(
               analysis.minimumAngleOfAttackRad
           )
        << " / "
        << radiansToDegrees(
               analysis.maximumAngleOfAttackRad
           )
        << " deg"
        << '\n'

        << "  effective wing alpha min/max: "
        << radiansToDegrees(
               analysis.minimumEffectiveWingAngleRad
           )
        << " / "
        << radiansToDegrees(
               analysis.maximumEffectiveWingAngleRad
           )
        << " deg"
        << '\n'

        << "  maximum absolute pitch rate: "
        << analysis.maximumAbsolutePitchRateRadps
        << " rad/s"
        << '\n'

        << "  Mach min/max: "
        << analysis.minimumMach
        << " / "
        << analysis.maximumMach
        << '\n'

        << "  maximum dynamic pressure: "
        << analysis.maximumDynamicPressurePa
        << " Pa"
        << '\n'

        << "  Cx min/max: "
        << analysis.minimumCx
        << " / "
        << analysis.maximumCx
        << '\n'

        << "  Cy min/max: "
        << analysis.minimumCy
        << " / "
        << analysis.maximumCy
        << '\n'

        << "  Mz min/max: "
        << analysis.minimumMz
        << " / "
        << analysis.maximumMz
        << '\n'

        << "  lift-to-drag min/max/final: "
        << analysis.minimumLiftToDragRatio
        << " / "
        << analysis.maximumLiftToDragRatio
        << " / "
        << analysis.finalLiftToDragRatio
        << '\n'

        << "  alpha settling time: "
        << analysis.angleOfAttackSettlingTimeS
        << " s"
        << '\n';

    if (!balance.available) {
        std::cout
            << "  aerodynamic balance: unavailable"
            << '\n';

        return;
    }

    std::cout
        << "\nAerodynamic balance at final Mach:"
        << '\n'

        << "  Mach: "
        << balance.mach
        << '\n'

        << "  mz_alpha: "
        << balance.mzAlphaPerRad
        << " 1/rad"
        << '\n'

        << "  statically stable: "
        << (
               balance.staticallyStable
                   ? "yes"
                   : "no"
           )
        << '\n'

        << "  trim alpha: "
        << radiansToDegrees(
               balance.trimAngleOfAttackRad
           )
        << " deg"
        << '\n'

        << "  trim effective wing alpha: "
        << radiansToDegrees(
               balance.trimEffectiveWingAngleRad
           )
        << " deg"
        << '\n'

        << "  trim Cx: "
        << balance.trimCx
        << '\n'

        << "  trim Cy: "
        << balance.trimCy
        << '\n'

        << "  trim lift-to-drag ratio: "
        << balance.trimLiftToDragRatio
        << '\n';
}

void runCalculationTable(
    const passive_flight::ObjectModel& object,
    const passive_flight::ForwardEulerSimulator& simulator
) {
    const std::array<CalculationCase, 6> cases{{
        {100.0, 200.0},
        {1000.0, 200.0},
        {1000.0, 300.0},
        {5000.0, 200.0},
        {5000.0, 300.0},
        {10000.0, 300.0}
    }};

    const auto options =
        makeCalculationOptions(false);

    printResultHeader();

    for (const auto& calculationCase : cases) {
        const auto result =
            simulator.simulate(
                makeRequest(
                    object,
                    calculationCase.altitudeM,
                    calculationCase.speedMps
                ),
                options
            );

        printResult(
            calculationCase,
            result
        );
    }
}

void createDetailedTrajectory(
    const passive_flight::ObjectModel& object,
    const passive_flight::ForwardEulerSimulator& simulator
) {
    constexpr double altitudeM = 5000.0;
    constexpr double speedMps = 300.0;

    const auto result =
        simulator.simulate(
            makeRequest(
                object,
                altitudeM,
                speedMps
            ),
            makeCalculationOptions(true)
        );

    constexpr const char* fileName =
        "nominal_trajectory.csv";

    std::cout
        << "\nDetailed trajectory case:"
        << '\n'
        << "  H0 = " << altitudeM << " m"
        << '\n'
        << "  V0 = " << speedMps << " m/s"
        << '\n'
        << "  termination = "
        << passive_flight::terminationReasonName(
               result.terminationReason
           )
        << '\n'
        << "  saved samples = "
        << result.history.size()
        << '\n';

    if (writeTrajectoryCsv(
            fileName,
            result
        )) {
        std::cout
            << "Trajectory was written to: "
            << fileName
            << '\n';
    } else {
        std::cout
            << "Failed to write trajectory file."
            << '\n';
    }

    const auto trajectoryAnalysis =
        passive_flight::analyzeTrajectory(
            result,
            object
        );

    const auto balance =
        passive_flight::analyzeAerodynamicBalance(
            object,
            result.history.empty()
                ? 0.8
                : result.history.back()
                    .diagnostics
                    .mach
        );

    printTrajectoryAnalysis(
        trajectoryAnalysis,
        balance
    );
}

} // namespace

int main() {
    configureConsoleEncoding();

    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const passive_flight::ForwardEulerSimulator simulator(
        passport.object
    );

    std::cout
        << "Passive Flight Core"
        << '\n'
        << "Object ID: "
        << passport.object.id
        << '\n'
        << "Object name: "
        << passport.object.metadata.displayName
        << '\n'
        << "Integration method: Forward Euler"
        << '\n'
        << "Time step: 0.001 s"
        << "\n\n";

    runCalculationTable(
        passport.object,
        simulator
    );

    createDetailedTrajectory(
        passport.object,
        simulator
    );

    return 0;
}