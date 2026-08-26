#include "passive_flight/ForwardEulerSimulator.hpp"
#include "passive_flight/ModelContract.hpp"
#include "passive_flight/ObjectPassport.hpp"

#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>

namespace {

struct CalculationCase {
    double altitudeM{};
    double speedMps{};
};

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

    request.objectId =
        object.id;

    request.release.altitudeM =
        altitudeM;

    request.release.speedMps =
        speedMps;

    return request;
}

passive_flight::SimulationOptions
makeCalculationOptions(
    bool saveHistory
) {
    passive_flight::SimulationOptions options;

    /*
     * Основной шаг метода Эйлера,
     * согласованный с постановкой задачи.
     */
    options.timeStepS = 0.001;

    options.maximumTimeS = 300.0;
    options.maximumSteps = 2'000'000;

    options.groundAltitudeM = 0.0;

    options.saveHistory = saveHistory;

    /*
     * При шаге 0.001 с сохраняется одна точка
     * через каждые 100 шагов, то есть через 0.1 с.
     */
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
            << sample.state.timeS
            << ','

            << sample.state.downrangeM
            << ','

            << sample.state.altitudeM
            << ','

            << sample.state.speedMps
            << ','

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

            << sample.state.pitchRateRadps
            << ','

            << sample.diagnostics.angleOfAttackRateRadps
            << ','

            << sample.diagnostics.mach
            << ','

            << sample.diagnostics.reynolds
            << ','

            << sample.diagnostics.dynamicPressurePa
            << ','

            << sample.diagnostics.dragCoefficient
            << ','

            << sample.diagnostics.liftCoefficient
            << ','

            << sample.diagnostics.pitchingMomentCoefficient
            << ','

            << sample.diagnostics.dragN
            << ','

            << sample.diagnostics.liftN
            << ','

            << sample.diagnostics.pitchingMomentNm
            << '\n';
    }

    return output.good();
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
        const auto request =
            makeRequest(
                object,
                calculationCase.altitudeM,
                calculationCase.speedMps
            );

        const auto result =
            simulator.simulate(
                request,
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

    const auto request =
        makeRequest(
            object,
            altitudeM,
            speedMps
        );

    const auto options =
        makeCalculationOptions(true);

    const auto result =
        simulator.simulate(
            request,
            options
        );

    constexpr const char* fileName =
        "nominal_trajectory.csv";

    std::cout << '\n';

    std::cout
        << "Detailed trajectory case:"
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
            << "Failed to write trajectory file: "
            << fileName
            << '\n';
    }
}

} // namespace

int main() {
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