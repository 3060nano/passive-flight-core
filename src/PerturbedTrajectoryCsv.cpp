#include "passive_flight/PerturbedTrajectoryCsv.hpp"

#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace passive_flight {

void writePerturbedTrajectoryCsv(
    const PerturbedSimulationResult& result,
    std::ostream& output
) {
    if (!output) {
        throw std::invalid_argument(
            "CSV output stream is not writable"
        );
    }

    output
        << "time_s"
        << ";nominal_speed_mps"
        << ";delta_speed_mps"
        << ";total_speed_mps"
        << ";nominal_flight_path_angle_rad"
        << ";delta_flight_path_angle_rad"
        << ";total_flight_path_angle_rad"
        << ";nominal_pitch_rate_radps"
        << ";delta_pitch_rate_radps"
        << ";total_pitch_rate_radps"
        << ";nominal_pitch_angle_rad"
        << ";delta_pitch_angle_rad"
        << ";total_pitch_angle_rad"
        << ";nominal_angle_of_attack_rad"
        << ";delta_angle_of_attack_rad"
        << ";total_angle_of_attack_rad"
        << ";nominal_downrange_m"
        << ";delta_downrange_m"
        << ";total_downrange_m"
        << ";nominal_altitude_m"
        << ";delta_altitude_m"
        << ";total_altitude_m"
        << '\n';

    output << std::setprecision(17);

    for (const auto& sample : result.history) {
        const double nominalAngleOfAttackRad =
            sample.nominalState.angleOfAttackRad();

        const double deltaAngleOfAttackRad =
            sample.perturbation.pitchAngleRad -
            sample.perturbation.flightPathAngleRad;

        const double totalAngleOfAttackRad =
            sample.totalState.angleOfAttackRad();

        output
            << sample.nominalState.timeS
            << ';' << sample.nominalState.speedMps
            << ';' << sample.perturbation.speedMps
            << ';' << sample.totalState.speedMps
            << ';' << sample.nominalState.flightPathAngleRad
            << ';' << sample.perturbation.flightPathAngleRad
            << ';' << sample.totalState.flightPathAngleRad
            << ';' << sample.nominalState.pitchRateRadps
            << ';' << sample.perturbation.pitchRateRadps
            << ';' << sample.totalState.pitchRateRadps
            << ';' << sample.nominalState.pitchAngleRad
            << ';' << sample.perturbation.pitchAngleRad
            << ';' << sample.totalState.pitchAngleRad
            << ';' << nominalAngleOfAttackRad
            << ';' << deltaAngleOfAttackRad
            << ';' << totalAngleOfAttackRad
            << ';' << sample.nominalState.downrangeM
            << ';' << sample.perturbation.downrangeM
            << ';' << sample.totalState.downrangeM
            << ';' << sample.nominalState.altitudeM
            << ';' << sample.perturbation.altitudeM
            << ';' << sample.totalState.altitudeM
            << '\n';
    }

    if (!output) {
        throw std::runtime_error(
            "Failed to write perturbed trajectory CSV"
        );
    }
}

} // namespace passive_flight
