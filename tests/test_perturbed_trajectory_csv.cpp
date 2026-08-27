#include "passive_flight/PerturbedTrajectoryCsv.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

int failureCount = 0;

void check(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        ++failureCount;
    }
}

passive_flight::PerturbedSimulationResult
makeResult() {
    passive_flight::PerturbedSimulationResult result;

    passive_flight::PerturbedTrajectorySample sample;

    sample.nominalState = {
        2.0,
        200.0,
        -0.1,
        0.02,
        -0.08,
        350.0,
        75.0
    };

    sample.perturbation = {
        1.0,
        0.01,
        -0.005,
        0.02,
        3.0,
        -2.0
    };

    sample.totalState = {
        sample.nominalState.timeS,
        sample.nominalState.speedMps +
            sample.perturbation.speedMps,
        sample.nominalState.flightPathAngleRad +
            sample.perturbation.flightPathAngleRad,
        sample.nominalState.pitchRateRadps +
            sample.perturbation.pitchRateRadps,
        sample.nominalState.pitchAngleRad +
            sample.perturbation.pitchAngleRad,
        sample.nominalState.downrangeM +
            sample.perturbation.downrangeM,
        sample.nominalState.altitudeM +
            sample.perturbation.altitudeM
    };

    result.history.push_back(sample);

    return result;
}

void testHeaderAndDataRow() {
    std::ostringstream output;

    passive_flight::writePerturbedTrajectoryCsv(
        makeResult(),
        output
    );

    const std::string csv = output.str();

    check(
        csv.starts_with("time_s;nominal_speed_mps;"),
        "CSV header starts with time and speed columns"
    );

    check(
        csv.find("delta_angle_of_attack_rad") !=
            std::string::npos,
        "CSV contains Delta alpha column"
    );

    check(
        csv.find("2;200;1;201;") !=
            std::string::npos,
        "CSV contains nominal, Delta and total speed"
    );

    check(
        csv.find(";350;3;353;75;-2;73\n") !=
            std::string::npos,
        "CSV contains position and altitude values"
    );

    std::size_t newlineCount = 0;

    for (const char character : csv) {
        if (character == '\n') {
            ++newlineCount;
        }
    }

    check(
        newlineCount == 2,
        "CSV contains one header and one data row"
    );
}

void testEmptyHistoryWritesHeader() {
    std::ostringstream output;

    passive_flight::writePerturbedTrajectoryCsv(
        {},
        output
    );

    const std::string csv = output.str();

    check(
        !csv.empty(),
        "Empty history still writes a header"
    );

    check(
        csv.find('\n') == csv.size() - 1,
        "Empty history writes exactly one line"
    );
}

void testInvalidStreamIsRejected() {
    std::ostringstream output;
    output.setstate(std::ios::badbit);

    bool exceptionThrown = false;

    try {
        passive_flight::writePerturbedTrajectoryCsv(
            makeResult(),
            output
        );
    } catch (const std::invalid_argument&) {
        exceptionThrown = true;
    }

    check(
        exceptionThrown,
        "Invalid CSV stream is rejected"
    );
}

} // namespace

int main() {
    testHeaderAndDataRow();
    testEmptyHistoryWritesHeader();
    testInvalidStreamIsRejected();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " perturbed trajectory CSV test(s) failed\n";

        return 1;
    }

    std::cout
        << "All perturbed trajectory CSV tests passed\n";

    return 0;
}
