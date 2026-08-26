#include "passive_flight/ModelContract.hpp"

#include <cmath>
#include <stdexcept>

namespace passive_flight {

ValidationIssues validate(
    const ReleaseConditions& release
) {
    ValidationIssues issues;

    if (!std::isfinite(release.altitudeM)) {
        issues.push_back({
            "release.altitudeM",
            "Altitude must be finite"
        });
    } else if (release.altitudeM <= 0.0) {
        issues.push_back({
            "release.altitudeM",
            "Altitude must be greater than zero"
        });
    }

    if (!std::isfinite(release.speedMps)) {
        issues.push_back({
            "release.speedMps",
            "Speed must be finite"
        });
    } else if (release.speedMps <= 0.0) {
        issues.push_back({
            "release.speedMps",
            "Speed must be greater than zero"
        });
    }

    return issues;
}

ValidationIssues validate(
    const SimulationRequest& request
) {
    ValidationIssues issues = validate(request.release);

    if (request.objectId.empty()) {
        issues.push_back({
            "objectId",
            "Object identifier must not be empty"
        });
    }

    return issues;
}

bool isValid(
    const ValidationIssues& issues
) noexcept {
    return issues.empty();
}

State makeHorizontalReleaseState(
    const ReleaseConditions& release
) {
    const ValidationIssues issues = validate(release);

    if (!isValid(issues)) {
        throw std::invalid_argument(
            "Invalid horizontal release conditions"
        );
    }

    State state;

    state.timeS = 0.0;

    state.speedMps = release.speedMps;
    state.flightPathAngleRad = 0.0;
    state.pitchRateRadps = 0.0;
    state.pitchAngleRad = 0.0;

    state.downrangeM = 0.0;
    state.altitudeM = release.altitudeM;

    return state;
}

SimulationSummary summarize(
    const SimulationResult& result
) {
    SimulationSummary summary;

    summary.downrangeM =
        result.finalState.downrangeM;

    summary.fallTimeS =
        result.finalState.timeS;

    summary.impactSpeedMps =
        result.finalState.speedMps;

    summary.impactFlightPathAngleRad =
        result.finalState.flightPathAngleRad;

    summary.impactPitchAngleRad =
        result.finalState.pitchAngleRad;

    summary.impactAngleOfAttackRad =
        result.finalState.angleOfAttackRad();

    summary.terminationReason =
        result.terminationReason;

    return summary;
}

const char* terminationReasonName(
    TerminationReason reason
) noexcept {
    switch (reason) {
    case TerminationReason::GroundReached:
        return "ground_reached";

    case TerminationReason::MaximumTimeReached:
        return "maximum_time_reached";

    case TerminationReason::MaximumStepsReached:
        return "maximum_steps_reached";

    case TerminationReason::InvalidInput:
        return "invalid_input";

    case TerminationReason::InvalidState:
        return "invalid_state";
    }

    return "unknown";
}

} // namespace passive_flight