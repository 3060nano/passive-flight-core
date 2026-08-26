#include "passive_flight/ObjectPassport.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <variant>

namespace {

using passive_flight::NoseShape;
using passive_flight::ObjectPassport;
using passive_flight::ParameterRecord;
using passive_flight::ParameterStatus;
using passive_flight::ValidationIssues;

constexpr double kDegreesToRadians =
    std::numbers::pi_v<double> / 180.0;

void require(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& name
) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(
            name
            + ": actual="
            + std::to_string(actual)
            + ", expected="
            + std::to_string(expected)
        );
    }
}

double numericValue(
    const ParameterRecord& record
) {
    const double* value =
        std::get_if<double>(&record.value);

    if (value == nullptr) {
        throw std::runtime_error(
            record.path
            + " does not contain a numeric value"
        );
    }

    return *value;
}

void testPassportIdentity() {
    const ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    require(
        passport.object.id
            == "ABSTRACT_500_UMPK_V1",
        "Object identifier is incorrect"
    );

    require(
        passport.object.metadata.modelVersion
            == "0.2.0",
        "Object model version is incorrect"
    );

    require(
        passport.object.body.noseShape
            == NoseShape::Ogival,
        "Baseline nose shape must be ogival"
    );
}

void testMassProperties() {
    const ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    requireNear(
        passport.object.mass.massKg,
        500.0,
        0.0,
        "Object mass"
    );

    requireNear(
        passport.object.mass
            .pitchMomentOfInertiaKgM2,
        250.0,
        0.0,
        "Pitch moment of inertia"
    );

    /*
     * Центр масс перемещён вперёд относительно
     * предыдущей конфигурации 1,20 м.
     */
    requireNear(
        passport.object.mass.centerOfMassXM,
        1.17,
        0.0,
        "Center of mass"
    );
}

void testWingGeometry() {
    const ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    requireNear(
        passport.object.wing.areaM2,
        0.475,
        0.0,
        "Wing area"
    );

    requireNear(
        passport.object.wing.spanM,
        1.760,
        0.0,
        "Wing span"
    );

    requireNear(
        passport.object.wing
            .meanAerodynamicChordM,
        0.274,
        0.0,
        "Wing mean aerodynamic chord"
    );

    requireNear(
        passport.object.wing.aspectRatio(),
        6.521263157894737,
        1.0e-12,
        "Wing aspect ratio"
    );

    /*
     * Предварительный положительный угол
     * установки крыла равен четырём градусам.
     */
    requireNear(
        passport.object.wing.installationAngleRad,
        4.0 * kDegreesToRadians,
        1.0e-12,
        "Wing installation angle"
    );
}

void testTailGeometry() {
    const ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    requireNear(
        passport.object.tail.areaM2,
        0.269,
        0.0,
        "Tail area"
    );

    requireNear(
        passport.object.tail.spanM,
        0.514,
        0.0,
        "Tail span"
    );

    requireNear(
        passport.object.tail.aspectRatio(),
        0.9821412639405205,
        1.0e-12,
        "Tail aspect ratio"
    );

    requireNear(
        passport.object.tail.installationAngleRad,
        0.0,
        0.0,
        "Tail installation angle"
    );
}

void testParameterProvenance() {
    const ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    const ParameterRecord* mass =
        passive_flight::findParameter(
            passport,
            "mass.massKg"
        );

    require(
        mass != nullptr,
        "Mass provenance record is missing"
    );

    requireNear(
        numericValue(*mass),
        500.0,
        0.0,
        "Recorded mass"
    );

    require(
        mass->status
            == ParameterStatus::Requirement,
        "Mass must have Requirement status"
    );

    const ParameterRecord* centerOfMass =
        passive_flight::findParameter(
            passport,
            "mass.centerOfMassXM"
        );

    require(
        centerOfMass != nullptr,
        "Center-of-mass provenance record is missing"
    );

    requireNear(
        numericValue(*centerOfMass),
        1.17,
        0.0,
        "Recorded center of mass"
    );

    require(
        centerOfMass->status
            == ParameterStatus::Provisional,
        "Center of mass must have Provisional status"
    );

    const ParameterRecord* wingArea =
        passive_flight::findParameter(
            passport,
            "wing.areaM2"
        );

    require(
        wingArea != nullptr,
        "Wing area provenance record is missing"
    );

    require(
        wingArea->status
            == ParameterStatus::SourceDocument,
        "Wing area must have SourceDocument status"
    );

    const ParameterRecord* wingInstallationAngle =
        passive_flight::findParameter(
            passport,
            "wing.installationAngleRad"
        );

    require(
        wingInstallationAngle != nullptr,
        "Wing installation-angle provenance record is missing"
    );

    requireNear(
        numericValue(*wingInstallationAngle),
        4.0 * kDegreesToRadians,
        1.0e-12,
        "Recorded wing installation angle"
    );

    require(
        wingInstallationAngle->status
            == ParameterStatus::Provisional,
        "Wing installation angle must have Provisional status"
    );

    const ParameterRecord* inertia =
        passive_flight::findParameter(
            passport,
            "mass.pitchMomentOfInertiaKgM2"
        );

    require(
        inertia != nullptr,
        "Inertia provenance record is missing"
    );

    require(
        inertia->status
            == ParameterStatus::Provisional,
        "Inertia must have Provisional status"
    );
}

void testValidPassport() {
    const ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    const ValidationIssues issues =
        passive_flight::validate(passport);

    if (!issues.empty()) {
        std::string message =
            "Passport validation failed:";

        for (const auto& issue : issues) {
            message +=
                "\n"
                + issue.field
                + ": "
                + issue.message;
        }

        throw std::runtime_error(message);
    }
}

void testInvalidMassDetected() {
    ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    passport.object.mass.massKg =
        -500.0;

    const ValidationIssues issues =
        passive_flight::validate(passport);

    require(
        !issues.empty(),
        "Negative mass must be detected"
    );
}

void testInvalidInstallationAngleDetected() {
    ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    passport.object.wing.installationAngleRad =
        std::numeric_limits<double>::quiet_NaN();

    const ValidationIssues issues =
        passive_flight::validate(passport);

    require(
        !issues.empty(),
        "Invalid wing installation angle must be detected"
    );
}

void testReferenceGeometryMismatchDetected() {
    ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    passport.object.reference.areaM2 =
        1.0;

    const ValidationIssues issues =
        passive_flight::validate(passport);

    require(
        !issues.empty(),
        "Reference area mismatch must be detected"
    );
}

void testDuplicateParameterDetected() {
    ObjectPassport passport =
        passive_flight::makeAbstract500UmpkPassport();

    require(
        !passport.parameters.empty(),
        "Passport parameter list must not be empty"
    );

    passport.parameters.push_back(
        passport.parameters.front()
    );

    const ValidationIssues issues =
        passive_flight::validate(passport);

    bool duplicateFound = false;

    for (const auto& issue : issues) {
        if (issue.message
            == "Duplicate parameter provenance record") {
            duplicateFound = true;
            break;
        }
    }

    require(
        duplicateFound,
        "Duplicate parameter must be detected"
    );
}

void testStatusNames() {
    require(
        std::string(
            passive_flight::parameterStatusName(
                ParameterStatus::SourceDocument
            )
        ) == "source_document",
        "Parameter status name is incorrect"
    );

    require(
        std::string(
            passive_flight::noseShapeName(
                NoseShape::Ogival
            )
        ) == "ogival",
        "Nose shape name is incorrect"
    );
}

} // namespace

int main() {
    try {
        testPassportIdentity();
        testMassProperties();
        testWingGeometry();
        testTailGeometry();
        testParameterProvenance();
        testValidPassport();
        testInvalidMassDetected();
        testInvalidInstallationAngleDetected();
        testReferenceGeometryMismatchDetected();
        testDuplicateParameterDetected();
        testStatusNames();

        std::cout
            << "All object passport tests passed."
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "Test failure: "
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }
}