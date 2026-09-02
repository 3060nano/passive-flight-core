#include "passive_flight/ObjectPassport.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <variant>

namespace {

using passive_flight::AerodynamicModelType;
using passive_flight::NoseShape;
using passive_flight::ObjectPassport;
using passive_flight::ParameterRecord;
using passive_flight::ParameterStatus;
using passive_flight::ValidationIssues;

constexpr double kDegreesToRadians =
    std::numbers::pi_v<double> /
    180.0;

void require(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        throw std::runtime_error(
            message
        );
    }
}

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& name
) {
    if (std::abs(
            actual -
            expected
        ) > tolerance) {
        throw std::runtime_error(
            name +
            ": actual=" +
            std::to_string(actual) +
            ", expected=" +
            std::to_string(expected)
        );
    }
}

double numericValue(
    const ParameterRecord& record
) {
    const double* value =
        std::get_if<double>(
            &record.value
        );

    if (value == nullptr) {
        throw std::runtime_error(
            record.path +
            " does not contain a numeric value"
        );
    }

    return *value;
}

void requireValid(
    const ObjectPassport& passport,
    const std::string& name
) {
    const ValidationIssues issues =
        passive_flight::validate(
            passport
        );

    if (issues.empty()) {
        return;
    }

    std::string message =
        name +
        " validation failed:";

    for (const auto& issue :
         issues) {
        message +=
            "\n" +
            issue.field +
            ": " +
            issue.message;
    }

    throw std::runtime_error(
        message
    );
}

void testAbstractPassportIdentity() {
    const ObjectPassport passport =
        passive_flight::
            makeAbstract500UmpkPassport();

    require(
        passport.object.id ==
            "ABSTRACT_500_UMPK_V1",
        "Abstract object identifier is incorrect"
    );

    require(
        passport.object.metadata
            .modelVersion ==
            "0.2.0",
        "Abstract object model version is incorrect"
    );

    require(
        passport.object.body.noseShape ==
            NoseShape::Ogival,
        "Baseline nose shape must be ogival"
    );

    require(
        passport.object.aerodynamicModelType ==
            AerodynamicModelType::
                PreliminaryGeometryBased,
        "Abstract object aerodynamic model type is incorrect"
    );
}

void testAbstractMassAndGeometry() {
    const ObjectPassport passport =
        passive_flight::
            makeAbstract500UmpkPassport();

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

    requireNear(
        passport.object.mass
            .centerOfMassXM,
        1.15,
        0.0,
        "Center of mass"
    );

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
        passport.object.wing
            .installationAngleRad,
        3.0 *
            kDegreesToRadians,
        1.0e-12,
        "Wing installation angle"
    );

    requireNear(
        passport.object.tail.areaM2,
        0.269,
        0.0,
        "Tail area"
    );
}

void testAbstractProvenance() {
    const ObjectPassport passport =
        passive_flight::
            makeAbstract500UmpkPassport();

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
        mass->status ==
            ParameterStatus::Requirement,
        "Abstract mass status"
    );

    const ParameterRecord* modelType =
        passive_flight::findParameter(
            passport,
            "aerodynamics.modelType"
        );

    require(
        modelType != nullptr,
        "Aerodynamic model type provenance is missing"
    );
}

void testFab1500TPassport() {
    const ObjectPassport passport =
        passive_flight::
            makeFab1500TPostnikovPassport();

    require(
        passport.object.id ==
            "FAB_1500T_POSTNIKOV_1979",
        "FAB-1500T identifier is incorrect"
    );

    require(
        passport.object.aerodynamicModelType ==
            AerodynamicModelType::Tabulated,
        "FAB-1500T must use tabulated aerodynamics"
    );

    requireNear(
        passport.object.mass.massKg,
        1519.0,
        0.0,
        "FAB-1500T mass"
    );

    requireNear(
        passport.object.mass
            .pitchMomentOfInertiaKgM2,
        1122.3,
        0.0,
        "FAB-1500T pitch inertia"
    );

    requireNear(
        passport.object.mass.centerOfMassXM,
        1.16,
        0.0,
        "FAB-1500T center of mass"
    );

    requireNear(
        passport.object.body.lengthM,
        3.46,
        0.0,
        "FAB-1500T length"
    );

    requireNear(
        passport.object.body.diameterM,
        0.58,
        0.0,
        "FAB-1500T diameter"
    );

    const double expectedArea =
        std::numbers::pi_v<double> *
        0.58 *
        0.58 /
        4.0;

    requireNear(
        passport.object.reference.areaM2,
        expectedArea,
        1.0e-12,
        "FAB-1500T midsection area"
    );

    requireNear(
        passport.object.reference
            .effectiveReferenceLengthM(),
        3.46,
        0.0,
        "FAB-1500T reference length"
    );

    require(
        passport.object.wing.areaM2 == 0.0 &&
        passport.object.tail.areaM2 == 0.0,
        "FAB-1500T must not contain fictitious lifting surfaces"
    );

    require(
        passport.object.tabulatedAerodynamics
            .cx0.size() == 19,
        "FAB-1500T Cx0 node count"
    );

    require(
        passport.object.tabulatedAerodynamics
            .cyAlpha.size() == 7,
        "FAB-1500T CyAlpha node count"
    );

    const ParameterRecord* source =
        passive_flight::findParameter(
            passport,
            "aerodynamics.source"
        );

    require(
        source != nullptr,
        "FAB-1500T aerodynamic source record is missing"
    );

    require(
        source->status ==
            ParameterStatus::SourceDocument,
        "FAB-1500T aerodynamic source status"
    );
}

void testBothPassportsAreValid() {
    requireValid(
        passive_flight::
            makeAbstract500UmpkPassport(),
        "Abstract passport"
    );

    requireValid(
        passive_flight::
            makeFab1500TPostnikovPassport(),
        "FAB-1500T passport"
    );
}

void testTabulatedObjectDoesNotRequireWing() {
    ObjectPassport passport =
        passive_flight::
            makeFab1500TPostnikovPassport();

    /*
     * Эти значения уже нулевые. Проверка фиксирует
     * важное правило архитектуры: Tabulated-объект
     * не обязан иметь крыло и стабилизатор.
     */
    passport.object.wing = {};
    passport.object.tail = {};

    const ValidationIssues issues =
        passive_flight::validate(
            passport.object
        );

    require(
        issues.empty(),
        "Tabulated object must not require wing/tail geometry"
    );
}

void testEmptyTabulatedTableDetected() {
    ObjectPassport passport =
        passive_flight::
            makeFab1500TPostnikovPassport();

    passport.object
        .tabulatedAerodynamics
        .cyAlpha
        .clear();

    const ValidationIssues issues =
        passive_flight::validate(
            passport.object
        );

    require(
        !issues.empty(),
        "Empty aerodynamic table must be detected"
    );
}

void testInvalidMassDetected() {
    ObjectPassport passport =
        passive_flight::
            makeAbstract500UmpkPassport();

    passport.object.mass.massKg =
        -500.0;

    const ValidationIssues issues =
        passive_flight::validate(
            passport
        );

    require(
        !issues.empty(),
        "Negative mass must be detected"
    );
}

void testInvalidInstallationAngleDetected() {
    ObjectPassport passport =
        passive_flight::
            makeAbstract500UmpkPassport();

    passport.object.wing
        .installationAngleRad =
        std::numeric_limits<double>::
            quiet_NaN();

    const ValidationIssues issues =
        passive_flight::validate(
            passport
        );

    require(
        !issues.empty(),
        "Invalid wing installation angle must be detected"
    );
}

void testReferenceGeometryMismatchDetected() {
    ObjectPassport passport =
        passive_flight::
            makeAbstract500UmpkPassport();

    passport.object.reference.areaM2 =
        1.0;

    const ValidationIssues issues =
        passive_flight::validate(
            passport
        );

    require(
        !issues.empty(),
        "Reference area mismatch must be detected"
    );
}

void testDuplicateParameterDetected() {
    ObjectPassport passport =
        passive_flight::
            makeAbstract500UmpkPassport();

    require(
        !passport.parameters.empty(),
        "Passport parameter list must not be empty"
    );

    passport.parameters.push_back(
        passport.parameters.front()
    );

    const ValidationIssues issues =
        passive_flight::validate(
            passport
        );

    bool duplicateFound =
        false;

    for (const auto& issue :
         issues) {
        if (issue.message ==
            "Duplicate parameter provenance record") {
            duplicateFound =
                true;
            break;
        }
    }

    require(
        duplicateFound,
        "Duplicate parameter must be detected"
    );
}

void testNames() {
    require(
        std::string(
            passive_flight::
                parameterStatusName(
                    ParameterStatus::
                        SourceDocument
                )
        ) == "source_document",
        "Parameter status name is incorrect"
    );

    require(
        std::string(
            passive_flight::
                noseShapeName(
                    NoseShape::Unspecified
                )
        ) == "unspecified",
        "Unspecified nose-shape name is incorrect"
    );

    require(
        std::string(
            passive_flight::
                aerodynamicModelTypeName(
                    AerodynamicModelType::
                        Tabulated
                )
        ) == "tabulated",
        "Aerodynamic model type name is incorrect"
    );
}

} // namespace

int main() {
    try {
        testAbstractPassportIdentity();
        testAbstractMassAndGeometry();
        testAbstractProvenance();
        testFab1500TPassport();
        testBothPassportsAreValid();
        testTabulatedObjectDoesNotRequireWing();
        testEmptyTabulatedTableDetected();
        testInvalidMassDetected();
        testInvalidInstallationAngleDetected();
        testReferenceGeometryMismatchDetected();
        testDuplicateParameterDetected();
        testNames();

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
