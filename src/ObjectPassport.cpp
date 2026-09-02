#include "passive_flight/ObjectPassport.hpp"

#include "passive_flight/KnownObjects.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace passive_flight {
namespace {

constexpr double kComparisonTolerance =
    1.0e-12;

void requirePositive(
    ValidationIssues& issues,
    double value,
    const std::string& field
) {
    if (!std::isfinite(value) ||
        value <= 0.0) {
        issues.push_back({
            field,
            "Value must be finite and greater than zero"
        });
    }
}

void requireNonNegative(
    ValidationIssues& issues,
    double value,
    const std::string& field
) {
    if (!std::isfinite(value) ||
        value < 0.0) {
        issues.push_back({
            field,
            "Value must be finite and non-negative"
        });
    }
}

void requireFinite(
    ValidationIssues& issues,
    double value,
    const std::string& field
) {
    if (!std::isfinite(value)) {
        issues.push_back({
            field,
            "Value must be finite"
        });
    }
}

void requireRatio(
    ValidationIssues& issues,
    double value,
    const std::string& field
) {
    if (!std::isfinite(value) ||
        value <= 0.0 ||
        value > 1.0) {
        issues.push_back({
            field,
            "Ratio must be in the interval (0, 1]"
        });
    }
}

bool valuesEqual(
    double first,
    double second
) {
    return
        std::abs(first - second) <=
        kComparisonTolerance;
}

void addNumber(
    ObjectPassport& passport,
    std::string path,
    double value,
    std::string unit,
    ParameterStatus status,
    std::string source,
    std::string note
) {
    passport.parameters.push_back({
        std::move(path),
        value,
        std::move(unit),
        status,
        std::move(source),
        std::move(note)
    });
}

void addText(
    ObjectPassport& passport,
    std::string path,
    std::string value,
    ParameterStatus status,
    std::string source,
    std::string note
) {
    passport.parameters.push_back({
        std::move(path),
        std::move(value),
        "-",
        status,
        std::move(source),
        std::move(note)
    });
}

void checkNumericRecord(
    ValidationIssues& issues,
    const ObjectPassport& passport,
    const std::string& path,
    double expectedValue
) {
    const ParameterRecord* record =
        findParameter(
            passport,
            path
        );

    if (record == nullptr) {
        issues.push_back({
            path,
            "Parameter provenance record is missing"
        });

        return;
    }

    const double* recordedValue =
        std::get_if<double>(
            &record->value
        );

    if (recordedValue == nullptr) {
        issues.push_back({
            path,
            "Parameter provenance record must contain a number"
        });

        return;
    }

    if (!valuesEqual(
            *recordedValue,
            expectedValue
        )) {
        issues.push_back({
            path,
            "Parameter record does not match the object model"
        });
    }
}

void checkTextRecord(
    ValidationIssues& issues,
    const ObjectPassport& passport,
    const std::string& path,
    const std::string& expectedValue
) {
    const ParameterRecord* record =
        findParameter(
            passport,
            path
        );

    if (record == nullptr) {
        issues.push_back({
            path,
            "Parameter provenance record is missing"
        });

        return;
    }

    const std::string* recordedValue =
        std::get_if<std::string>(
            &record->value
        );

    if (recordedValue == nullptr) {
        issues.push_back({
            path,
            "Parameter provenance record must contain text"
        });

        return;
    }

    if (*recordedValue != expectedValue) {
        issues.push_back({
            path,
            "Parameter record does not match the object model"
        });
    }
}

void validateMachTable(
    ValidationIssues& issues,
    const std::vector<MachCoefficientPoint>& table,
    const std::string& field
) {
    if (table.empty()) {
        issues.push_back({
            field,
            "Aerodynamic table must not be empty"
        });

        return;
    }

    double previousMach = 0.0;
    bool first = true;

    for (const MachCoefficientPoint& point :
         table) {
        if (!std::isfinite(point.mach) ||
            point.mach < 0.0) {
            issues.push_back({
                field,
                "Mach values must be finite and non-negative"
            });

            return;
        }

        if (!std::isfinite(point.value)) {
            issues.push_back({
                field,
                "Coefficient values must be finite"
            });

            return;
        }

        if (!first &&
            point.mach <= previousMach) {
            issues.push_back({
                field,
                "Mach values must be strictly increasing"
            });

            return;
        }

        previousMach =
            point.mach;

        first =
            false;
    }
}

void validateCommonObjectFields(
    ValidationIssues& issues,
    const ObjectModel& object
) {
    if (object.id.empty()) {
        issues.push_back({
            "object.id",
            "Object identifier must not be empty"
        });
    }

    if (object.metadata.displayName.empty()) {
        issues.push_back({
            "object.metadata.displayName",
            "Display name must not be empty"
        });
    }

    if (object.metadata.modelVersion.empty()) {
        issues.push_back({
            "object.metadata.modelVersion",
            "Model version must not be empty"
        });
    }

    requirePositive(
        issues,
        object.mass.massKg,
        "mass.massKg"
    );

    requirePositive(
        issues,
        object.mass
            .pitchMomentOfInertiaKgM2,
        "mass.pitchMomentOfInertiaKgM2"
    );

    requireFinite(
        issues,
        object.mass.centerOfMassXM,
        "mass.centerOfMassXM"
    );

    requirePositive(
        issues,
        object.reference.areaM2,
        "reference.areaM2"
    );

    requirePositive(
        issues,
        object.reference
            .effectiveReferenceLengthM(),
        "reference.referenceLengthM"
    );

    requirePositive(
        issues,
        object.body.lengthM,
        "body.lengthM"
    );

    requirePositive(
        issues,
        object.body.diameterM,
        "body.diameterM"
    );

    if (std::isfinite(
            object.mass.centerOfMassXM
        ) &&
        std::isfinite(
            object.body.lengthM
        ) &&
        (
            object.mass.centerOfMassXM < 0.0 ||
            object.mass.centerOfMassXM >
                object.body.lengthM
        )) {
        issues.push_back({
            "mass.centerOfMassXM",
            "Center of mass must lie inside the body length"
        });
    }
}

void validatePreliminaryGeometryObject(
    ValidationIssues& issues,
    const ObjectModel& object
) {
    requirePositive(
        issues,
        object.reference.spanM,
        "reference.spanM"
    );

    requirePositive(
        issues,
        object.reference
            .meanAerodynamicChordM,
        "reference.meanAerodynamicChordM"
    );

    const auto validateSurface =
        [&issues](
            const auto& surface,
            const std::string& prefix
        ) {
            requirePositive(
                issues,
                surface.areaM2,
                prefix + ".areaM2"
            );

            requirePositive(
                issues,
                surface.spanM,
                prefix + ".spanM"
            );

            requirePositive(
                issues,
                surface
                    .meanAerodynamicChordM,
                prefix +
                    ".meanAerodynamicChordM"
            );

            requireFinite(
                issues,
                surface.sweepHalfChordRad,
                prefix + ".sweepHalfChordRad"
            );

            requireRatio(
                issues,
                surface.taperRatio,
                prefix + ".taperRatio"
            );

            requireRatio(
                issues,
                surface.relativeThickness,
                prefix + ".relativeThickness"
            );

            requireFinite(
                issues,
                surface.installationAngleRad,
                prefix + ".installationAngleRad"
            );

            requireRatio(
                issues,
                surface.efficiencyFactor,
                prefix + ".efficiencyFactor"
            );

            requireFinite(
                issues,
                surface.aerodynamicCenterXM,
                prefix + ".aerodynamicCenterXM"
            );
        };

    validateSurface(
        object.wing,
        "wing"
    );

    validateSurface(
        object.tail,
        "tail"
    );

    requirePositive(
        issues,
        object.body.noseLengthM,
        "body.noseLengthM"
    );

    requirePositive(
        issues,
        object.body.tailLengthM,
        "body.tailLengthM"
    );

    requirePositive(
        issues,
        object.body
            .zeroLiftDragCoefficientOnFrontalArea,
        "body.zeroLiftDragCoefficientOnFrontalArea"
    );

    if (object.body.noseShape ==
        NoseShape::Unspecified) {
        issues.push_back({
            "body.noseShape",
            "Preliminary geometry-based model requires a nose shape"
        });
    }

    if (object.wing.aerodynamicCenterXM < 0.0 ||
        object.wing.aerodynamicCenterXM >
            object.body.lengthM) {
        issues.push_back({
            "wing.aerodynamicCenterXM",
            "Wing aerodynamic center must lie inside the body length"
        });
    }

    if (object.tail.aerodynamicCenterXM < 0.0 ||
        object.tail.aerodynamicCenterXM >
            object.body.lengthM) {
        issues.push_back({
            "tail.aerodynamicCenterXM",
            "Tail aerodynamic center must lie inside the body length"
        });
    }

    if (object.body.noseLengthM +
            object.body.tailLengthM >=
        object.body.lengthM) {
        issues.push_back({
            "body.lengthM",
            "Nose and tail lengths must leave a positive cylindrical section"
        });
    }

    if (!valuesEqual(
            object.reference.areaM2,
            object.wing.areaM2
        )) {
        issues.push_back({
            "reference.areaM2",
            "Reference area must equal wing area"
        });
    }

    if (!valuesEqual(
            object.reference.spanM,
            object.wing.spanM
        )) {
        issues.push_back({
            "reference.spanM",
            "Reference span must equal wing span"
        });
    }

    if (!valuesEqual(
            object.reference
                .meanAerodynamicChordM,
            object.wing
                .meanAerodynamicChordM
        )) {
        issues.push_back({
            "reference.meanAerodynamicChordM",
            "Reference chord must equal wing mean aerodynamic chord"
        });
    }
}

void validateTabulatedObject(
    ValidationIssues& issues,
    const ObjectModel& object
) {
    /*
     * Для обычной бомбы наличие крыла и стабилизатора
     * не требуется. Нулевые значения этих структур
     * являются допустимыми.
     */
    requireNonNegative(
        issues,
        object.reference.spanM,
        "reference.spanM"
    );

    requireNonNegative(
        issues,
        object.reference
            .meanAerodynamicChordM,
        "reference.meanAerodynamicChordM"
    );

    validateMachTable(
        issues,
        object.tabulatedAerodynamics.cx0,
        "tabulatedAerodynamics.cx0"
    );

    validateMachTable(
        issues,
        object.tabulatedAerodynamics
            .cxAlphaSquared,
        "tabulatedAerodynamics.cxAlphaSquared"
    );

    validateMachTable(
        issues,
        object.tabulatedAerodynamics.cyAlpha,
        "tabulatedAerodynamics.cyAlpha"
    );

    validateMachTable(
        issues,
        object.tabulatedAerodynamics.mzAlpha,
        "tabulatedAerodynamics.mzAlpha"
    );

    validateMachTable(
        issues,
        object.tabulatedAerodynamics
            .mzPitchRate,
        "tabulatedAerodynamics.mzPitchRate"
    );

    /*
     * validateMachTable() уже добавил диагностическую
     * ошибку для каждой пустой таблицы.
     *
     * Ниже используются front()/back(), поэтому до
     * вычисления общего диапазона Mach обязательно
     * прекращаем эту ветвь проверки, если хотя бы
     * одна таблица пуста.
     */
    if (object.tabulatedAerodynamics.cx0.empty() ||
        object.tabulatedAerodynamics
            .cxAlphaSquared.empty() ||
        object.tabulatedAerodynamics
            .cyAlpha.empty() ||
        object.tabulatedAerodynamics
            .mzAlpha.empty() ||
        object.tabulatedAerodynamics
            .mzPitchRate.empty()) {
        return;
    }

    const double commonMinimumMach =
        std::max({
            object.tabulatedAerodynamics
                .cx0.front().mach,
            object.tabulatedAerodynamics
                .cxAlphaSquared.front().mach,
            object.tabulatedAerodynamics
                .cyAlpha.front().mach,
            object.tabulatedAerodynamics
                .mzAlpha.front().mach,
            object.tabulatedAerodynamics
                .mzPitchRate.front().mach
        });

    const double commonMaximumMach =
        std::min({
            object.tabulatedAerodynamics
                .cx0.back().mach,
            object.tabulatedAerodynamics
                .cxAlphaSquared.back().mach,
            object.tabulatedAerodynamics
                .cyAlpha.back().mach,
            object.tabulatedAerodynamics
                .mzAlpha.back().mach,
            object.tabulatedAerodynamics
                .mzPitchRate.back().mach
        });

    if (commonMaximumMach <=
        commonMinimumMach) {
        issues.push_back({
            "tabulatedAerodynamics",
            "Aerodynamic tables must have a common Mach interval"
        });
    }
}

void appendAbstract500Provenance(
    ObjectPassport& passport
) {
    const ObjectModel& object =
        passport.object;

    addNumber(
        passport,
        "mass.massKg",
        object.mass.massKg,
        "kg",
        ParameterStatus::Requirement,
        "task-requirement",
        "Масса абстрактного объекта задана постановкой задачи"
    );

    addNumber(
        passport,
        "mass.pitchMomentOfInertiaKgM2",
        object.mass
            .pitchMomentOfInertiaKgM2,
        "kg*m^2",
        ParameterStatus::Provisional,
        "provisional-mass-model",
        "Предварительная оценка момента инерции"
    );

    addNumber(
        passport,
        "mass.centerOfMassXM",
        object.mass.centerOfMassXM,
        "m",
        ParameterStatus::Provisional,
        "static-stability-adjustment",
        "Предварительное положение центра масс"
    );

    addNumber(
        passport,
        "reference.areaM2",
        object.reference.areaM2,
        "m^2",
        ParameterStatus::Derived,
        "model-convention",
        "Характерная площадь равна площади крыла"
    );

    addNumber(
        passport,
        "reference.spanM",
        object.reference.spanM,
        "m",
        ParameterStatus::Derived,
        "model-convention",
        "Характерный размах равен размаху крыла"
    );

    addNumber(
        passport,
        "reference.meanAerodynamicChordM",
        object.reference
            .meanAerodynamicChordM,
        "m",
        ParameterStatus::Derived,
        "model-convention",
        "Характерная хорда равна САХ крыла"
    );

    addNumber(
        passport,
        "wing.areaM2",
        object.wing.areaM2,
        "m^2",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        passport,
        "wing.spanM",
        object.wing.spanM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        passport,
        "wing.meanAerodynamicChordM",
        object.wing
            .meanAerodynamicChordM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        passport,
        "wing.sweepHalfChordRad",
        object.wing.sweepHalfChordRad,
        "rad",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Стреловидность по линии половин хорд"
    );

    addNumber(
        passport,
        "wing.taperRatio",
        object.wing.taperRatio,
        "1",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Сужение крыла"
    );

    addNumber(
        passport,
        "wing.relativeThickness",
        object.wing.relativeThickness,
        "1",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Относительная толщина профиля крыла"
    );

    addNumber(
        passport,
        "wing.installationAngleRad",
        object.wing.installationAngleRad,
        "rad",
        ParameterStatus::Provisional,
        "trim-assumption",
        "Предварительный угол установки крыла"
    );

    addNumber(
        passport,
        "wing.efficiencyFactor",
        object.wing.efficiencyFactor,
        "1",
        ParameterStatus::Provisional,
        "provisional-aerodynamics",
        "Предварительный коэффициент эффективности крыла"
    );

    addNumber(
        passport,
        "wing.aerodynamicCenterXM",
        object.wing.aerodynamicCenterXM,
        "m",
        ParameterStatus::Provisional,
        "provisional-layout",
        "Предварительная координата аэродинамического центра крыла"
    );

    addNumber(
        passport,
        "wing.aspectRatio",
        object.wing.aspectRatio(),
        "1",
        ParameterStatus::Derived,
        "derived-from-wing-geometry",
        "Вычислено по span^2/area"
    );

    addNumber(
        passport,
        "tail.areaM2",
        object.tail.areaM2,
        "m^2",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        passport,
        "tail.spanM",
        object.tail.spanM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        passport,
        "tail.meanAerodynamicChordM",
        object.tail
            .meanAerodynamicChordM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Геометрия стабилизатора"
    );

    addNumber(
        passport,
        "tail.sweepHalfChordRad",
        object.tail.sweepHalfChordRad,
        "rad",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Стреловидность стабилизатора"
    );

    addNumber(
        passport,
        "tail.taperRatio",
        object.tail.taperRatio,
        "1",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Сужение стабилизатора"
    );

    addNumber(
        passport,
        "tail.relativeThickness",
        object.tail.relativeThickness,
        "1",
        ParameterStatus::Provisional,
        "baseline-assumption",
        "Предварительная относительная толщина"
    );

    addNumber(
        passport,
        "tail.installationAngleRad",
        object.tail.installationAngleRad,
        "rad",
        ParameterStatus::Provisional,
        "baseline-assumption",
        "Угол установки стабилизатора"
    );

    addNumber(
        passport,
        "tail.efficiencyFactor",
        object.tail.efficiencyFactor,
        "1",
        ParameterStatus::Provisional,
        "provisional-aerodynamics",
        "Предварительный коэффициент эффективности стабилизатора"
    );

    addNumber(
        passport,
        "tail.aerodynamicCenterXM",
        object.tail.aerodynamicCenterXM,
        "m",
        ParameterStatus::Provisional,
        "provisional-layout",
        "Предварительная координата аэродинамического центра стабилизатора"
    );

    addNumber(
        passport,
        "tail.aspectRatio",
        object.tail.aspectRatio(),
        "1",
        ParameterStatus::Derived,
        "derived-from-tail-geometry",
        "Вычислено по span^2/area"
    );

    addNumber(
        passport,
        "body.lengthM",
        object.body.lengthM,
        "m",
        ParameterStatus::Provisional,
        "provisional-layout",
        "Условная длина корпуса"
    );

    addNumber(
        passport,
        "body.diameterM",
        object.body.diameterM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Диаметр корпуса"
    );

    addNumber(
        passport,
        "body.noseLengthM",
        object.body.noseLengthM,
        "m",
        ParameterStatus::Provisional,
        "lift-note-estimate",
        "Предварительная длина носовой части"
    );

    addNumber(
        passport,
        "body.tailLengthM",
        object.body.tailLengthM,
        "m",
        ParameterStatus::Provisional,
        "provisional-layout",
        "Предварительная длина кормовой части"
    );

    addText(
        passport,
        "body.noseShape",
        noseShapeName(
            object.body.noseShape
        ),
        ParameterStatus::SourceDocument,
        "lift-note",
        "Форма носовой части"
    );

    addNumber(
        passport,
        "body.zeroLiftDragCoefficientOnFrontalArea",
        object.body
            .zeroLiftDragCoefficientOnFrontalArea,
        "1",
        ParameterStatus::Provisional,
        "provisional-aerodynamics",
        "Временный коэффициент сопротивления корпуса"
    );

    addText(
        passport,
        "aerodynamics.modelType",
        aerodynamicModelTypeName(
            object.aerodynamicModelType
        ),
        ParameterStatus::Derived,
        "model-architecture",
        "Способ получения аэродинамических коэффициентов"
    );
}

void appendFab1500TProvenance(
    ObjectPassport& passport
) {
    const ObjectModel& object =
        passport.object;

    constexpr const char* source =
        "postnikov-chuyko-1979-table-1.1";

    addNumber(
        passport,
        "mass.massKg",
        object.mass.massKg,
        "kg",
        ParameterStatus::SourceDocument,
        source,
        "Масса ФАБ-1500Т"
    );

    addNumber(
        passport,
        "mass.pitchMomentOfInertiaKgM2",
        object.mass
            .pitchMomentOfInertiaKgM2,
        "kg*m^2",
        ParameterStatus::SourceDocument,
        source,
        "Поперечный момент инерции Jzz"
    );

    addNumber(
        passport,
        "mass.centerOfMassXM",
        object.mass.centerOfMassXM,
        "m",
        ParameterStatus::SourceDocument,
        source,
        "Координата центра масс от носа"
    );

    addNumber(
        passport,
        "body.lengthM",
        object.body.lengthM,
        "m",
        ParameterStatus::SourceDocument,
        source,
        "Полная длина ФАБ-1500Т"
    );

    addNumber(
        passport,
        "body.diameterM",
        object.body.diameterM,
        "m",
        ParameterStatus::SourceDocument,
        source,
        "Максимальный диаметр ФАБ-1500Т"
    );

    addNumber(
        passport,
        "reference.areaM2",
        object.reference.areaM2,
        "m^2",
        ParameterStatus::Derived,
        source,
        "Площадь миделя pi*D^2/4"
    );

    addNumber(
        passport,
        "reference.referenceLengthM",
        object.reference
            .effectiveReferenceLengthM(),
        "m",
        ParameterStatus::SourceDocument,
        source,
        "Характерная длина коэффициента момента равна длине бомбы"
    );

    addText(
        passport,
        "aerodynamics.modelType",
        aerodynamicModelTypeName(
            object.aerodynamicModelType
        ),
        ParameterStatus::Derived,
        "model-architecture",
        "Готовые аэродинамические характеристики из паспорта"
    );

    addText(
        passport,
        "aerodynamics.source",
        source,
        ParameterStatus::SourceDocument,
        source,
        "Источник всех пяти аэродинамических таблиц ФАБ-1500Т"
    );

    addNumber(
        passport,
        "aerodynamics.cx0.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .cx0.size()
        ),
        "1",
        ParameterStatus::Derived,
        source,
        "Количество узлов Cx0(M)"
    );

    addNumber(
        passport,
        "aerodynamics.cxAlphaSquared.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .cxAlphaSquared.size()
        ),
        "1",
        ParameterStatus::Derived,
        source,
        "Количество узлов Cx^(alpha^2)(M)"
    );

    addNumber(
        passport,
        "aerodynamics.cyAlpha.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .cyAlpha.size()
        ),
        "1",
        ParameterStatus::Derived,
        source,
        "Количество узлов Cy^alpha(M)"
    );

    addNumber(
        passport,
        "aerodynamics.mzAlpha.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .mzAlpha.size()
        ),
        "1",
        ParameterStatus::Derived,
        source,
        "Количество узлов mz^alpha(M)"
    );

    addNumber(
        passport,
        "aerodynamics.mzPitchRate.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .mzPitchRate.size()
        ),
        "1",
        ParameterStatus::Derived,
        source,
        "Количество узлов mz^(omegaBar)(M)"
    );
}

void validateParameterList(
    ValidationIssues& issues,
    const ObjectPassport& passport
) {
    std::unordered_set<std::string>
        uniquePaths;

    for (const ParameterRecord& record :
         passport.parameters) {
        if (record.path.empty()) {
            issues.push_back({
                "parameters",
                "Parameter path must not be empty"
            });
        }

        if (!uniquePaths.insert(
                record.path
            ).second) {
            issues.push_back({
                record.path,
                "Duplicate parameter provenance record"
            });
        }

        if (record.unit.empty()) {
            issues.push_back({
                record.path,
                "Parameter unit must not be empty"
            });
        }

        if (record.source.empty()) {
            issues.push_back({
                record.path,
                "Parameter source must not be empty"
            });
        }

        if (const double* number =
                std::get_if<double>(
                    &record.value
                );
            number != nullptr &&
            !std::isfinite(*number)) {
            issues.push_back({
                record.path,
                "Recorded parameter value must be finite"
            });
        }
    }
}

void validateAbstract500Provenance(
    ValidationIssues& issues,
    const ObjectPassport& passport
) {
    const ObjectModel& object =
        passport.object;

    const std::vector<
        std::pair<std::string, double>
    > expectedNumbers{
        {"mass.massKg", object.mass.massKg},
        {
            "mass.pitchMomentOfInertiaKgM2",
            object.mass
                .pitchMomentOfInertiaKgM2
        },
        {
            "mass.centerOfMassXM",
            object.mass.centerOfMassXM
        },
        {
            "reference.areaM2",
            object.reference.areaM2
        },
        {
            "reference.spanM",
            object.reference.spanM
        },
        {
            "reference.meanAerodynamicChordM",
            object.reference
                .meanAerodynamicChordM
        },
        {"wing.areaM2", object.wing.areaM2},
        {"wing.spanM", object.wing.spanM},
        {
            "wing.meanAerodynamicChordM",
            object.wing
                .meanAerodynamicChordM
        },
        {
            "wing.sweepHalfChordRad",
            object.wing.sweepHalfChordRad
        },
        {
            "wing.taperRatio",
            object.wing.taperRatio
        },
        {
            "wing.relativeThickness",
            object.wing.relativeThickness
        },
        {
            "wing.installationAngleRad",
            object.wing.installationAngleRad
        },
        {
            "wing.efficiencyFactor",
            object.wing.efficiencyFactor
        },
        {
            "wing.aerodynamicCenterXM",
            object.wing.aerodynamicCenterXM
        },
        {
            "wing.aspectRatio",
            object.wing.aspectRatio()
        },
        {"tail.areaM2", object.tail.areaM2},
        {"tail.spanM", object.tail.spanM},
        {
            "tail.meanAerodynamicChordM",
            object.tail
                .meanAerodynamicChordM
        },
        {
            "tail.sweepHalfChordRad",
            object.tail.sweepHalfChordRad
        },
        {
            "tail.taperRatio",
            object.tail.taperRatio
        },
        {
            "tail.relativeThickness",
            object.tail.relativeThickness
        },
        {
            "tail.installationAngleRad",
            object.tail.installationAngleRad
        },
        {
            "tail.efficiencyFactor",
            object.tail.efficiencyFactor
        },
        {
            "tail.aerodynamicCenterXM",
            object.tail.aerodynamicCenterXM
        },
        {
            "tail.aspectRatio",
            object.tail.aspectRatio()
        },
        {"body.lengthM", object.body.lengthM},
        {"body.diameterM", object.body.diameterM},
        {
            "body.noseLengthM",
            object.body.noseLengthM
        },
        {
            "body.tailLengthM",
            object.body.tailLengthM
        },
        {
            "body.zeroLiftDragCoefficientOnFrontalArea",
            object.body
                .zeroLiftDragCoefficientOnFrontalArea
        }
    };

    for (const auto& [
        path,
        expectedValue
    ] : expectedNumbers) {
        checkNumericRecord(
            issues,
            passport,
            path,
            expectedValue
        );
    }

    checkTextRecord(
        issues,
        passport,
        "body.noseShape",
        noseShapeName(
            object.body.noseShape
        )
    );

    checkTextRecord(
        issues,
        passport,
        "aerodynamics.modelType",
        aerodynamicModelTypeName(
            object.aerodynamicModelType
        )
    );
}

void validateFab1500TProvenance(
    ValidationIssues& issues,
    const ObjectPassport& passport
) {
    const ObjectModel& object =
        passport.object;

    checkNumericRecord(
        issues,
        passport,
        "mass.massKg",
        object.mass.massKg
    );

    checkNumericRecord(
        issues,
        passport,
        "mass.pitchMomentOfInertiaKgM2",
        object.mass
            .pitchMomentOfInertiaKgM2
    );

    checkNumericRecord(
        issues,
        passport,
        "mass.centerOfMassXM",
        object.mass.centerOfMassXM
    );

    checkNumericRecord(
        issues,
        passport,
        "body.lengthM",
        object.body.lengthM
    );

    checkNumericRecord(
        issues,
        passport,
        "body.diameterM",
        object.body.diameterM
    );

    checkNumericRecord(
        issues,
        passport,
        "reference.areaM2",
        object.reference.areaM2
    );

    checkNumericRecord(
        issues,
        passport,
        "reference.referenceLengthM",
        object.reference
            .effectiveReferenceLengthM()
    );

    checkTextRecord(
        issues,
        passport,
        "aerodynamics.modelType",
        aerodynamicModelTypeName(
            object.aerodynamicModelType
        )
    );

    checkTextRecord(
        issues,
        passport,
        "aerodynamics.source",
        "postnikov-chuyko-1979-table-1.1"
    );

    checkNumericRecord(
        issues,
        passport,
        "aerodynamics.cx0.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .cx0.size()
        )
    );

    checkNumericRecord(
        issues,
        passport,
        "aerodynamics.cxAlphaSquared.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .cxAlphaSquared.size()
        )
    );

    checkNumericRecord(
        issues,
        passport,
        "aerodynamics.cyAlpha.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .cyAlpha.size()
        )
    );

    checkNumericRecord(
        issues,
        passport,
        "aerodynamics.mzAlpha.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .mzAlpha.size()
        )
    );

    checkNumericRecord(
        issues,
        passport,
        "aerodynamics.mzPitchRate.pointCount",
        static_cast<double>(
            object.tabulatedAerodynamics
                .mzPitchRate.size()
        )
    );
}

} // namespace

ObjectPassport makeAbstract500UmpkPassport() {
    constexpr double degreesToRadians =
        std::numbers::pi_v<double> /
        180.0;

    ObjectPassport passport;
    ObjectModel& object =
        passport.object;

    object.id =
        "ABSTRACT_500_UMPK_V1";

    object.metadata.displayName =
        "Абстрактный крылатый пассивный объект массой 500 кг";

    object.metadata.modelVersion =
        "0.2.0";

    object.metadata.description =
        "Учебная параметрическая модель, "
        "не являющаяся паспортом реального изделия";

    object.mass.massKg =
        500.0;

    object.mass.pitchMomentOfInertiaKgM2 =
        250.0;

    object.mass.centerOfMassXM =
        1.15;

    object.reference.areaM2 =
        0.475;

    object.reference.spanM =
        1.760;

    object.reference.meanAerodynamicChordM =
        0.274;

    /*
     * Для сохранения прежнего поведения
     * referenceLengthM оставляем равной нулю.
     * effectiveReferenceLengthM() вернёт САХ.
     */
    object.reference.referenceLengthM =
        0.0;

    object.wing.areaM2 =
        0.475;

    object.wing.spanM =
        1.760;

    object.wing.meanAerodynamicChordM =
        0.274;

    object.wing.sweepHalfChordRad =
        40.0 *
        degreesToRadians;

    object.wing.taperRatio =
        1.0;

    object.wing.relativeThickness =
        0.084;

    object.wing.installationAngleRad =
        3.0 *
        degreesToRadians;

    object.wing.efficiencyFactor =
        0.80;

    object.wing.aerodynamicCenterXM =
        1.15;

    object.tail.areaM2 =
        0.269;

    object.tail.spanM =
        0.514;

    object.tail.meanAerodynamicChordM =
        0.534;

    object.tail.sweepHalfChordRad =
        26.3 *
        degreesToRadians;

    object.tail.taperRatio =
        0.64;

    object.tail.relativeThickness =
        0.04;

    object.tail.installationAngleRad =
        0.0;

    object.tail.efficiencyFactor =
        0.75;

    object.tail.aerodynamicCenterXM =
        2.05;

    object.body.lengthM =
        2.40;

    object.body.diameterM =
        0.40;

    object.body.noseLengthM =
        0.49;

    object.body.tailLengthM =
        0.45;

    object.body.noseShape =
        NoseShape::Ogival;

    object.body
        .zeroLiftDragCoefficientOnFrontalArea =
        0.25;

    object.aerodynamicModelType =
        AerodynamicModelType::
            PreliminaryGeometryBased;

    appendAbstract500Provenance(
        passport
    );

    return passport;
}

ObjectPassport makeFab1500TPostnikovPassport() {
    ObjectPassport passport;

    passport.object =
        makeFab1500TPostnikovModel();

    appendFab1500TProvenance(
        passport
    );

    return passport;
}

const ParameterRecord* findParameter(
    const ObjectPassport& passport,
    const std::string& path
) noexcept {
    const auto iterator =
        std::find_if(
            passport.parameters.begin(),
            passport.parameters.end(),
            [&path](
                const ParameterRecord& record
            ) {
                return
                    record.path ==
                    path;
            }
        );

    if (iterator ==
        passport.parameters.end()) {
        return nullptr;
    }

    return &(*iterator);
}

ValidationIssues validate(
    const ObjectModel& object
) {
    ValidationIssues issues;

    validateCommonObjectFields(
        issues,
        object
    );

    switch (object.aerodynamicModelType) {
        case AerodynamicModelType::
            PreliminaryGeometryBased:
            validatePreliminaryGeometryObject(
                issues,
                object
            );
            break;

        case AerodynamicModelType::Tabulated:
            validateTabulatedObject(
                issues,
                object
            );
            break;
    }

    return issues;
}

ValidationIssues validate(
    const ObjectPassport& passport
) {
    ValidationIssues issues =
        validate(
            passport.object
        );

    validateParameterList(
        issues,
        passport
    );

    switch (
        passport.object.aerodynamicModelType
    ) {
        case AerodynamicModelType::
            PreliminaryGeometryBased:
            validateAbstract500Provenance(
                issues,
                passport
            );
            break;

        case AerodynamicModelType::Tabulated:
            validateFab1500TProvenance(
                issues,
                passport
            );
            break;
    }

    return issues;
}

const char* parameterStatusName(
    ParameterStatus status
) noexcept {
    switch (status) {
        case ParameterStatus::Requirement:
            return "requirement";

        case ParameterStatus::SourceDocument:
            return "source_document";

        case ParameterStatus::Derived:
            return "derived";

        case ParameterStatus::Provisional:
            return "provisional";

        case ParameterStatus::AuxiliaryTable:
            return "auxiliary_table";
    }

    return "unknown";
}

const char* noseShapeName(
    NoseShape shape
) noexcept {
    switch (shape) {
        case NoseShape::Unspecified:
            return "unspecified";

        case NoseShape::Ogival:
            return "ogival";

        case NoseShape::Conical:
            return "conical";
    }

    return "unknown";
}

const char* aerodynamicModelTypeName(
    AerodynamicModelType type
) noexcept {
    switch (type) {
        case AerodynamicModelType::
            PreliminaryGeometryBased:
            return "preliminary_geometry_based";

        case AerodynamicModelType::Tabulated:
            return "tabulated";
    }

    return "unknown";
}

} // namespace passive_flight
