#include "passive_flight/ObjectPassport.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace passive_flight {
namespace {

constexpr double kComparisonTolerance = 1.0e-12;

void requirePositive(
    ValidationIssues& issues,
    double value,
    const std::string& field
) {
    if (!std::isfinite(value) || value <= 0.0) {
        issues.push_back({
            field,
            "Value must be finite and greater than zero"
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

void checkNumericRecord(
    ValidationIssues& issues,
    const ObjectPassport& passport,
    const std::string& path,
    double expectedValue
) {
    const ParameterRecord* record =
        findParameter(passport, path);

    if (record == nullptr) {
        issues.push_back({
            path,
            "Parameter provenance record is missing"
        });

        return;
    }

    const double* recordedValue =
        std::get_if<double>(&record->value);

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
        findParameter(passport, path);

    if (record == nullptr) {
        issues.push_back({
            path,
            "Parameter provenance record is missing"
        });

        return;
    }

    const std::string* recordedValue =
        std::get_if<std::string>(&record->value);

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

} // namespace

ObjectPassport makeAbstract500UmpkPassport() {
    constexpr double degreesToRadians =
        std::numbers::pi_v<double> / 180.0;

    ObjectPassport passport;
    ObjectModel& object = passport.object;

    object.id =
        "ABSTRACT_500_UMPK_V1";

    object.metadata.displayName =
        "Абстрактный крылатый пассивный объект массой 500 кг";

    object.metadata.modelVersion =
        "0.2.0";

    object.metadata.description =
        "Учебная параметрическая модель, "
        "не являющаяся паспортом реального изделия";

    /*
     * Массово-инерционные характеристики.
     */
    object.mass.massKg =
        500.0;

    object.mass.pitchMomentOfInertiaKgM2 =
        250.0;

    /*
     * Центр масс перемещён вперёд с 1,20 до 1,17 м.
     *
     * Это обеспечивает предварительный запас
     * статической устойчивости относительно
     * расчётного аэродинамического фокуса.
     */
    object.mass.centerOfMassXM =
        1.17;

    /*
     * Характерная геометрия.
     *
     * Для объекта с несущим крылом характерной
     * площадью принимается площадь крыла.
     */
    object.reference.areaM2 =
        0.475;

    object.reference.spanM =
        1.760;

    object.reference.meanAerodynamicChordM =
        0.274;

    /*
     * Крыло.
     */
    object.wing.areaM2 =
        0.475;

    object.wing.spanM =
        1.760;

    object.wing.meanAerodynamicChordM =
        0.274;

    object.wing.sweepHalfChordRad =
        40.0 * degreesToRadians;

    object.wing.taperRatio =
        1.0;

    object.wing.relativeThickness =
        0.084;

    /*
     * Предварительный положительный угол установки крыла.
     *
     * При начальном угле атаки объекта alpha = 0
     * эффективный угол атаки крыла будет равен 4 градусам.
     */
    object.wing.installationAngleRad =
        4.0 * degreesToRadians;

    object.wing.efficiencyFactor =
        0.80;

    object.wing.aerodynamicCenterXM =
        1.15;

    /*
     * Горизонтальный стабилизатор.
     */
    object.tail.areaM2 =
        0.269;

    object.tail.spanM =
        0.514;

    object.tail.meanAerodynamicChordM =
        0.534;

    object.tail.sweepHalfChordRad =
        26.3 * degreesToRadians;

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

    /*
     * Корпус.
     */
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

    object.body.zeroLiftDragCoefficientOnFrontalArea =
        0.25;

    auto addNumber =
        [&passport](
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
        };

    auto addText =
        [&passport](
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
        };

    addNumber(
        "mass.massKg",
        object.mass.massKg,
        "kg",
        ParameterStatus::Requirement,
        "task-requirement",
        "Масса абстрактного объекта задана постановкой задачи"
    );

    addNumber(
        "mass.pitchMomentOfInertiaKgM2",
        object.mass.pitchMomentOfInertiaKgM2,
        "kg*m^2",
        ParameterStatus::Provisional,
        "provisional-mass-model",
        "Предварительная оценка для вытянутого тела массой 500 кг"
    );

    addNumber(
        "mass.centerOfMassXM",
        object.mass.centerOfMassXM,
        "m",
        ParameterStatus::Provisional,
        "static-stability-adjustment",
        "Центр масс предварительно установлен на 1,17 м "
        "для получения положительного запаса статической устойчивости"
    );

    addNumber(
        "reference.areaM2",
        object.reference.areaM2,
        "m^2",
        ParameterStatus::Derived,
        "model-convention",
        "Характерная площадь принята равной площади крыла"
    );

    addNumber(
        "reference.spanM",
        object.reference.spanM,
        "m",
        ParameterStatus::Derived,
        "model-convention",
        "Характерный размах принят равным размаху крыла"
    );

    addNumber(
        "reference.meanAerodynamicChordM",
        object.reference.meanAerodynamicChordM,
        "m",
        ParameterStatus::Derived,
        "model-convention",
        "Характерная длина принята равной САХ крыла"
    );

    addNumber(
        "wing.areaM2",
        object.wing.areaM2,
        "m^2",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        "wing.spanM",
        object.wing.spanM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        "wing.meanAerodynamicChordM",
        object.wing.meanAerodynamicChordM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        "wing.sweepHalfChordRad",
        object.wing.sweepHalfChordRad,
        "rad",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Стреловидность по линии половин хорд около 40 градусов"
    );

    addNumber(
        "wing.taperRatio",
        object.wing.taperRatio,
        "1",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Передняя и задняя кромки крыла параллельны"
    );

    addNumber(
        "wing.relativeThickness",
        object.wing.relativeThickness,
        "1",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Относительная толщина профиля крыла"
    );

    addNumber(
        "wing.installationAngleRad",
        object.wing.installationAngleRad,
        "rad",
        ParameterStatus::Provisional,
        "trim-assumption",
        "Предварительный угол установки крыла принят равным +4 градусам"
    );

    addNumber(
        "wing.efficiencyFactor",
        object.wing.efficiencyFactor,
        "1",
        ParameterStatus::Provisional,
        "provisional-aerodynamics",
        "Предварительный коэффициент эффективности крыла"
    );

    addNumber(
        "wing.aerodynamicCenterXM",
        object.wing.aerodynamicCenterXM,
        "m",
        ParameterStatus::Provisional,
        "provisional-layout",
        "Предварительная координата аэродинамического центра от носа"
    );

    addNumber(
        "wing.aspectRatio",
        object.wing.aspectRatio(),
        "1",
        ParameterStatus::Derived,
        "derived-from-wing-geometry",
        "Вычислено по формуле span^2/area"
    );

    addNumber(
        "tail.areaM2",
        object.tail.areaM2,
        "m^2",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        "tail.spanM",
        object.tail.spanM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Взято из файла Подъемная сила в записку"
    );

    addNumber(
        "tail.meanAerodynamicChordM",
        object.tail.meanAerodynamicChordM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Предварительно прочитано по геометрическому "
        "построению стабилизатора"
    );

    addNumber(
        "tail.sweepHalfChordRad",
        object.tail.sweepHalfChordRad,
        "rad",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Стреловидность по линии половин хорд около 26,3 градуса"
    );

    addNumber(
        "tail.taperRatio",
        object.tail.taperRatio,
        "1",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Сужение стабилизатора около 0,64"
    );

    addNumber(
        "tail.relativeThickness",
        object.tail.relativeThickness,
        "1",
        ParameterStatus::Provisional,
        "baseline-assumption",
        "Временно принято 0,04 вместо сомнительного значения 0,004"
    );

    addNumber(
        "tail.installationAngleRad",
        object.tail.installationAngleRad,
        "rad",
        ParameterStatus::Provisional,
        "baseline-assumption",
        "Угол установки стабилизатора принят равным нулю"
    );

    addNumber(
        "tail.efficiencyFactor",
        object.tail.efficiencyFactor,
        "1",
        ParameterStatus::Provisional,
        "provisional-aerodynamics",
        "Предварительный коэффициент эффективности стабилизатора"
    );

    addNumber(
        "tail.aerodynamicCenterXM",
        object.tail.aerodynamicCenterXM,
        "m",
        ParameterStatus::Provisional,
        "provisional-layout",
        "Предварительная координата аэродинамического центра от носа"
    );

    addNumber(
        "tail.aspectRatio",
        object.tail.aspectRatio(),
        "1",
        ParameterStatus::Derived,
        "derived-from-tail-geometry",
        "Вычислено по формуле span^2/area"
    );

    addNumber(
        "body.lengthM",
        object.body.lengthM,
        "m",
        ParameterStatus::Provisional,
        "provisional-layout",
        "Условная длина корпуса первого объекта"
    );

    addNumber(
        "body.diameterM",
        object.body.diameterM,
        "m",
        ParameterStatus::SourceDocument,
        "lift-note",
        "Диаметр корпуса около 400 мм"
    );

    addNumber(
        "body.noseLengthM",
        object.body.noseLengthM,
        "m",
        ParameterStatus::Provisional,
        "lift-note-estimate",
        "Предварительная оценка по расчётной записке"
    );

    addNumber(
        "body.tailLengthM",
        object.body.tailLengthM,
        "m",
        ParameterStatus::Provisional,
        "provisional-layout",
        "Условная длина суживающейся кормовой части"
    );

    addText(
        "body.noseShape",
        noseShapeName(object.body.noseShape),
        ParameterStatus::SourceDocument,
        "lift-note",
        "В базовой конфигурации используется оживальная носовая часть"
    );

    addNumber(
        "body.zeroLiftDragCoefficientOnFrontalArea",
        object.body.zeroLiftDragCoefficientOnFrontalArea,
        "1",
        ParameterStatus::Provisional,
        "provisional-aerodynamics",
        "Временный коэффициент до подключения таблицы Cx0(M)"
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
            [&path](const ParameterRecord& record) {
                return record.path == path;
            }
        );

    if (iterator == passport.parameters.end()) {
        return nullptr;
    }

    return &(*iterator);
}

ValidationIssues validate(
    const ObjectModel& object
) {
    ValidationIssues issues;

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
        object.mass.pitchMomentOfInertiaKgM2,
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
        object.reference.spanM,
        "reference.spanM"
    );

    requirePositive(
        issues,
        object.reference.meanAerodynamicChordM,
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
                surface.meanAerodynamicChordM,
                prefix + ".meanAerodynamicChordM"
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
        object.body.lengthM,
        "body.lengthM"
    );

    requirePositive(
        issues,
        object.body.diameterM,
        "body.diameterM"
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
        object.body.zeroLiftDragCoefficientOnFrontalArea,
        "body.zeroLiftDragCoefficientOnFrontalArea"
    );

    if (object.mass.centerOfMassXM < 0.0 ||
        object.mass.centerOfMassXM >
            object.body.lengthM) {
        issues.push_back({
            "mass.centerOfMassXM",
            "Center of mass must lie inside the body length"
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
            "Nose and tail lengths must leave "
            "a positive cylindrical section"
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
            object.reference.meanAerodynamicChordM,
            object.wing.meanAerodynamicChordM
        )) {
        issues.push_back({
            "reference.meanAerodynamicChordM",
            "Reference chord must equal wing mean aerodynamic chord"
        });
    }

    return issues;
}

ValidationIssues validate(
    const ObjectPassport& passport
) {
    ValidationIssues issues =
        validate(passport.object);

    std::unordered_set<std::string> uniquePaths;

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

    const ObjectModel& object =
        passport.object;

    const std::vector<
        std::pair<std::string, double>
    > expectedNumbers{
        {
            "mass.massKg",
            object.mass.massKg
        },
        {
            "mass.pitchMomentOfInertiaKgM2",
            object.mass.pitchMomentOfInertiaKgM2
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
            object.reference.meanAerodynamicChordM
        },
        {
            "wing.areaM2",
            object.wing.areaM2
        },
        {
            "wing.spanM",
            object.wing.spanM
        },
        {
            "wing.meanAerodynamicChordM",
            object.wing.meanAerodynamicChordM
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
        {
            "tail.areaM2",
            object.tail.areaM2
        },
        {
            "tail.spanM",
            object.tail.spanM
        },
        {
            "tail.meanAerodynamicChordM",
            object.tail.meanAerodynamicChordM
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
        {
            "body.lengthM",
            object.body.lengthM
        },
        {
            "body.diameterM",
            object.body.diameterM
        },
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
            object.body.zeroLiftDragCoefficientOnFrontalArea
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
        case NoseShape::Ogival:
            return "ogival";

        case NoseShape::Conical:
            return "conical";
    }

    return "unknown";
}

} // namespace passive_flight