#pragma once

#include "passive_flight/TabulatedAerodynamicData.hpp"

#include <string>

namespace passive_flight {

/*
 * Способ получения аэродинамических коэффициентов объекта.
 *
 * PreliminaryGeometryBased — текущая предварительная модель
 * абстрактного крылатого объекта.
 *
 * Tabulated — готовые аэродинамические характеристики
 * объекта заданы в паспорте в виде таблиц от числа Маха.
 */
enum class AerodynamicModelType {
    PreliminaryGeometryBased,
    Tabulated
};

/*
 * Форма носовой части.
 */
enum class NoseShape {
    Unspecified,
    Ogival,
    Conical
};

/*
 * Общие сведения об объекте.
 */
struct ObjectMetadata {
    std::string displayName;
    std::string modelVersion;
    std::string description;
};

/*
 * Массово-инерционные характеристики.
 */
struct MassProperties {
    double massKg{};
    double pitchMomentOfInertiaKgM2{};
    double centerOfMassXM{};
};

/*
 * Характерная геометрия.
 *
 * areaM2 используется для вычисления сил и моментов:
 *
 *     X  = q * S_ref * Cx;
 *     Y  = q * S_ref * Cy;
 *
 * referenceLengthM используется для размерного момента:
 *
 *     Mz = q * S_ref * L_ref * mz.
 *
 * Для крылатого объекта L_ref обычно равна САХ крыла.
 * Для осесимметричной бомбы L_ref может быть равна
 * полной длине корпуса.
 *
 * Поля spanM и meanAerodynamicChordM пока сохранены
 * для существующей геометрической аэродинамики.
 */
struct ReferenceGeometry {
    double areaM2{};
    double spanM{};
    double meanAerodynamicChordM{};
    double referenceLengthM{};

    [[nodiscard]]
    double effectiveReferenceLengthM() const noexcept {
        return
            referenceLengthM > 0.0
                ? referenceLengthM
                : meanAerodynamicChordM;
    }
};

/*
 * Геометрия крыла.
 */
struct WingGeometry {
    double areaM2{};
    double spanM{};
    double meanAerodynamicChordM{};

    double sweepHalfChordRad{};
    double taperRatio{};
    double relativeThickness{};
    double installationAngleRad{};

    double efficiencyFactor{};
    double aerodynamicCenterXM{};

    [[nodiscard]] double aspectRatio() const noexcept {
        if (areaM2 <= 0.0) {
            return 0.0;
        }

        return spanM * spanM / areaM2;
    }
};

/*
 * Геометрия горизонтального стабилизатора.
 */
struct TailGeometry {
    double areaM2{};
    double spanM{};
    double meanAerodynamicChordM{};

    double sweepHalfChordRad{};
    double taperRatio{};
    double relativeThickness{};
    double installationAngleRad{};

    double efficiencyFactor{};
    double aerodynamicCenterXM{};

    [[nodiscard]] double aspectRatio() const noexcept {
        if (areaM2 <= 0.0) {
            return 0.0;
        }

        return spanM * spanM / areaM2;
    }
};

/*
 * Геометрия корпуса.
 *
 * Для табличной модели не все геометрические поля
 * обязаны участвовать в расчёте аэродинамики.
 */
struct BodyGeometry {
    double lengthM{};
    double diameterM{};

    double noseLengthM{};
    double tailLengthM{};

    NoseShape noseShape{NoseShape::Unspecified};

    /*
     * Поле используется только текущей
     * PreliminaryGeometryBased-моделью.
     */
    double zeroLiftDragCoefficientOnFrontalArea{};
};

/*
 * Числовая модель пассивного объекта.
 *
 * Для варианта с готовыми аэродинамическими
 * характеристиками tabulatedAerodynamics является
 * частью полного паспорта объекта.
 */
struct ObjectModel {
    std::string id;
    ObjectMetadata metadata;

    MassProperties mass;
    ReferenceGeometry reference;

    WingGeometry wing;
    TailGeometry tail;
    BodyGeometry body;

    AerodynamicModelType aerodynamicModelType{
        AerodynamicModelType::PreliminaryGeometryBased
    };

    TabulatedAerodynamicData tabulatedAerodynamics;
};

} // namespace passive_flight
