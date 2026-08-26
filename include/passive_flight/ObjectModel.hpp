#pragma once

#include <string>

namespace passive_flight {

/*
 * Форма носовой части.
 *
 * В первой модели используется оживальная форма.
 * Коническая форма заранее добавлена в перечисление,
 * поскольку в дипломе планируется сравнение разных носовых частей.
 */
enum class NoseShape {
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
 * Эти значения используются для вычисления аэродинамических
 * сил и моментов:
 *
 *     X  = q * S * Cx;
 *     Y  = q * S * Cy;
 *     Mz = q * S * cBar * mz.
 */
struct ReferenceGeometry {
    double areaM2{};
    double spanM{};
    double meanAerodynamicChordM{};
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
        return spanM * spanM / areaM2;
    }
};

/*
 * Геометрия корпуса.
 */
struct BodyGeometry {
    double lengthM{};
    double diameterM{};

    double noseLengthM{};
    double tailLengthM{};

    NoseShape noseShape{NoseShape::Ogival};

    /*
     * Временный коэффициент сопротивления корпуса,
     * отнесённый к площади миделя.
     *
     * В дальнейшем он будет заменён табличной
     * зависимостью от числа Маха и формы носа.
     */
    double zeroLiftDragCoefficientOnFrontalArea{};
};

/*
 * Числовая модель пассивного объекта.
 *
 * Здесь находятся только данные, необходимые решателю.
 * Информация о происхождении параметров хранится отдельно
 * в ObjectPassport.
 */
struct ObjectModel {
    std::string id;
    ObjectMetadata metadata;

    MassProperties mass;
    ReferenceGeometry reference;

    WingGeometry wing;
    TailGeometry tail;
    BodyGeometry body;
};

} // namespace passive_flight