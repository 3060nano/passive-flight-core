#include "passive_flight/Aerodynamics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace passive_flight {
namespace {

void requireFinite(
    double value,
    const char* parameterName
) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(
            std::string(parameterName) +
            " must be finite"
        );
    }
}

void requirePositive(
    double value,
    const char* parameterName
) {
    requireFinite(value, parameterName);

    if (value <= 0.0) {
        throw std::invalid_argument(
            std::string(parameterName) +
            " must be positive"
        );
    }
}

void requireNonNegative(
    double value,
    const char* parameterName
) {
    requireFinite(value, parameterName);

    if (value < 0.0) {
        throw std::invalid_argument(
            std::string(parameterName) +
            " must be non-negative"
        );
    }
}

void validateSurface(
    const LiftingSurfaceAerodynamics& surface,
    const char* surfaceName
) {
    requirePositive(
        surface.areaM2,
        surfaceName
    );

    requirePositive(
        surface.aspectRatio,
        surfaceName
    );

    requirePositive(
        surface.efficiencyFactor,
        surfaceName
    );

    requireFinite(
        surface.halfChordSweepRad,
        surfaceName
    );

    requireFinite(
        surface.installationAngleRad,
        surfaceName
    );

    requireFinite(
        surface.aerodynamicCenterXM,
        surfaceName
    );

    if (surface.efficiencyFactor > 1.0) {
        throw std::invalid_argument(
            std::string(surfaceName) +
            " efficiency factor must not exceed one"
        );
    }

    const double maximumSweep =
        std::numbers::pi_v<double> / 2.0;

    if (std::abs(surface.halfChordSweepRad) >=
        maximumSweep) {
        throw std::invalid_argument(
            std::string(surfaceName) +
            " sweep angle must be below 90 degrees"
        );
    }
}

void validateGeometry(
    const AerodynamicGeometry& geometry
) {
    requirePositive(
        geometry.referenceAreaM2,
        "Reference area"
    );

    requirePositive(
        geometry.referenceChordM,
        "Reference chord"
    );

    requirePositive(
        geometry.bodyDiameterM,
        "Body diameter"
    );

    requireFinite(
        geometry.centerOfMassXM,
        "Center of mass position"
    );

    requireFinite(
        geometry.bodyAerodynamicCenterXM,
        "Body aerodynamic center position"
    );

    requirePositive(
        geometry.noseNormalForceFactor,
        "Nose normal-force factor"
    );

    requireNonNegative(
        geometry.downwashGradient,
        "Downwash gradient"
    );

    if (geometry.downwashGradient >= 1.0) {
        throw std::invalid_argument(
            "Downwash gradient must be below one"
        );
    }

    requireNonNegative(
        geometry.alphaDotDampingRatio,
        "Alpha-dot damping ratio"
    );

    validateSurface(
        geometry.wing,
        "Wing"
    );

    validateSurface(
        geometry.tail,
        "Tail"
    );
}

void validateDragTable(
    const std::vector<DragCoefficientPoint>& table
) {
    if (table.size() < 2) {
        throw std::invalid_argument(
            "Zero-lift drag table must contain at least two points"
        );
    }

    for (std::size_t index = 0;
         index < table.size();
         ++index) {
        requireNonNegative(
            table[index].mach,
            "Drag-table Mach number"
        );

        requirePositive(
            table[index].zeroLiftDragCoefficient,
            "Zero-lift drag coefficient"
        );

        if (index > 0 &&
            table[index].mach <= table[index - 1].mach) {
            throw std::invalid_argument(
                "Drag-table Mach numbers must be strictly increasing"
            );
        }
    }
}

void validateInput(
    const AerodynamicInput& input
) {
    requireNonNegative(
        input.mach,
        "Mach number"
    );

    requireFinite(
        input.angleOfAttackRad,
        "Angle of attack"
    );

    requireFinite(
        input.pitchRateRadS,
        "Pitch rate"
    );

    requireFinite(
        input.angleOfAttackRateRadS,
        "Angle-of-attack rate"
    );

    requireNonNegative(
        input.speedMps,
        "Flight speed"
    );
}

/**
 * Линейная интерполяция таблицы cx0(M).
 *
 * За пределами таблицы используется ближайшее
 * граничное значение. Такой подход не допускает
 * неконтролируемой экстраполяции.
 */
double interpolateDragCoefficient(
    double mach,
    const std::vector<DragCoefficientPoint>& table
) {
    if (mach <= table.front().mach) {
        return table.front().zeroLiftDragCoefficient;
    }

    if (mach >= table.back().mach) {
        return table.back().zeroLiftDragCoefficient;
    }

    const auto upper = std::upper_bound(
        table.begin(),
        table.end(),
        mach,
        [](double value, const DragCoefficientPoint& point) {
            return value < point.mach;
        }
    );

    const auto lower = upper - 1;

    const double interval =
        upper->mach - lower->mach;

    const double interpolationParameter =
        (mach - lower->mach) / interval;

    return
        lower->zeroLiftDragCoefficient +
        interpolationParameter *
        (
            upper->zeroLiftDragCoefficient -
            lower->zeroLiftDragCoefficient
        );
}

/**
 * Приближённая производная коэффициента подъёмной силы
 * конечного крыла по углу атаки.
 *
 * Используется выражение типа:
 *
 * c_y^alpha =
 *     2*pi*lambda /
 *     [
 *         2 + sqrt(
 *             4 +
 *             lambda^2 * beta^2 / cos^2(chi_0.5)
 *         )
 *     ].
 *
 * Здесь:
 * lambda  — удлинение поверхности;
 * beta    — поправка на сжимаемость;
 * chi_0.5 — стреловидность по линии половин хорд.
 *
 * В трансзвуковой области beta ограничивается,
 * потому что простая линейная теория там имеет
 * математическую особенность.
 */
double liftingSurfaceSlopePerRad(
    const LiftingSurfaceAerodynamics& surface,
    double mach
) {
    const double machExpression =
        std::abs(1.0 - mach * mach);

    const double beta =
        std::sqrt(std::max(0.04, machExpression));

    const double sweepCosine =
        std::cos(surface.halfChordSweepRad);

    const double compressibilityTerm =
        surface.aspectRatio *
        beta /
        sweepCosine;

    const double denominator =
        2.0 +
        std::sqrt(
            4.0 +
            compressibilityTerm *
            compressibilityTerm
        );

    const double slope =
        2.0 *
        std::numbers::pi_v<double> *
        surface.aspectRatio /
        denominator;

    /*
     * Ограничение защищает первую расчётную модель
     * от неограниченного роста производной вблизи M = 1.
     *
     * После оцифровки графиков это ограничение будет
     * заменено табличной зависимостью.
     */
    return std::min(slope, 8.0);
}

/**
 * Производная нормальной силы корпуса.
 *
 * Для тела вращения первого приближения используется
 * зависимость от отношения площади миделя к
 * характерной площади объекта.
 */
double bodyNormalForceSlopePerRad(
    const AerodynamicGeometry& geometry,
    double mach
) {
    const double bodyMidsectionAreaM2 =
        std::numbers::pi_v<double> *
        geometry.bodyDiameterM *
        geometry.bodyDiameterM /
        4.0;

    const double compressibilityCorrection =
        1.0 /
        std::sqrt(1.0 + mach * mach);

    return
        2.0 *
        geometry.noseNormalForceFactor *
        bodyMidsectionAreaM2 /
        geometry.referenceAreaM2 *
        compressibilityCorrection;
}

/**
 * Вычисляет вклад силы в коэффициент момента.
 *
 * Положительное направление координаты X принято
 * от носовой части к хвостовой.
 *
 * Сила, приложенная позади центра масс, создаёт
 * отрицательный, то есть пикирующий момент.
 */
double forceContributionToMoment(
    double forceCoefficient,
    double aerodynamicCenterXM,
    const AerodynamicGeometry& geometry
) {
    const double dimensionlessArm =
        (
            geometry.centerOfMassXM -
            aerodynamicCenterXM
        ) /
        geometry.referenceChordM;

    return forceCoefficient * dimensionlessArm;
}

} // namespace

AerodynamicGeometry makeAbstract500AerodynamicGeometry() {
    AerodynamicGeometry geometry;

    /*
     * Характерная площадь принимается равной
     * площади крыла.
     */
    geometry.referenceAreaM2 = 0.475;
    geometry.referenceChordM = 0.274;

    geometry.bodyDiameterM = 0.400;
    geometry.centerOfMassXM = 1.200;

    /*
     * Положение аэродинамического центра корпуса
     * пока принято расчётно.
     */
    geometry.bodyAerodynamicCenterXM = 0.800;

    /*
     * Базовая носовая часть — оживальная.
     */
    geometry.noseNormalForceFactor = 1.0;

    geometry.wing.areaM2 = 0.475;
    geometry.wing.aspectRatio =
        1.760 * 1.760 / 0.475;
    geometry.wing.halfChordSweepRad =
        40.0 * std::numbers::pi_v<double> / 180.0;
    geometry.wing.efficiencyFactor = 0.80;
    geometry.wing.installationAngleRad = 0.0;
    geometry.wing.aerodynamicCenterXM = 1.150;

    geometry.tail.areaM2 = 0.269;
    geometry.tail.aspectRatio =
        0.514 * 0.514 / 0.269;
    geometry.tail.halfChordSweepRad =
        26.3 * std::numbers::pi_v<double> / 180.0;
    geometry.tail.efficiencyFactor = 0.75;
    geometry.tail.installationAngleRad = 0.0;
    geometry.tail.aerodynamicCenterXM = 2.050;

    /*
     * Скос потока в районе стабилизатора
     * пока принят расчётно.
     */
    geometry.downwashGradient = 0.25;

    /*
     * Первая приближённая оценка влияния alpha_dot.
     * Коэффициент обязательно должен быть уточнён
     * по учебнику или данным преподавателя.
     */
    geometry.alphaDotDampingRatio = 0.35;

    return geometry;
}

std::vector<DragCoefficientPoint>
makeAbstract500ZeroLiftDragTable() {
    /*
     * Таблица извлечена из переданной выгрузки
     * модели SimInTech:
     *
     * my_diagramv14_out_0 — число Маха;
     * my_diagramv14_out_1 — коэффициент сопротивления.
     *
     * По приложенной записке коэффициенты уже
     * увеличены на 5 % для учёта крыла и других
     * добавленных элементов.
     */
    return {
        {0.40, 0.190},
        {0.55, 0.190},
        {0.65, 0.190},
        {0.75, 0.200},
        {0.80, 0.210},
        {0.85, 0.232},
        {0.90, 0.280},
        {1.00, 0.446},
        {1.05, 0.523},
        {1.10, 0.561},
        {1.15, 0.571},
        {1.25, 0.567},
        {1.35, 0.546},
        {1.50, 0.527},
        {1.75, 0.504},
        {2.00, 0.480},
        {2.50, 0.445},
        {3.00, 0.420},
        {3.50, 0.410}
    };
}

PreliminaryAerodynamicModel::PreliminaryAerodynamicModel()
    : PreliminaryAerodynamicModel(
          makeAbstract500AerodynamicGeometry(),
          makeAbstract500ZeroLiftDragTable()
      ) {
}

PreliminaryAerodynamicModel::PreliminaryAerodynamicModel(
    const AerodynamicGeometry& geometry,
    std::vector<DragCoefficientPoint> zeroLiftDragTable
)
    : geometry_(geometry),
      zeroLiftDragTable_(std::move(zeroLiftDragTable)) {
    validateGeometry(geometry_);
    validateDragTable(zeroLiftDragTable_);
}

AerodynamicCoefficients
PreliminaryAerodynamicModel::evaluate(
    const AerodynamicInput& input
) const {
    validateInput(input);

    AerodynamicCoefficients result;

    result.cx0 =
        interpolateDragCoefficient(
            input.mach,
            zeroLiftDragTable_
        );

    const double bodySlopePerRad =
        bodyNormalForceSlopePerRad(
            geometry_,
            input.mach
        );

    const double wingLocalSlopePerRad =
        liftingSurfaceSlopePerRad(
            geometry_.wing,
            input.mach
        );

    const double tailLocalSlopePerRad =
        liftingSurfaceSlopePerRad(
            geometry_.tail,
            input.mach
        );

    const double wingAreaRatio =
        geometry_.wing.areaM2 /
        geometry_.referenceAreaM2;

    const double tailAreaRatio =
        geometry_.tail.areaM2 /
        geometry_.referenceAreaM2;

    const double wingEffectiveAngleRad =
        input.angleOfAttackRad +
        geometry_.wing.installationAngleRad;

    /*
     * Угол атаки стабилизатора уменьшается
     * вследствие скоса потока за крылом.
     */
    const double tailEffectiveAngleRad =
        (
            1.0 - geometry_.downwashGradient
        ) *
        input.angleOfAttackRad +
        geometry_.tail.installationAngleRad;

    const double wingLocalCy =
        wingLocalSlopePerRad *
        wingEffectiveAngleRad;

    const double tailLocalCy =
        tailLocalSlopePerRad *
        tailEffectiveAngleRad;

    result.cyBody =
        bodySlopePerRad *
        input.angleOfAttackRad;

    result.cyWing =
        geometry_.wing.efficiencyFactor *
        wingAreaRatio *
        wingLocalCy;

    result.cyTail =
        geometry_.tail.efficiencyFactor *
        tailAreaRatio *
        tailLocalCy;

    result.cy =
        result.cyBody +
        result.cyWing +
        result.cyTail;

    /*
     * Индуктивное сопротивление:
     *
     * c_xi = c_y^2 / (pi * e * lambda).
     *
     * Для крыла и стабилизатора оно рассчитывается
     * отдельно, а затем приводится к характерной площади.
     */
    const double wingInducedDrag =
        geometry_.wing.efficiencyFactor *
        wingAreaRatio *
        wingLocalCy *
        wingLocalCy /
        (
            std::numbers::pi_v<double> *
            geometry_.wing.efficiencyFactor *
            geometry_.wing.aspectRatio
        );

    const double tailInducedDrag =
        geometry_.tail.efficiencyFactor *
        tailAreaRatio *
        tailLocalCy *
        tailLocalCy /
        (
            std::numbers::pi_v<double> *
            geometry_.tail.efficiencyFactor *
            geometry_.tail.aspectRatio
        );

    result.cxInduced =
        wingInducedDrag +
        tailInducedDrag;

    result.cx =
        result.cx0 +
        result.cxInduced;

    result.mzStatic =
        forceContributionToMoment(
            result.cyBody,
            geometry_.bodyAerodynamicCenterXM,
            geometry_
        ) +
        forceContributionToMoment(
            result.cyWing,
            geometry_.wing.aerodynamicCenterXM,
            geometry_
        ) +
        forceContributionToMoment(
            result.cyTail,
            geometry_.tail.aerodynamicCenterXM,
            geometry_
        );

    /*
     * Плечо стабилизатора относительно центра масс.
     */
    const double tailArmM =
        geometry_.tail.aerodynamicCenterXM -
        geometry_.centerOfMassXM;

    const double dimensionlessTailArm =
        tailArmM /
        geometry_.referenceChordM;

    /*
     * Производная демпфирующего момента:
     *
     * m_z^omega =
     * -2 * c_y_tail^alpha * eta_tail *
     * S_tail/S * (L_tail/b_a)^2.
     */
    result.mzPitchRateDerivative =
        -2.0 *
        tailLocalSlopePerRad *
        geometry_.tail.efficiencyFactor *
        tailAreaRatio *
        dimensionlessTailArm *
        dimensionlessTailArm;

    /*
     * Первая версия m_z по alpha_dot.
     *
     * Она задаётся как доля производной
     * демпфирующего момента.
     */
    result.mzAlphaDotDerivative =
        geometry_.alphaDotDampingRatio *
        result.mzPitchRateDerivative;

    double normalizedPitchRate = 0.0;
    double normalizedAlphaDot = 0.0;

    if (input.speedMps > 1.0e-9) {
        normalizedPitchRate =
            input.pitchRateRadS *
            geometry_.referenceChordM /
            (2.0 * input.speedMps);

        normalizedAlphaDot =
            input.angleOfAttackRateRadS *
            geometry_.referenceChordM /
            (2.0 * input.speedMps);
    }

    result.mzPitchDamping =
        result.mzPitchRateDerivative *
        normalizedPitchRate;

    result.mzAlphaDot =
        result.mzAlphaDotDerivative *
        normalizedAlphaDot;

    result.mz =
        result.mzStatic +
        result.mzPitchDamping +
        result.mzAlphaDot;

    /*
     * Аналитическая производная cy по alpha.
     */
    result.cyAlphaPerRad =
        bodySlopePerRad +
        geometry_.wing.efficiencyFactor *
        wingAreaRatio *
        wingLocalSlopePerRad +
        geometry_.tail.efficiencyFactor *
        tailAreaRatio *
        tailLocalSlopePerRad *
        (
            1.0 - geometry_.downwashGradient
        );

    /*
     * Аналитическая производная статического mz
     * по alpha.
     */
    result.mzAlphaPerRad =
        forceContributionToMoment(
            bodySlopePerRad,
            geometry_.bodyAerodynamicCenterXM,
            geometry_
        ) +
        forceContributionToMoment(
            geometry_.wing.efficiencyFactor *
                wingAreaRatio *
                wingLocalSlopePerRad,
            geometry_.wing.aerodynamicCenterXM,
            geometry_
        ) +
        forceContributionToMoment(
            geometry_.tail.efficiencyFactor *
                tailAreaRatio *
                tailLocalSlopePerRad *
                (
                    1.0 - geometry_.downwashGradient
                ),
            geometry_.tail.aerodynamicCenterXM,
            geometry_
        );

    return result;
}

const AerodynamicGeometry&
PreliminaryAerodynamicModel::geometry() const noexcept {
    return geometry_;
}

const std::vector<DragCoefficientPoint>&
PreliminaryAerodynamicModel::zeroLiftDragTable() const noexcept {
    return zeroLiftDragTable_;
}

} // namespace passive_flight