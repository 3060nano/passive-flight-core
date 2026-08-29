#include "passive_flight/Aerodynamics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
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
    requireFinite(
        value,
        parameterName
    );

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
    requireFinite(
        value,
        parameterName
    );

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
        std::numbers::pi_v<double> /
        2.0;

    if (
        std::abs(
            surface.halfChordSweepRad
        ) >= maximumSweep
    ) {
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

    for (
        std::size_t index = 0;
        index < table.size();
        ++index
    ) {
        requireNonNegative(
            table[index].mach,
            "Drag-table Mach number"
        );

        requirePositive(
            table[index].zeroLiftDragCoefficient,
            "Zero-lift drag coefficient"
        );

        if (
            index > 0 &&
            table[index].mach <=
                table[index - 1].mach
        ) {
            throw std::invalid_argument(
                "Drag-table Mach numbers must be strictly increasing"
            );
        }
    }
}

void validateDownwashGradientTable(
    const std::vector<DownwashGradientPoint>& table
) {
    /*
     * Пустая таблица допустима: в этом случае
     * используется geometry.downwashGradient.
     */
    if (table.empty()) {
        return;
    }

    if (table.size() < 2) {
        throw std::invalid_argument(
            "Downwash-gradient table must be empty "
            "or contain at least two points"
        );
    }

    for (
        std::size_t index = 0;
        index < table.size();
        ++index
    ) {
        requireNonNegative(
            table[index].mach,
            "Downwash-gradient Mach number"
        );

        requireNonNegative(
            table[index].gradient,
            "Downwash gradient"
        );

        if (
            index > 0 &&
            table[index].mach <=
                table[index - 1].mach
        ) {
            throw std::invalid_argument(
                "Downwash-gradient Mach numbers "
                "must be strictly increasing"
            );
        }
    }
}

void validateAlphaDotDerivativeTable(
    const std::vector<
        PitchMomentAlphaDotDerivativePoint
    >& table
) {
    /*
     * Пустая таблица допустима.
     *
     * Она означает, что вклад mz по alphaDot
     * пока физически не задан и отключён.
     */
    if (table.empty()) {
        return;
    }

    if (table.size() < 2) {
        throw std::invalid_argument(
            "Alpha-dot derivative table must be empty "
            "or contain at least two points"
        );
    }

    for (
        std::size_t index = 0;
        index < table.size();
        ++index
    ) {
        requireNonNegative(
            table[index].mach,
            "Alpha-dot derivative Mach number"
        );

        requireFinite(
            table[index].mzAlphaDotDerivative,
            "Alpha-dot moment derivative"
        );

        if (
            index > 0 &&
            table[index].mach <=
                table[index - 1].mach
        ) {
            throw std::invalid_argument(
                "Alpha-dot derivative Mach numbers "
                "must be strictly increasing"
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
 * граничное значение.
 */
double interpolateDragCoefficient(
    double mach,
    const std::vector<DragCoefficientPoint>& table
) {
    if (
        mach <=
        table.front().mach
    ) {
        return
            table.front()
                .zeroLiftDragCoefficient;
    }

    if (
        mach >=
        table.back().mach
    ) {
        return
            table.back()
                .zeroLiftDragCoefficient;
    }

    const auto upper =
        std::upper_bound(
            table.begin(),
            table.end(),
            mach,
            [](
                double value,
                const DragCoefficientPoint& point
            ) {
                return
                    value <
                    point.mach;
            }
        );

    const auto lower =
        upper - 1;

    const double interval =
        upper->mach -
        lower->mach;

    const double interpolationParameter =
        (
            mach -
            lower->mach
        ) /
        interval;

    return
        lower->zeroLiftDragCoefficient +
        interpolationParameter *
        (
            upper->zeroLiftDragCoefficient -
            lower->zeroLiftDragCoefficient
        );
}

/**
 * Интерполяция зависимости
 *
 *     epsilonAlpha(M) = d epsilon / d alpha.
 *
 * Если таблица пустая, возвращается резервное значение
 * из AerodynamicGeometry. За пределами непустой таблицы
 * используется ближайшее граничное значение.
 */
double interpolateDownwashGradient(
    double mach,
    const std::vector<DownwashGradientPoint>& table,
    double fallbackGradient
) {
    if (table.empty()) {
        return fallbackGradient;
    }

    if (mach <= table.front().mach) {
        return table.front().gradient;
    }

    if (mach >= table.back().mach) {
        return table.back().gradient;
    }

    const auto upper =
        std::upper_bound(
            table.begin(),
            table.end(),
            mach,
            [](
                double value,
                const DownwashGradientPoint& point
            ) {
                return value < point.mach;
            }
        );

    const auto lower = upper - 1;

    const double interpolationParameter =
        (mach - lower->mach) /
        (upper->mach - lower->mach);

    return
        lower->gradient +
        interpolationParameter *
        (upper->gradient - lower->gradient);
}

/**
 * Интерполяция зависимости
 *
 *     mzAlphaDotDerivative(M).
 *
 * Пустая таблица означает отсутствие надёжных
 * исходных данных и даёт производную, равную нулю.
 *
 * За пределами непустой таблицы используется
 * ближайшее граничное значение.
 */
double interpolateAlphaDotDerivative(
    double mach,
    const std::vector<
        PitchMomentAlphaDotDerivativePoint
    >& table
) {
    if (table.empty()) {
        return 0.0;
    }

    if (
        mach <=
        table.front().mach
    ) {
        return
            table.front()
                .mzAlphaDotDerivative;
    }

    if (
        mach >=
        table.back().mach
    ) {
        return
            table.back()
                .mzAlphaDotDerivative;
    }

    const auto upper =
        std::upper_bound(
            table.begin(),
            table.end(),
            mach,
            [](
                double value,
                const PitchMomentAlphaDotDerivativePoint& point
            ) {
                return
                    value <
                    point.mach;
            }
        );

    const auto lower =
        upper - 1;

    const double interval =
        upper->mach -
        lower->mach;

    const double interpolationParameter =
        (
            mach -
            lower->mach
        ) /
        interval;

    return
        lower->mzAlphaDotDerivative +
        interpolationParameter *
        (
            upper->mzAlphaDotDerivative -
            lower->mzAlphaDotDerivative
        );
}

/**
 * Приближённая производная коэффициента подъёмной силы
 * конечного крыла по углу атаки.
 *
 * Используется выражение:
 *
 * c_y^alpha =
 *
 *     2*pi*lambda /
 *
 *     [
 *         2 + sqrt(
 *             4 +
 *             lambda^2 * beta^2 /
 *             cos^2(chi_0.5)
 *         )
 *     ].
 */
double liftingSurfaceSlopePerRad(
    const LiftingSurfaceAerodynamics& surface,
    double mach
) {
    const double machExpression =
        std::abs(
            1.0 -
            mach * mach
        );

    const double beta =
        std::sqrt(
            std::max(
                0.04,
                machExpression
            )
        );

    const double sweepCosine =
        std::cos(
            surface.halfChordSweepRad
        );

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
     * Временное ограничение трансзвуковой
     * особенности первой приближённой модели.
     */
    return
        std::min(
            slope,
            8.0
        );
}

/**
 * Производная нормальной силы корпуса.
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
        std::sqrt(
            1.0 +
            mach * mach
        );

    return
        2.0 *
        geometry.noseNormalForceFactor *
        bodyMidsectionAreaM2 /
        geometry.referenceAreaM2 *
        compressibilityCorrection;
}

/**
 * Вклад нормальной силы отдельной части объекта
 * в коэффициент продольного момента.
 *
 * Координата X направлена от носа к хвосту.
 *
 * При расположении аэродинамического центра
 * позади центра масс вклад получается отрицательным.
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

    return
        forceCoefficient *
        dimensionlessArm;
}

/**
 * Производная демпфирующего момента для части объекта,
 * представленной сосредоточенной нормальной силой.
 *
 * Используемая нормировка:
 *
 *     omegaZBar = omega_z * referenceChord / (2 * V).
 *
 * Для производной нормальной силы c_y^alpha и плеча l:
 *
 *     mz^omega =
 *         -2 * c_y^alpha * (l / referenceChord)^2.
 *
 * Квадрат плеча обеспечивает отрицательный
 * демпфирующий вклад независимо от того, находится
 * характерная точка впереди или позади центра масс.
 *
 * Эта модель применяется здесь к корпусу и стабилизатору.
 * Для крыла она недостаточна из-за распределённой нагрузки
 * по хорде, поэтому вклад крыла рассчитывается отдельно.
 */
double concentratedNormalForcePitchRateDerivative(
    double normalForceSlopePerRad,
    double aerodynamicCenterXM,
    const AerodynamicGeometry& geometry
) {
    const double dimensionlessArm =
        (
            aerodynamicCenterXM -
            geometry.centerOfMassXM
        ) /
        geometry.referenceChordM;

    return
        -2.0 *
        normalForceSlopePerRad *
        dimensionlessArm *
        dimensionlessArm;
}

} // namespace

AerodynamicGeometry
makeAbstract500AerodynamicGeometry() {
    AerodynamicGeometry geometry;

    geometry.referenceAreaM2 =
        0.475;

    geometry.referenceChordM =
        0.274;

    geometry.bodyDiameterM =
        0.400;

    /*
     * Синхронизировано с текущим
     * ObjectPassport базового объекта.
     */
    geometry.centerOfMassXM =
        1.170;

    geometry.bodyAerodynamicCenterXM =
        0.800;

    geometry.noseNormalForceFactor =
        1.0;

    geometry.wing.areaM2 =
        0.475;

    geometry.wing.aspectRatio =
        1.760 *
        1.760 /
        0.475;

    geometry.wing.halfChordSweepRad =
        40.0 *
        std::numbers::pi_v<double> /
        180.0;

    geometry.wing.efficiencyFactor =
        0.80;

    /*
     * В текущем паспорте объекта крыло
     * установлено под углом +3 градуса.
     */
    geometry.wing.installationAngleRad =
        3.0 *
        std::numbers::pi_v<double> /
        180.0;

    geometry.wing.aerodynamicCenterXM =
        1.150;

    geometry.tail.areaM2 =
        0.269;

    geometry.tail.aspectRatio =
        0.514 *
        0.514 /
        0.269;

    geometry.tail.halfChordSweepRad =
        26.3 *
        std::numbers::pi_v<double> /
        180.0;

    geometry.tail.efficiencyFactor =
        0.75;

    geometry.tail.installationAngleRad =
        0.0;

    geometry.tail.aerodynamicCenterXM =
        2.050;

    /*
     * Резервное дозвуковое значение производной
     * среднего угла скоса потока. Основной путь для
     * базового объекта использует таблицу epsilonAlpha(M).
     */
    geometry.downwashGradient =
        0.57;

    return geometry;
}

std::vector<DragCoefficientPoint>
makeAbstract500ZeroLiftDragTable() {
    /*
     * Таблица извлечена из переданной
     * модели SimInTech.
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

std::vector<DownwashGradientPoint>
makeAbstract500DownwashGradientTable() {
    /*
     * Дозвуковая оценка epsilonAlpha(M) по формуле (3.34)
     * Лебедева--Чернобровкина для текущей геометрии.
     *
     * Основные использованные параметры:
     *
     *     kAlphaI / KAlphaII ~= 0.535;
     *     i ~= 0.85 (графическая оценка по рис. 3.17);
     *     zBarB ~= 0.79...0.80 (рис. 3.16);
     *     psiEpsilon = 1 для M <= 1.
     *
     * Точность ограничена графическим считыванием старых
     * номограмм, поэтому лишние десятичные знаки не хранятся.
     *
     * Для M > 1 текущий интерполятор зажимает последнее
     * значение 0.569. Это предварительный fallback до тех пор,
     * пока не станет возможен расчёт psiEpsilon(M) по полной
     * взаимной продольной геометрии крыла и стабилизатора.
     */
    return {
        {0.20, 0.576},
        {0.40, 0.576},
        {0.60, 0.576},
        {0.80, 0.569},
        {1.00, 0.569}
    };
}

std::vector<PitchMomentAlphaDotDerivativePoint>
makeAbstract500PitchMomentAlphaDotDerivativeTable() {
    /*
     * Надёжная зависимость
     *
     *     mzAlphaDotDerivative(M)
     *
     * пока отсутствует.
     *
     * Поэтому вклад alphaDot сейчас отключён.
     *
     * Когда таблица будет получена или оцифрована,
     * сюда достаточно добавить точки вида:
     *
     *     {Mach, mzAlphaDotDerivative}
     *
     * с обязательной проверкой соглашения
     * о нормировке alphaDot.
     */
    return {};
}

PreliminaryAerodynamicModel::
PreliminaryAerodynamicModel()
    : PreliminaryAerodynamicModel(
          makeAbstract500AerodynamicGeometry(),
          makeAbstract500ZeroLiftDragTable(),
          makeAbstract500DownwashGradientTable(),
          makeAbstract500PitchMomentAlphaDotDerivativeTable()
      ) {
}

PreliminaryAerodynamicModel::
PreliminaryAerodynamicModel(
    const AerodynamicGeometry& geometry,
    std::vector<DragCoefficientPoint>
        zeroLiftDragTable
)
    : PreliminaryAerodynamicModel(
          geometry,
          std::move(zeroLiftDragTable),
          {},
          {}
      ) {
}

PreliminaryAerodynamicModel::
PreliminaryAerodynamicModel(
    const AerodynamicGeometry& geometry,
    std::vector<DragCoefficientPoint>
        zeroLiftDragTable,
    std::vector<
        PitchMomentAlphaDotDerivativePoint
    > alphaDotDerivativeTable
)
    : PreliminaryAerodynamicModel(
          geometry,
          std::move(zeroLiftDragTable),
          {},
          std::move(alphaDotDerivativeTable)
      ) {
}

PreliminaryAerodynamicModel::
PreliminaryAerodynamicModel(
    const AerodynamicGeometry& geometry,
    std::vector<DragCoefficientPoint>
        zeroLiftDragTable,
    std::vector<DownwashGradientPoint>
        downwashGradientTable,
    std::vector<
        PitchMomentAlphaDotDerivativePoint
    > alphaDotDerivativeTable
)
    : geometry_(geometry),
      zeroLiftDragTable_(
          std::move(zeroLiftDragTable)
      ),
      downwashGradientTable_(
          std::move(downwashGradientTable)
      ),
      alphaDotDerivativeTable_(
          std::move(alphaDotDerivativeTable)
      ) {
    validateGeometry(geometry_);
    validateDragTable(zeroLiftDragTable_);
    validateDownwashGradientTable(
        downwashGradientTable_
    );
    validateAlphaDotDerivativeTable(
        alphaDotDerivativeTable_
    );
}

AerodynamicCoefficients
PreliminaryAerodynamicModel::evaluate(
    const AerodynamicInput& input
) const {
    validateInput(
        input
    );

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

    /*
     * Эффективный угол атаки крыла:
     *
     * alphaWing =
     *     alpha + iWing.
     *
     * Для текущего базового объекта
     * iWing = +3 градуса.
     */
    const double wingEffectiveAngleRad =
        input.angleOfAttackRad +
        geometry_.wing.installationAngleRad;

    result.downwashGradient =
        interpolateDownwashGradient(
            input.mach,
            downwashGradientTable_,
            geometry_.downwashGradient
        );

    /*
     * Угол атаки стабилизатора:
     *
     * alphaTail =
     *     (1 - dEpsilon/dAlpha) * alpha
     *     + iTail.
     */
    const double tailEffectiveAngleRad =
        (
            1.0 -
            result.downwashGradient
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
     * Индуктивное сопротивление крыла
     * и стабилизатора.
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

    /*
     * Статический продольный момент:
     *
     * mzStatic =
     *     mzBody +
     *     mzWing +
     *     mzTail.
     */
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
     * Полная производная демпфирующего момента
     * теперь разложена по физическим источникам.
     *
     * 1. Корпус.
     *
     * В первом приближении нормальная сила корпуса
     * считается сосредоточенной в его аэродинамическом
     * центре. Это даёт:
     *
     *     mzBody^omega =
     *         -2 * cyBody^alpha * (lBody / bRef)^2.
     */
    result.mzPitchRateBodyDerivative =
        concentratedNormalForcePitchRateDerivative(
            bodySlopePerRad,
            geometry_.bodyAerodynamicCenterXM,
            geometry_
        );

    /*
     * 2. Крыло.
     *
     * Для крыла нельзя корректно использовать только
     * плечо его аэродинамического центра: даже при оси
     * вращения в районе фокуса распределение местных
     * углов атаки по хорде создаёт демпфирующий момент.
     *
     * До реализации отдельной методики для крыла
     * этот вклад явно оставляется нулевым.
     * Это означает "не смоделировано", а не
     * физическое отсутствие демпфирования крыла.
     */
    result.mzPitchRateWingDerivative =
        0.0;

    /*
     * 3. Стабилизатор.
     *
     * Для удалённого стабилизатора применяется
     * прежнее приближение сосредоточенной нормальной
     * силы на длинном плече:
     *
     *     mzTail^omega =
     *         -2 * cyTailRigid^alpha * (lTail / bRef)^2.
     *
     * Здесь cyTailRigid^alpha включает отношение площадей
     * и efficiencyFactor, но пока не содержит дополнительный
     * вращательный вклад от изменения скоса потока.
     */
    const double tailRigidSlopePerRad =
        tailLocalSlopePerRad *
        geometry_.tail.efficiencyFactor *
        tailAreaRatio;

    result.mzPitchRateTailDerivative =
        concentratedNormalForcePitchRateDerivative(
            tailRigidSlopePerRad,
            geometry_.tail.aerodynamicCenterXM,
            geometry_
        );

    result.mzPitchRateDerivative =
        result.mzPitchRateBodyDerivative +
        result.mzPitchRateWingDerivative +
        result.mzPitchRateTailDerivative;

    /*
     * Производная момента по alphaDot
     * больше не оценивается через произвольное
     * отношение к mzPitchRateDerivative.
     *
     * Она является независимой
     * табличной функцией числа Маха.
     */
    result.mzAlphaDotDerivative =
        interpolateAlphaDotDerivative(
            input.mach,
            alphaDotDerivativeTable_
        );

    double normalizedPitchRate =
        0.0;

    double normalizedAlphaDot =
        0.0;

    if (
        input.speedMps >
        1.0e-9
    ) {
        normalizedPitchRate =
            input.pitchRateRadS *
            geometry_.referenceChordM /
            (
                2.0 *
                input.speedMps
            );

        normalizedAlphaDot =
            input.angleOfAttackRateRadS *
            geometry_.referenceChordM /
            (
                2.0 *
                input.speedMps
            );
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
     * Аналитическая производная cy
     * по углу атаки объекта.
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
            1.0 -
            result.downwashGradient
        );

    /*
     * Аналитическая производная
     * статического mz по alpha.
     *
     * Постоянный угол установки крыла
     * создаёт ненулевой свободный член mz,
     * но не изменяет производную по alpha.
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
                    1.0 -
                    result.downwashGradient
                ),
            geometry_.tail.aerodynamicCenterXM,
            geometry_
        );

    return result;
}

const AerodynamicGeometry&
PreliminaryAerodynamicModel::geometry()
    const noexcept {
    return geometry_;
}

const std::vector<DragCoefficientPoint>&
PreliminaryAerodynamicModel::
zeroLiftDragTable()
    const noexcept {
    return zeroLiftDragTable_;
}

const std::vector<DownwashGradientPoint>&
PreliminaryAerodynamicModel::
downwashGradientTable()
    const noexcept {
    return downwashGradientTable_;
}

const std::vector<
    PitchMomentAlphaDotDerivativePoint
>&
PreliminaryAerodynamicModel::
alphaDotDerivativeTable()
    const noexcept {
    return alphaDotDerivativeTable_;
}

} // namespace passive_flight
