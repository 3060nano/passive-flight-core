#pragma once

#include "passive_flight/Aerodynamics.hpp"

#include <vector>

namespace passive_flight {

/**
 * Одна точка произвольной аэродинамической зависимости
 * от числа Маха.
 */
struct MachCoefficientPoint {
    double mach{0.0};
    double value{0.0};
};

/**
 * Полный набор табличных аэродинамических характеристик
 * пассивного объекта для продольного движения.
 *
 * Для ФАБ-1500Т будут использованы:
 *
 *     Cx0(M);
 *     Cx^(alpha^2)(M);
 *     Cy^alpha(M);
 *     mz^alpha(M);
 *     mz^(omegaBar)(M).
 */
struct TabulatedAerodynamicData {
    std::vector<MachCoefficientPoint> cx0;
    std::vector<MachCoefficientPoint> cxAlphaSquared;
    std::vector<MachCoefficientPoint> cyAlpha;
    std::vector<MachCoefficientPoint> mzAlpha;
    std::vector<MachCoefficientPoint> mzPitchRate;
};

/**
 * Аэродинамическая модель, использующая готовые
 * табличные характеристики объекта.
 *
 * Это реализация варианта №1:
 *
 *     полный паспорт объекта
 *     -> готовые аэродинамические коэффициенты
 *     -> движение.
 *
 * В модели приняты формулы:
 *
 *     Cx =
 *         Cx0(M) +
 *         Cx^(alpha^2)(M) * alpha^2;
 *
 *     Cy =
 *         Cy^alpha(M) * alpha;
 *
 *     mz =
 *         mz^alpha(M) * alpha +
 *         mz^(omegaBar)(M) * omegaBar;
 *
 * где безразмерная угловая скорость определяется
 * по соглашению Лебедева--Чернобровкина:
 *
 *     omegaBar = omega_z * L_ref / V.
 *
 * Никакого множителя 1/2 здесь нет.
 */
class TabulatedAerodynamicModel final
    : public AerodynamicModel {
public:
    TabulatedAerodynamicModel(
        double referenceLengthM,
        TabulatedAerodynamicData data
    );

    [[nodiscard]]
    AerodynamicCoefficients evaluate(
        const AerodynamicInput& input
    ) const override;

    [[nodiscard]]
    double referenceLengthM() const noexcept;

    [[nodiscard]]
    const TabulatedAerodynamicData& data() const noexcept;

private:
    double referenceLengthM_{0.0};
    TabulatedAerodynamicData data_;
};

} // namespace passive_flight
