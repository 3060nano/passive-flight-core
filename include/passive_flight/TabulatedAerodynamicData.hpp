#pragma once

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
 * Полный набор готовых табличных аэродинамических
 * характеристик пассивного объекта.
 *
 * Для ФАБ-1500Т используются:
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

} // namespace passive_flight
