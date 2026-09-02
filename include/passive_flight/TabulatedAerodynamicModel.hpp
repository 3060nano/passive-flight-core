#pragma once

#include "passive_flight/Aerodynamics.hpp"
#include "passive_flight/TabulatedAerodynamicData.hpp"

namespace passive_flight {

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
     * Используются формулы:
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
     * где по Лебедеву--Чернобровкину:
     *
     *     omegaBar = omega_z * L_ref / V.
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
