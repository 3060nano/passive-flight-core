#include "passive_flight/TabulatedAerodynamicModel.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace passive_flight {
namespace {

void requireFinite(
    double value,
    const char* name
) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(
            std::string(name) +
            " must be finite"
        );
    }
}

void validateTable(
    const std::vector<MachCoefficientPoint>& table,
    const char* name
) {
    if (table.empty()) {
        throw std::invalid_argument(
            std::string(name) +
            " table must not be empty"
        );
    }

    double previousMach = 0.0;
    bool first = true;

    for (const MachCoefficientPoint& point : table) {
        requireFinite(
            point.mach,
            "Mach"
        );

        requireFinite(
            point.value,
            name
        );

        if (point.mach < 0.0) {
            throw std::invalid_argument(
                std::string(name) +
                " table contains negative Mach"
            );
        }

        if (!first &&
            point.mach <= previousMach) {
            throw std::invalid_argument(
                std::string(name) +
                " table Mach values must be "
                "strictly increasing"
            );
        }

        previousMach = point.mach;
        first = false;
    }
}

double interpolate(
    const std::vector<MachCoefficientPoint>& table,
    double mach,
    const char* name
) {
    if (mach < table.front().mach ||
        mach > table.back().mach) {
        throw std::out_of_range(
            std::string(name) +
            " table does not cover requested Mach"
        );
    }

    const auto upper = std::lower_bound(
        table.begin(),
        table.end(),
        mach,
        [](
            const MachCoefficientPoint& point,
            double value
        ) {
            return point.mach < value;
        }
    );

    if (upper == table.begin()) {
        return upper->value;
    }

    if (upper == table.end()) {
        return table.back().value;
    }

    if (std::abs(upper->mach - mach) <=
        1.0e-12) {
        return upper->value;
    }

    const auto lower =
        upper - 1;

    const double interval =
        upper->mach -
        lower->mach;

    const double fraction =
        (mach - lower->mach) /
        interval;

    return
        lower->value +
        fraction *
        (
            upper->value -
            lower->value
        );
}

} // namespace

TabulatedAerodynamicModel::
TabulatedAerodynamicModel(
    double referenceLengthM,
    TabulatedAerodynamicData data
)
    : referenceLengthM_(
          referenceLengthM
      ),
      data_(
          std::move(data)
      ) {
    if (!std::isfinite(referenceLengthM_) ||
        referenceLengthM_ <= 0.0) {
        throw std::invalid_argument(
            "Reference length must be "
            "finite and positive"
        );
    }

    validateTable(
        data_.cx0,
        "Cx0"
    );

    validateTable(
        data_.cxAlphaSquared,
        "CxAlphaSquared"
    );

    validateTable(
        data_.cyAlpha,
        "CyAlpha"
    );

    validateTable(
        data_.mzAlpha,
        "MzAlpha"
    );

    validateTable(
        data_.mzPitchRate,
        "MzPitchRate"
    );
}

AerodynamicCoefficients
TabulatedAerodynamicModel::evaluate(
    const AerodynamicInput& input
) const {
    requireFinite(
        input.mach,
        "Mach"
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

    requireFinite(
        input.speedMps,
        "Speed"
    );

    if (input.mach < 0.0) {
        throw std::invalid_argument(
            "Mach must be non-negative"
        );
    }

    if (input.speedMps <= 0.0) {
        throw std::invalid_argument(
            "Speed must be positive"
        );
    }

    const double cx0 =
        interpolate(
            data_.cx0,
            input.mach,
            "Cx0"
        );

    const double cxAlphaSquared =
        interpolate(
            data_.cxAlphaSquared,
            input.mach,
            "CxAlphaSquared"
        );

    const double cyAlpha =
        interpolate(
            data_.cyAlpha,
            input.mach,
            "CyAlpha"
        );

    const double mzAlpha =
        interpolate(
            data_.mzAlpha,
            input.mach,
            "MzAlpha"
        );

    const double mzPitchRateDerivative =
        interpolate(
            data_.mzPitchRate,
            input.mach,
            "MzPitchRate"
        );

    const double alpha =
        input.angleOfAttackRad;

    const double cxAngleDependent =
        cxAlphaSquared *
        alpha *
        alpha;

    const double cy =
        cyAlpha *
        alpha;

    const double mzStatic =
        mzAlpha *
        alpha;

    /*
     * Соглашение Лебедева--Чернобровкина:
     *
     *     omegaBar = omega_z * L_ref / V.
     *
     * В отличие от старой предварительной модели
     * множитель 1/2 здесь отсутствует.
     */
    const double omegaBar =
        input.pitchRateRadS *
        referenceLengthM_ /
        input.speedMps;

    const double mzPitchDamping =
        mzPitchRateDerivative *
        omegaBar;

    AerodynamicCoefficients result;

    result.cx0 =
        cx0;

    result.cxInduced =
        cxAngleDependent;

    result.cx =
        cx0 +
        cxAngleDependent;

    result.downwashGradient =
        0.0;

    /*
     * Для обычной бомбы разложение на корпус,
     * крыло и стабилизатор не используется.
     * Полный Cy записываем в корпусную составляющую,
     * чтобы диагностические суммы оставались прозрачными.
     */
    result.cyBody =
        cy;

    result.cyWing =
        0.0;

    result.cyTail =
        0.0;

    result.cy =
        cy;

    result.mzStatic =
        mzStatic;

    result.mzPitchDamping =
        mzPitchDamping;

    /*
     * В исходном табличном наборе ФАБ-1500Т
     * производная по alphaDot отсутствует.
     */
    result.mzAlphaDot =
        0.0;

    result.mz =
        mzStatic +
        mzPitchDamping;

    result.cyAlphaPerRad =
        cyAlpha;

    result.mzAlphaPerRad =
        mzAlpha;

    result.mzPitchRateBodyDerivative =
        mzPitchRateDerivative;

    result.mzPitchRateWingDerivative =
        0.0;

    result.mzPitchRateTailDerivative =
        0.0;

    result.mzPitchRateDerivative =
        mzPitchRateDerivative;

    result.mzAlphaDotDerivative =
        0.0;

    return result;
}

double
TabulatedAerodynamicModel::
referenceLengthM() const noexcept {
    return referenceLengthM_;
}

const TabulatedAerodynamicData&
TabulatedAerodynamicModel::data() const noexcept {
    return data_;
}

} // namespace passive_flight
