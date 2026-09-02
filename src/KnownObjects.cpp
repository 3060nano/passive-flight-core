#include "passive_flight/KnownObjects.hpp"

#include <numbers>

namespace passive_flight {

TabulatedAerodynamicData
makeFab1500TPostnikovAerodynamicData() {
    TabulatedAerodynamicData data;

    /*
     * Cx0(M).
     */
    data.cx0 = {
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

    /*
     * Cx^(alpha^2)(M), 1/rad^2.
     */
    data.cxAlphaSquared = {
        {0.40, 4.0},
        {0.75, 4.5},
        {0.90, 5.4},
        {1.10, 6.0},
        {1.35, 5.9},
        {2.00, 5.6},
        {3.50, 5.0}
    };

    /*
     * Cy^alpha(M), 1/rad.
     *
     * В таблице источника для ФАБ-1500Т
     * производная постоянна в приведённых узлах.
     */
    data.cyAlpha = {
        {0.40, 3.9},
        {0.75, 3.9},
        {0.90, 3.9},
        {1.10, 3.9},
        {1.35, 3.9},
        {2.00, 3.9},
        {3.50, 3.9}
    };

    /*
     * mz^alpha(M), 1/rad.
     */
    data.mzAlpha = {
        {0.40, -0.68},
        {0.75, -0.84},
        {0.90, -0.88},
        {1.10, -0.94},
        {1.35, -0.84},
        {2.00, -0.62},
        {3.50, -0.60}
    };

    /*
     * mz^(omegaBar)(M).
     *
     * Безразмерная угловая скорость:
     *
     *     omegaBar = omega_z * L / V.
     */
    data.mzPitchRate = {
        {0.40, -1.00},
        {0.75, -0.97},
        {0.90, -0.95},
        {1.10, -1.25},
        {1.35, -1.50},
        {2.00, -1.43},
        {3.50, -1.30}
    };

    return data;
}

ObjectModel makeFab1500TPostnikovModel() {
    ObjectModel object;

    object.id =
        "FAB_1500T_POSTNIKOV_1979";

    object.metadata.displayName =
        "ФАБ-1500Т — контрольная табличная модель";

    object.metadata.modelVersion =
        "1.0.0";

    object.metadata.description =
        "Контрольная модель свободнопадающей авиабомбы "
        "ФАБ-1500Т с готовыми массовыми и "
        "аэродинамическими характеристиками по "
        "Постникову и Чуйко, 1979.";

    /*
     * Массовые характеристики.
     */
    object.mass.massKg =
        1519.0;

    object.mass.pitchMomentOfInertiaKgM2 =
        1122.3;

    object.mass.centerOfMassXM =
        1.16;

    /*
     * Геометрия корпуса.
     */
    object.body.lengthM =
        3.46;

    object.body.diameterM =
        0.58;

    /*
     * Табличная аэродинамика уже включает влияние
     * реальной формы бомбы. Поэтому для текущей
     * проверки не вводим фиктивную геометрию
     * крыла, стабилизатора или носовой части.
     */
    object.body.noseLengthM =
        0.0;

    object.body.tailLengthM =
        0.0;

    object.body.noseShape =
        NoseShape::Unspecified;

    object.body.zeroLiftDragCoefficientOnFrontalArea =
        0.0;

    /*
     * Характерная площадь — площадь миделя:
     *
     *     S = pi * D^2 / 4.
     */
    object.reference.areaM2 =
        std::numbers::pi_v<double> *
        object.body.diameterM *
        object.body.diameterM /
        4.0;

    /*
     * Для табличной бомбы span и САХ отсутствуют.
     * Эти поля не используются Tabulated-моделью.
     */
    object.reference.spanM =
        0.0;

    object.reference.meanAerodynamicChordM =
        0.0;

    /*
     * В коэффициенте момента источника
     * характерной длиной является длина бомбы.
     */
    object.reference.referenceLengthM =
        object.body.lengthM;

    object.aerodynamicModelType =
        AerodynamicModelType::Tabulated;

    object.tabulatedAerodynamics =
        makeFab1500TPostnikovAerodynamicData();

    return object;
}

} // namespace passive_flight
