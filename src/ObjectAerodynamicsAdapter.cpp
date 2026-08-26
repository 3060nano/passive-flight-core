#include "passive_flight/ObjectAerodynamicsAdapter.hpp"

#include <stdexcept>

namespace passive_flight {
namespace {

/**
 * Поправочный коэффициент нормальной силы корпуса,
 * учитывающий форму носовой части.
 *
 * Значения являются предварительными:
 *
 * оживальная форма: 1.00;
 * коническая форма: 1.05.
 *
 * После оцифровки графиков учебника эта функция
 * будет заменена табличной зависимостью от:
 *
 * - числа Маха;
 * - удлинения носовой части;
 * - формы носовой части.
 */
double noseNormalForceFactor(
    NoseShape noseShape
) {
    switch (noseShape) {
        case NoseShape::Ogival:
            return 1.00;

        case NoseShape::Conical:
            return 1.05;
    }

    throw std::invalid_argument(
        "Unsupported nose shape"
    );
}

/**
 * Приближённое положение аэродинамического центра
 * нормальной силы корпуса.
 *
 * Координата отсчитывается от носа объекта.
 *
 * Для оживальной формы предварительно принимается:
 *
 * x_f_body = L_body / 3.
 *
 * Для конической формы центр давления смещается
 * немного вперёд:
 *
 * x_f_body = 0.30 * L_body.
 *
 * Позднее зависимость будет уточнена по графикам
 * Лебедева и Чернобровкина.
 */
double bodyAerodynamicCenterXM(
    const BodyGeometry& body
) {
    switch (body.noseShape) {
        case NoseShape::Ogival:
            return body.lengthM / 3.0;

        case NoseShape::Conical:
            return 0.30 * body.lengthM;
    }

    throw std::invalid_argument(
        "Unsupported nose shape"
    );
}

LiftingSurfaceAerodynamics makeWingAerodynamics(
    const WingGeometry& wing
) {
    LiftingSurfaceAerodynamics result;

    result.areaM2 =
        wing.areaM2;

    result.aspectRatio =
        wing.aspectRatio();

    /*
     * В ObjectModel поле называется sweepHalfChordRad,
     * а в AerodynamicGeometry — halfChordSweepRad.
     */
    result.halfChordSweepRad =
        wing.sweepHalfChordRad;

    result.efficiencyFactor =
        wing.efficiencyFactor;

    result.installationAngleRad =
        wing.installationAngleRad;

    result.aerodynamicCenterXM =
        wing.aerodynamicCenterXM;

    return result;
}

LiftingSurfaceAerodynamics makeTailAerodynamics(
    const TailGeometry& tail
) {
    LiftingSurfaceAerodynamics result;

    result.areaM2 =
        tail.areaM2;

    result.aspectRatio =
        tail.aspectRatio();

    /*
     * В ObjectModel поле называется sweepHalfChordRad,
     * а в AerodynamicGeometry — halfChordSweepRad.
     */
    result.halfChordSweepRad =
        tail.sweepHalfChordRad;

    result.efficiencyFactor =
        tail.efficiencyFactor;

    result.installationAngleRad =
        tail.installationAngleRad;

    result.aerodynamicCenterXM =
        tail.aerodynamicCenterXM;

    return result;
}

} // namespace

AerodynamicGeometry makeAerodynamicGeometry(
    const ObjectModel& object
) {
    AerodynamicGeometry result;

    /*
     * Характерные размеры всего объекта.
     */
    result.referenceAreaM2 =
        object.reference.areaM2;

    result.referenceChordM =
        object.reference.meanAerodynamicChordM;

    /*
     * Параметры корпуса.
     */
    result.bodyDiameterM =
        object.body.diameterM;

    result.centerOfMassXM =
        object.mass.centerOfMassXM;

    result.bodyAerodynamicCenterXM =
        bodyAerodynamicCenterXM(
            object.body
        );

    result.noseNormalForceFactor =
        noseNormalForceFactor(
            object.body.noseShape
        );

    /*
     * Параметры крыла и стабилизатора берутся
     * непосредственно из паспорта объекта.
     */
    result.wing =
        makeWingAerodynamics(
            object.wing
        );

    result.tail =
        makeTailAerodynamics(
            object.tail
        );

    /*
     * На данном этапе производная скоса потока
     * принимается постоянной.
     */
    result.downwashGradient = 0.25;

    /*
     * Первая приближённая оценка производной
     * момента по скорости изменения угла атаки.
     */
    result.alphaDotDampingRatio = 0.35;

    return result;
}

PreliminaryAerodynamicModel makeAerodynamicModel(
    const ObjectModel& object
) {
    return PreliminaryAerodynamicModel(
        makeAerodynamicGeometry(object),
        makeAbstract500ZeroLiftDragTable()
    );
}

} // namespace passive_flight