#include "passive_flight/ObjectAerodynamicsAdapter.hpp"

#include <memory>
#include <stdexcept>

namespace passive_flight {
namespace {

/**
 * Поправочный коэффициент нормальной силы корпуса,
 * учитывающий форму носовой части.
 *
 * Значения пока предварительные:
 *
 * оживальная форма: 1.00;
 * коническая форма: 1.05.
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
 */
double bodyAerodynamicCenterXM(
    const BodyGeometry& body
) {
    switch (body.noseShape) {
        case NoseShape::Ogival:
            return
                body.lengthM /
                3.0;

        case NoseShape::Conical:
            return
                0.30 *
                body.lengthM;
    }

    throw std::invalid_argument(
        "Unsupported nose shape"
    );
}

LiftingSurfaceAerodynamics
makeWingAerodynamics(
    const WingGeometry& wing
) {
    LiftingSurfaceAerodynamics result;

    result.areaM2 =
        wing.areaM2;

    result.aspectRatio =
        wing.aspectRatio();

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

LiftingSurfaceAerodynamics
makeTailAerodynamics(
    const TailGeometry& tail
) {
    LiftingSurfaceAerodynamics result;

    result.areaM2 =
        tail.areaM2;

    result.aspectRatio =
        tail.aspectRatio();

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

AerodynamicGeometry
makeAerodynamicGeometry(
    const ObjectModel& object
) {
    AerodynamicGeometry result;

    /*
     * Характерные размеры всего объекта.
     *
     * PreliminaryAerodynamicModel пока по-прежнему
     * использует САХ крыла как свою характерную длину.
     */
    result.referenceAreaM2 =
        object.reference.areaM2;

    result.referenceChordM =
        object.reference
            .meanAerodynamicChordM;

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
     * Параметры крыла и стабилизатора
     * берутся непосредственно из паспорта.
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
     * Резервное дозвуковое значение производной
     * среднего угла скоса потока.
     */
    result.downwashGradient =
        0.57;

    return result;
}

std::shared_ptr<const AerodynamicModel>
makeAerodynamicModel(
    const ObjectModel& object
) {
    /*
     * На первом этапе существующий объект продолжает
     * использовать старую PreliminaryAerodynamicModel.
     *
     * На следующем шаге здесь появится выбор между
     * Preliminary/GeometryBased и Tabulated по паспорту.
     */
    return
        std::make_shared<
            PreliminaryAerodynamicModel
        >(
            makeAerodynamicGeometry(
                object
            ),
            makeAbstract500ZeroLiftDragTable(),
            makeAbstract500DownwashGradientTable(),
            makeAbstract500PitchMomentAlphaDotDerivativeTable()
        );
}

} // namespace passive_flight
