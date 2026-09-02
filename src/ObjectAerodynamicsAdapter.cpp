#include "passive_flight/ObjectAerodynamicsAdapter.hpp"

#include "passive_flight/TabulatedAerodynamicModel.hpp"

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

        case NoseShape::Unspecified:
            break;
    }

    throw std::invalid_argument(
        "Nose shape is not available for "
        "preliminary geometry-based aerodynamics"
    );
}

/**
 * Приближённое положение аэродинамического центра
 * нормальной силы корпуса.
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

        case NoseShape::Unspecified:
            break;
    }

    throw std::invalid_argument(
        "Nose shape is not available for "
        "preliminary geometry-based aerodynamics"
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
    if (object.aerodynamicModelType !=
        AerodynamicModelType::PreliminaryGeometryBased) {
        throw std::invalid_argument(
            "AerodynamicGeometry is only available "
            "for PreliminaryGeometryBased objects"
        );
    }

    AerodynamicGeometry result;

    result.referenceAreaM2 =
        object.reference.areaM2;

    result.referenceChordM =
        object.reference
            .meanAerodynamicChordM;

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

    result.wing =
        makeWingAerodynamics(
            object.wing
        );

    result.tail =
        makeTailAerodynamics(
            object.tail
        );

    result.downwashGradient =
        0.57;

    return result;
}

std::shared_ptr<const AerodynamicModel>
makeAerodynamicModel(
    const ObjectModel& object
) {
    switch (object.aerodynamicModelType) {
        case AerodynamicModelType::PreliminaryGeometryBased:
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

        case AerodynamicModelType::Tabulated:
            return
                std::make_shared<
                    TabulatedAerodynamicModel
                >(
                    object.reference
                        .effectiveReferenceLengthM(),
                    object.tabulatedAerodynamics
                );
    }

    throw std::invalid_argument(
        "Unsupported aerodynamic model type"
    );
}

} // namespace passive_flight
