#include "passive_flight/ObjectAerodynamicsAdapter.hpp"
#include "passive_flight/ObjectPassport.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
#include <string>

namespace {

int failureCount = 0;

void check(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        ++failureCount;
    }
}

void checkNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message
) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr
            << "FAILED: " << message
            << "; expected " << expected
            << ", actual " << actual
            << ", tolerance " << tolerance
            << '\n';

        ++failureCount;
    }
}

passive_flight::AerodynamicInput makeTestInput() {
    passive_flight::AerodynamicInput input;

    input.mach = 0.8;

    input.angleOfAttackRad =
        5.0 *
        std::numbers::pi_v<double> /
        180.0;

    input.pitchRateRadS = 0.0;
    input.angleOfAttackRateRadS = 0.0;
    input.speedMps = 300.0;

    return input;
}

void testBaselineConversion() {
    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const auto geometry =
        passive_flight::makeAerodynamicGeometry(
            passport.object
        );

    checkNear(
        geometry.referenceAreaM2,
        passport.object.reference.areaM2,
        1.0e-12,
        "Reference area is copied from ObjectModel"
    );

    checkNear(
        geometry.referenceChordM,
        passport.object.reference.meanAerodynamicChordM,
        1.0e-12,
        "Reference chord is copied from ObjectModel"
    );

    checkNear(
        geometry.bodyDiameterM,
        passport.object.body.diameterM,
        1.0e-12,
        "Body diameter is copied from ObjectModel"
    );

    checkNear(
        geometry.centerOfMassXM,
        passport.object.mass.centerOfMassXM,
        1.0e-12,
        "Center of mass is copied from ObjectModel"
    );

    checkNear(
        geometry.wing.areaM2,
        passport.object.wing.areaM2,
        1.0e-12,
        "Wing area is copied from ObjectModel"
    );

    checkNear(
        geometry.wing.aspectRatio,
        passport.object.wing.aspectRatio(),
        1.0e-12,
        "Wing aspect ratio is calculated from ObjectModel"
    );

    checkNear(
        geometry.tail.areaM2,
        passport.object.tail.areaM2,
        1.0e-12,
        "Tail area is copied from ObjectModel"
    );

    checkNear(
        geometry.tail.aspectRatio,
        passport.object.tail.aspectRatio(),
        1.0e-12,
        "Tail aspect ratio is calculated from ObjectModel"
    );

    checkNear(
        geometry.bodyAerodynamicCenterXM,
        passport.object.body.lengthM / 3.0,
        1.0e-12,
        "Ogival body aerodynamic center"
    );

    checkNear(
        geometry.noseNormalForceFactor,
        1.0,
        1.0e-12,
        "Ogival nose normal-force factor"
    );
}

void testWingGeometryInfluence() {
    auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const auto baselineGeometry =
        passive_flight::makeAerodynamicGeometry(
            passport.object
        );

    passport.object.wing.spanM *= 1.20;

    const auto modifiedGeometry =
        passive_flight::makeAerodynamicGeometry(
            passport.object
        );

    check(
        modifiedGeometry.wing.aspectRatio >
            baselineGeometry.wing.aspectRatio,
        "Increasing wing span increases aspect ratio"
    );

    const passive_flight::PreliminaryAerodynamicModel baselineModel(
        baselineGeometry,
        passive_flight::makeAbstract500ZeroLiftDragTable()
    );

    const passive_flight::PreliminaryAerodynamicModel modifiedModel(
        modifiedGeometry,
        passive_flight::makeAbstract500ZeroLiftDragTable()
    );

    const auto input = makeTestInput();

    const auto baselineResult =
        baselineModel.evaluate(input);

    const auto modifiedResult =
        modifiedModel.evaluate(input);

    check(
        modifiedResult.cyWing >
            baselineResult.cyWing,
        "Changing wing geometry changes wing lift"
    );

    check(
        std::abs(
            modifiedResult.mz -
            baselineResult.mz
        ) > 1.0e-6,
        "Changing wing geometry changes pitching moment"
    );
}

void testTailGeometryInfluence() {
    auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const auto baselineModel =
        passive_flight::makeAerodynamicModel(
            passport.object
        );

    passport.object.tail.areaM2 *= 1.25;

    const auto modifiedModel =
        passive_flight::makeAerodynamicModel(
            passport.object
        );

    const auto input = makeTestInput();

    const auto baselineResult =
        baselineModel.evaluate(input);

    const auto modifiedResult =
        modifiedModel.evaluate(input);

    check(
        modifiedResult.cyTail >
            baselineResult.cyTail,
        "Increasing tail area increases tail normal force"
    );

    check(
        std::abs(
            modifiedResult.mz -
            baselineResult.mz
        ) > 1.0e-6,
        "Changing tail area changes pitching moment"
    );
}

void testNoseShapeInfluence() {
    auto ogivalPassport =
        passive_flight::makeAbstract500UmpkPassport();

    auto conicalPassport =
        ogivalPassport;

    conicalPassport.object.body.noseShape =
        passive_flight::NoseShape::Conical;

    const auto ogivalGeometry =
        passive_flight::makeAerodynamicGeometry(
            ogivalPassport.object
        );

    const auto conicalGeometry =
        passive_flight::makeAerodynamicGeometry(
            conicalPassport.object
        );

    check(
        conicalGeometry.noseNormalForceFactor >
            ogivalGeometry.noseNormalForceFactor,
        "Nose shape changes body normal-force factor"
    );

    check(
        conicalGeometry.bodyAerodynamicCenterXM <
            ogivalGeometry.bodyAerodynamicCenterXM,
        "Conical nose shifts body aerodynamic center forward"
    );

    const auto ogivalModel =
        passive_flight::makeAerodynamicModel(
            ogivalPassport.object
        );

    const auto conicalModel =
        passive_flight::makeAerodynamicModel(
            conicalPassport.object
        );

    const auto input = makeTestInput();

    const auto ogivalResult =
        ogivalModel.evaluate(input);

    const auto conicalResult =
        conicalModel.evaluate(input);

    check(
        std::abs(
            conicalResult.cyBody -
            ogivalResult.cyBody
        ) > 1.0e-6,
        "Nose shape changes body normal force"
    );

    check(
        std::abs(
            conicalResult.mz -
            ogivalResult.mz
        ) > 1.0e-6,
        "Nose shape changes pitching moment"
    );
}

void testReadyModelCreation() {
    const auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    const auto model =
        passive_flight::makeAerodynamicModel(
            passport.object
        );

    const auto result =
        model.evaluate(
            makeTestInput()
        );

    check(
        result.cx > 0.0,
        "Adapter-created model calculates positive Cx"
    );

    check(
        result.cy > 0.0,
        "Adapter-created model calculates positive Cy"
    );

    check(
        std::isfinite(result.mz),
        "Adapter-created model calculates finite Mz"
    );
}

} // namespace

int main() {
    testBaselineConversion();
    testWingGeometryInfluence();
    testTailGeometryInfluence();
    testNoseShapeInfluence();
    testReadyModelCreation();

    if (failureCount != 0) {
        std::cerr
            << failureCount
            << " object-aerodynamics adapter test(s) failed"
            << '\n';

        return 1;
    }

    std::cout
        << "All object-aerodynamics adapter tests passed"
        << '\n';

    return 0;
}