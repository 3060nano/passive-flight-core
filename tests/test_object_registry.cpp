#include "passive_flight/ObjectRegistry.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void require(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename ExceptionType, typename Function>
void requireThrows(
    Function function,
    const std::string& message
) {
    try {
        function();
    } catch (const ExceptionType&) {
        return;
    } catch (...) {
        throw std::runtime_error(
            message +
            ": unexpected exception type"
        );
    }

    throw std::runtime_error(
        message +
        ": exception was not thrown"
    );
}

passive_flight::SimulationOptions
makeTestOptions() {
    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 30.0;
    options.maximumSteps = 100000;
    options.saveHistory = false;

    return options;
}

void testEmptyRegistry() {
    const passive_flight::ObjectRegistry registry;

    require(
        registry.empty(),
        "New registry must be empty"
    );

    require(
        registry.size() == 0,
        "New registry size must be zero"
    );

    require(
        !registry.contains(
            "UNKNOWN_OBJECT"
        ),
        "Empty registry must not contain objects"
    );

    require(
        registry.findObject(
            "UNKNOWN_OBJECT"
        ) == nullptr,
        "Unknown object lookup must return nullptr"
    );

    require(
        registry.findPassport(
            "UNKNOWN_OBJECT"
        ) == nullptr,
        "Unknown passport lookup must return nullptr"
    );
}

void testDefaultRegistry() {
    const auto registry =
        passive_flight::makeDefaultObjectRegistry();

    require(
        !registry.empty(),
        "Default registry must not be empty"
    );

    require(
        registry.size() == 2,
        "Default registry must contain two objects"
    );

    require(
        registry.contains(
            "ABSTRACT_500_UMPK_V1"
        ),
        "Abstract object must be registered"
    );

    require(
        registry.contains(
            "FAB_1500T_POSTNIKOV_1979"
        ),
        "FAB-1500T must be registered"
    );

    const auto* abstractObject =
        registry.findObject(
            "ABSTRACT_500_UMPK_V1"
        );

    require(
        abstractObject != nullptr,
        "Abstract object lookup failed"
    );

    require(
        abstractObject->mass.massKg == 500.0,
        "Abstract object mass is incorrect"
    );

    const auto* fab =
        registry.findObject(
            "FAB_1500T_POSTNIKOV_1979"
        );

    require(
        fab != nullptr,
        "FAB-1500T lookup failed"
    );

    require(
        fab->mass.massKg == 1519.0,
        "FAB-1500T mass is incorrect"
    );

    require(
        fab->aerodynamicModelType ==
            passive_flight::AerodynamicModelType::Tabulated,
        "FAB-1500T must use tabulated aerodynamics"
    );
}

void testDescriptors() {
    const auto registry =
        passive_flight::makeDefaultObjectRegistry();

    const std::vector<
        passive_flight::ObjectDescriptor
    > descriptors =
        registry.descriptors();

    require(
        descriptors.size() == 2,
        "Descriptor list size is incorrect"
    );

    require(
        descriptors[0].id ==
            "ABSTRACT_500_UMPK_V1",
        "First descriptor identifier is incorrect"
    );

    require(
        descriptors[1].id ==
            "FAB_1500T_POSTNIKOV_1979",
        "Second descriptor identifier is incorrect"
    );

    require(
        !descriptors[0].displayName.empty(),
        "Abstract descriptor display name must not be empty"
    );

    require(
        !descriptors[1].displayName.empty(),
        "FAB-1500T descriptor display name must not be empty"
    );

    require(
        descriptors[0].modelVersion ==
            "0.2.0",
        "Abstract descriptor model version is incorrect"
    );

    require(
        descriptors[1].modelVersion ==
            "1.0.0",
        "FAB-1500T descriptor model version is incorrect"
    );
}

void testThirdObjectCanBeAdded() {
    passive_flight::ObjectRegistry registry =
        passive_flight::makeDefaultObjectRegistry();

    auto third =
        passive_flight::makeAbstract500UmpkPassport();

    third.object.id =
        "ABSTRACT_500_RESEARCH_VARIANT";

    third.object.metadata.displayName =
        "Исследовательская модификация";

    third.object.metadata.modelVersion =
        "0.1.0";

    registry.add(
        std::move(third)
    );

    require(
        registry.size() == 3,
        "Registry must contain three objects"
    );

    require(
        registry.contains(
            "ABSTRACT_500_RESEARCH_VARIANT"
        ),
        "Third object was not registered"
    );

    const auto descriptors =
        registry.descriptors();

    require(
        descriptors.size() == 3,
        "Three descriptors are expected"
    );

    require(
        descriptors[2].id ==
            "ABSTRACT_500_RESEARCH_VARIANT",
        "Third descriptor is incorrect"
    );
}

void testDuplicateIsRejected() {
    passive_flight::ObjectRegistry registry;

    registry.add(
        passive_flight::makeFab1500TPostnikovPassport()
    );

    requireThrows<std::invalid_argument>(
        [&registry]() {
            registry.add(
                passive_flight::
                    makeFab1500TPostnikovPassport()
            );
        },
        "Duplicate object identifier must be rejected"
    );

    require(
        registry.size() == 1,
        "Duplicate insertion must not change registry"
    );
}

void testInvalidPassportIsRejected() {
    passive_flight::ObjectRegistry registry;

    auto passport =
        passive_flight::makeFab1500TPostnikovPassport();

    passport.object.mass.massKg =
        -1519.0;

    requireThrows<std::invalid_argument>(
        [&registry, &passport]() {
            registry.add(passport);
        },
        "Invalid passport must be rejected"
    );

    require(
        registry.empty(),
        "Invalid passport must not be inserted"
    );
}

void testInconsistentPassportIsRejected() {
    passive_flight::ObjectRegistry registry;

    auto passport =
        passive_flight::makeAbstract500UmpkPassport();

    passport.object.wing.spanM =
        1.900;

    passport.object.reference.spanM =
        1.900;

    requireThrows<std::invalid_argument>(
        [&registry, &passport]() {
            registry.add(passport);
        },
        "Inconsistent provenance must be rejected"
    );

    require(
        registry.empty(),
        "Inconsistent passport must not be inserted"
    );
}

void testUnknownObjectSimulation() {
    const auto registry =
        passive_flight::makeDefaultObjectRegistry();

    passive_flight::SimulationRequest request;

    request.objectId =
        "UNKNOWN_OBJECT";

    request.release.altitudeM =
        1000.0;

    request.release.speedMps =
        200.0;

    const auto result =
        registry.simulate(request);

    require(
        result.terminationReason ==
            passive_flight::TerminationReason::InvalidInput,
        "Unknown object simulation must return InvalidInput"
    );
}

void testAbstractObjectSimulation() {
    const auto registry =
        passive_flight::makeDefaultObjectRegistry();

    passive_flight::SimulationRequest request;

    request.objectId =
        "ABSTRACT_500_UMPK_V1";

    request.release.altitudeM =
        100.0;

    request.release.speedMps =
        200.0;

    const auto result =
        registry.simulate(
            request,
            makeTestOptions()
        );

    require(
        result.terminationReason ==
            passive_flight::TerminationReason::GroundReached,
        "Abstract object must reach ground"
    );

    require(
        result.finalState.downrangeM > 0.0,
        "Abstract object downrange must be positive"
    );
}

void testFab1500TSimulation() {
    const auto registry =
        passive_flight::makeDefaultObjectRegistry();

    passive_flight::SimulationRequest request;

    request.objectId =
        "FAB_1500T_POSTNIKOV_1979";

    request.release.altitudeM =
        100.0;

    request.release.speedMps =
        200.0;

    const auto result =
        registry.simulate(
            request,
            makeTestOptions()
        );

    require(
        result.terminationReason ==
            passive_flight::TerminationReason::GroundReached,
        "FAB-1500T must reach ground"
    );

    require(
        result.finalState.downrangeM > 800.0 &&
        result.finalState.downrangeM < 1000.0,
        "FAB-1500T downrange sanity range"
    );

    require(
        result.finalState.timeS > 4.0 &&
        result.finalState.timeS < 5.0,
        "FAB-1500T fall-time sanity range"
    );

    require(
        result.finalState.speedMps > 180.0 &&
        result.finalState.speedMps < 220.0,
        "FAB-1500T impact-speed sanity range"
    );
}

} // namespace

int main() {
    try {
        testEmptyRegistry();
        testDefaultRegistry();
        testDescriptors();
        testThirdObjectCanBeAdded();
        testDuplicateIsRejected();
        testInvalidPassportIsRejected();
        testInconsistentPassportIsRejected();
        testUnknownObjectSimulation();
        testAbstractObjectSimulation();
        testFab1500TSimulation();

        std::cout
            << "All object registry tests passed."
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "Test failure: "
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }
}
