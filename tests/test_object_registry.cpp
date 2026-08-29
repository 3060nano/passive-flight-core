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
        registry.size() == 1,
        "Default registry must contain one object"
    );

    require(
        registry.contains(
            "ABSTRACT_500_UMPK_V1"
        ),
        "Default object must be registered"
    );

    const auto* object =
        registry.findObject(
            "ABSTRACT_500_UMPK_V1"
        );

    require(
        object != nullptr,
        "Default object lookup failed"
    );

    require(
        object->mass.massKg == 500.0,
        "Default object mass is incorrect"
    );

    require(
        object->mass.centerOfMassXM == 1.15,
        "Default object center of mass is incorrect"
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
        descriptors.size() == 1,
        "Descriptor list size is incorrect"
    );

    require(
        descriptors[0].id ==
            "ABSTRACT_500_UMPK_V1",
        "Descriptor identifier is incorrect"
    );

    require(
        !descriptors[0].displayName.empty(),
        "Descriptor display name must not be empty"
    );

    require(
        descriptors[0].modelVersion ==
            "0.2.0",
        "Descriptor model version is incorrect"
    );
}

void testSecondObjectCanBeAdded() {
    passive_flight::ObjectRegistry registry;

    auto first =
        passive_flight::makeAbstract500UmpkPassport();

    auto second =
        passive_flight::makeAbstract500UmpkPassport();

    /*
     * В этом тесте проверяется только возможность
     * регистрации нескольких объектов.
     *
     * Геометрия не изменяется, поскольку изменение
     * паспортного параметра требует одновременного
     * обновления его provenance-записи.
     */
    second.object.id =
        "ABSTRACT_500_RESEARCH_VARIANT";

    second.object.metadata.displayName =
        "Исследовательская модификация";

    second.object.metadata.modelVersion =
        "0.1.0";

    registry.add(
        std::move(first)
    );

    registry.add(
        std::move(second)
    );

    require(
        registry.size() == 2,
        "Registry must contain two objects"
    );

    require(
        registry.contains(
            "ABSTRACT_500_RESEARCH_VARIANT"
        ),
        "Second object was not registered"
    );

    const auto descriptors =
        registry.descriptors();

    require(
        descriptors.size() == 2,
        "Two descriptors are expected"
    );

    require(
        descriptors[0].id ==
            "ABSTRACT_500_UMPK_V1",
        "Registration order must be preserved"
    );

    require(
        descriptors[1].id ==
            "ABSTRACT_500_RESEARCH_VARIANT",
        "Second descriptor is incorrect"
    );

    const auto* secondObject =
        registry.findObject(
            "ABSTRACT_500_RESEARCH_VARIANT"
        );

    require(
        secondObject != nullptr,
        "Second object lookup failed"
    );

    require(
        secondObject->metadata.displayName ==
            "Исследовательская модификация",
        "Second object display name is incorrect"
    );
}

void testDuplicateIsRejected() {
    passive_flight::ObjectRegistry registry;

    registry.add(
        passive_flight::makeAbstract500UmpkPassport()
    );

    requireThrows<std::invalid_argument>(
        [&registry]() {
            registry.add(
                passive_flight::
                    makeAbstract500UmpkPassport()
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
        passive_flight::makeAbstract500UmpkPassport();

    passport.object.mass.massKg =
        -500.0;

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

    /*
     * Изменение геометрии без изменения provenance-записи
     * должно обнаруживаться валидатором.
     */
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

void testKnownObjectSimulation() {
    const auto registry =
        passive_flight::makeDefaultObjectRegistry();

    passive_flight::SimulationRequest request;

    request.objectId =
        "ABSTRACT_500_UMPK_V1";

    request.release.altitudeM =
        100.0;

    request.release.speedMps =
        200.0;

    passive_flight::SimulationOptions options;

    options.timeStepS = 0.001;
    options.maximumTimeS = 30.0;
    options.maximumSteps = 100000;
    options.saveHistory = false;

    const auto result =
        registry.simulate(
            request,
            options
        );

    require(
        result.terminationReason ==
            passive_flight::TerminationReason::GroundReached,
        "Known object simulation must reach ground"
    );

    require(
        result.finalState.downrangeM > 0.0,
        "Known object downrange must be positive"
    );

    require(
        result.finalState.timeS > 0.0,
        "Known object fall time must be positive"
    );
}

} // namespace

int main() {
    try {
        testEmptyRegistry();
        testDefaultRegistry();
        testDescriptors();
        testSecondObjectCanBeAdded();
        testDuplicateIsRejected();
        testInvalidPassportIsRejected();
        testInconsistentPassportIsRejected();
        testUnknownObjectSimulation();
        testKnownObjectSimulation();

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