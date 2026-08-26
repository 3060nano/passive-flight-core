#include "passive_flight/ObjectRegistry.hpp"

#include "passive_flight/ForwardEulerSimulator.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace passive_flight {
namespace {

std::string makeValidationErrorMessage(
    const ValidationIssues& issues
) {
    std::string message =
        "Object passport validation failed";

    for (const ValidationIssue& issue : issues) {
        message +=
            "\n" +
            issue.field +
            ": " +
            issue.message;
    }

    return message;
}

} // namespace

void ObjectRegistry::add(
    ObjectPassport passport
) {
    const ValidationIssues issues =
        validate(passport);

    if (!issues.empty()) {
        throw std::invalid_argument(
            makeValidationErrorMessage(
                issues
            )
        );
    }

    const std::string objectId =
        passport.object.id;

    if (contains(objectId)) {
        throw std::invalid_argument(
            "Object identifier is already registered: " +
            objectId
        );
    }

    const std::size_t newIndex =
        passports_.size();

    passports_.push_back(
        std::move(passport)
    );

    indexById_.emplace(
        objectId,
        newIndex
    );
}

bool ObjectRegistry::contains(
    const std::string& objectId
) const noexcept {
    return
        indexById_.find(objectId) !=
        indexById_.end();
}

const ObjectPassport*
ObjectRegistry::findPassport(
    const std::string& objectId
) const noexcept {
    const auto iterator =
        indexById_.find(objectId);

    if (iterator == indexById_.end()) {
        return nullptr;
    }

    return &passports_[iterator->second];
}

const ObjectModel*
ObjectRegistry::findObject(
    const std::string& objectId
) const noexcept {
    const ObjectPassport* passport =
        findPassport(objectId);

    if (passport == nullptr) {
        return nullptr;
    }

    return &passport->object;
}

std::vector<ObjectDescriptor>
ObjectRegistry::descriptors() const {
    std::vector<ObjectDescriptor> result;

    result.reserve(
        passports_.size()
    );

    for (const ObjectPassport& passport :
         passports_) {
        ObjectDescriptor descriptor;

        descriptor.id =
            passport.object.id;

        descriptor.displayName =
            passport.object.metadata.displayName;

        descriptor.modelVersion =
            passport.object.metadata.modelVersion;

        result.push_back(
            std::move(descriptor)
        );
    }

    return result;
}

std::size_t ObjectRegistry::size() const noexcept {
    return passports_.size();
}

bool ObjectRegistry::empty() const noexcept {
    return passports_.empty();
}

SimulationResult ObjectRegistry::simulate(
    const SimulationRequest& request,
    const SimulationOptions& options
) const {
    const ObjectModel* object =
        findObject(
            request.objectId
        );

    if (object == nullptr) {
        SimulationResult result;

        result.terminationReason =
            TerminationReason::InvalidInput;

        return result;
    }

    const ForwardEulerSimulator simulator(
        *object
    );

    return simulator.simulate(
        request,
        options
    );
}

ObjectRegistry makeDefaultObjectRegistry() {
    ObjectRegistry registry;

    registry.add(
        makeAbstract500UmpkPassport()
    );

    return registry;
}

} // namespace passive_flight