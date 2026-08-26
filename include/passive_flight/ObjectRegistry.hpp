#pragma once

#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/Types.hpp"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace passive_flight {

/**
 * Краткое описание объекта для внешнего интерфейса.
 *
 * Эта структура впоследствии будет использоваться:
 *
 * - DLL;
 * - блоком SimInTech;
 * - пользовательским приложением;
 * - списком выбора объекта.
 */
struct ObjectDescriptor {
    std::string id;
    std::string displayName;
    std::string modelVersion;
};

/**
 * Реестр расчётных объектов.
 *
 * Каждый объект имеет уникальный строковый идентификатор.
 * Один и тот же решатель может использоваться для любого
 * объекта, зарегистрированного в реестре.
 */
class ObjectRegistry {
public:
    ObjectRegistry() = default;

    /**
     * Добавляет паспорт объекта.
     *
     * Перед добавлением выполняется полная валидация.
     *
     * @throws std::invalid_argument:
     * - если паспорт некорректен;
     * - если идентификатор уже зарегистрирован.
     */
    void add(
        ObjectPassport passport
    );

    /**
     * Проверяет наличие объекта.
     */
    [[nodiscard]]
    bool contains(
        const std::string& objectId
    ) const noexcept;

    /**
     * Возвращает паспорт объекта.
     *
     * Если объект не найден, возвращается nullptr.
     */
    [[nodiscard]]
    const ObjectPassport* findPassport(
        const std::string& objectId
    ) const noexcept;

    /**
     * Возвращает только расчётную модель объекта.
     *
     * Если объект не найден, возвращается nullptr.
     */
    [[nodiscard]]
    const ObjectModel* findObject(
        const std::string& objectId
    ) const noexcept;

    /**
     * Возвращает список доступных объектов
     * в порядке их регистрации.
     */
    [[nodiscard]]
    std::vector<ObjectDescriptor>
    descriptors() const;

    /**
     * Количество объектов в реестре.
     */
    [[nodiscard]]
    std::size_t size() const noexcept;

    /**
     * Проверяет, что реестр пуст.
     */
    [[nodiscard]]
    bool empty() const noexcept;

    /**
     * Выполняет моделирование объекта, выбранного
     * через SimulationRequest.objectId.
     *
     * Если идентификатор неизвестен, возвращает результат
     * с причиной TerminationReason::InvalidInput.
     */
    [[nodiscard]]
    SimulationResult simulate(
        const SimulationRequest& request,
        const SimulationOptions& options = {}
    ) const;

private:
    std::vector<ObjectPassport> passports_;

    std::unordered_map<
        std::string,
        std::size_t
    > indexById_;
};

/**
 * Создаёт стандартный реестр ядра.
 *
 * Сейчас он содержит один объект:
 *
 * ABSTRACT_500_UMPK_V1.
 *
 * В дальнейшем сюда будут добавлены новые паспорта.
 */
[[nodiscard]]
ObjectRegistry makeDefaultObjectRegistry();

} // namespace passive_flight