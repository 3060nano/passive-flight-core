#pragma once

#include "passive_flight/ModelContract.hpp"
#include "passive_flight/ObjectModel.hpp"

#include <string>
#include <variant>
#include <vector>

namespace passive_flight {

/*
 * Статус исходного параметра.
 */
enum class ParameterStatus {
    /*
     * Значение задано постановкой задачи.
     */
    Requirement,

    /*
     * Значение взято из приложенного документа.
     */
    SourceDocument,

    /*
     * Значение вычислено по другим исходным данным.
     */
    Derived,

    /*
     * Значение временно принято для запуска модели.
     */
    Provisional,

    /*
     * Значение взято из вспомогательной таблицы
     * или сторонней модели.
     */
    AuxiliaryTable
};

/*
 * Параметр может быть числом или текстовым значением.
 *
 * Текстовый вариант необходим, например, для формы носа.
 */
using ParameterValue = std::variant<double, std::string>;

/*
 * Описание происхождения одного параметра.
 */
struct ParameterRecord {
    /*
     * Стабильный путь параметра, например:
     *
     *     mass.massKg
     *     wing.areaM2
     *     body.noseShape
     */
    std::string path;

    ParameterValue value;

    /*
     * Единица измерения.
     *
     * Для безразмерных параметров используется "1".
     * Для текстовых параметров используется "-".
     */
    std::string unit;

    ParameterStatus status{ParameterStatus::Provisional};

    /*
     * Краткий идентификатор источника.
     */
    std::string source;

    /*
     * Пояснение к значению или допущению.
     */
    std::string note;
};

/*
 * Полный паспорт объекта.
 *
 * object содержит числовую модель для решателя.
 * parameters содержит происхождение каждого значения.
 */
struct ObjectPassport {
    ObjectModel object;
    std::vector<ParameterRecord> parameters;
};

/*
 * Создаёт паспорт первого абстрактного объекта.
 */
[[nodiscard]] ObjectPassport makeAbstract500UmpkPassport();

/*
 * Ищет параметр по стабильному пути.
 *
 * Возвращает nullptr, если параметр отсутствует.
 */
[[nodiscard]] const ParameterRecord* findParameter(
    const ObjectPassport& passport,
    const std::string& path
) noexcept;

/*
 * Проверяет числовую модель объекта.
 */
[[nodiscard]] ValidationIssues validate(
    const ObjectModel& object
);

/*
 * Проверяет паспорт вместе с происхождением параметров.
 */
[[nodiscard]] ValidationIssues validate(
    const ObjectPassport& passport
);

/*
 * Возвращает текстовое название статуса.
 */
[[nodiscard]] const char* parameterStatusName(
    ParameterStatus status
) noexcept;

/*
 * Возвращает текстовое название формы носа.
 */
[[nodiscard]] const char* noseShapeName(
    NoseShape shape
) noexcept;

} // namespace passive_flight