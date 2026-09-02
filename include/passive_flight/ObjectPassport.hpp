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
    Requirement,
    SourceDocument,
    Derived,
    Provisional,
    AuxiliaryTable
};

/*
 * Параметр может быть числом или текстовым значением.
 */
using ParameterValue = std::variant<double, std::string>;

/*
 * Описание происхождения одного параметра.
 */
struct ParameterRecord {
    std::string path;
    ParameterValue value;
    std::string unit;
    ParameterStatus status{ParameterStatus::Provisional};
    std::string source;
    std::string note;
};

/*
 * Полный паспорт объекта.
 *
 * object содержит числовую модель для решателя.
 * parameters содержит происхождение основных
 * паспортных величин и аэродинамического набора.
 */
struct ObjectPassport {
    ObjectModel object;
    std::vector<ParameterRecord> parameters;
};

/*
 * Создаёт паспорт абстрактного крылатого объекта.
 */
[[nodiscard]]
ObjectPassport makeAbstract500UmpkPassport();

/*
 * Создаёт полный контрольный паспорт ФАБ-1500Т
 * с готовыми табличными аэродинамическими
 * характеристиками.
 */
[[nodiscard]]
ObjectPassport makeFab1500TPostnikovPassport();

/*
 * Ищет параметр по стабильному пути.
 */
[[nodiscard]]
const ParameterRecord* findParameter(
    const ObjectPassport& passport,
    const std::string& path
) noexcept;

/*
 * Проверяет числовую модель объекта.
 *
 * Набор обязательных полей зависит от
 * aerodynamicModelType.
 */
[[nodiscard]]
ValidationIssues validate(
    const ObjectModel& object
);

/*
 * Проверяет паспорт вместе с происхождением
 * основных параметров.
 */
[[nodiscard]]
ValidationIssues validate(
    const ObjectPassport& passport
);

[[nodiscard]]
const char* parameterStatusName(
    ParameterStatus status
) noexcept;

[[nodiscard]]
const char* noseShapeName(
    NoseShape shape
) noexcept;

[[nodiscard]]
const char* aerodynamicModelTypeName(
    AerodynamicModelType type
) noexcept;

} // namespace passive_flight
