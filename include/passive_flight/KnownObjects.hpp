#pragma once

#include "passive_flight/ObjectModel.hpp"

namespace passive_flight {

/**
 * Создаёт контрольную числовую модель авиабомбы ФАБ-1500Т
 * с готовыми массовыми и аэродинамическими характеристиками.
 *
 * Источник:
 * А.Г. Постников, В.С. Чуйко,
 * "Методы решения прикладных задач внешней баллистики",
 * 1979, таблица 1.1.
 *
 * Модель предназначена прежде всего для валидации
 * уравнений движения passive-flight-core.
 *
 * На этом этапе объект ещё не добавляется в ObjectRegistry:
 * сначала проверяется непосредственно связка
 *
 *     табличная аэродинамика -> динамика -> траектория.
 */
[[nodiscard]]
ObjectModel makeFab1500TPostnikovModel();

/**
 * Возвращает табличные аэродинамические характеристики
 * ФАБ-1500Т из того же источника.
 */
[[nodiscard]]
TabulatedAerodynamicData
makeFab1500TPostnikovAerodynamicData();

} // namespace passive_flight
