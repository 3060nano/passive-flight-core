#pragma once

#include "passive_flight/Types.hpp"

#include <string>
#include <vector>

namespace passive_flight {

    /*
     * Описание одной ошибки входных данных.
     */
    struct ValidationIssue {
        std::string field;
        std::string message;
    };

    using ValidationIssues = std::vector<ValidationIssue>;

    /*
     * Формирует начальное состояние горизонтального сброса:
     *
     *     t0      = 0;
     *     x0      = 0;
     *     H0      = заданная высота;
     *     V0      = заданная скорость;
     *     Theta0  = 0;
     *     theta0  = 0;
     *     omegaZ0 = 0;
     *     alpha0  = 0.
     *
     * При недопустимых исходных данных функция выбрасывает
     * std::invalid_argument.
     */
    [[nodiscard]] State makeHorizontalReleaseState(
        const ReleaseConditions& release
    );

    /*
     * Формирует краткий результат из полного результата моделирования.
     */
    [[nodiscard]] SimulationSummary summarize(
        const SimulationResult& result
    );

    /*
     * Проверяет параметры сброса.
     */
    [[nodiscard]] ValidationIssues validate(
        const ReleaseConditions& release
    );

    /*
     * Проверяет полный запрос на моделирование.
     */
    [[nodiscard]] ValidationIssues validate(
        const SimulationRequest& request
    );

    /*
     * Возвращает true, если список ошибок пуст.
     */
    [[nodiscard]] bool isValid(
        const ValidationIssues& issues
    ) noexcept;

    /*
     * Возвращает текстовое название причины остановки.
     *
     * Функция пригодится для журналирования, тестов
     * и последующего C API.
     */
    [[nodiscard]] const char* terminationReasonName(
        TerminationReason reason
    ) noexcept;

} // namespace passive_flight