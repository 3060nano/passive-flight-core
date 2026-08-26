#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
    #if defined(PF_C_API_EXPORTS)
        #define PF_API __declspec(dllexport)
    #else
        #define PF_API __declspec(dllimport)
    #endif

    #define PF_CALL __cdecl
#else
    #define PF_API __attribute__((visibility("default")))
    #define PF_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Коды завершения функций C API.
 */
typedef enum PFResultCode {
    PF_RESULT_OK = 0,

    PF_RESULT_NULL_ARGUMENT = 1,
    PF_RESULT_INVALID_INPUT = 2,
    PF_RESULT_OBJECT_NOT_FOUND = 3,
    PF_RESULT_BUFFER_TOO_SMALL = 4,
    PF_RESULT_INDEX_OUT_OF_RANGE = 5,
    PF_RESULT_SIMULATION_FAILED = 6,
    PF_RESULT_INTERNAL_ERROR = 7
} PFResultCode;

/*
 * Причины завершения моделирования.
 *
 * Значения не зависят от внутреннего C++ enum.
 */
typedef enum PFTerminationReason {
    PF_TERMINATION_GROUND_REACHED = 0,
    PF_TERMINATION_MAXIMUM_TIME_REACHED = 1,
    PF_TERMINATION_MAXIMUM_STEPS_REACHED = 2,
    PF_TERMINATION_INVALID_INPUT = 3,
    PF_TERMINATION_INVALID_STATE = 4
} PFTerminationReason;

/*
 * Входные параметры расчёта.
 *
 * objectId должен указывать на строку UTF-8,
 * завершённую нулевым символом.
 */
typedef struct PFSimulationInput {
    const char* objectId;

    double releaseAltitudeM;
    double releaseSpeedMps;
} PFSimulationInput;

/*
 * Краткий результат расчёта.
 */
typedef struct PFSimulationOutput {
    double downrangeM;
    double fallTimeS;

    double impactSpeedMps;
    double impactFlightPathAngleRad;
    double impactPitchAngleRad;
    double impactAngleOfAttackRad;

    int32_t terminationReason;
} PFSimulationOutput;

/*
 * Одна точка траектории.
 */
typedef struct PFTrajectoryPoint {
    double timeS;

    double downrangeM;
    double altitudeM;
    double speedMps;

    double flightPathAngleRad;
    double pitchAngleRad;
    double pitchRateRadps;
    double angleOfAttackRad;

    double mach;
    double dynamicPressurePa;

    double cx;
    double cy;
    double mz;
} PFTrajectoryPoint;

/*
 * Возвращает версию C API.
 *
 * Указатель существует на протяжении всей работы DLL.
 * Освобождать память не нужно.
 */
PF_API const char* PF_CALL
pfGetApiVersion(void);

/*
 * Возвращает текстовое имя кода результата.
 *
 * Освобождать память не нужно.
 */
PF_API const char* PF_CALL
pfGetResultCodeName(
    int32_t resultCode
);

/*
 * Количество зарегистрированных объектов.
 */
PF_API uint64_t PF_CALL
pfGetObjectCount(void);

/*
 * Получает идентификатор объекта по индексу.
 *
 * requiredSize получает необходимый размер буфера
 * с учётом завершающего нулевого символа.
 *
 * Для определения размера можно вызвать:
 *
 * buffer = NULL;
 * bufferSize = 0.
 *
 * В этом случае функция вернёт
 * PF_RESULT_BUFFER_TOO_SMALL.
 */
PF_API int32_t PF_CALL
pfGetObjectId(
    uint64_t objectIndex,
    char* buffer,
    uint64_t bufferSize,
    uint64_t* requiredSize
);

/*
 * Получает отображаемое название объекта в UTF-8.
 */
PF_API int32_t PF_CALL
pfGetObjectDisplayName(
    uint64_t objectIndex,
    char* buffer,
    uint64_t bufferSize,
    uint64_t* requiredSize
);

/*
 * Выполняет расчёт без сохранения полной истории.
 *
 * Основные выходы:
 *
 * - downrangeM;
 * - fallTimeS;
 * - impactSpeedMps;
 * - углы встречи.
 */
PF_API int32_t PF_CALL
pfCalculate(
    const PFSimulationInput* input,
    PFSimulationOutput* output
);

/*
 * Выполняет расчёт с сохранением траектории.
 *
 * Рекомендуется вызывать функцию два раза.
 *
 * Первый вызов:
 *
 * points = NULL;
 * pointCapacity = 0.
 *
 * Функция заполнит requiredPointCount.
 *
 * Второй вызов:
 *
 * передаётся массив необходимого размера.
 *
 * Точки сохраняются с интервалом около 0,1 с.
 */
PF_API int32_t PF_CALL
pfCalculateTrajectory(
    const PFSimulationInput* input,
    PFSimulationOutput* output,

    PFTrajectoryPoint* points,
    uint64_t pointCapacity,

    uint64_t* requiredPointCount,
    uint64_t* writtenPointCount
);

#ifdef __cplusplus
}
#endif