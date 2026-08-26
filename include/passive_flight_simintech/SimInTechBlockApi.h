#pragma once

/*
 * Эти стандартные заголовки должны подключаться
 * раньше служебного c_types.h из SimInTech.
 *
 * c_types.h использует:
 *
 * - uint64_t;
 * - uintptr_t;
 * - intptr_t;
 * - size_t.
 *
 * Но самостоятельно необходимые заголовки
 * официальный файл не подключает.
 */
#include <stddef.h>
#include <stdint.h>

#include "c_types.h"

#if defined(_WIN32)
    #if defined(PF_SIT_BLOCK_EXPORTS)
        #define PF_SIT_API __declspec(dllexport)
    #else
        #define PF_SIT_API __declspec(dllimport)
    #endif

    #define PF_SIT_CALL __stdcall
#else
    #define PF_SIT_API
    #define PF_SIT_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Нативный интерфейс расчётного модуля SimInTech.
 *
 * Сигнатуры функций соответствуют шаблону
 * MinGW_DLL из поставки SimInTech.
 */

PF_SIT_API int PF_SIT_CALL
INFO_FUNC(
    int* nExtVars,
    int* nDinVars,
    int* nAlgVars,
    int* nStateVars,
    int* nConsts,

    int* sizeOfStateVars,
    int* sizeOfConsts,
    int* sizeOfLocalVars,

    int* dinVarsDimension,
    int* algVarsDimension,

    void** extVarsInfo,
    void** dinVarsInfo,
    void** algVarsInfo,
    void** stateVarsInfo,
    void** constInfo,

    solver_struct* solverData,
    unsigned int* schemeHash32,
    char* algorithmName,
    void** algorithmObjectId
);

PF_SIT_API int PF_SIT_CALL
INIT_FUNC(
    double step,
    double modelTime,

    void** extVarsAddress,

    double* dinVars,
    double* derivatives,
    double* algVars,
    double* algFunctions,

    void* stateVars,
    void* consts,
    void* locals,

    solver_struct* solverData,
    void* algorithmObjectId
);

PF_SIT_API int PF_SIT_CALL
RUN_FUNC(
    int action,
    double step,
    double modelTime,

    void** extVarsAddress,

    double* dinVars,
    double* derivatives,
    double* algVars,
    double* algFunctions,

    void* stateVars,
    void* consts,
    void* locals,

    solver_struct* solverData,
    void* algorithmObjectId
);

PF_SIT_API int PF_SIT_CALL
STATE_FUNC(
    int action,
    double step,
    double modelTime,

    void** extVarsAddress,

    double* dinVars,
    double* derivatives,
    double* algVars,
    double* algFunctions,

    void* stateVars,
    void* consts,
    void* locals,

    solver_struct* solverData,
    void* algorithmObjectId
);

PF_SIT_API int PF_SIT_CALL
CONTUR_FUNC(
    int action,
    double step,
    double modelTime,

    void** extVarsAddress,

    double* dinVars,
    double* derivatives,
    double* algVars,
    double* algFunctions,

    void* stateVars,
    void* consts,
    void* locals,

    solver_struct* solverData,
    void* algorithmObjectId
);

#ifdef __cplusplus
}
#endif