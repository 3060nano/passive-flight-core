#include "passive_flight_simintech/SimInTechBlockApi.h"
#include "passive_flight_c_api/PassiveFlightCApi.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * ПОРЯДОК ВНЕШНИХ ПЕРЕМЕННЫХ SIMINTECH
 *
 * Входы:
 *
 * 0 — releaseAltitudeM;
 * 1 — releaseSpeedMps;
 * 2 — objectIndex.
 *
 * Выходы:
 *
 * 3  — downrangeM;
 * 4  — fallTimeS;
 * 5  — impactSpeedMps;
 * 6  — impactFlightPathAngleRad;
 * 7  — impactPitchAngleRad;
 * 8  — impactAngleOfAttackRad;
 * 9  — terminationReason;
 * 10 — resultCode.
 *
 * Имена input:N и out:N являются частью соглашения
 * загрузчика DLL SimInTech и связывают записи таблицы
 * с графическими портами блока.
 */
enum {
    PF_SIT_RELEASE_ALTITUDE_INDEX = 0,
    PF_SIT_RELEASE_SPEED_INDEX = 1,
    PF_SIT_OBJECT_INDEX = 2,

    PF_SIT_DOWNRANGE_INDEX = 3,
    PF_SIT_FALL_TIME_INDEX = 4,
    PF_SIT_IMPACT_SPEED_INDEX = 5,
    PF_SIT_IMPACT_PATH_ANGLE_INDEX = 6,
    PF_SIT_IMPACT_PITCH_ANGLE_INDEX = 7,
    PF_SIT_IMPACT_ATTACK_ANGLE_INDEX = 8,
    PF_SIT_TERMINATION_REASON_INDEX = 9,
    PF_SIT_RESULT_CODE_INDEX = 10,

    PF_SIT_OUTPUT_COUNT = 8,
    PF_SIT_INPUT_COUNT = 3,
    PF_SIT_EXTERNAL_VARIABLE_COUNT = 11
};

/*
 * Значения выходов по умолчанию.
 */
static double defaultDownrangeM =
    0.0;

static double defaultFallTimeS =
    0.0;

static double defaultImpactSpeedMps =
    0.0;

static double defaultImpactPathAngleRad =
    0.0;

static double defaultImpactPitchAngleRad =
    0.0;

static double defaultImpactAttackAngleRad =
    0.0;

static double defaultTerminationReason =
    (double) PF_TERMINATION_INVALID_INPUT;

static double defaultResultCode =
    (double) PF_RESULT_INVALID_INPUT;

/*
 * Значения входов по умолчанию.
 */
static double defaultReleaseAltitudeM =
    1000.0;

static double defaultReleaseSpeedMps =
    200.0;

static double defaultObjectIndex =
    0.0;

/*
 * Таблица внешних переменных.
 *
 * ВАЖНО:
 *
 * Имена записей должны иметь вид input:N и out:N.
 */
static ext_var_info_record externalVariables[
    PF_SIT_EXTERNAL_VARIABLE_COUNT
] = {
    {
        "input:0",
        "Release altitude, m",
        &defaultReleaseAltitudeM,
        vt_double,
        {1, 0, 0},
        PF_SIT_RELEASE_ALTITUDE_INDEX,
        dir_input,
        sizeof(double),
        0
    },
    {
        "input:1",
        "Release speed, m/s",
        &defaultReleaseSpeedMps,
        vt_double,
        {1, 0, 0},
        PF_SIT_RELEASE_SPEED_INDEX,
        dir_input,
        sizeof(double),
        0
    },
    {
        "input:2",
        "Object registry index",
        &defaultObjectIndex,
        vt_double,
        {1, 0, 0},
        PF_SIT_OBJECT_INDEX,
        dir_input,
        sizeof(double),
        0
    },
    {
        "out:0",
        "Calculated downrange, m",
        &defaultDownrangeM,
        vt_double,
        {1, 0, 0},
        PF_SIT_DOWNRANGE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:1",
        "Fall time, s",
        &defaultFallTimeS,
        vt_double,
        {1, 0, 0},
        PF_SIT_FALL_TIME_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:2",
        "Impact speed, m/s",
        &defaultImpactSpeedMps,
        vt_double,
        {1, 0, 0},
        PF_SIT_IMPACT_SPEED_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:3",
        "Impact flight path angle, rad",
        &defaultImpactPathAngleRad,
        vt_double,
        {1, 0, 0},
        PF_SIT_IMPACT_PATH_ANGLE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:4",
        "Impact pitch angle, rad",
        &defaultImpactPitchAngleRad,
        vt_double,
        {1, 0, 0},
        PF_SIT_IMPACT_PITCH_ANGLE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:5",
        "Impact angle of attack, rad",
        &defaultImpactAttackAngleRad,
        vt_double,
        {1, 0, 0},
        PF_SIT_IMPACT_ATTACK_ANGLE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:6",
        "Simulation termination reason",
        &defaultTerminationReason,
        vt_double,
        {1, 0, 0},
        PF_SIT_TERMINATION_REASON_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:7",
        "Passive Flight DLL result code",
        &defaultResultCode,
        vt_double,
        {1, 0, 0},
        PF_SIT_RESULT_CODE_INDEX,
        dir_out,
        sizeof(double),
        0
    }
};

/*
 * Локальные данные одного экземпляра блока.
 *
 * Кэш предотвращает повторный расчёт полной траектории
 * на каждом шаге SimInTech.
 */
typedef struct PFSimInTechLocalState {
    int initialized;

    double lastReleaseAltitudeM;
    double lastReleaseSpeedMps;
    double lastObjectIndex;

    double downrangeM;
    double fallTimeS;

    double impactSpeedMps;
    double impactFlightPathAngleRad;
    double impactPitchAngleRad;
    double impactAngleOfAttackRad;

    int32_t terminationReason;
    int32_t resultCode;
} PFSimInTechLocalState;

static TTaskInfoStruct taskInfo;

static const unsigned int schemeHash32 =
    0x50465334U;

static double* externalDouble(
    void** extVarsAddress,
    int index
) {
    if (extVarsAddress == NULL) {
        return NULL;
    }

    return (double*) extVarsAddress[index];
}

static void writeExternalDouble(
    void** extVarsAddress,
    int index,
    double value
) {
    double* destination =
        externalDouble(
            extVarsAddress,
            index
        );

    if (destination != NULL) {
        *destination = value;
    }
}

static int allExternalVariablesAreAvailable(
    void** extVarsAddress
) {
    int index;

    if (extVarsAddress == NULL) {
        return 0;
    }

    for (index = 0;
         index < PF_SIT_EXTERNAL_VARIABLE_COUNT;
         ++index) {
        if (extVarsAddress[index] == NULL) {
            return 0;
        }
    }

    return 1;
}

static void clearLocalResult(
    PFSimInTechLocalState* localState
) {
    if (localState == NULL) {
        return;
    }

    localState->downrangeM = 0.0;
    localState->fallTimeS = 0.0;

    localState->impactSpeedMps = 0.0;
    localState->impactFlightPathAngleRad = 0.0;
    localState->impactPitchAngleRad = 0.0;
    localState->impactAngleOfAttackRad = 0.0;

    localState->terminationReason =
        PF_TERMINATION_INVALID_INPUT;

    localState->resultCode =
        PF_RESULT_INVALID_INPUT;
}

static void writeLocalResultToPorts(
    void** extVarsAddress,
    const PFSimInTechLocalState* localState
) {
    if (localState == NULL) {
        return;
    }

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_DOWNRANGE_INDEX,
        localState->downrangeM
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_FALL_TIME_INDEX,
        localState->fallTimeS
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_IMPACT_SPEED_INDEX,
        localState->impactSpeedMps
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_IMPACT_PATH_ANGLE_INDEX,
        localState->impactFlightPathAngleRad
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_IMPACT_PITCH_ANGLE_INDEX,
        localState->impactPitchAngleRad
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_IMPACT_ATTACK_ANGLE_INDEX,
        localState->impactAngleOfAttackRad
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_TERMINATION_REASON_INDEX,
        (double)
            localState->terminationReason
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_RESULT_CODE_INDEX,
        (double)
            localState->resultCode
    );
}

static int objectIndexToId(
    double objectIndexValue,
    char* objectId,
    uint64_t objectIdCapacity
) {
    uint64_t objectIndex;
    uint64_t objectCount;
    uint64_t requiredSize = 0;

    if (objectId == NULL ||
        objectIdCapacity == 0) {
        return PF_RESULT_NULL_ARGUMENT;
    }

    objectId[0] = '\0';

    if (!isfinite(objectIndexValue) ||
        objectIndexValue < 0.0 ||
        floor(objectIndexValue) !=
            objectIndexValue) {
        return PF_RESULT_INVALID_INPUT;
    }

    objectIndex =
        (uint64_t) objectIndexValue;

    objectCount =
        pfGetObjectCount();

    if (objectIndex >= objectCount) {
        return PF_RESULT_INDEX_OUT_OF_RANGE;
    }

    return pfGetObjectId(
        objectIndex,
        objectId,
        objectIdCapacity,
        &requiredSize
    );
}

static int inputsHaveChanged(
    const PFSimInTechLocalState* localState,
    double releaseAltitudeM,
    double releaseSpeedMps,
    double objectIndex
) {
    if (localState == NULL ||
        !localState->initialized) {
        return 1;
    }

    return
        localState->lastReleaseAltitudeM !=
            releaseAltitudeM ||
        localState->lastReleaseSpeedMps !=
            releaseSpeedMps ||
        localState->lastObjectIndex !=
            objectIndex;
}

static void calculateIfNecessary(
    void** extVarsAddress,
    PFSimInTechLocalState* localState
) {
    double releaseAltitudeM;
    double releaseSpeedMps;
    double objectIndex;

    char objectId[512];

    int objectResult;

    double* releaseAltitudeAddress;
    double* releaseSpeedAddress;
    double* objectIndexAddress;

    if (localState == NULL ||
        extVarsAddress == NULL) {
        return;
    }

    releaseAltitudeAddress =
        externalDouble(
            extVarsAddress,
            PF_SIT_RELEASE_ALTITUDE_INDEX
        );

    releaseSpeedAddress =
        externalDouble(
            extVarsAddress,
            PF_SIT_RELEASE_SPEED_INDEX
        );

    objectIndexAddress =
        externalDouble(
            extVarsAddress,
            PF_SIT_OBJECT_INDEX
        );

    if (releaseAltitudeAddress == NULL ||
        releaseSpeedAddress == NULL ||
        objectIndexAddress == NULL) {
        clearLocalResult(
            localState
        );

        localState->resultCode =
            PF_RESULT_NULL_ARGUMENT;

        return;
    }

    releaseAltitudeM =
        *releaseAltitudeAddress;

    releaseSpeedMps =
        *releaseSpeedAddress;

    objectIndex =
        *objectIndexAddress;

    if (!inputsHaveChanged(
            localState,
            releaseAltitudeM,
            releaseSpeedMps,
            objectIndex
        )) {
        return;
    }

    localState->lastReleaseAltitudeM =
        releaseAltitudeM;

    localState->lastReleaseSpeedMps =
        releaseSpeedMps;

    localState->lastObjectIndex =
        objectIndex;

    localState->initialized = 1;

    clearLocalResult(
        localState
    );

    objectResult =
        objectIndexToId(
            objectIndex,
            objectId,
            sizeof(objectId)
        );

    if (objectResult != PF_RESULT_OK) {
        localState->resultCode =
            objectResult;

        return;
    }

    localState->resultCode =
        pfSimInTechCalculate(
            objectId,
            releaseAltitudeM,
            releaseSpeedMps,

            &localState->downrangeM,
            &localState->fallTimeS,

            &localState->impactSpeedMps,
            &localState
                ->impactFlightPathAngleRad,
            &localState
                ->impactPitchAngleRad,
            &localState
                ->impactAngleOfAttackRad,

            &localState->terminationReason
        );
}

int PF_SIT_CALL
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
    unsigned int* schemeHash,
    char* algorithmName,
    void** algorithmObjectId
) {
    (void) algorithmName;

    if (nExtVars == NULL ||
        nDinVars == NULL ||
        nAlgVars == NULL ||
        nStateVars == NULL ||
        nConsts == NULL ||

        sizeOfStateVars == NULL ||
        sizeOfConsts == NULL ||
        sizeOfLocalVars == NULL ||

        dinVarsDimension == NULL ||
        algVarsDimension == NULL ||

        extVarsInfo == NULL ||
        dinVarsInfo == NULL ||
        algVarsInfo == NULL ||
        stateVarsInfo == NULL ||
        constInfo == NULL ||

        schemeHash == NULL) {
        return r_Fail;
    }

    *nExtVars =
        PF_SIT_EXTERNAL_VARIABLE_COUNT;

    *nDinVars = 0;
    *nAlgVars = 0;
    *nStateVars = 0;
    *nConsts = 0;

    *sizeOfStateVars = 0;
    *sizeOfConsts = 0;

    *sizeOfLocalVars =
        (int) sizeof(
            PFSimInTechLocalState
        );

    *dinVarsDimension = 0;
    *algVarsDimension = 0;

    *extVarsInfo =
        externalVariables;

    *dinVarsInfo = NULL;
    *algVarsInfo = NULL;
    *stateVarsInfo = NULL;
    *constInfo = NULL;

    *schemeHash =
        schemeHash32;

    if (algorithmObjectId != NULL) {
        *algorithmObjectId =
            NULL;
    }

    memset(
        &taskInfo,
        0,
        sizeof(taskInfo)
    );

    taskInfo.AbsErr = 1.0e-6;
    taskInfo.RelErr = 1.0e-4;

    taskInfo.Hmin = 0.001;
    taskInfo.Hmax = 0.001;
    taskInfo.StartStep = 0.001;

    taskInfo.EndTime = 300.0;
    taskInfo.MaxLoopIt = 10;

    taskInfo.TaskDAELibraryName =
        "0";

    if (solverData != NULL) {
        solverData->TaskInfo =
            &taskInfo;
    }

    return r_Success;
}

int PF_SIT_CALL
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
) {
    PFSimInTechLocalState* localState =
        (PFSimInTechLocalState*) locals;

    (void) step;
    (void) modelTime;
    (void) dinVars;
    (void) derivatives;
    (void) algVars;
    (void) algFunctions;
    (void) stateVars;
    (void) consts;
    (void) solverData;
    (void) algorithmObjectId;

    if (localState == NULL) {
        return r_Fail;
    }

    memset(
        localState,
        0,
        sizeof(*localState)
    );

    clearLocalResult(
        localState
    );

    writeLocalResultToPorts(
        extVarsAddress,
        localState
    );

    return r_Success;
}

int PF_SIT_CALL
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
) {
    PFSimInTechLocalState* localState =
        (PFSimInTechLocalState*) locals;

    (void) step;
    (void) modelTime;
    (void) dinVars;
    (void) derivatives;
    (void) algVars;
    (void) algFunctions;
    (void) stateVars;
    (void) consts;
    (void) solverData;
    (void) algorithmObjectId;

    if (localState == NULL ||
        !allExternalVariablesAreAvailable(
            extVarsAddress
        )) {
        return r_Fail;
    }

    switch (action) {
        case f_Stop:
        case f_GetDeri:
        case f_GetAlgFun:
            return r_Success;

        default:
            calculateIfNecessary(
                extVarsAddress,
                localState
            );

            writeLocalResultToPorts(
                extVarsAddress,
                localState
            );

            return r_Success;
    }
}

int PF_SIT_CALL
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
) {
    (void) action;
    (void) step;
    (void) modelTime;
    (void) extVarsAddress;
    (void) dinVars;
    (void) derivatives;
    (void) algVars;
    (void) algFunctions;
    (void) stateVars;
    (void) consts;
    (void) locals;
    (void) solverData;
    (void) algorithmObjectId;

    return r_Success;
}

int PF_SIT_CALL
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
) {
    (void) action;
    (void) step;
    (void) modelTime;
    (void) extVarsAddress;
    (void) dinVars;
    (void) derivatives;
    (void) algVars;
    (void) algFunctions;
    (void) stateVars;
    (void) consts;
    (void) locals;
    (void) solverData;
    (void) algorithmObjectId;

    return r_Success;
}

