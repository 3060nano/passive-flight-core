#include "passive_flight_simintech/SimInTechBlockApi.h"
#include "passive_flight_c_api/PassiveFlightCApi.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*
 * Внешние переменные блока траектории.
 *
 * Входы:
 *
 *   input:0 — высота сброса, м;
 *   input:1 — скорость сброса, м/с;
 *   input:2 — индекс объекта в реестре.
 *
 * Выходы:
 *
 *   out:0  — текущее время траектории, с;
 *   out:1  — текущий относ, м;
 *   out:2  — текущая высота, м;
 *   out:3  — текущая скорость, м/с;
 *   out:4  — угол наклона траектории, рад;
 *   out:5  — угол тангажа, рад;
 *   out:6  — угловая скорость тангажа, рад/с;
 *   out:7  — угол атаки, рад;
 *   out:8  — полное время падения, с;
 *   out:9  — признак окончания траектории;
 *   out:10 — код результата C API.
 */
enum PFSimInTechTrajectoryVariableIndex {
    PF_TRAJECTORY_INPUT_ALTITUDE = 0,
    PF_TRAJECTORY_INPUT_SPEED = 1,
    PF_TRAJECTORY_INPUT_OBJECT_INDEX = 2,

    PF_TRAJECTORY_OUTPUT_TIME = 3,
    PF_TRAJECTORY_OUTPUT_DOWNRANGE = 4,
    PF_TRAJECTORY_OUTPUT_ALTITUDE = 5,
    PF_TRAJECTORY_OUTPUT_SPEED = 6,
    PF_TRAJECTORY_OUTPUT_PATH_ANGLE = 7,
    PF_TRAJECTORY_OUTPUT_PITCH_ANGLE = 8,
    PF_TRAJECTORY_OUTPUT_PITCH_RATE = 9,
    PF_TRAJECTORY_OUTPUT_ATTACK_ANGLE = 10,
    PF_TRAJECTORY_OUTPUT_FALL_TIME = 11,
    PF_TRAJECTORY_OUTPUT_FINISHED = 12,
    PF_TRAJECTORY_OUTPUT_RESULT_CODE = 13,

    PF_TRAJECTORY_INPUT_COUNT = 3,
    PF_TRAJECTORY_OUTPUT_COUNT = 11,
    PF_TRAJECTORY_EXTERNAL_VARIABLE_COUNT = 14
};

static double defaultReleaseAltitudeM = 1000.0;
static double defaultReleaseSpeedMps = 200.0;
static double defaultObjectIndex = 0.0;

static double defaultTrajectoryTimeS = 0.0;
static double defaultDownrangeM = 0.0;
static double defaultAltitudeM = 0.0;
static double defaultSpeedMps = 0.0;
static double defaultFlightPathAngleRad = 0.0;
static double defaultPitchAngleRad = 0.0;
static double defaultPitchRateRadps = 0.0;
static double defaultAngleOfAttackRad = 0.0;
static double defaultFallTimeS = 0.0;
static double defaultFinished = 0.0;
static double defaultResultCode =
    (double) PF_RESULT_INVALID_INPUT;

static ext_var_info_record externalVariables[
    PF_TRAJECTORY_EXTERNAL_VARIABLE_COUNT
] = {
    {
        "input:0",
        "Release altitude, m",
        &defaultReleaseAltitudeM,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_INPUT_ALTITUDE,
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
        PF_TRAJECTORY_INPUT_SPEED,
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
        PF_TRAJECTORY_INPUT_OBJECT_INDEX,
        dir_input,
        sizeof(double),
        0
    },
    {
        "out:0",
        "Current trajectory time, s",
        &defaultTrajectoryTimeS,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_TIME,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:1",
        "Current downrange, m",
        &defaultDownrangeM,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_DOWNRANGE,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:2",
        "Current altitude, m",
        &defaultAltitudeM,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_ALTITUDE,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:3",
        "Current speed, m/s",
        &defaultSpeedMps,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_SPEED,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:4",
        "Current flight path angle, rad",
        &defaultFlightPathAngleRad,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_PATH_ANGLE,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:5",
        "Current pitch angle, rad",
        &defaultPitchAngleRad,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_PITCH_ANGLE,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:6",
        "Current pitch rate, rad/s",
        &defaultPitchRateRadps,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_PITCH_RATE,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:7",
        "Current angle of attack, rad",
        &defaultAngleOfAttackRad,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_ATTACK_ANGLE,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:8",
        "Total fall time, s",
        &defaultFallTimeS,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_FALL_TIME,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:9",
        "Trajectory finished flag",
        &defaultFinished,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_FINISHED,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:10",
        "Passive Flight C API result code",
        &defaultResultCode,
        vt_double,
        {1, 0, 0},
        PF_TRAJECTORY_OUTPUT_RESULT_CODE,
        dir_out,
        sizeof(double),
        0
    }
};

typedef struct PFSimInTechTrajectoryLocalState {
    int initialized;

    double lastReleaseAltitudeM;
    double lastReleaseSpeedMps;
    double lastObjectIndex;

    PFSimulationOutput summary;
    PFTrajectoryPoint* points;
    uint64_t pointCount;

    int32_t resultCode;
} PFSimInTechTrajectoryLocalState;

static TTaskInfoStruct taskInfo;

static const unsigned int schemeHash32 =
    0x50465431U;

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
         index <
            PF_TRAJECTORY_EXTERNAL_VARIABLE_COUNT;
         ++index) {
        if (extVarsAddress[index] == NULL) {
            return 0;
        }
    }

    return 1;
}

static void clearSummary(
    PFSimulationOutput* summary
) {
    if (summary == NULL) {
        return;
    }

    memset(
        summary,
        0,
        sizeof(*summary)
    );

    summary->terminationReason =
        PF_TERMINATION_INVALID_INPUT;
}

static void releaseTrajectory(
    PFSimInTechTrajectoryLocalState* localState
) {
    if (localState == NULL) {
        return;
    }

    free(localState->points);
    localState->points = NULL;
    localState->pointCount = 0;

    clearSummary(
        &localState->summary
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

    objectCount = pfGetObjectCount();

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
    const PFSimInTechTrajectoryLocalState* localState,
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

static void calculateTrajectoryIfNecessary(
    void** extVarsAddress,
    PFSimInTechTrajectoryLocalState* localState
) {
    double* altitudeAddress;
    double* speedAddress;
    double* objectIndexAddress;

    double releaseAltitudeM;
    double releaseSpeedMps;
    double objectIndex;

    char objectId[512];

    PFSimulationInput input;
    uint64_t requiredPointCount = 0;
    uint64_t writtenPointCount = 0;

    int32_t result;

    if (extVarsAddress == NULL ||
        localState == NULL) {
        return;
    }

    altitudeAddress =
        externalDouble(
            extVarsAddress,
            PF_TRAJECTORY_INPUT_ALTITUDE
        );

    speedAddress =
        externalDouble(
            extVarsAddress,
            PF_TRAJECTORY_INPUT_SPEED
        );

    objectIndexAddress =
        externalDouble(
            extVarsAddress,
            PF_TRAJECTORY_INPUT_OBJECT_INDEX
        );

    if (altitudeAddress == NULL ||
        speedAddress == NULL ||
        objectIndexAddress == NULL) {
        releaseTrajectory(localState);
        localState->resultCode =
            PF_RESULT_NULL_ARGUMENT;
        return;
    }

    releaseAltitudeM = *altitudeAddress;
    releaseSpeedMps = *speedAddress;
    objectIndex = *objectIndexAddress;

    if (!inputsHaveChanged(
            localState,
            releaseAltitudeM,
            releaseSpeedMps,
            objectIndex
        )) {
        return;
    }

    releaseTrajectory(localState);

    localState->lastReleaseAltitudeM =
        releaseAltitudeM;
    localState->lastReleaseSpeedMps =
        releaseSpeedMps;
    localState->lastObjectIndex =
        objectIndex;
    localState->initialized = 1;
    localState->resultCode =
        PF_RESULT_INVALID_INPUT;

    result = objectIndexToId(
        objectIndex,
        objectId,
        sizeof(objectId)
    );

    if (result != PF_RESULT_OK) {
        localState->resultCode = result;
        return;
    }

    input.objectId = objectId;
    input.releaseAltitudeM = releaseAltitudeM;
    input.releaseSpeedMps = releaseSpeedMps;

    result = pfCalculateTrajectory(
        &input,
        &localState->summary,
        NULL,
        0,
        &requiredPointCount,
        &writtenPointCount
    );

    if (result != PF_RESULT_BUFFER_TOO_SMALL ||
        requiredPointCount == 0) {
        localState->resultCode = result;
        return;
    }

    if (requiredPointCount >
        SIZE_MAX / sizeof(PFTrajectoryPoint)) {
        localState->resultCode =
            PF_RESULT_INTERNAL_ERROR;
        return;
    }

    localState->points =
        (PFTrajectoryPoint*) calloc(
            (size_t) requiredPointCount,
            sizeof(PFTrajectoryPoint)
        );

    if (localState->points == NULL) {
        localState->resultCode =
            PF_RESULT_INTERNAL_ERROR;
        return;
    }

    result = pfCalculateTrajectory(
        &input,
        &localState->summary,
        localState->points,
        requiredPointCount,
        &requiredPointCount,
        &writtenPointCount
    );

    if (result != PF_RESULT_OK ||
        writtenPointCount == 0) {
        releaseTrajectory(localState);
        localState->resultCode = result;
        return;
    }

    localState->pointCount =
        writtenPointCount;
    localState->resultCode =
        PF_RESULT_OK;
}

static double interpolateValue(
    double first,
    double second,
    double ratio
) {
    return first +
        (second - first) * ratio;
}

static PFTrajectoryPoint trajectoryPointAtTime(
    const PFSimInTechTrajectoryLocalState* localState,
    double requestedTimeS
) {
    uint64_t lowerIndex;
    uint64_t upperIndex;

    const PFTrajectoryPoint* first;
    const PFTrajectoryPoint* second;

    double intervalS;
    double ratio;

    PFTrajectoryPoint result;

    memset(&result, 0, sizeof(result));

    if (localState == NULL ||
        localState->points == NULL ||
        localState->pointCount == 0) {
        return result;
    }

    if (requestedTimeS <=
        localState->points[0].timeS) {
        return localState->points[0];
    }

    if (requestedTimeS >=
        localState->points[
            localState->pointCount - 1
        ].timeS) {
        return localState->points[
            localState->pointCount - 1
        ];
    }

    lowerIndex = 0;
    upperIndex =
        localState->pointCount - 1;

    while (upperIndex - lowerIndex > 1) {
        const uint64_t middleIndex =
            lowerIndex +
            (upperIndex - lowerIndex) / 2;

        if (localState->points[
                middleIndex
            ].timeS <= requestedTimeS) {
            lowerIndex = middleIndex;
        } else {
            upperIndex = middleIndex;
        }
    }

    first = &localState->points[lowerIndex];
    second = &localState->points[upperIndex];

    intervalS =
        second->timeS - first->timeS;

    if (intervalS <= 0.0) {
        return *first;
    }

    ratio =
        (requestedTimeS - first->timeS) /
        intervalS;

    result.timeS = requestedTimeS;
    result.downrangeM = interpolateValue(
        first->downrangeM,
        second->downrangeM,
        ratio
    );
    result.altitudeM = interpolateValue(
        first->altitudeM,
        second->altitudeM,
        ratio
    );
    result.speedMps = interpolateValue(
        first->speedMps,
        second->speedMps,
        ratio
    );
    result.flightPathAngleRad = interpolateValue(
        first->flightPathAngleRad,
        second->flightPathAngleRad,
        ratio
    );
    result.pitchAngleRad = interpolateValue(
        first->pitchAngleRad,
        second->pitchAngleRad,
        ratio
    );
    result.pitchRateRadps = interpolateValue(
        first->pitchRateRadps,
        second->pitchRateRadps,
        ratio
    );
    result.angleOfAttackRad = interpolateValue(
        first->angleOfAttackRad,
        second->angleOfAttackRad,
        ratio
    );
    result.mach = interpolateValue(
        first->mach,
        second->mach,
        ratio
    );
    result.dynamicPressurePa = interpolateValue(
        first->dynamicPressurePa,
        second->dynamicPressurePa,
        ratio
    );
    result.cx = interpolateValue(
        first->cx,
        second->cx,
        ratio
    );
    result.cy = interpolateValue(
        first->cy,
        second->cy,
        ratio
    );
    result.mz = interpolateValue(
        first->mz,
        second->mz,
        ratio
    );

    return result;
}

static void clearOutputPorts(
    void** extVarsAddress,
    int32_t resultCode
) {
    int index;

    for (index = PF_TRAJECTORY_OUTPUT_TIME;
         index <= PF_TRAJECTORY_OUTPUT_FINISHED;
         ++index) {
        writeExternalDouble(
            extVarsAddress,
            index,
            0.0
        );
    }

    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_RESULT_CODE,
        (double) resultCode
    );
}

static void writeTrajectoryOutputs(
    void** extVarsAddress,
    const PFSimInTechTrajectoryLocalState* localState,
    double modelTime
) {
    double requestedTimeS;
    double finalTimeS;
    double finished;

    PFTrajectoryPoint point;

    if (localState == NULL ||
        localState->resultCode != PF_RESULT_OK ||
        localState->points == NULL ||
        localState->pointCount == 0) {
        clearOutputPorts(
            extVarsAddress,
            localState == NULL
                ? PF_RESULT_NULL_ARGUMENT
                : localState->resultCode
        );
        return;
    }

    requestedTimeS =
        isfinite(modelTime) && modelTime > 0.0
            ? modelTime
            : 0.0;

    finalTimeS =
        localState->points[
            localState->pointCount - 1
        ].timeS;

    finished =
        requestedTimeS >= finalTimeS
            ? 1.0
            : 0.0;

    point = trajectoryPointAtTime(
        localState,
        requestedTimeS
    );

    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_TIME,
        point.timeS
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_DOWNRANGE,
        point.downrangeM
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_ALTITUDE,
        point.altitudeM
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_SPEED,
        point.speedMps
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_PATH_ANGLE,
        point.flightPathAngleRad
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_PITCH_ANGLE,
        point.pitchAngleRad
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_PITCH_RATE,
        point.pitchRateRadps
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_ATTACK_ANGLE,
        point.angleOfAttackRad
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_FALL_TIME,
        localState->summary.fallTimeS
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_FINISHED,
        finished
    );
    writeExternalDouble(
        extVarsAddress,
        PF_TRAJECTORY_OUTPUT_RESULT_CODE,
        (double) localState->resultCode
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
        PF_TRAJECTORY_EXTERNAL_VARIABLE_COUNT;
    *nDinVars = 0;
    *nAlgVars = 0;
    *nStateVars = 0;
    *nConsts = 0;

    *sizeOfStateVars = 0;
    *sizeOfConsts = 0;
    *sizeOfLocalVars =
        (int) sizeof(
            PFSimInTechTrajectoryLocalState
        );

    *dinVarsDimension = 0;
    *algVarsDimension = 0;

    *extVarsInfo = externalVariables;
    *dinVarsInfo = NULL;
    *algVarsInfo = NULL;
    *stateVarsInfo = NULL;
    *constInfo = NULL;

    *schemeHash = schemeHash32;

    if (algorithmObjectId != NULL) {
        *algorithmObjectId = NULL;
    }

    memset(&taskInfo, 0, sizeof(taskInfo));

    taskInfo.AbsErr = 1.0e-6;
    taskInfo.RelErr = 1.0e-4;
    taskInfo.Hmin = 0.001;
    taskInfo.Hmax = 0.001;
    taskInfo.StartStep = 0.001;
    taskInfo.EndTime = 300.0;
    taskInfo.MaxLoopIt = 10;
    taskInfo.TaskDAELibraryName = "0";

    if (solverData != NULL) {
        solverData->TaskInfo = &taskInfo;
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
    PFSimInTechTrajectoryLocalState* localState =
        (PFSimInTechTrajectoryLocalState*) locals;

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

    memset(localState, 0, sizeof(*localState));
    clearSummary(&localState->summary);
    localState->resultCode =
        PF_RESULT_INVALID_INPUT;

    clearOutputPorts(
        extVarsAddress,
        localState->resultCode
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
    PFSimInTechTrajectoryLocalState* localState =
        (PFSimInTechTrajectoryLocalState*) locals;

    (void) step;
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

    if (action == f_Stop) {
        releaseTrajectory(localState);
        localState->initialized = 0;
        return r_Success;
    }

    if (action == f_GetDeri ||
        action == f_GetAlgFun) {
        return r_Success;
    }

    calculateTrajectoryIfNecessary(
        extVarsAddress,
        localState
    );

    writeTrajectoryOutputs(
        extVarsAddress,
        localState,
        modelTime
    );

    return r_Success;
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
