#include "passive_flight_simintech/SimInTechBlockApi.h"

#include "passive_flight_c_api/PassiveFlightCApi.h"
#include "passive_flight_c_api/PerturbedTrajectoryCApi.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*
 * Отдельный SimInTech-блок временной истории
 * линейного продольного возмущённого движения.
 *
 * Входы 0..8:
 *
 * H0, V0, objectIndex,
 * Delta V0, Delta Theta0, Delta omega_z0,
 * Delta theta0, Delta x0, Delta H0.
 *
 * Выходы 0..24:
 *
 * time;
 *
 * nominal:
 * x*, H*, V*, Theta*, omega_z*, theta*, alpha*;
 *
 * perturbation:
 * Delta x, Delta H, Delta V, Delta Theta,
 * Delta omega_z, Delta theta, Delta alpha;
 *
 * total:
 * x, H, V, Theta, omega_z, theta, alpha;
 *
 * nominal fall time;
 * finished;
 * resultCode.
 */

enum {
    PF_PT_INPUT_ALTITUDE = 0,
    PF_PT_INPUT_SPEED = 1,
    PF_PT_INPUT_OBJECT_INDEX = 2,
    PF_PT_INPUT_DELTA_SPEED = 3,
    PF_PT_INPUT_DELTA_PATH_ANGLE = 4,
    PF_PT_INPUT_DELTA_PITCH_RATE = 5,
    PF_PT_INPUT_DELTA_PITCH_ANGLE = 6,
    PF_PT_INPUT_DELTA_DOWNRANGE = 7,
    PF_PT_INPUT_DELTA_ALTITUDE = 8,

    PF_PT_OUTPUT_TIME = 9,

    PF_PT_OUTPUT_NOMINAL_DOWNRANGE = 10,
    PF_PT_OUTPUT_NOMINAL_ALTITUDE = 11,
    PF_PT_OUTPUT_NOMINAL_SPEED = 12,
    PF_PT_OUTPUT_NOMINAL_PATH_ANGLE = 13,
    PF_PT_OUTPUT_NOMINAL_PITCH_RATE = 14,
    PF_PT_OUTPUT_NOMINAL_PITCH_ANGLE = 15,
    PF_PT_OUTPUT_NOMINAL_ATTACK_ANGLE = 16,

    PF_PT_OUTPUT_DELTA_DOWNRANGE = 17,
    PF_PT_OUTPUT_DELTA_ALTITUDE = 18,
    PF_PT_OUTPUT_DELTA_SPEED = 19,
    PF_PT_OUTPUT_DELTA_PATH_ANGLE = 20,
    PF_PT_OUTPUT_DELTA_PITCH_RATE = 21,
    PF_PT_OUTPUT_DELTA_PITCH_ANGLE = 22,
    PF_PT_OUTPUT_DELTA_ATTACK_ANGLE = 23,

    PF_PT_OUTPUT_TOTAL_DOWNRANGE = 24,
    PF_PT_OUTPUT_TOTAL_ALTITUDE = 25,
    PF_PT_OUTPUT_TOTAL_SPEED = 26,
    PF_PT_OUTPUT_TOTAL_PATH_ANGLE = 27,
    PF_PT_OUTPUT_TOTAL_PITCH_RATE = 28,
    PF_PT_OUTPUT_TOTAL_PITCH_ANGLE = 29,
    PF_PT_OUTPUT_TOTAL_ATTACK_ANGLE = 30,

    PF_PT_OUTPUT_FALL_TIME = 31,
    PF_PT_OUTPUT_FINISHED = 32,
    PF_PT_OUTPUT_RESULT_CODE = 33,

    PF_PT_EXTERNAL_VARIABLE_COUNT = 34
};

static double defaultReleaseAltitudeM = 100.0;
static double defaultReleaseSpeedMps = 200.0;
static double defaultObjectIndex = 0.0;

static double defaultDeltaSpeedMps = 1.0;
static double defaultDeltaPathAngleRad = 0.0;
static double defaultDeltaPitchRateRadps = 0.0;
static double defaultDeltaPitchAngleRad =
    0.0017453292519943296;
static double defaultDeltaDownrangeM = 0.0;
static double defaultDeltaAltitudeM = 0.0;

static double defaultZero = 0.0;
static double defaultResultCode =
    (double) PF_RESULT_INVALID_INPUT;

static ext_var_info_record externalVariables[
    PF_PT_EXTERNAL_VARIABLE_COUNT
] = {
    {
        "input:0",
        "Release altitude, m",
        &defaultReleaseAltitudeM,
        vt_double,
        {1, 0, 0},
        0,
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
        1,
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
        2,
        dir_input,
        sizeof(double),
        0
    },
    {
        "input:3",
        "Initial Delta V, m/s",
        &defaultDeltaSpeedMps,
        vt_double,
        {1, 0, 0},
        3,
        dir_input,
        sizeof(double),
        0
    },
    {
        "input:4",
        "Initial Delta Theta, rad",
        &defaultDeltaPathAngleRad,
        vt_double,
        {1, 0, 0},
        4,
        dir_input,
        sizeof(double),
        0
    },
    {
        "input:5",
        "Initial Delta omega_z, rad/s",
        &defaultDeltaPitchRateRadps,
        vt_double,
        {1, 0, 0},
        5,
        dir_input,
        sizeof(double),
        0
    },
    {
        "input:6",
        "Initial Delta theta, rad",
        &defaultDeltaPitchAngleRad,
        vt_double,
        {1, 0, 0},
        6,
        dir_input,
        sizeof(double),
        0
    },
    {
        "input:7",
        "Initial Delta x, m",
        &defaultDeltaDownrangeM,
        vt_double,
        {1, 0, 0},
        7,
        dir_input,
        sizeof(double),
        0
    },
    {
        "input:8",
        "Initial Delta H, m",
        &defaultDeltaAltitudeM,
        vt_double,
        {1, 0, 0},
        8,
        dir_input,
        sizeof(double),
        0
    },
    {
        "out:0",
        "Trajectory time, s",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        9,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:1",
        "Nominal downrange, m",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        10,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:2",
        "Nominal altitude, m",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        11,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:3",
        "Nominal speed, m/s",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        12,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:4",
        "Nominal Theta, rad",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        13,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:5",
        "Nominal omega_z, rad/s",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        14,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:6",
        "Nominal theta, rad",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        15,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:7",
        "Nominal alpha, rad",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        16,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:8",
        "Delta downrange, m",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        17,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:9",
        "Delta altitude, m",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        18,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:10",
        "Delta V, m/s",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        19,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:11",
        "Delta Theta, rad",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        20,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:12",
        "Delta omega_z, rad/s",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        21,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:13",
        "Delta theta, rad",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        22,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:14",
        "Delta alpha, rad",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        23,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:15",
        "Total downrange, m",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        24,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:16",
        "Total altitude, m",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        25,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:17",
        "Total speed, m/s",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        26,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:18",
        "Total Theta, rad",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        27,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:19",
        "Total omega_z, rad/s",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        28,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:20",
        "Total theta, rad",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        29,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:21",
        "Total alpha, rad",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        30,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:22",
        "Nominal fall time, s",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        31,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:23",
        "Trajectory finished flag",
        &defaultZero,
        vt_double,
        {1, 0, 0},
        32,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:24",
        "Passive Flight C API result code",
        &defaultResultCode,
        vt_double,
        {1, 0, 0},
        33,
        dir_out,
        sizeof(double),
        0
    }
};

typedef struct PFPerturbedTrajectoryLocalState {
    int initialized;

    double lastValues[9];

    PFPerturbedTrajectoryPoint* points;
    uint64_t pointCount;

    int32_t resultCode;
} PFPerturbedTrajectoryLocalState;

static TTaskInfoStruct taskInfo;

static const unsigned int schemeHash32 =
    0x50465450U;

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
         index < PF_PT_EXTERNAL_VARIABLE_COUNT;
         ++index) {
        if (extVarsAddress[index] == NULL) {
            return 0;
        }
    }

    return 1;
}

static void releaseTrajectory(
    PFPerturbedTrajectoryLocalState* localState
) {
    if (localState == NULL) {
        return;
    }

    free(
        localState->points
    );

    localState->points = NULL;
    localState->pointCount = 0;
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

static int readInputs(
    void** extVarsAddress,
    double values[9]
) {
    int index;

    for (index = 0; index < 9; ++index) {
        double* address =
            externalDouble(
                extVarsAddress,
                index
            );

        if (address == NULL) {
            return 0;
        }

        values[index] =
            *address;
    }

    return 1;
}

static int inputsHaveChanged(
    const PFPerturbedTrajectoryLocalState* localState,
    const double values[9]
) {
    int index;

    if (localState == NULL ||
        !localState->initialized) {
        return 1;
    }

    for (index = 0; index < 9; ++index) {
        if (localState->lastValues[index] !=
            values[index]) {
            return 1;
        }
    }

    return 0;
}

static void rememberInputs(
    PFPerturbedTrajectoryLocalState* localState,
    const double values[9]
) {
    int index;

    for (index = 0; index < 9; ++index) {
        localState->lastValues[index] =
            values[index];
    }

    localState->initialized = 1;
}

static void calculateTrajectoryIfNecessary(
    void** extVarsAddress,
    PFPerturbedTrajectoryLocalState* localState
) {
    double values[9];

    char objectId[512];

    PFPerturbedSimulationInput input;

    uint64_t requiredPointCount = 0;
    uint64_t writtenPointCount = 0;

    int32_t result;

    if (localState == NULL ||
        extVarsAddress == NULL) {
        return;
    }

    if (!readInputs(
            extVarsAddress,
            values
        )) {
        releaseTrajectory(
            localState
        );

        localState->resultCode =
            PF_RESULT_NULL_ARGUMENT;

        return;
    }

    if (!inputsHaveChanged(
            localState,
            values
        )) {
        return;
    }

    releaseTrajectory(
        localState
    );

    rememberInputs(
        localState,
        values
    );

    localState->resultCode =
        PF_RESULT_INVALID_INPUT;

    result =
        objectIndexToId(
            values[PF_PT_INPUT_OBJECT_INDEX],
            objectId,
            sizeof(objectId)
        );

    if (result != PF_RESULT_OK) {
        localState->resultCode =
            result;
        return;
    }

    memset(
        &input,
        0,
        sizeof(input)
    );

    input.objectId =
        objectId;

    input.releaseAltitudeM =
        values[PF_PT_INPUT_ALTITUDE];

    input.releaseSpeedMps =
        values[PF_PT_INPUT_SPEED];

    input.deltaSpeedMps =
        values[PF_PT_INPUT_DELTA_SPEED];

    input.deltaFlightPathAngleRad =
        values[PF_PT_INPUT_DELTA_PATH_ANGLE];

    input.deltaPitchRateRadps =
        values[PF_PT_INPUT_DELTA_PITCH_RATE];

    input.deltaPitchAngleRad =
        values[PF_PT_INPUT_DELTA_PITCH_ANGLE];

    input.deltaDownrangeM =
        values[PF_PT_INPUT_DELTA_DOWNRANGE];

    input.deltaAltitudeM =
        values[PF_PT_INPUT_DELTA_ALTITUDE];

    result =
        pfCalculatePerturbedTrajectory(
            &input,
            NULL,
            0,
            &requiredPointCount,
            &writtenPointCount
        );

    if (result != PF_RESULT_BUFFER_TOO_SMALL ||
        requiredPointCount == 0) {
        localState->resultCode =
            result;
        return;
    }

    if (requiredPointCount >
        SIZE_MAX /
            sizeof(PFPerturbedTrajectoryPoint)) {
        localState->resultCode =
            PF_RESULT_INTERNAL_ERROR;
        return;
    }

    localState->points =
        (PFPerturbedTrajectoryPoint*) calloc(
            (size_t) requiredPointCount,
            sizeof(PFPerturbedTrajectoryPoint)
        );

    if (localState->points == NULL) {
        localState->resultCode =
            PF_RESULT_INTERNAL_ERROR;
        return;
    }

    result =
        pfCalculatePerturbedTrajectory(
            &input,
            localState->points,
            requiredPointCount,
            &requiredPointCount,
            &writtenPointCount
        );

    if (result != PF_RESULT_OK ||
        writtenPointCount == 0) {
        releaseTrajectory(
            localState
        );

        localState->resultCode =
            result;
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

#define PF_INTERPOLATE_FIELD(field) \
    result.field = interpolateValue( \
        first->field, \
        second->field, \
        ratio \
    )

static PFPerturbedTrajectoryPoint
trajectoryPointAtTime(
    const PFPerturbedTrajectoryLocalState* localState,
    double requestedTimeS
) {
    uint64_t lowerIndex;
    uint64_t upperIndex;

    const PFPerturbedTrajectoryPoint* first;
    const PFPerturbedTrajectoryPoint* second;

    double intervalS;
    double ratio;

    PFPerturbedTrajectoryPoint result;

    memset(
        &result,
        0,
        sizeof(result)
    );

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
            lowerIndex =
                middleIndex;
        } else {
            upperIndex =
                middleIndex;
        }
    }

    first =
        &localState->points[lowerIndex];

    second =
        &localState->points[upperIndex];

    intervalS =
        second->timeS -
        first->timeS;

    if (intervalS <= 0.0) {
        return *first;
    }

    ratio =
        (requestedTimeS - first->timeS) /
        intervalS;

    result.timeS =
        requestedTimeS;

    PF_INTERPOLATE_FIELD(nominalDownrangeM);
    PF_INTERPOLATE_FIELD(nominalAltitudeM);
    PF_INTERPOLATE_FIELD(nominalSpeedMps);
    PF_INTERPOLATE_FIELD(nominalFlightPathAngleRad);
    PF_INTERPOLATE_FIELD(nominalPitchRateRadps);
    PF_INTERPOLATE_FIELD(nominalPitchAngleRad);
    PF_INTERPOLATE_FIELD(nominalAngleOfAttackRad);

    PF_INTERPOLATE_FIELD(deltaDownrangeM);
    PF_INTERPOLATE_FIELD(deltaAltitudeM);
    PF_INTERPOLATE_FIELD(deltaSpeedMps);
    PF_INTERPOLATE_FIELD(deltaFlightPathAngleRad);
    PF_INTERPOLATE_FIELD(deltaPitchRateRadps);
    PF_INTERPOLATE_FIELD(deltaPitchAngleRad);
    PF_INTERPOLATE_FIELD(deltaAngleOfAttackRad);

    PF_INTERPOLATE_FIELD(totalDownrangeM);
    PF_INTERPOLATE_FIELD(totalAltitudeM);
    PF_INTERPOLATE_FIELD(totalSpeedMps);
    PF_INTERPOLATE_FIELD(totalFlightPathAngleRad);
    PF_INTERPOLATE_FIELD(totalPitchRateRadps);
    PF_INTERPOLATE_FIELD(totalPitchAngleRad);
    PF_INTERPOLATE_FIELD(totalAngleOfAttackRad);

    return result;
}

#undef PF_INTERPOLATE_FIELD

static void clearOutputPorts(
    void** extVarsAddress,
    int32_t resultCode
) {
    int index;

    for (index = PF_PT_OUTPUT_TIME;
         index < PF_PT_OUTPUT_RESULT_CODE;
         ++index) {
        writeExternalDouble(
            extVarsAddress,
            index,
            0.0
        );
    }

    writeExternalDouble(
        extVarsAddress,
        PF_PT_OUTPUT_RESULT_CODE,
        (double) resultCode
    );
}

static void writePoint(
    void** extVarsAddress,
    const PFPerturbedTrajectoryPoint* point,
    double fallTimeS,
    double finished,
    int32_t resultCode
) {
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_TIME, point->timeS);

    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_NOMINAL_DOWNRANGE, point->nominalDownrangeM);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_NOMINAL_ALTITUDE, point->nominalAltitudeM);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_NOMINAL_SPEED, point->nominalSpeedMps);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_NOMINAL_PATH_ANGLE, point->nominalFlightPathAngleRad);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_NOMINAL_PITCH_RATE, point->nominalPitchRateRadps);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_NOMINAL_PITCH_ANGLE, point->nominalPitchAngleRad);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_NOMINAL_ATTACK_ANGLE, point->nominalAngleOfAttackRad);

    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_DELTA_DOWNRANGE, point->deltaDownrangeM);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_DELTA_ALTITUDE, point->deltaAltitudeM);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_DELTA_SPEED, point->deltaSpeedMps);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_DELTA_PATH_ANGLE, point->deltaFlightPathAngleRad);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_DELTA_PITCH_RATE, point->deltaPitchRateRadps);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_DELTA_PITCH_ANGLE, point->deltaPitchAngleRad);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_DELTA_ATTACK_ANGLE, point->deltaAngleOfAttackRad);

    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_TOTAL_DOWNRANGE, point->totalDownrangeM);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_TOTAL_ALTITUDE, point->totalAltitudeM);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_TOTAL_SPEED, point->totalSpeedMps);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_TOTAL_PATH_ANGLE, point->totalFlightPathAngleRad);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_TOTAL_PITCH_RATE, point->totalPitchRateRadps);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_TOTAL_PITCH_ANGLE, point->totalPitchAngleRad);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_TOTAL_ATTACK_ANGLE, point->totalAngleOfAttackRad);

    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_FALL_TIME, fallTimeS);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_FINISHED, finished);
    writeExternalDouble(extVarsAddress, PF_PT_OUTPUT_RESULT_CODE, (double) resultCode);
}

static void writeTrajectoryOutputs(
    void** extVarsAddress,
    const PFPerturbedTrajectoryLocalState* localState,
    double modelTime
) {
    double requestedTimeS;
    double finalTimeS;
    double finished;

    PFPerturbedTrajectoryPoint point;

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
        isfinite(modelTime) &&
        modelTime > 0.0
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

    point =
        trajectoryPointAtTime(
            localState,
            requestedTimeS
        );

    writePoint(
        extVarsAddress,
        &point,
        finalTimeS,
        finished,
        localState->resultCode
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
        PF_PT_EXTERNAL_VARIABLE_COUNT;

    *nDinVars = 0;
    *nAlgVars = 0;
    *nStateVars = 0;
    *nConsts = 0;

    *sizeOfStateVars = 0;
    *sizeOfConsts = 0;

    *sizeOfLocalVars =
        (int) sizeof(
            PFPerturbedTrajectoryLocalState
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
    PFPerturbedTrajectoryLocalState* localState =
        (PFPerturbedTrajectoryLocalState*) locals;

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
    PFPerturbedTrajectoryLocalState* localState =
        (PFPerturbedTrajectoryLocalState*) locals;

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
        releaseTrajectory(
            localState
        );

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
