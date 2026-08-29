#include "passive_flight_simintech/SimInTechBlockApi.h"

#include "passive_flight_c_api/PassiveFlightCApi.h"
#include "passive_flight_c_api/PerturbedFlightCApi.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * ОТДЕЛЬНЫЙ БЛОК ВОЗМУЩЁННОГО ДВИЖЕНИЯ SIMINTECH
 *
 * Входы:
 *
 * 0 — releaseAltitudeM;
 * 1 — releaseSpeedMps;
 * 2 — objectIndex;
 * 3 — Delta V0, m/s;
 * 4 — Delta Theta0, rad;
 * 5 — Delta omega_z0, rad/s;
 * 6 — Delta theta0, rad;
 * 7 — Delta x0, m;
 * 8 — Delta H0, m.
 *
 * Выходы:
 *
 * 9  — Delta t_f, s;
 * 10 — Delta L, m;
 * 11 — Delta V_impact, m/s;
 * 12 — Delta Theta_impact, rad;
 * 13 — Delta omega_z_impact, rad/s;
 * 14 — Delta theta_impact, rad;
 * 15 — Delta alpha_impact, rad;
 *
 * 16 — perturbed impact time, s;
 * 17 — perturbed impact downrange, m;
 * 18 — perturbed impact speed, m/s;
 * 19 — perturbed impact Theta, rad;
 * 20 — perturbed impact omega_z, rad/s;
 * 21 — perturbed impact theta, rad;
 * 22 — perturbed impact alpha, rad;
 *
 * 23 — terminationReason;
 * 24 — resultCode.
 */
enum {
    PF_SIT_P_RELEASE_ALTITUDE_INDEX = 0,
    PF_SIT_P_RELEASE_SPEED_INDEX = 1,
    PF_SIT_P_OBJECT_INDEX = 2,

    PF_SIT_P_DELTA_SPEED_INDEX = 3,
    PF_SIT_P_DELTA_PATH_ANGLE_INDEX = 4,
    PF_SIT_P_DELTA_PITCH_RATE_INDEX = 5,
    PF_SIT_P_DELTA_PITCH_ANGLE_INDEX = 6,
    PF_SIT_P_DELTA_DOWNRANGE_INDEX = 7,
    PF_SIT_P_DELTA_ALTITUDE_INDEX = 8,

    PF_SIT_P_DELTA_FALL_TIME_INDEX = 9,
    PF_SIT_P_DELTA_IMPACT_RANGE_INDEX = 10,
    PF_SIT_P_DELTA_IMPACT_SPEED_INDEX = 11,
    PF_SIT_P_DELTA_IMPACT_PATH_ANGLE_INDEX = 12,
    PF_SIT_P_DELTA_IMPACT_PITCH_RATE_INDEX = 13,
    PF_SIT_P_DELTA_IMPACT_PITCH_ANGLE_INDEX = 14,
    PF_SIT_P_DELTA_IMPACT_ATTACK_ANGLE_INDEX = 15,

    PF_SIT_P_IMPACT_TIME_INDEX = 16,
    PF_SIT_P_IMPACT_RANGE_INDEX = 17,
    PF_SIT_P_IMPACT_SPEED_INDEX = 18,
    PF_SIT_P_IMPACT_PATH_ANGLE_INDEX = 19,
    PF_SIT_P_IMPACT_PITCH_RATE_INDEX = 20,
    PF_SIT_P_IMPACT_PITCH_ANGLE_INDEX = 21,
    PF_SIT_P_IMPACT_ATTACK_ANGLE_INDEX = 22,

    PF_SIT_P_TERMINATION_REASON_INDEX = 23,
    PF_SIT_P_RESULT_CODE_INDEX = 24,

    PF_SIT_P_INPUT_COUNT = 9,
    PF_SIT_P_OUTPUT_COUNT = 16,
    PF_SIT_P_EXTERNAL_VARIABLE_COUNT = 25
};

/*
 * Значения входов по умолчанию.
 *
 * По умолчанию воспроизводится тот же тип
 * начального возмущения, который использовался
 * в perturbed_flight_demo:
 *
 * Delta V0 = 1 m/s;
 * Delta theta0 = 0.1 deg;
 * остальные Delta = 0.
 */
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

/*
 * Значения выходов по умолчанию.
 */
static double defaultDeltaFallTimeS = 0.0;
static double defaultDeltaImpactRangeM = 0.0;
static double defaultDeltaImpactSpeedMps = 0.0;
static double defaultDeltaImpactPathAngleRad = 0.0;
static double defaultDeltaImpactPitchRateRadps = 0.0;
static double defaultDeltaImpactPitchAngleRad = 0.0;
static double defaultDeltaImpactAttackAngleRad = 0.0;

static double defaultImpactTimeS = 0.0;
static double defaultImpactRangeM = 0.0;
static double defaultImpactSpeedMps = 0.0;
static double defaultImpactPathAngleRad = 0.0;
static double defaultImpactPitchRateRadps = 0.0;
static double defaultImpactPitchAngleRad = 0.0;
static double defaultImpactAttackAngleRad = 0.0;

static double defaultTerminationReason =
    (double) PF_TERMINATION_INVALID_INPUT;

static double defaultResultCode =
    (double) PF_RESULT_INVALID_INPUT;

static ext_var_info_record externalVariables[
    PF_SIT_P_EXTERNAL_VARIABLE_COUNT
] = {
    {
        "input:0",
        "Release altitude, m",
        &defaultReleaseAltitudeM,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_RELEASE_ALTITUDE_INDEX,
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
        PF_SIT_P_RELEASE_SPEED_INDEX,
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
        PF_SIT_P_OBJECT_INDEX,
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
        PF_SIT_P_DELTA_SPEED_INDEX,
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
        PF_SIT_P_DELTA_PATH_ANGLE_INDEX,
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
        PF_SIT_P_DELTA_PITCH_RATE_INDEX,
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
        PF_SIT_P_DELTA_PITCH_ANGLE_INDEX,
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
        PF_SIT_P_DELTA_DOWNRANGE_INDEX,
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
        PF_SIT_P_DELTA_ALTITUDE_INDEX,
        dir_input,
        sizeof(double),
        0
    },

    {
        "out:0",
        "Delta impact time, s",
        &defaultDeltaFallTimeS,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_DELTA_FALL_TIME_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:1",
        "Delta impact downrange, m",
        &defaultDeltaImpactRangeM,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_DELTA_IMPACT_RANGE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:2",
        "Delta impact speed, m/s",
        &defaultDeltaImpactSpeedMps,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_DELTA_IMPACT_SPEED_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:3",
        "Delta impact Theta, rad",
        &defaultDeltaImpactPathAngleRad,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_DELTA_IMPACT_PATH_ANGLE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:4",
        "Delta impact omega_z, rad/s",
        &defaultDeltaImpactPitchRateRadps,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_DELTA_IMPACT_PITCH_RATE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:5",
        "Delta impact theta, rad",
        &defaultDeltaImpactPitchAngleRad,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_DELTA_IMPACT_PITCH_ANGLE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:6",
        "Delta impact alpha, rad",
        &defaultDeltaImpactAttackAngleRad,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_DELTA_IMPACT_ATTACK_ANGLE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:7",
        "Perturbed impact time, s",
        &defaultImpactTimeS,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_IMPACT_TIME_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:8",
        "Perturbed impact downrange, m",
        &defaultImpactRangeM,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_IMPACT_RANGE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:9",
        "Perturbed impact speed, m/s",
        &defaultImpactSpeedMps,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_IMPACT_SPEED_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:10",
        "Perturbed impact Theta, rad",
        &defaultImpactPathAngleRad,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_IMPACT_PATH_ANGLE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:11",
        "Perturbed impact omega_z, rad/s",
        &defaultImpactPitchRateRadps,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_IMPACT_PITCH_RATE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:12",
        "Perturbed impact theta, rad",
        &defaultImpactPitchAngleRad,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_IMPACT_PITCH_ANGLE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:13",
        "Perturbed impact alpha, rad",
        &defaultImpactAttackAngleRad,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_IMPACT_ATTACK_ANGLE_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:14",
        "Simulation termination reason",
        &defaultTerminationReason,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_TERMINATION_REASON_INDEX,
        dir_out,
        sizeof(double),
        0
    },
    {
        "out:15",
        "Passive Flight DLL result code",
        &defaultResultCode,
        vt_double,
        {1, 0, 0},
        PF_SIT_P_RESULT_CODE_INDEX,
        dir_out,
        sizeof(double),
        0
    }
};

typedef struct PFPerturbedSimInTechLocalState {
    int initialized;

    double lastReleaseAltitudeM;
    double lastReleaseSpeedMps;
    double lastObjectIndex;

    double lastDeltaSpeedMps;
    double lastDeltaPathAngleRad;
    double lastDeltaPitchRateRadps;
    double lastDeltaPitchAngleRad;
    double lastDeltaDownrangeM;
    double lastDeltaAltitudeM;

    PFPerturbedImpactOutput output;
    int32_t resultCode;
} PFPerturbedSimInTechLocalState;

static TTaskInfoStruct taskInfo;

static const unsigned int schemeHash32 =
    0x50465031U;

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
         index < PF_SIT_P_EXTERNAL_VARIABLE_COUNT;
         ++index) {
        if (extVarsAddress[index] == NULL) {
            return 0;
        }
    }

    return 1;
}

static void clearLocalResult(
    PFPerturbedSimInTechLocalState* localState
) {
    if (localState == NULL) {
        return;
    }

    memset(
        &localState->output,
        0,
        sizeof(localState->output)
    );

    localState->output.terminationReason =
        PF_TERMINATION_INVALID_INPUT;

    localState->resultCode =
        PF_RESULT_INVALID_INPUT;
}

static void writeLocalResultToPorts(
    void** extVarsAddress,
    const PFPerturbedSimInTechLocalState* localState
) {
    const PFPerturbedImpactOutput* output;

    if (localState == NULL) {
        return;
    }

    output =
        &localState->output;

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_DELTA_FALL_TIME_INDEX,
        output->deltaFallTimeS
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_DELTA_IMPACT_RANGE_INDEX,
        output->deltaDownrangeM
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_DELTA_IMPACT_SPEED_INDEX,
        output->deltaImpactSpeedMps
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_DELTA_IMPACT_PATH_ANGLE_INDEX,
        output->deltaImpactFlightPathAngleRad
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_DELTA_IMPACT_PITCH_RATE_INDEX,
        output->deltaImpactPitchRateRadps
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_DELTA_IMPACT_PITCH_ANGLE_INDEX,
        output->deltaImpactPitchAngleRad
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_DELTA_IMPACT_ATTACK_ANGLE_INDEX,
        output->deltaImpactAngleOfAttackRad
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_IMPACT_TIME_INDEX,
        output->perturbedImpactTimeS
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_IMPACT_RANGE_INDEX,
        output->perturbedImpactDownrangeM
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_IMPACT_SPEED_INDEX,
        output->perturbedImpactSpeedMps
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_IMPACT_PATH_ANGLE_INDEX,
        output->perturbedImpactFlightPathAngleRad
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_IMPACT_PITCH_RATE_INDEX,
        output->perturbedImpactPitchRateRadps
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_IMPACT_PITCH_ANGLE_INDEX,
        output->perturbedImpactPitchAngleRad
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_IMPACT_ATTACK_ANGLE_INDEX,
        output->perturbedImpactAngleOfAttackRad
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_TERMINATION_REASON_INDEX,
        (double) output->terminationReason
    );

    writeExternalDouble(
        extVarsAddress,
        PF_SIT_P_RESULT_CODE_INDEX,
        (double) localState->resultCode
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
    const PFPerturbedSimInTechLocalState* localState,
    const PFPerturbedSimulationInput* input,
    double objectIndex
) {
    if (localState == NULL ||
        input == NULL ||
        !localState->initialized) {
        return 1;
    }

    return
        localState->lastReleaseAltitudeM !=
            input->releaseAltitudeM ||
        localState->lastReleaseSpeedMps !=
            input->releaseSpeedMps ||
        localState->lastObjectIndex !=
            objectIndex ||

        localState->lastDeltaSpeedMps !=
            input->deltaSpeedMps ||
        localState->lastDeltaPathAngleRad !=
            input->deltaFlightPathAngleRad ||
        localState->lastDeltaPitchRateRadps !=
            input->deltaPitchRateRadps ||
        localState->lastDeltaPitchAngleRad !=
            input->deltaPitchAngleRad ||
        localState->lastDeltaDownrangeM !=
            input->deltaDownrangeM ||
        localState->lastDeltaAltitudeM !=
            input->deltaAltitudeM;
}

static void rememberInputs(
    PFPerturbedSimInTechLocalState* localState,
    const PFPerturbedSimulationInput* input,
    double objectIndex
) {
    localState->lastReleaseAltitudeM =
        input->releaseAltitudeM;

    localState->lastReleaseSpeedMps =
        input->releaseSpeedMps;

    localState->lastObjectIndex =
        objectIndex;

    localState->lastDeltaSpeedMps =
        input->deltaSpeedMps;

    localState->lastDeltaPathAngleRad =
        input->deltaFlightPathAngleRad;

    localState->lastDeltaPitchRateRadps =
        input->deltaPitchRateRadps;

    localState->lastDeltaPitchAngleRad =
        input->deltaPitchAngleRad;

    localState->lastDeltaDownrangeM =
        input->deltaDownrangeM;

    localState->lastDeltaAltitudeM =
        input->deltaAltitudeM;

    localState->initialized = 1;
}

static int readInput(
    void** extVarsAddress,
    int index,
    double* value
) {
    double* address;

    if (value == NULL) {
        return 0;
    }

    address =
        externalDouble(
            extVarsAddress,
            index
        );

    if (address == NULL) {
        return 0;
    }

    *value =
        *address;

    return 1;
}

static void calculateIfNecessary(
    void** extVarsAddress,
    PFPerturbedSimInTechLocalState* localState
) {
    PFPerturbedSimulationInput input;
    double objectIndex;
    char objectId[512];
    int objectResult;

    if (localState == NULL ||
        extVarsAddress == NULL) {
        return;
    }

    memset(
        &input,
        0,
        sizeof(input)
    );

    if (!readInput(
            extVarsAddress,
            PF_SIT_P_RELEASE_ALTITUDE_INDEX,
            &input.releaseAltitudeM
        ) ||
        !readInput(
            extVarsAddress,
            PF_SIT_P_RELEASE_SPEED_INDEX,
            &input.releaseSpeedMps
        ) ||
        !readInput(
            extVarsAddress,
            PF_SIT_P_OBJECT_INDEX,
            &objectIndex
        ) ||
        !readInput(
            extVarsAddress,
            PF_SIT_P_DELTA_SPEED_INDEX,
            &input.deltaSpeedMps
        ) ||
        !readInput(
            extVarsAddress,
            PF_SIT_P_DELTA_PATH_ANGLE_INDEX,
            &input.deltaFlightPathAngleRad
        ) ||
        !readInput(
            extVarsAddress,
            PF_SIT_P_DELTA_PITCH_RATE_INDEX,
            &input.deltaPitchRateRadps
        ) ||
        !readInput(
            extVarsAddress,
            PF_SIT_P_DELTA_PITCH_ANGLE_INDEX,
            &input.deltaPitchAngleRad
        ) ||
        !readInput(
            extVarsAddress,
            PF_SIT_P_DELTA_DOWNRANGE_INDEX,
            &input.deltaDownrangeM
        ) ||
        !readInput(
            extVarsAddress,
            PF_SIT_P_DELTA_ALTITUDE_INDEX,
            &input.deltaAltitudeM
        )) {
        clearLocalResult(
            localState
        );

        localState->resultCode =
            PF_RESULT_NULL_ARGUMENT;

        return;
    }

    if (!inputsHaveChanged(
            localState,
            &input,
            objectIndex
        )) {
        return;
    }

    rememberInputs(
        localState,
        &input,
        objectIndex
    );

    clearLocalResult(
        localState
    );

    /*
     * clearLocalResult не должен сбрасывать флаг
     * и сохранённые входы.
     */
    localState->initialized = 1;

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

    input.objectId =
        objectId;

    localState->resultCode =
        pfCalculatePerturbedImpact(
            &input,
            &localState->output
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
        PF_SIT_P_EXTERNAL_VARIABLE_COUNT;

    *nDinVars = 0;
    *nAlgVars = 0;
    *nStateVars = 0;
    *nConsts = 0;

    *sizeOfStateVars = 0;
    *sizeOfConsts = 0;

    *sizeOfLocalVars =
        (int) sizeof(
            PFPerturbedSimInTechLocalState
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
    PFPerturbedSimInTechLocalState* localState =
        (PFPerturbedSimInTechLocalState*) locals;

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
    PFPerturbedSimInTechLocalState* localState =
        (PFPerturbedSimInTechLocalState*) locals;

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
