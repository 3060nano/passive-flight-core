#include "passive_flight_simintech/SimInTechBlockApi.h"

#include "passive_flight_c_api/PassiveFlightCApi.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    TEST_EXTERNAL_VARIABLE_COUNT = 25
};

static void require(
    int condition,
    const char* message
) {
    if (!condition) {
        fprintf(
            stderr,
            "Test failure: %s\n",
            message
        );

        exit(
            EXIT_FAILURE
        );
    }
}

static void requireNear(
    double actual,
    double expected,
    double tolerance,
    const char* message
) {
    if (fabs(
            actual - expected
        ) > tolerance) {
        fprintf(
            stderr,
            "Test failure: %s: "
            "actual=%.15f, expected=%.15f\n",
            message,
            actual,
            expected
        );

        exit(
            EXIT_FAILURE
        );
    }
}

int main(void) {
    int nExtVars = 0;
    int nDinVars = 0;
    int nAlgVars = 0;
    int nStateVars = 0;
    int nConsts = 0;

    int sizeOfStateVars = 0;
    int sizeOfConsts = 0;
    int sizeOfLocalVars = 0;

    int dinVarsDimension = 0;
    int algVarsDimension = 0;

    void* extVarsInfo = NULL;
    void* dinVarsInfo = NULL;
    void* algVarsInfo = NULL;
    void* stateVarsInfo = NULL;
    void* constInfo = NULL;

    solver_struct solverData;

    unsigned int schemeHash = 0;
    char algorithmName[256] = {0};
    void* algorithmObjectId = NULL;

    void* locals;

    double values[
        TEST_EXTERNAL_VARIABLE_COUNT
    ];

    void* addresses[
        TEST_EXTERNAL_VARIABLE_COUNT
    ];

    int index;
    int infoResult;
    int initResult;
    int runResult;

    memset(
        &solverData,
        0,
        sizeof(solverData)
    );

    infoResult =
        INFO_FUNC(
            &nExtVars,
            &nDinVars,
            &nAlgVars,
            &nStateVars,
            &nConsts,

            &sizeOfStateVars,
            &sizeOfConsts,
            &sizeOfLocalVars,

            &dinVarsDimension,
            &algVarsDimension,

            &extVarsInfo,
            &dinVarsInfo,
            &algVarsInfo,
            &stateVarsInfo,
            &constInfo,

            &solverData,
            &schemeHash,
            algorithmName,
            &algorithmObjectId
        );

    require(
        infoResult == r_Success,
        "INFO_FUNC failed"
    );

    require(
        nExtVars ==
            TEST_EXTERNAL_VARIABLE_COUNT,
        "Incorrect external variable count"
    );

    require(
        nDinVars == 0 &&
        nAlgVars == 0 &&
        nStateVars == 0 &&
        nConsts == 0,
        "Unexpected internal SimInTech variables"
    );

    require(
        sizeOfLocalVars > 0,
        "Local state size must be positive"
    );

    require(
        extVarsInfo != NULL,
        "External variable information is null"
    );

    require(
        solverData.TaskInfo != NULL,
        "Task information is null"
    );

    require(
        schemeHash != 0,
        "Scheme hash is zero"
    );

    locals =
        calloc(
            1,
            (size_t) sizeOfLocalVars
        );

    require(
        locals != NULL,
        "Cannot allocate local state"
    );

    for (index = 0;
         index < TEST_EXTERNAL_VARIABLE_COUNT;
         ++index) {
        values[index] = 0.0;
        addresses[index] =
            &values[index];
    }

    /*
     * Базовый режим:
     *
     * H0 = 100 m;
     * V0 = 200 m/s;
     * objectIndex = 0;
     *
     * Delta V0 = 1 m/s;
     * Delta theta0 = 0.1 deg.
     */
    values[0] = 100.0;
    values[1] = 200.0;
    values[2] = 0.0;

    values[3] = 1.0;
    values[4] = 0.0;
    values[5] = 0.0;
    values[6] =
        0.1 *
        3.14159265358979323846 /
        180.0;
    values[7] = 0.0;
    values[8] = 0.0;

    initResult =
        INIT_FUNC(
            0.001,
            0.0,
            addresses,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            locals,
            &solverData,
            algorithmObjectId
        );

    require(
        initResult == r_Success,
        "INIT_FUNC failed"
    );

    runResult =
        RUN_FUNC(
            f_InitState,
            0.001,
            0.0,
            addresses,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            locals,
            &solverData,
            algorithmObjectId
        );

    require(
        runResult == r_Success,
        "RUN_FUNC failed"
    );

    requireNear(
        values[23],
        (double)
            PF_TERMINATION_GROUND_REACHED,
        0.0,
        "Incorrect termination reason"
    );

    requireNear(
        values[24],
        (double) PF_RESULT_OK,
        0.0,
        "Incorrect result code"
    );

    require(
        isfinite(values[9]),
        "Delta impact time is not finite"
    );

    require(
        isfinite(values[10]),
        "Delta impact range is not finite"
    );

    require(
        isfinite(values[11]),
        "Delta impact speed is not finite"
    );

    require(
        isfinite(values[12]),
        "Delta impact Theta is not finite"
    );

    require(
        isfinite(values[13]),
        "Delta impact omega_z is not finite"
    );

    require(
        isfinite(values[14]),
        "Delta impact theta is not finite"
    );

    require(
        isfinite(values[15]),
        "Delta impact alpha is not finite"
    );

    require(
        fabs(values[10]) > 1.0e-9,
        "Nonzero perturbation did not change impact range"
    );

    require(
        values[16] > 0.0,
        "Perturbed impact time must be positive"
    );

    require(
        values[17] > 0.0,
        "Perturbed impact range must be positive"
    );

    require(
        values[18] > 0.0,
        "Perturbed impact speed must be positive"
    );

    /*
     * Теперь обнуляем все начальные возмущения.
     * Блок обязан сбросить кэш и пересчитать режим.
     */
    values[3] = 0.0;
    values[4] = 0.0;
    values[5] = 0.0;
    values[6] = 0.0;
    values[7] = 0.0;
    values[8] = 0.0;

    runResult =
        RUN_FUNC(
            f_UpdateOuts,
            0.001,
            1.0,
            addresses,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            locals,
            &solverData,
            algorithmObjectId
        );

    require(
        runResult == r_Success,
        "Zero-perturbation RUN_FUNC failed"
    );

    requireNear(
        values[9],
        0.0,
        1.0e-12,
        "Zero perturbation changed impact time"
    );

    requireNear(
        values[10],
        0.0,
        1.0e-12,
        "Zero perturbation changed impact range"
    );

    requireNear(
        values[11],
        0.0,
        1.0e-12,
        "Zero perturbation changed impact speed"
    );

    requireNear(
        values[12],
        0.0,
        1.0e-12,
        "Zero perturbation changed impact Theta"
    );

    requireNear(
        values[13],
        0.0,
        1.0e-12,
        "Zero perturbation changed impact omega_z"
    );

    requireNear(
        values[14],
        0.0,
        1.0e-12,
        "Zero perturbation changed impact theta"
    );

    requireNear(
        values[15],
        0.0,
        1.0e-12,
        "Zero perturbation changed impact alpha"
    );

    /*
     * Несуществующий индекс объекта.
     */
    values[2] = 100.0;

    runResult =
        RUN_FUNC(
            f_UpdateOuts,
            0.001,
            2.0,
            addresses,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            NULL,
            locals,
            &solverData,
            algorithmObjectId
        );

    require(
        runResult == r_Success,
        "Invalid object RUN_FUNC failed"
    );

    requireNear(
        values[24],
        (double)
            PF_RESULT_INDEX_OUT_OF_RANGE,
        0.0,
        "Invalid object index was not detected"
    );

    free(
        locals
    );

    printf(
        "All perturbed SimInTech block tests passed.\n"
    );

    return EXIT_SUCCESS;
}
