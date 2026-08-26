#include "passive_flight_simintech/SimInTechBlockApi.h"
#include "passive_flight_c_api/PassiveFlightCApi.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    TEST_EXTERNAL_VARIABLE_COUNT = 11
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

        exit(EXIT_FAILURE);
    }
}

static void requireNear(
    double actual,
    double expected,
    double tolerance,
    const char* message
) {
    if (fabs(actual - expected) >
        tolerance) {
        fprintf(
            stderr,
            "Test failure: %s: "
            "actual=%.15f, expected=%.15f\n",
            message,
            actual,
            expected
        );

        exit(EXIT_FAILURE);
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

    int infoResult;
    int initResult;
    int runResult;

    void* locals;

    double values[
        TEST_EXTERNAL_VARIABLE_COUNT
    ];

    void* addresses[
        TEST_EXTERNAL_VARIABLE_COUNT
    ];

    int index;

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
        nDinVars == 0,
        "Dynamic variable count must be zero"
    );

    require(
        nAlgVars == 0,
        "Algebraic variable count must be zero"
    );

    require(
        nStateVars == 0,
        "State variable count must be zero"
    );

    require(
        nConsts == 0,
        "Constant count must be zero"
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
         index <
            TEST_EXTERNAL_VARIABLE_COUNT;
         ++index) {
        values[index] = 0.0;
        addresses[index] =
            &values[index];
    }

    /*
     * Входы:
     *
     * H0 = 100 м;
     * V0 = 200 м/с;
     * objectIndex = 0.
     */
    values[0] = 100.0;
    values[1] = 200.0;
    values[2] = 0.0;

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
        values[3],
        1370.709,
        1.0,
        "Incorrect downrange"
    );

    requireNear(
        values[4],
        7.374,
        0.01,
        "Incorrect fall time"
    );

    requireNear(
        values[5],
        176.351,
        0.1,
        "Incorrect impact speed"
    );

    require(
        values[6] < 0.0,
        "Impact path angle must be negative"
    );

    require(
        values[7] < 0.0,
        "Impact pitch angle must be negative"
    );

    require(
        values[8] > 0.0,
        "Impact angle of attack must be positive"
    );

    requireNear(
        values[9],
        (double)
            PF_TERMINATION_GROUND_REACHED,
        0.0,
        "Incorrect termination reason"
    );

    requireNear(
        values[10],
        (double) PF_RESULT_OK,
        0.0,
        "Incorrect result code"
    );

    /*
     * Проверяем изменение входа.
     * Кэш должен сброситься и выполнить новый расчёт.
     */
    values[0] = 1000.0;

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
        "Second RUN_FUNC call failed"
    );

    require(
        values[3] > 3000.0,
        "Changed altitude did not update range"
    );

    require(
        values[4] > 20.0,
        "Changed altitude did not update fall time"
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
        "Invalid object call failed"
    );

    requireNear(
        values[10],
        (double)
            PF_RESULT_INDEX_OUT_OF_RANGE,
        0.0,
        "Invalid object index was not detected"
    );

    free(
        locals
    );

    printf(
        "All native SimInTech block tests passed.\n"
    );

    return EXIT_SUCCESS;
}