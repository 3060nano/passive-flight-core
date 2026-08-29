#include "passive_flight_simintech/SimInTechBlockApi.h"
#include "passive_flight_c_api/PassiveFlightCApi.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    TEST_EXTERNAL_VARIABLE_COUNT = 34
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

    double values[
        TEST_EXTERNAL_VARIABLE_COUNT
    ];

    void* addresses[
        TEST_EXTERNAL_VARIABLE_COUNT
    ];

    void* locals;
    double finalTimeS;

    int index;
    int result;

    memset(
        &solverData,
        0,
        sizeof(solverData)
    );

    result =
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
        result == r_Success,
        "INFO_FUNC failed"
    );

    require(
        nExtVars ==
            TEST_EXTERNAL_VARIABLE_COUNT,
        "Incorrect external variable count"
    );

    require(
        sizeOfLocalVars > 0,
        "Local state size must be positive"
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
     * Inputs 0..8.
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

    result =
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
        result == r_Success,
        "INIT_FUNC failed"
    );

    result =
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
        result == r_Success,
        "Initial RUN_FUNC failed"
    );

    /*
     * Output index mapping:
     *
     * 9 time
     * 10..16 nominal
     * 17..23 Delta
     * 24..30 total
     * 31 perturbed fall time
     * 32 finished
     * 33 result code
     */
    requireNear(
        values[9],
        0.0,
        1.0e-12,
        "Initial trajectory time is incorrect"
    );

    requireNear(
        values[10],
        0.0,
        1.0e-12,
        "Initial nominal x is incorrect"
    );

    requireNear(
        values[11],
        100.0,
        1.0e-9,
        "Initial nominal H is incorrect"
    );

    requireNear(
        values[12],
        200.0,
        1.0e-9,
        "Initial nominal V is incorrect"
    );

    requireNear(
        values[19],
        1.0,
        1.0e-12,
        "Initial Delta V is incorrect"
    );

    requireNear(
        values[22],
        values[6],
        1.0e-12,
        "Initial Delta theta is incorrect"
    );

    requireNear(
        values[23],
        values[6],
        1.0e-12,
        "Initial Delta alpha is incorrect"
    );

    requireNear(
        values[26],
        201.0,
        1.0e-9,
        "Initial total speed is incorrect"
    );

    requireNear(
        values[33],
        (double) PF_RESULT_OK,
        0.0,
        "Initial calculation returned an error"
    );

    finalTimeS =
        values[31];

    require(
        finalTimeS > 0.0,
        "Perturbed fall time must be positive"
    );

    /*
     * Проверяем выдачу внутри траектории.
     */
    result =
        RUN_FUNC(
            f_UpdateOuts,
            0.001,
            finalTimeS * 0.5,
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
        result == r_Success,
        "Mid-flight RUN_FUNC failed"
    );

    require(
        values[9] > 0.0 &&
        values[9] < finalTimeS,
        "Mid-flight time is incorrect"
    );

    require(
        values[11] > 0.0 &&
        values[11] < 100.0,
        "Mid-flight nominal altitude is incorrect"
    );

    requireNear(
        values[26],
        values[12] + values[19],
        1.0e-10,
        "Mid-flight total speed is inconsistent"
    );

    requireNear(
        values[27],
        values[13] + values[20],
        1.0e-10,
        "Mid-flight total Theta is inconsistent"
    );

    requireNear(
        values[29],
        values[15] + values[22],
        1.0e-10,
        "Mid-flight total theta is inconsistent"
    );

    requireNear(
        values[30],
        values[29] - values[27],
        1.0e-10,
        "Mid-flight total alpha is inconsistent"
    );

    /*
     * После собственного момента падения
     * возмущённого объекта блок должен
     * удерживать терминальную точку H = 0
     * и выставить finished=1.
     */
    result =
        RUN_FUNC(
            f_UpdateOuts,
            0.001,
            finalTimeS + 1.0,
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
        result == r_Success,
        "Final RUN_FUNC failed"
    );

    requireNear(
        values[9],
        finalTimeS,
        1.0e-9,
        "Final trajectory time must be clamped"
    );

    requireNear(
        values[25],
        0.0,
        1.0e-8,
        "Final perturbed altitude must be ground"
    );

    requireNear(
        values[25],
        values[11] + values[18],
        1.0e-9,
        "Final total altitude is inconsistent"
    );

    requireNear(
        values[32],
        1.0,
        0.0,
        "Finished flag was not set"
    );

    /*
     * Изменение Delta V0 должно инвалидировать кэш.
     */
    values[3] = 0.0;
    values[6] = 0.0;

    result =
        RUN_FUNC(
            f_UpdateOuts,
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
        result == r_Success,
        "Cache invalidation RUN_FUNC failed"
    );

    requireNear(
        values[19],
        0.0,
        1.0e-12,
        "Changed Delta V0 did not invalidate cache"
    );

    requireNear(
        values[23],
        0.0,
        1.0e-12,
        "Changed Delta theta0 did not invalidate cache"
    );

    result =
        RUN_FUNC(
            f_Stop,
            0.001,
            finalTimeS,
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
        result == r_Success,
        "Stop RUN_FUNC failed"
    );

    free(
        locals
    );

    printf(
        "All perturbed SimInTech trajectory block tests passed.\n"
    );

    return EXIT_SUCCESS;
}
