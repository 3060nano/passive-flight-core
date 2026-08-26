#include "passive_flight_c_api/PassiveFlightCApi.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace {

#ifdef _WIN32

using ModuleHandle = HMODULE;

ModuleHandle loadModule(
    const char* path
) {
    return LoadLibraryA(path);
}

void unloadModule(
    ModuleHandle module
) {
    if (module != nullptr) {
        FreeLibrary(module);
    }
}

void* findSymbol(
    ModuleHandle module,
    const char* symbolName
) {
    return reinterpret_cast<void*>(
        GetProcAddress(
            module,
            symbolName
        )
    );
}

std::string moduleErrorMessage() {
    return
        "Windows error code: " +
        std::to_string(
            static_cast<unsigned long>(
                GetLastError()
            )
        );
}

void configureConsoleEncoding() {
    SetConsoleCP(CP_UTF8);
    SetConsoleOutputCP(CP_UTF8);
}

const char* defaultLibraryName() {
    return "libpassive_flight_c_api.dll";
}

#else

using ModuleHandle = void*;

ModuleHandle loadModule(
    const char* path
) {
    return dlopen(
        path,
        RTLD_NOW
    );
}

void unloadModule(
    ModuleHandle module
) {
    if (module != nullptr) {
        dlclose(module);
    }
}

void* findSymbol(
    ModuleHandle module,
    const char* symbolName
) {
    return dlsym(
        module,
        symbolName
    );
}

std::string moduleErrorMessage() {
    const char* message = dlerror();

    if (message == nullptr) {
        return "Unknown dynamic-loader error";
    }

    return message;
}

void configureConsoleEncoding() {
}

const char* defaultLibraryName() {
#if defined(__APPLE__)
    return "libpassive_flight_c_api.dylib";
#else
    return "libpassive_flight_c_api.so";
#endif
}

#endif

class DynamicModule {
public:
    explicit DynamicModule(
        const std::string& path
    )
        : handle_(
              loadModule(
                  path.c_str()
              )
          ) {
        if (handle_ == nullptr) {
            throw std::runtime_error(
                "Failed to load library: " +
                path +
                "\n" +
                moduleErrorMessage()
            );
        }
    }

    ~DynamicModule() {
        unloadModule(handle_);
    }

    DynamicModule(
        const DynamicModule&
    ) = delete;

    DynamicModule& operator=(
        const DynamicModule&
    ) = delete;

    template <typename FunctionType>
    [[nodiscard]]
    FunctionType function(
        const char* functionName
    ) const {
        void* address =
            findSymbol(
                handle_,
                functionName
            );

        if (address == nullptr) {
            throw std::runtime_error(
                "Function was not found in DLL: " +
                std::string(functionName) +
                "\n" +
                moduleErrorMessage()
            );
        }

        return reinterpret_cast<FunctionType>(
            address
        );
    }

private:
    ModuleHandle handle_{};
};

using GetApiVersionFunction =
    const char* (PF_CALL*)(void);

using GetResultCodeNameFunction =
    const char* (PF_CALL*)(int32_t);

using GetObjectCountFunction =
    uint64_t (PF_CALL*)(void);

using GetObjectIdFunction =
    int32_t (PF_CALL*)(
        uint64_t,
        char*,
        uint64_t,
        uint64_t*
    );

using GetObjectDisplayNameFunction =
    int32_t (PF_CALL*)(
        uint64_t,
        char*,
        uint64_t,
        uint64_t*
    );

using CalculateFunction =
    int32_t (PF_CALL*)(
        const PFSimulationInput*,
        PFSimulationOutput*
    );

using CalculateTrajectoryFunction =
    int32_t (PF_CALL*)(
        const PFSimulationInput*,
        PFSimulationOutput*,
        PFTrajectoryPoint*,
        uint64_t,
        uint64_t*,
        uint64_t*
    );

struct ApiFunctions {
    GetApiVersionFunction getApiVersion{};
    GetResultCodeNameFunction getResultCodeName{};

    GetObjectCountFunction getObjectCount{};
    GetObjectIdFunction getObjectId{};
    GetObjectDisplayNameFunction getObjectDisplayName{};

    CalculateFunction calculate{};
    CalculateTrajectoryFunction calculateTrajectory{};
};

ApiFunctions loadApiFunctions(
    const DynamicModule& module
) {
    ApiFunctions functions;

    functions.getApiVersion =
        module.function<GetApiVersionFunction>(
            "pfGetApiVersion"
        );

    functions.getResultCodeName =
        module.function<GetResultCodeNameFunction>(
            "pfGetResultCodeName"
        );

    functions.getObjectCount =
        module.function<GetObjectCountFunction>(
            "pfGetObjectCount"
        );

    functions.getObjectId =
        module.function<GetObjectIdFunction>(
            "pfGetObjectId"
        );

    functions.getObjectDisplayName =
        module.function<GetObjectDisplayNameFunction>(
            "pfGetObjectDisplayName"
        );

    functions.calculate =
        module.function<CalculateFunction>(
            "pfCalculate"
        );

    functions.calculateTrajectory =
        module.function<
            CalculateTrajectoryFunction
        >(
            "pfCalculateTrajectory"
        );

    return functions;
}

void require(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message
) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message);
    }
}

std::string readString(
    uint64_t objectIndex,
    GetObjectIdFunction function
) {
    uint64_t requiredSize = 0;

    const int32_t queryResult =
        function(
            objectIndex,
            nullptr,
            0,
            &requiredSize
        );

    require(
        queryResult ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "String size query failed"
    );

    require(
        requiredSize > 1,
        "Invalid string buffer size"
    );

    std::vector<char> buffer(
        static_cast<std::size_t>(
            requiredSize
        )
    );

    const int32_t readResult =
        function(
            objectIndex,
            buffer.data(),
            requiredSize,
            &requiredSize
        );

    require(
        readResult == PF_RESULT_OK,
        "String reading failed"
    );

    return std::string(
        buffer.data()
    );
}
    
void printSummary(
    const PFSimulationOutput& output
) {
    std::cout
        << "Calculation result:"
        << '\n'
        << "  downrange: "
        << output.downrangeM
        << " m"
        << '\n'
        << "  fall time: "
        << output.fallTimeS
        << " s"
        << '\n'
        << "  impact speed: "
        << output.impactSpeedMps
        << " m/s"
        << '\n'
        << "  impact trajectory angle: "
        << output.impactFlightPathAngleRad
        << " rad"
        << '\n'
        << "  impact pitch angle: "
        << output.impactPitchAngleRad
        << " rad"
        << '\n'
        << "  impact angle of attack: "
        << output.impactAngleOfAttackRad
        << " rad"
        << '\n'
        << "  termination reason: "
        << output.terminationReason
        << '\n';
}

void runSmokeTest(
    const std::string& libraryPath
) {
    std::cout
        << "Loading DLL: "
        << libraryPath
        << '\n';

    const DynamicModule module(
        libraryPath
    );

    const ApiFunctions api =
        loadApiFunctions(module);

    require(
        api.getApiVersion() != nullptr,
        "API version pointer is null"
    );

    std::cout
        << "C API version: "
        << api.getApiVersion()
        << '\n';

    const uint64_t objectCount =
        api.getObjectCount();

    require(
        objectCount > 0,
        "DLL contains no registered objects"
    );

    std::cout
        << "Registered objects: "
        << objectCount
        << '\n';

    for (uint64_t index = 0;
         index < objectCount;
         ++index) {
        const std::string objectId =
            readString(
                index,
                api.getObjectId
            );

        const std::string displayName =
            readString(
                index,
                api.getObjectDisplayName
            );

        std::cout
            << "  [" << index << "] "
            << objectId
            << " — "
            << displayName
            << '\n';
    }

    const std::string objectId =
        readString(
            0,
            api.getObjectId
        );

    PFSimulationInput input{};

    input.objectId =
        objectId.c_str();

    input.releaseAltitudeM =
        100.0;

    input.releaseSpeedMps =
        200.0;

    PFSimulationOutput summaryOutput{};

    const int32_t summaryResult =
        api.calculate(
            &input,
            &summaryOutput
        );

    if (summaryResult != PF_RESULT_OK) {
        throw std::runtime_error(
            "Summary calculation failed: " +
            std::string(
                api.getResultCodeName(
                    summaryResult
                )
            )
        );
    }

    printSummary(
        summaryOutput
    );

    uint64_t requiredPointCount = 0;
    uint64_t writtenPointCount = 0;

    PFSimulationOutput trajectoryQueryOutput{};

    const int32_t queryResult =
        api.calculateTrajectory(
            &input,
            &trajectoryQueryOutput,
            nullptr,
            0,
            &requiredPointCount,
            &writtenPointCount
        );

    require(
        queryResult ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "Trajectory size query failed"
    );

    require(
        requiredPointCount > 1,
        "Trajectory contains insufficient points"
    );

    std::vector<PFTrajectoryPoint> points(
        static_cast<std::size_t>(
            requiredPointCount
        )
    );

    PFSimulationOutput trajectoryOutput{};

    const int32_t trajectoryResult =
        api.calculateTrajectory(
            &input,
            &trajectoryOutput,
            points.data(),
            requiredPointCount,
            &requiredPointCount,
            &writtenPointCount
        );

    if (trajectoryResult != PF_RESULT_OK) {
        throw std::runtime_error(
            "Trajectory calculation failed: " +
            std::string(
                api.getResultCodeName(
                    trajectoryResult
                )
            )
        );
    }

    require(
        writtenPointCount ==
            requiredPointCount,
        "Incorrect number of written points"
    );

    requireNear(
        points.front().timeS,
        0.0,
        1.0e-12,
        "First trajectory point has incorrect time"
    );

    requireNear(
        points.front().altitudeM,
        input.releaseAltitudeM,
        1.0e-12,
        "First trajectory point has incorrect altitude"
    );

    requireNear(
        points.back().altitudeM,
        0.0,
        1.0e-12,
        "Last trajectory point is not on ground"
    );

    requireNear(
        points.back().downrangeM,
        trajectoryOutput.downrangeM,
        1.0e-9,
        "Trajectory range does not match summary"
    );

    requireNear(
        points.back().timeS,
        trajectoryOutput.fallTimeS,
        1.0e-9,
        "Trajectory time does not match summary"
    );

    requireNear(
        summaryOutput.downrangeM,
        trajectoryOutput.downrangeM,
        1.0e-9,
        "Two DLL calculations produced different range"
    );

    std::cout
        << "Trajectory points: "
        << writtenPointCount
        << '\n'
        << "First point:"
        << '\n'
        << "  t = "
        << points.front().timeS
        << " s"
        << '\n'
        << "  H = "
        << points.front().altitudeM
        << " m"
        << '\n'
        << "Last point:"
        << '\n'
        << "  t = "
        << points.back().timeS
        << " s"
        << '\n'
        << "  x = "
        << points.back().downrangeM
        << " m"
        << '\n'
        << "  H = "
        << points.back().altitudeM
        << " m"
        << '\n';

    std::cout
        << "DLL smoke test passed successfully."
        << '\n';
}

} // namespace

int main(
    int argc,
    char* argv[]
) {
    configureConsoleEncoding();

    try {
        const std::string libraryPath =
            argc >= 2
                ? argv[1]
                : defaultLibraryName();

        runSmokeTest(
            libraryPath
        );

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "DLL smoke test failed:"
            << '\n'
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }
}