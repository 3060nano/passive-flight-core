cmake_minimum_required(VERSION 3.20)

project(
        PassiveFlightCore
        VERSION 0.14.0
        LANGUAGES C CXX
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
        passive_flight_core
        STATIC
        src/ModelContract.cpp
        src/ObjectPassport.cpp
        src/StandardAtmosphere.cpp
        src/Aerodynamics.cpp
        src/ObjectAerodynamicsAdapter.cpp
        src/NominalLongitudinalDynamics.cpp
        src/ForwardEulerSimulator.cpp
        src/TrajectoryAnalysis.cpp
        src/ObjectRegistry.cpp
)

set_target_properties(
        passive_flight_core
        PROPERTIES
        POSITION_INDEPENDENT_CODE ON
)

target_include_directories(
        passive_flight_core
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
    target_compile_options(
            passive_flight_core
            PRIVATE
            /W4
            /permissive-
    )
else()
    target_compile_options(
            passive_flight_core
            PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endif()

add_library(
        passive_flight_c_api
        SHARED
        src_c_api/PassiveFlightCApi.cpp
)

target_compile_definitions(
        passive_flight_c_api
        PRIVATE
        PF_C_API_EXPORTS
)

target_include_directories(
        passive_flight_c_api
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
        passive_flight_c_api
        PRIVATE
        passive_flight_core
)

set_target_properties(
        passive_flight_c_api
        PROPERTIES
        OUTPUT_NAME "passive_flight_c_api"
)

if(MSVC)
    target_compile_options(
            passive_flight_c_api
            PRIVATE
            /W4
            /permissive-
    )
else()
    target_compile_options(
            passive_flight_c_api
            PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )
endif()

if(MINGW)
    target_link_options(
            passive_flight_c_api
            PRIVATE
            -static-libgcc
            -static-libstdc++
    )
endif()

add_executable(
        passive_flight_demo
        apps/nominal_flight_demo.cpp
)

target_link_libraries(
        passive_flight_demo
        PRIVATE
        passive_flight_core
)

add_executable(
        passive_flight_dll_smoke_test
        apps/dll_smoke_test.cpp
)

target_include_directories(
        passive_flight_dll_smoke_test
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_dependencies(
        passive_flight_dll_smoke_test
        passive_flight_c_api
)

if(UNIX AND NOT APPLE)
    target_link_libraries(
            passive_flight_dll_smoke_test
            PRIVATE
            dl
    )
endif()


if(WIN32 AND MINGW)
    add_library(
            passive_flight_simintech
            SHARED
            src_simintech/SimInTechBlock.c
            src_simintech/PassiveFlightSimInTech.def
    )

    target_compile_definitions(
            passive_flight_simintech
            PRIVATE
            PF_SIT_BLOCK_EXPORTS
    )

    target_include_directories(
            passive_flight_simintech
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/third_party/simintech
    )

    target_link_libraries(
            passive_flight_simintech
            PRIVATE
            passive_flight_c_api
    )

    target_compile_options(
            passive_flight_simintech
            PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -fpack-struct=1
    )

    target_link_options(
            passive_flight_simintech
            PRIVATE
            -Wl,--add-stdcall-alias
            -static-libgcc
    )

    set_target_properties(
            passive_flight_simintech
            PROPERTIES
            PREFIX ""
            OUTPUT_NAME "passive_flight_simintech"
    )

    add_library(
            passive_flight_trajectory_simintech
            SHARED
            src_simintech/SimInTechTrajectoryBlock.c
            src_simintech/PassiveFlightTrajectorySimInTech.def
    )

    target_compile_definitions(
            passive_flight_trajectory_simintech
            PRIVATE
            PF_SIT_BLOCK_EXPORTS
    )

    target_include_directories(
            passive_flight_trajectory_simintech
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/third_party/simintech
    )

    target_link_libraries(
            passive_flight_trajectory_simintech
            PRIVATE
            passive_flight_c_api
    )

    target_compile_options(
            passive_flight_trajectory_simintech
            PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -fpack-struct=1
    )

    target_link_options(
            passive_flight_trajectory_simintech
            PRIVATE
            -Wl,--add-stdcall-alias
            -static-libgcc
    )

    set_target_properties(
            passive_flight_trajectory_simintech
            PROPERTIES
            PREFIX ""
            OUTPUT_NAME "passive_flight_trajectory_simintech"
    )
endif()

include(CTest)

if(BUILD_TESTING)
    function(add_core_test target source test_name)
        add_executable(
                ${target}
                ${source}
        )

        target_link_libraries(
                ${target}
                PRIVATE
                passive_flight_core
        )

        add_test(
                NAME ${test_name}
                COMMAND ${target}
        )
    endfunction()

    add_core_test(
            test_model_contract
            tests/test_model_contract.cpp
            model_contract
    )

    add_core_test(
            test_object_passport
            tests/test_object_passport.cpp
            object_passport
    )

    add_core_test(
            test_atmosphere
            tests/test_atmosphere.cpp
            atmosphere
    )

    add_core_test(
            test_aerodynamics
            tests/test_aerodynamics.cpp
            aerodynamics
    )

    add_core_test(
            test_object_aerodynamics_adapter
            tests/test_object_aerodynamics_adapter.cpp
            object_aerodynamics_adapter
    )

    add_core_test(
            test_nominal_longitudinal_dynamics
            tests/test_nominal_longitudinal_dynamics.cpp
            nominal_longitudinal_dynamics
    )

    add_core_test(
            test_forward_euler_simulator
            tests/test_forward_euler_simulator.cpp
            forward_euler_simulator
    )

    add_core_test(
            test_trajectory_analysis
            tests/test_trajectory_analysis.cpp
            trajectory_analysis
    )

    add_core_test(
            test_object_registry
            tests/test_object_registry.cpp
            object_registry
    )

    add_executable(
            test_c_api
            tests/test_c_api.cpp
    )

    target_link_libraries(
            test_c_api
            PRIVATE
            passive_flight_c_api
    )

    add_test(
            NAME c_api
            COMMAND test_c_api
    )

    add_test(
            NAME dll_smoke
            COMMAND
            passive_flight_dll_smoke_test
            $<TARGET_FILE:passive_flight_c_api>
    )

    if(WIN32 AND MINGW)
        add_executable(
                test_simintech_block
                tests/test_simintech_block.c
        )

        target_include_directories(
                test_simintech_block
                PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/include
                ${CMAKE_CURRENT_SOURCE_DIR}/third_party/simintech
        )

        target_compile_options(
                test_simintech_block
                PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -fpack-struct=1
        )

        target_link_libraries(
                test_simintech_block
                PRIVATE
                passive_flight_simintech
                passive_flight_c_api
        )

        add_test(
                NAME simintech_block
                COMMAND test_simintech_block
        )

        add_executable(
                test_simintech_trajectory_block
                tests/test_simintech_trajectory_block.c
        )

        target_include_directories(
                test_simintech_trajectory_block
                PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/include
                ${CMAKE_CURRENT_SOURCE_DIR}/third_party/simintech
        )

        target_compile_options(
                test_simintech_trajectory_block
                PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -fpack-struct=1
        )

        target_link_libraries(
                test_simintech_trajectory_block
                PRIVATE
                passive_flight_trajectory_simintech
                passive_flight_c_api
        )

        add_test(
                NAME simintech_trajectory_block
                COMMAND test_simintech_trajectory_block
        )
    endif()

endif()
