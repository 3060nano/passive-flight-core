cmake_minimum_required(VERSION 3.20)

project(
        PassiveFlightCore
        VERSION 0.10.0
        LANGUAGES CXX
)

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

add_executable(
        passive_flight_demo
        apps/nominal_flight_demo.cpp
)

target_link_libraries(
        passive_flight_demo
        PRIVATE
        passive_flight_core
)

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
endif()