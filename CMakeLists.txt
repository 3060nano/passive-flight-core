cmake_minimum_required(VERSION 3.20)

project(
        PassiveFlightCore
        VERSION 0.8.0
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
    add_executable(
            test_model_contract
            tests/test_model_contract.cpp
    )

    target_link_libraries(
            test_model_contract
            PRIVATE
            passive_flight_core
    )

    add_test(
            NAME model_contract
            COMMAND test_model_contract
    )

    add_executable(
            test_object_passport
            tests/test_object_passport.cpp
    )

    target_link_libraries(
            test_object_passport
            PRIVATE
            passive_flight_core
    )

    add_test(
            NAME object_passport
            COMMAND test_object_passport
    )

    add_executable(
            test_atmosphere
            tests/test_atmosphere.cpp
    )

    target_link_libraries(
            test_atmosphere
            PRIVATE
            passive_flight_core
    )

    add_test(
            NAME atmosphere
            COMMAND test_atmosphere
    )

    add_executable(
            test_aerodynamics
            tests/test_aerodynamics.cpp
    )

    target_link_libraries(
            test_aerodynamics
            PRIVATE
            passive_flight_core
    )

    add_test(
            NAME aerodynamics
            COMMAND test_aerodynamics
    )

    add_executable(
            test_object_aerodynamics_adapter
            tests/test_object_aerodynamics_adapter.cpp
    )

    target_link_libraries(
            test_object_aerodynamics_adapter
            PRIVATE
            passive_flight_core
    )

    add_test(
            NAME object_aerodynamics_adapter
            COMMAND test_object_aerodynamics_adapter
    )

    add_executable(
            test_nominal_longitudinal_dynamics
            tests/test_nominal_longitudinal_dynamics.cpp
    )

    target_link_libraries(
            test_nominal_longitudinal_dynamics
            PRIVATE
            passive_flight_core
    )

    add_test(
            NAME nominal_longitudinal_dynamics
            COMMAND test_nominal_longitudinal_dynamics
    )

    add_executable(
            test_forward_euler_simulator
            tests/test_forward_euler_simulator.cpp
    )

    target_link_libraries(
            test_forward_euler_simulator
            PRIVATE
            passive_flight_core
    )

    add_test(
            NAME forward_euler_simulator
            COMMAND test_forward_euler_simulator
    )
endif()