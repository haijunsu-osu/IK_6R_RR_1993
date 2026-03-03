cmake_minimum_required(VERSION 3.20)
project(rr1993_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(EIGEN3_INCLUDE_DIR
    "C:/Program Files/MATLAB/R2025b/toolbox/shared/robotics/externalDependency/eigen/include/eigen3"
    CACHE PATH "Path to Eigen3 headers")

if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
    message(FATAL_ERROR "Eigen headers not found at: ${EIGEN3_INCLUDE_DIR}")
endif()

add_library(rr1993_solver
    src/ik_6r_general.cpp
)

target_include_directories(rr1993_solver
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

if(MSVC)
    target_compile_options(rr1993_solver PRIVATE /O2 /EHsc /W3 /permissive-)
    target_compile_definitions(rr1993_solver PRIVATE NOMINMAX)
endif()

add_executable(test_ik_6r_general
    tests/test_ik_6r_general.cpp
)
target_link_libraries(test_ik_6r_general PRIVATE rr1993_solver)
if(MSVC)
    target_compile_options(test_ik_6r_general PRIVATE /O2 /EHsc /W3 /permissive-)
endif()

add_executable(test_manseur_doty_16
    tests/test_manseur_doty_16.cpp
)
target_link_libraries(test_manseur_doty_16 PRIVATE rr1993_solver)
if(MSVC)
    target_compile_options(test_manseur_doty_16 PRIVATE /O2 /EHsc /W3 /permissive-)
endif()

add_executable(benchmark_1x1
    tests/benchmark_1x1.cpp
)
target_link_libraries(benchmark_1x1 PRIVATE rr1993_solver)
if(MSVC)
    target_compile_options(benchmark_1x1 PRIVATE /O2 /EHsc /W3 /permissive-)
endif()

add_executable(benchmark_random
    tests/benchmark_random.cpp
)
target_link_libraries(benchmark_random PRIVATE rr1993_solver)
if(MSVC)
    target_compile_options(benchmark_random PRIVATE /O2 /EHsc /W3 /permissive-)
endif()
