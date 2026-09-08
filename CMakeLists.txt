cmake_minimum_required(VERSION 3.16)
project(edgecar_replay_lab VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(EDGECAR_BUILD_TESTS "Build deterministic unit and scenario tests" ON)
option(EDGECAR_ENABLE_SANITIZERS "Enable address and undefined behaviour sanitizers" OFF)

add_library(edgecar_core
    src/pipeline.cpp
    src/scenario.cpp
    src/safety.cpp
    src/telemetry.cpp
    src/types.cpp
)
target_include_directories(edgecar_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(edgecar_core PUBLIC cxx_std_17)
target_compile_options(edgecar_core PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Wconversion>
)
if(EDGECAR_ENABLE_SANITIZERS AND NOT WIN32 AND NOT MSVC)
    target_compile_options(edgecar_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(edgecar_core PUBLIC -fsanitize=address,undefined)
endif()

add_executable(edgecar-replay apps/edgecar_replay.cpp)
target_link_libraries(edgecar-replay PRIVATE edgecar_core)

add_executable(edgecar-bench apps/edgecar_bench.cpp)
target_link_libraries(edgecar-bench PRIVATE edgecar_core)

add_executable(edgecar-report apps/edgecar_report.cpp)
target_link_libraries(edgecar-report PRIVATE edgecar_core)

set_target_properties(edgecar-replay edgecar-bench edgecar-report PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(EDGECAR_BUILD_TESTS)
    enable_testing()
    add_executable(edgecar-tests tests/test_edgecar.cpp)
    target_link_libraries(edgecar-tests PRIVATE edgecar_core)
    add_test(NAME edgecar_unit_and_scenarios COMMAND edgecar-tests)
endif()
