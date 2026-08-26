cmake_minimum_required(VERSION 3.28)

project(
    avernal-physics-jolt
    VERSION 0.1.0
    DESCRIPTION "Jolt Physics backend for the Avernal physics API"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

include(FetchContent)

set(AVERNAL_CORE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../avernal-core")
set(AVERNAL_PHYSICS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../avernal-physics")

if(NOT EXISTS "${AVERNAL_CORE_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "avernal-core not found at:\n"
        "  ${AVERNAL_CORE_SOURCE_DIR}\n"
        "Place avernal-core as a sibling of this project, or set "
        "FETCHCONTENT_SOURCE_DIR_AVERNAL_CORE.")
endif()

if(NOT EXISTS "${AVERNAL_PHYSICS_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "avernal-physics not found at:\n"
        "  ${AVERNAL_PHYSICS_SOURCE_DIR}\n"
        "Place avernal-physics as a sibling of this project, or set "
        "FETCHCONTENT_SOURCE_DIR_AVERNAL_PHYSICS.")
endif()

FetchContent_Declare(
    avernal_core
    SOURCE_DIR "${AVERNAL_CORE_SOURCE_DIR}"
    EXCLUDE_FROM_ALL
)

FetchContent_Declare(
    avernal_physics
    SOURCE_DIR "${AVERNAL_PHYSICS_SOURCE_DIR}"
    EXCLUDE_FROM_ALL
)

set(AVERNAL_CORE_BUILD_DEMO OFF)
set(AVERNAL_PHYSICS_BUILD_DEMO OFF)

set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(OVERRIDE_CXX_FLAGS OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
set(ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
set(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(PROFILER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(ENABLE_OBJECT_STREAM OFF CACHE BOOL "" FORCE)
set(JPH_USE_DX12 OFF CACHE BOOL "" FORCE)
set(JPH_USE_VK OFF CACHE BOOL "" FORCE)
set(JPH_USE_MTL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG v5.6.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR Build
    SYSTEM
    EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(avernal_core avernal_physics JoltPhysics)

add_library(avernal-physics-jolt)
add_library(avernal::physics_jolt ALIAS avernal-physics-jolt)

target_sources(
    avernal-physics-jolt
    PRIVATE src/jolt_world.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS include
        FILES
            include/avernal/physics/jolt.hpp
)

target_link_libraries(avernal-physics-jolt PUBLIC avernal::physics PRIVATE Jolt)
target_compile_features(avernal-physics-jolt PUBLIC cxx_std_23)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(avernal-physics-jolt PRIVATE -Wall -Wextra -Wpedantic)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(avernal-physics-jolt PRIVATE -Wno-language-extension-token)
    endif()
elseif(MSVC)
    target_compile_options(avernal-physics-jolt PRIVATE /W4)
endif()

option(AVERNAL_PHYSICS_JOLT_BUILD_DEMO "Build the Avernal Jolt physics demo executable" ON)

if(AVERNAL_PHYSICS_JOLT_BUILD_DEMO)
    add_executable(avernal-physics-jolt-demo apps/demo/main.cpp)
    target_link_libraries(avernal-physics-jolt-demo PRIVATE avernal::physics_jolt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(avernal-physics-jolt-demo PRIVATE -Wall -Wextra -Wpedantic)
    elseif(MSVC)
        target_compile_options(avernal-physics-jolt-demo PRIVATE /W4)
    endif()
endif()

if(PROJECT_IS_TOP_LEVEL)
    option(AVERNAL_PHYSICS_JOLT_BUILD_TESTS "Build Avernal Jolt physics unit tests" ON)
else()
    option(AVERNAL_PHYSICS_JOLT_BUILD_TESTS "Build Avernal Jolt physics unit tests" OFF)
endif()

if(AVERNAL_PHYSICS_JOLT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
