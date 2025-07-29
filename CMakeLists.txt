# Disable in-source builds to prevent source tree corruption
if("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
    message(
        FATAL_ERROR
        "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
"
    )
endif()

cmake_minimum_required(VERSION 3.24)

# Make `GIT_SUBMODULES ""` initialize no submodules
cmake_policy(SET CMP0097 NEW)

set(CMAKE_COLOR_DIAGNOSTICS ON)

# Set default build type to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

if(WIN32)
    project(DAREBench CXX)
else()
    project(DAREBench CXX Fortran)
endif()

# Generate compile_commands.json by default
if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS "YES" CACHE STRING "" FORCE)
endif()

file(GLOB_RECURSE cpp_src src/*.cpp)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set_property(
        SOURCE ${cpp_src}
        PROPERTY
            COMPILE_FLAGS
                "-Wno-array-bounds -Wno-class-memaccess -Wno-dangling-reference -Wno-maybe-uninitialized -Wno-stringop-overflow"
    )
endif()

if(NOT WIN32)
    file(GLOB_RECURSE fortran_src src/*.f)
    set_property(
        SOURCE ${fortran_src}
        PROPERTY
            COMPILE_FLAGS
                "-Wno-compare-reals -Wno-conversion -Wno-function-elimination -Wno-maybe-uninitialized -Wno-unused-dummy-argument"
    )
else()
    list(REMOVE_ITEM cpp_src ${CMAKE_CURRENT_SOURCE_DIR}/src/DARE_SLICOT.cpp)
endif()
add_executable(DAREBench ${cpp_src} ${fortran_src})

if(NOT MSVC)
    target_compile_options(DAREBench PRIVATE -Wall -Wextra -Werror -pedantic)
else()
    target_compile_options(DAREBench PRIVATE /MP /utf-8 /WX)
endif()
target_compile_definitions(DAREBench PRIVATE NDEBUG)
target_compile_features(DAREBench PUBLIC cxx_std_23)
target_include_directories(DAREBench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# FIXME: Linker emits LTO type mismatch warning for sb02od_(), and benchmark
#        crashes on sb02od_()
# set_property(TARGET DAREBench PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

include(FetchContent)

# Options for using a package manager (e.g., vcpkg) for certain dependencies
option(USE_SYSTEM_EIGEN "Use system eigen" OFF)
option(USE_SYSTEM_BENCHMARK "Use system benchmark" OFF)

# Eigen dependency
if(NOT USE_SYSTEM_EIGEN)
    set(EIGEN_BUILD_CMAKE_PACKAGE TRUE)
    set(EIGEN_BUILD_BLAS OFF)
    set(EIGEN_BUILD_LAPACK OFF)
    FetchContent_Declare(
        Eigen3
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        # master on 2024-11-14
        GIT_TAG 0fb2ed140d4fc0108553ecfb25f2d7fc1a9319a1
    )
    FetchContent_MakeAvailable(Eigen3)
else()
    find_package(Eigen3 CONFIG REQUIRED)
endif()
target_link_libraries(DAREBench PRIVATE Eigen3::Eigen)

if(NOT WIN32)
    # lapack dependency
    if(
        CMAKE_CROSSCOMPILING
        AND "${TOOLCHAIN_TRIPLE}" STREQUAL "arm-nilrt-linux-gnueabi"
    )
        add_library(blas_athena SHARED IMPORTED)
        set_target_properties(
            blas_athena
            PROPERTIES
                IMPORTED_LOCATION
                    ${CMAKE_CURRENT_SOURCE_DIR}/${TOOLCHAIN_TRIPLE}/lib/libblas.so.3
        )
        add_library(lapack_athena SHARED IMPORTED)
        set_target_properties(
            lapack_athena
            PROPERTIES
                IMPORTED_LOCATION
                    ${CMAKE_CURRENT_SOURCE_DIR}/${TOOLCHAIN_TRIPLE}/lib/liblapack.so.3
        )
        target_link_libraries(DAREBench PRIVATE blas_athena lapack_athena)
    else()
        include(FindLAPACK)
        target_link_libraries(DAREBench PRIVATE LAPACK::LAPACK)
    endif()
endif()

# benchmark dependency
option(BENCHMARK_ENABLE_TESTING "Enable testing of the benchmark library." OFF)
if(NOT USE_SYSTEM_BENCHMARK)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.9.2
        PATCH_COMMAND
            git apply ${CMAKE_CURRENT_SOURCE_DIR}/benchmark-athena-cpuinfo.patch
        UPDATE_DISCONNECTED TRUE
    )
    FetchContent_MakeAvailable(benchmark)
else()
    find_package(benchmark CONFIG REQUIRED)
endif()
target_link_libraries(DAREBench PRIVATE benchmark::benchmark)
