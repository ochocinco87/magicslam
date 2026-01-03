cmake_minimum_required(VERSION 3.14)
project(MagicSLAM VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(ENABLE_NEON "Enable ARM NEON optimizations" OFF)
option(USE_FIXED_POINT "Use fixed-point arithmetic" OFF)
option(BUILD_EXAMPLES "Build example applications" ON)
option(BUILD_TESTS "Build unit tests" ON)

# Target platform
set(TARGET_PLATFORM "GENERIC" CACHE STRING "Target platform (GENERIC, AR1, AR2)")

# Platform-specific settings
if(TARGET_PLATFORM STREQUAL "AR1")
    set(ENABLE_NEON ON)
    add_definitions(-DTARGET_AR1)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+simd -mtune=cortex-a55")
elseif(TARGET_PLATFORM STREQUAL "AR2")
    set(ENABLE_NEON ON)
    add_definitions(-DTARGET_AR2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a+simd -mtune=cortex-a78")
endif()

if(ENABLE_NEON)
    add_definitions(-D__ARM_NEON__ -D__ARM_NEON)
endif()

if(USE_FIXED_POINT)
    add_definitions(-DUSE_FIXED_POINT)
endif()

# Compiler flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
    if(ENABLE_NEON)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
    endif()
endif()

# Header-only library
add_library(magicslam INTERFACE)
target_include_directories(magicslam INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(magicslam INTERFACE cxx_std_17)

# Examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation
include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS magicslam EXPORT magicslamTargets)
install(EXPORT magicslamTargets
    FILE magicslamTargets.cmake
    NAMESPACE MagicSLAM::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/magicslam
)

# Package config
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/magicslamConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/magicslamConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/magicslamConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/magicslam
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/magicslamConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/magicslamConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/magicslam
)
