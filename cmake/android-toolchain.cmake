# Android NDK Toolchain for MagicSLAM
# Targets Qualcomm Snapdragon AR1 Gen 1 (ARM64 + NEON)

set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_SYSTEM_VERSION 30)  # Android 11+ (API 30)
set(CMAKE_ANDROID_ARCH_ABI arm64-v8a)
set(CMAKE_ANDROID_STL_TYPE c++_shared)

# NDK path - set via environment or override
if(NOT CMAKE_ANDROID_NDK)
    if(DEFINED ENV{ANDROID_NDK})
        set(CMAKE_ANDROID_NDK $ENV{ANDROID_NDK})
    elseif(DEFINED ENV{ANDROID_NDK_HOME})
        set(CMAKE_ANDROID_NDK $ENV{ANDROID_NDK_HOME})
    else()
        message(FATAL_ERROR "ANDROID_NDK or ANDROID_NDK_HOME not set")
    endif()
endif()

# Enable NEON SIMD (critical for AR1 performance)
set(CMAKE_ANDROID_ARM_NEON ON)

# Compiler flags for Snapdragon AR1 (Cortex-A55/A78 based)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a+simd -mtune=cortex-a55")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+simd -mtune=cortex-a55")

# Release optimizations
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -ffast-math -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math -DNDEBUG")

# Enable LTO for smaller binary
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

message(STATUS "Android NDK: ${CMAKE_ANDROID_NDK}")
message(STATUS "Target ABI: ${CMAKE_ANDROID_ARCH_ABI}")
message(STATUS "NEON: ${CMAKE_ANDROID_ARM_NEON}")
