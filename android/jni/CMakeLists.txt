cmake_minimum_required(VERSION 3.18)
project(magicslam_jni)

# Find MagicSLAM headers
set(MAGICSLAM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

# JNI shared library
add_library(magicslam_jni SHARED
    magicslam_jni.cpp
)

target_include_directories(magicslam_jni PRIVATE
    ${MAGICSLAM_INCLUDE_DIR}
)

target_compile_features(magicslam_jni PRIVATE cxx_std_17)

# Enable NEON
target_compile_definitions(magicslam_jni PRIVATE
    __ARM_NEON__
    __ARM_NEON
)

# Link Android log library
find_library(log-lib log)
target_link_libraries(magicslam_jni ${log-lib})

# Strip for release
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set_target_properties(magicslam_jni PROPERTIES
        LINK_FLAGS "-s"
    )
endif()
