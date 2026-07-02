cmake_minimum_required(VERSION 3.18)
project(cis565_ScanMatching)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Enable C++17 for host code (required for rocPRIM/rocThrust)
set(CMAKE_CXX_STANDARD 17)

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

########################################
# HIP/CUDA Setup
########################################
option(USE_HIP "Build with HIP for AMD GPUs" OFF)

if(USE_HIP)
    # enable_language(HIP) honors -DCMAKE_HIP_ARCHITECTURES, else auto-detects the
    # host GPU, else errors on a no-GPU host.
    enable_language(HIP)

    # rocThrust is a drop-in for CUDA Thrust
    find_package(rocthrust REQUIRED)
else()
    find_package(CUDA 10 REQUIRED)
    include(${CMAKE_MODULE_PATH}/CUDAComputesList.cmake)

    list(APPEND CUDA_NVCC_FLAGS ${CUDA_GENERATE_CODE})
    list(APPEND CUDA_NVCC_FLAGS_DEBUG "-g -G")
    set(CUDA_VERBOSE_BUILD ON)

    if(WIN32)
        # Set up include and lib paths
        set(CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER} CACHE FILEPATH "Host side compiler used by NVCC" FORCE)
    endif(WIN32)
endif()
########################################

find_package(OpenGL REQUIRED)

if(UNIX)
    find_package(glfw3 REQUIRED)
    find_package(GLEW REQUIRED)
    set(LIBRARIES glfw ${GLEW_LIBRARIES} ${OPENGL_gl_LIBRARY})
else(UNIX)
    set(EXTERNAL "external")

    set(GLFW_ROOT_DIR ${EXTERNAL})
    set(GLFW_USE_STATIC_LIBS ON)
    find_package(GLFW REQUIRED)

    set(GLEW_ROOT_DIR ${EXTERNAL})
    set(GLEW_USE_STATIC_LIBS ON)
    find_package(GLEW REQUIRED)

    add_definitions(${GLEW_DEFINITIONS})
    include_directories(${GLEW_INCLUDE_DIR} ${GLFW_INCLUDE_DIR})
    set(LIBRARIES ${GLEW_LIBRARY} ${GLFW_LIBRARY} ${OPENGL_LIBRARY})
endif(UNIX)

set(GLM_ROOT_DIR "external")
find_package(GLM REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})

set(headers
    src/cuda_to_hip.h
    src/cudaMat4.hpp
    src/glm_device.h
    src/glslUtility.hpp
    src/scanmatch.h
    src/octree.h
    src/main.hpp
    src/pointcloud.h
    src/utilityCore.hpp
    )

set(cuda_sources
    src/scanmatch.cu
    src/octree.cu
    src/pointcloud.cu
    )

set(sources
    src/glslUtility.cpp
    src/main.cpp
    src/utilityCore.cpp
    )

list(SORT headers)
list(SORT cuda_sources)
list(SORT sources)

source_group(Headers FILES ${headers})
source_group(Sources FILES ${sources} ${cuda_sources})

if(USE_HIP)
    # Mark all sources as HIP language so they compile with hipcc
    # This ensures HIP headers work correctly in all sources
    set_source_files_properties(${cuda_sources} ${sources} PROPERTIES LANGUAGE HIP)

    add_executable(${CMAKE_PROJECT_NAME} ${sources} ${cuda_sources} ${headers})
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_HIP GLM_FORCE_CUDA)
    target_link_libraries(${CMAKE_PROJECT_NAME} ${LIBRARIES} roc::rocthrust)
else()
    cuda_add_executable(${CMAKE_PROJECT_NAME} ${sources} ${cuda_sources} ${headers})
    target_link_libraries(${CMAKE_PROJECT_NAME} ${LIBRARIES})
endif()

add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/shaders
        ${CMAKE_BINARY_DIR}/shaders
    )
