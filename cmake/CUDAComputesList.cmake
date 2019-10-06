## CUDA Compute detection code
## Sourced from ArrayFire https://github.com/arrayfire/arrayfire/
## Under BSD-3 Clause License https://github.com/arrayfire/arrayfire/blob/devel/LICENSE

#Disables running cuda_compute_check.c when build windows using remote
OPTION(CUDA_COMPUTE_DETECT "Run autodetection of CUDA Architecture" ON)
MARK_AS_ADVANCED(CUDA_COMPUTE_DETECT)

IF(CUDA_COMPUTE_DETECT AND NOT DEFINED COMPUTES_DETECTED_LIST)
    #############################
    #Sourced from:
    #https://raw.githubusercontent.com/jwetzl/CudaLBFGS/master/CheckComputeCapability.cmake
    #############################
    # Check for GPUs present and their compute capability
    # based on http://stackoverflow.com/questions/2285185/easiest-way-to-test-for-existence-of-cuda-capable-gpu-from-cmake/2297877#2297877 (Christopher Bruns)

    IF(CUDA_FOUND)
        MESSAGE(STATUS "${CMAKE_MODULE_PATH}/cuda_compute_capability.cpp")

        TRY_RUN(RUN_RESULT_VAR COMPILE_RESULT_VAR
                ${PROJECT_BINARY_DIR}
                ${CMAKE_MODULE_PATH}/cuda_compute_capability.cpp
                CMAKE_FLAGS
                -DINCLUDE_DIRECTORIES:STRING=${CUDA_TOOLKIT_INCLUDE}
                -DLINK_LIBRARIES:STRING=${CUDA_CUDART_LIBRARY}
                COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
                RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)

        MESSAGE(STATUS "CUDA Compute Detection Output: ${RUN_OUTPUT_VAR}")
        MESSAGE(STATUS "CUDA Compute Detection Return: ${RUN_RESULT_VAR}")

        # COMPILE_RESULT_VAR is TRUE when compile succeeds
        # Check Return Value of main() from RUN_RESULT_VAR
        # RUN_RESULT_VAR is 0 when a GPU is found
        # RUN_RESULT_VAR is 1 when errors occur

        IF(COMPILE_RESULT_VAR AND RUN_RESULT_VAR EQUAL 0)
            MESSAGE(STATUS "CUDA Compute Detection Worked")
            # Convert output into a list of computes
            STRING(REPLACE " " ";" COMPUTES_DETECTED_LIST ${RUN_OUTPUT_VAR})
            SET(CUDA_HAVE_GPU TRUE CACHE BOOL "Whether CUDA-capable GPU is present")
        ELSE()
            MESSAGE(STATUS "CUDA Compute Detection Failed")
            SET(CUDA_HAVE_GPU FALSE CACHE BOOL "Whether CUDA-capable GPU is present")
        ENDIF()
    ENDIF(CUDA_FOUND)
ENDIF()

IF(    CUDA_COMPUTE_20
    OR CUDA_COMPUTE_30
    OR CUDA_COMPUTE_32
    OR CUDA_COMPUTE_35
    OR CUDA_COMPUTE_37
    OR CUDA_COMPUTE_50
    OR CUDA_COMPUTE_52
    OR CUDA_COMPUTE_53
    OR CUDA_COMPUTE_60
    OR CUDA_COMPUTE_61
    OR CUDA_COMPUTE_62
    OR CUDA_COMPUTE_70
    OR CUDA_COMPUTE_72
    OR CUDA_COMPUTE_75
    )
    SET(FALLBACK OFF)
ELSE()
    SET(FALLBACK ON)
ENDIF()

LIST(LENGTH COMPUTES_DETECTED_LIST COMPUTES_LEN)
IF(${COMPUTES_LEN} EQUAL 0 AND ${FALLBACK})
    MESSAGE(STATUS "You can use -DCOMPUTES_DETECTED_LIST=\"AB;XY\" (semicolon separated list of CUDA Compute versions to enable the specified computes")
    MESSAGE(STATUS "Individual compute versions flags are also available under CMake Advance options")
    LIST(APPEND COMPUTES_DETECTED_LIST "30" "50" "60" "70")
    MESSAGE(STATUS "No computes detected. Fall back to 30, 50, 60 70")
ENDIF()

LIST(LENGTH COMPUTES_DETECTED_LIST COMPUTES_LEN)
MESSAGE(STATUS "Number of Computes Detected = ${COMPUTES_LEN}")

FOREACH(COMPUTE_DETECTED ${COMPUTES_DETECTED_LIST})
    SET(CUDA_COMPUTE_${COMPUTE_DETECTED} ON CACHE BOOL "" FORCE)
ENDFOREACH()

MACRO(SET_COMPUTE VERSION)
    SET(CUDA_GENERATE_CODE_${VERSION} "-gencode arch=compute_${VERSION},code=sm_${VERSION}")
    SET(CUDA_GENERATE_CODE ${CUDA_GENERATE_CODE} ${CUDA_GENERATE_CODE_${VERSION}})
    LIST(APPEND COMPUTE_VERSIONS "${VERSION}")
    ADD_DEFINITIONS(-DCUDA_COMPUTE_${VERSION})
    MESSAGE(STATUS "Setting Compute ${VERSION} to ON")
ENDMACRO(SET_COMPUTE)

# Iterate over compute versions. Create variables and enable computes if needed
FOREACH(VER 20 30 32 35 37 50 52 53 60 61 62 70 72 75)
    OPTION(CUDA_COMPUTE_${VER} "CUDA Compute Capability ${VER}" OFF)
    MARK_AS_ADVANCED(CUDA_COMPUTE_${VER})
    IF(${CUDA_COMPUTE_${VER}})
        SET_COMPUTE(${VER})
    ENDIF()
ENDFOREACH()
