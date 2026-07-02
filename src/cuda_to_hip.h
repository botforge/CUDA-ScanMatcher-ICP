/**
 * @file cuda_to_hip.h
 * @brief CUDA-to-HIP compatibility header for ROCm builds
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 * @author Jeff Daily <jeff.daily@amd.com>
 *
 * On ROCm this header maps CUDA runtime and GL-interop symbols to their HIP
 * equivalents. On CUDA it is a pass-through to the CUDA headers.
 *
 * GLM device function support: GLM 0.9.9 uses __CUDACC__ + CUDA_VERSION>=7000
 * to apply __host__ __device__ qualifiers. We define these BEFORE including
 * GLM. rocThrust is designed to work with __HIP__ (defined by hipcc) and will
 * select the HIP backend correctly as long as it sees __HIP__ before we define
 * __CUDACC__. Since Thrust's backend selection happens at include time, we
 * define the GLM macros here (after hip_runtime.h) and rely on include order
 * in source files: include Thrust headers BEFORE glm headers.
 *
 * This port was authored with assistance from Claude, an AI assistant by
 * Anthropic.
 */
#pragma once

#if defined(USE_HIP) || defined(__HIP_PLATFORM_AMD__)

#include <hip/hip_runtime.h>
#include <hip/hip_gl_interop.h>

// Runtime API
#define cudaMalloc                  hipMalloc
#define cudaFree                    hipFree
#define cudaMemcpy                  hipMemcpy
#define cudaMemset                  hipMemset
#define cudaMemcpyHostToDevice      hipMemcpyHostToDevice
#define cudaMemcpyDeviceToHost      hipMemcpyDeviceToHost
#define cudaDeviceSynchronize       hipDeviceSynchronize
#define cudaThreadSynchronize       hipDeviceSynchronize  // deprecated alias

// Error handling
#define cudaError_t                 hipError_t
#define cudaSuccess                 hipSuccess
#define cudaGetLastError            hipGetLastError
#define cudaGetErrorString          hipGetErrorString

// Device properties
#define cudaDeviceProp              hipDeviceProp_t
#define cudaGetDeviceCount          hipGetDeviceCount
#define cudaGetDeviceProperties     hipGetDeviceProperties
#define cudaSetDevice               hipSetDevice

// OpenGL interop (using the modern graphics resource API)
// The old cudaGL* functions are deprecated; HIP uses hipGraphics* API
#define cudaGraphicsResource_t       hipGraphicsResource_t
#define cudaGraphicsGLRegisterBuffer hipGraphicsGLRegisterBuffer
#define cudaGraphicsRegisterFlagsNone hipGraphicsRegisterFlagsNone
#define cudaGraphicsMapResources     hipGraphicsMapResources
#define cudaGraphicsResourceGetMappedPointer hipGraphicsResourceGetMappedPointer
#define cudaGraphicsUnmapResources   hipGraphicsUnmapResources
#define cudaGraphicsUnregisterResource hipGraphicsUnregisterResource

#else  // CUDA path

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#endif
