/**
 * @file glm_device.h
 * @brief GLM with device function support for HIP/CUDA
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 * @author Jeff Daily <jeff.daily@amd.com>
 *
 * Include this header instead of <glm/glm.hpp> when GLM is used in device code.
 * On HIP, this enables __host__ __device__ qualifiers on GLM math functions by
 * defining __CUDACC__ and CUDA_VERSION before including GLM.
 *
 * IMPORTANT: Include Thrust headers BEFORE this header. rocThrust's backend
 * selection checks __CUDACC__ before __HIP__, so defining __CUDACC__ before
 * including Thrust would select the CUDA backend incorrectly.
 */
#pragma once

#if defined(USE_HIP) || defined(__HIP_PLATFORM_AMD__)
// GLM 0.9.9 uses __CUDACC__ + CUDA_VERSION>=7000 to apply __host__ __device__
// qualifiers. Define these so GLM treats hipcc like nvcc.
#ifndef __CUDACC__
#define __CUDACC__ 1
#endif
#ifndef CUDA_VERSION
#define CUDA_VERSION 8000
#endif
#ifndef GLM_FORCE_CUDA
#define GLM_FORCE_CUDA
#endif
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
