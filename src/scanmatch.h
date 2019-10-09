#pragma once

#include <stdio.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/device_vector.h>
#include <cuda.h>
#include <cmath>
#include <vector>
#include "pointcloud.h"
#include "utilityCore.hpp"


namespace ScanMatch {
    void copyPointCloudToVBO(float *vbodptr_positions, float *vbodptr_rgb);
    void endSimulation();
    void unitTest();

	//CPU SCANMATCHING
    void initSimulationCPU(int N);
	void stepICPCPU();
	void findNNCPU(pointcloud* src, pointcloud* target, float* dist, int* indicies, int N);
	void reshuffleCPU(pointcloud* a, int* indicies, int N);
	void bestFitTransform(pointcloud* src, pointcloud* target, int N, glm::mat3 &R, glm::vec3 &t);

	//GPU_NAIVE SCANMATCHING
    void initSimulationGPU(int N);
} 