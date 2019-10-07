#pragma once
#include "pointcloud.h"
#include <cuda.h>

#define blockSize 128
#define checkCUDAErrorWithLine(msg) checkCUDAError(msg, __LINE__)

pointcloud::pointcloud(): isTarget(false), N(500){
	dev_pos = new glm::vec3[500];
	dev_rgb = new glm::vec3[500];
}

pointcloud::pointcloud(bool target, int numPoints): isTarget(target), N(numPoints){
	dev_pos = new glm::vec3[N];
	dev_rgb = new glm::vec3[N];
}


/******************
* CPU Methods *
******************/

/**
 * Initialize and fills dev_pos and dev_rgb array in CPU
*/
void pointcloud::initCPU() {
	buildSinusoidCPU();
}

/**
 * Populates dev_pos with a 3D Sinusoid (with or without Noise) on the CPU
*/
void pointcloud::buildSinusoidCPU() {
	float y_interval = 2.5 * PI / N;
	for (int idx = 0; idx < N; idx++) {
		dev_pos[idx] = glm::vec3(0.5f, idx*y_interval, sin(idx*y_interval));
		dev_rgb[idx] = glm::vec3(0.1f, 0.8f, 0.5f);
	}
}

/**
 * Copies dev_pos and dev_rgb into the VBO in the CPU implementation
 * This assumes that dev_pos is already filled but is on CPU
 * REALLY HACKY WAY TO DO IT
*/
void pointcloud::pointCloudToVBOCPU(float *vbodptr_positions, float *vbodptr_rgb, float s_scale) {
	glm::vec3* tempPos;
	glm::vec3 * tempRGB;

	//Malloc Temporary Buffers
	cudaMalloc((void**)&tempPos, N * sizeof(glm::vec3));
	cudaMalloc((void**)&tempRGB, N * sizeof(glm::vec3));
	utilityCore::checkCUDAErrorWithLine("cudaMalloc Pointcloud failed!");

	//Memcpy dev_pos and dev_rgb into temporary buffers
	cudaMemcpy(tempPos, dev_pos, N * sizeof(glm::vec3), cudaMemcpyHostToDevice);
	cudaMemcpy(tempRGB, dev_rgb, N * sizeof(glm::vec3), cudaMemcpyHostToDevice);
	utilityCore::checkCUDAErrorWithLine("cudaMemcpy Pointcloud failed!");

	//Now on Device
	dev_pos = tempPos;
	dev_rgb = tempRGB;
}

pointcloud::~pointcloud() {
}
