#pragma once
#include "pointcloud.h"
#include <cuda.h>

#define blockSize 128
#define checkCUDAErrorWithLine(msg) checkCUDAError(msg, __LINE__)
#define ORANGE glm::vec3(1.0f, 0.5f, 0.7f)
#define GREEN glm::vec3(0.7f, 0.3f, 0.9f)

__host__ __device__ unsigned int hash(unsigned int a) {
  a = (a + 0x7ed55d16) + (a << 12);
  a = (a ^ 0xc761c23c) ^ (a >> 19);
  a = (a + 0x165667b1) + (a << 5);
  a = (a + 0xd3a2646c) ^ (a << 9);
  a = (a + 0xfd7046c5) + (a << 3);
  a = (a ^ 0xb55a4f09) ^ (a >> 16);
  return a;
}

__host__ __device__ glm::vec3 generateRandomVec3(int index) {
  thrust::default_random_engine rng(hash((int)(index)));
  thrust::uniform_real_distribution<float> unitDistrib(0.f, 0.1f);

  return glm::vec3((float)unitDistrib(rng), (float)unitDistrib(rng), (float)unitDistrib(rng));
}

__global__ void kernRotTrans(glm::vec3* pos, glm::mat4 rotationMat, glm::vec3 t, int N) {
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  if (idx < N) {
	glm::vec3 rotated = glm::vec3(rotationMat * glm::vec4(pos[idx], 1.0f));
	pos[idx] = rotated + t;
  }
}

__global__ void kernSetRGB(glm::vec3* rgb, glm::vec3 color, int N) {
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  if (idx < N) {
	rgb[idx].x = color.r;
	rgb[idx].y = color.g;
	rgb[idx].z = color.b;
  }
}

/**
* Generates Sinusoids for Target
*/
__global__ void kernBuildTargetSinusoid(glm::vec3* pos, glm::vec3* rgb, glm::mat4 rotationMat, float y_interval, int N) {
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  if (idx < N) {
    glm::vec3 t = generateRandomVec3(idx);
	//glm::vec3 t(0.0f, 0.0f, 0.0f);
    pos[idx].x = 0.7f;
    pos[idx].y = idx * y_interval;
    pos[idx].z = sinf(idx*y_interval);
	pos[idx] = pos[idx] + t;
	rgb[idx].x = 0.f;
	rgb[idx].y = 0.9f;
	rgb[idx].z = 0.2f;
  }
}

/**
* Generates Sinusoids for SRC
*/
__global__ void kernBuildSrcSinusoid(glm::vec3* pos, glm::vec3* rgb, glm::mat4 rotationMat, float y_interval, int N) {
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  if (idx < N) {
    glm::vec3 t = generateRandomVec3(idx);
	//glm::vec3 t(0.1f, 0.0f, 0.0f);
    pos[idx].x = 0.7f;
    pos[idx].y = idx * y_interval;
    pos[idx].z = sinf(idx*y_interval);
	glm::vec3 rotated = glm::vec3(rotationMat * glm::vec4(pos[idx], 1.0f));
	pos[idx] = rotated + t;
	rgb[idx].x = 1.f;
	rgb[idx].y = 0.5f;
	rgb[idx].z = 0.1f;
  }
}

/**
* Copy the Pointcloud Positions into the VBO so that they can be drawn by OpenGL.
*/
__global__ void kernCopyPositionsToVBO(int N, glm::vec3 *pos, float *vbo, float s_scale, int vbo_offset) {
  int index = threadIdx.x + (blockIdx.x * blockDim.x);

  float c_scale = -1.0f / s_scale;

  if (index < N) {
	index += vbo_offset;
	vbo[4 * index + 0] = pos[index].x * c_scale - 0.f;
    vbo[4 * index + 1] = (pos[index].y-6.f) * c_scale;
    vbo[4 * index + 2] = pos[index].z * c_scale;
    vbo[4 * index + 3] = 1.0f;
  }
}

/**
* Copy the Pointcloud RGB's into the VBO so that they can be drawn by OpenGL.
*/
__global__ void kernCopyRGBToVBO(int N, glm::vec3 *rgb, float *vbo, float s_scale, int vbo_offset) {
  int index = threadIdx.x + (blockIdx.x * blockDim.x);

  if (index < N) {
	index += vbo_offset;
    vbo[4 * index + 0] = rgb[index].x + 0.3f;
    vbo[4 * index + 1] = rgb[index].y + 0.3f;
    vbo[4 * index + 2] = rgb[index].z + 0.3f;
    vbo[4 * index + 3] = 1.0f;
  }
}

pointcloud::pointcloud(): isTarget(false), N(500){
}

pointcloud::pointcloud(bool target, int numPoints): isTarget(target), N(numPoints){
}

pointcloud::pointcloud(bool target, int numPoints, bool gpu): isTarget(target), N(numPoints), isGPU(gpu){
}

/******************
* CPU Methods *
******************/

/**
 * Initialize and fills dev_pos and dev_rgb array in CPU
*/
void pointcloud::initCPU() {
	dev_pos = new glm::vec3[N];
	dev_matches = new glm::vec3[N];
	dev_rgb = new glm::vec3[N];
	buildSinusoidCPU();
}

/**
 * Populates dev_pos with a 3D Sinusoid (with or without Noise) on the CPU
*/
void pointcloud::buildSinusoidCPU() {
	float y_interval = (2.5 * PI) / N;

	//RNG (Predetermine Rotation)
	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<float> u01(0, 0.1);
	glm::vec3 r(0.1f, 0.2f, 0.0f);
	float angle = -1.f * PI;
	//float angle = 0.f;
	glm::mat4 rotationMat = glm::rotate(angle, r);

	for (int idx = 0; idx < N; idx++) {
		glm::vec3 pos;
		glm::vec3 rgb;
		if (isTarget) { //Leave Original Pointcloud
			pos = glm::vec3(0.7f, idx*y_interval, sin(idx*y_interval));
			rgb = glm::vec3(0.f, 0.9f, 0.2f);

			//Create & Apply Translation for Pointcloud Effect
			glm::vec3 t(u01(e2), u01(e2), u01(e2));
			//glm::vec3 t(0.0, 0.0, 0.0);
			pos += t;
		}
		else { //Add Multiplicative Noise, Rotation, Translation to OG
			pos = glm::vec3(0.7f, idx*y_interval, sin(idx*y_interval));
			rgb = glm::vec3(1.0f, 0.5f, 0.1f);

			//Create Translation and Rotation
			//glm::vec3 t(0.1f, 0.f, 0.f);
			glm::vec3 t(u01(e2), u01(e2), u01(e2));

			//Apply Translation and Rotation 
			glm::vec3 rotated = glm::vec3(rotationMat * glm::vec4(pos, 1.0));
			pos = rotated + t;
		}
#if DEBUG
		printf("IDX %d\n", idx);
		utilityCore::printVec3(pos);
#endif
		dev_pos[idx] = pos;
		dev_rgb[idx] = rgb;
	}
#if DEBUG
	printf("=================================================\n");
#endif
}

/**
 * Copies dev_pos and dev_rgb into the VBO in the CPU implementation
 * This assumes that dev_pos is already filled but is on CPU
 * REALLY WACK WAY TO DO IT
*/
void pointcloud::pointCloudToVBOCPU(float *vbodptr_positions, float *vbodptr_rgb, float s_scale) {
	glm::vec3* tempPos;
	glm::vec3 * tempRGB;
	int vbo_offset = isTarget ? 0 : 0;

	//Malloc Temporary Buffers
	cudaMalloc((void**)&tempPos, N * sizeof(glm::vec3));
	cudaMalloc((void**)&tempRGB, N * sizeof(glm::vec3));
	utilityCore::checkCUDAErrorWithLine("cudaMalloc Pointcloud failed!");

	//Memcpy dev_pos and dev_rgb into temporary buffers
	cudaMemcpy(tempPos, dev_pos, N * sizeof(glm::vec3), cudaMemcpyHostToDevice);
	cudaMemcpy(tempRGB, dev_rgb, N * sizeof(glm::vec3), cudaMemcpyHostToDevice);
	utilityCore::checkCUDAErrorWithLine("cudaMemcpy Pointcloud failed!");

	//Launching Kernels
	dim3 fullBlocksPerGrid((N + blockSize - 1) / blockSize);
	kernCopyPositionsToVBO << <fullBlocksPerGrid, blockSize >> >(N, tempPos, vbodptr_positions, s_scale, vbo_offset);
	kernCopyRGBToVBO << <fullBlocksPerGrid, blockSize >> >(N, tempRGB, vbodptr_rgb, s_scale, vbo_offset);
	utilityCore::checkCUDAErrorWithLine("copyPointCloudToVBO failed!");
	cudaDeviceSynchronize();

	//Now Flipping original pointer to device so we don't crash on termination
	dev_tempcpupos = tempPos;
	dev_tempcpurgb = tempRGB;
}

/******************
* GPU Methods *
******************/

/**
 * Initialize and fills dev_pos and dev_rgb array in CPU
*/
void pointcloud::initGPU() {
	dim3 fullBlocksPerGrid((N + blockSize - 1) / blockSize);
	
	//cudaMalloc position, matches & rgb arrays
	cudaMalloc((void**)&dev_pos, N * sizeof(glm::vec3));
	utilityCore::checkCUDAErrorWithLine("cudaMalloc dev_pos failed");

	cudaMalloc((void**)&dev_matches, N * sizeof(glm::vec3));
	utilityCore::checkCUDAErrorWithLine("cudaMalloc dev_matches failed");

	cudaMalloc((void**)&dev_rgb, N * sizeof(glm::vec3));
	utilityCore::checkCUDAErrorWithLine("cudaMalloc dev_rgb failed");
	buildSinusoidGPU();
}

void pointcloud::initGPU(std::vector<glm::vec3> coords) {
	dim3 fullBlocksPerGrid((N + blockSize - 1) / blockSize);
	
	//cudaMalloc position, matches & rgb arrays
	cudaMalloc((void**)&dev_pos, N * sizeof(glm::vec3));
	utilityCore::checkCUDAErrorWithLine("cudaMalloc dev_pos failed");

	cudaMalloc((void**)&dev_matches, N * sizeof(glm::vec3));
	utilityCore::checkCUDAErrorWithLine("cudaMalloc dev_matches failed");

	cudaMalloc((void**)&dev_rgb, N * sizeof(glm::vec3));
	utilityCore::checkCUDAErrorWithLine("cudaMalloc dev_rgb failed");
	printf("SIZE IS: %d \n", coords.size());
	if (coords.size() > 0) {
		buildCoordsGPU(coords);
	}
	else {
		buildSinusoidGPU();
	}
}

void pointcloud::buildCoordsGPU(std::vector<glm::vec3> coords) {
	dim3 fullBlocksPerGrid((N + blockSize - 1) / blockSize);
	glm::vec3* coordPos = &coords[0];
	cudaMemcpy(dev_pos, coordPos, N * sizeof(glm::vec3), cudaMemcpyHostToDevice);
	if (isTarget) {
		kernSetRGB<<<fullBlocksPerGrid, blockSize>>>(dev_rgb, GREEN, N);
	}
	else {
		kernSetRGB<<<fullBlocksPerGrid, blockSize>>>(dev_rgb, ORANGE, N);
		float angle = 0.1 * PI;
		//float angle = 0.f;
		glm::vec3 axis(1.f, 0.f, 0.f);
		//glm::vec3 t(9.0f, 0.f, 0.f);
		/glm::vec3 t(0.8f, 0.f, 0.f);
		glm::mat4 rotationMatrix = glm::rotate(angle, axis);
		kernRotTrans << <fullBlocksPerGrid, blockSize >> > (dev_pos, rotationMatrix, t, N);
	}
}

/**
 * Populates dev_pos with a 3D Sinusoid (with or without Noise) on the GPU
 * Fills dev_pos & dev_rgb
*/
void pointcloud::buildSinusoidGPU() {
	dim3 fullBlocksPerGrid((N + blockSize - 1) / blockSize);
	float y_interval = (2.5 * PI) / N;
	glm::vec3 r(0.f, 1.0f, 1.0f);
	float angle = -0.7f * PI;
	//float angle = 0.0f;
	glm::mat4 rotationMat = glm::rotate(angle, r);
	
	if (isTarget) {
		kernBuildTargetSinusoid<<<fullBlocksPerGrid, blockSize>>>(dev_pos, dev_rgb, rotationMat, y_interval, N);
	}
	else {
		kernBuildSrcSinusoid<<<fullBlocksPerGrid, blockSize>>>(dev_pos, dev_rgb, rotationMat, y_interval, N);
	}
}

void pointcloud::pointCloudToVBOGPU(float *vbodptr_positions, float *vbodptr_rgb, float s_scale) {
	int vbo_offset = isTarget ? 0 : 0;

	//Launching Kernels
	dim3 fullBlocksPerGrid((N + blockSize - 1) / blockSize);
	kernCopyPositionsToVBO<<<fullBlocksPerGrid, blockSize >>>(N, dev_pos, vbodptr_positions, s_scale, vbo_offset);
	kernCopyRGBToVBO <<<fullBlocksPerGrid, blockSize >>>(N, dev_rgb, vbodptr_rgb, s_scale, vbo_offset);
	utilityCore::checkCUDAErrorWithLine("copyPointCloudToVBO failed!");
	cudaDeviceSynchronize();
}


pointcloud::~pointcloud() {
	if (isGPU) {
		cudaFree(dev_pos);
		cudaFree(dev_rgb);
	}
	else {
		cudaFree(dev_tempcpupos);
		cudaFree(dev_tempcpurgb);
	}
}
