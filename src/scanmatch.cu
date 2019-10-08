#define GLM_FORCE_CUDA
#include <stdio.h>
#include <cuda.h>
#include <cmath>
#include <glm/glm.hpp>
#include "utilityCore.hpp"
#include "scanmatch.h"

#define MAX_ICP_ITERS 1000
#define EPSILON 0.001

// LOOK-2.1 potentially useful for doing grid-based neighbor search
#ifndef imax
#define imax( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef imin
#define imin( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

#define checkCUDAErrorWithLine(msg) utilityCore::checkCUDAError(msg, __LINE__)

/*****************
* Configuration *
*****************/

/*! Block size used for CUDA kernel launch. */
#define blockSize 128

/*! Size of the starting area in simulation space. 
 * FOR SINE TEST: 2.f
 * FOR ELEPHANT OBJ: 
 * FOR WAYMO DATASET: 
*/

#define scene_scale 2.f

/***********************************************
* Kernel state (pointers are device pointers) *
***********************************************/

int numObjects;
dim3 threadsPerBlock(blockSize);

glm::vec3 *dev_pos;
glm::vec3 *dev_rgb;

pointcloud* target_pc;
pointcloud* src_pc;

/******************
* initSimulation *
******************/

__host__ __device__ unsigned int hash(unsigned int a) {
  a = (a + 0x7ed55d16) + (a << 12);
  a = (a ^ 0xc761c23c) ^ (a >> 19);
  a = (a + 0x165667b1) + (a << 5);
  a = (a + 0xd3a2646c) ^ (a << 9);
  a = (a + 0xfd7046c5) + (a << 3);
  a = (a ^ 0xb55a4f09) ^ (a >> 16);
  return a;
}

/**
* LOOK-1.2 - this is a typical helper function for a CUDA kernel.
* Function for generating a random vec3.
*/
__host__ __device__ glm::vec3 generateRandomVec3(float time, int index) {
  thrust::default_random_engine rng(hash((int)(index * time)));
  thrust::uniform_real_distribution<float> unitDistrib(-1, 1);

  return glm::vec3((float)unitDistrib(rng), (float)unitDistrib(rng), (float)unitDistrib(rng));
}

/**
* Initialize memory, update some globals
*/
void ScanMatch::initSimulation(int N) {
  numObjects = N;

  //Setup and initialize source and target pointcloud
  src_pc = new pointcloud(false, numObjects);
  src_pc->initCPU();

  target_pc = new pointcloud(true, numObjects);
  target_pc->initCPU();
}

/******************
* copyPointCloudToVBO *
******************/

/**
* Wrapper for call to the kernCopyboidsToVBO CUDA kernel.
*/
void ScanMatch::copyPointCloudToVBO(float *vbodptr_positions, float *vbodptr_rgb) {

  //IF CPU
  src_pc->pointCloudToVBOCPU(vbodptr_positions, vbodptr_rgb, scene_scale);
  target_pc->pointCloudToVBOCPU(vbodptr_positions + 4*numObjects, vbodptr_rgb + 4*numObjects, scene_scale);
}


/******************
* stepSimulation *
******************/

/**
* LOOK-1.2 You can use this as a helper for kernUpdateVelocityBruteForce.
* __device__ code can be called from a __global__ context
* Compute the new velocity on the body with index `iSelf` due to the `N` boids
* in the `pos` and `vel` arrays.
*/
__device__ glm::vec3 computeVelocityChange(int N, int iSelf, const glm::vec3 *pos, const glm::vec3 *vel) {
  // Rule 1: boids fly towards their local perceived center of mass, which excludes themselves
  // Rule 2: boids try to stay a distance d away from each other
  // Rule 3: boids try to match the speed of surrounding boids
  return glm::vec3(0.0f, 0.0f, 0.0f);
}

__global__ void kernUpdateVelocityBruteForce(int N, glm::vec3 *pos,
  glm::vec3 *vel1, glm::vec3 *vel2) {
  // Compute a new velocity based on pos and vel1
  // Clamp the speed
  // Record the new velocity into vel2. Question: why NOT vel1?
}

/**
* LOOK-1.2 Since this is pretty trivial, we implemented it for you.
* For each of the `N` bodies, update its position based on its current velocity.
*/
__global__ void kernUpdatePos(int N, float dt, glm::vec3 *pos, glm::vec3 *vel) {
  // Update position by velocity
  int index = threadIdx.x + (blockIdx.x * blockDim.x);
  if (index >= N) {
    return;
  }
  glm::vec3 thisPos = pos[index];
  thisPos += vel[index] * dt;

  // Wrap the boids around so we don't lose them
  thisPos.x = thisPos.x < -scene_scale ? scene_scale : thisPos.x;
  thisPos.y = thisPos.y < -scene_scale ? scene_scale : thisPos.y;
  thisPos.z = thisPos.z < -scene_scale ? scene_scale : thisPos.z;

  thisPos.x = thisPos.x > scene_scale ? -scene_scale : thisPos.x;
  thisPos.y = thisPos.y > scene_scale ? -scene_scale : thisPos.y;
  thisPos.z = thisPos.z > scene_scale ? -scene_scale : thisPos.z;

  pos[index] = thisPos;
}

__device__ int gridIndex3Dto1D(int x, int y, int z, int gridResolution) {
  return x + y * gridResolution + z * gridResolution * gridResolution;
}

__global__ void kernComputeIndices(int N, int gridResolution,
  glm::vec3 gridMin, float inverseCellWidth,
  glm::vec3 *pos, int *indices, int *gridIndices) {
}

void ScanMatch::endSimulation() {
	src_pc->~pointcloud();
	target_pc->~pointcloud();
}

/******************
* CPU SCANMATCHING *
******************/

/**
 * Main Algorithm for Running ICP on the CPU
 * Finds homogenous transform between src_pc and target_pc 
*/
void ScanMatch::ICPCPU() {
	for (int i = 0; i < MAX_ICP_ITERS; ++i) {
		//1: Find Nearest Neigbors and Reshuffle
		float* dist = new float[numObjects];
		int* indicies = new int[numObjects];
		//ScanMatch::findNNCPU(src_pc, target_pc, dist, indicies, numObjects);
		//reshuffleCPU(target_pc, indicies, numObjects);
	}
}

