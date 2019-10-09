#define GLM_FORCE_CUDA
#include <stdio.h>
#include <cuda.h>
#include <cmath>
#include <glm/glm.hpp>
#include "utilityCore.hpp"
#include "scanmatch.h"
#include "svd3.h"

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
void ScanMatch::stepICPCPU() {
	//1: Find Nearest Neigbors and Reshuffle
	float* dist = new float[numObjects];
	int* indicies = new int[numObjects];
	ScanMatch::findNNCPU(src_pc, target_pc, dist, indicies, numObjects);
	ScanMatch::reshuffleCPU(target_pc, indicies, numObjects);

	//2: Find Best Fit Transformation
	glm::mat3 R;
	glm::vec3 t;
	ScanMatch::bestFitTransform(src_pc, target_pc, numObjects, R, t);

	//3: Update each src_point
	glm::vec3* src_dev_pos = src_pc->dev_pos;
	for (int i = 0; i < numObjects; ++i) {
		src_dev_pos[i] = src_dev_pos[i] + t;
	}
}

/**
 * Finds Nearest Neighbors of target pc in src pc
 * @args: src, target -> PointClouds w/ filled dev_pos
 * @returns: 
	* dist -> N array -> ith index = dist(src[i], closest_point in target)
	* indicies -> N array w/ ith index = index of the closest point in target to src[i]
*/
void ScanMatch::findNNCPU(pointcloud* src, pointcloud* target, float* dist, int *indicies, int N) {
	glm::vec3* src_dev_pos = src->dev_pos;
	glm::vec3* target_dev_pos = target->dev_pos;
	for (int src_idx = 0; src_idx < N; ++src_idx) { //Iterate through each source point
		glm::vec3 src_pt = src_dev_pos[src_idx];
		float minDist = INFINITY;
		int idx_minDist = -1;
		for (int tgt_idx = 0; tgt_idx < N; ++tgt_idx) { //Iterate through each tgt point and find closest
			glm::vec3 tgt_pt = target_dev_pos[tgt_idx];
			float d = glm::distance(src_pt, tgt_pt);
			if (d < minDist) {
				minDist = d;
				idx_minDist = tgt_idx;
			}
		}
		//Update dist and indicies
		dist[src_idx] = minDist;
		indicies[src_idx] = idx_minDist;
	}
}

/**
 * Reshuffles pointcloud a as per indicies, puts these in dev_matches
 * NOT ONE TO ONE SO NEED TO MAKE A COPY!
*/
void ScanMatch::reshuffleCPU(pointcloud* a, int* indicies, int N) {
	glm::vec3 *a_dev_matches = a->dev_matches;
	glm::vec3 *a_dev_pos = a->dev_pos;
	for (int i = 0; i < N; ++i) {
		a_dev_matches[i] = a_dev_pos[indicies[i]];
	}
}

/**
 * Calculates transform T that maps from src to target
 * Assumes dev_matches is filled for target
*/
void ScanMatch::bestFitTransform(pointcloud* src, pointcloud* target, int N, glm::mat3 &R, glm::vec3 &t){
	glm::vec3* src_norm = new glm::vec3[N];
	glm::vec3* target_norm = new glm::vec3[N];
	glm::vec3 src_centroid(0.f);
	glm::vec3 target_centroid(0.f);
	glm::vec3* src_pos = src->dev_pos;
	glm::vec3* target_matches = src->dev_matches;

	//1:Calculate centroids and norm src and target
	for (int i = 0; i < N; ++i) {
		src_centroid += src_pos[i];
		target_centroid += target_matches[i];
	}
	src_centroid = src_centroid / glm::vec3(N);
	target_centroid = target_centroid / glm::vec3(N);
	for (int j = 0; j < N; ++j) {
		src_norm[j] = src_pos[j]  - src_centroid;
		target_norm[j] = target_matches[j] - target_centroid;
	}

	//1:Multiply src.T (3 x N) by target (N x 3) = H (3 x 3)
	float H[3][3] = { 0 };
	for (int i = 0; i < N; ++i) { //3 x N by N x 3 matmul
		for (int out_row = 0; out_row < 3; out_row++) {
			for (int out_col = 0; out_col < 3; out_col++) {
				H[out_row][out_col] += src_norm[i][out_row] + target_norm[i][out_col];
			}
		}
	}

	//2:calculate SVD of H to get U, S & V
	float U[3][3] = { 0 };
	float S[3][3] = { 0 };
	float V[3][3] = { 0 };
	//svd(H[0][0], H[0][1], H[0][2], H[1][0], H[1][1], H[1][2], H[2][0], H[2][1], H[2][2],
		//U[0][0], U[0][1], U[0][2], U[1][0], U[1][1], U[1][2], U[2][0], U[2][1], U[2][2],
		//S[0][0], S[0][1], S[0][2], S[1][0], S[1][1], S[1][2], S[2][0], S[2][1], S[2][2],
		//V[0][0], V[0][1], V[0][2], V[1][0], V[1][1], V[1][2], V[2][0], V[2][1], V[2][2]
		//);
	glm::mat3 matU(glm::vec3(U[0][0], U[1][0], U[2][0]), glm::vec3(U[0][1], U[1][1], U[2][1]), glm::vec3(U[0][2], U[1][2], U[2][2]));
	glm::mat3 matV(glm::vec3(V[0][0], V[0][1], V[0][2]), glm::vec3(V[1][0], V[1][1], V[1][2]), glm::vec3(V[2][0], V[2][1], V[2][2]));

	//2:Rotation Matrix and Translation Vector
	R = glm::transpose(matU * matV);
	t = target_centroid - R * (src_centroid);
}
