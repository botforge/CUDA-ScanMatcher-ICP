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

#define DEBUG false

/*****************
* Configuration *
*****************/

/*! Block size used for CUDA kernel launch. */
#define blockSize 128

/*! Size of the starting area in simulation space. 
 * FOR SINE TEST: 2.f
 * FOR ELEPHANT OBJ: 
 * FOR BUDDHA OBJ: 1 << 2;
 * FOR WAYMO DATASET: 1 << 5;
*/

#define scene_scale 1 << 3

/***********************************************
* Kernel state (pointers are device pointers) *
***********************************************/

int numObjects;
dim3 threadsPerBlock(blockSize);

glm::vec3 *dev_pos;
glm::vec3 *dev_rgb;

pointcloud* target_pc;
pointcloud* src_pc;

//OCTREE pointer (all octnodes lie in device memory)
Octree* octree;

/******************
* initSimulation *
******************/
/**
* Initialize memory, update some globals
*/
void ScanMatch::initSimulationCPU(int N, std::vector<glm::vec3> coords) {
  numObjects = N;

  //Setup and initialize source and target pointcloud
  src_pc = new pointcloud(false, numObjects, false);
  src_pc->initCPU();
  target_pc = new pointcloud(true, numObjects, false);
  target_pc->initCPU();
}

void ScanMatch::initSimulationGPU(int N , std::vector<glm::vec3> coords) {
  numObjects = N;

  //Setup and initialize source and target pointcloud
  src_pc = new pointcloud(false, numObjects, true);
  src_pc->initGPU(coords);
  target_pc = new pointcloud(true, numObjects, true);
  target_pc->initGPU(coords);
}

void ScanMatch::initSimulationGPUOCTREE(int N , std::vector<glm::vec3> coords) {
  numObjects = N;
  //First create the octree 
  octree = new Octree(glm::vec3(0.f, 0.f, 0.f), 4.f, coords);
  octree->create();
  octree->compact();

  //Extract Final Data from it
  int numElts = octree->stackPointer;
  int numCoords = octree->compactedCoords.size();
  glm::vec3* octoCoords = octree->compactedCoords.data();
  OctNode* octNodePool = octree->octNodePool.data();


	printf("NUM ELTS %d \n", numCoords);
	printf("numOBJECTS %d \n", numObjects);
	
	//Send stuff to device
	glm::vec3* dev_octoCoords;
	glm::vec3* dev_octNodePool;

	cudaMalloc((void**)&dev_octoCoords, numObjects * sizeof(glm::vec3));
	cudaMalloc((void**)&dev_octNodePool, numElts * sizeof(OctNode));
	utilityCore::checkCUDAError("cudaMalloc octor failed", __LINE__);

	cudaMemcpy(dev_octoCoords, octoCoords, numObjects * sizeof(glm::vec3), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_octNodePool, octNodePool, numElts * sizeof(OctNode), cudaMemcpyHostToDevice);
	utilityCore::checkCUDAError("cudaMalloc octor failed", __LINE__);
	
  //Setup and initialize source and target pointcloud
  src_pc = new pointcloud(false, numObjects, true);
  src_pc->initGPU(coords);
  target_pc = new pointcloud(true, numObjects, true);
  target_pc->initGPUWOCTREE(dev_octoCoords);
  //target_pc->initGPU(coords);
}

/******************
* copyPointCloudToVBO *
******************/

/**
* Wrapper for call to the kernCopyboidsToVBO CUDA kernel.
*/
void ScanMatch::copyPointCloudToVBO(float *vbodptr_positions, float *vbodptr_rgb, bool usecpu) {

	if (usecpu) { //IF CPU
	  src_pc->pointCloudToVBOCPU(vbodptr_positions, vbodptr_rgb, scene_scale);
	  target_pc->pointCloudToVBOCPU(vbodptr_positions + 4*numObjects, vbodptr_rgb + 4*numObjects, scene_scale);
	}
	else { //IF GPU
		src_pc->pointCloudToVBOGPU(vbodptr_positions, vbodptr_rgb, scene_scale);
		target_pc->pointCloudToVBOGPU(vbodptr_positions + 4*numObjects, vbodptr_rgb + 4*numObjects, scene_scale);
	}
}


/******************
* stepSimulation *
******************/

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
#if DEBUG
	printf("NEAREST NEIGHBORS \n");
#endif // DEBUG

	ScanMatch::findNNCPU(src_pc, target_pc, dist, indicies, numObjects);
#if DEBUG
	printf("RESHUFFLE\n");
#endif // DEBUG

	ScanMatch::reshuffleCPU(target_pc, indicies, numObjects);

	//2: Find Best Fit Transformation
	glm::mat3 R;
	glm::vec3 t;
	ScanMatch::bestFitTransform(src_pc, target_pc, numObjects, R, t);


	//3: Update each src_point
	glm::vec3* src_dev_pos = src_pc->dev_pos;
	for (int i = 0; i < numObjects; ++i) {
		src_dev_pos[i] = glm::transpose(R) * src_dev_pos[i] + t;
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

#if DEBUG
		printf("IDX: %d - MINDIST %f\n", src_idx, minDist);
		printf("IDX: %d - indicies %d\n", src_idx, idx_minDist);
#endif // DEBUG

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

#if DEBUG
		printf("DEV MATCHES\n");
		utilityCore::printVec3(a->dev_matches[i]);
		printf("DEV POS\n");
		utilityCore::printVec3(a_dev_pos[i]);
#endif // DEBUG
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
	glm::vec3* target_matches = target->dev_matches;

	//1:Calculate centroids and norm src and target
	for (int i = 0; i < N; ++i) {
		src_centroid += src_pos[i];
		target_centroid += target_matches[i];
	}
	src_centroid = src_centroid / glm::vec3(N);
	target_centroid = target_centroid / glm::vec3(N);

#if DEBUG
	printf("SRC CENTROID\n");
	utilityCore::printVec3(src_centroid);
	printf("TARGET CENTROID\n");
	utilityCore::printVec3(target_centroid);
#endif // DEBUG

	for (int j = 0; j < N; ++j) {
		src_norm[j] = src_pos[j]  - src_centroid;
		target_norm[j] = target_matches[j] - target_centroid;
#if DEBUG
		printf("SRC NORM IDX %d\n", j);
		utilityCore::printVec3(src_norm[j]);
		printf("TARGET NORM IDX %d\n", j);
		utilityCore::printVec3(target_norm[j]);
#endif // DEBUG
	}

	//1:Multiply src.T (3 x N) by target (N x 3) = H (3 x 3)
	float H[3][3] = { 0 };
	for (int i = 0; i < N; ++i) { //3 x N by N x 3 matmul
		for (int out_row = 0; out_row < 3; out_row++) {
			for (int out_col = 0; out_col < 3; out_col++) {
				H[out_row][out_col] += src_norm[i][out_row] * target_norm[i][out_col];
			}
		}
	}
	
#if DEBUG
	printf("H MATRIX ======================================================\n");
    std::cout << H[0][0] << " " << H[1][0] << " " << H[2][0] << " " << std::endl;
    std::cout << H[0][1] << " " << H[1][1] << " " << H[2][1] << " " << std::endl;
    std::cout << H[0][2] << " " << H[1][2] << " " << H[2][2] << " " << std::endl;
	printf("======================================================\n");
#endif // DEBUG

	//2:calculate SVD of H to get U, S & V
	float U[3][3] = { 0 };
	float S[3][3] = { 0 };
	float V[3][3] = { 0 };
	svd(H[0][0], H[0][1], H[0][2], H[1][0], H[1][1], H[1][2], H[2][0], H[2][1], H[2][2],
		U[0][0], U[0][1], U[0][2], U[1][0], U[1][1], U[1][2], U[2][0], U[2][1], U[2][2],
		S[0][0], S[0][1], S[0][2], S[1][0], S[1][1], S[1][2], S[2][0], S[2][1], S[2][2],
		V[0][0], V[0][1], V[0][2], V[1][0], V[1][1], V[1][2], V[2][0], V[2][1], V[2][2]
		);
	glm::mat3 matU(glm::vec3(U[0][0], U[1][0], U[2][0]), glm::vec3(U[0][1], U[1][1], U[2][1]), glm::vec3(U[0][2], U[1][2], U[2][2]));
	glm::mat3 matV(glm::vec3(V[0][0], V[0][1], V[0][2]), glm::vec3(V[1][0], V[1][1], V[1][2]), glm::vec3(V[2][0], V[2][1], V[2][2]));

#if DEBUG
	printf("U MATRIX\n");
	utilityCore::printMat3(matU);
	printf("V MATRIX\n");
	utilityCore::printMat3(matV);
#endif // DEBUG

	//2:Rotation Matrix and Translation Vector
	R = (matU * matV);
	t = target_centroid - R * (src_centroid);

#if DEBUG
	printf("ROTATION\n");
	utilityCore::printMat3(R);
	printf("TRANSLATION\n");
	utilityCore::printVec3(t);
#endif // DEBUG
}

/******************
* GPU NAIVE SCANMATCHING *
******************/

__global__ void kernUpdatePositions(glm::vec3* src_pos, glm::mat3 R, glm::vec3 t, int N) {
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  if (idx < N) {
	  src_pos[idx] = (R) * src_pos[idx] + t;
  }
}

/**
 * Main Algorithm for Running ICP on the GPU
 * Finds homogenous transform between src_pc and target_pc 
*/
void ScanMatch::stepICPGPU_NAIVE() {

	//cudaMalloc dist and indicies
	float* dist;
	int* indicies;

	cudaMalloc((void**)&dist, numObjects * sizeof(float));
	utilityCore::checkCUDAError("cudaMalloc dist failed", __LINE__);

	cudaMalloc((void**)&indicies, numObjects * sizeof(int));
	utilityCore::checkCUDAError("cudaMalloc indicies failed", __LINE__);
	cudaMemset(dist, 0, numObjects * sizeof(float));
	cudaMemset(indicies, -1, numObjects * sizeof(int));

	//1: Find Nearest Neigbors and Reshuffle
	ScanMatch::findNNGPU_NAIVE(src_pc, target_pc, dist, indicies, numObjects);
	cudaDeviceSynchronize();
	ScanMatch::reshuffleGPU(target_pc, indicies, numObjects);
	cudaDeviceSynchronize();
	//2: Find Best Fit Transformation
	glm::mat3 R;
	glm::vec3 t;
	ScanMatch::bestFitTransformGPU(src_pc, target_pc, numObjects, R, t);
	cudaDeviceSynchronize();

	//3: Update each src_point via Kernel Call
	dim3 fullBlocksPerGrid((numObjects + blockSize - 1) / blockSize);
	kernUpdatePositions<<<fullBlocksPerGrid, blockSize>>>(src_pc->dev_pos, R, t, numObjects);

	//cudaFree dist and indicies
	cudaFree(dist);
	cudaFree(indicies);
}

/*
 * Parallely compute NN for each point in the pointcloud
 */
__global__ void kernNNGPU_NAIVE(glm::vec3* src_pos, glm::vec3* target_pos, float* dist, int* indicies, int N) {
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  if (idx < N) {
	  float minDist = INFINITY;
	  float idx_minDist = -1;
	  glm::vec3 src_pt = src_pos[idx];
	  for (int tgt_idx = 0; tgt_idx < N; ++tgt_idx) { //Iterate through each tgt & find closest
		  glm::vec3 tgt_pt = target_pos[tgt_idx];
		  float d = glm::distance(src_pt, tgt_pt);
		  //float d = sqrtf(powf((tgt_pt.x - src_pt.x), 2.f) + powf((tgt_pt.y - src_pt.y), 2.f) + powf((tgt_pt.z - src_pt.z), 2.f));
		  if (d < minDist) {
			  minDist = d;
			  idx_minDist = tgt_idx;
		  }
	  }
	  dist[idx] = minDist;
	  indicies[idx] = idx_minDist;
  }
}

/**
 * Finds Nearest Neighbors of target pc in src pc
 * @args: src, target -> PointClouds w/ filled dev_pos IN GPU
 * @returns: 
	* dist -> N array -> ith index = dist(src[i], closest_point in target) (on GPU)
	* indicies -> N array w/ ith index = index of the closest point in target to src[i] (on GPU)
*/
void ScanMatch::findNNGPU_NAIVE(pointcloud* src, pointcloud* target, float* dist, int *indicies, int N) {
	//Launch a kernel (paralellely compute NN for each point)
	dim3 fullBlocksPerGrid((N + blockSize - 1) / blockSize);
	kernNNGPU_NAIVE<<<fullBlocksPerGrid, blockSize>>>(src->dev_pos, target->dev_pos, dist, indicies, N);
}

/*
 * Parallely reshuffle pos by indicies and fill matches
 */
__global__ void kernReshuffleGPU(glm::vec3* pos, glm::vec3* matches, int *indicies, int N) {
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  if (idx < N) {
	  //matches[idx] = pos[idx];
	  matches[idx] = pos[indicies[idx]];
  }
}

/**
 * Reshuffles pointcloud a as per indicies, puts these in dev_matches
 * NOT ONE TO ONE SO NEED TO MAKE A COPY!
*/
void ScanMatch::reshuffleGPU(pointcloud* a, int* indicies, int N) {
	dim3 fullBlocksPerGrid((N + blockSize - 1) / blockSize);
	kernReshuffleGPU<<<fullBlocksPerGrid, blockSize>>>(a->dev_pos, a->dev_matches, indicies, N);
}

__global__ void kernComputeNorms(glm::vec3* src_norm, glm::vec3* target_norm, glm::vec3* pos, glm::vec3* matches, glm::vec3 pos_centroid, glm::vec3 matches_centroid, int N) {
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  if (idx < N) {
	  src_norm[idx] = pos[idx] - pos_centroid;
	  target_norm[idx] = matches[idx] - matches_centroid;
  }
}

__global__ void kernComputeHarray(glm::mat3* Harray, glm::vec3* src_norm, glm::vec3* target_norm, int N) {
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  if (idx < N) {
	  Harray[idx] = glm::mat3(glm::vec3(src_norm[idx]) * target_norm[idx].x,
		  glm::vec3(src_norm[idx]) * target_norm[idx].y,
		  glm::vec3(src_norm[idx]) * target_norm[idx].z);
 }
}

/**
 * Calculates transform T that maps from src to target
 * Assumes dev_matches is filled for target
*/
void ScanMatch::bestFitTransformGPU(pointcloud* src, pointcloud* target, int N, glm::mat3 &R, glm::vec3 &t){

	glm::vec3* src_norm;
	glm::vec3* target_norm;
	glm::mat3* Harray;

	//cudaMalloc Norms and Harray
	cudaMalloc((void**)&src_norm, N * sizeof(glm::vec3));
	cudaMalloc((void**)&target_norm, N * sizeof(glm::vec3));
	cudaMalloc((void**)&Harray, N * sizeof(glm::mat3));
	cudaMemset(Harray, 0, N * sizeof(glm::mat3));


	//Thrust device pointers for calculating centroids
	thrust::device_ptr<glm::vec3> src_thrustpos(src->dev_pos);
	thrust::device_ptr<glm::vec3> target_thrustmatches(target->dev_matches);
	thrust::device_ptr<glm::mat3> harray_thrust = thrust::device_pointer_cast(Harray);

	//1: Calculate centroids
	glm::vec3 src_centroid(0.f);
	glm::vec3 target_centroid(0.f);
	src_centroid = glm::vec3(thrust::reduce(src_thrustpos, src_thrustpos + N, glm::vec3(0.f), thrust::plus<glm::vec3>()));
	cudaDeviceSynchronize();
	target_centroid = glm::vec3(thrust::reduce(target_thrustmatches, target_thrustmatches + N, glm::vec3(0.f), thrust::plus<glm::vec3>()));
	cudaDeviceSynchronize();
	src_centroid /= glm::vec3(N);
	target_centroid /= glm::vec3(N);

#if DEBUG
	printf("SRC CENTROID\n");
	utilityCore::printVec3(src_centroid);
	printf("TARGET CENTROID\n");
	utilityCore::printVec3(target_centroid);
#endif // DEBUG

	//2: Compute Norm via Kernel Call
	dim3 fullBlocksPerGrid((N + blockSize - 1) / blockSize);
	kernComputeNorms<<<fullBlocksPerGrid, blockSize>>>(src_norm, target_norm, src->dev_pos, target->dev_matches, src_centroid, target_centroid, N);
	cudaDeviceSynchronize();
	utilityCore::checkCUDAError("Compute Norms Failed", __LINE__);

	//3:Multiply src.T (3 x N) by target (N x 3) = H (3 x 3) via a kernel call
	kernComputeHarray<<<fullBlocksPerGrid, blockSize>>>(Harray, src_norm, target_norm, N);
	cudaDeviceSynchronize();
	utilityCore::checkCUDAError("Compute HARRAY Failed", __LINE__);

	/*
	glm::mat3 H = thrust::reduce(harray_thrust, harray_thrust + N, glm::mat3(0.f), thrust::plus<glm::mat3>());
	cudaThreadSynchronize();
	*/

	glm::mat3* Hcpu = new glm::mat3[N];
	cudaMemcpy(Hcpu, Harray, N * sizeof(glm::mat3), cudaMemcpyDeviceToHost);
	utilityCore::checkCUDAError("REDUCE HARRAY Failed", __LINE__);
	cudaDeviceSynchronize();
	glm::mat3 H(0.f);
	for (int i = 0; i < N; ++i) {
		H += Hcpu[i];
	}
	//4:Calculate SVD of H to get U, S & V
	float U[3][3] = { 0 };
	float S[3][3] = { 0 };
	float V[3][3] = { 0 };
	svd(H[0][0], H[0][1], H[0][2], H[1][0], H[1][1], H[1][2], H[2][0], H[2][1], H[2][2],
		U[0][0], U[0][1], U[0][2], U[1][0], U[1][1], U[1][2], U[2][0], U[2][1], U[2][2],
		S[0][0], S[0][1], S[0][2], S[1][0], S[1][1], S[1][2], S[2][0], S[2][1], S[2][2],
		V[0][0], V[0][1], V[0][2], V[1][0], V[1][1], V[1][2], V[2][0], V[2][1], V[2][2]
		);
	glm::mat3 matU(glm::vec3(U[0][0], U[1][0], U[2][0]), glm::vec3(U[0][1], U[1][1], U[2][1]), glm::vec3(U[0][2], U[1][2], U[2][2]));
	glm::mat3 matV(glm::vec3(V[0][0], V[0][1], V[0][2]), glm::vec3(V[1][0], V[1][1], V[1][2]), glm::vec3(V[2][0], V[2][1], V[2][2]));

	//5:Rotation Matrix and Translation Vector
	R = (matU * matV);
	t = target_centroid - (R) * (src_centroid);

	//cudaMalloc Norms and Harray
	cudaFree(src_norm);
	cudaFree(target_norm); 
	cudaFree(Harray);
}
