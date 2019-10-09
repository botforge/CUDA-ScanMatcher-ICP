/**
 * @file	  pointcloud.h
 * @brief     Create, Render and Destroy a pointcloud
 * @authors   Dhruv Karthik
 * @date      2019
 */
#pragma once
#include <glm/glm.hpp>
#include <stdio.h>
#include <stdio.h>
#include <cmath>
#include <cuda.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/device_vector.h>
#include "utilityCore.hpp"

class pointcloud {
public:

	glm::vec3 *dev_pos;
	glm::vec3 *dev_matches;
	glm::vec3 *dev_rgb;

	glm::vec3 *dev_tempcpupos;
	glm::vec3 *dev_tempcpurgb;
	bool isTarget; 
	int N; //Number of points
	
	pointcloud();
	pointcloud(bool target, int num_points);

	//CPU METHODS
	void initCPU();
	void buildSinusoidCPU();
    void pointCloudToVBOCPU(float *vbodptr_positions, float *vbodptr_rgb, float s_scale);

	//GPU METHODS
	void initGPU();
	void buildSinusoidGPU();
    void pointCloudToVBOGPU(float *vbodptr_positions, float *vbodptr_rgb, float s_scale);

	~pointcloud();
};
