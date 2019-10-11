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
	bool isGPU;
	int N; //Number of points
	
	pointcloud();
	pointcloud(bool target, int num_points);
	pointcloud(bool target, int num_points, bool isGPU);

	//CPU METHODS
	void initCPU();
	void initCPU(std::vector<glm::vec3> coords);
	void buildSinusoidCPU();
    void pointCloudToVBOCPU(float *vbodptr_positions, float *vbodptr_rgb, float s_scale);

	//GPU METHODS
	void initGPU();
	void initGPU(std::vector<glm::vec3> coords);
	void buildSinusoidGPU();
	void buildCoordsGPU(std::vector<glm::vec3> coords);
    void pointCloudToVBOGPU(float *vbodptr_positions, float *vbodptr_rgb, float s_scale);

	
	void initGPUWOCTREE(glm::vec3* dev_octocoords);
	~pointcloud();
};
