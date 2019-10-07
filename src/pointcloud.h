/**
 * @file	  pointcloud.h
 * @brief     Creates a pointcloud (from a file or otherwise) and methods to render it
 * @authors   Dhruv Karthik
 * @date      2019
 */
#pragma once
#include <glm/glm.hpp>
#include <stdio.h>
#include <stdio.h>
#include <cmath>
#include "utilityCore.hpp"

class pointcloud {
public:

	glm::vec3 *dev_pos;
	glm::vec3 *dev_rgb;
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
