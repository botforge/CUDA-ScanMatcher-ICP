/**
 * @file	  pointcloud.h
 * @brief     Creates a pointcloud
 * @authors   Dhruv Karthik
 * @date      2019
 */
#pragma once
#include <glm/glm.hpp>
#include <stdio.h>
#include <stdio.h>

class pointcloud {
public:
	glm::vec3 *dev_pos;
	glm::vec3 *dev_rgb;
	
	pointcloud();
	~pointcloud();
};

