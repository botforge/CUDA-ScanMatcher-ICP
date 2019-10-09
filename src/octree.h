/**
 * @file      octree.h
 * @brief     Octree implementation to speed up nearest neighbor search
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

class OctNode {
	OctNode* children[2][2][2]; //x, y, z
	std::vector<int> tgt_indicies; //data 
	glm::vec3 midpoint_pos; //global coordinates of midpoint
	bool isLeaf;
};

class Octree {
public:
	OctNode* rootNode; //Actually a dev pointer 
	Octree();
	void create(std::vector<glm::vec3> coords);
};
