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

typedef unsigned long long octKey;
#define MAX_PTS_PER_OCTANT 50

struct OctNode {
	octKey firstChild;
	unsigned int data_startidx;
	unsigned int data_endidx;
	glm::vec3 center;
	float halfLength;
	int count;
	bool isLeaf;
};

class Octree {
public:
	float rootHalfLength;
	glm::vec3 rootCenter;
	std::vector<OctNode> octNodePool;
	std::vector<glm::vec3> octCoords;
	std::vector<glm::vec3> coords;
	OctNode rootNode;
	std::vector<glm::vec3> compactedCoords;
	int stackPointer;
	Octree(glm::vec3 rCenter, float rHalfLength, std::vector<glm::vec3> c);
	void create();
	void insert(octKey currKey, glm::vec3 data, float halfLength, glm::vec3 center);
	void compact();
};
