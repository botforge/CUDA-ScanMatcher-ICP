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
#define MAX_PTS_PER_OCTANT 1000
#define MULT 10

struct OctNodeGPU {
	int firstChildIdx;
	int data_startIdx;
	int count;
	glm::vec3 center;
	bool isLeaf;
};

struct OctNode {
	int firstChildIdx;
	std::vector<OctNode*> children;
	std::vector<glm::vec3> data;
	std::vector<int> data_idx;
	glm::vec3 center;
	float halfLength;
	bool isLeaf;
};

class Octree {
public:
	OctNode* rootNode;
	std::vector<OctNode*> nodePool;
	std::vector<glm::vec3> coords;
	std::vector<OctNodeGPU> gpuNodePool;
	std::vector<glm::vec3> gpuCoords;
	Octree(glm::vec3 rCenter, float rHalfLength, std::vector<glm::vec3> c);
	void create();
	void insert(octKey currKey, glm::vec3 data, int data_idx);
	void compact();
};
/*
struct OctNode {
	octKey firstChild;
	long long data_startidx;
	long long data_endidx;
	glm::vec3 center;
	float halfLength;
	long long count;
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
*/
