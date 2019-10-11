#include "octree.h"

Octree::Octree(glm::vec3 rCenter, float rHalfLength, std::vector<glm::vec3> c){
	coords = c;
	//Initialize RootNode
	rootNode = new OctNode();
	rootNode -> firstChildIdx = 0;
	rootNode ->children = std::vector<OctNode*>();
	rootNode -> data = std::vector<glm::vec3>();
	rootNode -> data_idx = std::vector<int>();
	rootNode -> center = rCenter;
	rootNode -> halfLength = rHalfLength;
	rootNode -> isLeaf = true;
	nodePool.push_back(rootNode);
}

void Octree::create() {
	octKey root = 0;
	for (int i = 0; i < coords.size(); ++i) {
		insert(root, coords[i], i);
	}
}

void Octree::insert(octKey currKey, glm::vec3 data, int data_idx) {
	OctNode* currOctNode = nodePool[currKey];

	//Option 1:Check if we're at a leaf, then add the point
	if (currOctNode->isLeaf) {
		bool hasMaxData = currOctNode->data.size() >= MAX_PTS_PER_OCTANT;
		if (!hasMaxData) { //If we haven't surpassed MAX_PTS_PER_OCTANT, just append the data
			currOctNode->data.push_back(data);
			currOctNode->data_idx.push_back(data_idx);
		}
		else { //We have surpassed MAX_PTS_PER_OCTANT
			//1:Subdivide
			octKey baseNewKey = nodePool.size();
			float newHalfLength = currOctNode->halfLength / 2.f;
			currOctNode->firstChildIdx = baseNewKey;
			currOctNode->isLeaf = false;
			for (int z = 0; z < 2; ++z) {
				for (int y = 0; y < 2; ++y) {
					for (int x = 0; x < 2; ++x) {
						//Update the code
						octKey newKey = baseNewKey + (x + 2 * y + 4 * z);

						//Update the center
						glm::vec3 center = currOctNode->center;
						glm::vec3 newCenter(0.f);
						newCenter.x = center.x + newHalfLength * (x ? 1 : -1);
						newCenter.y = center.y + newHalfLength * (y ? 1 : -1);
						newCenter.z = center.z + newHalfLength * (z ? 1 : -1);

						//Update new entry in octNodePool
						OctNode* newNode = new OctNode();
						newNode->firstChildIdx = 0;
						newNode->center = newCenter;
						newNode->halfLength = newHalfLength;
						newNode->isLeaf = true;
						nodePool.push_back(newNode);

						//Add child to parent
						currOctNode->children.push_back(newNode);
					}
				}
			}
			//2:Redistribute Points
			for (int i = 0; i < currOctNode->data.size(); ++i) {
				insert(currKey, currOctNode->data[i], currOctNode->data_idx[i]);
			}
			insert(currKey, data, data_idx);
		}
	}
	else {
		glm::vec3 center = currOctNode->center;
		//Determine which octant the point lies in (0 is bottom-back-left)
		uint8_t x = data.x > center.x;
		uint8_t y = data.y > center.y;
		uint8_t z = data.z > center.z;

		//Update the code
		octKey newKey = currOctNode->firstChildIdx + (x + 2 * y + 4 * z);

		//Update the halfLength
		float newHalfLength = currOctNode->halfLength / 2.f;

		//Update the center
		glm::vec3 newCenter(0.f);
		newCenter.x = center.x + newHalfLength * (x ? 1 : -1);
		newCenter.y = center.y + newHalfLength * (y ? 1 : -1);
		newCenter.z = center.z + newHalfLength * (z ? 1 : -1);

		insert(newKey, data, data_idx);
	}
}

void Octree::compact() {
	for (int i = 0; i < nodePool.size(); ++i) {
		OctNode* currNode = nodePool[i];
		OctNodeGPU gpuNode;

		//copy currNode into gpuNode
		gpuNode.firstChildIdx = currNode->firstChildIdx;
		gpuNode.isLeaf = currNode->isLeaf;
		gpuNode.center = currNode->center;
		if (gpuNode.isLeaf && currNode->data.size() > 0) {
			gpuNode.count = currNode->data.size();
			gpuNode.data_startIdx = gpuCoords.size();
			for (int k = 0; k < gpuNode.count; ++k) {
				gpuCoords.push_back(currNode->data[k]);
			}
		}
		else {
			gpuNode.count = 0;
			gpuNode.data_startIdx = -1;
		}
		gpuNodePool.push_back(gpuNode);
	}
}
