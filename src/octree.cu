#include "octree.h"

Octree::Octree(glm::vec3 rCenter, float rHalfLength, std::vector<glm::vec3> c) : rootCenter(rCenter), rootHalfLength(rHalfLength), octNodePool(std::vector<OctNode>(c.size() * MAX_PTS_PER_OCTANT * 8)), octCoords(std::vector<glm::vec3>(c.size() * MAX_PTS_PER_OCTANT * 8)), coords(c), stackPointer(0){
	rootNode.firstChild = 0;
	rootNode.data_startidx = 0;
	rootNode.data_endidx = 0;

	rootNode.center = rootCenter;
	rootNode.halfLength = rootHalfLength;
	rootNode.count = 0;
	rootNode.isLeaf = true;

	octNodePool[0] = (rootNode);
}

/**
* Wrapper call to recursive "insert" call
*/
void Octree::create() {
	octKey root = 1;
	for (int i = 0; i < coords.size(); ++i) {
		insert(root, coords[i], rootHalfLength, rootCenter);
	}
}

void Octree::insert(octKey currKey, glm::vec3 data, float halfLength, glm::vec3 center) {
	long long fixkey;
	if (currKey == 1) {
		fixkey = currKey - 1;
	}
	else {
		fixkey = currKey;
	}
	OctNode &currOctNode = octNodePool[fixkey];

	//Option 1:Check if we're at a leaf, then add the point
	if (currOctNode.isLeaf) {
		bool hasMaxData = currOctNode.count >= MAX_PTS_PER_OCTANT + 0;
		if (!hasMaxData) { //If we haven't surpassed MAX_PTS_PER_OCTANT, just append the data
			octCoords[currOctNode.data_startidx + currOctNode.count] = data;
			currOctNode.count += 1;
		}
		else { //We have surpassed MAX_PTS_PER_OCTANT
			//1:Subdivide 
			stackPointer += 8;
			octKey baseNewKey = stackPointer;
			float newHalfLength = halfLength / 2.f;
			currOctNode.firstChild = baseNewKey; //Reparent the child
			currOctNode.isLeaf = false;

			for (int z = 0; z < 2; ++z) {
				for (int y = 0; y < 2; ++y) {
					for (int x = 0; x < 2; ++x) {
						//Update the code
						octKey newKey = baseNewKey + (x + 2 * y + 4 * z);

						//Update the halflength
						float newHalfLength = halfLength / 2.f;

						//Update the center
						glm::vec3 newCenter(0.f);
						newCenter.x = center.x + newHalfLength * (x ? 1 : -1);
						newCenter.y = center.y + newHalfLength * (y ? 1 : -1);
						newCenter.z = center.z + newHalfLength * (z ? 1 : -1);

						//Update new entry in octNodePool
						OctNode &newNode = octNodePool[newKey];
						newNode.firstChild = 0;
						newNode.data_startidx = (stackPointer/8) + (MAX_PTS_PER_OCTANT) * (x+2*y+4*z+1);
						newNode.center = newCenter;
						newNode.halfLength = newHalfLength;
						newNode.count = 0;
						newNode.isLeaf = true;
					}
				}
			}
			//2:Redistribute Points
			for (int i = currOctNode.data_startidx; i < currOctNode.data_startidx + currOctNode.count; ++i) {
				insert(currKey, octCoords[i], halfLength, center);
			}
			insert(currKey, data, halfLength, center);
		}
	}
	else { //Option 2: We're not at a leaf, so have to go one level deeper

		//Determine which octant the point lies in (0 is bottom-back-left)
		uint8_t x = data.x > center.x;
		uint8_t y = data.y > center.y;
		uint8_t z = data.z > center.z;

		//Update the code
		octKey newKey = currOctNode.firstChild + (x + 2 * y + 4 * z);

		//Update the halfLength
		float newHalfLength = halfLength / 2.f;

		//Update the center
		glm::vec3 newCenter(0.f);
		newCenter.x = center.x + newHalfLength * (x ? 1 : -1);
		newCenter.y = center.y + newHalfLength * (y ? 1 : -1);
		newCenter.z = center.z + newHalfLength * (z ? 1 : -1);

		insert(newKey, data, newHalfLength, newCenter);
	}
}
