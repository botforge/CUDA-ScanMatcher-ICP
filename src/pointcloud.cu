#include "pointcloud.h"



pointcloud::pointcloud(){
}

pointcloud::pointcloud(bool target, int numPoints){
	isTarget = target;
	N = numPoints;
}

/**
 * Initialize and fills dev_pos and dev_rgb array in CPU
*/
void pointcloud::initCPU() {
	buildSinusoidCPU();
}

/**
 * Populates dev_pos with a 3D Sinusoid (with or without Noise) on the CPU
*/
void pointcloud::buildSinusoidCPU() {
	dev_pos = new glm::vec3[N];
	dev_rgb = new glm::vec3[N];

	float y_interval = 2.5 * PI / N;
	for (int idx = 0; idx < N; idx++) {
		dev_pos[idx] = glm::vec3(0.5f, idx*y_interval, sin(idx*y_interval));
		dev_rgb[idx] = glm::vec3(0.1f, 0.8f, 0.5f);
	}
}

pointcloud::~pointcloud() {
}
