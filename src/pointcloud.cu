#include "pointcloud.h"



pointcloud::pointcloud(): isTarget(false), N(500){
	dev_pos = new glm::vec3[500];
	dev_rgb = new glm::vec3[500];
}

pointcloud::pointcloud(bool target, int numPoints): isTarget(target), N(numPoints){
	dev_pos = new glm::vec3[N];
	dev_rgb = new glm::vec3[N];
}


/******************
* CPU Methods *
******************/

/**
 * Initialize and fills dev_pos and dev_rgb array in CPU
*/
void pointcloud::initCPU() {
	//buildSinusoidCPU();
}

/**
 * Populates dev_pos with a 3D Sinusoid (with or without Noise) on the CPU
*/
void pointcloud::buildSinusoidCPU() {
	float y_interval = 2.5 * PI / N;
	for (int idx = 0; idx < N; idx++) {
		dev_pos[idx] = glm::vec3(0.5f, idx*y_interval, sin(idx*y_interval));
		dev_rgb[idx] = glm::vec3(0.1f, 0.8f, 0.5f);
	}
}

/**
 * Copies dev_pos and dev_rgb into the VBO
*/
void pointcloud::pointCloudToVBOCPU(float *vbodptr_positions, float *vbodptr_rgb, float s_scale) {
	float c_scale = -1.0f / s_scale;
	for (int idx = 0; idx < N; ++idx) {
		//Push Positions to VBO
		vbodptr_positions[4 * idx + 0] = dev_pos[idx].x * c_scale;
		vbodptr_positions[4 * idx + 1] = dev_pos[idx].y * c_scale;
		vbodptr_positions[4 * idx + 2] = dev_pos[idx].z * c_scale;
		vbodptr_positions[4 * idx + 3] = 1.0f;

		//Push RGB to VBO
		vbodptr_rgb[4 * idx + 0] = dev_rgb[idx].x + 0.3f;
		vbodptr_rgb[4 * idx + 1] = dev_rgb[idx].y + 0.3f;
		vbodptr_rgb[4 * idx + 2] = dev_rgb[idx].z + 0.3f;
		vbodptr_rgb[4 * idx + 3] = 1.0f;
	}
}

pointcloud::~pointcloud() {
}
