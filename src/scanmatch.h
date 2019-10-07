#pragma once

#include <stdio.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/device_vector.h>
#include <cuda.h>
#include <cmath>
#include <vector>
#include "pointcloud.h"
#include "utilityCore.hpp"


namespace ScanMatch {
    void initSimulation(int N);
    void copyPointCloudToVBO(float *vbodptr_positions, float *vbodptr_rgb);
    void endSimulation();
    void unitTest();
}
