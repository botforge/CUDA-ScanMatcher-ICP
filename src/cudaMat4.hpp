#pragma once

#include <glm/glm.hpp>
#include <cuda_runtime.h>

struct cudaMat3 {
    glm::vec3 x;
    glm::vec3 y;
    glm::vec3 z;
};

struct cudaMat4 {
    glm::vec4 x;
    glm::vec4 y;
    glm::vec4 z;
    glm::vec4 w;
};
