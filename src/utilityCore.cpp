#include <iostream>
#include <cstdio>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include "utilityCore.hpp"

/**
* Check for CUDA errors; print and exit if there was a problem.
*/
void utilityCore::checkCUDAError(const char *msg, int line) {
  cudaError_t err = cudaGetLastError();
  if (cudaSuccess != err) {
    if (line >= 0) {
      fprintf(stderr, "Line %d: ", line);
    }
    fprintf(stderr, "Cuda error: %s: %s.\n", msg, cudaGetErrorString(err));
    exit(EXIT_FAILURE);
  }
}

float utilityCore::clamp(float f, float min, float max) {
    if (f < min) {
        return min;
    } else if (f > max) {
        return max;
    } else {
        return f;
    }
}

bool utilityCore::replaceString(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos) {
        return false;
    }
    str.replace(start_pos, from.length(), to);
    return true;
}

std::string utilityCore::convertIntToString(int number) {
    std::stringstream ss;
    ss << number;
    return ss.str();
}

glm::vec3 utilityCore::clampRGB(glm::vec3 color) {
    if (color[0] < 0) {
        color[0] = 0;
    } else if (color[0] > 255) {
        color[0] = 255;
    }
    if (color[1] < 0) {
        color[1] = 0;
    } else if (color[1] > 255) {
        color[1] = 255;
    }
    if (color[2] < 0) {
        color[2] = 0;
    } else if (color[2] > 255) {
        color[2] = 255;
    }
    return color;
}

bool utilityCore::epsilonCheck(float a, float b) {
    if (fabs(fabs(a) - fabs(b)) < EPSILON) {
        return true;
    } else {
        return false;
    }
}

void utilityCore::printCudaMat4(const cudaMat4 &m) {
    utilityCore::printVec4(m.x);
    utilityCore::printVec4(m.y);
    utilityCore::printVec4(m.z);
    utilityCore::printVec4(m.w);
}

glm::mat4 utilityCore::buildTransformationMatrix(glm::vec3 translation, glm::vec3 rotation, glm::vec3 scale) {
    glm::mat4 translationMat = glm::translate(glm::mat4(), translation);
    glm::mat4 rotationMat = glm::rotate(glm::mat4(), rotation.x, glm::vec3(1, 0, 0));
    rotationMat = rotationMat * glm::rotate(glm::mat4(), rotation.y, glm::vec3(0, 1, 0));
    rotationMat = rotationMat * glm::rotate(glm::mat4(), rotation.z, glm::vec3(0, 0, 1));
    glm::mat4 scaleMat = glm::scale(glm::mat4(), scale);
    return translationMat * rotationMat * scaleMat;
}

cudaMat4 utilityCore::glmMat4ToCudaMat4(const glm::mat4 &a) {
    cudaMat4 m;
    glm::mat4 aTr = glm::transpose(a);
    m.x = aTr[0];
    m.y = aTr[1];
    m.z = aTr[2];
    m.w = aTr[3];
    return m;
}

glm::mat4 utilityCore::cudaMat4ToGlmMat4(const cudaMat4 &a) {
    glm::mat4 m;
    m[0] = a.x;
    m[1] = a.y;
    m[2] = a.z;
    m[3] = a.w;
    return glm::transpose(m);
}

std::vector<std::string> utilityCore::tokenizeString(std::string str) {
    std::stringstream strstr(str);
    std::istream_iterator<std::string> it(strstr);
    std::istream_iterator<std::string> end;
    std::vector<std::string> results(it, end);
    return results;
}

std::istream& utilityCore::safeGetline(std::istream& is, std::string& t) {
    t.clear();

    // The characters in the stream are read one-by-one using a std::streambuf.
    // That is faster than reading them one-by-one using the std::istream.
    // Code that uses streambuf this way must be guarded by a sentry object.
    // The sentry object performs various tasks,
    // such as thread synchronization and updating the stream state.

    std::istream::sentry se(is, true);
    std::streambuf* sb = is.rdbuf();

    for (;;) {
        int c = sb->sbumpc();
        switch (c) {
        case '\n':
            return is;
        case '\r':
            if (sb->sgetc() == '\n') {
                sb->sbumpc();
            }
            return is;
        case EOF:
            // Also handle the case when the last line has no line ending
            if (t.empty()) {
                is.setstate(std::ios::eofbit);
            }
            return is;
        default:
            t += (char)c;
        }
    }

    return is;
}
//-----------------------------
//-------GLM Printers----------
//-----------------------------

void utilityCore::printMat3(const glm::mat3 &m) {
    std::cout << m[0][0] << " " << m[1][0] << " " << m[2][0] << " " << std::endl;
    std::cout << m[0][1] << " " << m[1][1] << " " << m[2][1] << " " << std::endl;
    std::cout << m[0][2] << " " << m[1][2] << " " << m[2][2] << " " << std::endl;
}

void utilityCore::printMat4(const glm::mat4 &m) {
    std::cout << m[0][0] << " " << m[1][0] << " " << m[2][0] << " " << m[3][0] << std::endl;
    std::cout << m[0][1] << " " << m[1][1] << " " << m[2][1] << " " << m[3][1] << std::endl;
    std::cout << m[0][2] << " " << m[1][2] << " " << m[2][2] << " " << m[3][2] << std::endl;
    std::cout << m[0][3] << " " << m[1][3] << " " << m[2][3] << " " << m[3][3] << std::endl;
}

void utilityCore::printVec4(const glm::vec4 &m) {
    std::cout << m[0] << " " << m[1] << " " << m[2] << " " << m[3] << std::endl;
}

void utilityCore::printVec3(const glm::vec3 &m) {
    std::cout << m[0] << " " << m[1] << " " << m[2] << std::endl;
}
