#pragma once

#include <algorithm>
#include <istream>
#include <ostream>
#include <iterator>
#include <cuda.h>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include<glm/gtx/transform.hpp>
#include "cudaMat4.hpp"

#define PI                          3.1415926535897932384626422832795028841971
#define TWO_PI                      6.2831853071795864769252867665590057683943
#define SQRT_OF_ONE_THIRD           0.5773502691896257645091487805019574556476
#define E                           2.7182818284590452353602874713526624977572
#define G                           6.67384e-11
#define EPSILON                     .000000001
#define ZERO_ABSORPTION_EPSILON     0.00001

namespace utilityCore {
extern void checkCUDAError(const char *msg, int line);
extern float clamp(float f, float min, float max);
extern bool replaceString(std::string& str, const std::string& from, const std::string& to);
extern glm::vec3 clampRGB(glm::vec3 color);
extern bool epsilonCheck(float a, float b);
extern std::vector<std::string> tokenizeString(std::string str);
extern cudaMat4 glmMat4ToCudaMat4(const glm::mat4 &a);
extern glm::mat4 cudaMat4ToGlmMat4(const cudaMat4 &a);
extern glm::mat4 buildTransformationMatrix(glm::vec3 translation, glm::vec3 rotation, glm::vec3 scale);
extern void printCudaMat4(const cudaMat4 &m);
extern std::string convertIntToString(int number);
extern std::istream& safeGetline(std::istream& is, std::string& t); //Thanks to http://stackoverflow.com/a/6089413

//-----------------------------
//-------GLM Printers----------
//-----------------------------
extern void printMat4(const glm::mat4 &);
extern void printMat3(const glm::mat3 &);
extern void printVec4(const glm::vec4 &);
extern void printVec3(const glm::vec3 &);
}
