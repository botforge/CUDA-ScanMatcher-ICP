/**
 * @file      glslUtility.hpp
 * @brief     A utility namespace for loading GLSL shaders.
 * @authors   Varun Sampath, Patrick Cozzi, Karl Li
 * @date      2012
 * @copyright University of Pennsylvania
 */

#pragma once

#include <GL/glew.h>

namespace glslUtility {
GLuint createDefaultProgram(const char *attributeLocations[],
                            GLuint numberOfLocations);
GLuint createProgram(const char *vertexShaderPath,
                     const char *fragmentShaderPath,
                     const char *attributeLocations[],
                     GLuint numberOfLocations);
GLuint createProgram(const char *vertexShaderPath,
                     const char *geometryShaderPath,
                     const char *fragmentShaderPath,
                     const char *attributeLocations[],
                     GLuint numberOfLocations);
}
