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
