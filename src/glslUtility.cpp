/**
 * @file      glslUtility.cpp
 * @brief     A utility namespace for loading GLSL shaders.
 * @authors   Varun Sampath, Patrick Cozzi, Karl Li
 * @date      2012
 * @copyright University of Pennsylvania
 */

#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <cstring>
#include <memory>
#include "glslUtility.hpp"

using std::ios;

namespace glslUtility {
typedef struct {
    GLint vertex;
    GLint fragment;
    GLint geometry;
} shaders_t;

char* loadFile(const char *fname, GLint &fSize) {
    // file read based on example in cplusplus.com tutorial
    std::ifstream file(fname, ios::in | ios::binary | ios::ate);
    if (file.is_open()) {
        unsigned int size = (unsigned int)file.tellg();
        fSize = size;
        char *memblock = new char [size];
        file.seekg(0, ios::beg);
        file.read(memblock, size);
        file.close();
        std::cout << "file " << fname << " loaded" << std::endl;
        return memblock;
    }

    std::cout << "Unable to open file " << fname << std::endl;
    exit(1);
}

// printShaderInfoLog
// From OpenGL Shading Language 3rd Edition, p215-216
// Display (hopefully) useful error messages if shader fails to compile
void printShaderInfoLog(GLint shader) {
    int infoLogLen = 0;
    int charsWritten = 0;

    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLen);

    if (infoLogLen > 1) {
        std::unique_ptr<GLchar[]> infoLog{ new GLchar[infoLogLen] };
        // error check for fail to allocate memory omitted
        glGetShaderInfoLog(shader, infoLogLen, &charsWritten, infoLog.get());
        std::cout << "InfoLog:" << std::endl << infoLog.get() << std::endl;
    }
}

void printLinkInfoLog(GLint prog) {
    int infoLogLen = 0;
    int charsWritten = 0;

    glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &infoLogLen);

    if (infoLogLen > 1) {
        std::unique_ptr<GLchar[]> infoLog{ new GLchar[infoLogLen] };
        // error check for fail to allocate memory omitted
        glGetProgramInfoLog(prog, infoLogLen, &charsWritten, infoLog.get());
        std::cout << "InfoLog:" << std::endl << infoLog.get() << std::endl;
    }
}

shaders_t loadShaders(const char * vert_path, const char * geom_path, const char * frag_path) {
    GLint f, g = -1, v;

    char *vs, *gs, *fs;

    v = glCreateShader(GL_VERTEX_SHADER);
    if (geom_path) {
        g = glCreateShader(GL_GEOMETRY_SHADER);
    }
    f = glCreateShader(GL_FRAGMENT_SHADER);

    // load shaders & get length of each
    GLint vlen;
    GLint glen;
    GLint flen;
    vs = loadFile(vert_path, vlen);
    if (geom_path) {
        gs = loadFile(geom_path, glen);
    }
    fs = loadFile(frag_path, flen);

    const char * vv = vs;
    const char * gg = geom_path ? gs : NULL;
    const char * ff = fs;

    glShaderSource(v, 1, &vv, &vlen);
    if (geom_path) {
        glShaderSource(g, 1, &gg, &glen);
    }
    glShaderSource(f, 1, &ff, &flen);

    GLint compiled;

    glCompileShader(v);
    glGetShaderiv(v, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        std::cout << "Vertex shader not compiled." << std::endl;
    }
    printShaderInfoLog(v);

    if (geom_path) {
        glCompileShader(g);
        glGetShaderiv(g, GL_COMPILE_STATUS, &compiled);
        if (!compiled) {
            std::cout << "Geometry shader not compiled." << std::endl;
        }
        printShaderInfoLog(g);
    }

    glCompileShader(f);
    glGetShaderiv(f, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        std::cout << "Fragment shader not compiled." << std::endl;
    }
    printShaderInfoLog(f);

    shaders_t out;
    out.vertex = v;
    out.geometry = g;
    out.fragment = f;

    delete [] vs; // dont forget to free allocated memory, or else really bad things start happening
    if (geom_path) {
        delete[] gs;
    }
    delete [] fs; // we allocated this in the loadFile function...

    return out;
}

void attachAndLinkProgram(GLuint program, shaders_t shaders) {
    glAttachShader(program, shaders.vertex);
    if (shaders.geometry >= 0) {
        glAttachShader(program, shaders.geometry);
    }
    glAttachShader(program, shaders.fragment);

    glLinkProgram(program);
    GLint linked;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (!linked) {
        std::cout << "Program did not link." << std::endl;
    }
    printLinkInfoLog(program);
}

GLuint createProgram(
    const char *vertexShaderPath,
    const char *fragmentShaderPath,
    const char *attributeLocations[], GLuint numberOfLocations) {
    glslUtility::shaders_t shaders = glslUtility::loadShaders(vertexShaderPath, NULL, fragmentShaderPath);

    GLuint program = glCreateProgram();

    for (GLuint i = 0; i < numberOfLocations; ++i) {
        glBindAttribLocation(program, i, attributeLocations[i]);
    }

    glslUtility::attachAndLinkProgram(program, shaders);

    return program;
}

GLuint createProgram(
    const char *vertexShaderPath,
    const char *geometryShaderPath,
    const char *fragmentShaderPath,
    const char *attributeLocations[], GLuint numberOfLocations) {
    glslUtility::shaders_t shaders = glslUtility::loadShaders(vertexShaderPath, geometryShaderPath, fragmentShaderPath);

    GLuint program = glCreateProgram();

    for (GLuint i = 0; i < numberOfLocations; ++i) {
        glBindAttribLocation(program, i, attributeLocations[i]);
    }

    glslUtility::attachAndLinkProgram(program, shaders);

    return program;
}
}
