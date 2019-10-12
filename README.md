CUDA Scan Matcher via ICP - Octree Optimized 
===============================================================================

**University of Pennsylvania, CIS 565: GPU Programming and Architecture**

Dhruv Karthik: [LinkedIn](https://www.linkedin.com/in/dhruv_karthik/)

Tested on: Windows 10 Home, Intel(R) Core(TM) i7-8700 CPU @ 3.20GHz, 16GM, GTX 2070 - Compute Capability 7.5
____________________________________________________________________________________
![Developer](https://img.shields.io/badge/Developer-Dhruv-0f97ff.svg?style=flat) ![CUDA 10.1](https://img.shields.io/badge/CUDA-10.1-yellow.svg) ![Built](https://img.shields.io/appveyor/ci/gruntjs/grunt.svg) ![Issues](https://img.shields.io/badge/issues-none-green.svg)
____________________________________________________________________________________
<p align="center">
  <img  src="img/waymotrue.gif">
</p>

Table of contents
=================
  * [Scan Matching Algorithm](#scan-matching-algorithm)
  * [Results](#results)
  * [Octree Optimization](#optimization)
  * [Performance Analysis](#performance-analysis)
  * [Credits & Acknowledgments](#credits)
   
# Scan Matching Algorithm
An intuitive understanding of Scan Matching is best illustrated in this 11 second [video](https://www.youtube.com/watch?v=uzOCS_gdZuM).
Scan Matching seeks to align two similar pointclouds by finding the transformation between them. It does so via the iterative closest point algorithm outlined below in pseudocode:
```bash
#Aligns pointcloud A to pointcloud B
def scan_match(pointcloud A, pointcloud B):
  1. For each point in A, find the closest point in B
  2. Compute a 3-D transformation matrix that aligns the points (Use SVD & Least Squares Regression)
  3. Update all points in the target by the transformation matrix
  4. Repeat steps 1-3 until some epsilon convergence
  RETURN : Some Transformation matrix T
```
The key benefit from CUDA is in step 1. Finding the nearest neighbors synchronously takes far longer than finding nearest neighbors via some optimized datastructure like an octree. I implemented 3 versions: A CPU version, a Naive CUDA version, and an Octree version.  

# Results
<p align="center">
  <img  src="img/bunnytrue.gif">
</p>

<p align="center">
  <img  src="img/buddhatrue.gif">
</p>

<p align="center">
  <img  src="img/dragontrue.gif">
</p>

# Sparse Octree Optimization
A sparse octree is an optimized datastructure that allows for fast nearest neighbor calculations. It is represented as a tree in which each internal node has exactly eight children. We use octrees in order to subdivide our 3D space, ensuring that no *octant* has more than *n* points. If some node has more than *n* vertices, we recursively subdivide this node by splitting it into eight children, and redistributing the points among these 8 children. This speeds up nearest neighbor searches as now, we only need to search for neighbors within our octant. 
  <p align="center">
    <img  src="https://developer.nvidia.com/sites/all/modules/custom/gpugems/books/GPUGems2/elementLinks/37_octree_03.jpg">
  </p>
