#ifndef RECONSTRUCT_CUDA_CUH
#define RECONSTRUCT_CUDA_CUH
#pragma once
#include "memory_management.cuh"



bool cuda_generate_pointcloud_base_table();

__global__ void kernel_reconstruct_pointcloud_base_table(float * const confidence_map,float * const phase_x , float * const pointcloud,float * const depth);

__device__ float bilinear_interpolation(float x, float y, int map_width, float *mapping);









#endif