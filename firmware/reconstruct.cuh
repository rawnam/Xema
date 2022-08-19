#ifndef RECONSTRUCT_CUDA_CUH
#define RECONSTRUCT_CUDA_CUH
#pragma once 
#include <opencv2/core.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
 


bool cuda_set_param_dlp_resolution(int width,int height);

bool cuda_set_param_confidence(float val);

bool cuda_set_param_z_range(float min,float max);
 

__global__ void kernel_reconstruct_pointcloud_base_table(int width,int height,float * const xL_rotate_x,float * const xL_rotate_y,float * const single_pattern_mapping,float * const R_1,float b,
float * const confidence_map,float * const phase_x , float * const pointcloud,float * const depth);

__device__ float bilinear_interpolation(float x, float y, int map_width, float *mapping);









#endif