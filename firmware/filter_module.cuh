#pragma once
#ifndef DF8_ENCODE_CUDA_CUH_1
#define DF8_ENCODE_CUDA_CUH_1
#include "filter_module.cuh"
#include <device_launch_parameters.h> 
//#include <device_functions.h>
#include <cuda_runtime.h>
#include <iostream>
#include <stdint.h>
#include <vector>
#include <opencv2/core.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <device_launch_parameters.h>
#include <device_functions.h>
#include <cuda_texture_types.h>
#include <texture_types.h>
#include "cuda_runtime.h" 
#include <cuda_runtime.h>

#include <iostream>
#include <stdint.h>
#include <vector>  


void filter_reflect_noise(uint32_t img_height, uint32_t img_width,float * const unwrap_map);

__global__ void cuda_filter_reflect_noise(uint32_t img_height, uint32_t img_width,float * const unwrap_map);


#endif