#ifndef ENCODE_CUDA_CUH
#define ENCODE_CUDA_CUH
#pragma once
#include "memory_management.cuh"


bool cuda_compute_phase_shift(int serial_flag); 

bool cuda_unwrap_phase_shift(int serial_flag);

bool cuda_normalize_phase(int serial_flag);
//kernel
__global__ void kernel_four_step_phase_shift(unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
float * const d_out, float * const confidence); 

__global__ void kernel_six_step_phase_shift(unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float * const d_out, float * const confidence);
 
__global__ void kernel_unwrap_variable_phase(float * const d_in_wrap_abs, float * const d_in_wrap_high,float const rate,float threshold, float * const d_out);
 
__global__ void kernel_normalize_phase(float * const d_in_unwrap_map, float rate,  float * const d_out_normal_map);

 

#endif