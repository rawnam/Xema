#ifndef MINSW_CUDA_CUH
#define MINSW_CUDA_CUH
#pragma once

const int ThresholdMapSeries = 16;
const int Minsw8MapSeries = 17;
const int binMapSeries = 18;
   

int cuda_copy_minsw8_pattern_to_memory(unsigned char* pattern_ptr,int serial_flag);

int cuda_handle_minsw8(int flag);
    
__global__ void kernel_generate_threshold_map(int width,int height,unsigned char * const d_in_white, unsigned char * const d_in_black,unsigned char * const d_out_threshold);

__global__ void kernel_threshold_patterns(int width,int height,unsigned char * const d_in_pattern, unsigned char * const d_in_threshold,int places,unsigned char* const d_out_bin);
 
__global__ void kernel_minsw8_to_bin(int width,int height,unsigned char * const minsw8_code,unsigned char * const d_in_minsw8, unsigned char * const d_out_bin);

__global__ void kernel_bin_unwrap(int width,int height,unsigned char * const d_in_bin, float * const d_in_wrap,float * const d_out_unwrap);

#endif