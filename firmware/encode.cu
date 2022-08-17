#include "encode.cuh"

 #define CHECK(call)\
{\
  const cudaError_t error=call;\
  if(error!=cudaSuccess)\
  {\
      printf("ERROR: %s:%d,",__FILE__,__LINE__);\
      printf("code:%d,reason:%s\n",error,cudaGetErrorString(error));\
      exit(1);\
  }\
}

dim3 threadsPerBlock_encode(8, 8);
dim3 blocksPerGrid_encode((d_image_width_ + threadsPerBlock_encode.x - 1) / threadsPerBlock_encode.x,
(d_image_height_ + threadsPerBlock_encode.y - 1) / threadsPerBlock_encode.y);

//kernel
__global__ void kernel_four_step_phase_shift(unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,float * const d_out, float * const confidence)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * d_image_width_ + idx;

	if (idx < d_image_width_ && idy < d_image_height_)
	{

		float a = d_in_3[offset] - d_in_1[offset];
		float b = d_in_0[offset] - d_in_2[offset];

		int over_num = 0;
		if(d_in_0[offset]>= 255)
		{
			over_num++;
		}
		if (d_in_1[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_2[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_3[offset] >= 255)
		{
			over_num++;
		}

		if(over_num> 1)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = CV_PI + std::atan2(a, b);
		}
 

	}
}


__global__ void kernel_six_step_phase_shift(unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * d_image_width_ + idx;
	float s_0 =  0;
	float s_1 =  0.866025;
	float s_2 =  0.866025;
	float s_3 =  0;
	float s_4 =  -0.866025;
	float s_5 =  -0.866025;
	float c_0 =  1;
	float c_1 =  0.5;
	float c_2 =  -0.5;
	float c_3 =  -1;
	float c_4 =  -0.5;
	float c_5 =  0.5;
	
	if (idx < d_image_width_ && idy < d_image_height_)
	{

		float a = c_0 *d_in_3[offset] + c_1 *d_in_4[offset] + c_2 *d_in_5[offset] + c_3* d_in_0[offset] +c_4*d_in_1[offset] + c_5*d_in_2[offset];
		float b = s_0 *d_in_3[offset] + s_1 *d_in_4[offset] + s_2 *d_in_5[offset] + s_3* d_in_0[offset] +s_4*d_in_1[offset] + s_5*d_in_2[offset];


		int over_num = 0;
		if(d_in_0[offset]>= 255)
		{
			over_num++;
		}
		if (d_in_1[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_2[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_3[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_4[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_5[offset] >= 255)
		{
			over_num++;
		}

		if(over_num> 3)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = CV_PI + std::atan2(a, b);
		}
  
		// confidence[offset] = std::sqrt(a*a + b*b);
		// d_out[offset] = DF_PI + std::atan2(a, b);
	}
} 

bool cuda_compute_phase_shift(int serial_flag)
{
	 
	switch(serial_flag)
	{
		case 0:
		{ 
			int i= 0;
			kernel_four_step_phase_shift << <blocksPerGrid_encode, threadsPerBlock_encode >> > (d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);

				// kernel_four_step_phase_shift_texture<< <blocksPerGrid_encode, threadsPerBlock_encode >> >(serial_flag,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 1:
		{

			int i= 4;
			kernel_four_step_phase_shift << <blocksPerGrid_encode, threadsPerBlock_encode >> > (d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
				
				// kernel_four_step_phase_shift_texture<< <blocksPerGrid_encode, threadsPerBlock_encode >> >(serial_flag,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
			
		}
		break;
		case 2:
		{ 
			int i= 8;
			kernel_four_step_phase_shift << <blocksPerGrid_encode, threadsPerBlock_encode >> > (d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
				
				// kernel_four_step_phase_shift_texture<< <blocksPerGrid_encode, threadsPerBlock_encode >> >(serial_flag,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 3:
		{ 
			int i= 12; 
			kernel_six_step_phase_shift << <blocksPerGrid_encode, threadsPerBlock_encode >> > (d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3],d_patterns_list_[i + 4],d_patterns_list_[i + 5] ,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
 
            
				// cuda_six_step_phase_shift_texture<< <blocksPerGrid_encode, threadsPerBlock_encode >> > (d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
				// cudaDeviceSynchronize();

				// cv::Mat phase(1200, 1920, CV_32F, cv::Scalar(0));
				// CHECK(cudaMemcpy(phase.data, d_wrap_map_list_[serial_flag], 1 * image_height_ * image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
				// cv::imwrite("phase1.tiff",phase);
		}
		break;
		case 4:
		{
			int i= 18;
			kernel_four_step_phase_shift << <blocksPerGrid_encode, threadsPerBlock_encode >> > (d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 5:
		{
			int i= 22;
			kernel_four_step_phase_shift << <blocksPerGrid_encode, threadsPerBlock_encode >> > (d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 6:
		{
			int i= 26;
			kernel_four_step_phase_shift << <blocksPerGrid_encode, threadsPerBlock_encode >> > (d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
  
		default :
			break;
	}

	
	
	return true;
}


bool cuda_normalize_phase(int serial_flag)
{
    switch(serial_flag)
	{ 
        case 0:
		{   
            kernel_normalize_phase<< <blocksPerGrid_encode, threadsPerBlock_encode >> >(d_unwrap_map_list_[0], (float)128.0, d_unwrap_map_list_[0]);  
		}
		break; 
		case 1:
		{   
  
            kernel_normalize_phase<< <blocksPerGrid_encode, threadsPerBlock_encode >> >(d_unwrap_map_list_[1], (float)18., d_unwrap_map_list_[1]); 
		}
		break;

		case 2:
		{ 
			kernel_normalize_phase<< <blocksPerGrid_encode, threadsPerBlock_encode >> >(d_unwrap_map_list_[1], (float)72., d_unwrap_map_list_[1]); 
		}
		break;

		default :
			break;
	}


	return true;
}

bool cuda_unwrap_phase_shift(int serial_flag)
{

	switch(serial_flag)
	{ 
		case 1:
		{  
            kernel_unwrap_variable_phase<< <blocksPerGrid_encode, threadsPerBlock_encode >> >(d_wrap_map_list_[0], d_wrap_map_list_[1], 8.0, CV_PI, d_unwrap_map_list_[0]);
  
		}
		break;

		case 2:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase << <blocksPerGrid_encode, threadsPerBlock_encode >> >(d_unwrap_map_list_[0], d_wrap_map_list_[2], 4.0,CV_PI, d_unwrap_map_list_[0]); 
			// CHECK ( cudaGetLastError () );
		}
		break;
		case 3:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase << <blocksPerGrid_encode, threadsPerBlock_encode >> >(d_unwrap_map_list_[0], d_wrap_map_list_[3], 4.0,1.5, d_unwrap_map_list_[0]); 
 
		}
		break;
		case 4:
		{
 
		}
		break;
		case 5:
		{
			kernel_unwrap_variable_phase << <blocksPerGrid_encode, threadsPerBlock_encode >> >(d_wrap_map_list_[4], d_wrap_map_list_[5], 8.0,CV_PI, d_unwrap_map_list_[1]);
		}
		break;
		case 6:
		{
			kernel_unwrap_variable_phase << <blocksPerGrid_encode, threadsPerBlock_encode >> >(d_unwrap_map_list_[1], d_wrap_map_list_[6], 4.0,CV_PI, d_unwrap_map_list_[1]);
 
			LOG(INFO)<<"unwrap 6:  ";

		}
		break;
		case 7:
		{
			kernel_unwrap_variable_phase << <blocksPerGrid_encode, threadsPerBlock_encode >> >(d_unwrap_map_list_[1], d_wrap_map_list_[7], 4.0,CV_PI, d_unwrap_map_list_[1]);
 
		 	LOG(INFO)<<"unwrap 7:  ";

		}
		break;
 

		default :
			break;
	}


	return true;
}

__global__ void kernel_unwrap_variable_phase(float * const d_in_wrap_abs, float * const d_in_wrap_high,float const rate,float threshold, float * const d_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	int offset = idy * d_image_width_ + idx;

	if (idx < d_image_width_ && idy < d_image_height_)
	{

		/*****************************************************************************/

		float temp = 0.5 + (rate * d_in_wrap_abs[offset] - d_in_wrap_high[offset]) / (2*CV_PI);
		int k = temp;
        
		float unwrap_value =  2*CV_PI*k + d_in_wrap_high[offset]; 
  
        float err = unwrap_value - (rate * d_in_wrap_abs[offset]);

		if(abs(err)> threshold)
		{
			d_out[offset] = -10.0; 
		}
		else
		{ 
			d_out[offset] = unwrap_value;
		}

		/******************************************************************/
	}
}


__global__ void kernel_normalize_phase(float * const d_in_unwrap_map, float rate,  float * const d_out_normal_map)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	const unsigned int offset = idy*d_image_width_ + idx;

	if (idx < d_image_width_ && idy < d_image_height_)
	{

		/*****************************************************************************/ 
		d_out_normal_map[offset] = d_in_unwrap_map[offset] /rate;  

		/******************************************************************/
	}
}