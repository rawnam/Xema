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


//kernel
__global__ void kernel_four_step_phase_shift(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,float * const d_out, float * const confidence)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height)
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


__global__ void kernel_six_step_phase_shift(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;
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
	
	if (idx < width && idy < height)
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


__global__ void kernel_unwrap_variable_phase(int width,int height,float * const d_in_wrap_abs, float * const d_in_wrap_high,float const rate,float threshold, float * const d_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	int offset = idy * width + idx;

	if (idx < width && idy < height)
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


__global__ void kernel_unwrap_variable_phase_base_confidence(int width,int height,float * const d_in_wrap_abs, float * const d_in_wrap_high,float const rate,float threshold, float fisher_rate, float* const d_fisher_confidence_mask, float * const d_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	int offset = idy * width + idx;

	if (idx < width && idy < height)
	{

		/*****************************************************************************/

		float temp = 0.5 + (rate * d_in_wrap_abs[offset] - d_in_wrap_high[offset]) / (2*CV_PI);
		int k = temp;
        
		float unwrap_value =  2*CV_PI*k + d_in_wrap_high[offset]; 
  
        float err = unwrap_value - (rate * d_in_wrap_abs[offset]);

		d_fisher_confidence_mask[offset] = d_fisher_confidence_mask[offset] + (abs(err) * fisher_rate);

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


__global__ void kernel_normalize_phase(int width,int height,float * const d_in_unwrap_map, float rate,  float * const d_out_normal_map)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	const unsigned int offset = idy*width + idx;
 
	if (idx < width && idy < height)
	{

		/*****************************************************************************/ 
		d_out_normal_map[offset] = d_in_unwrap_map[offset] /rate;  

		/******************************************************************/
	}
}


__global__ void kernel_merge_six_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,unsigned short* const d_in_4,unsigned short* const d_in_5,int repetition_count,
	uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;
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
	
	if (idx < img_width && idy < img_height)
	{

		float a = c_0 *d_in_3[offset] + c_1 *d_in_4[offset] + c_2 *d_in_5[offset] + c_3* d_in_0[offset] +c_4*d_in_1[offset] + c_5*d_in_2[offset];
		float b = s_0 *d_in_3[offset] + s_1 *d_in_4[offset] + s_2 *d_in_5[offset] + s_3* d_in_0[offset] +s_4*d_in_1[offset] + s_5*d_in_2[offset];

  
		confidence[offset] = std::sqrt(a*a + b*b);
		d_out[offset] = CV_PI + std::atan2(a, b);
	}

	
}


__global__ void kernel_merge_four_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,int repetition_count,uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	int max_pixel = 255*repetition_count;

	if (idx < img_width && idy < img_height)
	{

		float a = d_in_3[offset] - d_in_1[offset];
		float b = d_in_0[offset] - d_in_2[offset];

		int over_num = 0;
		if(d_in_0[offset]>= max_pixel)
		{
			over_num++;
		}
		if (d_in_1[offset] >= max_pixel)
		{
			over_num++;
		}
		if (d_in_2[offset] >= max_pixel)
		{
			over_num++;
		}
		if (d_in_3[offset] >= max_pixel)
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
