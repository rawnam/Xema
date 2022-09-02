#include "reconstruct.cuh"
#include <opencv2/core.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

__device__ float d_confidence_ = 10;

__device__ int d_dlp_width_ = 0;
__device__ int d_dlp_height_ = 0;
__device__ float d_max_phase_ = 2* CV_PI; 

__device__ float d_min_z_ = 10; 
__device__ float d_max_z_ = 3000;


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
 
 
bool cuda_set_param_dlp_resolution(int width,int height)
{
	cudaError_t error_code = cudaMemcpyToSymbol(d_dlp_width_, &width, sizeof(float));

	if(error_code!= cudaSuccess)
	{
		return false;
	}

	error_code = cudaMemcpyToSymbol(d_dlp_height_, &height, sizeof(float));

	if(error_code!= cudaSuccess)
	{
		return false;
	}

	return true;
}
 
bool cuda_set_param_z_range(float min,float max)
{
	cudaError_t error_code = cudaMemcpyToSymbol(d_min_z_, &min, sizeof(float));

	if(error_code!= cudaSuccess)
	{
		return false;
	}

	error_code = cudaMemcpyToSymbol(d_max_z_, &max, sizeof(float));

	if(error_code!= cudaSuccess)
	{
		return false;
	}

	return true;
}


bool cuda_set_param_confidence(float val)
{
	cudaError_t error_code = cudaMemcpyToSymbol(d_confidence_, &val, sizeof(float));

	if(error_code!= cudaSuccess)
	{
		return false;
	}

	return true;
}

__device__ float bilinear_interpolation(float x, float y, int map_width, float *mapping)
{

	int x1 = floor(x);
	int y1 = floor(y);
	int x2 = x1 + 1;
	int y2 = y1 + 1;

	//row-y,col-x

	if (x1 == 1919) {
		float out = mapping[y1 *map_width + x1];
		return out;
	}
	else {
		float fq11 = mapping[y1 *map_width + x1];
		float fq21 = mapping[y1 *map_width + x2];
		float fq12 = mapping[y2 *map_width + x1];
		float fq22 = mapping[y2 *map_width + x2];

		if (-2 == fq11 || -2 == fq21 || -2 == fq12 || -2 == fq22)
		{
			return -2;
		}

		float out = fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1);

		return out;
	}
	 

}

__global__ void kernel_reconstruct_pointcloud_base_table(int width,int height,float * const xL_rotate_x,float * const xL_rotate_y,float * const single_pattern_mapping,float * const R_1,float b,
float * const confidence_map,float * const phase_x , float * const pointcloud,float * const depth)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

  
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height)
	{
		/****************************************************************************/
		//phase to position
		float Xp = phase_x[offset] * d_dlp_width_ /d_max_phase_; 
		// float Xp = (phase_x[offset] * 1280) /(2*CV_PI); 
        float Xcr = xL_rotate_x[offset];
        float Ycr = xL_rotate_y[offset];
 
 
        float Xpr = bilinear_interpolation(Xp, (Ycr + 1) * 2000, 2000, single_pattern_mapping);
        float delta_X = std::abs(Xcr - Xpr); 
        float Z = b / delta_X;
	
		float X_L = Z * Xcr * R_1[0] + Z * Ycr * R_1[1] + Z * R_1[2];
		float Y_L = Z * Xcr * R_1[3] + Z * Ycr * R_1[4] + Z * R_1[5];
		float Z_L = Z * Xcr * R_1[6] + Z * Ycr * R_1[7] + Z * R_1[8];
 
  
		if(Z_L > d_min_z_ && Z_L< d_max_z_ && Xp > 0)
		// if(confidence_map[offset] > 10 && Z_L > 10 && Z_L< 3000 && Xp > 0)
		{
		    pointcloud[3 * offset + 0] = X_L;
		    pointcloud[3 * offset + 1] = Y_L;
		    pointcloud[3 * offset + 2] = Z_L; 
			
		    depth[offset] = Z_L; 
		}
		else
		{
		    pointcloud[3 * offset + 0] = 0;
		    pointcloud[3 * offset + 1] = 0;
		    pointcloud[3 * offset + 2] = 0; 
			
		    depth[offset] = 0; 
		}

		
		if (-2 == Xcr || -2 == Ycr || -2 == Xpr)
		{
			pointcloud[3 * offset + 0] = 0;
		    pointcloud[3 * offset + 1] = 0;
		    pointcloud[3 * offset + 2] = 0; 
			
		    depth[offset] = 0; 
		}
  
		/******************************************************************/


	}
}

__device__ float mini_bilinear_interpolation(float x, float y, int map_width, float *mapping)
{
	//map_width = 129;

	//先找到这个点所对应的mini中的四个角点
	//然后将这四个点算出来
	//最后双线性插值

	int index_x1 = floor(x / 16);
	int index_y1 = floor((y-1301) / 16);
	int index_x2 = index_x1 + 1;
	int index_y2 = index_y1 + 1;

	int x1 = index_x1 * 16;
	int y1 = index_y1 * 16 + 1301;
	int x2 = x1 + 16;
	int y2 = y1 + 16;

	//因为我生成的表比原来大，所以无需考虑边界条件
	//fq_xy
	float fq11 = mapping[index_y1 *map_width + index_x1];
	float fq21 = mapping[index_y1 *map_width + index_x2];
	float fq12 = mapping[index_y2 *map_width + index_x1];
	float fq22 = mapping[index_y2 *map_width + index_x2];

	float out = (fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1))/256.;

	return out;
}


__global__ void kernel_reconstruct_pointcloud_base_minitable(uint32_t img_height, uint32_t img_width, float* const xL_rotate_x, float* const xL_rotate_y, float* const single_pattern_minimapping, float* const R_1, float b, 
 float* const confidence_map, float* const phase_x,float* const pointcloud, float* const depth)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;


	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		/****************************************************************************/
		//phase to position
		// float Xp = phase_x[serial_id] * 1280.0 / (128.0 * 2 * DF_PI);
		float Xp = phase_x[serial_id] * d_dlp_width_ /d_max_phase_; 

        float Xcr = xL_rotate_x[serial_id];
        float Ycr = xL_rotate_y[serial_id];
		// float Xcr = bilinear_interpolation(idx, idy, 1920, xL_rotate_x);
		// float Ycr = bilinear_interpolation(idx, idy, 1920, xL_rotate_y);
		//修改此处即可，需要自己写一个函数去查表
		float Xpr = mini_bilinear_interpolation(Xp, (Ycr + 1) * 2000, 128, single_pattern_minimapping);
		float delta_X = std::abs(Xcr - Xpr);
		float Z = b / delta_X;

		float X_L = Z * Xcr * R_1[0] + Z * Ycr * R_1[1] + Z * R_1[2];
		float Y_L = Z * Xcr * R_1[3] + Z * Ycr * R_1[4] + Z * R_1[5];
		float Z_L = Z * Xcr * R_1[6] + Z * Ycr * R_1[7] + Z * R_1[8];


		if(confidence_map[serial_id] > d_confidence_ && Z_L > d_min_z_ && Z_L< d_max_z_ && Xp > 0)
		// if (confidence_map[serial_id] > 10 && Z_L > 100 && Z_L < 2000)
		{
			pointcloud[3 * serial_id + 0] = X_L;
			pointcloud[3 * serial_id + 1] = Y_L;
			pointcloud[3 * serial_id + 2] = Z_L;

			depth[serial_id] = Z_L;
		}
		else
		{
			pointcloud[3 * serial_id + 0] = 0;
			pointcloud[3 * serial_id + 1] = 0;
			pointcloud[3 * serial_id + 2] = 0;

			depth[serial_id] = 0;
		}

		/******************************************************************/


	}
}
