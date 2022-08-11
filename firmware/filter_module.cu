#pragma once
#include "filter_module.cuh"

#include "easylogging++.h"
// int image_width_ = 1920;
// int image_height_ = 1200;
  
void filter_reflect_noise(uint32_t img_height, uint32_t img_width,float * const unwrap_map)
{
    // dim3 threadsPerBlock_p(img_width);
    // dim3 blocksPerGrid_p(img_height);


    dim3 threadsPerBlock_p(4, 4);
    // dim3 blocksPerGrid_p(15,2);
    dim3 blocksPerGrid_p((40 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x,
    (30 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y);
   
 
 	cuda_filter_reflect_noise << <blocksPerGrid_p, threadsPerBlock_p >> > ( img_height, img_width, unwrap_map);
}

__global__ void cuda_filter_reflect_noise(uint32_t img_height, uint32_t img_width,float * const unwrap_map)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
  
    int offset_y = idy * 40 + idx; 
 
    int nr = img_height;
    int nc = img_width;

	if (offset_y < img_height)
	{ 
        //读数据

        // float* phasePtr = unwrap_map +offset_y*img_width ;

        float* phasePtr = new float[img_width]; 
        for(int c= 0;c< img_width;c++)
        {
            int offset = offset_y*img_width + c;
            //  printf("offset:%d\n", offset); 
            phasePtr[c] = unwrap_map[offset];
        }

		/*****************************************************************************/
        int flag_here_is_up = 0;//为零表示没有突变，否则表示突变的点的col
        int flag_here_is_down;// 为零表示没有下降趋势，否则表示开始下降的点
        int flag_the_up_count;// 大于10表示没有突起且稀疏的点，否则表示应当给予删除

        int flag_the_up_num = 0;

        int c_before = 0;
		flag_here_is_up = 0;
		float max = -1; 
		// unsigned char* maskPtr = myMask.ptr<unsigned char>(r);
		int count = 0; 
		int flag = 0;
		for (int c = 10; c < nc; c += 1)
		{
			if (phasePtr[c_before] == -10.)
			{
				// 初次进入 
				for (int c_temp = 0; c_temp < nc; c_temp += 1)
				{
					if (phasePtr[c_temp] != -10.)
					{
						c_before = c_temp;
						c = c_before;
						flag = 1;
						break;
					}
				}
				if (flag == 0)
				{
					c = nc;
					//std::cout << "出" << std::endl;
					continue;
				}
				//if (phasePtr[c_before] != -10)
					//std::cout << "有值" << c_before << std::endl;
			}
			if (phasePtr[c] == -10)
			{
				int flag = 0;
				for (int c_temp = c; c_temp < nc; c_temp += 1)
				{
					if (phasePtr[c_temp] != -10)
					{
						c = c_temp;
						flag = 1;
						break;
					}
				}
				if (flag == 0)
				{
					c = nc;
					continue;
				}
			}

			if (phasePtr[c] <= phasePtr[c_before] && count < 100)
			{
				if (count == 0)
				{
					flag_here_is_up = c;
				}
				// 若右边比左边小
				count += 1;
				//phasePtr[c] = -1;
				//std::cout << "f" << std::endl;
			}
			else
			{
				c_before = c;
				if (count == 0)
				{
					flag_the_up_num += 1;
					//count = 0;

					flag_here_is_up = 0;
				}
				else
				{
					int num_up = 0;
					//if (1)
					//{
					//	flag_here_is_up = c - count - 5;
					//}
					int temp_num = -1;
					for (int del = 0; del < 3; del += 1)
					{

						while (phasePtr[flag_here_is_up + temp_num] == -10)
						{
							if (flag_here_is_up + temp_num == 0)
							{
								break;
							}
							temp_num -= 1;
						}

						phasePtr[flag_here_is_up + temp_num] = -10;
					}
					float min = 0;
					int num_temp_ = 0;
					int count_temp = 0;
					for (int cc = flag_here_is_up; cc < c; cc += 1)
					{
						// 添加判断语句，使得需要使用的
						// 倒着循环过去，记录最小值，小于最小值的要保留，否则删除
						num_temp_ += 1;
						// 通过while寻找到前一个点，若更大则删除，若更小，则记录
						while (phasePtr[c - num_temp_] == -10)
						{
							num_temp_ += 1;
							cc += 1;
						}
						if (min == 0)
						{
							min = phasePtr[c - num_temp_];
							phasePtr[c - num_temp_] = -10;
							continue;
						}
						if (phasePtr[c - num_temp_] < min)
						{
							min = phasePtr[c - num_temp_];
						}
						else
						{
							phasePtr[c - num_temp_] = -10;
							// maskPtr[c - num_temp_] = 255;
						}
					}
					flag_here_is_up = 0;
					count = 0;
				}
			}

		}

 

		/*****************************************************************************/
        for(int c= 0;c< img_width;c++)
        {
            unwrap_map[offset_y*img_width + c] = phasePtr[c];
        }

        delete []phasePtr;
        /*****************************************************************************/
	}
}