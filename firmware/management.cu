#include "management.cuh"


int h_image_width_ = 0;
int h_image_height_ = 0;


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

dim3 threadsPerBlock(8, 8);
dim3 blocksPerGrid((d_image_width_ + threadsPerBlock.x - 1) / threadsPerBlock.x,
(d_image_height_ + threadsPerBlock.y - 1) / threadsPerBlock.y);



// int d_image_width_ = 1920;
// int d_image_height_ = 1200;
// bool load_calib_data_flag_ = false;


SystemConfigDataStruct cuda_system_config_settings_machine_;
void cuda_set_param_system_config(SystemConfigDataStruct param)
{
	cuda_system_config_settings_machine_ = param;
}

bool cuda_set_projector_version(int version)
{
    switch (version)
    {
    case DF_PROJECTOR_3010:
    {
		int dlp_width = 1280;
		int dlp_height = 720;
		cuda_set_param_dlp_resolution(dlp_width,dlp_height);
 

        return true;
    }
    break;

    case DF_PROJECTOR_4710:
    {
		int dlp_width = 1920;
		int dlp_height = 1080;
 
		cuda_set_param_dlp_resolution(dlp_width,dlp_height);

 
        return true;
    }
    break;

    default:
        break;
    }

	return false;
}

bool cuda_set_camera_resolution(int width,int height)
{
	h_image_width_ = width;
	h_image_height_ = height;
 
	d_image_width_ = width;
	d_image_height_ = height;

	cudaError_t error_code = cudaMemcpyToSymbol(d_image_width_, &width, sizeof(int)); 
	if(error_code!= cudaSuccess)
	{
		return false;
	}
		
	error_code = cudaMemcpyToSymbol(d_image_height_, &height, sizeof(int));

	if(error_code!= cudaSuccess)
	{
		return false;
	}
 

	blocksPerGrid.x = (width + threadsPerBlock.x - 1) / threadsPerBlock.x;
	blocksPerGrid.y = (height + threadsPerBlock.y - 1) / threadsPerBlock.y;

	return true;
}


//分配basic内存
bool cuda_malloc_basic_memory()
{
    for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
    {
        cudaMalloc((void **)&d_patterns_list_[i], d_image_height_ * d_image_width_ * sizeof(unsigned char)); 
    }

    // cudaBindTexture(0,texture_patterns_0,d_patterns_list_[0]);
	// cudaBindTexture(0,texture_patterns_1,d_patterns_list_[1]);
	// cudaBindTexture(0,texture_patterns_2,d_patterns_list_[2]);
	// cudaBindTexture(0,texture_patterns_3,d_patterns_list_[3]);
	// cudaBindTexture(0,texture_patterns_4,d_patterns_list_[4]);
	// cudaBindTexture(0,texture_patterns_5,d_patterns_list_[5]);
	// cudaBindTexture(0,texture_patterns_6,d_patterns_list_[6]);
	// cudaBindTexture(0,texture_patterns_7,d_patterns_list_[7]);
	// cudaBindTexture(0,texture_patterns_8,d_patterns_list_[8]);
	// cudaBindTexture(0,texture_patterns_9,d_patterns_list_[9]);
	// cudaBindTexture(0,texture_patterns_10,d_patterns_list_[10]);
	// cudaBindTexture(0,texture_patterns_11,d_patterns_list_[11]);
	// cudaBindTexture(0,texture_patterns_12,d_patterns_list_[12]);
	// cudaBindTexture(0,texture_patterns_13,d_patterns_list_[13]);
	// cudaBindTexture(0,texture_patterns_14,d_patterns_list_[14]);
	// cudaBindTexture(0,texture_patterns_15,d_patterns_list_[15]);
	// cudaBindTexture(0,texture_patterns_16,d_patterns_list_[16]);
	// cudaBindTexture(0,texture_patterns_17,d_patterns_list_[17]);
	// cudaBindTexture(0,texture_patterns_18,d_patterns_list_[18]);

 

	for (int i = 0; i< MAX_WRAP_NUMBER; i++)
	{
		cudaMalloc((void**)&d_wrap_map_list_[i], d_image_height_*d_image_width_ * sizeof(float));
		cudaMalloc((void**)&d_confidence_map_list_[i], d_image_height_*d_image_width_ * sizeof(float)); 
	}

	for (int i = 0; i< MAX_UNWRAP_NUMBER; i++)
	{
		cudaMalloc((void**)&d_unwrap_map_list_[i], d_image_height_*d_image_width_ * sizeof(float)); 
	} 

	cudaMalloc((void**)&d_brightness_map_, d_image_height_*d_image_width_ * sizeof(unsigned char)); 
	cudaMalloc((void**)&d_mask_map_, d_image_height_*d_image_width_ * sizeof(unsigned char)); 
	cudaMalloc((void**)&d_fisher_mask_, d_image_height_ * d_image_width_ * sizeof(unsigned char));


	cudaMalloc((void**)&d_camera_intrinsic_, 3*3 * sizeof(float));
	cudaMalloc((void**)&d_project_intrinsic_, 3 * 3 * sizeof(float));

	cudaMalloc((void**)&d_camera_distortion_, 1* 5 * sizeof(float));
	cudaMalloc((void**)&d_projector_distortion_, 1 * 5 * sizeof(float));

	cudaMalloc((void**)&d_rotation_matrix_, 3 * 3 * sizeof(float));
	cudaMalloc((void**)&d_translation_matrix_, 1 * 3 * sizeof(float));


	cudaMalloc((void**)&d_fisher_confidence_map, d_image_height_*d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_point_cloud_map_, 3*d_image_height_*d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_depth_map_, d_image_height_*d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_triangulation_error_map_, d_image_height_*d_image_width_ * sizeof(float));
 
 	cudaMalloc((void**)&d_single_pattern_mapping_, 4000*2000 * sizeof(float)); 
	cudaMalloc((void**)&d_single_pattern_minimapping_, 128*128 * sizeof(float)); 
	cudaMalloc((void**)&d_xL_rotate_x_, d_image_height_*d_image_width_ * sizeof(float)); 
	cudaMalloc((void**)&d_xL_rotate_y_, d_image_height_*d_image_width_ * sizeof(float)); 
	cudaMalloc((void**)&d_R_1_, 3*3 * sizeof(float)); 

    LOG(INFO)<<"d_image_width_: "<<d_image_width_;
    LOG(INFO)<<"d_image_height_: "<<d_image_height_;
	cudaDeviceSynchronize();
	return true;
}

bool cuda_free_basic_memory()
{

	for (int i = 0; i< MAX_PATTERNS_NUMBER; i++)
	{  
		cudaFree(d_patterns_list_[i]); 
	}

	// cudaUnbindTexture(texture_patterns_0);
	// cudaUnbindTexture(texture_patterns_1);
	// cudaUnbindTexture(texture_patterns_2);
	// cudaUnbindTexture(texture_patterns_3);
	// cudaUnbindTexture(texture_patterns_4);
	// cudaUnbindTexture(texture_patterns_5);
	// cudaUnbindTexture(texture_patterns_6);
	// cudaUnbindTexture(texture_patterns_7);
	// cudaUnbindTexture(texture_patterns_8);
	// cudaUnbindTexture(texture_patterns_9);
	// cudaUnbindTexture(texture_patterns_10);
	// cudaUnbindTexture(texture_patterns_11);
	// cudaUnbindTexture(texture_patterns_12);
	// cudaUnbindTexture(texture_patterns_13);
	// cudaUnbindTexture(texture_patterns_14);
	// cudaUnbindTexture(texture_patterns_15);
	// cudaUnbindTexture(texture_patterns_16);
	// cudaUnbindTexture(texture_patterns_17);
	// cudaUnbindTexture(texture_patterns_18);

	for (int i = 0; i< MAX_WRAP_NUMBER; i++)
	{  
		cudaFree(d_wrap_map_list_[i]);
		cudaFree(d_confidence_map_list_[i]); 
	}

	for (int i = 0; i< MAX_UNWRAP_NUMBER; i++)
	{ 
		cudaFree(d_unwrap_map_list_[i]); 
	}

	cudaFree(d_fisher_confidence_map);
	cudaFree(d_fisher_mask_);
    cudaFree(d_mask_map_);
    cudaFree(d_brightness_map_);
    cudaFree(d_point_cloud_map_);
    cudaFree(d_depth_map_);
    cudaFree(d_triangulation_error_map_);

    cudaFree(d_camera_intrinsic_);
	cudaFree(d_project_intrinsic_); 
	cudaFree(d_camera_distortion_);
	cudaFree(d_projector_distortion_); 
	cudaFree(d_rotation_matrix_);
	cudaFree(d_translation_matrix_);
 
    cudaFree(d_single_pattern_mapping_);
    cudaFree(d_single_pattern_minimapping_);
    cudaFree(d_xL_rotate_x_);
    cudaFree(d_xL_rotate_y_);
    cudaFree(d_R_1_);
 

	return true;
}

 //分配hdr内存
bool cuda_malloc_hdr_memory()
{
	for (int i = 0; i< D_HDR_MAX_NUM; i++)
	{
		cudaMalloc((void**)&d_hdr_depth_map_list_[i], d_image_height_*d_image_width_ * sizeof(float));
		cudaMalloc((void**)&d_hdr_brightness_list_[i], d_image_height_*d_image_width_ * sizeof(unsigned char)); 
		cudaMalloc((void**)&d_hdr_bright_pixel_sum_list_[i], 1 * sizeof(float)); 
	}
	cudaDeviceSynchronize();
	return true;
}

bool cuda_free_hdr_memory()
{
    for (int i = 0; i< D_HDR_MAX_NUM; i++)
	{ 
		cudaFree(d_hdr_depth_map_list_[i]);
		cudaFree(d_hdr_brightness_list_[i]);
		cudaFree(d_hdr_bright_pixel_sum_list_[i]);
	}
	
	return true;
}

//分配repetition内存
bool cuda_malloc_repetition_memory()
{
	//分配重复patterns数据
	for(int i= 0;i< D_REPETITIONB_MAX_NUM*6;i++)
	{
		cudaMalloc((void**)&d_repetition_patterns_list_[i], d_image_height_*d_image_width_ * sizeof(unsigned char)); 
	}

	for(int i= 0;i< 6;i++)
	{
		cudaMalloc((void**)&d_repetition_merge_patterns_list_[i], d_image_height_*d_image_width_ * sizeof(unsigned short)); 
	}
 
 	for(int i= 0;i< D_REPETITION_02_MAX_NUM;i++)
	{
		cudaMalloc((void**)&d_repetition_02_merge_patterns_list_[i], d_image_height_*d_image_width_ * sizeof(unsigned short)); 
	}
	cudaDeviceSynchronize();
	return true;
}

bool cuda_free_repetition_memory()
{

	//分配重复patterns数据
	for(int i= 0;i< D_REPETITIONB_MAX_NUM*6;i++)
	{
		cudaFree(d_repetition_patterns_list_[i]); 
	}

	for(int i= 0;i< 6;i++)
	{
		cudaFree(d_repetition_merge_patterns_list_[i]);  
	}

	for(int i= 0;i< D_REPETITION_02_MAX_NUM;i++)
	{
		cudaFree(d_repetition_02_merge_patterns_list_[i]);  
	}
	
	return true;
}


/********************************************************************************************/
//copy 
void cuda_copy_calib_data(float* camera_intrinsic, float* project_intrinsic, float* camera_distortion,
	float* projector_distortion, float* rotation_matrix, float* translation_matrix)
{
  
	CHECK(cudaMemcpy(d_camera_intrinsic_, camera_intrinsic, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_project_intrinsic_, project_intrinsic, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpy(d_camera_distortion_, camera_distortion, 1 * 5 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_projector_distortion_, projector_distortion, 1 * 5 * sizeof(float), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpy(d_rotation_matrix_, rotation_matrix, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_translation_matrix_, translation_matrix, 1* 3 * sizeof(float), cudaMemcpyHostToDevice));

	load_calib_data_flag_ = 1;

 
}

void cuda_copy_talbe_to_memory(float* mapping,float* mini_mapping,float* rotate_x,float* rotate_y,float* r_1,float base_line)
{
   
	CHECK(cudaMemcpyAsync(d_R_1_, r_1, 3*3 * sizeof(float), cudaMemcpyHostToDevice)); 
	CHECK(cudaMemcpyAsync(d_single_pattern_minimapping_, mini_mapping, 128 * 128 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_single_pattern_mapping_, mapping, 4000*2000 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_xL_rotate_x_, rotate_x, d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_xL_rotate_y_, rotate_y, d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyHostToDevice));
	
    d_baseline_ = base_line;  
 

	LOG(INFO)<<"d_baseline_: "<<d_baseline_;
	cudaDeviceSynchronize();
}


bool cuda_copy_pattern_to_memory(unsigned char* pattern_ptr,int serial_flag)
{
	if(serial_flag> MAX_PATTERNS_NUMBER)
	{
		return false;
	}

	CHECK(cudaMemcpyAsync(d_patterns_list_[serial_flag], pattern_ptr, d_image_height_*d_image_width_* sizeof(unsigned char), cudaMemcpyHostToDevice)); 
}

void cuda_copy_pointcloud_from_memory(float* pointcloud)
{ 
	CHECK(cudaMemcpy(pointcloud, d_point_cloud_map_, 3 * d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
}

void cuda_copy_depth_from_memory(float* depth)
{
	CHECK(cudaMemcpy(depth, d_depth_map_, d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
} 

void cuda_copy_brightness_from_memory(unsigned char* brightness)
{
	CHECK(cudaMemcpy(brightness, d_brightness_map_, d_image_height_*d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost)); 
}

void cuda_copy_brightness_to_memory(unsigned char* brightness)
{ 
	CHECK(cudaMemcpyAsync(d_brightness_map_, brightness, d_image_height_*d_image_width_* sizeof(unsigned char), cudaMemcpyHostToDevice)); 
}

/********************************************************************************************/


bool cuda_compute_phase_shift(int serial_flag)
{
	 
	switch(serial_flag)
	{
		case 0:
		{ 
        	LOG(INFO)<<"kernel_four_step_phase_shift:"<<d_image_width_;
			int i= 0;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);

				// kernel_four_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> >(serial_flag,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 1:
		{

			int i= 4;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
				
				// kernel_four_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> >(serial_flag,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
			
		}
		break;
		case 2:
		{ 
			int i= 8;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
				
				// kernel_four_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> >(serial_flag,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 3:
		{ 
			int i= 12; 
			kernel_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3],d_patterns_list_[i + 4],d_patterns_list_[i + 5] ,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
 
            
				// cuda_six_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> > (d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
				// cudaDeviceSynchronize();

				// cv::Mat phase(1200, 1920, CV_32F, cv::Scalar(0));
				// CHECK(cudaMemcpy(phase.data, d_wrap_map_list_[serial_flag], 1 * image_height_ * image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
				// cv::imwrite("phase1.tiff",phase);
		}
		break;
		case 4:
		{
			int i= 18;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 5:
		{
			int i= 22;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 6:
		{
			int i= 26;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
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
            kernel_normalize_phase<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], (float)128.0, d_unwrap_map_list_[0]);  
		}
		break; 
		case 1:
		{   
  
            kernel_normalize_phase<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[1], (float)18., d_unwrap_map_list_[1]); 
		}
		break;

		case 2:
		{ 
			kernel_normalize_phase<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[1], (float)72., d_unwrap_map_list_[1]); 
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
            kernel_unwrap_variable_phase<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_wrap_map_list_[0], d_wrap_map_list_[1], 8.0, CV_PI, d_unwrap_map_list_[0]);
  
		}
		break;

		case 2:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], d_wrap_map_list_[2], 4.0,CV_PI, d_unwrap_map_list_[0]); 
			// CHECK ( cudaGetLastError () );
		}
		break;
		case 3:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], d_wrap_map_list_[3], 4.0,1.5, d_unwrap_map_list_[0]); 
 
		}
		break;
		case 4:
		{
 
		}
		break;
		case 5:
		{
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_wrap_map_list_[4], d_wrap_map_list_[5], 8.0,CV_PI, d_unwrap_map_list_[1]);
		}
		break;
		case 6:
		{
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[1], d_wrap_map_list_[6], 4.0,CV_PI, d_unwrap_map_list_[1]);
 
			LOG(INFO)<<"unwrap 6:  ";

		}
		break;
		case 7:
		{
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[1], d_wrap_map_list_[7], 4.0,CV_PI, d_unwrap_map_list_[1]);
 
		 	LOG(INFO)<<"unwrap 7:  ";

		}
		break;
 

		default :
			break;
	}


	return true;
}

bool cuda_unwrap_phase_shift_base_fisher_confidence(int serial_flag)
{

	switch(serial_flag)
	{ 
		case 1:
		{  
            kernel_unwrap_variable_phase_base_confidence<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_wrap_map_list_[0], d_wrap_map_list_[1], 8.0, CV_PI, FISHER_RATE_1, d_fisher_confidence_map, d_unwrap_map_list_[0]);
  
		}
		break;

		case 2:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase_base_confidence << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], d_wrap_map_list_[2], 4.0,CV_PI, FISHER_RATE_2, d_fisher_confidence_map, d_unwrap_map_list_[0]); 
			// CHECK ( cudaGetLastError () );
		}
		break;
		case 3:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase_base_confidence << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], d_wrap_map_list_[3], 4.0,1.5, FISHER_RATE_3, d_fisher_confidence_map, d_unwrap_map_list_[0]); 
 
		}
		break;
		default :
			break;
	}


	return true;
}

/********************************************************************************************************************************************/

bool cuda_generate_pointcloud_base_minitable()
{
		if(1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_reflect_filter)
	{ 
		LOG(INFO)<<"filter_reflect_noise start:"; 
		cuda_filter_reflect_noise(d_unwrap_map_list_[0]); 

		cudaDeviceSynchronize();
		LOG(INFO)<<"filter_reflect_noise end";
	}

	kernel_reconstruct_pointcloud_base_minitable<< <blocksPerGrid, threadsPerBlock>> > (d_image_width_,d_image_height_,d_xL_rotate_x_,d_xL_rotate_y_,d_single_pattern_minimapping_,d_R_1_,d_baseline_,
	d_confidence_map_list_[3],d_unwrap_map_list_[0],d_point_cloud_map_,d_depth_map_);

 
}


bool cuda_generate_pointcloud_base_table()
{
	// cv::Mat phase(2048,2448,CV_32FC1,cv::Scalar(0));
	// CHECK(cudaMemcpy(phase.data, d_unwrap_map_list_[0], 1 * d_image_height_ * d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	// cv::imwrite("phase.tiff", phase);
	
	// if(1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_reflect_filter)
	// { 
	// 	LOG(INFO)<<"filter_reflect_noise start:"; 
	// 	cuda_filter_reflect_noise(d_unwrap_map_list_[0]); 
	//
	// 	cudaDeviceSynchronize();
	// 	LOG(INFO)<<"filter_reflect_noise end";
	// }

	kernel_reconstruct_pointcloud_base_table << <blocksPerGrid, threadsPerBlock>> > (d_image_width_,d_image_height_,d_xL_rotate_x_,d_xL_rotate_y_,d_single_pattern_mapping_,d_R_1_,d_baseline_,
	d_confidence_map_list_[3],d_unwrap_map_list_[0],d_point_cloud_map_,d_depth_map_);

	// cv::Mat depth(2048,2448,CV_32FC1,cv::Scalar(0));
	// CHECK(cudaMemcpy(depth.data, d_depth_map_, 1 * d_image_height_ * d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	// cv::imwrite("depth.tiff", depth);
}

/********************************************************************************************************************************************/

bool cuda_copy_result_to_hdr(int serial_flag,int brigntness_serial)
{
	if(!load_calib_data_flag_)
	{
		return false;
	}
 

	CHECK(cudaMemcpyAsync(d_hdr_depth_map_list_[serial_flag], d_depth_map_, 1 * d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToDevice)); 
	CHECK(cudaMemcpyAsync(d_hdr_brightness_list_[serial_flag], d_patterns_list_[brigntness_serial], 1 * d_image_height_*d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToDevice));

	float val  = 0;
	CHECK(cudaMemcpyAsync(d_hdr_bright_pixel_sum_list_[serial_flag], &val, sizeof(float), cudaMemcpyHostToDevice)); 
 	cuda_count_sum_pixel << <blocksPerGrid, threadsPerBlock >> > (d_hdr_brightness_list_[serial_flag],d_image_height_,d_image_width_,d_hdr_bright_pixel_sum_list_[serial_flag]);
 
	LOG(INFO)<<"cuda_copy_result_to_hdr: "<<serial_flag;
	return true;
}


bool cuda_merge_hdr_data(int hdr_num,float* depth_map, unsigned char* brightness)
{
	
	LOG(INFO)<<"sum pixels ";
	float sum_pixels_list[6];  

    for(int i= 0;i<hdr_num;i++)
    { 
		CHECK(cudaMemcpy(&sum_pixels_list[i], d_hdr_bright_pixel_sum_list_[i], 1* sizeof(float), cudaMemcpyDeviceToHost));
    }
 
 
	std::vector<float> param_list;
	std::vector<int> id; 
	std::vector<bool> flag_list;

	for (int i = 0; i < hdr_num; i++)
	{ 
        param_list.push_back(sum_pixels_list[i]);
		id.push_back(0);
		flag_list.push_back(true);
    } 
   	std::sort(param_list.begin(),param_list.end(),std::greater<float>());
 
 
	for (int i = 0; i < hdr_num; i++)
	{ 
		
		for(int j= 0;j< hdr_num;j++)
		{
			if(param_list[i] == sum_pixels_list[j])
			{
				if(flag_list[j])
				{ 
					id[i] = j;
					flag_list[j] = false; 
					break;
				}
			}
		}
		 
    } 

 
	for (int i = 0; i < hdr_num; i++)
	{ 
        LOG(INFO)<<"sum pixels "<<i<<": "<<sum_pixels_list[i]<<" _ "<<id[i];
    }
 

	switch(hdr_num)
	{
		case 1:
		{

			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_list_[0], 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[0], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		} 
		break;
		case 2:
		{
			cuda_merge_hdr_2 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]], d_hdr_brightness_list_[id[0]], 
				d_hdr_brightness_list_[id[1]], h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);

				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 3:
		{
			cuda_merge_hdr_3 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]], d_hdr_brightness_list_[id[0]], 
				d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 4:
		{
			cuda_merge_hdr_4 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],d_hdr_depth_map_list_[id[3]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], 
				h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 5:
		{
			cuda_merge_hdr_5 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],
				d_hdr_depth_map_list_[id[3]],d_hdr_depth_map_list_[id[4]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]], 
				h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 6:
		{
			cuda_merge_hdr_6 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],
				d_hdr_depth_map_list_[id[3]],d_hdr_depth_map_list_[id[4]],d_hdr_depth_map_list_[id[5]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]], 
				 d_hdr_brightness_list_[id[5]], 
				h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;

		default:
		 		return false;

	}

 	// CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[id[0]], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
 	CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[hdr_num-1], 1*h_image_height_*h_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
	LOG(INFO)<<"DHR Finished!";

	return true;
}
/********************************************************************************************************************************************/

bool cuda_copy_repetition_pattern_to_memory(unsigned char* patterns_ptr,int serial_flag)
{
	CHECK(cudaMemcpyAsync(d_repetition_patterns_list_[serial_flag], patterns_ptr, h_image_height_*h_image_width_* sizeof(unsigned char), cudaMemcpyHostToDevice));
}

bool cuda_merge_repetition_patterns(int repetition_serial)
{

	int merge_serial = repetition_serial%6; 
	kernel_merge_pattern<< <blocksPerGrid, threadsPerBlock >> >(d_repetition_patterns_list_[repetition_serial],h_image_height_, h_image_width_,d_repetition_merge_patterns_list_[merge_serial]);

	return true;
}


bool cuda_compute_merge_phase(int repetition_count)
{

	kernel_merge_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_merge_patterns_list_[0], d_repetition_merge_patterns_list_[1],
		d_repetition_merge_patterns_list_[2],d_repetition_merge_patterns_list_[3],d_repetition_merge_patterns_list_[4],d_repetition_merge_patterns_list_[5] ,
		repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[3], d_confidence_map_list_[3]);

	return true;
}


bool cuda_clear_repetition_02_patterns()
{
	for(int i = 0;i< D_REPETITION_02_MAX_NUM;i++)
	{ 
		 cudaMemset(d_repetition_02_merge_patterns_list_[i], 0,h_image_height_*h_image_width_*sizeof(ushort));
		// CHECK(cudaMemcpyAsync(d_repetition_02_merge_patterns_list_[i], &val,image_width_* image_height_*sizeof(ushort), cudaMemcpyHostToDevice));
	}
	// cudaDeviceSynchronize();
  
  return true;
}

bool cuda_merge_repetition_02_patterns(int repetition_serial)
{
	// int merge_serial = repetition_serial%19; 
	kernel_merge_pattern<< <blocksPerGrid, threadsPerBlock >> >(d_patterns_list_[repetition_serial],h_image_height_, h_image_width_,d_repetition_02_merge_patterns_list_[repetition_serial]);

	return true;
}


bool cuda_compute_merge_repetition_02_phase(int repetition_count,int phase_num)
{
	
	kernel_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[0], d_repetition_02_merge_patterns_list_[1],
		d_repetition_02_merge_patterns_list_[2],d_repetition_02_merge_patterns_list_[3],repetition_count, h_image_height_, h_image_width_,d_wrap_map_list_[0], d_confidence_map_list_[0]);
			
	kernel_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5],
		d_repetition_02_merge_patterns_list_[6],d_repetition_02_merge_patterns_list_[7],repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[1], d_confidence_map_list_[1]);

	kernel_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[8], d_repetition_02_merge_patterns_list_[9],
		d_repetition_02_merge_patterns_list_[10],d_repetition_02_merge_patterns_list_[11],repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[2], d_confidence_map_list_[2]);
	
	kernel_merge_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[12], d_repetition_02_merge_patterns_list_[13],
		d_repetition_02_merge_patterns_list_[14],d_repetition_02_merge_patterns_list_[15],d_repetition_02_merge_patterns_list_[16],d_repetition_02_merge_patterns_list_[17] ,
		repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[3], d_confidence_map_list_[3]);

	if(1 == phase_num)
	{
		kernel_merge_brigntness_map<< <blocksPerGrid, threadsPerBlock >> >(d_repetition_02_merge_patterns_list_[18],repetition_count,h_image_height_, h_image_width_,d_brightness_map_);
	}
	else if (2 == phase_num)
	{

		int i = 18;
		kernel_merge_four_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[i + 0], d_repetition_02_merge_patterns_list_[i + 1],
																			   d_repetition_02_merge_patterns_list_[i + 2], d_repetition_02_merge_patterns_list_[i + 3], repetition_count, h_image_height_, h_image_width_,d_wrap_map_list_[4], d_confidence_map_list_[4]);

		i = 22;
		kernel_merge_four_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[i + 0], d_repetition_02_merge_patterns_list_[i + 1],
																			   d_repetition_02_merge_patterns_list_[i + 2], d_repetition_02_merge_patterns_list_[i + 3], repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[5], d_confidence_map_list_[5]);

		i = 26;
		kernel_merge_four_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[i + 0], d_repetition_02_merge_patterns_list_[i + 1],
																			   d_repetition_02_merge_patterns_list_[i + 2], d_repetition_02_merge_patterns_list_[i + 3], repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[6], d_confidence_map_list_[6]);

		i = 30;
		kernel_merge_six_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[i + 0], d_repetition_02_merge_patterns_list_[i + 1],
																			  d_repetition_02_merge_patterns_list_[i + 2], d_repetition_02_merge_patterns_list_[i + 3], d_repetition_02_merge_patterns_list_[i + 4], d_repetition_02_merge_patterns_list_[i + 5],
																			  repetition_count, h_image_height_, h_image_width_, d_wrap_map_list_[7], d_confidence_map_list_[7]);

		kernel_merge_brigntness_map<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[36], repetition_count, h_image_height_, h_image_width_,d_brightness_map_);

		 
	}

	return true;
}
/********************************************************************************************************************************************/
//filter
void cuda_remove_points_base_radius_filter(float dot_spacing,float radius,int threshold_num)
{
	LOG(INFO)<<"remove_base_radius_filter start:"; 
 
	// //相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12000 mm，典型值0.54mm（1200） 

	float d2 = dot_spacing*dot_spacing;
	float r2 = radius*radius;
	
    // cudaFuncSetCacheConfig (cuda_filter_radius_outlier_removal, cudaFuncCachePreferL1);
	kernel_filter_radius_outlier_removal << <blocksPerGrid, threadsPerBlock >> > (h_image_height_,h_image_width_,d_point_cloud_map_,d_mask_map_,d2,r2,threshold_num); 
	
	cudaDeviceSynchronize();

	// cv::Mat mask(1200, 1920, CV_8U, cv::Scalar(0));
	// CHECK(cudaMemcpy(mask.data, d_mask_map_, 1 * h_image_height_ * h_image_width_ * sizeof(uchar), cudaMemcpyDeviceToHost));
	// cv::imwrite("mask.bmp", mask);
	LOG(INFO)<<"remove start:";
	kernel_removal_points_base_mask << <blocksPerGrid, threadsPerBlock >> > (h_image_height_,h_image_width_,d_point_cloud_map_,d_depth_map_,d_mask_map_); 

    cudaDeviceSynchronize();
 

	LOG(INFO)<<"remove_base_radius_filter finished!";
}


void cuda_filter_reflect_noise(float * const unwrap_map)
{
    // dim3 threadsPerBlock_p(img_width);
    // dim3 blocksPerGrid_p(img_height);

	//按行来组织线程
    dim3 threadsPerBlock_p(4, 4);
    // dim3 blocksPerGrid_p(15,2);
    dim3 blocksPerGrid_p;
	if(1200 == h_image_height_)
	{
		blocksPerGrid_p.x = (40 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (30 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	else if(2048 == h_image_height_)
	{
		blocksPerGrid_p.x = (64 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (32 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}

 
 	kernel_filter_reflect_noise << <blocksPerGrid_p, threadsPerBlock_p >> > ( h_image_height_,h_image_width_, unwrap_map);
}


void fisher_filter(float fisher_confidence_val)
{
	//按行来组织线程
    dim3 threadsPerBlock_p(4, 4);
    dim3 blocksPerGrid_p;
	if(1200 == h_image_height_)
	{
		blocksPerGrid_p.x = (40 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (30 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	else if(2048 == h_image_height_)
	{
		blocksPerGrid_p.x = (64 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (32 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	LOG(INFO)<<"fisher start"; 
	kernel_fisher_filter <<< blocksPerGrid_p, threadsPerBlock_p >>> (h_image_height_, h_image_width_, (FISHER_CENTER_LOW + (fisher_confidence_val * FISHER_CENTER_RATE)), d_fisher_confidence_map, d_fisher_mask_, d_unwrap_map_list_[0]);//
	cudaDeviceSynchronize();
	LOG(INFO)<<"fisher end"; 
}



/*****************************************************************************************************************************************************/
//repetition

void cuda_copy_phase_from_cuda_memory(float* phase_x,float* phase_y)
{
	CHECK(cudaMemcpy(phase_x, d_unwrap_map_list_[0], d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
	CHECK(cudaMemcpy(phase_y, d_unwrap_map_list_[1], d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
}




/*****************************************************************************************************************************************************/









