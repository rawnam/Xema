#include "scan3d.h"
#include "easylogging++.h"
// #include "encode_cuda.cuh" 
#include "../test/LookupTableFunction.h"  
#include "protocol.h"
#include "management.cuh" 

 
Scan3D::Scan3D()
{
    max_camera_exposure_ = 28000;
    min_camera_exposure_ = 1000;

    led_current_ = 1023;
    camera_exposure_ = 12000;
    camera_gain_ = 0;

    generate_brightness_model_ = 1;
    generate_brightness_exposure_ = 12000;

    camera_opened_flag_ = false;

    
    fisher_confidence_val_ = -30;
}

Scan3D::~Scan3D()
{

}

int Scan3D::init()
{
    int ret = 0;
    //光机初始化
    ret = lc3010_.init();
    if (DF_SUCCESS != ret)
    {
        LOG(ERROR) << "lc3010 init FAILED; CODE : " << ret;
    }

    ret = lc3010_.SetLedCurrent(255, 255, 255);
    if (DF_SUCCESS != ret)
    {
        LOG(ERROR) << "lc3010 SetLedCurrent FAILED; CODE : " << ret;
    }

    lc3010_.enable_solid_field();
    LOG(INFO)<<"lc3010 init";

    //相机初始化
    camera_ = new CameraGalaxy(); 
    if(!camera_->openCamera())
    { 
        LOG(INFO)<<"Open Galaxy Camera Error!";

        delete camera_;
        camera_ = new CameraBasler();
        if (!camera_->openCamera())
        {
            LOG(INFO) << "Open Basler Camera Error!"; 
            camera_opened_flag_ = false;
            return DF_ERROR_2D_CAMERA;
        }
        else
        { 
            LOG(INFO)<<"Open Basler Camera:";
            camera_opened_flag_ = true;
        }
    }
    else
    {
        LOG(INFO)<<"Open Galaxy Camera:";
        camera_opened_flag_ = true;
    }
    camera_->switchToExternalTriggerMode();    
    LOG(INFO)<<"switchToExternalTriggerMode!"; 

    camera_->getImageSize(image_width_,image_height_);

    float min_exposure = 0;
    camera_->getMinExposure(min_exposure);
    LOG(INFO)<<"scan3d min_exposure: "<<min_exposure; 
    lc3010_.set_camera_min_exposure(min_exposure);

    buff_brightness_ = new unsigned char[image_width_*image_height_];
    buff_depth_ = new float[image_width_*image_height_];
    buff_pointcloud_ = new float[3*image_width_*image_height_];
 
    /*****************************************************************************************/
    

    lc3010_.read_dmd_device_id(projector_version_);
     
    if(!setProjectorVersion(projector_version_))
    {
        LOG(INFO)<<"set Camera Version Failed!"; 
    }

    /**********************************************************************************************/

    if(0!= image_width_ && 0!= image_height_)
    {
        cuda_set_camera_resolution(image_width_, image_height_);
        // cuda初始化
        //  cuda_malloc_memory();
        cuda_malloc_basic_memory();
        cuda_malloc_hdr_memory();
        cuda_malloc_repetition_memory();

        if (!loadCalibData())
        {
            LOG(INFO)<<"Load Calib Error!"; 
            return false;
        }
    }
    else
    {
        return false;
    }




    return true;
 
} 


int Scan3D::reopenCamera()
{
    LOG(ERROR)<<"reopen Camera:";
    if(!camera_->closeCamera())
    {
        LOG(ERROR)<<"close Camera Error!";
    }

    if(camera_->openCamera())
    {
        camera_->setExposure(camera_exposure_);
    }
    else
    {
        return DF_ERROR_2D_CAMERA;
    }
   
    LOG(ERROR)<<"reopen Camera finished!";
    return DF_SUCCESS;    
}

int Scan3D::initCache()
{
    std::memset(buff_brightness_,0,sizeof(char)*image_width_*image_height_);
    std::memset(buff_depth_,0,sizeof(float)*image_width_*image_height_);
    std::memset(buff_pointcloud_,0,sizeof(float)*image_width_*image_height_*3); 
}

bool Scan3D::cameraIsValid()
{
    return camera_opened_flag_;
}

bool Scan3D::triggerLineIsValid()
{ 
    if(!camera_->switchToExternalTriggerMode())
    {
        LOG(INFO) << "switch To External Trigger Mode failed!";
        return false;
    }

    lc3010_.pattern_mode_brightness();

    if(!camera_->streamOn())
    {
        LOG(INFO) << "Stream On failed!";
        return false; 
    } 
    
    LOG(INFO) << "Stream On";
    lc3010_.start_pattern_sequence();

    unsigned char *buff= new unsigned char[image_width_*image_height_];
    if (!camera_->grap(buff))
    {
        LOG(INFO) << "grap  failed!";
        delete[] buff;
        camera_->streamOff();
        return false;
    } 

    delete []buff; 
    camera_->streamOff();

    LOG(INFO) << "Stream Off";

    return true;
}

bool Scan3D::setParamHdr(int num,std::vector<int> led_list,std::vector<int> exposure_list)
{
    if(led_list.size() != exposure_list.size() || exposure_list.size() != 6)
    {
        return false;
    }

    hdr_num_ = num;

    led_current_list_ = led_list;
    camera_exposure_list_ = exposure_list;

    return true;
}

bool Scan3D::setParamExposure(float exposure)
{

    if(exposure > max_camera_exposure_ || exposure < min_camera_exposure_)
    {
        return false;
    }

    if (!camera_->setExposure(exposure))
    {
        return false;
    }

    lc3010_.set_camera_exposure(exposure);
  
    camera_exposure_ = exposure;

    return true;
}

bool Scan3D::setParamGain(float gain)
{
    if (!camera_->setGain(gain))
    {
         return false;
    }

    camera_gain_ = gain;
    
    return true;
}


bool Scan3D::setParamLedCurrent(int current)
{

    if(DF_SUCCESS!= lc3010_.SetLedCurrent(current,current,current))
    {
        LOG(ERROR)<<"Set Led Current failed";
        return false;
    }

    led_current_ = current;

    return true;
}


bool Scan3D::setParamConfidence(float confidence)
{
    return cuda_set_param_confidence(confidence);  
}

bool Scan3D::setProjectorVersion(int version)
{
    switch (version)
    {
    case DF_PROJECTOR_3010:
    {
        cuda_set_projector_version(DF_PROJECTOR_3010);
        max_camera_exposure_ = 100000;
        min_camera_exposure_ = 1700;
        return true;
    }
    break;

    case DF_PROJECTOR_4710:
    {

        cuda_set_projector_version(DF_PROJECTOR_4710);
        max_camera_exposure_ = 28000; 
        min_camera_exposure_ = 1700;
        return true;
    }
    break;

    default:
        break;
    }

    return false;
}


void Scan3D::getProjectorVersion(int &version)
{
    version = projector_version_;
}

bool Scan3D::setParamGenerateBrightness(int model, int exposure)
{
    if (model == 1 || model == 2 || model == 3)
    {
        generate_brightness_model_ = model;
        generate_brightness_exposure_ = exposure;

        return true;
    }

    return false;
}

void Scan3D::setParamSystemConfig(SystemConfigDataStruct param)
{
    system_config_settings_machine_ = param;
    cuda_set_param_system_config(param);
}

void Scan3D::setParamFisherConfidence(float confidence)
{
    fisher_confidence_val_ = confidence; 
}
/******************************************************************************************************************************************/


bool Scan3D::captureTextureImage(int model,float exposure,unsigned char* buff)
{ 


    switch (model)
    {
        case 1:
        { 
            setParamExposure(exposure);
            camera_->switchToExternalTriggerMode();
	        lc3010_.pattern_mode_brightness(); 
            camera_->streamOn();
            LOG(INFO) << "Stream On"; 
            lc3010_.start_pattern_sequence();

            if(!camera_->grap(buff))
            { 
                 LOG(INFO) << "grap brightness failed!";
            }
            else
            {
                LOG(INFO) << "grap brightness!";
            }
            camera_->streamOff();
            LOG(INFO) << "Stream Off";
        }
        break;
        case 2:
        {
             
            lc3010_.stop_pattern_sequence();
            lc3010_.init();
            //发光，自定义曝光时间
            lc3010_.enable_solid_field();

            LOG(INFO) << "sleep:";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            LOG(INFO) << "end";

            camera_->switchToInternalTriggerMode(); 
            camera_->setExposure(exposure); 
            camera_->streamOn();
            LOG(INFO) << "Stream On"; 

            if(!camera_->grap(buff))
            { 
                 LOG(INFO) << "grap brightness failed!";
            }
            else
            {
                LOG(INFO) << "grap brightness!";
            }

            
            camera_->streamOff();
            LOG(INFO) << "Stream Off";
            
            lc3010_.disable_solid_field();
            camera_->switchToExternalTriggerMode();
            camera_->setExposure(camera_exposure_);
        }
        break;
    case 3:
    {

            // lc3010_.stop_pattern_sequence(); 
            // lc3010_.init(); 

            camera_->switchToInternalTriggerMode(); 
            if(!camera_->setExposure(exposure))
            {
                LOG(INFO) << "setExposure Failed!";
            } 
            else
            {  
                LOG(INFO) << "set Exposure: "<<exposure;
            }
            camera_->streamOn();
            LOG(INFO) << "Stream On"; 
  
            if(!camera_->grap(buff))
            { 
                 LOG(INFO) << "grap generate brightness failed!";
            }
            else
            {
                LOG(INFO) << "grap generate brightness!";
            }

            
            camera_->streamOff();
            LOG(INFO) << "Stream Off";
            camera_->switchToExternalTriggerMode();
            camera_->setExposure(camera_exposure_);

    }
    break; 
    default:
        break;
    }

 

    return true;
}

/*******************************************************************************************************************************************/

bool Scan3D::captureRaw01(unsigned char* buff)
{

    lc3010_.pattern_mode01();
    if (!camera_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return false;
    }

    lc3010_.start_pattern_sequence();

    int img_size = image_width_*image_height_;

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for (int i = 0; i < 24; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_->grap(img_ptr))
        {
            camera_->streamOff();
            return false;
        }
 
        memcpy(buff+img_size*i, img_ptr, img_size);
  
    }

    camera_->streamOff();

        
    if (1 != generate_brightness_model_)
    { 
        captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,img_ptr);
        memcpy(buff+img_size*23, img_ptr, img_size);
    }
  
    delete []img_ptr;
}

bool Scan3D::captureRaw02(unsigned char* buff)
{
 
    lc3010_.pattern_mode02();
    if (!camera_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return false;
    }

    lc3010_.start_pattern_sequence();

    int img_size = image_width_*image_height_;

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for (int i = 0; i < 37; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_->grap(img_ptr))
        {
            camera_->streamOff();
            return false;
        }
 
        memcpy(buff+img_size*i, img_ptr, img_size);
  
    }

    camera_->streamOff();

    
    if (1 != generate_brightness_model_)
    { 
        captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,img_ptr);
        memcpy(buff+img_size*36, img_ptr, img_size);
    }

    delete []img_ptr;
  
 
    return true;
}

bool Scan3D::captureRaw03(unsigned char* buff)
{

    lc3010_.pattern_mode03();
    if (!camera_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return false;
    }

    lc3010_.start_pattern_sequence();

    int img_size = image_width_*image_height_;

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for (int i = 0; i < 31; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_->grap(img_ptr))
        {
            camera_->streamOff();
            return false;
        }
 
        memcpy(buff+img_size*i, img_ptr, img_size);
  
    }

    camera_->streamOff();
    if (1 != generate_brightness_model_)
    {
        captureTextureImage(generate_brightness_model_, generate_brightness_exposure_, img_ptr);
        memcpy(buff + img_size * 30, img_ptr, img_size);
    }

    delete []img_ptr;
 
    return true;
}

bool Scan3D::captureRaw04(unsigned char* buff)
{

    lc3010_.pattern_mode04();
    if (!camera_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return false;
    }

    lc3010_.start_pattern_sequence();

    int img_size = image_width_*image_height_;

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for (int i = 0; i < 19; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_->grap(img_ptr))
        {
            camera_->streamOff();
            return false;
        }
 
        memcpy(buff+img_size*i, img_ptr, img_size);
  
    }

    camera_->streamOff();

    if (1 != generate_brightness_model_)
    {
        captureTextureImage(generate_brightness_model_, generate_brightness_exposure_, img_ptr);
        memcpy(buff + img_size * 18, img_ptr, img_size);
    }

    delete[] img_ptr;

    return true;
}


bool Scan3D::captureRaw04Repetition01(int repetition_count,unsigned char* buff)
{
    
    // lc3010_.pattern_mode04_repetition(repetition_count);
    // if (!camera_->streamOn())
    // {
    //     LOG(INFO) << "Stream On Error";
    //     return false;
    // }

    // lc3010_.start_pattern_sequence();

    // int img_size = image_width_*image_height_;

    // unsigned char *img_ptr= new unsigned char[image_width_*image_height_]; 

    // int capture_num= 19 + 6*(repetition_count-1);

    // for (int i = 0; i < capture_num; i++)
    // {
    //     LOG(INFO)<<"grap "<<i<<" image:";
    //     if (!camera_->grap(img_ptr))
    //     {
    //         camera_->streamOff();
    //         return false;
    //     }
 
    //     memcpy(buff+img_size*i, img_ptr, img_size);
  
    // }

    // delete []img_ptr;
    // camera_->streamOff();
   
    return true;
}


bool Scan3D::capturePhase02Repetition02(int repetition_count,float* phase_x,float* phase_y,unsigned char* brightness)
{
    cuda_clear_repetition_02_patterns();

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for (int r = 0; r < repetition_count; r++)
    {
        int n = 0;
  
            LOG(INFO) << "pattern_mode02";
            lc3010_.pattern_mode02();
            
            if(!camera_->streamOn())
            {
                LOG(INFO) << "stream on failed!";
                return false;
            } 
            lc3010_.start_pattern_sequence();
            LOG(INFO) << "start_pattern_sequence";

            for (int i = 0; i < 37; i++)
            {
                LOG(INFO) << "receiving " << i << "th image";
                bool status = camera_->grap(img_ptr);
                LOG(INFO) << "status=" << status;

                if (status)
                {
                    if (i != 36 || 1 == generate_brightness_model_)
                    {
                        cuda_copy_pattern_to_memory(img_ptr, i);
                        cuda_merge_repetition_02_patterns(i);
                    }
                }
                else
                {
                    LOG(INFO) << "grad failed!";
                    camera_->streamOff(); 
                    delete []img_ptr;
                    return false;
                }
 
     
            }

            camera_->streamOff(); 
            lc3010_.stop_pattern_sequence();

            if (1 != generate_brightness_model_)
            {

                captureTextureImage(generate_brightness_model_, generate_brightness_exposure_, img_ptr);

                cuda_copy_pattern_to_memory(img_ptr, 36);
                cuda_merge_repetition_02_patterns(36);
            }

            /***********************************************************************************************/ 
    }


    delete []img_ptr;

    cuda_compute_merge_repetition_02_phase(repetition_count, 2);
    LOG(INFO) << "parallel_cuda_compute_mergerepetition_02_phase";
    cuda_unwrap_phase_shift(1);
    cuda_unwrap_phase_shift(2);
    cuda_unwrap_phase_shift(3);
    cuda_unwrap_phase_shift(5);
    cuda_unwrap_phase_shift(6);
    cuda_unwrap_phase_shift(7);
    cuda_normalize_phase(0);
    cuda_normalize_phase(2);

    cuda_copy_phase_from_cuda_memory(phase_x, phase_y);
    cuda_copy_brightness_from_memory(brightness); 
 
    return true;
}

bool Scan3D::captureFrame04()
{
 
    lc3010_.pattern_mode04();
    LOG(INFO) << "Stream On:";
    if (!camera_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return false;
    }

    lc3010_.start_pattern_sequence();

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for (int i = 0; i < 19; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_->grap(img_ptr))
        {
            
            delete[] img_ptr; 
            camera_->streamOff();
            return false;
        }
        LOG(INFO)<<"finished!";

        if(18 == i)
        {
            cuda_copy_brightness_to_memory(img_ptr);
        }

        cuda_copy_pattern_to_memory(img_ptr, i);

        // copy to gpu
        switch (i)
        {
        case 4:
        {
        LOG(INFO)<<"cuda_compute_phase_shift:";
            cuda_compute_phase_shift(0); 
        }
        break;
        case 8:
        {
            cuda_compute_phase_shift(1);
        }
        break;
        case 10:
        {
            cuda_unwrap_phase_shift(1);
        }
        break;
        case 12:
        {
            cuda_compute_phase_shift(2);
        }
        break;
        case 15:
        {
            cuda_unwrap_phase_shift(2);
        }
        break;
        case 18:
        {


            cuda_compute_phase_shift(3);
            cuda_unwrap_phase_shift(3);
            cuda_normalize_phase(0);

            cuda_generate_pointcloud_base_table();
            //  cudaDeviceSynchronize();
            LOG(INFO) << "cuda_generate_pointcloud_base_table";
        }

        default:
            break;
        }
    }

    delete[] img_ptr;

    camera_->streamOff();
    LOG(INFO) << "Stream Off";

 
    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_pointcloud_from_memory(buff_pointcloud_);

    if (1 == generate_brightness_model_)
    { 
        cuda_copy_brightness_from_memory(buff_brightness_);
    }
    else
    {

        captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    }

 
    return true;
}

int Scan3D::captureFrame04BaseConfidence()
{
    
    int ret = DF_SUCCESS;

    ret = lc3010_.pattern_mode04();
    if(DF_SUCCESS != ret)
    {
        return ret;
    }


    LOG(INFO) << "cuda_clear_reconstruct_cache:";
    cuda_clear_reconstruct_cache();
    initCache();

    LOG(INFO) << "Stream On:";
    if (!camera_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return DF_ERROR_CAMERA_STREAM;
    }

    lc3010_.start_pattern_sequence();

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for (int i = 0; i < 19; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_->grap(img_ptr))
        {
            
            delete[] img_ptr; 
            camera_->streamOff(); 
            if(i == 0)
            {
                return DF_ERROR_LOST_TRIGGER;
            }

            return DF_ERROR_CAMERA_GRAP;
        }
        LOG(INFO)<<"finished!";

        if(18 == i)
        {
            cuda_copy_brightness_to_memory(img_ptr);
        }

        cuda_copy_pattern_to_memory(img_ptr, i);

        // copy to gpu
        switch (i)
        {
        case 4:
        {
        LOG(INFO)<<"cuda_compute_phase_shift:";
            cuda_compute_phase_shift(0); 
        }
        break;
        case 8:
        {
            cuda_compute_phase_shift(1);
        }
        break;
        case 10:
        {
            cuda_unwrap_phase_shift_base_fisher_confidence(1);
        }
        break;
        case 12:
        {
            cuda_compute_phase_shift(2);
        }
        break;
        case 15:
        {
            cuda_unwrap_phase_shift_base_fisher_confidence(2);
        }
        break;
        case 18:
        {


            cuda_compute_phase_shift(3);
            if (1 == system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify)
            {
                cv::Mat convolution_kernal = cv::getGaussianKernel(system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r, system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma * 0.02, CV_32F);
	            convolution_kernal = convolution_kernal * convolution_kernal.t();
                cuda_copy_convolution_kernal_to_memory((float*)convolution_kernal.data, system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
                cuda_rectify_six_step_pattern_phase(0, system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
            }
            cuda_unwrap_phase_shift_base_fisher_confidence(3);
            fisher_filter(fisher_confidence_val_);
            cuda_normalize_phase(0);

            cuda_generate_pointcloud_base_table();
            LOG(INFO) << "cuda_generate_pointcloud_base_table";
        }

        default:
            break;
        }
    }

    delete[] img_ptr;

    camera_->streamOff();
    LOG(INFO) << "Stream Off";
    
    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_pointcloud_from_memory(buff_pointcloud_);

    if (1 == generate_brightness_model_)
    { 
        cuda_copy_brightness_from_memory(buff_brightness_);
    }
    // }
    // else
    // {
    //     captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    // }

    return DF_SUCCESS;
}

void Scan3D::mergeBrightness()
{
    cuda_merge_brigntness(hdr_num_, buff_brightness_);  
}

int Scan3D::captureFrame04HdrBaseConfidence()
{
    cuda_clear_reconstruct_cache();
    initCache();

    LOG(INFO)<<"Mixed HDR Exposure Base Confidence:";  
    int frame_status = DF_SUCCESS;
 
    for(int i= 0;i< hdr_num_;i++)
    {
        int led_current = led_current_list_[i];
        int ret = lc3010_.SetLedCurrent(led_current, led_current, led_current);
        if (DF_SUCCESS != ret)
        {
            LOG(ERROR) << "set led error: " << ret;
            frame_status = DF_ERROR_LIGHTCRAFTER_SET_CURRENT;
        }

        LOG(INFO)<<"Set LED: "<<led_current;
 
        float exposure = camera_exposure_list_[i];

        if (exposure > max_camera_exposure_)
        {
            exposure = max_camera_exposure_;
        }
        else if (exposure < min_camera_exposure_)
        {
            exposure = min_camera_exposure_;
        }

        LOG(INFO) << "Set Camera Exposure Time: " << exposure;

        if(camera_->setExposure(exposure))
        {
            lc3010_.set_camera_exposure(exposure);
        } 

        ret = captureFrame04BaseConfidence();
        if (DF_SUCCESS != ret)
        {
            LOG(ERROR) << "captureFrame04BaseConfidence code: " << ret;
            frame_status = ret;
        }
        cuda_copy_result_to_hdr(i,18); 
    }
 


    cuda_merge_hdr_data(hdr_num_, buff_depth_, buff_brightness_);  

    
    // if (1 != generate_brightness_model_)
    // { 
    //     captureTextureImage(generate_brightness_model_, generate_brightness_exposure_, buff_brightness_);
    // }
    /******************************************************************************************************/
 
    int ret = lc3010_.SetLedCurrent(led_current_, led_current_, led_current_); 
    if(DF_SUCCESS != ret)
    {
        LOG(ERROR)<<"set led error: "<<ret; 
        frame_status = DF_ERROR_LIGHTCRAFTER_SET_CURRENT;
    }
    
    LOG(INFO) << "Set Led: " << led_current_ << "\n"; 
    LOG(INFO) << "Set Camera Exposure Time: " << camera_exposure_ << "\n"; 
    if (camera_->setExposure(camera_exposure_))
    {
        lc3010_.set_camera_exposure(camera_exposure_);
    }

    return frame_status;
}

bool Scan3D::captureFrame04Hdr()
{

    LOG(INFO)<<"Mixed HDR Exposure:";  
 
 
    for(int i= 0;i< hdr_num_;i++)
    {
        int led_current = led_current_list_[i];
        lc3010_.SetLedCurrent(led_current,led_current,led_current); 
        
        LOG(INFO)<<"Set LED: "<<led_current;
 
        float exposure = camera_exposure_list_[i];

        if (exposure > max_camera_exposure_)
        {
            exposure = max_camera_exposure_;
        }
        else if (exposure < min_camera_exposure_)
        {
            exposure = min_camera_exposure_;
        }

        LOG(INFO) << "Set Camera Exposure Time: " << exposure;

        if(camera_->setExposure(exposure))
        {
            lc3010_.set_camera_exposure(exposure);
        } 

        captureFrame04(); 
        cuda_copy_result_to_hdr(i,18); 
    }
 


    cuda_merge_hdr_data(hdr_num_, buff_depth_, buff_brightness_);  

    
    if (1 != generate_brightness_model_)
    { 
        captureTextureImage(generate_brightness_model_, generate_brightness_exposure_, buff_brightness_);
    }
    /******************************************************************************************************/
 
    lc3010_.SetLedCurrent(led_current_, led_current_, led_current_); 
    LOG(INFO) << "Set Camera Exposure Time: " << camera_exposure_ << "\n"; 
    if (camera_->setExposure(camera_exposure_))
    {
        lc3010_.set_camera_exposure(camera_exposure_);
    }

    return true;
}


bool Scan3D::captureFrame04Repetition01(int repetition_count)
{

    lc3010_.pattern_mode04_repetition(repetition_count);
    LOG(INFO) << "Stream On:";
    if (!camera_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return false;
    }

    lc3010_.start_pattern_sequence();

    unsigned char *img_ptr = new unsigned char[image_width_ * image_height_];

    int capture_count = 19 + 6 * (repetition_count - 1);

    for (int i = 0; i < capture_count; i++)
    {
        LOG(INFO) << "receiving " << i << "th image"; 
        if (!camera_->grap(img_ptr))
        {
            
            delete[] img_ptr; 
            camera_->streamOff();
 
            return false;
        } 
 

        int sync_serial_num = i;

        if (i < 12)
        {
            sync_serial_num = i;
            cuda_copy_pattern_to_memory(img_ptr, i);
        }
        else if (i > 11 && i < 12 + 6 * repetition_count)
        {

            if (12 == i)
            {
                sync_serial_num = 12;
            }
            else
            {
                sync_serial_num = 13;
            }

            cuda_copy_repetition_pattern_to_memory(img_ptr, i - 12);
            cuda_merge_repetition_patterns(i - 12); 
            LOG(INFO) << "repetition " << i - 12 << "th image";
        }
        else
        {

            sync_serial_num = i - 6 * (repetition_count - 1);
            cuda_copy_pattern_to_memory(img_ptr, sync_serial_num);
            if (18 == sync_serial_num)
            {
                cuda_copy_brightness_to_memory(img_ptr);
            }
        }

        // copy to gpu
        switch (sync_serial_num)
        {
        case 4:
        {
            cuda_compute_phase_shift(0);
        }
        break;
        case 8:
        {
            cuda_compute_phase_shift(1);
        }
        break;
        case 10:
        {
            cuda_unwrap_phase_shift(1);
        }
        break;
        case 12:
        {
            cuda_compute_phase_shift(2);
            cuda_unwrap_phase_shift(2);
        }
        break;
        // case 15:
        // {
        // }
        // break;
        case 18:
        {

            cuda_compute_merge_phase(repetition_count);
            cudaDeviceSynchronize();
            // parallel_cuda_compute_phase(3);
            cuda_unwrap_phase_shift(3);

            cuda_normalize_phase(0);
            cuda_generate_pointcloud_base_table();
            //  cudaDeviceSynchronize();
            LOG(INFO) << "generate_pointcloud_base_table";
        }
        break;

            break;

        default:
            break;
        }
    }



    delete[] img_ptr;

    camera_->streamOff();
    LOG(INFO) << "Stream Off";
  
    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_pointcloud_from_memory(buff_pointcloud_);

    if (1 == generate_brightness_model_)
    { 
        cuda_copy_brightness_from_memory(buff_brightness_);
    }
    else
    {

        captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    }


    return true;
}


int Scan3D::captureFrame04Repetition02BaseConfidence(int repetition_count)
{

    int frame_status = DF_SUCCESS;
    cuda_clear_repetition_02_patterns();
    cuda_clear_reconstruct_cache();
    initCache();

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for(int r= 0;r< repetition_count;r++)
    {
        int n = 0;
  
        LOG(INFO) << "pattern_mode04";
        int ret = lc3010_.pattern_mode04();
        if(DF_SUCCESS != ret)
        {
            frame_status = ret;
            return ret;
        }
        if (!camera_->streamOn())
        {
            LOG(INFO) << "Stream On Error";
            frame_status = DF_ERROR_CAMERA_STREAM;
            return DF_ERROR_CAMERA_STREAM;
        }

        lc3010_.start_pattern_sequence();
        LOG(INFO) << "start_pattern_sequence";

        for (int i = 0; i < 19; i++)
        {
            LOG(INFO) << "receiving " << i << "th image";
            bool status = camera_->grap(img_ptr);
            LOG(INFO) << "status=" << status;

            if (status)
            {

                cuda_copy_pattern_to_memory(img_ptr, i);
                cuda_merge_repetition_02_patterns(i);
            }
            else
            {
                LOG(INFO) << "grad failed!";
                camera_->streamOff();
                delete[] img_ptr;

                if (i == 0)
                {
                    return DF_ERROR_LOST_TRIGGER;
                }

                frame_status = DF_ERROR_CAMERA_GRAP;
                return DF_ERROR_CAMERA_GRAP;
            }
        }

        /*********************************************************************************************/
        camera_->streamOff();
        lc3010_.stop_pattern_sequence(); 
        LOG(INFO) << "GXStreamOff";
        /***********************************************************************************************/
    }

    delete[] img_ptr;

    cuda_compute_merge_repetition_02_phase(repetition_count,1);
    LOG(INFO) << "parallel_cuda_compute_mergerepetition_02_phase";
    cuda_unwrap_phase_shift_base_fisher_confidence(1);
    cuda_unwrap_phase_shift_base_fisher_confidence(2);
    if (1 == system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify)
    {
        cv::Mat convolution_kernal = cv::getGaussianKernel(system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r, system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma * 0.02, CV_32F);
        convolution_kernal = convolution_kernal * convolution_kernal.t();
        cuda_copy_convolution_kernal_to_memory((float*)convolution_kernal.data, system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
        cuda_rectify_six_step_pattern_phase(1, system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
    }
    cuda_unwrap_phase_shift_base_fisher_confidence(3);

    fisher_filter(fisher_confidence_val_);
    cuda_normalize_phase(0);

    LOG(INFO) << "parallel_cuda_unwrap_phase";
    cuda_generate_pointcloud_base_table();
    // depth_filter(system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r / 1000.);
    LOG(INFO) << "generate_pointcloud_base_table";


    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_pointcloud_from_memory(buff_pointcloud_);

    // if (1 == generate_brightness_model_)
    // { 
        cuda_copy_brightness_from_memory(buff_brightness_);
    // }
    // else
    // {

    //     captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    // }

    return frame_status;
}

bool Scan3D::captureFrame04Repetition02(int repetition_count)
{
 
    cuda_clear_repetition_02_patterns();

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for(int r= 0;r< repetition_count;r++)
    {
        int n = 0;
  
        LOG(INFO) << "pattern_mode04";
        lc3010_.pattern_mode04();

        if (!camera_->streamOn())
        {
            LOG(INFO) << "Stream On Error";
            return false;
        }

        lc3010_.start_pattern_sequence();
        LOG(INFO) << "start_pattern_sequence";

        for (int i = 0; i < 19; i++)
        {
            LOG(INFO) << "receiving " << i << "th image";
            bool status = camera_->grap(img_ptr);
            LOG(INFO) << "status=" << status;

            if (status)
            {

                cuda_copy_pattern_to_memory(img_ptr, i);
                cuda_merge_repetition_02_patterns(i);
            }
            else
            {
                LOG(INFO) << "grad failed!";
                camera_->streamOff();
                delete[] img_ptr;
                if (i == 0)
                {
                        return DF_ERROR_LOST_TRIGGER;
                }
                return false;
            }
        }

        /*********************************************************************************************/
        camera_->streamOff();
        lc3010_.stop_pattern_sequence(); 
        LOG(INFO) << "GXStreamOff";
        /***********************************************************************************************/
    }

    delete[] img_ptr;

    cuda_compute_merge_repetition_02_phase(repetition_count,1);
    LOG(INFO) << "parallel_cuda_compute_mergerepetition_02_phase";
    cuda_unwrap_phase_shift(1);
    cuda_unwrap_phase_shift(2);
    cuda_unwrap_phase_shift(3);

    cuda_normalize_phase(0);
    LOG(INFO) << "parallel_cuda_unwrap_phase";
    cuda_generate_pointcloud_base_table();
    LOG(INFO) << "generate_pointcloud_base_table";


    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_pointcloud_from_memory(buff_pointcloud_);

    if (1 == generate_brightness_model_)
    { 
        cuda_copy_brightness_from_memory(buff_brightness_);
    }
    else
    {

        captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    }

    return true;
}


bool Scan3D::captureFrame05()
{

    lc3010_.pattern_mode04();
    LOG(INFO) << "Stream On:";
    if (!camera_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return false;
    }

    lc3010_.start_pattern_sequence();

    unsigned char *img_ptr= new unsigned char[image_width_*image_height_];

    for (int i = 0; i < 19; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_->grap(img_ptr))
        {
            
            delete[] img_ptr; 
            camera_->streamOff();
            if (i == 0)
            {
                return DF_ERROR_LOST_TRIGGER;
            }
            return false;
        }
        LOG(INFO)<<"finished!";

        if(18 == i)
        {
            cuda_copy_brightness_to_memory(img_ptr);
        }

        cuda_copy_pattern_to_memory(img_ptr, i);

        // copy to gpu
        switch (i)
        {
        case 4:
        {
        LOG(INFO)<<"cuda_compute_phase_shift:";
            cuda_compute_phase_shift(0); 
        }
        break;
        case 8:
        {
            cuda_compute_phase_shift(1);
        }
        break;
        case 10:
        {
            cuda_unwrap_phase_shift(1);
        }
        break;
        case 12:
        {
            cuda_compute_phase_shift(2);
        }
        break;
        case 15:
        {
            cuda_unwrap_phase_shift(2);
        }
        break;
        case 18:
        {


            cuda_compute_phase_shift(3);
            cuda_unwrap_phase_shift(3);
            cuda_normalize_phase(0);

            cuda_generate_pointcloud_base_minitable();
            //  cudaDeviceSynchronize();
            LOG(INFO) << "cuda_generate_pointcloud_base_table";
        }

        default:
            break;
        }
    }

    delete[] img_ptr;

    camera_->streamOff();
    LOG(INFO) << "Stream Off";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter)
    {
        float r = system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r;
        int num = system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num;
        cuda_remove_points_base_radius_filter(0.5,r,num);
    }

    
    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_pointcloud_from_memory(buff_pointcloud_);

    if (1 == generate_brightness_model_)
    { 
        cuda_copy_brightness_from_memory(buff_brightness_);
    }
    else
    {

        captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    }

 
    return true;
}


bool Scan3D::captureFrame03()
{

    // lc3010_.pattern_mode03();
    // LOG(INFO) << "Stream On:";
    // if (!camera_->streamOn())
    // {
    //     LOG(INFO) << "Stream On Error";
    //     return false;
    // }

    // lc3010_.start_pattern_sequence();

    // unsigned char *img_ptr = new unsigned char[image_width_ * image_height_];

    // for (int i = 0; i < 31; i++)
    // {

    //     LOG(INFO) << "grap " << i << " image:";
    //     if (!camera_->grap(img_ptr))
    //     {

    //         delete[] img_ptr;
    //         camera_->streamOff();
    //         return false;
    //     }
    //     LOG(INFO) << "finished!";

    //     parallel_cuda_copy_signal_patterns(img_ptr, i);

    //     // copy to gpu
    //     switch (i)
    //     {
    //     case 4:
    //     {
    //         parallel_cuda_compute_phase(0);
    //     }
    //     break;
    //     case 8:
    //     {
    //         parallel_cuda_compute_phase(1);
    //         parallel_cuda_unwrap_phase(1);
    //     }
    //     break;
    //     case 12:
    //     {
    //         parallel_cuda_compute_phase(2);
    //         parallel_cuda_unwrap_phase(2);
    //     }
    //     break;
    //     case 18:
    //     {
    //         parallel_cuda_compute_phase(3);
    //         parallel_cuda_unwrap_phase(3);
    //     }
    //     break;
    //     case 21:
    //     {
    //         parallel_cuda_compute_phase(4);
    //     }
    //     break;
    //     case 25:
    //     {
    //         parallel_cuda_compute_phase(5);
    //         parallel_cuda_unwrap_phase(5);
    //     }
    //     break;
    //     case 30:
    //     {
    //         parallel_cuda_compute_phase(6);
    //         parallel_cuda_unwrap_phase(6);

    //         // cudaDeviceSynchronize();
    //         parallel_cuda_reconstruct();
    //     }
    //     break;

    //     default:
    //         break;
    //     }
    // }

    // delete[] img_ptr;

    // camera_->streamOff();
    // LOG(INFO) << "Stream Off";

    // parallel_cuda_copy_result_from_gpu(buff_depth_, buff_brightness_);
    // // reconstruct_copy_pointcloud_from_cuda_memory(buff_pointcloud_);

    // if (1 != generate_brightness_model_)
    // {
    //     captureTextureImage(generate_brightness_model_, generate_brightness_exposure_, buff_brightness_);
    // }

    return true;
}


bool Scan3D::captureFrame01()
{ 
    // int buffer_size = image_width_*image_height_*24;
    // unsigned char* buffer = new unsigned char[buffer_size];

    // if (!captureRaw01(buffer))
    // { 
    //     LOG(INFO) << "capture Raw 01 Failed!"; 
    //     delete[] buffer;
    // }
 
    // std::vector<unsigned char*> patterns_ptr_list;
    // for(int i=0; i<24; i++)
    // {
	//     patterns_ptr_list.push_back(((unsigned char*)(buffer+i*image_width_*image_height_)));
    // }
  
    // cuda_get_frame_base_24(patterns_ptr_list, buff_depth_,buff_brightness_);


    // delete[] buffer;
    return true;
}


bool Scan3D::testCaptureFrame01(unsigned char* buffer)
{
    // std::vector<unsigned char*> patterns_ptr_list;
    // for(int i=0; i<24; i++)
    // {
	//     patterns_ptr_list.push_back(((unsigned char*)(buffer+i*image_width_*image_height_)));
    // }
  
    // cuda_get_frame_base_24(patterns_ptr_list, buff_depth_,buff_brightness_);
}
/***********************************************************************************************************************/

bool Scan3D::readCalibParam()
{
    std::ifstream ifile; 
    ifile.open("calib_param.txt");

    if(!ifile.is_open())
    {
        return false;
    }

    int n_params = sizeof(calib_param_)/sizeof(float);
    for(int i=0; i<n_params; i++)
    {
	    ifile>>(((float*)(&calib_param_))[i]); 
    }
    ifile.close();
    return true;
}


bool Scan3D::loadCalibData()
{

    if(!readCalibParam())
    {
        LOG(INFO)<<"Read Calib Param Error!";  
        return false; 
    }
    else
    {
        
        LOG(INFO)<<"cuda_copy_calib_data:";  
        cuda_copy_calib_data(calib_param_.camera_intrinsic, 
		         calib_param_.projector_intrinsic, 
			 calib_param_.camera_distortion,
	                 calib_param_.projector_distortion, 
			 calib_param_.rotation_matrix, 
			 calib_param_.translation_matrix); 
    }

   
	LookupTableFunction lookup_table_machine_; 
    MiniLookupTableFunction minilookup_table_machine_;

	lookup_table_machine_.setCalibData(calib_param_);
    minilookup_table_machine_.setCalibData(calib_param_);

    LOG(INFO)<<"start read table:";
    
    cv::Mat xL_rotate_x;
    cv::Mat xL_rotate_y;
    cv::Mat rectify_R1;
    cv::Mat pattern_mapping;
    cv::Mat pattern_minimapping(128,128,CV_32F,cv::Scalar(0));

    bool read_map_ok = lookup_table_machine_.readTableFloat("./", xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping,pattern_minimapping,image_width_,image_height_);
  
 
    if(read_map_ok)
    {  
        LOG(INFO)<<"read table finished!";
	    cv::Mat R1_t = rectify_R1.t();
        xL_rotate_x.convertTo(xL_rotate_x, CV_32F);
        xL_rotate_y.convertTo(xL_rotate_y, CV_32F);
        R1_t.convertTo(R1_t, CV_32F);
        pattern_mapping.convertTo(pattern_mapping, CV_32F);
        pattern_minimapping.convertTo(pattern_minimapping, CV_32F); 

        float b = sqrt(pow(calib_param_.translation_matrix[0], 2) + pow(calib_param_.translation_matrix[1], 2) + pow(calib_param_.translation_matrix[2], 2));

        LOG(INFO)<<"start copy table:";
        cuda_copy_talbe_to_memory((float*)pattern_mapping.data,(float*)pattern_minimapping.data, (float*)xL_rotate_x.data,(float*)xL_rotate_y.data,(float*)R1_t.data,b);
        LOG(INFO)<<"copy finished!";
    }

    return true;
}

void Scan3D::copyBrightnessData(unsigned char* &ptr)
{ 
	memcpy(ptr, buff_brightness_, sizeof(unsigned char)*image_height_*image_width_); 
}

void Scan3D::copyDepthData(float* &ptr)
{ 
	memcpy(ptr, buff_depth_, sizeof(float)*image_height_*image_width_);
} 

void Scan3D::copyPointcloudData(float* &ptr)
{ 
    // reconstruct_copy_pointcloud_from_cuda_memory(ptr);
}

void Scan3D::getCameraResolution(int &width, int &height)
{
    width = image_width_;
    height = image_height_;
}

void Scan3D::removeOutlierBaseRadiusFilter()
{
    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter)
    {
        float r = system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r;
        int num = system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num;
        LOG(INFO)<<"radius_filter_r: "<<r;
        LOG(INFO)<<"num: "<<num; 

        cuda_remove_points_base_radius_filter(0.5,r,num);

        cuda_copy_depth_from_memory(buff_depth_);
    }
}

void Scan3D::removeOutlierBaseDepthFilter()
{
    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter)
    {
        float depth_threshold = system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold * (-0.045) + 5.;
        LOG(INFO)<<"depth_filter_threshold: "<<depth_threshold;

        depth_filter(depth_threshold / 1000.);

        cuda_copy_depth_from_memory(buff_depth_);
    }
}
