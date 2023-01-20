#pragma once
#ifndef __XEMA_XCAMERA_H__
#define __XEMA_XCAMERA_H__
#include<vector>
#include<string>

extern "C" {
    namespace XEMA {

        using namespace std;
#ifdef _WIN32 
#define XEMA_API __declspec(dllexport)

#elif __linux
#define XEMA_API 
#endif
		 
#define MAX_CAM_SIZE 10

        //typedef enum CameraStateEnum
        //{
        //    CS_Ready = 0,       //创建实例后转到此状态
        //    CS_Connecting,      //调用connect后未连接成功时转到此状态
        //    CS_Connected,       //连接成功后,未获取过帧数据时转到此状态
        //    CS_Disconnected,    //主动调用disconnect且成功时转到此状态
        //    CS_Working,         //连接成功且已获取过帧数据时转到此状态
        //    CS_ProcessError,    //调用相机sdk失败且需要主动尝试恢复工作时转到此状态(一般在处理数据出现错误时)
        //    CS_Error            //调用相机sdk失败且不需要主动尝试恢复工作时转到此状态
        //}CameraStateEnum;

        //typedef enum CameraErrorEnum
        //{
        //    CE_NONE = 0,        //无错误
        //    CE_NOTSUPPORTED,    //不支持
        //    CE_NOTINWORKING,    //未在连接状态
        //    CE_SDKFAILED,       //调用sdk失败
        //    CE_JSONFORMAT,      //json格式错误
        //    CE_CAMNOTFOUND      //相机未找到
        //}CameraErrorEnum;

        //typedef struct CameraBaseInfo
        //{
        //    char id[50];
        //    int port;
        //    bool isIpType;
        //    bool is2D;
        //}CameraBaseInfo;
		 
            //相机标定参数结构体
        typedef struct CalibrationParam
        {
            //相机内参
            float intrinsic[3 * 3];
            //相机外参
            float extrinsic[4 * 4];
            //相机畸变，只用前5个
            float distortion[1 * 12];//<k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4>

        }CalibrationParam;

        class XCamera
        {
        public:
            XCamera() = default;
            virtual ~XCamera() = default;
            XCamera(const XCamera&) = delete;
            XCamera& operator=(const XCamera&) = delete;
			 
			//功能： 连接相机
			//输入参数： camera_id（相机ip地址）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
			virtual int connect(const char* camera_id) = 0;
			 
			//功能： 获取相机分辨率
			//输入参数： 无
			//输出参数： width(图像宽)、height(图像高)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			virtual  int  getCameraResolution(int* width, int* height) = 0;
			 
			//功能： 采集一帧数据并阻塞至返回状态
			//输入参数： exposure_num（曝光次数）：设置值为1为单曝光，大于1为多曝光模式（具体参数在相机gui中设置）.
			//输出参数： timestamp(时间戳)
			//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
			virtual  int captureData(int exposure_num, char* timestamp) = 0;
			 
			//功能： 获取深度图
			//输入参数：无
			//输出参数： depth(深度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual  int getDepthData(float* depth) = 0;
			
			//功能： 获取点云
			//输入参数：无
			//输出参数： point_cloud(点云)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual  int getPointcloudData(float* point_cloud) = 0;
			 
			//功能： 获取亮度图
			//输入参数：无
			//输出参数： brightness(亮度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual  int getBrightnessData(unsigned char* brightness)  = 0;
			 
			//功能： 获取校正到基准平面的高度映射图
			//输入参数：无  
			//输出参数： height_map(高度映射图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual  int getHeightMapData(float* height_map)  = 0;
			 
			//功能： 获取基准平面参数
			//输入参数：无
			//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual  int getStandardPlaneParam(float* R, float* T)  = 0;
			 
			//功能： 获取校正到基准平面的高度映射图
			//输入参数：R(旋转矩阵)、T(平移矩阵)
			//输出参数： height_map(高度映射图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual  int getHeightMapDataBaseParam(float* R, float* T, float* height_map)  = 0;
			  
			//功能： 断开相机连接
			//输入参数： camera_id（相机ip地址）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示断开成功;返回-1表示断开失败.
			virtual  int disconnect(const char* camera_id) = 0;
			 
			//功能： 获取相机标定参数
			//输入参数： 无
			//输出参数： calibration_param（相机标定参数结构体）
			//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
			virtual  int getCalibrationParam(struct CalibrationParam* calibration_param) = 0;


			/***************************************************************************************************************************************************************/
			//参数设置
			 
			//功能： 设置LED电流
			//输入参数： led（电流值）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamLedCurrent(int led) = 0;
			 
			//功能： 设置LED电流
			//输入参数： 无
			//输出参数： led（电流值）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamLedCurrent(int& led) = 0;
			  
			 
			//功能： 设置基准平面的外参
			//输入参数：R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamStandardPlaneExternal(float* R, float* T) = 0;
			 
			//功能： 获取基准平面的外参
			//输入参数：无
			//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamStandardPlaneExternal(float* R, float* T) = 0;
			 
			//功能： 设置生成亮度图参数
			//输入参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamGenerateBrightness(int model, float exposure) = 0;
			 
			//功能： 获取生成亮度图参数
			//输入参数： 无
			//输出参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamGenerateBrightness(int& model, float& exposure) = 0;
			 
			//功能： 设置相机曝光时间
			//输入参数：exposure(相机曝光时间)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamCameraExposure(float exposure) = 0;
			 
			//功能： 获取相机曝光时间
			//输入参数： 无
			//输出参数：exposure(相机曝光时间)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamCameraExposure(float& exposure) = 0;
			 
			//功能： 设置混合多曝光参数（最大曝光次数为6次）
			//输入参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）、led_param[6]（6个led亮度参数、前num个有效）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamMixedHdr(int num, int exposure_param[6], int led_param[6]) = 0;
			 
			//功能： 获取混合多曝光参数（最大曝光次数为6次）
			//输入参数： 无
			//输出参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）、led_param[6]（6个led亮度参数、前num个有效）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamMixedHdr(int& num, int exposure_param[6], int led_param[6]) = 0;
			 
			//功能： 设置相机曝光时间
			//输入参数：confidence(相机置信度)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamCameraConfidence(float confidence) = 0;
			 
			//功能： 获取相机曝光时间
			//输入参数： 无
			//输出参数：confidence(相机置信度)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamCameraConfidence(float& confidence) = 0;
			 
			//功能： 设置相机增益
			//输入参数：gain(相机增益)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamCameraGain(float gain) = 0;
			 
			//功能： 获取相机增益
			//输入参数： 无
			//输出参数：gain(相机增益)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamCameraGain(float& gain) = 0;
			 
			//功能： 设置点云平滑参数
			//输入参数：smoothing(0:关、1-5:平滑程度由低到高)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamSmoothing(int smoothing) = 0;
			 
			//功能： 设置点云平滑参数
			//输入参数：无
			//输出参数：smoothing(0:关、1-5:平滑程度由低到高)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamSmoothing(int& smoothing) = 0;
			 
			//功能： 设置点云半径滤波参数
			//输入参数：use(开关：1开、0关)、radius(半径）、num（有效点）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamRadiusFilter(int use, float radius, int num) = 0;
			 
			//功能： 获取点云半径滤波参数
			//输入参数：无
			//输出参数：use(开关：1开、0关)、radius(半径）、num（有效点）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamRadiusFilter(int& use, float& radius, int& num) = 0;
			 
			//功能： 设置外点过滤阈值
			//输入参数：threshold(阈值0-100)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamOutlierFilter(float threshold) = 0;
			 
			//功能： 获取外点过滤阈值
			//输入参数： 无
			//输出参数：threshold(阈值0-100)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual  int getParamOutlierFilter(float& threshold) = 0;
			 
			//功能： 设置多曝光模式
			//输入参数： model(1：HDR(默认值)、2：重复曝光)
			//输出参数：无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamMultipleExposureModel(int model) = 0;
			 
			//功能： 设置重复曝光数
			//输入参数： num(2-10)
			//输出参数：无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual  int setParamRepetitionExposureNum(int num) = 0;
        };

        XEMA_API void* createCamera();
        XEMA_API void destroyCamera(void*);

    }
}
#endif