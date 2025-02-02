#include "open_cam3d.h"
#include "socket_tcp.h"
#include <assert.h>
#include <iostream>
#include <thread>
#include "utils.h"
#include "../firmware/easylogging++.h"
#include<chrono>
#include<ctime>
#include <time.h>
#include <stddef.h> 
#include "../test/triangulation.h"
#include "../firmware/protocol.h" 
#include "../firmware/system_config_settings.h"
#include <configuring_network.h>
#include "xema_enums.h"

#ifdef _WIN32 
 
#elif __linux 
#include <dirent.h> 
#include <unistd.h>
#endif 

using namespace std;
using namespace std::chrono;

/**********************************************************************************************************************/
//socket
INITIALIZE_EASYLOGGINGPP

XemaEngine engine_ = XemaEngine::Normal;

XemaPixelType pixel_type_ = XemaPixelType::Mono;

//const int image_width = 1920;
//const int image_height = 1200;
//const int image_size = image_width * image_height;
bool connected = false;
long long token = 0;
//const char* camera_id_;
std::string camera_id_;
std::thread heartbeat_thread;
int heartbeat_error_count_ = 0;

extern SOCKET g_sock_heartbeat;
extern SOCKET g_sock;

int (*p_OnDropped)(void*) = 0;

//int camera_version = 0;

int multiple_exposure_model_ = 1;
int repetition_exposure_model_ = 2;


std::timed_mutex command_mutex_;
std::timed_mutex undistort_mutex_;
/**************************************************************************************************************/


struct CameraCalibParam calibration_param_;
bool connected_flag_ = false;

int camera_width_ = 1920;
int camera_height_ = 1200;
int image_size_ = camera_width_ * camera_height_;

const char* camera_ip_ = "";


int depth_buf_size_ = 0;
int pointcloud_buf_size_ = 0;
int brightness_bug_size_ = 0;
float* point_cloud_buf_ = NULL;
float* trans_point_cloud_buf_ = NULL;
bool transform_pointcloud_flag_ = false;
float* depth_buf_ = NULL;
unsigned char* brightness_buf_ = NULL;
float* undistort_map_x_ = NULL;
float* undistort_map_y_ = NULL;
float* distorted_map_x_ = NULL;
float* distorted_map_y_ = NULL;


unsigned char* rgb_buf_ = NULL;
bool bayer_to_rgb_flag_ = false;
/**************************************************************************************************************************/

std::time_t getTimeStamp(long long& msec)
{
	std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
	seconds sec = duration_cast<seconds>(tp.time_since_epoch());


	std::time_t timestamp = tmp.count();

	msec = tmp.count() - sec.count() * 1000;
	//std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
	return timestamp;
}

std::tm* gettm(long long timestamp)
{
	auto milli = timestamp + (long long)8 * 60 * 60 * 1000; //此处转化为东八区北京时间，如果是其它时区需要按需求修改
	auto mTime = std::chrono::milliseconds(milli);
	auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
	auto tt = std::chrono::system_clock::to_time_t(tp);
	std::tm* now = std::gmtime(&tt);
	//printf("%4d年%02d月%02d日 %02d:%02d:%02d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
	return now;
}


std::string get_timestamp()
{

	long long msec = 0;
	char time_str[7][16];
	auto t = getTimeStamp(msec);
	//std::cout << "Millisecond timestamp is: " << t << std::endl;
	auto time_ptr = gettm(t);
	sprintf(time_str[0], "%02d", time_ptr->tm_year + 1900); //月份要加1
	sprintf(time_str[1], "%02d", time_ptr->tm_mon + 1); //月份要加1
	sprintf(time_str[2], "%02d", time_ptr->tm_mday);//天
	sprintf(time_str[3], "%02d", time_ptr->tm_hour);//时
	sprintf(time_str[4], "%02d", time_ptr->tm_min);// 分
	sprintf(time_str[5], "%02d", time_ptr->tm_sec);//时
	sprintf(time_str[6], "%02lld", msec);// 分
	//for (int i = 0; i < 7; i++)
	//{
	//	std::cout << "time_str[" << i << "] is: " << time_str[i] << std::endl;
	//}

	std::string timestamp = "";

	timestamp += time_str[0];
	timestamp += "-";
	timestamp += time_str[1];
	timestamp += "-";
	timestamp += time_str[2];
	timestamp += " ";
	timestamp += time_str[3];
	timestamp += ":";
	timestamp += time_str[4];
	timestamp += ":";
	timestamp += time_str[5];
	timestamp += ",";
	timestamp += time_str[6];

	//std::cout << timestamp << std::endl;

	return timestamp;
}


/***************************************************************************************************************************/
//网格掉线
int on_dropped(void* param)
{
	LOG(INFO) << "Network dropped!" << std::endl;
	return 0;
}


bool transformPointcloudInv(float* point_cloud_map, float* rotate, float* translation)
{

	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{

			int offset = r * camera_width_ + c;

			float x = point_cloud_map[3 * offset + 0] + translation[0];
			float y = point_cloud_map[3 * offset + 1] + translation[1];
			float z = point_cloud_map[3 * offset + 2] + translation[2];

			point_cloud_map[3 * offset + 0] = rotate[0] * x + rotate[1] * y + rotate[2] * z;
			point_cloud_map[3 * offset + 1] = rotate[3] * x + rotate[4] * y + rotate[5] * z;
			point_cloud_map[3 * offset + 2] = rotate[6] * x + rotate[7] * y + rotate[8] * z;

		}

	}


	return true;
}

bool transformPointcloud(float* org_point_cloud_map, float* transform_point_cloud_map, float* rotate, float* translation)
{


	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{

			int offset = r * camera_width_ + c;

			float x = org_point_cloud_map[3 * offset + 0];
			float y = org_point_cloud_map[3 * offset + 1];
			float z = org_point_cloud_map[3 * offset + 2];

			//if (z > 0)
			//{
			transform_point_cloud_map[3 * offset + 0] = rotate[0] * x + rotate[1] * y + rotate[2] * z + translation[0];
			transform_point_cloud_map[3 * offset + 1] = rotate[3] * x + rotate[4] * y + rotate[5] * z + translation[1];
			transform_point_cloud_map[3 * offset + 2] = rotate[6] * x + rotate[7] * y + rotate[8] * z + translation[2];

			//}
			//else
			//{
			   // point_cloud_map[3 * offset + 0] = 0;
			   // point_cloud_map[3 * offset + 1] = 0;
			   // point_cloud_map[3 * offset + 2] = 0;
			//}


		}

	}


	return true;
}

//函数名： undistortDepthTransformPointcloud
//功能： 深度图转点云接口
//输入参数：undistort_depth_map（深度图）
//输出参数：point_cloud_map（点云）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败. 
int  undistortDepthTransformPointcloud(float* undistort_depth_map, float* undistort_point_cloud_map)
{

	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}

	float camera_fx = calibration_param_.camera_intrinsic[0];
	float camera_fy = calibration_param_.camera_intrinsic[4];

	float camera_cx = calibration_param_.camera_intrinsic[2];
	float camera_cy = calibration_param_.camera_intrinsic[5];

  
	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{



			int offset = r * camera_width_ + c;
			if (undistort_depth_map[offset] > 0)
			{
				double undistort_x = c;
				double undistort_y = r;
 

				undistort_point_cloud_map[3 * offset + 0] = (undistort_x - camera_cx) * undistort_depth_map[offset] / camera_fx;
				undistort_point_cloud_map[3 * offset + 1] = (undistort_y - camera_cy) * undistort_depth_map[offset] / camera_fy;
				undistort_point_cloud_map[3 * offset + 2] = undistort_depth_map[offset];


			}
			else
			{
				undistort_point_cloud_map[3 * offset + 0] = 0;
				undistort_point_cloud_map[3 * offset + 1] = 0;
				undistort_point_cloud_map[3 * offset + 2] = 0;
			}


		}

	}


	return DF_SUCCESS;
}


DF_SDK_API int DfRgbToGray(unsigned char* src, unsigned char* dst)
{
	int height = camera_height_;
	int width = camera_width_;

	for (int r = 0; r < height; r++)
	{

		for (int c = 0; c < width; c++)
		{
			//0.299 R  + 0.587 G + B
			dst[r * width + c] = src[3 * (r * width + c) + 0] * 0.299 + 
				0.587 * src[3 * (r * width + c) + 1] + 0.114 * src[3 * (r * width + c) + 2];
		}

	}


	return 0;
}
 
DF_SDK_API int DfBayerToRgb(unsigned char* src, unsigned char* dst)
{
  
	int height = camera_height_;
	int width = camera_width_;

	//按3×3的邻域处理，边缘需填充至少1个像素的宽度
	int nBorder = 1;
	unsigned char* bayer = (unsigned char*)malloc(sizeof(unsigned char) * (width + 2 * nBorder) * (height + 2 * nBorder));
	memset(bayer, 0, sizeof(unsigned char) * (width + 2 * nBorder) * (height + 2 * nBorder));

	for (int r = 0; r < height; r++)
	{
		for (int c = 0; c < width; c++)
		{
			bayer[(r + nBorder) * (width + 2 * nBorder) + (c + nBorder)] = src[r * width + c];
		}
	}

	for (int b = 0; b < nBorder; b++)
	{
		for (int r = 0; r < height; r++)
		{
			bayer[(r + nBorder) * (width + 2 * nBorder) + b] = src[r * width];
			bayer[(r + nBorder) * (width + 2 * nBorder) + b + 2 * nBorder + width - 1] = src[r * width + width - 1];
		}
	}

	for (int b = 0; b < nBorder; b++)
	{
		for (int c = 0; c < width + 2 * nBorder; c++)
		{
			bayer[b * (width + 2 * nBorder) + c] = bayer[nBorder * (width + 2 * nBorder) + c];
			bayer[(nBorder + height + b) * (width + 2 * nBorder) + c] = bayer[(nBorder + height - 1) * (width + 2 * nBorder) + c];
		}
	}
	 
	unsigned char* p_rgb = (unsigned char*)malloc(sizeof(unsigned char) * 3 * (width + 2 * nBorder) * (height + 2 * nBorder));
	memset(p_rgb, 0, sizeof(unsigned char) * 3 * (width + 2 * nBorder) * (height + 2 * nBorder));


	unsigned char* pBayer = bayer;
	unsigned char* pRGB = p_rgb;
	int nW = width + 2 * nBorder;
	int nH = height + 2 * nBorder;


	for (int i = nBorder; i < nH - nBorder; i++)
	{
		for (int j = nBorder; j < nW - nBorder; j++)
		{
			//3×3邻域像素定义
			/*
			 * |M00 M01 M02|
			 * |M10 M11 M12|
			 * |M20 M21 M22|
			*/
			int nM00 = (i - 1) * nW + (j - 1); int nM01 = (i - 1) * nW + (j + 0);  int nM02 = (i - 1) * nW + (j + 1);
			int nM10 = (i - 0) * nW + (j - 1); int nM11 = (i - 0) * nW + (j + 0);  int nM12 = (i - 0) * nW + (j + 1);
			int nM20 = (i + 1) * nW + (j - 1); int nM21 = (i + 1) * nW + (j + 0);  int nM22 = (i + 1) * nW + (j + 1);

			if (i % 2 == 0)
			{
				if (j % 2 == 0)     //偶数行偶数列
				{
					pRGB[i * nW * 3 + j * 3 + 0] = pBayer[nM11];//b
					pRGB[i * nW * 3 + j * 3 + 2] = ((pBayer[nM00] + pBayer[nM02] + pBayer[nM20] + pBayer[nM22]) >> 2);//r
					pRGB[i * nW * 3 + j * 3 + 1] = ((pBayer[nM01] + pBayer[nM10] + pBayer[nM12] + pBayer[nM21]) >> 2);//g
				}
				else             //偶数行奇数列
				{
					pRGB[i * nW * 3 + j * 3 + 1] = pBayer[nM11];//g
					pRGB[i * nW * 3 + j * 3 + 2] = (pBayer[nM01] + pBayer[nM21]) >> 1;//r
					pRGB[i * nW * 3 + j * 3 + 0] = (pBayer[nM10] + pBayer[nM12]) >> 1;//b
				}
			}
			else
			{
				if (j % 2 == 0)     //奇数行偶数列
				{
					pRGB[i * nW * 3 + j * 3 + 1] = pBayer[nM11];//g
					pRGB[i * nW * 3 + j * 3 + 2] = (pBayer[nM10] + pBayer[nM12]) >> 1;//r
					pRGB[i * nW * 3 + j * 3 + 0] = (pBayer[nM01] + pBayer[nM21]) >> 1;//b
				}
				else             //奇数行奇数列
				{
					pRGB[i * nW * 3 + j * 3 + 2] = pBayer[nM11];//r
					pRGB[i * nW * 3 + j * 3 + 1] = (pBayer[nM01] + pBayer[nM21] + pBayer[nM10] + pBayer[nM12]) >> 2;//g
					pRGB[i * nW * 3 + j * 3 + 0] = (pBayer[nM00] + pBayer[nM02] + pBayer[nM20] + pBayer[nM22]) >> 2;//b
				}
			}
		}
	}


	for (int r = 0; r < height; r++)
	{
		for (int c = 0; c < width; c++)
		{
			dst[3 * (r * width + c) + 0] = pRGB[3 * ((r + nBorder) * (width + 2 * nBorder) + c + nBorder) + 0];
			dst[3 * (r * width + c) + 1] = pRGB[3 * ((r + nBorder) * (width + 2 * nBorder) + c + nBorder) + 1];
			dst[3 * (r * width + c) + 2] = pRGB[3 * ((r + nBorder) * (width + 2 * nBorder) + c + nBorder) + 2];
		}
	}


	free(bayer);
	free(p_rgb);

	return 0;
}

int depthTransformPointcloud(float* depth_map, float* point_cloud_map)
{

	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}

	float camera_fx = calibration_param_.camera_intrinsic[0];
	float camera_fy = calibration_param_.camera_intrinsic[4];

	float camera_cx = calibration_param_.camera_intrinsic[2];
	float camera_cy = calibration_param_.camera_intrinsic[5];


	float k1 = calibration_param_.camera_distortion[0];
	float k2 = calibration_param_.camera_distortion[1];
	float p1 = calibration_param_.camera_distortion[2];
	float p2 = calibration_param_.camera_distortion[3];
	float k3 = calibration_param_.camera_distortion[4];


	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{



			int offset = r * camera_width_ + c;
			if (depth_map[offset] > 0)
			{
				//double undistort_x = c;
				//double undistort_y = r;
				//undistortPoint(c, r, camera_fx, camera_fy,
				//	camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

				float undistort_x = undistort_map_x_[offset];
				float undistort_y = undistort_map_y_[offset];

				point_cloud_map[3 * offset + 0] = (undistort_x - camera_cx) * depth_map[offset] / camera_fx;
				point_cloud_map[3 * offset + 1] = (undistort_y - camera_cy) * depth_map[offset] / camera_fy;
				point_cloud_map[3 * offset + 2] = depth_map[offset];


			}
			else
			{
				point_cloud_map[3 * offset + 0] = 0;
				point_cloud_map[3 * offset + 1] = 0;
				point_cloud_map[3 * offset + 2] = 0;
			}


		}

	}


	return DF_SUCCESS;
}
//
//void distortPoint(/*double fx, double fy, double u0, double v0, double k1, double k2, double p1, double p2, double k3, */float inputU, float inputV, float& outputU, float& outputV)
//{
//	float fx = calibration_param_.camera_intrinsic[0];
//	float fy = calibration_param_.camera_intrinsic[4];
//
//	float u0 = calibration_param_.camera_intrinsic[2];
//	float v0 = calibration_param_.camera_intrinsic[5];
//
//
//	float k1 = calibration_param_.camera_distortion[0];
//	float k2 = calibration_param_.camera_distortion[1];
//	float p1 = calibration_param_.camera_distortion[2];
//	float p2 = calibration_param_.camera_distortion[3];
//	float k3 = calibration_param_.camera_distortion[4];
//	float k4 = 0;
//	float k5 = 0;
//	float k6 = 0;
//
//	float x = (inputU - u0) / fx, y = (inputV - v0) / fy;
//
//	float x2 = x * x, y2 = y * y;
//	float r2 = x2 + y2, _2xy = 2 * x * y;
//	float kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
//	// 归一化坐标转化为图像坐标
//	outputU = fx * (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2)) + u0;
//	outputV = fy * (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy) + v0;
//
//}


int undistortBrightnessMap(unsigned char* brightness_map) //最近邻
{
 
	LOG(INFO) << "undistortBrightnessMap:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}

	int nr = camera_height_;
	int nc = camera_width_;

	unsigned char* brightness_map_temp = new unsigned char[nr * nc];
	memset(brightness_map_temp, 0, sizeof(unsigned char) * nr * nc);

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;
			float distort_x, distort_y;
			//distortPoint(c, r, distort_x, distort_y);
			distort_x = distorted_map_x_[offset];
			distort_y = distorted_map_y_[offset];
			// 进行双线性差值
			if (distort_x > 0 && distort_x < nc - 1 && distort_y > 0 && distort_y < nr - 1)
			{
				float l_t_brightness = brightness_map[(int)distort_y * camera_width_ + (int)distort_x];
				float l_b_brightness = brightness_map[(int)(distort_y + 1) * camera_width_ + (int)distort_x];
				float r_t_brightness = brightness_map[(int)distort_y * camera_width_ + (int)(distort_x + 1)];
				float r_b_brightness = brightness_map[(int)(distort_y + 1) * camera_width_ + (int)(distort_x + 1)];

				float rate_y = distort_y - (int)distort_y;
				float rate_x = distort_x - (int)distort_x;

				brightness_map_temp[offset] = (unsigned char)(l_t_brightness * (1 - rate_x) * (1 - rate_y) + l_b_brightness * (1 - rate_x) * rate_y + r_t_brightness * rate_x * (1 - rate_y) + r_b_brightness * rate_x * rate_y + 0.5);
			}
		}

	}

	memcpy(brightness_map, brightness_map_temp, sizeof(unsigned char) * nr * nc);
	delete[] brightness_map_temp;
	brightness_map_temp = NULL;
	return DF_SUCCESS;
}



int undistortColorBrightnessMap(unsigned char* brightness_map) //最近邻
{

	int nr = camera_height_;
	int nc = camera_width_;

	unsigned char* b_map = new unsigned char[nr * nc];
	memset(b_map, 0, sizeof(unsigned char) * nr * nc);

	unsigned char* g_map = new unsigned char[nr * nc];
	memset(g_map, 0, sizeof(unsigned char) * nr * nc);

	unsigned char* r_map = new unsigned char[nr * nc];
	memset(r_map, 0, sizeof(unsigned char) * nr * nc);

	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			b_map[r * nc + c] = brightness_map[3 * (r * nc + c) + 0];
			g_map[r * nc + c] = brightness_map[3 * (r * nc + c) + 1];
			r_map[r * nc + c] = brightness_map[3 * (r * nc + c) + 2];

		}

	}

	if (DF_SUCCESS != undistortBrightnessMap(b_map))
	{
		delete[] b_map;
		delete[] g_map;
		delete[] r_map;
		return DF_FAILED;
	}

	if (DF_SUCCESS != undistortBrightnessMap(g_map))
	{
		delete[] b_map;
		delete[] g_map;
		delete[] r_map;
		return DF_FAILED;
	}

	if (DF_SUCCESS != undistortBrightnessMap(r_map))
	{
		delete[] b_map;
		delete[] g_map;
		delete[] r_map;
		return DF_FAILED;
	}


	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			brightness_map[3 * (r * nc + c) + 0] = b_map[r * nc + c];
			brightness_map[3 * (r * nc + c) + 1] = g_map[r * nc + c];
			brightness_map[3 * (r * nc + c) + 2] = r_map[r * nc + c];
		}

	}

	delete[] b_map;
	delete[] g_map;
	delete[] r_map;

	return DF_SUCCESS;
}



int undistortDepthMap(float* depth_map)
{
	// 使用双线性差值
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}

	int nr = camera_height_;
	int nc = camera_width_;

	float* depth_map_temp = new float[nr * nc];
	memset(depth_map_temp, -1, sizeof(float) * nr * nc);

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;
			float distort_x, distort_y;
			//distortPoint(c, r, distort_x, distort_y);
			distort_x = distorted_map_x_[offset];
			distort_y = distorted_map_y_[offset];
			// 进行双线性差值
			if (distort_x > 0 && distort_x < nc - 1 && distort_y > 0 && distort_y < nr - 1)
			{
				float l_t_depth = depth_map[(int)distort_y * camera_width_ + (int)distort_x];
				float l_b_depth = depth_map[(int)(distort_y + 1) * camera_width_ + (int)distort_x];
				float r_t_depth = depth_map[(int)distort_y * camera_width_ + (int)(distort_x + 1)];
				float r_b_depth = depth_map[(int)(distort_y + 1) * camera_width_ + (int)(distort_x + 1)];

				if (l_t_depth <= 0 || l_b_depth <= 0 || r_t_depth <= 0 || r_b_depth <= 0)
				{
					depth_map_temp[offset] = depth_map[(int)(distort_y + 0.5) * camera_width_ + (int)(distort_x + 0.5)];
					continue;
				}

				float rate_y = distort_y - (int)distort_y;
				float rate_x = distort_x - (int)distort_x;

				depth_map_temp[offset] = l_t_depth * (1 - rate_x) * (1 - rate_y) + l_b_depth * (1 - rate_x) * rate_y + r_t_depth * rate_x * (1 - rate_y) + r_b_depth * rate_x * rate_y;
			}
		}

	}

	memcpy(depth_map, depth_map_temp, sizeof(float) * nr * nc);
	delete[] depth_map_temp;
	depth_map_temp = NULL;
	return DF_SUCCESS;
}



 
void rolloutHandler(const char* filename, std::size_t size)
{
#ifdef _WIN32 
	/// 备份日志
	system("mkdir xemaLog"); 
	system("DIR .\\xemaLog\\ .log / B > LIST.TXT"); 
	ifstream name_in("LIST.txt",ios_base::in);//文件流
 
	int num = 0;
	std::vector<std::string> name_list;
	char buf[1024] = { 0 };
	while (name_in.getline(buf, sizeof(buf)))
	{ 
		//std::cout << "name: " << buf << std::endl;
		num++;
		name_list.push_back(std::string(buf));
	}

	if (num < 5)
	{
		num++;
	}
	else
	{
		num = 5;
		name_list.pop_back();
	}
 

	for (int i = num; i > 0 && !name_list.empty(); i--)
	{
		std::stringstream ss;
		std::string path = ".\\xemaLog\\" + name_list.back();
		name_list.pop_back();
		ss << "move " << path << " xemaLog\\log_" << i-1 << ".log";
		std::cout << ss.str() << std::endl;
		system(ss.str().c_str());
	}

	std::stringstream ss;
	ss << "move " << filename << " xemaLog\\log_0" <<".log";
	system(ss.str().c_str());
#elif __linux 

	/// 备份日志
	if (access("xemaLog", F_OK) != 0)
	{
		system("mkdir xemaLog");
	}

	std::vector<std::string> name_list;
	std::string suffix = "log";
	DIR* dir;
	struct dirent* ent;
	if ((dir = opendir("xemaLog")) != NULL)
	{
		while ((ent = readdir(dir)) != NULL)
		{
			/* print all the files and directories within directory */
			// printf("%s\n", ent->d_name);

			std::string name = ent->d_name;

			if (name.size() < 3)
			{
				continue;
			}

			std::string curSuffix = name.substr(name.size() - 3);

			if (suffix == curSuffix)
			{
				name_list.push_back(name);
			}
		}
		closedir(dir);
	}

	sort(name_list.begin(), name_list.end());

	int num = name_list.size();
	if (num < 5)
	{
		num++;
	}
	else
	{
		num = 5;
		name_list.pop_back();
	}


	for (int i = num; i > 0 && !name_list.empty(); i--)
	{
		std::stringstream ss;
		std::string path = "./xemaLog/" + name_list.back();
		name_list.pop_back();
		ss << "mv " << path << " xemaLog/log_" << i - 1 << ".log";
		std::cout << ss.str() << std::endl;
		system(ss.str().c_str());
	}

	std::stringstream ss;
	ss << "mv " << filename << " xemaLog/log_0" << ".log";
	system(ss.str().c_str());

#endif 

}
 

//函数名： DfConnect
//功能： 连接相机
//输入参数： camera_id（相机id）
//输出参数： 无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfConnect(const char* camera_id)
{

	/*******************************************************************************************************************/
	//关闭log输出
	el::Configurations conf;
	conf.setToDefault();
	//conf.setGlobally(el::ConfigurationType::Format, "[%datetime{%H:%m:%s} | %level] %msg");

#ifdef _WIN32 
	//conf.setGlobally(el::ConfigurationType::Filename, "log\\log_%datetime{%Y%M%d}.log");
	conf.setGlobally(el::ConfigurationType::Filename, "xema_log.log");
#elif __linux 
	//conf.setGlobally(el::ConfigurationType::Filename, "log/log_%datetime{%Y%M%d}.log");
	conf.setGlobally(el::ConfigurationType::Filename, "xema_log.log");
#endif 
	conf.setGlobally(el::ConfigurationType::Enabled, "true");
	conf.setGlobally(el::ConfigurationType::ToFile, "true");
	//conf.setGlobally(el::ConfigurationType::MaxLogFileSize, "204800");//1024*1024*1024=1073741824 
	el::Loggers::reconfigureAllLoggers(conf); 
	el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");

	/*******************************************************************************************************************/

	LOG(INFO) << "DfConnect: ";
	DfRegisterOnDropped(on_dropped);


	int ret = DfConnectNet(camera_id);
	if (ret != DF_SUCCESS)
	{
		return ret;
	}

	ret = DfGetCalibrationParam(calibration_param_);

	if (ret != DF_SUCCESS)
	{
		DfDisconnectNet();
		return ret;
	}
	 
	int width, height;
	ret = DfGetCameraResolution(&width, &height);
	if (ret != DF_SUCCESS)
	{
		DfDisconnectNet();
		return ret;
	}

	if (width <= 0 || height <= 0)
	{
		DfDisconnectNet();
		return DF_ERROR_2D_CAMERA;
	} 

	int pixel_type = 0;

	ret = DfGetCameraPixelType(pixel_type);
	if (ret == DF_SUCCESS)
	{
		pixel_type_ = XemaPixelType(pixel_type);
	}
	else if (ret == DF_UNKNOWN)
	{
		pixel_type_ = XemaPixelType::Mono;
	}

	camera_width_ = width;
	camera_height_ = height;

	camera_ip_ = camera_id;
	connected_flag_ = true;

	image_size_ = camera_width_ * camera_height_;

	depth_buf_size_ = image_size_ * 1 * 4;
	depth_buf_ = (float*)(new char[depth_buf_size_]);

	pointcloud_buf_size_ = depth_buf_size_ * 3;
	point_cloud_buf_ = (float*)(new char[pointcloud_buf_size_]);

	trans_point_cloud_buf_ = (float*)(new char[pointcloud_buf_size_]);

	brightness_bug_size_ = image_size_;
	brightness_buf_ = new unsigned char[brightness_bug_size_];
	 
	rgb_buf_ = new unsigned char[3*brightness_bug_size_];
	/******************************************************************************************************/
	//产生畸变校正表
	undistort_map_x_ = (float*)(new char[depth_buf_size_]);
	undistort_map_y_ = (float*)(new char[depth_buf_size_]);

	distorted_map_x_ = (float*)(new char[depth_buf_size_]);
	distorted_map_y_ = (float*)(new char[depth_buf_size_]);


	float camera_fx = calibration_param_.camera_intrinsic[0];
	float camera_fy = calibration_param_.camera_intrinsic[4];

	float camera_cx = calibration_param_.camera_intrinsic[2];
	float camera_cy = calibration_param_.camera_intrinsic[5];


	float k1 = calibration_param_.camera_distortion[0];
	float k2 = calibration_param_.camera_distortion[1];
	float p1 = calibration_param_.camera_distortion[2];
	float p2 = calibration_param_.camera_distortion[3];
	float k3 = calibration_param_.camera_distortion[4];


	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{
			double undistort_x = c;
			double undistort_y = r;


			int offset = r * camera_width_ + c;

			undistortPoint(c, r, camera_fx, camera_fy,
				camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

			undistort_map_x_[offset] = (float)undistort_x;
			undistort_map_y_[offset] = (float)undistort_y;
		}

	}


	// 生成畸变矫正的畸变表
#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{
			float distorted_x, distorted_y;


			int offset = r * camera_width_ + c;

			distortPoint(camera_fx, camera_fy,camera_cx, camera_cy, k1, k2, k3, p1, p2,
				c, r, distorted_x, distorted_y);

			distorted_map_x_[offset] = (float)distorted_x;
			distorted_map_y_[offset] = (float)distorted_y;
		}

	}

	/*****************************************************************************************************************/
	 
	el::Loggers::addFlag(el::LoggingFlag::StrictLogFileSizeCheck);
	el::Loggers::reconfigureAllLoggers(el::ConfigurationType::MaxLogFileSize, "104857600");//100MB 104857600

	/// 注册回调函数
	el::Helpers::installPreRollOutCallback(rolloutHandler);


	/********************************************************************************************************/


	  
	return 0;
}

//函数名： DfGetCameraChannels
//功能： 获取相机图像通道数
//输入参数： 无
//输出参数： channels(通道数)
//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
DF_SDK_API int  DfGetCameraChannels(int* channels)
{
	int type = 0;
	int ret = DfGetCameraPixelType(type);

	if (ret != DF_SUCCESS)
	{ 
		return ret;
	}

	XemaPixelType pixel = (XemaPixelType)type;

	switch (pixel)
	{
	case XemaPixelType::Mono:
	{
		*channels = 1;
	}
	break;
	case XemaPixelType::BayerRG8:
	{
		*channels = 3;
	}
	break;

	default:
		*channels = 1;
	}
	 
 

	return DF_SUCCESS;
}

//函数名： DfGetCameraResolution
//功能： 获取相机分辨率
//输入参数： 无
//输出参数： width(图像宽)、height(图像高)
//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
DF_SDK_API int DfGetCameraResolution(int* width, int* height)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetCameraResolution:";
	*width = camera_width_;
	*height = camera_height_;

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_CAMERA_RESOLUTION, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(width), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(height), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{

		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);

	LOG(INFO) << "width: "<< *width;
	LOG(INFO) << "height: "<< *height;
	return DF_SUCCESS;

}

//函数名： DfCaptureRepetitionData
//功能： 采集一帧数据并阻塞至返回状态
//输入参数：repetition_count（重复次数）、 exposure_num（曝光次数）：设置值为1为单曝光，大于1为多曝光模式（具体参数在相机gui中设置）.
//输出参数： timestamp(时间戳)
//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfCaptureRepetitionData(int repetition_count, int exposure_num, char* timestamp)
{

	int ret = -1;

	//if (exposure_num > 1)
	//{
	//	LOG(TRACE) << " Get Frame HDR:";
	//	ret = DfGetFrameHdr(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
	//}
	//else
	//{

	LOG(TRACE) << " Get Repetition Frame04:";
	ret = DfGetRepetitionFrame04(repetition_count, depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
	//}


	//LOG(TRACE) << "Debug Get Temperature:"; 
	//float temperature_value = 0; 
	//ret = DfnGetDeviceTemperature(temperature_value); 
	//LOG(TRACE) << "Temperature: "<< temperature_value;

	//LOG(TRACE) << "Debug Disconnect:";
	//DfnDisconnect();


	//LOG(INFO) << "Debug Disconnect Finished";

	std::string time = get_timestamp();
	for (int i = 0; i < time.length(); i++)
	{
		timestamp[i] = time[i];
	}

	transform_pointcloud_flag_ = false;


	return 0;
}

//函数名： DfGetCaptureEngine
//功能： 设置采集引擎
//输入参数：
//输出参数：engine
//返回值： 类型（int）:返回0表示设置参数成功;返回-1表示设置参数失败。
DF_SDK_API int DfGetCaptureEngine(XemaEngine& engine)
{
	engine = engine_;

	return 0;
}

//函数名： DfSetCaptureEngine
//功能： 设置采集引擎
//输入参数：engine
//输出参数：  
//返回值： 类型（int）:返回0表示设置参数成功;返回-1表示设置参数失败。
DF_SDK_API int DfSetCaptureEngine(XemaEngine engine)
{
	engine_ = engine;

	//std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	//while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	//{
	//	LOG(INFO) << "--";
	//}

	//int val = (int)engine;

	//LOG(INFO) << "DfSetCaptureEngine: " << val;
	//int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	//if (ret == DF_FAILED)
	//{
	//	close_socket(g_sock);
	//	return DF_FAILED;
	//}
	//ret = send_command(DF_CMD_SET_PARAM_CAPTURE_ENGINE, g_sock);
	//ret = send_buffer((char*)&token, sizeof(token), g_sock);
	//int command;
	//ret = recv_command(&command, g_sock);
	//if (ret == DF_FAILED)
	//{
	//	LOG(ERROR) << "Failed to recv command";
	//	close_socket(g_sock);
	//	return DF_FAILED;
	//}

	//if (command == DF_CMD_OK)
	//{
	//	ret = send_buffer((char*)(&val), sizeof(val), g_sock);
	//	if (ret == DF_FAILED)
	//	{
	//		close_socket(g_sock);
	//		return DF_FAILED;
	//	}
	//}
	//else if (command == DF_CMD_REJECT)
	//{
	//	close_socket(g_sock);
	//	return DF_BUSY;
	//}
	//else if (command == DF_CMD_UNKNOWN)
	//{
	//	close_socket(g_sock);
	//	return DF_UNKNOWN;
	//}

	//close_socket(g_sock);



	return DF_SUCCESS;
	 
}

//函数名： DfCaptureData
//功能： 采集一帧数据并阻塞至返回状态
//输入参数： exposure_num（曝光次数）：大于1的为多曝光模式
//输出参数： timestamp(时间戳)
//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp)
{

	LOG(INFO) << "DfCaptureData: "<< exposure_num;
	int ret = -1;

	if (exposure_num > 1)
	{
		switch (multiple_exposure_model_)
		{
		case 1:
		{
			switch (engine_)
			{
			case XemaEngine::Normal:
			{
				ret = DfGetFrameHdr(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_); 
				if (DF_SUCCESS != ret)
				{
					return ret;
				}
			}
				break;
			case XemaEngine::Reflect:
			{
				ret = DfGetFrame06Hdr(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
				if (DF_SUCCESS != ret)
				{
					return ret;
				}
			}
				break;
			case XemaEngine::Black:
			{
				ret = DfGetFrame06HdrMono12(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
				if (DF_SUCCESS != ret)
				{
					return ret;
				}
			}
			break;
			default:
				break;
			}
 
		}
		break;

		case 2:
		{
			switch (engine_)
			{
			case XemaEngine::Normal:
			{
				ret = DfGetRepetitionFrame04(repetition_exposure_model_, depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
				if (DF_SUCCESS != ret)
				{
					return ret;
				}
			}
			break;
			case XemaEngine::Reflect:
			{
				ret = DfGetRepetitionFrame06(repetition_exposure_model_, depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
				if (DF_SUCCESS != ret)
				{
					return ret;
				}
			}
			break;
			case XemaEngine::Black:
			{
				ret = DfGetRepetitionFrame06Mono12(repetition_exposure_model_, depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
				if (DF_SUCCESS != ret)
				{
					return ret;
				}
			}
			break;
			default:
				break;
			}



		}
		default:
			break;
		}
		 
	}
	else
	{
		switch (engine_)
		{
		case XemaEngine::Normal:
		{
			ret = DfGetFrame04(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
			if (DF_SUCCESS != ret)
			{
				return ret;
			}
		}
		break;
		case XemaEngine::Reflect:
		{
			ret = DfGetFrame06(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
			if (DF_SUCCESS != ret)
			{
				return ret;
			}
		}
		break;
		case XemaEngine::Black:
		{
			ret = DfGetFrame06Mono12(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
			if (DF_SUCCESS != ret)
			{
				return ret;
			}
		}
		break;
		default:
			break;
		}



	}
 
	 
	bayer_to_rgb_flag_ = false;
	 
	std::string time = get_timestamp();
	for (int i = 0; i < time.length(); i++)
	{
		timestamp[i] = time[i];
	}

	transform_pointcloud_flag_ = false;

	int status = DF_SUCCESS;
	ret = DfGetFrameStatus(status);
	if (DF_SUCCESS != ret && DF_UNKNOWN != ret)
	{
		LOG(INFO) << "DfGetFrameStatus Failed!";
		//return ret;
	}

	LOG(INFO) << "Frame Status: " << status; 
	if (DF_SUCCESS != status)
	{
		return status;
	}

	return DF_SUCCESS;
}

//函数名： DfGetDepthData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： depth(深度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetDepthData(unsigned short* depth)
{

	LOG(INFO) << "DfGetDepthData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	LOG(INFO) << "Trans Depth:";
	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;

			if (depth_buf_[offset] > 0)
			{
				depth[offset] = depth_buf_[offset] * 10 + 0.5;
			}
			else
			{
				depth[offset] = 0;
			}

		}


	}

	LOG(INFO) << "Get Depth!";

	return 0;
}

//函数名： DfGetDepthDataFloat
//功能： 获取深度图
//输入参数：无
//输出参数： depth(深度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetDepthDataFloat(float* depth)
{
	LOG(INFO) << "DfGetDepthDataFloat:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	LOG(INFO) << "Trans Depth:";
	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

	#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{ 
		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c; 
			depth[offset] = depth_buf_[offset]; 

		} 

	}

	LOG(INFO) << "Get Depth!";

	return DF_SUCCESS;
}

//函数名： DfGetUndistortDepthDataFloat
//功能： 获取去畸变后的深度图
//输入参数：无
//输出参数： depth(深度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetUndistortDepthDataFloat(float* depth)
{
	LOG(INFO) << "DfGetUndistortDepthDataFloat:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	LOG(INFO) << "Trans Depth:";
	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

	#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{ 
		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c; 
			depth[offset] = depth_buf_[offset]; 

		} 

	}

	undistortDepthMap(depth);

	LOG(INFO) << "Get Undistort Depth!";

	return DF_SUCCESS;
}

//函数名： DfGetUndistortColorBrightnessData
//功能： 获取去畸变后的彩色亮度图
//输入参数：无
//输出参数： brightness(亮度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetUndistortColorBrightnessData(unsigned char* brightness, XemaColor color)
{
	std::unique_lock<std::timed_mutex> lck(undistort_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetUndistortColorBrightnessData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	if (pixel_type_ == XemaPixelType::BayerRG8)
	{

		if (!bayer_to_rgb_flag_)
		{
			DfBayerToRgb(brightness_buf_, rgb_buf_);
			bayer_to_rgb_flag_ = true;
		}
		else
		{
			return DF_FAILED;
		}

		if (NULL == rgb_buf_)
		{
			return DF_FAILED;
		}

		switch (color)
		{
		case XemaColor::Rgb:
		{
  
			memcpy(brightness, rgb_buf_, 3 * brightness_bug_size_);
			if (DF_SUCCESS != undistortColorBrightnessMap(brightness))
			{
				return DF_FAILED;
			}

		}
		break;
		case XemaColor::Bgr:
		{ 

			for (int i = 0; i < brightness_bug_size_; i++)
			{
				brightness[3 * i + 0] = rgb_buf_[3 * i + 2];
				brightness[3 * i + 1] = rgb_buf_[3 * i + 1];
				brightness[3 * i + 2] = rgb_buf_[3 * i + 0];
			}

			if (DF_SUCCESS != undistortColorBrightnessMap(brightness))
			{
				return DF_FAILED;
			}
		}
		break;
		case XemaColor::Bayer:
		{ 
			memcpy(brightness, brightness_buf_, brightness_bug_size_);

			if (DF_SUCCESS != undistortBrightnessMap(brightness))
			{
				return DF_FAILED;
			}
		}
		break;
		default:
			break;
		}

	}


	return DF_SUCCESS;
}

//函数名： DfGetColorBrightnessData
//功能： 获取亮度图
//输入参数：无
//输出参数： brightness(亮度图),color(亮度图颜色类型)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetColorBrightnessData(unsigned char* brightness, XemaColor color)
{
	LOG(INFO) << "DfGetColorBrightnessData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}

	if(pixel_type_ == XemaPixelType::BayerRG8)
	{

		if (!bayer_to_rgb_flag_)
		{
			DfBayerToRgb(brightness_buf_, rgb_buf_);
			bayer_to_rgb_flag_ = true;
		}

		switch (color)
		{
		case XemaColor::Rgb:
		{
			memcpy(brightness, rgb_buf_,3*brightness_bug_size_);
		}
			break;
		case XemaColor::Bgr:
		{
			for (int i = 0; i < brightness_bug_size_; i++)
			{
				brightness[3 * i + 0] = rgb_buf_[3 * i + 2];
				brightness[3 * i + 1] = rgb_buf_[3 * i + 1];
				brightness[3 * i + 2] = rgb_buf_[3 * i + 0];
			}
		}
			break;
		case XemaColor::Bayer:
		{
			memcpy(brightness, brightness_buf_, brightness_bug_size_);
		}
			break;
		default:
			break;
		}

	}


	return DF_SUCCESS;
}

//函数名： DfGetBrightnessData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： brightness(亮度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetBrightnessData(unsigned char* brightness)
{ 
	LOG(INFO) << "DfGetBrightnessData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	LOG(INFO) << "Trans Brightness:";

	if (pixel_type_ == XemaPixelType::Mono)
	{
		memcpy(brightness, brightness_buf_, brightness_bug_size_); 
	}
	else if (pixel_type_ == XemaPixelType::BayerRG8)
	{
		if (!bayer_to_rgb_flag_)
		{
			DfBayerToRgb(brightness_buf_, rgb_buf_);
			bayer_to_rgb_flag_ = true;
		}

		DfRgbToGray(rgb_buf_, brightness);
	}
	else
	{
		return DF_FAILED;
	}


	//brightness = brightness_buf_;

	LOG(INFO) << "Get Brightness!";

	return DF_SUCCESS;
}

//函数名： DfGetUndistortBrightnessData
//功能： 获取去畸变后的亮度图
//输入参数：无
//输出参数： brightness(亮度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetUndistortBrightnessData(unsigned char* brightness)
{ 
	LOG(INFO) << "DfGetBrightnessData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	LOG(INFO) << "Trans Brightness:";

	memcpy(brightness, brightness_buf_, brightness_bug_size_);

	undistortBrightnessMap(brightness);

	//brightness = brightness_buf_;

	LOG(INFO) << "Get Undistort Brightness!";

	return 0;
}

//函数名： DfGetStandardPlaneParam
//功能： 获取基准平面参数
//输入参数：无
//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetStandardPlaneParam(float* R, float* T)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetStandardPlaneParam: ";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	int param_buf_size = 12 * 4;
	float* plane_param = new float[param_buf_size];

	ret = send_command(DF_CMD_GET_STANDARD_PLANE_PARAM, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, param_buf_size=" << param_buf_size;
		ret = recv_buffer((char*)plane_param, param_buf_size, g_sock);
		LOG(INFO) << "plane param received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);


	memcpy(R, plane_param, 9 * 4);
	memcpy(T, plane_param + 9, 3 * 4);

	delete[] plane_param;

	LOG(INFO) << "Get plane param success";
	return DF_SUCCESS;

}

//函数名： DfGetHeightMapDataBaseParam
//功能： 获取校正到基准平面的高度映射图
//输入参数：R(旋转矩阵)、T(平移矩阵)
//输出参数： height_map(高度映射图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetHeightMapDataBaseParam(float* R, float* T, float* height_map)
{
	LOG(INFO) << "DfGetHeightMapDataBaseParam:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}
 


	if (!transform_pointcloud_flag_)
	{
		depthTransformPointcloud((float*)depth_buf_, (float*)point_cloud_buf_);
		transform_pointcloud_flag_ = true;
	}
	 
	transformPointcloud((float*)point_cloud_buf_, (float*)trans_point_cloud_buf_, R, T);


	int nr = camera_height_;
	int nc = camera_width_;
#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;
			if (depth_buf_[offset] > 0)
			{
				height_map[offset] = trans_point_cloud_buf_[offset * 3 + 2];
			}
			else
			{
				height_map[offset] = NULL;
			}

		}


	}


	LOG(INFO) << "Get Height Map!";

	return 0;
}

//函数名： DfGetHeightMapData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： height_map(高度映射图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetHeightMapData(float* height_map)
{

	LOG(INFO) << "DfGetHeightMapData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	struct SystemConfigParam system_config_param;
	int ret_code = DfGetSystemConfigParam(system_config_param);
	if (0 != ret_code)
	{
		std::cout << "Get Param Error;";
		return -1;
	}

	LOG(INFO) << "Transform Pointcloud:";

	if (!transform_pointcloud_flag_)
	{
		depthTransformPointcloud((float*)depth_buf_, (float*)point_cloud_buf_);
		transform_pointcloud_flag_ = true;
	}

	//memcpy(trans_point_cloud_buf_, point_cloud_buf_, pointcloud_buf_size_);
	transformPointcloud((float*)point_cloud_buf_, (float*)trans_point_cloud_buf_, system_config_param.standard_plane_external_param, &system_config_param.standard_plane_external_param[9]);


	int nr = camera_height_;
	int nc = camera_width_;
#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;
			if (depth_buf_[offset] > 0)
			{
				height_map[offset] = trans_point_cloud_buf_[offset * 3 + 2];
			}
			else
			{
				height_map[offset] = NULL;
			}

		}


	}


	LOG(INFO) << "Get Height Map!";

	return 0;
}

//函数名： DfGetPointcloudData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： point_cloud(点云)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetPointcloudData(float* point_cloud)
{
	LOG(INFO) << "DfGetPointcloudData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	if (!transform_pointcloud_flag_)
	{
		depthTransformPointcloud(depth_buf_, point_cloud_buf_);
		transform_pointcloud_flag_ = true;
	}

	memcpy(point_cloud, point_cloud_buf_, pointcloud_buf_size_);


	LOG(INFO) << "Get Pointcloud!";

	return 0;
}

//函数名： DfConnect
//功能： 断开相机连接
//输入参数： camera_id（相机id）
//输出参数： 无
//返回值： 类型（int）:返回0表示断开成功;返回-1表示断开失败.
DF_SDK_API int DfDisconnect(const char* camera_id)
{
  
	LOG(INFO) << "DfDisconnect:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	int ret = DfDisconnectNet();

	if (DF_FAILED == ret)
	{
		return DF_FAILED;
	}

	delete[] depth_buf_;
	delete[] brightness_buf_;
	delete[] point_cloud_buf_;
	delete[] trans_point_cloud_buf_;
	delete[] undistort_map_x_;
	delete[] undistort_map_y_;
	delete[] distorted_map_x_;
	delete[] distorted_map_y_;
	delete[] rgb_buf_;

	depth_buf_ = NULL;
	brightness_buf_ = NULL;
	point_cloud_buf_ = NULL;
	trans_point_cloud_buf_ = NULL;
	undistort_map_x_ = NULL;
	undistort_map_y_ = NULL;
	distorted_map_x_ = NULL;
	distorted_map_y_ = NULL;
	rgb_buf_ = NULL;

	connected_flag_ = false;

	/// 注销回调函数
	el::Helpers::uninstallPreRollOutCallback();

	return DF_SUCCESS;
}

//函数名： DfGetCalibrationParam
//功能： 获取相机标定参数
//输入参数： 无
//输出参数： calibration_param（相机标定参数结构体）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetCalibrationParam(struct CalibrationParam* calibration_param)
{
	LOG(INFO) << "DfGetCalibrationParam:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}

	//calibration_param = &calibration_param_;

	for (int i = 0; i < 9; i++)
	{
		calibration_param->intrinsic[i] = calibration_param_.camera_intrinsic[i];
	}

	for (int i = 0; i < 5; i++)
	{
		calibration_param->distortion[i] = calibration_param_.camera_distortion[i];
	}

	for (int i = 5; i < 12; i++)
	{
		calibration_param->distortion[i] = 0;
	}

	float extrinsic[4 * 4] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

	for (int i = 0; i < 16; i++)
	{
		calibration_param->extrinsic[i] = extrinsic[i];
	}


	return 0;
}


/***************************************************************************************************************************************************************/





/*******************************************************************************************************************/

int HeartBeat()
{
	//std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	//while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	//{
	//	LOG(INFO) << "--";
	//}

	LOG(TRACE) << "heart beat: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_HEARTBEAT, g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to send disconnection cmd";
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	ret = send_buffer((char*)&token, sizeof(token), g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to send token";
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	 
	int command;
	ret = recv_command(&command, g_sock_heartbeat); 
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	if (command == DF_CMD_OK)
	{
		ret = DF_SUCCESS;
	}
	else if (command == DF_CMD_REJECT)
	{
		ret = DF_BUSY;
	}
	else
	{
		LOG(ERROR) << "Failed recv heart beat command";
		assert(0);
	}
	close_socket(g_sock_heartbeat);
	return ret;
}

int HeartBeat_loop()
{
	heartbeat_error_count_ = 0;
	while (connected)
	{
		int ret = HeartBeat();
		if (ret == DF_FAILED)
		{
			heartbeat_error_count_++; 
			LOG(ERROR) << "heartbeat error count: " << heartbeat_error_count_;

			if (heartbeat_error_count_ > 2)
			{
				LOG(ERROR) << "close connect";
				connected = false;
				//close_socket(g_sock); 
				p_OnDropped(0);
			}

		}
		else if (DF_SUCCESS == ret)
		{
			heartbeat_error_count_ = 0;
		}


		for (int i = 0; i < 100; i++)
		{
			if (!connected)
			{
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	return 0;
}


DF_SDK_API int DfConnectNet(const char* ip)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfConnectNet: ";
	/*******************************************************************************************************************/
	//关闭log输出
	//el::Configurations conf;
	//conf.setToDefault();
	//conf.setGlobally(el::ConfigurationType::Format, "[%datetime{%H:%m:%s} | %level] %msg");
	//conf.setGlobally(el::ConfigurationType::Filename, "log\\log_%datetime{%Y%M%d}.log");
	//conf.setGlobally(el::ConfigurationType::Enabled, "true");
	//conf.setGlobally(el::ConfigurationType::ToFile, "true");
	//el::Loggers::reconfigureAllLoggers(conf);
	//el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");


	//DfRegisterOnDropped(on_dropped);
	/*******************************************************************************************************************/


	camera_id_ = ip;
	LOG(INFO) << "start connection: " << ip;
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	LOG(INFO) << "sending connection cmd";
	ret = send_command(DF_CMD_CONNECT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to send connection cmd";
		close_socket(g_sock);
		return DF_FAILED;
	}
	int command;
	ret = recv_command(&command, g_sock); 
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}
	if (ret == DF_SUCCESS)
	{
		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "Recieved connection ok";
			ret = recv_buffer((char*)&token, sizeof(token), g_sock);
			if (ret == DF_SUCCESS)
			{
				connected = true;
				LOG(INFO) << "token: " << token;
				close_socket(g_sock);
				if (heartbeat_thread.joinable())
				{
					heartbeat_thread.join();
				}
				heartbeat_thread = std::thread(HeartBeat_loop);
				return DF_SUCCESS;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "connection rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}
	}
	else
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	return DF_FAILED;
}

DF_SDK_API int DfDisconnectNet()
{

	LOG(INFO) << "token " << token << " try to disconnection";
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
		 
	}


	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to setup_socket";
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_DISCONNECT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to send disconnection cmd";
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to send token";
		close_socket(g_sock);
		return DF_FAILED;
	}
	int command;
	ret = recv_command(&command, g_sock); 
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}
	connected = false;
	token = 0;

	if (heartbeat_thread.joinable())
	{
		heartbeat_thread.join();
	}

	LOG(INFO) << "Camera disconnected";
	return close_socket(g_sock);
}

//函数名： DfGetFocusingImage
//功能： 获取一个对焦图数据
//输入参数：image_buf_size（亮度图尺寸sizeof(unsigned char) * width * height）
//输出参数：image
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetFocusingImage(unsigned char* image, int image_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetFocusingImage";
	assert(image_buf_size >= image_size_ * sizeof(unsigned char));
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	//std::cout << "1" << std::endl;
	ret = send_command(DF_CMD_GET_FOCUSING_IMAGE, g_sock);
	//std::cout << "send token " << token<< std::endl;
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock); 
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}
	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		ret = recv_buffer((char*)image, image_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get Focusing Image rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get  Focusing Image success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int GetBrightness(unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "GetBrightness";
	assert(brightness_buf_size >= image_size_ * sizeof(unsigned char));
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	//std::cout << "1" << std::endl;
	ret = send_command(DF_CMD_GET_BRIGHTNESS, g_sock);
	//std::cout << "send token " << token<< std::endl;
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}
	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get brightness rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get brightness success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetCameraData(
	short* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size,
	short* point_cloud, int point_cloud_buf_size,
	unsigned char* confidence, int confidence_buf_size)
{
	int ret = DF_SUCCESS;
	if (depth)
	{
		assert(depth_buf_size >= image_size_ * sizeof(short));
		send_command(DF_CMD_GET_DEPTH, g_sock);
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			return DF_FAILED;
		}
	}
	if (brightness)
	{
		GetBrightness(brightness, brightness_buf_size);
	}

	if (point_cloud)
	{
		assert(point_cloud_buf_size >= image_size_ * sizeof(short) * 3);
		send_command(DF_CMD_GET_POINTCLOUD, g_sock);
		ret = recv_buffer((char*)point_cloud, point_cloud_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			return DF_FAILED;
		}
	}
	if (confidence)
	{
		assert(confidence_buf_size >= image_size_ * sizeof(unsigned char));
		send_command(DF_CMD_GET_CONFIDENCE, g_sock);
		ret = recv_buffer((char*)confidence, confidence_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			return DF_FAILED;
		}
	}
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrameHdr(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "GetFrameHdr";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_FRAME_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}



		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}




	close_socket(g_sock);

	//int status = -1;
	//ret = DfGetFrameStatus(status);
	//if (ret == DF_FAILED)
	//{
	//	LOG(ERROR) << "Failed to DfGetFrameStatus";
	//	close_socket(g_sock);
	//	return DF_ERROR_NETWORK;
	//}

	//LOG(INFO) << "Frame Status: " << status;
	//if (DF_SUCCESS != ret)
	//{
	//	return status;
	//}


	LOG(INFO) << "Get frame success";
	return DF_SUCCESS;
}

//函数名： DfGetRepetitionPhase02
//功能： 获取一帧数据（亮度图+相位图），基于Raw02相位图，所有条纹图重复count次
//输入参数：count（重复次数）、phase_buf_size（相位图尺寸）、brightness_buf_size（亮度图尺寸）
//输出参数：phase_x（相位图x）、phase_y（相位图y）、brightness（亮度图）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetRepetitionPhase02(int count, float* phase_x, float* phase_y, int phase_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetRepetitionPhase02";
	assert(phase_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PHASE_02_REPETITION, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";

		ret = send_buffer((char*)(&count), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer x, phase_buf_size=" << phase_buf_size;
		ret = recv_buffer((char*)phase_x, phase_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer y, phase_buf_size=" << phase_buf_size;
		ret = recv_buffer((char*)phase_y, phase_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}



		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetRepetitionFrame06
//功能： 获取一帧数据（亮度图+深度图），基于Raw06图重复count次
//输入参数：count（重复次数）、depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
//输出参数：depth（深度图）、brightness（亮度图）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetRepetitionFrame06(int count, float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "GetRepetitionFrame06";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_REPETITION_FRAME_06, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&count), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetRepetitionFrame04
//功能： 获取一帧数据（亮度图+深度图），基于Raw04相位图，6步相移的图重复count次
//输入参数：count（重复次数）、depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
//输出参数：depth（深度图）、brightness（亮度图）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetRepetitionFrame04(int count, float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "GetRepetition01Frame04";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_REPETITION_FRAME_04, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&count), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetRepetitionFrame03(int count, float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetRepetitionFrame03";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_REPETITION_FRAME_03, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&count), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrame03(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetFrame03";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_FRAME_03, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrame06Hdr(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetFrame06Hdr";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	} 
	
	ret = send_command(DF_CMD_GET_FRAME_06_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);


	LOG(INFO) << "Get FRAME 06 HDR success";
	return DF_SUCCESS;
}


//函数名： DfGetRepetitionFrame06Mono12
//功能： 获取一帧数据（亮度图+深度图），基于Raw06图重复count次
//输入参数：count（重复次数）、depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
//输出参数：depth（深度图）、brightness（亮度图）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetRepetitionFrame06Mono12(int count, float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetRepetitionFrame06Mono12";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_REPETITION_FRAME_06_MONO12, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&count), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetFrame06HdrMono12
//功能： 获取一帧数据（亮度图+深度图），基于Raw06相位图
//输入参数：depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
//输出参数：depth（深度图）、brightness（亮度图）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetFrame06HdrMono12(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{

	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "DfGetFrame06HdrMono12";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	ret = send_command(DF_CMD_GET_FRAME_06_HDR_MONO12, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		LOG(INFO) << "Get frame DF_CMD_UNKNOWN";
		return DF_UNKNOWN;
	}

	close_socket(g_sock);



	LOG(INFO) << "Get frame06 success";
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrame06Mono12(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{

	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetFrame06";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	 
	ret = send_command(DF_CMD_GET_FRAME_06_MONO12, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		LOG(INFO) << "Get frame DF_CMD_UNKNOWN";
		return DF_UNKNOWN;
	}

	close_socket(g_sock);



	LOG(INFO) << "Get frame06 success";
	return DF_SUCCESS;
}

//函数名： DfGetFrame06
//功能： 获取一帧数据（亮度图+深度图），基于Raw06相位图
//输入参数：depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
//输出参数：depth（深度图）、brightness（亮度图）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetFrame06(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetFrame06";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_FRAME_06, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		LOG(INFO) << "Get frame DF_CMD_UNKNOWN";
		return DF_UNKNOWN;
	}

	close_socket(g_sock);

 

	LOG(INFO) << "Get frame06 success";
	return DF_SUCCESS;
}


DF_SDK_API int DfGetFrame04(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetFrame04";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_FRAME_04, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);

	//int status = -1;
	//ret = DfGetFrameStatus(status);
	//if (ret == DF_FAILED)
	//{
	//	LOG(ERROR) << "Failed to DfGetFrameStatus";
	//	close_socket(g_sock);
	//	return DF_ERROR_NETWORK;
	//}

	//LOG(INFO) << "Frame Status: " << status;
	//if (DF_SUCCESS != ret)
	//{
	//	return status;
	//}

	LOG(INFO) << "Get frame04 success";
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrame05(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetFrame05";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_FRAME_05, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrame01(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "GetFrame01";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_FRAME_01, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}



		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetPointCloud(float* point_cloud, int point_cloud_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "GetPointCloud";
	assert(point_cloud_buf_size == image_size_ * sizeof(float) * 3);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_POINTCLOUD, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, point_cloud_buf_size=" << point_cloud_buf_size;
		ret = recv_buffer((char*)point_cloud, point_cloud_buf_size, g_sock);
		LOG(INFO) << "point_cloud received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get point_cloud rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get point_cloud success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetCameraRawData04Repetition
//功能： 采集一组相移图，一共13+6*repetition_count幅，12个四步相移条纹图+6*repetition_count个垂直方向的六步相移条纹图+一个亮度图
//输入参数：raw_buf_size（13+6*repetition_count张8位图的尺寸）
//输出参数：raw
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetCameraRawData04Repetition(unsigned char* raw, int raw_buf_size, int repetition_count)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "GetRaw04Repetition";

		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_04_REPETITION, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			ret = send_buffer((char*)(&repetition_count), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

//函数名： DfGetCameraRawData04
//功能： 采集一组相移图，一共19幅，12个四步相移条纹图+6个垂直方向的六步相移条纹图+一个亮度图
//输入参数：raw_buf_size（19张8位图的尺寸）
//输出参数：raw
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetCameraRawData04(unsigned char* raw, int raw_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "GetRaw04";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * 19);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_04, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

//函数名： DfGetCameraRawData05
//功能： 采集一组相移图，一共16幅，8个xor码+6个垂直方向的六步相移条纹图 + 黑白2个图
//输入参数：raw_buf_size（16张8位图的尺寸）
//输出参数：raw
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetCameraRawData05(unsigned char* raw, int raw_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "DfGetCameraRawData05";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * 16);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_05, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

//函数名： DfGetCameraRawData06
//功能： 采集一组相移图，一共18幅，10个minsw码+6个垂直方向的六步相移条纹图 + 黑白2个图
//输入参数：raw_buf_size（18张8位图的尺寸）
//输出参数：raw
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetCameraRawData06(unsigned char* raw, int raw_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "DfGetCameraRawData06";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * 18);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_06, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

//函数名： DfGetCameraRawData08
//功能： 采集一组相移图，一共14幅，8个XOR码+6个垂直方向的六步相移条纹图+一个亮度图+一个暗图
//输入参数：raw_buf_size（16张8位图的尺寸）
//输出参数：raw
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetCameraRawData08(unsigned char* raw, int raw_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "DfGetCameraRawData08";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * 16);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_08, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

DF_SDK_API int DfGetCameraRawData03(unsigned char* raw, int raw_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "GetRaw03";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * 31);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_03, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

DF_SDK_API int DfGetCameraRawData02(unsigned char* raw, int raw_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "GetRawTest";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * 37);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_TEST, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

DF_SDK_API int DfGetCameraRawDataTest(unsigned char* raw, int raw_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "GetRawTest";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * 37);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_TEST, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}



DF_SDK_API int DfGetCameraRawData01(unsigned char* raw, int raw_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "Get Raw 01";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * 24);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

DF_SDK_API int DfGetDeviceTemperature(float& temperature)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_TEMPERATURE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&temperature), sizeof(temperature), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		LOG(ERROR) << "CPU temperature: "<< temperature;
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

// --------------------------------------------------------------
// -- Enable and disable checkerboard, by wantong, 2022-01-27
DF_SDK_API int DfEnableCheckerboard(float& temperature)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_ENABLE_CHECKER_BOARD, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&temperature), sizeof(temperature), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfDisableCheckerboard(float& temperature)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_DISABLE_CHECKER_BOARD, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&temperature), sizeof(temperature), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}
// --------------------------------------------------------------
DF_SDK_API int DfLoadPatternData(int buildDataSize, char* LoadBuffer)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_LOAD_PATTERN_DATA, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&buildDataSize), sizeof(buildDataSize), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer(LoadBuffer, buildDataSize, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfProgramPatternData(char* org_buffer, char* back_buffer, unsigned int pattern_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_PROGRAM_PATTERN_DATA, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&pattern_size), sizeof(pattern_size), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer(org_buffer, pattern_size, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer(back_buffer, pattern_size, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}
// --------------------------------------------------------------
DF_SDK_API int DfGetNetworkBandwidth(int& speed)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_NETWORK_BANDWIDTH, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&speed), sizeof(speed), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

// --------------------------------------------------------------
DF_SDK_API int DfSelfTest(char* pTest, int length)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SELF_TEST, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer(pTest, length, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

// --------------------------------------------------------------
DF_SDK_API int DfGetFirmwareVersion(char* pVersion, int length)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_FIRMWARE_VERSION, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer(pVersion, length, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetProductInfo(char* info, int length)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PRODUCT_INFO, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer(info, length, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//函数名： DfGetCameraPixelType
//功能： 获取相机像素类型
//输入参数：无
//输出参数： type（类型）
//返回值： 类型（int）:返回0表示获取数据成功;否则表示获取数据失败.
DF_SDK_API int DfGetCameraPixelType(int& type)
{

	//std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	//while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	//{
	//	LOG(INFO) << "--";
	//}

	int get_type = 0;

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_ERROR_NETWORK;
	}
	ret = send_command(DF_CMD_GET_CAMERA_PIXEL_TYPE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_ERROR_NETWORK;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&get_type), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_ERROR_NETWORK;
		}

		LOG(INFO) << "Frame Status: " << get_type;
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	type = get_type;
	return DF_SUCCESS;
}

//函数名： DfGetFrameStatus
//功能： 获取当前帧数据状态
//输入参数：无
//输出参数： status（状态码）
//返回值： 类型（int）:返回0表示获取数据成功;否则表示获取数据失败.
DF_SDK_API int DfGetFrameStatus(int& status)
{
	//std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	//while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	//{
	//	LOG(INFO) << "--";
	//}

	int get_status = 0;

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_ERROR_NETWORK;
	}
	ret = send_command(DF_CMD_GET_FRAME_STATUS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_ERROR_NETWORK;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&get_status), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_ERROR_NETWORK;
		}

		LOG(INFO) << "Frame Status: "<< status;
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock); 
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	status = get_status;
	return DF_SUCCESS;
}

DF_SDK_API int DfGetProjectorTemperature(float& temperature)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PROJECTOR_TEMPERATURE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&temperature), sizeof(temperature), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		LOG(ERROR) << "projector temperature: " << temperature;
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

// --------------------------------------------------------------
DF_SDK_API int DfGetSystemConfigParam(struct SystemConfigParam& config_param)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&config_param), sizeof(config_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfSetSystemConfigParam(const struct SystemConfigParam& config_param)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&config_param), sizeof(config_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetCalibrationParam(struct CameraCalibParam& calibration_param)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_CAMERA_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&calibration_param), sizeof(calibration_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetCalibrationLookTable
//功能：设置标定参数接口
//输入参数：calibration_param（标定参数）,rotate_x、rotate_y, rectify_r1, mapping
//输出参数：无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfSetCalibrationLookTable(const struct CameraCalibParam& calibration_param, float* rotate_x,
	float* rotate_y, float* rectify_r1, float* mapping, float* mini_mapping, int width, int height)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_CAMERA_LOOKTABLE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "start send_buffer: calibration_param";
		ret = send_buffer((char*)(&calibration_param), sizeof(calibration_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		/*****************************************************************/

		LOG(INFO) << "start send_buffer rotate_x size: " << width * height * sizeof(float);
		ret = send_buffer((char*)(rotate_x), width * height * sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		LOG(INFO) << "start send_buffer: rotate_y";
		ret = send_buffer((char*)(rotate_y), width * height * sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "start send_buffer: rectify_r1";
		ret = send_buffer((char*)(rectify_r1), 3 * 3 * sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		LOG(INFO) << "start send_buffer: mapping";
		ret = send_buffer((char*)(mapping), 4000 * 2000 * sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "start send_buffer: mapping";
		ret = send_buffer((char*)(mini_mapping), 128 * 128 * sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//函数名： DfSetCalibrationMiniLookTable
//功能：设置压缩表标定参数接口
//输入参数：calibration_param（标定参数）,rotate_x、rotate_y, rectify_r1, mapping
//输出参数：无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfSetCalibrationMiniLookTable(const struct CameraCalibParam& calibration_param, float* rotate_x,
	float* rotate_y, float* rectify_r1, float* mapping)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_CAMERA_MINILOOKTABLE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "start send_buffer: calibration_param";
		ret = send_buffer((char*)(&calibration_param), sizeof(calibration_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		/*****************************************************************/

		LOG(INFO) << "start send_buffer rotate_x size: " << 1920 * 1200 * sizeof(float);
		ret = send_buffer((char*)(rotate_x), 1920 * 1200 * sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		LOG(INFO) << "start send_buffer: rotate_y";
		ret = send_buffer((char*)(rotate_y), 1920 * 1200 * sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "start send_buffer: rectify_r1";
		ret = send_buffer((char*)(rectify_r1), 3 * 3 * sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		LOG(INFO) << "start send_buffer: mapping";
		ret = send_buffer((char*)(mapping), 128 * 128 * sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


DF_SDK_API int DfSetCalibrationParam(const struct CameraCalibParam& calibration_param)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_CAMERA_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&calibration_param), sizeof(calibration_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfRegisterOnDropped(int (*p_function)(void*))
{
	p_OnDropped = p_function;
	return 0;
}


DF_SDK_API int DfSetAutoExposure(int flag, int& exposure, int& led)
{

	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}


	if (flag != 1 && flag != 2)
	{
		LOG(INFO) << "error flag:  " << flag << std::endl;
		return DF_FAILED;
	}

 

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (1 == flag)
	{
		ret = send_command(DF_CMD_SET_AUTO_EXPOSURE_BASE_ROI, g_sock);
	}
	else if (2 == flag)
	{
		ret = send_command(DF_CMD_SET_AUTO_EXPOSURE_BASE_BOARD, g_sock);
	}

	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&exposure), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&led), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

/*****************************************************************************************************/


//函数名： DfSetParamSmoothing
//功能： 设置点云平滑参数
//输入参数：smoothing(0:关、1：小、2：中、3：大)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamSmoothing(int smoothing)
{

	LOG(INFO) << "DfSetParamSmoothing:";
	int ret = -1;
	if (0 == smoothing)
	{
		ret = DfSetParamBilateralFilter(0, 5);
	}
	else
	{
		ret = DfSetParamBilateralFilter(1, 2 * smoothing + 1);
	}

	return ret;
}

//函数名： DfGetParamSmoothing
//功能： 设置点云平滑参数
//输入参数：无
//输出参数：smoothing(0:关、1：小、2：中、3：大)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamSmoothing(int& smoothing)
{

	LOG(INFO) << "DfGetParamSmoothing:";
	int use = 0;
	int d = 0;

	int ret = DfGetParamBilateralFilter(use, d);

	if (DF_FAILED == ret)
	{
		return DF_FAILED;
	}

	if (0 == use)
	{
		smoothing = 0;
	}
	else if (1 == use)
	{
		smoothing = d / 2;
	}
	return DF_SUCCESS;
}

//函数名： DfSetParamBilateralFilter
//功能： 设置双边滤波参数
//输入参数： use（开关：1为开、0为关）、param_d（平滑系数：3、5、7、9、11）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfSetParamBilateralFilter(int use, int param_d)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (use != 1 && use != 0)
	{
		std::cout << "use param should be 1 or 0:  " << use << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_BILATERAL_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&param_d), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamBilateralFilter
//功能： 获取混合多曝光参数（最大曝光次数为6次）
//输入参数： 无
//输出参数： use（开关：1为开、0为关）、param_d（平滑系数：3、5、7、9、11）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetParamBilateralFilter(int& use, int& param_d)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_BILATERAL_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&param_d), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamRadiusFilter
//功能： 设置点云半径滤波参数
//输入参数：use(开关：1开、0关)、radius(半径）、num（有效点）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamRadiusFilter(int use, float radius, int num)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamRadiusFilter:";
	if (use != 1 && use != 0)
	{
		std::cout << "use param should be 1 or 0:  " << use << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_RADIUS_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&radius), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&num), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamRadiusFilter
//功能： 获取点云半径滤波参数
//输入参数：无
//输出参数：use(开关：1开、0关)、radius(半径）、num（有效点）
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamRadiusFilter(int& use, float& radius, int& num)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamRadiusFilter:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_RADIUS_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&radius), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&num), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamDepthFilter
//功能： 设置深度图滤波参数
//输入参数：use(开关：1开、0关)、depth_filterthreshold(深度图在1000mm距离过滤的噪声阈值)
//输出参数： 无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamDepthFilter(int use, float depth_filter_threshold)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamDepthFilter:";
	if (use != 1 && use != 0)
	{
		std::cout << "use param should be 1 or 0:  " << use << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_DEPTH_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&depth_filter_threshold), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamDepthFilter
//功能： 设置深度图滤波参数
//输入参数：use(开关：1开、0关)、depth_filterthreshold(深度图在1000mm距离过滤的噪声阈值)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
DF_SDK_API int DfGetParamDepthFilter(int& use, float& depth_filter_threshold)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamDepthFilter:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_DEPTH_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&depth_filter_threshold), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamReflectFilter
//功能： 设置点云半径滤波参数
//输入参数：use(开关：1开、0关) 
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamReflectFilter(int use)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	if (use != 1 && use != 0)
	{
		std::cout << "use param should be 1 or 0:  " << use << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_REFLECT_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamGrayRectify
//功能： 设置点云灰度补偿参数
//输入参数：use(开关：1开、0关)、radius(半径：3、5、7、9）、sigma（补偿强度，范围1-100）
//输出参数： 无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamGrayRectify(int use, int radius, float sigma)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (radius % 2 != 1 || radius > 9 || radius < 3 || sigma < 1 || sigma > 100)
	{
		LOG(INFO) << "error param range";
		return DF_FAILED;
	}

	LOG(INFO) << "DfSetParamGrayRectify:";
	if (use != 1 && use != 0)
	{
		std::cout << "use param should be 1 or 0:  " << use << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_GRAY_RECTIFY, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&radius), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&sigma), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamGrayRectify
//功能： 获取点云灰度补偿参数
//输入参数：无
//输出参数：use(开关：1开、0关)、radius(半径：3、5、7、9）、sigma（补偿强度，范围0-100）
//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
DF_SDK_API int DfGetParamGrayRectify(int& use, int& radius, float& sigma)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamGrayRectify:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_GRAY_RECTIFY, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&radius), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&sigma), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetBoardInspect
//功能： 设置标定板检测
//输入参数：enable(开关) 
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetBoardInspect(bool enable)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_INSPECT_MODEL_FIND_BOARD, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		int use = 1;

		if (enable)
		{
			use = 1;
		}
		else
		{
			use = 0;
		}

		ret = send_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//函数名： DfGetParamReflectFilter
//功能： 设置点云平滑参数
//输入参数：无
//输出参数：use(开关：1开、0关)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamReflectFilter(int& use)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_REFLECT_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		 
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamOutlierFilter
//功能： 设置过滤阈值
//输入参数：threshold(阈值0-100)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamOutlierFilter(float threshold)
{ 
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamOutlierFilter:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_FISHER_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&threshold), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamOutlierFilter
//功能： 获取过滤阈值
//输入参数： 无
//输出参数：threshold(阈值0-100)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamOutlierFilter(float& threshold)
{ 
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamOutlierFilter:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_FISHER_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&threshold), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamMultipleExposureModel
//功能： 设置多曝光模式
//输入参数： model(1：HDR(默认值)、2：重复曝光)
//输出参数：无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamMultipleExposureModel(int model)
{
	if (model != 1 && model != 2)
	{
		return DF_ERROR_INVALID_PARAM;
	}
	multiple_exposure_model_ = model;

	return DF_SUCCESS;
}

//函数名： DfSetParamRepetitionExposureNum
//功能： 设置重复曝光数
//输入参数： num(2-10)
//输出参数：无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamRepetitionExposureNum(int num)
{
	if (num < 2 || num >10)
	{ 
		return DF_ERROR_INVALID_PARAM;
	}

	repetition_exposure_model_ = num;

	return DF_SUCCESS;
}

//函数名： DfGetParamOffset
//功能： 获取补偿参数
//输入参数：无
//输出参数：offset(补偿值)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamOffset(float& offset)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_OFFSET, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&offset), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamOffset
//功能： 设置补偿参数
//输入参数：offset(补偿值)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamOffset(float offset)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (offset < 0)
	{
		std::cout << "offset param out of range!" << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_OFFSET, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&offset), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamCameraConfidence
//功能： 设置相机曝光时间
//输入参数：confidence(相机置信度)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamCameraConfidence(float confidence)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamCameraConfidence:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_CAMERA_CONFIDENCE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&confidence), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamCameraConfidence
//功能： 获取相机曝光时间
//输入参数： 无
//输出参数：confidence(相机置信度)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamCameraConfidence(float& confidence)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamCameraConfidence:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_CAMERA_CONFIDENCE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&confidence), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamCameraGain
//功能： 设置相机增益
//输入参数：gain(相机增益)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamCameraGain(float gain)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamCameraGain:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_CAMERA_GAIN, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&gain), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamCameraGain
//功能： 获取相机增益
//输入参数： 无
//输出参数：gain(相机增益)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamCameraGain(float& gain)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "DfGetParamCameraGain:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_CAMERA_GAIN, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&gain), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetSdkVersion
//功能： 获取sdk版本
//输入参数：无
//输出参数：version(版本)
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfGetSdkVersion(char version[64])
{
	std::strcpy(version, "v1.5.1"); 

	return DF_SUCCESS;
}

//函数名： DfCaptureBrightnessData
//功能： 获取亮度图
//输入参数：无
//输出参数： brightness(亮度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfCaptureBrightnessData(unsigned char* brightness, XemaColor color)
{
	LOG(INFO) << "brightness_bug_size: " << brightness_bug_size_;
	int ret = DF_SUCCESS;
	ret = GetBrightness(brightness_buf_, brightness_bug_size_);

	if (DF_SUCCESS != ret)
	{
		return ret;
	}


	if (pixel_type_ == XemaPixelType::BayerRG8)
	{ 
		DfBayerToRgb(brightness_buf_, rgb_buf_);
	 
		switch (color)
		{
		case XemaColor::Rgb:
		{
			memcpy(brightness, rgb_buf_, 3 * brightness_bug_size_);
		}
		break;
		case XemaColor::Bgr:
		{
			for (int i = 0; i < brightness_bug_size_; i++)
			{
				brightness[3 * i + 0] = rgb_buf_[3 * i + 2];
				brightness[3 * i + 1] = rgb_buf_[3 * i + 1];
				brightness[3 * i + 2] = rgb_buf_[3 * i + 0];
			}
		}
		break;
		case XemaColor::Bayer:
		{
			memcpy(brightness, brightness_buf_, brightness_bug_size_);
		}
		break;
		case XemaColor::Gray:
		{  
			DfRgbToGray(rgb_buf_, brightness);
		}
		break;
		default:
			break;
		}

	}
	else
	{ 

		switch (color)
		{
		case XemaColor::Rgb:
		{
			for (int i = 0; i < brightness_bug_size_; i++)
			{
				brightness[3 * i + 0] = brightness_buf_[i];
				brightness[3 * i + 1] = brightness_buf_[i];
				brightness[3 * i + 2] = brightness_buf_[i];
			}
		}
		break;
		case XemaColor::Bgr:
		{
			for (int i = 0; i < brightness_bug_size_; i++)
			{
				brightness[3 * i + 0] = brightness_buf_[i];
				brightness[3 * i + 1] = brightness_buf_[i];
				brightness[3 * i + 2] = brightness_buf_[i];
			}
		}
		break;
		case XemaColor::Bayer:
		{
			memcpy(brightness, brightness_buf_, brightness_bug_size_);
		}
		break;
		case XemaColor::Gray:
		{
			memcpy(brightness, brightness_buf_, brightness_bug_size_);
		}
		break;
		default:
			break;
		}
		 
	}


	return ret;
}

//函数名： DfGetParamBrightnessGain
//功能： 获取亮度图增益
//输入参数：无
//输出参数：gain(亮度图增益)
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfGetParamBrightnessGain(float& gain)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamBrightnessGain:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_BRIGHTNESS_GAIN, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{ 
		ret = recv_buffer((char*)(&gain), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamBrightnessExposureModel
//功能： 设置亮度图曝光模式
//输入参数： model（1：单曝光、2：曝光融合）
//输出参数： 无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamBrightnessExposureModel(int model)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamBrightnessExposureModel:";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_BRIGHTNESS_EXPOSURE_MODEL, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&model), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamBrightnessExposureModel
//功能： 获取亮度图曝光模式
//输入参数： 无
//输出参数： model（1：单曝光、2：曝光融合）
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfGetParamBrightnessExposureModel(int& model)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamBrightnessExposureModel:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_BRIGHTNESS_EXPOSURE_MODEL, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&model), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamBrightnessGain
//功能： 设置亮度图增益
//输入参数：gain(亮度图增益)
//输出参数： 无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamBrightnessGain(float gain)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamBrightnessGain:";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_BRIGHTNESS_GAIN, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&gain), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamCameraExposure
//功能： 设置相机曝光时间
//输入参数：exposure(相机曝光时间)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamCameraExposure(float exposure)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamCameraExposure:";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&exposure), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamCameraExposure
//功能： 获取相机曝光时间
//输入参数： 无
//输出参数：exposure(相机曝光时间)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamCameraExposure(float& exposure)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamCameraExposure:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&exposure), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamGenerateBrightness
//功能： 设置生成亮度图参数
//输入参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamGenerateBrightness(int model, float exposure)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamGenerateBrightness: ";
	if (exposure < 20 || exposure> 1000000)
	{
		std::cout << "exposure param out of range!" << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&model), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&exposure), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamGenerateBrightness
//功能： 获取生成亮度图参数
//输入参数： 无
//输出参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamGenerateBrightness(int& model, float& exposure)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamGenerateBrightness: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&model), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&exposure), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//函数名： DfSetParamStandardPlaneExternal
//功能： 设置基准平面的外参
//输入参数：R(旋转矩阵：3*3)、T(平移矩阵：3*1)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamStandardPlaneExternal(float* R, float* T)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamStandardPlaneExternal: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}
	 
	if (command == DF_CMD_OK)
	{

		float plane_param[12];

		memcpy(plane_param, R, 9 * sizeof(float));
		memcpy(plane_param + 9, T, 3 * sizeof(float));

		ret = send_buffer((char*)(plane_param), sizeof(float) * 12, g_sock);


		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;


}

//函数名： DfGetParamStandardPlaneExternal
//功能： 获取基准平面的外参
//输入参数：无
//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamStandardPlaneExternal(float* R, float* T)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamStandardPlaneExternal: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		//int param_buf_size = 12 * sizeof(float);
		//float* plane_param = new float[param_buf_size];
		float plane_param[12];

		ret = recv_buffer((char*)(plane_param), sizeof(float) * 12, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		memcpy(R, plane_param, 9 * sizeof(float));
		memcpy(T, plane_param + 9, 3 * sizeof(float));

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamBrightnessHdrExposure
//功能： 设置亮度图多曝光参数（最大曝光次数为10次）
//输入参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效））
//输出参数： 无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamBrightnessHdrExposure(int num, int exposure_param[10])
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (num < 2 || num>10)
	{
		LOG(INFO) << "Error num:"<< num;
		return DF_FAILED;
	}

	LOG(INFO) << "DfSetParamBrightnessHdrExposure:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_BRIGHTNESS_HDR_EXPOSURE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		int param[11] = {2,10000,20000,30000,40000,50000,60000,70000,80000,90000,100000 };

		param[0] = num;

		memcpy(param + 1, exposure_param, sizeof(int) * num);

		ret = send_buffer((char*)(param), sizeof(int) * 11, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamBrightnessHdrExposure
//功能： 设置亮度图多曝光参数（最大曝光次数为10次）
//输入参数：无 
//输出参数：num（曝光次数）、exposure_param[10]（10个曝光参数、前num个有效））
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfGetParamBrightnessHdrExposure(int& num, int exposure_param[10])
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamBrightnessHdrExposure:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_BRIGHTNESS_HDR_EXPOSURE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		int param[11];

		ret = recv_buffer((char*)(param), sizeof(int) * 11, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		 
		memcpy(exposure_param, param + 1, sizeof(int) * 10); 
		num = param[0];

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamMixedHdr
//功能： 设置混合多曝光参数（最大曝光次数为6次）
//输入参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）、led_param[6]（6个led亮度参数、前num个有效）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfSetParamMixedHdr(int num, int exposure_param[6], int led_param[6])
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamMixedHdr:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_MIXED_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		int param[13];
		param[0] = num;

		memcpy(param + 1, exposure_param, sizeof(int) * 6);
		memcpy(param + 7, led_param, sizeof(int) * 6);

		ret = send_buffer((char*)(param), sizeof(int) * 13, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamMixedHdr
//功能： 获取混合多曝光参数（最大曝光次数为6次）
//输入参数： 无
//输出参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）、led_param[6]（6个led亮度参数、前num个有效）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetParamMixedHdr(int& num, int exposure_param[6], int led_param[6])
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamMixedHdr:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_MIXED_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		int param[13];

		ret = recv_buffer((char*)(param), sizeof(int) * 13, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		memcpy(exposure_param, param + 1, sizeof(int) * 6);
		memcpy(led_param, param + 7, sizeof(int) * 6);
		num = param[0];

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//函数名： DfSetParamHdr
//功能： 设置多曝光参数（最大曝光次数为6次）
//输入参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfSetParamHdr(int num, int exposure_param[6])
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		int param[7];
		param[0] = num;

		memcpy(param + 1, exposure_param, sizeof(int) * 6);

		ret = send_buffer((char*)(param), sizeof(int) * 7, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//函数名： DfGetParamHdr
//功能： 设置多曝光参数（最大曝光次数为6次）
//输入参数： 无
//输出参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetParamHdr(int& num, int exposure_param[6])
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		int param[7];

		ret = recv_buffer((char*)(param), sizeof(int) * 7, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		memcpy(exposure_param, param + 1, sizeof(int) * 6);
		num = param[0];

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//函数名： DfSetParamLedCurrent
//功能： 设置LED电流
//输入参数： led（电流值）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfSetParamLedCurrent(int led)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamLedCurrent: "<<led;
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_LED_CURRENT, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&led), sizeof(led), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//函数名： DfGetParamLedCurrent
//功能： 设置LED电流
//输入参数： 无
//输出参数： led（电流值）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetParamLedCurrent(int& led)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamLedCurrent: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_CAMERA_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&led), sizeof(led), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	LOG(INFO) << "led: " << led;
	return DF_SUCCESS;
}

//函数名：  DfGetProjectVersion
//功能：    获取相机型号
//输入参数：无
//输出参数：型号（3010、4710）
//返回值：  类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetProjectorVersion(int& version)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_PROJECTOR_VERSION, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&version), sizeof(version), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名：  DfGetCameraVersion
//功能：    获取相机型号
//输入参数：无
//输出参数：型号（800、1800）
//返回值：  类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetCameraVersion(int& version)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_CAMERA_VERSION, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&version), sizeof(version), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetTestFrame01
//功能： 传送一组Raw01的相移图到firmware,重建回一帧数据（亮度图+深度图）
//输入参数：raw（相移图地址）、raw_buf_size（相移图尺寸）、 depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
//输出参数：depth（深度图）、brightness（亮度图）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetTestFrame01(unsigned char* raw, int raw_buf_size, float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "GetTestFrame01";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_TEST_GET_FRAME_01, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";

		ret = send_buffer((char*)raw, raw_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(INFO) << "send raw Failed!";
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

/*********************************************************************************************************/
