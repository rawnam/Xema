#ifndef DEXFORCE_PROTOCAL_H
#define DEXFORCE_PROTOCAL_H

#include "camera_param.h"

#define DF_PORT 8080

#define DF_CMD_CONNECT 10000001
#define DF_CMD_DISCONNECT 10000002
#define DF_CMD_GET_BRIGHTNESS 10000003
#define DF_CMD_GET_DEPTH 10000004
#define DF_CMD_GET_POINTCLOUD 10000005
#define DF_CMD_GET_CONFIDENCE 10000006
#define DF_CMD_GET_RAW 10000007
#define DF_CMD_GET_TEMPERATURE 10000008
#define DF_CMD_SET_CAMERA_PARAMETERS 10000009
#define DF_CMD_GET_CAMERA_PARAMETERS 10000010
#define DF_CMD_SET_PATTERN_TABLE 10000011
#define DF_CMD_GET_PATTERN_TABLE 10000012
#define DF_CMD_GET_RAW_TEST 10000013
#define DF_CMD_ENABLE_CHECKER_BOARD 10000014
#define DF_CMD_DISABLE_CHECKER_BOARD 10000015
#define DF_CMD_LOAD_PATTERN_DATA 10000016
#define DF_CMD_PROGRAM_PATTERN_DATA 10000017
#define DF_CMD_GET_NETWORK_BANDWIDTH 10000018
#define DF_CMD_GET_FIRMWARE_VERSION 10000019
#define DF_CMD_SET_CAMERA_LOOKTABLE 10000020

#define DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS 10000109
#define DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS 10000110
#define DF_CMD_GET_STANDARD_PLANE_PARAM 10000111

#define DF_CMD_GET_RAW_01 11000001
#define DF_CMD_GET_RAW_02 11000002
#define DF_CMD_GET_RAW_03 11000003
#define DF_CMD_GET_RAW_04 11000004


#define DF_CMD_GET_FRAME_01 12000001
#define DF_CMD_GET_FRAME_02 12000002
#define DF_CMD_GET_FRAME_03 12000003
#define DF_CMD_GET_FRAME_04 12000004

#define DF_CMD_GET_REPETITION_FRAME_03 12000203

#define DF_CMD_GET_FRAME_HDR 12000103


#define DF_CMD_GET_PARAM_LED_CURRENT 13100000
#define DF_CMD_GET_PARAM_HDR 13100001
#define DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME 13100002
#define DF_CMD_GET_PARAM_CAMERA_GAIN 13100003
#define DF_CMD_GET_PARAM_EXTERNAL_PARAM_FLAG 13100004
#define DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM 13100005
#define DF_CMD_GET_PARAM_STANDARD_PLANE 13100006
#define DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS 13100007
 

#define DF_CMD_SET_PARAM_LED_CURRENT 13200000
#define DF_CMD_SET_PARAM_HDR 13200001
#define DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME 13200002
#define DF_CMD_SET_PARAM_CAMERA_GAIN 13200003
#define DF_CMD_SET_PARAM_EXTERNAL_PARAM_FLAG 13200004
#define DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM 13200005
#define DF_CMD_SET_PARAM_STANDARD_PLANE 13200006
#define DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS 13200007
#define DF_CMD_SET_PARAM_OFFSET 13200008




#define DF_CMD_OK 20000001
#define DF_CMD_REJECT 20000002
#define DF_CMD_UNKNOWN 20000003

#define DF_CMD_HEARTBEAT 30000001

#define DF_SUCCESS 0
#define DF_FAILED 1
#define DF_UNKNOWN 2


#endif
