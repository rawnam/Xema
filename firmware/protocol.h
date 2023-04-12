#ifndef DEXFORCE_PROTOCAL_H
#define DEXFORCE_PROTOCAL_H

#include "../sdk/camera_status.h"
#include "camera_param.h"

#define DF_PORT 8080
// #define DFX_800 800
// #define DFX_1800 1800
#define DF_PROJECTOR_3010 3010
#define DF_PROJECTOR_4710 4710

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
#define DF_CMD_SET_AUTO_EXPOSURE_BASE_ROI 10000021
#define DF_CMD_SET_AUTO_EXPOSURE_BASE_BOARD 10000022
#define DF_CMD_SELF_TEST 10000023
#define DF_CMD_GET_PROJECTOR_TEMPERATURE 10000024
#define DF_CMD_SET_CAMERA_MINILOOKTABLE 10000025
#define DF_CMD_GET_FOCUSING_IMAGE 10000026
#define DF_CMD_GET_CAMERA_RESOLUTION 10000027
#define DF_CMD_GET_PRODUCT_INFO 10000028
#define DF_CMD_GET_FRAME_STATUS 10000030


#define DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS 10000109
#define DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS 10000110
#define DF_CMD_GET_STANDARD_PLANE_PARAM 10000111

#define DF_CMD_GET_RAW_01 11000001
#define DF_CMD_GET_RAW_02 11000002
#define DF_CMD_GET_RAW_03 11000003
#define DF_CMD_GET_RAW_04 11000004 
#define DF_CMD_GET_RAW_05 11000005 //xor gray code + phase shift
#define DF_CMD_GET_RAW_06 11000006 //minsw gray code + phase shift
#define DF_CMD_GET_RAW_08 11000008 //test code

#define DF_CMD_GET_RAW_01_REPETITION 11000201
#define DF_CMD_GET_RAW_02_REPETITION 11000202
#define DF_CMD_GET_RAW_03_REPETITION 11000203
#define DF_CMD_GET_RAW_04_REPETITION 11000204

#define DF_CMD_GET_PHASE_01 11100001
#define DF_CMD_GET_PHASE_02 11100002
#define DF_CMD_GET_PHASE_03 11100003
#define DF_CMD_GET_PHASE_04 11100004

#define DF_CMD_GET_PHASE_01_REPETITION 11000201
#define DF_CMD_GET_PHASE_02_REPETITION 11000202
#define DF_CMD_GET_PHASE_03_REPETITION 11000203
#define DF_CMD_GET_PHASE_04_REPETITION 11000204

#define DF_CMD_GET_FRAME_01 12000001
#define DF_CMD_GET_FRAME_02 12000002
#define DF_CMD_GET_FRAME_03 12000003
#define DF_CMD_GET_FRAME_04 12000004
#define DF_CMD_GET_FRAME_05 12000005 //xor gray code + phase shift
#define DF_CMD_GET_FRAME_06 12000006 //minsw gray code + phase shift

#define DF_CMD_GET_REPETITION_FRAME_03 12000203
#define DF_CMD_GET_REPETITION_FRAME_04 12000204
#define DF_CMD_GET_REPETITION_FRAME_05 12000205
#define DF_CMD_GET_REPETITION_FRAME_06 12000206

#define DF_CMD_GET_FRAME_HDR 12000103
#define DF_CMD_GET_FRAME_06_HDR 12000106



#define DF_CMD_GET_PARAM_LED_CURRENT 13100000
#define DF_CMD_GET_PARAM_HDR 13100001
#define DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME 13100002
#define DF_CMD_GET_PARAM_CAMERA_GAIN 13100003
#define DF_CMD_GET_PARAM_EXTERNAL_PARAM_FLAG 13100004
#define DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM 13100005
#define DF_CMD_GET_PARAM_STANDARD_PLANE 13100006
#define DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS 13100007
#define DF_CMD_GET_PARAM_OFFSET 13100008
#define DF_CMD_GET_PARAM_MIXED_HDR 13100009
#define DF_CMD_GET_PARAM_CAMERA_VERSION 13100010
#define DF_CMD_GET_PARAM_BILATERAL_FILTER 13100011
#define DF_CMD_GET_PARAM_CAMERA_CONFIDENCE 13100012
#define DF_CMD_GET_PARAM_RADIUS_FILTER 13100013
#define DF_CMD_GET_PARAM_REFLECT_FILTER 13100014
#define DF_CMD_GET_PARAM_FISHER_FILTER 13100015
#define DF_CMD_GET_PARAM_BOARD_MESSAGE 13100016
#define DF_CMD_GET_PARAM_PROJECTOR_VERSION 13100017
#define DF_CMD_GET_PARAM_DEPTH_FILTER 13100018


#define DF_CMD_SET_PARAM_LED_CURRENT 13200000
#define DF_CMD_SET_PARAM_HDR 13200001
#define DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME 13200002
#define DF_CMD_SET_PARAM_CAMERA_GAIN 13200003
#define DF_CMD_SET_PARAM_EXTERNAL_PARAM_FLAG 13200004
#define DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM 13200005
#define DF_CMD_SET_PARAM_STANDARD_PLANE 13200006 
#define DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS 13200007
#define DF_CMD_SET_PARAM_OFFSET 13200008
#define DF_CMD_SET_PARAM_MIXED_HDR 13200009
#define DF_CMD_SET_PARAM_CAMERA_VERSION 13200010
#define DF_CMD_SET_PARAM_BILATERAL_FILTER 13200011
#define DF_CMD_SET_PARAM_CAMERA_CONFIDENCE 13200012
#define DF_CMD_SET_PARAM_RADIUS_FILTER 13200013
#define DF_CMD_SET_PARAM_REFLECT_FILTER 13200014
#define DF_CMD_SET_PARAM_FISHER_FILTER 13200015
#define DF_CMD_SET_PARAM_BOARD_MESSAGE 13200016
#define DF_CMD_SET_PARAM_PROJECTOR_VERSION 13200017
#define DF_CMD_SET_PARAM_DEPTH_FILTER 13200018



#define DF_CMD_SET_INSPECT_MODEL_FIND_BOARD 13300000


#define DF_CMD_TEST_GET_FRAME_01 22000001
#define DF_CMD_TEST_GET_FRAME_02 22000002
#define DF_CMD_TEST_GET_FRAME_03 22000003
#define DF_CMD_TEST_GET_FRAME_04 22000004
#define DF_CMD_TEST_GET_FRAME_05 22000005


#define DF_CMD_OK 20000001
#define DF_CMD_REJECT 20000002
#define DF_CMD_UNKNOWN 20000003

#define DF_CMD_HEARTBEAT 30000001


#define INFO_SIZE (10 * 1024)
// ----------- update server -------------
#define UPDATE_PORT 9090
#define UPDATE_CLIENT 9091
#define UPDATE_SERVER 9092

#define DF_CMD_KILL_CAMERA_SERVER 14000001
#define DF_CMD_GET_CAMERA_SERVER 14000002
#define DF_CMD_CHMOD_CAMERA_SERVER 14000003
#define DF_CMD_REBOOT_DEVICE 14000004

#endif
