#ifndef __COMMON_FCN__H__
#define __COMMON_FCN__H__
#pragma once

#include <iostream>
#include <stdio.h>
#include <winsock2.h>							//包含socket的头文件
#include <ws2tcpip.h> 
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cvUeye.h>
#include <vector>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <ctime>

#pragma comment (lib, "ws2_32.lib")				//加载 ws2_32.dll

using namespace cv;

#define pi 3.14159f

#define Fixed_cam_above 0
#define Cam_with_conveyor 1
#define Eye_in_hand 2
#define NOT_SELECTED 3
#define QUIT 4

// material definition
#define polygon_box 0
#define glass_cover 1

#define debug_mode			// program can't run without this definition

//////////////////////////////////		switches		/////////////////////////////////////////

// display command window or not
#define show_cmd					

#define use_SDK

// img preprocess switch 
#ifdef white_belt
#define invert_BI_polygon				// invert polygon binary image 
#endif		
//#define invert_BI_rec			
//#define use_background_diff			// use background differiential, if defined, you need to take away all items on the conveyor first,then follow the instructor

/////////////////////////////////////////////////////////////////////////////////////////////////

void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c);
Mat str2mat(char str[255]);
float rad2deg(float rad);
float deg2rad(float deg);
void socket_send(SOCKET sock, const char * message);
void mat2str(Mat posture, std::string &Send);
#endif 