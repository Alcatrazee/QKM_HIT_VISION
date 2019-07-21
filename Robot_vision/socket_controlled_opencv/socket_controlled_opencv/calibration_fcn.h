#ifndef __CALIBRATION__H__
#define __CALIBRATION__H__
#pragma once
#include <common_fcn.h>

float RMS_ERROR(Mat T, Mat true_coordinate, Mat pix_coordinate);
void detect_circles(Mat input_img, std::vector<Vec3f> &circles, uchar robot_model);
bool calibrate(Mat input_img, std::vector<Vec3f> circles, Mat &T, SOCKET sock);
void calibration_wizzad(SOCKET servSock, SOCKET clntSock, uchar robot_model, uchar material);
#endif