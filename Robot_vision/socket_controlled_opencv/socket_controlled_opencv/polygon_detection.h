#ifndef __POLYGON_DETECTION__H__
#define __POLYGON_DETECTION__H__
#pragma once

#include <common_fcn.h>

#define triangle 0
#define rectangle 1
#define pandegon 2
#define hexagon 3
#define circle_shape 4
#define invalid_shape 5

inline void get_params_from_file(std::ifstream &ifile_thresh, int &thresh_rec, int &thresh_poly, double &exposure);
Mat auto_gamma(Mat src_img, float target_mean);
void readT(std::ifstream &ifile, cv::Mat &T);
void readT(std::ifstream &ifile, cv::Mat &T, cv::Mat &photo_posture);
inline float get_distance(float x_coor1, float y_coor1, float x_coor2, float y_coor2);
int find_shortest_index(float length[3]);
int find_shortest_index_polygon(float length[5]);
void Get_real_world_coordinate(cv::Mat T, cv::Mat positions, cv::Mat &postures, Mat cal_pos, Mat current_pos, uchar robot_model);
float calculate_rect_area(cv::Point2f vertices[4], float &sum_length);
int Find_closest_rec(std::vector<RotatedRect > &contours_rec_real, float mcx, float mcy, std::vector<bool> &usable_rec_flag);
bool Find_fittest_box(cv::Mat &shape_vec, std::vector<RotatedRect> rec_vec, std::vector<bool> usable_rec_flag, int &index_of_rec);
void Polygon_detection(cv::Mat &srcImg, cv::Mat &output_mat, uchar robot_model);
void str2mat_param(std::ifstream &ifile, int &thresh_rec, int &thresh_poly, double &exposure);
Mat locate_glass_cover(Mat img);
double test_run(IDS_CAM &cap, uchar robot_model, double exposure, int &rec_thresh, int &poly_thresh);
void save_parameters(SOCKET clntSock, uchar robot_model, uchar material, int rec_thresh, int poly_thresh, double exposure);
void polygon_detect_wizzad(SOCKET servSock, SOCKET clntSock, uchar robot_model, uchar material);
#endif