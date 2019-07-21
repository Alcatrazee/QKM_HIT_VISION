#include <common_fcn.h>
#include <polygon_detection.h>
#include <calibration_fcn.h>

clock_t start_t,end_t;
extern uchar conveyor_color;

#ifdef use_background_diff
Mat background;
#endif

// can't use this namespace,otherwise socket will not wait for conection.
// using namespace std;

inline void get_params_from_file(std::ifstream &ifile_thresh,int &thresh_rec, int &thresh_poly,double &exposure) {
	if (ifile_thresh.is_open())
		str2mat_param(ifile_thresh, thresh_rec, thresh_poly,exposure);
}

// function:tune gamma value so that mean of grayscale value stays with in range of 5 around target mean
// principle: Kp control law and integration
// what for: reduce affection of change of light
// input:Mat image, color space :grayscale
// target_mean: target mean of grayscale value 
// warning: might not be able to achieve all target value,however,through tunning alpha and beta
// where alpha means contrast value and beta means brightness gain.
//////////////////////////////////////////////////////////////////////////////////////////
//		overall formular is  : value_new = ( alpha * value_old + beta )^gamma			//
//////////////////////////////////////////////////////////////////////////////////////////
// created in 2019-6-27 11:58
Mat auto_gamma(Mat src_img, float target_mean) {
	int max_tunning = 100;
	double alpha = 1.4, beta = 0;
	Mat copy_img = src_img.clone();
	double gamma_ = 100;
	float error_reservation = 5;
	float Kp = 0.5;
	bool tune_succeed = false;
	Mat empty;
	while (max_tunning--) {
		Mat LookupTable(1, 256, CV_8U);
		uchar *p = LookupTable.ptr();
		for (int i = 0; i < 256; i++)
			p[i] = saturate_cast <uchar>(pow(i / 255.0, gamma_ / 100)*255.0);
		LUT(src_img, LookupTable, copy_img);
		Mat final_img = Mat::zeros(copy_img.rows, copy_img.cols, CV_8UC1);
		copy_img.convertTo(final_img, -1, alpha, beta);
		float mean_of_img = mean(copy_img)[0];
		float error = target_mean - mean_of_img;
		//imshow("output", copy_img);
		std::cout << mean_of_img << "\t" << error << std::endl;
		if (abs(error) > error_reservation) {
			gamma_ += -Kp*error;
			gamma_ = saturate_cast <uchar>(gamma_);
		}
		else {
			tune_succeed = true;
			break;
		}
	}
	if (tune_succeed) {
		//cout << "grayscale value converged." << endl;
		return copy_img;
	}
	else {
		//cout << "grayscale value does not converge(on time)." << endl;
		return empty;
	}
}

// read file to get matrix
void readT(std::ifstream &ifile, cv::Mat &T) {
	std::string data_T;
	char ctemp;
	int i = 0;
	while (ifile.get(ctemp)) {
		if (ctemp == ',' || ctemp == ']' || ctemp == ';')
			data_T.push_back('|');
		else if (ctemp == '[' || ctemp == '\n') {
			data_T.push_back(' ');
		}
		else {
			data_T.push_back(ctemp);
		}
		if (ctemp == ']')
			break;
		i++;
	}
	// split string
	std::istringstream iss(data_T);
	std::string stemp;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			getline(iss, stemp, '|');
			T.at<float>(i, j) = stof(stemp);
		}
	}
	// close file
	ifile.close();
}

void readT(std::ifstream &ifile, cv::Mat &T, cv::Mat &photo_posture) {
	std::string data_T;
	char ctemp;
	int i = 0;
	while (ifile.get(ctemp)) {
		if (ctemp == ',' || ctemp == ']' || ctemp == ';')
			data_T.push_back('|');
		else if (ctemp == '[' || ctemp == '\n') {
			data_T.push_back(' ');
		}
		else {
			data_T.push_back(ctemp);
		}
		if (ctemp == ']')
			break;
		i++;
	}
	// split string
	std::istringstream iss(data_T);
	std::string stemp;
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 3; j++) {
			getline(iss, stemp, '|');
			if (i<3)
				T.at<float>(i, j) = stof(stemp);
			else
				photo_posture.at<float>(i - 3, j) = stof(stemp);
		}
	}
	// close file
	ifile.close();
}

// calculate Eclidien distance
inline float get_distance(float x_coor1, float y_coor1, float x_coor2, float y_coor2) {
	float delta_X = x_coor1 - x_coor2;
	float delta_Y = y_coor1 - y_coor2;
	return sqrt(delta_X*delta_X + delta_Y*delta_Y);
}

// find smallest element's index
int find_shortest_index(float length[3]) {
	for (int i = 0; i < 3; i++) {
		if (length[i] == *std::min_element(length, length + 3))
			return i;
	}
	return -1;
}

// find smallest element's index
int find_shortest_index_polygon(float length[5]) {
	for (int i = 0; i < 5; i++) {
		if (length[i] == *std::min_element(length, length + 5))
			return i;
	}
	return -1;
}



// calculate real world coordinate with transformation matrix
void Get_real_world_coordinate(cv::Mat T, cv::Mat positions, cv::Mat &postures, Mat cal_pos, Mat current_pos,uchar robot_model) {
	// traverse every rows in posture matrix
	for (int i = 0; i < positions.rows; i++) {
		cv::Mat vec_temp;
		float actual_roll = 0;
		cv::Mat position_n = (cv::Mat_<float>(3, 1) << positions.at<float>(i, 2), positions.at<float>(i, 3), 1);
		cv::Mat direction_vector_end = (cv::Mat_<float>(3, 1) << positions.at<float>(i, 5), positions.at<float>(i, 6), 1);

		cv::Mat real_world_pos;									
		cv::Mat real_world_vec_end;

		// get real world posture
		real_world_pos = T*position_n;
		real_world_vec_end = T*direction_vector_end;			// end point of real world direction vector 
		
		actual_roll = 180 - rad2deg(atan2(real_world_vec_end.at<float>(1) - real_world_pos.at<float>(1), real_world_vec_end.at<float>(0) - real_world_pos.at<float>(0))); // revert the direction
		if (actual_roll > 180)
			actual_roll -= 360;
		else
			actual_roll += 360;
		 
		//	principle: T_true = T_R_2 * inv(T_R_1)
		//			   Posture_true = T_true * position
		if (cal_pos.rows > 0 && current_pos.rows > 0&& robot_model==Eye_in_hand) {
			Mat transformation_matrix = current_pos*cal_pos.inv();
			float angle_in_rad = deg2rad(actual_roll);
			Mat posture_matrix = (cv::Mat_<float>(3, 3) << cos(angle_in_rad), -sin(angle_in_rad), real_world_pos.at<float>(0), sin(angle_in_rad), cos(angle_in_rad), real_world_pos.at<float>(1), 0, 0, 1);
			Mat real_world_posture = transformation_matrix*posture_matrix;
			// assignment  [x,y,roll]
			real_world_pos.at<float>(0) = real_world_posture.at<float>(0, 2);
			real_world_pos.at<float>(1) = real_world_posture.at<float>(1, 2);
			actual_roll =180-rad2deg(atan2(-real_world_posture.at<float>(0, 1), real_world_posture.at<float>(0, 0)));
			if (actual_roll > 180)
				actual_roll -= 360;
			else
				actual_roll += 360;
			//std::cout << real_world_posture << std::endl;
		}
		
		// assignment
		positions.at<float>(i, 2) = real_world_pos.at<float>(0);
		positions.at<float>(i, 3) = real_world_pos.at<float>(1);
		positions.at<float>(i, 4) = actual_roll;
		
		// push data to tail of vector
		for (int j = 0; j < 5; j++) {
			vec_temp.push_back(positions.at<float>(i, j));
		}
		// push back vector to output matrix
		postures.push_back(vec_temp.t());
	}
	
}

// calculate area of a minAreaRectangle
float calculate_rect_area(cv::Point2f vertices[4], float &sum_length) {
	float length[3] = { 0 };
	float area = 0;
	for (int j = 0; j < 3; j++) {
		length[j] = get_distance(vertices[0].x, vertices[0].y, vertices[j + 1].x, vertices[j + 1].y);
	}
	std::sort(length, length + 3);
	area = length[0] * length[1];
	sum_length = (length[0] + length[1]) * 2;
	return area;
}

// find closest rectangle of a polygon 
int Find_closest_rec(std::vector<RotatedRect > &contours_rec_real, float mcx, float mcy, std::vector<bool> &usable_rec_flag) {
	std::vector<float>  distance_of_com;
	for (int i = 0; i < contours_rec_real.size(); i++) {
		distance_of_com.push_back(get_distance(contours_rec_real[i].center.x, contours_rec_real[i].center.y, mcx, mcy));
	}
	std::vector<float>::iterator smallest = std::min_element(std::begin(distance_of_com), std::end(distance_of_com));
	int index = std::distance(std::begin(distance_of_com), smallest);
	if (*smallest >= 100 || usable_rec_flag[index] == false) {
		return -1;
	}
	usable_rec_flag[index] = false;
	return index;
}

// find out rectangle to fit polygon base on distance,then set angle.
bool Find_fittest_box(cv::Mat &shape_vec, std::vector<RotatedRect> rec_vec, std::vector<bool> usable_rec_flag,int &index_of_rec) {
	if (rec_vec.size() != 0 && shape_vec.rows != 0) {
		// traverse every polygon
			index_of_rec = Find_closest_rec(rec_vec, shape_vec.at<float>(2), shape_vec.at<float>(3),usable_rec_flag);
			// go to next loop if we can't find any outer rectangles that is close enough to the polygon
			if (index_of_rec == -1)
				return false;
			// process differs from different shape.
			// actually only direction is required to be fixed.
			if (shape_vec.at<float>(1) == circle_shape) {
				shape_vec.at<float>(4) = (deg2rad(rec_vec[index_of_rec].angle));
			}
			else if (shape_vec.at<float>(1)<4) {
				std::vector<float> rec_angle;
				std::vector<float> poly_angle;
				// find out all possible angle of a rectangle(only two directions are prefered).
				for (int j = 0; j < 2; j++) {
					rec_angle.push_back(deg2rad(rec_vec[index_of_rec].angle) + j*pi);
					if (rec_angle[j] > pi) {
						rec_angle.at(j) -= 6.28;
					}
					else if (rec_angle[j] < -pi) {
						rec_angle.at(j) += 6.28;
					}
				}
				// find out all possible direction of a polygon
				for (int j = 0; j < shape_vec.at<float>(1) + 3; j++) {
					poly_angle.push_back(shape_vec.at<float>(4) + j * 2 * pi / (shape_vec.at<float>(1) + 3));
					if (poly_angle[j] > pi) {
						poly_angle.at(j) -= 6.28;
					}
					else if (poly_angle[j] < -pi) {
						poly_angle.at(j) += 6.28;
					}
				}
				// find out direciton that closest and assign it to the final output
				for (int j = 0; j < (shape_vec.at<float>( 1) + 3); j++) {
					for (int k = 0; k < rec_angle.size(); k++) {
						if (abs(poly_angle[j] - rec_angle[k]) <= deg2rad(5)) {
							// index shape x y dir
							shape_vec.at<float>(4) = poly_angle[j];
							j = 6;
							break;
						}
					}
				}
			}
			return true;
		}
	return false;
}

// usage: detect polygons and return their position in graph
// last update: 2-7-2019 
#define max_sides 30
#define max_length_rec 2000
#define min_length_rec 800
void Polygon_detection(cv::Mat &srcImg, cv::Mat &output_mat,uchar robot_model) {
	int max_area, min_area, max_area_rec, min_area_rec, middle_value = 82600;;
	// preprocess (ends in line 442)
	cv::Mat show_img = srcImg.clone();													// clone image
	float des_mean = 0;
	if (robot_model == Fixed_cam_above) {
		max_area_rec = middle_value + 10000;
		min_area_rec = middle_value - 10000;
		max_area = middle_value*0.1333*1.5;								// area ratio of  pandagon/outer_rectangle = 0.13339
		min_area = middle_value*0.1333*0.5;
		if(conveyor_color == 'W')
			des_mean = 110;
		else if (conveyor_color == 'G')
			des_mean = 86;

	}
	else if (robot_model == Cam_with_conveyor) {
		max_area = 6000;
		min_area = 4000;
		max_area_rec = 60000;
		min_area_rec = 45000;
		des_mean = 86;
	}
	else if (robot_model == Eye_in_hand) {
		max_area = 42500;
		min_area = 17500;
		des_mean = 100;	
	}
	/*Mat temp = auto_gamma(show_img, des_mean);
	if (temp.rows != 0) {
		srcImg = temp.clone();
	}*/

#ifdef use_SDK
	cv::Mat threshed_poly(srcImg.rows,srcImg.cols,CV_8UC1);
	cv::Mat threshed_rec(srcImg.rows, srcImg.cols, CV_8UC1);
#else
	cv::Mat threshed_poly;
	cv::Mat threshed_rec;
#endif

#ifdef use_background_diff
	// background differential
	srcImg = abs(srcImg - background);
	for (int rows = 0; rows < srcImg.rows; rows++) {
		for (int cols = 0; cols < srcImg.cols; cols++) {
			if (srcImg.at<uchar>(rows, cols) >= 8) {
				srcImg.at<uchar>(rows, cols) = show_img.at<uchar>(rows, cols);
			}
		}
	}
#endif	

#ifndef use_SDK
	cvtColor(srcImg, srcImg, CV_BGR2GRAY);
#endif

	// retrive ROI
	if (robot_model == Fixed_cam_above) {
		int cut_cols;
		cvtColor(show_img, show_img, CV_GRAY2BGR);
		if (conveyor_color == 'W')
			cut_cols = 240;
		else if (conveyor_color == 'G')
			cut_cols = 400;
		for (int rows = 0; rows < srcImg.rows; rows++) {
			for (int cols = 0; cols < cut_cols; cols++) {
				srcImg.at<char>(rows, cols) = 0;
			}
		}
	}
	
	// noise filter
	//GaussianBlur(srcImg, srcImg, cv::Size(3, 3), 5);
	if (robot_model != Eye_in_hand) {
		threshold(srcImg, threshed_rec, cv::getTrackbarPos("outer", "out_img"), 255, CV_THRESH_BINARY);
		threshold(srcImg, threshed_poly, cv::getTrackbarPos("inner", "out_img"), 255, CV_THRESH_BINARY);
	}
	else {
		threshold(srcImg, threshed_poly, cv::getTrackbarPos("thresh", "out_img"), 255, CV_THRESH_BINARY);
	}
	
	int morph_value = 5;
	cv::Mat element(morph_value, morph_value, CV_32FC1);
	cv::Mat morphed = Mat::zeros(threshed_poly.size(),CV_8UC1);
	
	// execute morph operation twice to get better binary image
	morphologyEx(threshed_poly, morphed, CV_MOP_OPEN, element);
	morphologyEx(morphed, morphed, CV_MOP_OPEN, element);

	// execute morph operation twice to get better 2-bit image
	morphologyEx(threshed_rec, threshed_rec, CV_MOP_OPEN, element);
	morphologyEx(threshed_rec, threshed_rec, CV_MOP_OPEN, element);

#ifdef invert_BI_polygon
	morphed = 255 - morphed;
#endif 

#ifdef invert_BI_rec
	threshed_rec = 255 - threshed_rec;
#endif
	// end of preprocess

	// find contours
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarcy;
	std::vector<std::vector<cv::Point> > contours_rec;
	std::vector<cv::Vec4i> hierarcy_rec;
	std::vector<bool> usable_box_flags;

	std::vector<RotatedRect> contours_rec_real, contours_rec_show;								// for polygon-box comparing
	findContours(morphed, contours, hierarcy, RETR_LIST, CHAIN_APPROX_SIMPLE);
	// Eye_in_hand doesn't need outer rectangle
	if (robot_model != Eye_in_hand) {
		findContours(threshed_rec, contours_rec, hierarcy_rec, RETR_LIST, CHAIN_APPROX_SIMPLE);
		// find out outer rectangles
		if (contours_rec.size() > 0) {
			for (int i = 0; i < contours_rec.size(); i++) {
				double area_rec = contourArea(contours_rec[i]);
				if (area_rec <= max_area_rec&&area_rec >= min_area_rec) {
					std::vector<std::vector<cv::Point> > contours_rec_poly(contours_rec.size());
					// get length of arc then calculate the epislon to arroxy a polygon
					double length_of_rec = arcLength(contours_rec[i], true);
					double epsilon = 0.01 * length_of_rec;
					approxPolyDP(cv::Mat(contours_rec[i]), contours_rec_poly[i], epsilon, true);

					// get minimum area fitted rectangle
					RotatedRect rec;
					Point2f vertices[4];
					rec = minAreaRect(contours_rec_poly[i]);

					cv::Point2f length_width = rec.size;
					float area = length_width.x*length_width.y;
					if (abs(area_rec - area) / area_rec >= 0.1)
						continue;

					// get angle of rectangle by figuring out the direction vector of the shortest side 
					rec.points(vertices);
					float dir = 0;
					float length[3] = { 0 };
					for (int j = 0; j < 3; j++) {
						length[j] = get_distance(vertices[0].x, vertices[0].y, vertices[j + 1].x, vertices[j + 1].y);
					}
					int shortest_index = find_shortest_index(length);	// find shortest edge
					Point dir_vec;
					// direction vector
					dir_vec.x = vertices[shortest_index + 1].x - vertices[0].x;
					dir_vec.y = vertices[shortest_index + 1].y - vertices[0].y;
					dir = rad2deg(atan2(dir_vec.y, dir_vec.x) + pi / 2);

					contours_rec_show.push_back(rec);

					// assign direction
					rec.angle = dir;

					// draw a circle in the center of the rectangle
					circle(show_img, rec.center, 5, Scalar(0, 255, 0), 2);

					contours_rec_real.push_back(rec);
					usable_box_flags.push_back(true);
				}
			}
		}
	}

	// process small contours
	const cv::String polygon_name[5] = { "triangle","rectangle","pandegon" ,"hexagon","circle" };
	if (contours.size() != 0) {
		// declare some variables needed
		std::vector<std::vector<float> > contours_poly_real;				// contours to fit the rectangle with each polygon
		int show_msgs_index = 0;											// index of objects
																			
		// geometric charactoristic variables of polygons
		std::vector<cv::Moments>mu(contours.size());
		std::vector<std::vector<cv::Point> > contours_poly(contours.size());
		std::vector<cv::Point2f>mc(contours.size());
		// traverse every contour to get conors' coordinates.
		for (int i = 0; i < contours.size(); i++) {
			int shape_of_polygon = 0;
			double epsilon = 0.01 * arcLength(contours[i], true);
			approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 2, true);
			double area = 0;
			double arc_length = 0;
			arc_length = arcLength(contours_poly[i], true);					// get arc length
			area = contourArea(contours_poly[i], false);					// get area of contour

			// judge if the polygon's area is within range and limit number of sides
			if (area >= max_area || area <= min_area || contours_poly[i].size() >= max_sides)
				continue;

			// calculate moments of a contour
			mu[i] = moments(contours_poly[i], false);
			mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);													// center of mass

			float radius = 0;																									// radius of enclosing circle
			minEnclosingCircle(contours_poly[i], cv::Point2f(mc[i].x, mc[i].y), radius);										// get minimum enclosing circle to the polygon
			//circle(show_img, Point(mc[i].x, mc[i].y), radius, Scalar(0, 255, 0), 2);
			if (mc[i].x + radius >= srcImg.cols || mc[i].x - radius <= 0 || mc[i].y + radius >= srcImg.rows || mc[i].y - radius <= 0)
				continue;

			//circle(show_img, Point(mc[i].x, mc[i].y), radius, Scalar(0, 255, 0), 2);		// draw the enclosing circle

			// start polygon identication
			// this program combined the expected area and length and number of sides of each true polygon to improve the accuracy
			float diff_area[5] = { 0 };							// error of each contours' area to each true polygon  
			float diff_len[5] = { 0 };							// error of each contours' length to each true polygon  
			float error_ratio_area[5] = { 0 };					// normalized error ratio
			float error_ratio_len[5] = { 0 };					// normalized error ratio
			float error_ratio_sum[5] = { 0 };					// sum of normalized error ratio
			float cof_ratio[3] = { 0.475f,0.475f,0.05f };		// cofficient for each ratio
			float length_of_rec = 0;							// length of minAreaRect
			float error_ratio_sides[5] = { 0,0,0,0,0 };			// error of ratio of sides
			float num_of_sides[5] = { 3,4,5,6,max_sides };		// true number of sides
																// traverse each true polygon to get the error ratio
			for (int k = 0; k < 5; k++) {
				if (k == 1) {
					// special case : rectangle
					// use minimum area rectangle to fit the polygon and calculate the error of length and area
					cv::RotatedRect r = minAreaRect(contours_poly[i]);
					cv::Point2f vertices[4];
					r.points(vertices);
					diff_area[k] = abs(area - calculate_rect_area(vertices, length_of_rec));
					diff_len[k] = abs(arc_length - length_of_rec);
				}
				else if (k == 4) {
					// special case : circle
					// different formular
					diff_area[k] = abs(area - pi*radius*radius);
					diff_len[k] = abs(arc_length - 2 * pi*radius);
				}
				else {
					// normal case : triangle,rectangle,pandegon,hexagon 
					diff_area[k] = abs(area - 0.5*(k + 3)*radius*radius*sin(2 * pi / (k + 3)));
					diff_len[k] = abs(arc_length - radius*(sin(pi / (k + 3))) * 2 * (k + 3));
				}
				//calculate the error array
				error_ratio_area[k] = diff_area[k] / area;
				error_ratio_len[k] = diff_len[k] / arc_length;
				error_ratio_sides[k] = abs(contours_poly[i].size() / num_of_sides[k] - 1); // formular : |num_of_sides/true_num_of_sides -1|

				// calculate the final error ratio for being each polygon
				error_ratio_sum[k] = cof_ratio[0] * error_ratio_area[k] + cof_ratio[1] * error_ratio_len[k] + cof_ratio[2] * error_ratio_sides[k];
			}
			shape_of_polygon = find_shortest_index_polygon(error_ratio_sum);	// find index of smallest error element
			if (error_ratio_sum[shape_of_polygon] > 1) {						// error ratio limitation
				shape_of_polygon = invalid_shape;
			}

			// get position and orientation of polygon then draw the arror line pointing toward X axis of the object
			float dir = 0;
			if (shape_of_polygon == triangle || shape_of_polygon == pandegon || shape_of_polygon == hexagon || shape_of_polygon == circle_shape) {
				std::vector<cv::Point>dir_vec(contours.size());
				dir_vec[i].x = contours_poly[i][0].x - mc[i].x;
				dir_vec[i].y = contours_poly[i][0].y - mc[i].y;
				dir = atan2(dir_vec[i].y, dir_vec[i].x);
			}
			else if (shape_of_polygon == rectangle) {
				cv::RotatedRect r = minAreaRect(contours_poly[i]);
				dir = deg2rad(r.angle);
			}
			// specified for Eye_in_hand
			// choose angle that pointing front or rear 
			if (robot_model == Eye_in_hand) {
				cv::Mat row_vec = (cv::Mat_<float>(1, 7) << show_msgs_index, shape_of_polygon, mc[i].x, mc[i].y, dir, 1, 1);
				// direction correction
				std::vector<float> angle_of_screen;
				std::vector<float> angle_of_polygon;
				for (int j = 0; j < shape_of_polygon + 3; j++) {
					angle_of_polygon.push_back(dir + j * 2 * pi / (shape_of_polygon + 3));
					if (angle_of_polygon[j] > pi) {
						angle_of_polygon.at(j) -= 6.28;
					}
					else if (angle_of_polygon[j] < -pi) {
						angle_of_polygon.at(j) += 6.28;
					}
				}
				angle_of_screen.push_back(0);
				angle_of_screen.push_back(pi);
				angle_of_screen.push_back(-pi);
				// find out direciton that closest and assign it to the final output
				for (int j = 0; j < shape_of_polygon+3; j++) {
					for (int k = 0; k < angle_of_screen.size(); k++) {
						if (abs(angle_of_polygon[j] - angle_of_screen[k]) <= deg2rad(10)) {
							// index shape x y dir
							row_vec.at<float>(4) = angle_of_polygon[j];
							j = 6;
							break;
						}
					}
				}
				drawContours(show_img, contours_poly, i, cv::Scalar(0, 0, 255), 4,8);										// draw outline of each polygon
				putText(show_img, polygon_name[shape_of_polygon], cv::Point(mc[i].x, mc[i].y + 60), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 4);		// write shape of polygon
				putText(show_img, std::to_string(show_msgs_index), cv::Point(mc[i].x, mc[i].y - 30), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 4,8,false);	// write number of polygon		

				Point actual_pix_axis;
				row_vec.at<float>(5) = actual_pix_axis.x = row_vec.at<float>(2) + cos(row_vec.at<float>(4)) * 100;
				row_vec.at<float>(6) = actual_pix_axis.y = row_vec.at<float>(3) + sin(row_vec.at<float>(4)) * 100;
				arrowedLine(show_img, Point(row_vec.at<float>(2), row_vec.at<float>(3)), actual_pix_axis, cv::Scalar(0, 255, 0), 5);
				output_mat.push_back(row_vec);
				show_msgs_index++;
			}else if (shape_of_polygon != invalid_shape) {
				int fittest_rec = 0;
				// if there's a closest box ,then pair them else next polygon 
				cv::Mat row_vec = (cv::Mat_<float>(1, 7) << show_msgs_index,shape_of_polygon, mc[i].x, mc[i].y, dir,1,1);		// declare a vector to fill in the output matrix
				if (Find_fittest_box(row_vec, contours_rec_real, usable_box_flags, fittest_rec)) {
					circle(show_img, mc[i], 6, cv::Scalar(0, 0, 0), -1, 8, 0);													// draw circle to indicate the mass of center of each polygon	
					if (shape_of_polygon == circle_shape)
						circle(show_img, Point(mc[i].x, mc[i].y), radius, Scalar(0, 0, 255), 2);
					else
						drawContours(show_img, contours_poly, i, cv::Scalar(0, 0, 255), 2, 8);										// draw outline of each polygon
					putText(show_img, polygon_name[shape_of_polygon], cv::Point(mc[i].x, mc[i].y + 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);		// write shape of polygon
					putText(show_img, std::to_string(show_msgs_index), mc[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);	// write number of polygon			
					
					Point2f vertices[4];
					contours_rec_show[fittest_rec].points(vertices);
					for (int j = 0; j<4; j++)
					{
						line(show_img, vertices[j], vertices[(j + 1) % 4], Scalar(255, 0, 0), 2, 8);	// draw outline of minimum area rectangle
					}

					Point actual_pix_axis;
					row_vec.at<float>(5) = actual_pix_axis.x = row_vec.at<float>(2) + cos(row_vec.at<float>(4)) * 50;
					row_vec.at<float>(6) = actual_pix_axis.y = row_vec.at<float>(3) + sin(row_vec.at<float>(4)) * 50;
					arrowedLine(show_img, Point(row_vec.at<float>(2), row_vec.at<float>(3)), actual_pix_axis, cv::Scalar(0, 255, 0), 2);
					
					output_mat.push_back(row_vec);
					show_msgs_index++;
				}
			}
		}
	}
	// show image
	Mat combined;
	cvtColor(morphed, morphed, CV_GRAY2BGR);													// binary graph -> BGR graph
	cvtColor(threshed_rec, threshed_rec, CV_GRAY2BGR);											// binary graph -> BGR graph
	double font_scalar = morphed.cols / 512 * 1.5;
	Point text_origin(0, morphed.cols / 512 * 35);
	int thickness = morphed.cols / 512 * 2;
	if(robot_model==Eye_in_hand){
		putText(morphed, "binary image", text_origin, FONT_HERSHEY_SIMPLEX, font_scalar, Scalar(255, 0, 0), thickness, 8, false);
		putText(show_img, "output image", text_origin, FONT_HERSHEY_SIMPLEX, font_scalar, Scalar(0, 255, 0), thickness, 8, false);
		vconcat(morphed, show_img, combined);
		resize(combined, combined, Size(combined.cols * 768 / combined.rows, 768));
	}
	else {
		putText(morphed, "find inner graph", text_origin, FONT_HERSHEY_SIMPLEX, font_scalar, Scalar(255, 0, 0), thickness, 8, false);
		putText(threshed_rec, "find outer graph", text_origin, FONT_HERSHEY_SIMPLEX, font_scalar, Scalar(0, 255, 0), thickness, 8, false);

		hconcat(morphed, threshed_rec, combined);

		Mat combined_origin = cv::Mat::zeros(srcImg.rows, srcImg.cols, CV_8UC3);
		cvtColor(srcImg, srcImg, CV_GRAY2BGR);
		putText(show_img, "output image", text_origin, FONT_HERSHEY_SIMPLEX, font_scalar, Scalar(0, 255, 0), thickness, 8, false);
		putText(srcImg, "diff(if switched on)", text_origin, FONT_HERSHEY_SIMPLEX, font_scalar, Scalar(0, 255, 0), thickness, 8, false);

		hconcat(srcImg, show_img, combined_origin);
		vconcat(combined, combined_origin, combined);
		resize(combined, combined, Size(combined.cols * 512 / combined.rows, 512));
	}
	imshow("out_img", combined);
}

void str2mat_param(std::ifstream &ifile,int &thresh_rec,int &thresh_poly,double &exposure) {
	std::string data_T;
	char ctemp;
	int i = 0;
	while (ifile.get(ctemp)) {
		if (ctemp == ',' || ctemp == ']' || ctemp == ';')
			data_T.push_back('|');
		else if (ctemp == '[' || ctemp == '\n') {
			data_T.push_back(' ');
		}
		else {
			data_T.push_back(ctemp);
		}
		if (ctemp == ']')
			break;
		i++;
	}
	// split string
	std::istringstream iss(data_T);
	std::string stemp;
	getline(iss, stemp, '|');
	thresh_rec = stoi(stemp);
	getline(iss, stemp, '|');
	thresh_poly = stoi(stemp);
	getline(iss, stemp, '|');
	exposure = stof(stemp);
	// close file
	ifile.close();
}

// if it's unable to detect any glass cover,tune maximun and minimun radius and 
// tune threshold value and area-radius constrains below
// found out ratio of area of outer rectangle and radius of the button is 8106.7
// How it works: 1. find out all circles(by comparing area with standard circle with same radius)
//				 2. find out all rectangles(with difference of area less than 0.8 times area)
//				 3. fit each circle with rectangles(by checking if the circle is inside a rectangle)
//				 4. fill in the matrix,direction is a vector pointing to the circle from mass center of the rectangle
#define min_outer_area 100000
#define max_outer_area 170000
#define max_radius 	25.97f			//  170000/8106.7+5	
#define min_radius 12.3354f			// 100000/8106.7-5	
// loacate all glass cover in pixel coordinate
Mat locate_glass_cover(Mat img) {
#ifndef use_SDK
	cvtColor(img, img, CV_BGR2GRAY);
#endif
	Mat show_img = img.clone();
	threshold(img, img, getTrackbarPos("thresh", "out_img"), 255, CV_THRESH_BINARY);
	std::vector<std::vector<cv::Point> > contours;									// contours for outer rectangle
	std::vector<std::vector<cv::Point> > contours_cir;								// contours for circles
	std::vector<cv::Vec4i> hierarcy, hierarcy_cir;									
	findContours(img, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(img, contours_cir, hierarcy_cir, CV_RETR_LIST, CHAIN_APPROX_SIMPLE);
	cvtColor(show_img, show_img, CV_GRAY2BGR);
	// 
	std::vector<cv::Point2f> centers;
	std::vector<cv::Point2f> usable_circles;
	std::vector<float> radius;
	//
	Mat output_vec;
	// find out all circles
	for (int i = 0; i < contours_cir.size(); i++) {
		Point2f center;
		float one_radius;
		minEnclosingCircle(contours_cir[i], center, one_radius);
		centers.push_back(center);
		radius.push_back(one_radius);
		float area = contourArea(contours_cir[i]);
		float ref_area = (radius[i] * radius[i] * pi);
		if (area / ref_area<0.8 || one_radius>max_radius|| one_radius<min_radius)
			continue;
		//circle(show_img, centers[i], radius[i], Scalar(100, 200, 0), 2);
		usable_circles.push_back(centers[i]);
	}
	// find outer contours
	int index = 0;
	std::vector<RotatedRect> rec;
	std::vector<cv::Point2f>mc(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		rec.push_back(minAreaRect(contours[i]));
		Point2f vertices[4];
		rec[i].points(vertices);
		float area = contourArea(contours[i]);
		if (area < min_outer_area)
			continue;
		mc[i] = rec[i].center;
		std::string num = std::to_string(index);
		// fit outer contour to circle
		for (int j = 0; j < usable_circles.size(); j++) {
			double in_or_out = pointPolygonTest(contours[i], Point2f(usable_circles[j].x, usable_circles[j].y), false);
			if (in_or_out == 1) {
				for (int k = 0; k<4; k++)
				{
					line(show_img, vertices[k], vertices[(k + 1) % 4], Scalar(255, 0, 0), 2, 8);	// draw outline of minimum area rectangle
				}
				Mat temp_matrix = (cv::Mat_<float>(1, 7) << index, 5, mc[i].x, mc[i].y, 0, usable_circles[j].x, usable_circles[j].y);
				output_vec.push_back(temp_matrix);
				arrowedLine(show_img, mc[i], usable_circles[j], Scalar(0, 255, 0), 3);
				//putText(show_img, num, usable_circles[j], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255),2);
				putText(show_img, num, mc[i], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255),2);
				index++;
				break;
			}
		}
	}
	putText(show_img, "output image", Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8, false);
	resize(show_img, show_img,Size(show_img.cols*show_img.rows/512,512));
	cvtColor(img, img, CV_GRAY2BGR);
	resize(img, img, Size(show_img.cols*show_img.rows / 512, 512));
	putText(img, "binary image", Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8, false);

	Mat final_show;
	vconcat(show_img, img, final_show);
	resize(final_show, final_show, Size(final_show.cols * 768 / final_show.rows, 768));
	imshow("out_img", final_show);
	return output_vec;
}

double test_run(IDS_CAM &cap,uchar robot_model,double exposure,int &rec_thresh,int &poly_thresh) {
	destroyAllWindows();
	namedWindow("out_img");
	double max_exposure, min_exposure;
	cap.get_exposure_range(max_exposure, min_exposure);
	int i_exposure = exposure * 100;
	if (robot_model == Fixed_cam_above || robot_model == Cam_with_conveyor) {
		createTrackbar("outer", "out_img", &rec_thresh, 255);
		createTrackbar("inner", "out_img", &poly_thresh, 255);
	}
	else if (robot_model == Eye_in_hand) {
		createTrackbar("thresh", "out_img", &rec_thresh, 255);
	}
	createTrackbar("exposure", "out_img", &i_exposure, (int)(max_exposure * 100));

	while (true) {
#ifdef use_SDK
		Mat img = cap.grab_image();
#else
		cap>>img;
		cvtColor(img, img, CV_BGR2GRAY);
#endif
		Mat show_img,copy_img = img.clone();
		cap.set_exposure((double)cv::getTrackbarPos("exposure", "out_img")/100);
		if (robot_model==Fixed_cam_above||robot_model==Cam_with_conveyor) {
			Mat threshed_rec, threshed_poly, concated_temp, morphed, morph_temp;
			threshold(img, threshed_rec, cv::getTrackbarPos("outer", "out_img"), 255, CV_THRESH_BINARY);
			threshold(img, threshed_poly, cv::getTrackbarPos("inner", "out_img"), 255, CV_THRESH_BINARY);

			int morph_value = 5;
			Mat element(morph_value, morph_value, CV_32FC1);

			// execute morph operation twice to get better binary image
			morphologyEx(threshed_poly, morphed, CV_MOP_OPEN, element);
			morphologyEx(morphed, morphed, CV_MOP_OPEN, element);

			// execute morph operation twice to get better 2-bit image
			morphologyEx(threshed_rec, threshed_rec, CV_MOP_OPEN, element);
			morphologyEx(threshed_rec, threshed_rec, CV_MOP_OPEN, element);

		#ifdef invert_BI_polygon
			morphed = 255 - morphed;
		#endif 

		#ifdef invert_BI_rec
			threshed_rec = 255 - threshed_rec;
		#endif

		#ifdef use_background_diff
			img = abs(img - background);
		#endif
			cvtColor(morphed, morphed, CV_GRAY2BGR);
			cvtColor(threshed_rec, threshed_rec, CV_GRAY2BGR);
			putText(morphed, "find inner graph", Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, 8, false);
			putText(threshed_rec, "find outer graph", Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8, false);
			Mat combined;
			hconcat(morphed, threshed_rec, combined);

			Mat combined_origin;
			cvtColor(img, img, CV_GRAY2BGR);
			cvtColor(copy_img, copy_img, CV_GRAY2BGR);
			putText(copy_img, "output image", Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8, false);
			putText(img, "diff(if switched on)", Point(0, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8, false);

			hconcat(img, copy_img, combined_origin);
			vconcat(combined, combined_origin, show_img);
			resize(show_img, show_img, Size(show_img.cols * 512 / show_img.rows, 512));
		}
		else if (robot_model == Eye_in_hand) {
			Mat clone_img = img.clone();
			threshold(img, img, getTrackbarPos("thresh", "out_img"), 255, CV_THRESH_BINARY);
			vconcat(img, clone_img, show_img);
			resize(show_img, show_img, Size(show_img.cols * 768 / show_img.rows, 768));
		}
		// show image
		imshow("out_img", show_img);
		if (waitKey(30) > 0)
			break;
	}
	destroyAllWindows();
	return double(i_exposure/100);
}

void save_parameters(SOCKET clntSock,uchar robot_model,uchar material,int rec_thresh,int poly_thresh,double exposure) {
	// parameter saving
	std::ofstream ofile;
	socket_send(clntSock, "saving parameters....");
	switch (robot_model) {
	case Fixed_cam_above:
		if(conveyor_color=='W')
			ofile.open("FCWC_params.xml");
		else if(conveyor_color == 'G')
			ofile.open("FCGC_params.xml");
		break;
	case Cam_with_conveyor:
		ofile.open("CV_params.xml");
		break;
	case Eye_in_hand:
		if (material == polygon_box)
			ofile.open("EIH_params_normal.xml");
		else if (material == glass_cover)
			ofile.open("EIH_params_glass.xml");
		break;
	}
	Mat thresh_mat = (Mat_<float>(1, 3) << rec_thresh, poly_thresh, exposure);
	ofile << thresh_mat << std::endl;
	ofile.close();
	socket_send(clntSock, "\nparameters saved...");
}

// wizzard
void polygon_detect_wizzad(SOCKET servSock, SOCKET clntSock, uchar robot_model,uchar material) {
	std::ifstream ifile;
	Mat cal_position = cv::Mat::zeros(3, 3, CV_32FC1), current_position = cv::Mat::zeros(3, 3, CV_32FC1);
	cv::Mat T = cv::Mat::zeros(3, 3, CV_32FC1);
	// read different file depends on different mounting method
	if (robot_model == Fixed_cam_above) {
		if (conveyor_color == 'W')
			ifile.open("FCAV_cal_data_white_conv.xml");
		else if (conveyor_color == 'G')
			ifile.open("FCAV__cal_data_green_conv.xml");

		if (ifile.is_open())
			readT(ifile, T);
		else {
			socket_send(clntSock, "robot vision not calibrated,please reboot this program and calibrate.");
			ifile.close();
			return;
		}
		ifile.close();
	}else
	if (robot_model == Cam_with_conveyor) {
		ifile.open("CV_cal_data.xml");
		if (ifile.is_open()) {
			readT(ifile, T);
		}else {
			socket_send(clntSock, "robot vision not calibrated,please reboot this program and calibrate.");
			ifile.close();
			return;
		}
		ifile.close();
	}
	else if (robot_model == Eye_in_hand) {
		if(material==polygon_box)
			ifile.open("EIH_vision_cal_data_normal.xml");
		else if(material == glass_cover)
			ifile.open("EIH_vision_cal_data_glass.xml");
		if (ifile.is_open())
			readT(ifile, T,cal_position);
		else {
			socket_send(clntSock, "robot vision not calibrated,please reboot this program and calibrate.");
			ifile.close();
			return;
		}
		ifile.close();
	}
	socket_send(clntSock, "Initializeing polygon detection program....");


	// initialize camera with SDK
	double exposure = 20;

	cv::Mat cap_img;
	int rec_thresh;
	int poly_thresh;
	
	// read threshold parameters file ,but if it's the fisrt time to run ,program will
	// initialize the value automatically
	std::string file_n;
	if (robot_model == Fixed_cam_above) {
		if (conveyor_color == 'W') {
			file_n = "FCWC_params.xml";
			rec_thresh = 22;
			poly_thresh = 190;
		}	
		else if (conveyor_color == 'G') {
			file_n = "FCGC_params.xml";
			rec_thresh = 185;
			poly_thresh = 95;
		}

	}
	else if (robot_model == Cam_with_conveyor) {
		file_n = "CV_params.xml";
		rec_thresh = 185;
		poly_thresh = 95;
	}
	else if (robot_model == Eye_in_hand&&material==polygon_box) {
		file_n = "EIH_params_normal.xml";
		rec_thresh = 60;
		poly_thresh = rec_thresh;
	}
	else if (robot_model == Eye_in_hand&&material == glass_cover) {
		file_n = "EIH_params_glass.xml";
		rec_thresh = 60;
		poly_thresh = rec_thresh;
	}
	// read threshold parameters from file
	std::ifstream ifile_thresh;
	ifile_thresh.open(file_n.data());
	if(ifile_thresh.is_open())
		get_params_from_file(ifile_thresh,rec_thresh, poly_thresh, exposure);

#ifdef use_SDK
	int cam_ID = 0;
	if (robot_model == Fixed_cam_above&&conveyor_color == 'W')
		cam_ID = 1;
	else if(robot_model == Fixed_cam_above&&conveyor_color == 'G')
		cam_ID = 2;

	IDS_CAM cap(cam_ID, 15, 2);

	if (cap.cam_exist == false) {
		socket_send(clntSock, "camera not connected,please check the connection.");
		cap.release();
		cv::destroyAllWindows();
		return;
	}

	if (robot_model == Fixed_cam_above) {
		cap.set_pix_clock(30);
	}
	else if (robot_model == Cam_with_conveyor) {
		cap.set_pix_clock(30);
	}
	else if (robot_model == Eye_in_hand) {
		cap.set_pix_clock(20);
	}
	cap.flash_io_high();
	cap.set_exposure(exposure);
	for (int i = 0; i < 25; i++) {
		cap.grab_image();
		waitKey(1);
	}
#else 
	VideoCapture cap(1);
	double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, width * 2);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, height * 2);
#endif
	

#ifdef use_background_diff
	char temp_buffer[MAXBYTE] = { 0 };
	socket_send(clntSock, "Please take away any item on the conveyor.Input anything to continue.\n Or input Q or q to quit");
	recv(clntSock, temp_buffer, MAXBYTE, NULL);
	if (!strcmp(temp_buffer,"Q")|| !strcmp(temp_buffer,"q")) {
		cap.release();
		cv::destroyAllWindows();
		return;
	}
#ifndef use_SDK
	cap >> background;
#else 
	background = cap.grab_image();
#endif
#endif
	socket_send(clntSock, "Program initialized.\nInput S to take a picture and process.\nInput T to tune parameters.\nInput F to save parameters.\nInput Q to go back");
	while (true) {
		char szBuffer[MAXBYTE] = { 0 };
		recv(clntSock, szBuffer, MAXBYTE, NULL);
		if (!strcmp("S", szBuffer)) {
			// create window with trackbars
			if (getWindowProperty("out_img", WND_PROP_VISIBLE) == -1) {
				cv::namedWindow("out_img");
				if (robot_model != Eye_in_hand) {
					cv::createTrackbar("outer", "out_img", &rec_thresh, 255);
					cv::createTrackbar("inner", "out_img", &poly_thresh, 255);
				}
				else {
					cv::createTrackbar("thresh", "out_img", &rec_thresh, 255);
				}
			}
			start_t = clock();
#ifdef use_SDK
		cap.flash_io_low();			// flash io set to low(two wires turn connected)
		cap_img = cap.grab_image();	// grab grayscale image 
		
		//waitKey(5);					// delay then disconnecte flash io
		cap.flash_io_high();
#else 
		cap >> cap_img;
#endif
			cv::Mat position;
			cv::Mat posture;
			// detect polygon and return postures and shapes in matrix format: [index shape x y roll dirx diry] row vector of this column vector 
			if (material == polygon_box)
				Polygon_detection(cap_img, position, robot_model);
			else if (material == glass_cover)
				position = locate_glass_cover(cap_img);

			cv::waitKey(1);
			// get current position and orientation to calculate posture
			bool decode_input_succ = false;										// flag to judge if we have decoded the input posture correctly
			if (robot_model == Eye_in_hand&&position.rows!=0) {
				while (true) {
					//socket_send(clntSock,"1000");
					socket_send(clntSock, "input current position in format like x,y,roll , input Q to abort.");
					recv(clntSock, szBuffer, MAXBYTE, NULL);
					if (!strcmp("Q", szBuffer)) {
						szBuffer[0] = 0;
						break;
					}
					try {
						current_position = str2mat(szBuffer);
					}
					catch (cv::Exception ex) {
						continue;
					}
					decode_input_succ = true;
					break;
				}
			}
			// convert pixel coordinate to real world coordinate,only when position is not empty
			if (position.rows != 0 && decode_input_succ==true&&robot_model==Eye_in_hand) {
				Get_real_world_coordinate(T, position, posture, cal_position, current_position, robot_model);
				std::string Send;
				mat2str(posture,Send);						// convert matrix to string format to send
				socket_send(clntSock, Send.data());
			}
			else if (position.rows != 0 && robot_model != Eye_in_hand) {
				Get_real_world_coordinate(T, position, posture, cal_position, current_position, robot_model);
				std::string Send;
				mat2str(posture, Send);						// convert matrix to string format to send
				socket_send(clntSock, Send.data());
			}else{
				socket_send(clntSock, "[No Item]");			// send no item when there's no item detected
			}
			end_t = clock();
			std::cout << "process time:"<<((double)(end_t - start_t) / CLOCKS_PER_SEC) *1000<<"ms"<< std::endl;		// print time consumsion per process
		}
		else if (!strcmp("T", szBuffer)) {
			socket_send(clntSock,"Tunning program,please tune exposure and threshold value for later use.\nPress any key to quit tunning mode");
			exposure = test_run(cap,robot_model,exposure,rec_thresh,poly_thresh);
			socket_send(clntSock, "Tunning program exited.");
			save_parameters(clntSock, robot_model, material, rec_thresh, poly_thresh, exposure);
			socket_send(clntSock, "\nProgram initialized.\nInput S to take a picture and process.\nInput T to tune parameters.\nInput F to save parameters.\nInput Q to go back");
		}
		else if (!strcmp("F", szBuffer)) {
			save_parameters(clntSock, robot_model, material, rec_thresh, poly_thresh, exposure);
			socket_send(clntSock, "\nProgram initialized.\nInput S to take a picture and process.\nInput T to tune parameters.\nInput F to save parameters.\nInput Q to go back");
		}
		else if (!strcmp("Q", szBuffer)|| !strcmp("q", szBuffer)) {		// quiting procedure
			save_parameters(clntSock, robot_model, material, rec_thresh, poly_thresh, exposure);
			socket_send(clntSock, "\nGoing back.");
			break;
		}
	}
	cap.release();
	cv::destroyAllWindows();
}
