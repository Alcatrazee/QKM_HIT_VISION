#include <common_fcn.h>
#include <calibration_fcn.h>
#include <polygon_detection.h>

#pragma comment (lib, "ws2_32.lib")				//╪сть ws2_32.dll

extern uchar conveyor_color;

// calculate RMS and mean of error 
// formular : sqrt((E1^2+E2^2+E3^2+...En^2)/n)		RMS
// formular : (E1+...En)/n						    mean
// E is the difference between actual position and estimated position(both in robot's frame)
// created in 2019-6-26
// last update: 2019-6-26
float RMS_ERROR(Mat T, Mat true_coordinate, Mat pix_coordinate) {
	Mat estimated_act_pos = T*pix_coordinate.t();											// calculate actual pos estimation
	Mat error_mat = estimated_act_pos - true_coordinate.t();
	Mat tempx, tempy;
	float RMS_ERROR_X, RMS_ERROR_Y, RMS_ERROR;
	pow(error_mat.rowRange(0, 1), 2, tempx);
	RMS_ERROR_X = sqrt(sum(tempx)[0] / tempx.cols);

	pow(error_mat.rowRange(1, 2), 2, tempy);
	RMS_ERROR_Y = sqrt(sum(tempy)[0] / tempy.cols);

	RMS_ERROR = sqrt((RMS_ERROR_X*RMS_ERROR_X + RMS_ERROR_Y*RMS_ERROR_Y) / 2);

	std::cout << "mean error:" << (mean(error_mat.rowRange(0, 1))[0] + mean(error_mat.rowRange(1, 2))[1]) / 2 << std::endl;
	std::cout << "RMS error:" << RMS_ERROR << std::endl;

	return RMS_ERROR;
}

// find out circles and circle them
void detect_circles(Mat input_img, std::vector<Vec3f> &circles,uchar robot_model) {
	Mat gray;
#ifndef use_SDK
	//cvtColor(input_img, gray, CV_BGR2GRAY);
#endif
	//GaussianBlur(input_img, input_img, Size(5, 5), 3);
	Mat copy_input = input_img.clone();
	// set roi for fixed cam
	if (robot_model == Fixed_cam_above) {
		// get ROI
		for (int rows = 0; rows < input_img.rows; rows++) {
			for (int cols = 0; cols < 275; cols++) {
				copy_input.at<uchar>(rows, cols) = 0;
			}
		}
	}
	// find out all circles from the image
	try {
		if (robot_model == Fixed_cam_above || robot_model == Cam_with_conveyor)
			HoughCircles(copy_input, circles, CV_HOUGH_GRADIENT, 1, 25, 150, 50, 20, 50);
		else
			HoughCircles(copy_input, circles, CV_HOUGH_GRADIENT, 1,25,120,50,10,130);
	}
	catch(cv::Exception d){
		std::cout << circles.size() << "No circles detected.Please tune parameter of houghcircles function or camera parameters" << std::endl;
		imshow("out_img", input_img);
		return ;
	}
	// convert grayscale image to BGR image.
	cvtColor(input_img, input_img, CV_GRAY2BGR);
	for (size_t i = 0; i < circles.size(); ++i)
	{
		// write coordinates in the image
		String coordinate = "(";
		coordinate += std::to_string((int)circles[i][0]);
		coordinate += ",";
		coordinate += std::to_string((int)(circles[i][1]));
		coordinate += ")";
		circle(input_img, Point(cvRound(circles[i][0]), cvRound(circles[i][1])), cvRound(circles[i][2]), Scalar(0, 255, 0), 1);		// draw circle around the circle
		circle(input_img, Point(cvRound(circles[i][0]), cvRound(circles[i][1])), 5, Scalar(0, 255, 0), 1);							// draw center of circle
		putText(input_img, coordinate, Point(circles[i][0] - 10, circles[i][1] + 50), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2);	// write position in image.
	}
	resize(input_img, input_img, Size(input_img.cols * 512 / input_img.rows, 512));
	imshow("out_img", input_img);
}

// calibration function
// format of T: translation matrix:					[A  B  E]
//													[C  D  F]
//													[0  0  1]
// get paramerters ABCDEF by least square method.
// format :    [x1,x2,x3...xn].T = [xp1,yp1,1;xp2,yp2,1...xpn,ypn,1]*[A,B,E].T     to get cofficient ABE
//			   [y1,y2,y3...yn].T = [xp1,yp1,1;xp2,yp2,1...xpn,ypn,1]*[C,D,F].T     to get cofficient CDF
#define rec_buffer_size 100
bool calibrate(Mat input_img, std::vector<Vec3f> circles, Mat &T, SOCKET sock)
{
	Mat copy_input = input_img.clone();

	// necessary matrices declaration 
	int size_of_mat = 0;
	if (circles.size() >= 9) {
		size_of_mat = 9;
	}
	else if (circles.size() < 3) {
		socket_send(sock, "ERROR: Number of targets are less than 3, at least 3 points are needed to calibrate a camera.Quiting.\n");
		return false;
	}
	else {
		size_of_mat = circles.size();
	}
	static Mat cof_mat = Mat::zeros(size_of_mat, 3, CV_32FC1);
	static Mat actual_x_vec = Mat::zeros(size_of_mat, 1, CV_32FC1);
	static Mat ABE;
	static Mat actual_y_vec = Mat::zeros(size_of_mat, 1, CV_32FC1);
	static Mat CDF;
	static Mat additional_vec = (Mat_<float>(3, 1) << 0, 0, 1);

#ifdef use_SDK
	// convert color space to BGR to write notification and circles
	cvtColor(copy_input, copy_input, CV_GRAY2BGR);
#endif
	// stage 1. draw all outer circles and center of them
	for (int i = 0; i < circles.size(); i++) {
		// write coordinates in the image
		String coordinate = "(";
		coordinate += std::to_string((int)circles[i][0]);
		coordinate += ",";
		coordinate += std::to_string((int)(circles[i][1]));
		coordinate += ")";
		circle(copy_input, Point(cvRound(circles[i][0]), cvRound(circles[i][1])), cvRound(circles[i][2]), Scalar(0, 255, 0), 1);
		circle(copy_input, Point(cvRound(circles[i][0]), cvRound(circles[i][1])), 5, Scalar(0, 255, 0), 2);
		putText(copy_input, std::to_string(i + 1), Point(circles[i][0] - 10, circles[i][1] + 10), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2);
		putText(copy_input, coordinate, Point(circles[i][0] - 10, circles[i][1] + 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2);
	}
	// show image
	resize(copy_input, copy_input, Size(copy_input.cols * 512 / copy_input.rows, 512));
	imshow("out_img", copy_input);
	waitKey(1);
	std::vector<std::string> actual_pos;
	while (true) {
		//  send notifications
		const char* notification = "Please input actual position each point in this format x1,y1,x2,y2,x3,y3,x4,y4...\ninput Q or q to stop calibration";
		send(sock, notification, strlen(notification) + sizeof(char), NULL);

		// receive message 
		char rec_buffer[rec_buffer_size] = { 0 };
		recv(sock, rec_buffer, rec_buffer_size, NULL);
		send(sock, rec_buffer, rec_buffer_size, NULL);
		
		if (!strcmp("Q", rec_buffer)|| !strcmp("q", rec_buffer)) {
			return false;
		}

		SplitString(rec_buffer, actual_pos, ",");
		if (actual_pos.size() == circles.size() * 2) {
			break;
		}
		else {
			actual_pos.clear();
		}
	}
	// input assignment
	int assign_counter = 0;
	for (int i = 0; i < circles.size()*2; i+=2) {
		cof_mat.at<float>(assign_counter, 0) = circles[assign_counter][0];
		cof_mat.at<float>(assign_counter, 1) = circles[assign_counter][1];
		cof_mat.at<float>(assign_counter, 2) = 1;
		// matrices assignment
		char  c_actual_pos[10] = { 0 };
		// assign x array
		for (int k = 0; k < actual_pos[i].size(); k++) {
			c_actual_pos[k] = actual_pos[i][k];
		}
		actual_x_vec.at<float>(assign_counter) = atof((char*)c_actual_pos);

		// clear array
		for (int k = 0; k < actual_pos[i].size(); k++) {
			c_actual_pos[k] = 0;
		}

		// assign y array
		for (int k = 0; k < actual_pos[i+1].size(); k++) {
			c_actual_pos[k] = actual_pos[i + 1][k];
		}
		actual_y_vec.at<float>(assign_counter) = atof((char*)c_actual_pos);
		assign_counter++;
	}
	//std::cout << cof_mat << std::endl;

	// transformation matrix calculation
	socket_send(sock, "Data is ready, calibrate now ? Y/N\n");
	// while true is for input assertion
	while (true) {
		char rec_buffer[rec_buffer_size] = { 0 };
		recv(sock, rec_buffer, rec_buffer_size, NULL);
		if (!strcmp("Y", rec_buffer)) {
			// solve linear equations to get ABCDEF
			solve(cof_mat, actual_x_vec, ABE, CV_SVD);
			solve(cof_mat, actual_y_vec, CDF, CV_SVD);

			vconcat(ABE.t(), CDF.t(), T);
			vconcat(T, additional_vec.t(), T);
			std::cout << T << std::endl;
			// calculate RMS
			Mat overall_pix_position, overall_actual_position;
			hconcat(actual_x_vec, actual_y_vec, overall_actual_position);
			hconcat(overall_actual_position, Mat::ones(Size(1,4),CV_32FC1), overall_actual_position);
			RMS_ERROR(T, overall_actual_position, cof_mat);
			return true;
		}
		else if (!strcmp("N", rec_buffer)) {
			socket_send(sock, "Going back.\n");
			return false;
		}
		else {
			socket_send(sock, "Please input Yes(Y) or No(N).\n");
		}
	}
}

// interface by socket(commandline only)
void calibration_wizzad(SOCKET servSock, SOCKET clntSock,uchar robot_model,uchar material) {
	// send notification
	socket_send(clntSock, "Calibration program initializing....Please wait.");
	// variables declearation
	std::vector<Vec3f> circles;
	Mat circles_img;
	Mat T;
	Mat cap_img;
	double exposure = 0;
	// send notification
	socket_send(clntSock, "\nOpening camera.");
	// camera instantiaion
#ifdef use_SDK
	int cam_ID = 0;
	if (robot_model == Fixed_cam_above&&conveyor_color == 'W')
		cam_ID = 1;
	else if (robot_model == Fixed_cam_above&&conveyor_color == 'G')
		cam_ID = 2;

	IDS_CAM cap(cam_ID, 15, 2);

	if (cap.cam_exist == false) {
		socket_send(clntSock, "camera not connected,please check the connection.");
		return;
	}
		
	if (robot_model == Fixed_cam_above) {
		cap.set_pix_clock(30);
		cap.set_exposure(5);
	}
	else if (robot_model == Cam_with_conveyor) {
		cap.set_pix_clock(30);
		cap.set_exposure(14.096);
	}
	else if (robot_model == Eye_in_hand) {
		cap.set_pix_clock(15);
		cap.set_exposure(100);
	}
	// read threshold parameters file ,but if it's the fisrt time to run ,program will
	// initialize the value automatically
	std::string file_n;
	int rec_thresh;
	int poly_thresh;
	if (robot_model == Fixed_cam_above) {
		if(conveyor_color=='W')
			file_n = "FCWC_params.xml";
		else if(conveyor_color=='G')
			file_n = "FCGC_params.xml";
	}
	else if (robot_model == Cam_with_conveyor) {
		file_n = "CV_params.xml";
	}
	else if (robot_model == Eye_in_hand&&material == polygon_box) {
		file_n = "EIH_params_normal.xml";
	}
	else if (robot_model == Eye_in_hand&&material == glass_cover) {
		file_n = "EIH_params_glass.xml";
	}
	// read threshold parameters from file
	std::ifstream ifile_thresh;
	ifile_thresh.open(file_n.data());
	if (ifile_thresh.is_open())
		get_params_from_file(ifile_thresh, rec_thresh, poly_thresh, exposure);
	cap.set_exposure(exposure);
#else 
	VideoCapture cap(1);
	double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, width * 2);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, height * 2);
#endif
	socket_send(clntSock, "\nCalibration program initialization done.");
	// program start
	while (true) {
		socket_send(clntSock, "\nWaiting for command.\nCommand: S :   Find circles\n         C :   Calibrate\n         T :   Test show\n         Q :   Back\n");
		char szBuffer[MAXBYTE] = { 0 };
		recv(clntSock, szBuffer, MAXBYTE, NULL);
#ifdef use_SDK
		cap_img = cap.grab_image();		// can be used directly,for the output of the cam is of grayscale.
#else 
		cap >> cap_img;
#endif
		if (!strcmp("T", szBuffer)) {
			socket_send(clntSock, "Command received:Watch mode. \nPress any key to quit");
			double max, min;
			cap.get_exposure_range(max, min);
			int i_exposure = exposure * 100;
			int threshold_value = 100;
			destroyAllWindows();
			namedWindow("test window");
			createTrackbar("exposure", "test window", &i_exposure, (int)(max * 100));
			createTrackbar("threshold", "test window",&threshold_value,255);
			while (true) {
#ifdef use_SDK
				Mat img = cap.grab_image();		// can be used directly,for the output of the cam is of grayscale.
#else 
				cap >> img;
				cvtColor(img,img,CV_BGR2GRAY);
#endif
				cap.set_exposure(double(getTrackbarPos("exposure","test window"))/100);
				Mat clone_img = img.clone();
				Mat show_img;
				threshold(img, img, getTrackbarPos("threshold", "test window"), 255, CV_THRESH_BINARY);
				vconcat(img, clone_img, show_img);
				resize(show_img, show_img, Size(show_img.cols * 768 / show_img.rows, 768));
				imshow("test window", show_img);
				if (waitKey(20) > 0)
					break;
			}
			destroyAllWindows();
			exposure = (double)i_exposure / 100;
			save_parameters(clntSock, robot_model, material, rec_thresh, poly_thresh, exposure);
		}else if (!strcmp("S", szBuffer)) {				// funtion: snap a image and find circles
			socket_send(clntSock, "Command received:Take picture and find out circles. \n");
#ifdef use_SDK
			cap_img = cap.grab_image();		// can be used directly,for the output of the cam is of grayscale.
#else 
			cap >> cap_img;
#endif
			detect_circles(cap_img, circles,robot_model);
			if (circles.size() == 0) {
				socket_send(clntSock, "No circle found,please tune parameters of function HoughCircle. ");
			}
		}// calibration starts from here
		else if (!strcmp("C", szBuffer)) {
			socket_send(clntSock, "Command received:Take picture and calibrate. \n");
			// image acqusition
#ifdef use_SDK
			cap_img = cap.grab_image();
#else 
			cap >> cap_img;
#endif
			// detect circles
			detect_circles(cap_img, circles,robot_model);
			// calibrate based on robot-cam mounting method
			// the only difference is what we are gonna write to a file.
			if (robot_model == Fixed_cam_above) {
				calibrate(cap_img, circles, T, clntSock);
				std::ofstream ofile;
				if(conveyor_color=='W')	// using while conveyor 
					ofile.open("FCAV_cal_data_white_conv.xml");
				else if(conveyor_color=='G')
					ofile.open("FCAV__cal_data_green_conv.xmll");

				ofile << T << std::endl;
				socket_send(clntSock, "write file finished");
				std::cout << "write file finished" << std::endl;
				ofile.close();
			}else if (robot_model == Cam_with_conveyor) {
				socket_send(clntSock, "Please turn on conveyor till calibration board is within robot's workspace,then input A to continue\n");
				recv(clntSock, szBuffer, MAXBYTE, NULL);
				calibrate(cap_img, circles, T, clntSock);
				// write file 
				std::ofstream ofile;
				ofile.open("CV_cal_data.xml");
				ofile << T << std::endl;
				socket_send(clntSock, "write file finished");
				std::cout << "write file finished" << std::endl;
				ofile.close();
			}else if (robot_model == Eye_in_hand) {
				// get posture in the moment of picture acquistion
				cv::Mat calibration_posture;
				// input assert
				while (true) {
					socket_send(clntSock, "Please input posture of camera-attached arm(manipulator) in form of X,Y,Roll.(only when the image plane is paralle to the container plane) \n");
					recv(clntSock, szBuffer, MAXBYTE, NULL);
					try {
						calibration_posture = str2mat(szBuffer);
					}
					catch (cv::Exception ex) {
						continue;
					}
					break;
				}
				// calibrate
				calibrate(cap_img, circles, T, clntSock);
				cv::Mat output_mat;
				vconcat(T, calibration_posture,output_mat);				// concate two matrix for output
				std::ofstream ofile;
				if(material==polygon_box)								// different material makes different file due to the height of platforms are different
					ofile.open("EIH_vision_cal_data_normal.xml");
				else if (material == glass_cover)
					ofile.open("EIH_vision_cal_data_glass.xml");
				ofile << output_mat << std::endl;
				socket_send(clntSock, "write file finished");
				ofile.close();
			}
			socket_send(clntSock, "calibration done.");
		}else if (!strcmp("Q", szBuffer)) {
			socket_send(clntSock, "Going back.");
			break;
		}
		waitKey(10);
	}
	// quit program.
	cap.release();
	destroyAllWindows();
}
