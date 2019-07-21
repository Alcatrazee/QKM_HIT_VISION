#include <common_fcn.h>

// global variables
uchar conveyor_color = 'W';

void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
	std::string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (std::string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}

// crack input posture into float matrix
Mat str2mat(char str[255]) {
	Mat temp;
	std::vector<std::string> stemp;

	SplitString(str, stemp, ",");

	for (int i = 0; i < stemp.size(); i++) {
		temp.push_back(atof(stemp[i].data()));
	}
	float angle = temp.at<double>(2)*3.14159 / 180;
	Mat output_mat = (cv::Mat_<float>(3, 3) << cos(angle), -sin(angle), (float)temp.at<double>(0), sin(angle), cos(angle), (float)temp.at<double>(1), 0, 0, 1);
	return output_mat;
}

// you know it
float rad2deg(float rad) {
	return rad * 180 / pi;
}

float deg2rad(float deg) {
	return deg * pi / 180;
}

// just for convenience
void socket_send(SOCKET sock, const char * message) {
	send(sock, message, strlen(message) + sizeof(char), NULL);
}

// transform a matrix into string
void mat2str(Mat posture, std::string &Send) {
	// outer loop: rows
	
	for (int i = 0; i < posture.rows; i++) {
		Send.push_back('[');
		// inner loop: columns
		for (int j = 0; j < posture.cols; j++) {
			std::string update_char;
			update_char = std::to_string(posture.at<float>(i, j));
			// inner of inner loop : each char.
			for (int k = 0; k < update_char.size(); k++) {
				Send.push_back(update_char[k]);
			}
			if (j != posture.cols - 1) {
				Send.push_back(',');
			}
			else {
				Send.push_back(']');
			}
		}
		Send.push_back('\n');
	}
}