#ifndef _CVUEYE__H__
#define _CVUEYE__H__

#pragma once

#include <uEye.h>
#include <opencv2\opencv.hpp>

using namespace cv;

class IDS_CAM {
public:
	IDS_CAM(int cam_id, int pixel_clock_rank, int resolution_scalar);			// contruct function, will open camera so that user doesn't need to open cam by him self
	void open_cam();									// no need to call it,because contruct function will do it.
	void set_pix_clock(UINT pix_clock);
	void release();										// release camera before exiting
	void IDS_CAM::flash_io_high();
	void IDS_CAM::flash_io_low();
	Mat grab_image();
	void set_exposure(double exposure_time);
	void get_exposure_range(double &max, double &min);
	~IDS_CAM();
	bool cam_exist;
private:
	HIDS m_hCam;
	// needed memory
	char* m_pcImgMem;
	int m_nImgId;
	int RetVal;
	int m_nDout;		// IO_FLASH_MODE_CONSTANT_HIGH  or  IO_FLASH_MODE_CONSTANT_LOW
	SENSORINFO MySensor;
	int _pixel_clock;			// 1:minimun  2:medium   3:maximum
	int res_scalar;		// 1:no resolution scaling 2:scale by 2  all choice:1,2,3,4,5,6,8,16
	double dblNewFactor;
	VOID * nparam;
};

#endif