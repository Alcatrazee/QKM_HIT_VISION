#include "uEye.h"
#include <iostream>
#include "opencv2\opencv.hpp"
#include "cvUeye.h"

using namespace cv;

void IDS_CAM::set_pix_clock(UINT pix_clock) {
	//set pixel clock
	UINT nRange[3];
	// Get pixel clock range
	INT nRet = is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)nRange, sizeof(nRange));
	if (nRet == IS_SUCCESS)
	{
		UINT nMin = nRange[0];
		UINT nMax = nRange[1];
		UINT nInc = nRange[2];
		UINT choice;
		//set max pixel clock
		/*switch (rank) {
		case 1:	nRet = is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_SET, (void*)&nMin, sizeof(nMin));	break;
		case 2:	nRet = is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_SET, (void*)&nInc, sizeof(nInc));	break;
		case 3:	nRet = is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_SET, (void*)&nMax, sizeof(nMax));	break;
		}*/
		if(pix_clock<=nMax&&pix_clock>=nMin)
			nRet = is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_SET, (void*)&pix_clock, sizeof(pix_clock));
		else
			std::cout << "pixel clock is out of range" << std::endl;
	}
	else
	{
		std::cout << "get pixel clock value faild" << std::endl;
	}
}

IDS_CAM::~IDS_CAM() {
	release();
}

void IDS_CAM::release() {
	is_FreeImageMem(m_hCam, m_pcImgMem, m_nImgId);
	is_ExitCamera(m_hCam);
}

IDS_CAM::IDS_CAM(int cam_id,int pixel_clock,int resolution_scalar) {
	m_hCam = (HIDS)cam_id;
	m_nDout = IO_FLASH_MODE_CONSTANT_HIGH;
	_pixel_clock = pixel_clock;
	res_scalar = resolution_scalar;
	open_cam();
}

void IDS_CAM::open_cam() {
	// open camera
	RetVal = is_InitCamera(&m_hCam, NULL);
	if (RetVal != IS_SUCCESS)
	{
		std::cout << "init camera faild" << std::endl;
		cam_exist = false;
	}

	set_pix_clock(_pixel_clock);

	// set display mode
	RetVal = is_SetDisplayMode(m_hCam, IS_SET_DM_DIB);
	if (RetVal != IS_SUCCESS)
	{
		std::cout << "set display mode faild" << std::endl;
	}

	// get sensor info and geometry info
	RetVal = is_GetSensorInfo(m_hCam, &MySensor);
	if (RetVal != IS_SUCCESS)
	{
		std::cout << "set sensor mode faild" << std::endl;
	}

	// setup color format 
	// use grayscale image to save computaion resource
	int nbpp;
	RetVal = is_SetColorMode(m_hCam, IS_CM_MONO8);
	if (RetVal != IS_SUCCESS)
	{
		std::cout << "set color mode BGR8 faild" << std::endl;
	}
	nbpp = 8;

	// reduce resolution
	int scalar = IS_BINNING_DISABLE;
	switch (res_scalar) {
	case 1: scalar = IS_BINNING_DISABLE; break;
	case 2: scalar = IS_BINNING_2X_HORIZONTAL | IS_BINNING_2X_VERTICAL; break;
	case 3: scalar = IS_BINNING_3X_HORIZONTAL| IS_BINNING_3X_VERTICAL; break;
	case 4: scalar = IS_BINNING_4X_HORIZONTAL| IS_BINNING_4X_VERTICAL; break;
	case 5: scalar = IS_BINNING_5X_HORIZONTAL| IS_BINNING_5X_VERTICAL; break;
	case 6: scalar = IS_BINNING_6X_HORIZONTAL| IS_BINNING_6X_VERTICAL; break;
	case 8: scalar = IS_BINNING_8X_HORIZONTAL| IS_BINNING_8X_VERTICAL; break;
	case 16: scalar = IS_BINNING_16X_HORIZONTAL| IS_BINNING_16X_VERTICAL; break;
	}
	is_SetBinning(m_hCam, scalar);

	// allocate image buffer
	RetVal = is_AllocImageMem(m_hCam, (int)MySensor.nMaxWidth / res_scalar,(int)MySensor.nMaxHeight / res_scalar, nbpp, &m_pcImgMem, &m_nImgId);
	if (RetVal != IS_SUCCESS)
	{
		std::cout << "allocate memory faild" << std::endl;
	}
	// set the image buffer for grabbing
	RetVal = is_SetImageMem(m_hCam, m_pcImgMem, m_nImgId);
	if (RetVal != IS_SUCCESS)
	{
		std::cout << "set memory faild" << std::endl;
	}
}

// set flash IO 
void IDS_CAM::flash_io_high() {
	m_nDout = IO_FLASH_MODE_CONSTANT_HIGH;
	is_IO(m_hCam, IS_IO_CMD_FLASH_SET_MODE, (void*)&m_nDout, sizeof(m_nDout));
}

void IDS_CAM::flash_io_low() {
	m_nDout = IO_FLASH_MODE_CONSTANT_LOW;
	is_IO(m_hCam, IS_IO_CMD_FLASH_SET_MODE, (void*)&m_nDout, sizeof(m_nDout));
}

Mat IDS_CAM::grab_image() {
	//相机采集图像数据转cv::Mat---注意第3个参数要和相机色彩格式（is_SetColorMode）对应，mono8对应CV_8UC1,BGR8对应CV_8UC3,注意OpenCV中默认的色彩排序是BGR
	int ret = is_FreezeVideo(m_hCam, IS_WAIT);
	return cv::Mat((int)MySensor.nMaxHeight / res_scalar, (int)MySensor.nMaxWidth / res_scalar, CV_8UC1, m_pcImgMem).clone();
}

void IDS_CAM::set_exposure(double exposure_time) {
	DOUBLE nRange[3];
	RetVal = is_Exposure(m_hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, nRange, sizeof(nRange));
	if (exposure_time >= nRange[0] && exposure_time <= nRange[1]) {
		nparam = &exposure_time;
		RetVal = is_Exposure(m_hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, nparam, sizeof(nparam));
		if (RetVal != IS_SUCCESS) {
			std::cout << "set camera exposure failed." << std::endl;
		}
	}
	else {
		std::cout << "exposure time out of range" << std::endl;
	}
}

void IDS_CAM::get_exposure_range(double &max,double &min) {
	DOUBLE nRange[3];
	RetVal = is_Exposure(m_hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, nRange, sizeof(nRange));
	max = nRange[1];
	min = nRange[0];
}