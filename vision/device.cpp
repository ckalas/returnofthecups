#include "device.h"

// Do not call directly even in child
void MyFreenectDevice::VideoCallback(void* _rgb, uint32_t timestamp) {
	m_rgb_mutex.lock();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	rgbMat.data = rgb;
	m_new_rgb_frame = true;
	m_rgb_mutex.unlock();
}

// Do not call directly even in child
void MyFreenectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
	m_depth_mutex.lock();
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	depthMat.data = (uchar*) depth;
	m_new_depth_frame = true;
	m_depth_mutex.unlock();
}

bool MyFreenectDevice::getVideo(Mat& output) {
	m_rgb_mutex.lock();
	if(m_new_rgb_frame) {
		cv::cvtColor(rgbMat, output, CV_RGB2BGR);
		m_new_rgb_frame = false;
		m_rgb_mutex.unlock();
		return true;
	} else {
		m_rgb_mutex.unlock();
		return false;
	}
}

bool MyFreenectDevice::getDepth(Mat& output) {
		m_depth_mutex.lock();
		if(m_new_depth_frame) {
			depthMat.copyTo(output);
			m_new_depth_frame = false;
			m_depth_mutex.unlock();
			return true;
		} else {
			m_depth_mutex.unlock();
			return false;
		}
	}

void MyFreenectDevice::getCameraParams(Mat *mtx, Mat *dist, Mat *mtxInv) {
	// hard coded in values
	//double camera[] = {507.10760296, 0, 318.69253398, 0,512.9704932, 220.92629737, 0,0,1};
	double camera[] = {526.58629724, 0., 316.48890383, 0., 526.6042738, 259.81542393, 0., 0., 1.};
	//double distCoeffs[] = {0.14619644,-0.24519646, -0.02039532, 0.00233164, -0.00931531};
	double distCoeffs[] = { 2.54245396e-01, -1.01704633e+00, -6.74375784e-04, -1.17022929e-04, 1.80537097e+00};
	*mtx = Mat(3,3, CV_64F,camera);
	*dist = Mat(5,1,CV_64F, distCoeffs);
	*mtxInv = mtx->inv();
}