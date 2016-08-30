/*****************************************************************************
*	CameraCalibration class  
*		encapculates the intrinsic camera parameters anddistortion parameters
*****************************************************************************/

#pragma once
#ifndef _CAMERA_CALIBRATION_
#define _CAMERA_CALIBRATION_


////////////////////////////////////////////////////////////////////
// File includes:
#include <opencv2/opencv.hpp>


/**
* A camera calibration class that stores intrinsic matrix and distortion coefficients.
*/
class CameraCalibration
{
public:
	CameraCalibration();
	CameraCalibration(float fx, float fy, float cx, float cy);
	CameraCalibration(float fx, float fy, float cx, float cy, float distorsionCoeff[5]);


	const cv::Matx33f& getIntrinsic() const;
	const cv::Matx<float,5,1>&  getDistorsion() const;


	float& fx();
	float& fy();


	float& cx();
	float& cy();


	float fx() const;
	float fy() const;


	float cx() const;
	float cy() const;

private:
	cv::Matx33f     m_intrinsic;
	cv::Matx<float,5,1>     m_distortion;
};


#endif

