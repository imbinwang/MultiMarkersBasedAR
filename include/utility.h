/*****************************************************************************
*		includes some utility functions
*****************************************************************************/

#pragma once
#ifndef _UTILITY_
#define _UTILITY_

#include "CameraCalibration.h"
#include "GeometryTypes.h"
#include "GLM.h"
#include <vector>
#include <opencv2/opencv.hpp>

float perimeter(const std::vector<cv::Point2f> &a);

bool isInto(cv::Mat &contour, std::vector<cv::Point2f> &b);


void projectPoints(const std::vector<cv::Point3f> &points3d, const CameraCalibration &camCalib,
	const Transformation &trans, std::vector<cv::Point2f> &points2d);

void calculateObjTransformation(const Transformation &markerTransformation, 
	float *obj2markerTranslation, Transformation &objTransformation);

void drawCoordinate(const CameraCalibration &camCalib,
	const Transformation &trans, cv::Mat &img);

void drawObject( const CameraCalibration &camCalib,const Transformation &trans,
	GLMmodel *objModel, cv::Mat &img);

#endif
