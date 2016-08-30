#include "../include/utility.h"

float perimeter(const std::vector<cv::Point2f> &a)
{
	float sum=0, dx, dy;

	for (size_t i=0;i<a.size();i++)
	{
		size_t i2=(i+1) % a.size();

		dx = a[i].x - a[i2].x;
		dy = a[i].y - a[i2].y;

		sum += sqrt(dx*dx + dy*dy);
	}

	return sum;
}


bool isInto(cv::Mat &contour, std::vector<cv::Point2f> &b)
{
	for (size_t i=0;i<b.size();i++)
	{
		if (cv::pointPolygonTest( contour,b[i],false)>0) 
			return true;
	}
	return false;
}


void projectPoints(const std::vector<cv::Point3f> &points3d, const CameraCalibration &camCalib,
	const Transformation &trans, std::vector<cv::Point2f> &points2d)
{
	cv::Mat intrinsic = cv::Mat::eye(3,3,CV_32FC1);
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			intrinsic.at<float>(i,j) = camCalib.getIntrinsic()(i,j);
		}
	}

	cv::Mat extrinsic(3,4,CV_32FC1);
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			extrinsic.at<float>(i,j) = trans.r().mat[i][j];
		}

		extrinsic.at<float>(i,3) = trans.t().data[i];
	}

	int pNum = (int)points3d.size();
	cv::Mat ps3d(4, pNum, CV_32FC1);
	for(int j=0; j<pNum; ++j)
	{
		ps3d.at<float>(0,j) = points3d[j].x;
		ps3d.at<float>(1,j) = points3d[j].y;
		ps3d.at<float>(2,j) = points3d[j].z;
		ps3d.at<float>(3,j) = 1;
	}

	cv::Mat ps2d(3, pNum, CV_32FC1);
	ps2d = intrinsic*extrinsic*ps3d;
	for(int j=0; j<pNum; ++j)
	{
		float u = ps2d.at<float>(0,j)/ps2d.at<float>(2,j);
		float v = ps2d.at<float>(1,j)/ps2d.at<float>(2,j);
		points2d.push_back(cv::Point2f(u,v));
	}
}

void calculateObjTransformation(const Transformation &markerTransformation, 
	float *obj2markerTranslation, Transformation &objTransformation)
{
	// just consider one marker here
	const Matrix33 &rMat = markerTransformation.r();
	const Vector3 &tVec = markerTransformation.t();

	Matrix33 objRMat = Matrix33::identity();
	Vector3 objTVec = Vector3::zero();

	for( int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			objRMat.mat[i][j] = rMat.mat[i][j];
		}
	}

	//the translation =  rMat*obj2markerTranslation + tVec
	for(int i=0; i<3; ++i)
	{
		float val = 0;
		for(int j=0; j<3; ++j)
		{
			val += (rMat.mat[i][j] * obj2markerTranslation[j]);
		}

		objTVec.data[i] = val + tVec.data[i];
	}

	objTransformation = Transformation(objRMat, objTVec);
}

void drawCoordinate(const CameraCalibration &camCalib,
	const Transformation &trans, cv::Mat &img)
{
	// camera parameters and distortion coefficients
	const cv::Matx33f camIntrinsic = camCalib.getIntrinsic();
	const cv::Matx<float,5,1> camDistortion = camCalib.getDistorsion();

	// coordiantes points in 3d
	std::vector<cv::Point3f> coor_points_3d;
	coor_points_3d.push_back(cv::Point3f(0.f,0.f,0.f));
	coor_points_3d.push_back(cv::Point3f(4.f,0.f,0.f));
	coor_points_3d.push_back(cv::Point3f(0.f,4.f,0.f));
	coor_points_3d.push_back(cv::Point3f(0.f,0.f,4.f));

	// project above points to 2d
	std::vector<cv::Point2f> coor_points_2d;

	coor_points_2d.clear();

	cv::Matx33f rotMat(trans.r().data);
	std::vector<float> rVec;
	cv::Rodrigues(rotMat, rVec);

	std::vector<float> tVec(trans.t().data, trans.t().data+3);

	coor_points_2d.clear();
	cv::projectPoints(coor_points_3d, rVec, tVec, camIntrinsic, camDistortion, coor_points_2d);

	cv::line(img, cv::Point((int)coor_points_2d[0].x, (int)coor_points_2d[0].y),
		cv::Point((int)coor_points_2d[1].x, (int)coor_points_2d[1].y), CV_RGB(255,0,0),2);
	cv::line(img, cv::Point((int)coor_points_2d[1].x, (int)coor_points_2d[1].y),
		cv::Point((int)coor_points_2d[1].x, (int)coor_points_2d[1].y), CV_RGB(255,0,0),5);

	cv::line(img, cv::Point((int)coor_points_2d[0].x, (int)coor_points_2d[0].y),
		cv::Point((int)coor_points_2d[2].x, (int)coor_points_2d[2].y), CV_RGB(0,255,0),2);
	cv::line(img, cv::Point((int)coor_points_2d[2].x, (int)coor_points_2d[2].y),
		cv::Point((int)coor_points_2d[2].x, (int)coor_points_2d[2].y), CV_RGB(0,255,0),5);

	cv::line(img, cv::Point((int)coor_points_2d[0].x, (int)coor_points_2d[0].y),
		cv::Point((int)coor_points_2d[3].x, (int)coor_points_2d[3].y), CV_RGB(0,0,255),2);
	cv::line(img, cv::Point((int)coor_points_2d[3].x, (int)coor_points_2d[3].y),
		cv::Point((int)coor_points_2d[3].x, (int)coor_points_2d[3].y), CV_RGB(0,0,255),5);
}

void drawObject( const CameraCalibration &camCalib,const Transformation &trans,
	GLMmodel *objModel, cv::Mat &img)
{
	// camera parameters and distortion coefficients
	const cv::Matx33f camIntrinsic = camCalib.getIntrinsic();
	const cv::Matx<float,5,1> camDistortion = camCalib.getDistorsion();

	// get obj vertices data
	unsigned int verticesNum = objModel->numvertices;
	GLfloat *vertices = objModel->vertices;
	std::vector<cv::Point3f> points3d(verticesNum);
	for(size_t i=1; i<=verticesNum; ++i)
	{
		points3d[i-1] = cv::Point3f(vertices[3 * i + 0],
									vertices[3 * i + 1],
									vertices[3 * i + 2]);
	}

	//get obj edges date
	unsigned int edgesNum = objModel->numLines;
	GLMLine *edges = objModel->lines;

	// project above points to 2d
	std::vector<cv::Point2f> points2d;
	
	cv::Matx33f rotMat(trans.r().data);
	std::vector<float> rVec;
	cv::Rodrigues(rotMat, rVec);

	std::vector<float> tVec(trans.t().data, trans.t().data+3);

	points2d.clear();
	cv::projectPoints(points3d, rVec, tVec, camIntrinsic, camDistortion, points2d);
	//projectPoints(points3d, camCalib, trans[0], points2d);

	//draw the 2d points
	for(size_t j=0; j<verticesNum; ++j)
	{
		cv::line(img, cv::Point((int)points2d[j].x, (int)points2d[j].y),
		cv::Point((int)points2d[j].x, (int)points2d[j].y), CV_RGB(0,0,0),2);
	}

	// and draw the edges
	for(size_t j=0; j<edgesNum; ++j)
	{
		unsigned int p0= edges[j].vindices[0]-1;
		unsigned int p1 = edges[j].vindices[1]-1;
		cv::line(img, cv::Point((int)points2d[p0].x, (int)points2d[p0].y),
		cv::Point((int)points2d[p1].x, (int)points2d[p1].y), CV_RGB(255,255,255),1);
	}
}