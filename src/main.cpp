#include "../include/CameraCalibration.h"
#include "../include/GLM.h"
#include "../include/Marker.h"
#include "../include/MarkerDetector.h"
#include "../include/utility.h"

#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
	//load camera intrinsic parameters
	float fx = 840.83909392801706;
	float fy = 840.83909392801706;
	float cx = 319.5;
	float cy = 239.5;
	float distortionCoeff[5] = {-0.0073528715923502968, 1.3968282968421968, 
		0., 0.,-9.3220236317679284};
	CameraCalibration camCalib(fx,fy,cx,cy,distortionCoeff);

	// marker location configuration. Set the location of object w.r.t the marker origin point.
	float objOriginZ = 1.5f; //this is for model box, 半径。 因为模型局部坐标系xoy平面并不与marker所在平面一样。
	cv::Size2f markerSize = cv::Size2f(5, 5); // marker的真实边长，单位厘米，与模型的度量单位一致

	float obj2marker213Translation[3] = {11.3f, 6.8f, objOriginZ};// 模型相对于marker213的平移向量，手工测量
	float obj2marker231Translation[3] = {-11.3f, 6.5f, objOriginZ};// 模型相对于marker231的平移向量，手工测量
	float obj2marker561Translation[3] = {11.5f, -7.5f, objOriginZ};// 模型相对于marker561的平移向量，手工测量
	float obj2marker411Translation[3] = {-11.3f, -7.6f, objOriginZ};// 模型相对于marker411的平移向量，手工测量	

	//load obj file
	std::string objFilePath = "data/box.obj";
	GLMmodel *objModel = glmReadOBJ(const_cast<char*>(objFilePath.c_str()));

	//initial marker detector
	MarkerDetector markerDetector(camCalib, markerSize);

	// read video or camera
	cv::VideoCapture vc;
	vc.open(0);

	if( !vc.isOpened() )
	{
		printf("Cannot open the camera\n");
		return -1;
	}
	else		
	{
		printf("Open the camera\n\n");
		printf("Help:\n");
		printf("\t s -- Start to record video and pose\n"
			"\t f --  Finish the recording\n"
			"\t ESC --  Quit\n");
	}

	const int frameWidth = (int) vc.get(CV_CAP_PROP_FRAME_WIDTH);
	const int frameHeight = (int) vc.get(CV_CAP_PROP_FRAME_HEIGHT);
	const double frameFPS = 30.0;
	cv::Mat frame, arDrawing;

	// record the video and pose
	std::string videoPath = "data/video.avi";
	cv::VideoWriter vw;
	vw.open(videoPath, CV_FOURCC('D','I','V','X'),frameFPS, cv::Size(frameWidth, frameHeight), true);
	if( !vw.isOpened() )
	{
		printf("Cannot write the video\n");
		return -1;
	}

	std::string poseFile = "data/pose.txt";
	std::ofstream outPose;
	outPose.open(poseFile);
	if( !outPose.is_open() )
	{
		printf("Cannot write the pose\n");
		return -1;
	}

	// initial GUI
	char *winNameDisplay = "ARScene";
	cv::namedWindow(winNameDisplay);
	uchar key = 0;
	uchar controlKey = 0;
	for(;;)
	{
		vc >> frame;
		if( frame.empty() ) break;
		frame.copyTo( arDrawing );

		if(key=='s') controlKey = 's';
		if(key=='f') controlKey = 'f';
		if(key==27) break;

		// start video capturation and pose recording
		if(controlKey=='s')
		{
			// record video
			vw << frame;

			// find all the possible good markers
			std::vector<Marker> markers;
			markerDetector.findMarkers(frame, markers);

			// manipulate all the possible good markers
			std::pair<int, Transformation> possible_markerid_transformation_pair;
			for(size_t i=0; i<markers.size();++i)
			{
				// 这里我没做逻辑判断，将由每个marker推导出的物体姿态都输出了， 到时候选择一个就行
				// 没有有效的marker时，6个姿态参数没有写入pose文件，这里可能要注意逐帧姿态对应的问题
				int marker_id = markers[i].m_id;
				if(marker_id==213 || marker_id==231 || marker_id==411 || marker_id==561)
				{
					switch(marker_id)
					{
					case 213:
						{
							Transformation objTransformation;
							calculateObjTransformation(markers[i].m_transformation, obj2marker213Translation, objTransformation);

							cv::Matx33f rotMat(objTransformation.r().data);
							std::vector<float> rVec;
							cv::Rodrigues(rotMat, rVec);

							outPose << rVec[0] << " " << rVec[1] << " " << rVec[2] << " "
								<< objTransformation.t().data[0] << " "
								<< objTransformation.t().data[1] << " "
								<< objTransformation.t().data[2] << std::endl;

							//printf("id%d: %f, %f, %f, %f, %f, %f\n", markers[i].m_id, rVec[0], rVec[1], rVec[2], 
							//	objTransformation.t().data[0], objTransformation.t().data[1], objTransformation.t().data[2]);

							drawCoordinate(camCalib, markers[i].m_transformation, arDrawing);
							drawObject(camCalib, objTransformation, objModel, arDrawing);

							break;
						}
					case 231:
						{
							Transformation objTransformation;
							calculateObjTransformation(markers[i].m_transformation, obj2marker231Translation, objTransformation);

							cv::Matx33f rotMat(objTransformation.r().data);
							std::vector<float> rVec;
							cv::Rodrigues(rotMat, rVec);

							outPose << rVec[0] << " " << rVec[1] << " " << rVec[2] << " "
								<< objTransformation.t().data[0] << " "
								<< objTransformation.t().data[1] << " "
								<< objTransformation.t().data[2] << std::endl;

							//printf("id %d: %f, %f, %f, %f, %f, %f\n", markers[i].m_id, rVec[0], rVec[1], rVec[2], 
							//	objTransformation.t().data[0], objTransformation.t().data[1], objTransformation.t().data[2]);

							drawCoordinate(camCalib, markers[i].m_transformation, arDrawing);
							drawObject(camCalib, objTransformation, objModel, arDrawing);

							break;
						}
					case 411:
						{
							Transformation objTransformation;
							calculateObjTransformation(markers[i].m_transformation, obj2marker411Translation, objTransformation);

							cv::Matx33f rotMat(objTransformation.r().data);
							std::vector<float> rVec;
							cv::Rodrigues(rotMat, rVec);

							outPose << rVec[0] << " " << rVec[1] << " " << rVec[2] << " "
								<< objTransformation.t().data[0] << " "
								<< objTransformation.t().data[1] << " "
								<< objTransformation.t().data[2] << std::endl;

							//printf("id %d: %f, %f, %f, %f, %f, %f\n", markers[i].m_id, rVec[0], rVec[1], rVec[2], 
							//	objTransformation.t().data[0], objTransformation.t().data[1], objTransformation.t().data[2]);

							drawCoordinate(camCalib, markers[i].m_transformation, arDrawing);
							drawObject(camCalib, objTransformation, objModel, arDrawing);

							break;
						}
					case 561:
						{
							Transformation objTransformation;
							calculateObjTransformation(markers[i].m_transformation, obj2marker561Translation, objTransformation);

							cv::Matx33f rotMat(objTransformation.r().data);
							std::vector<float> rVec;
							cv::Rodrigues(rotMat, rVec);

							outPose << rVec[0] << " " << rVec[1] << " " << rVec[2] << " "
								<< objTransformation.t().data[0] << " "
								<< objTransformation.t().data[1] << " "
								<< objTransformation.t().data[2] << std::endl;

							//printf("id %d: %f, %f, %f, %f, %f, %f\n", markers[i].m_id, rVec[0], rVec[1], rVec[2], 
							//	objTransformation.t().data[0], objTransformation.t().data[1], objTransformation.t().data[2]);

							drawCoordinate(camCalib, markers[i].m_transformation, arDrawing);
							drawObject(camCalib, objTransformation, objModel, arDrawing);

							break;
						}
					}
				}


			}

			// 没有marker时，写到文件去的6个姿态参数也全为零
			if(markers.size() == 0)
			{
				outPose << 0 << " " << 0 << " " << 0 << " "
					<< 0 << " "
					<< 0 << " "
					<< 0 << std::endl;
			}
		}

		// finish the recording
		if(controlKey=='f')
		{
			vw.release();
			outPose.close();
		}

		cv::imshow(winNameDisplay, arDrawing);
		key = cv::waitKey(10);
	}

	cv::destroyWindow(winNameDisplay);
	vc.release();

	glmDelete(objModel);

	return 0;
}