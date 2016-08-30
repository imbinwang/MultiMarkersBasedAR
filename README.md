# Multiple Markers Based AR #

### Introduction ###

This code repository demonstrated the augmented reality by multiple different makers. 

### Requirements ###

1. OS Platform: Windows
2. [OpenCV](http://opencv.org/): The version should be below 3.0, because the 2.X APIs are different from 3.X somehow. 
3. Before you can run the code project, you should configure the correct pathes of includes, libs and dlls for aforementioned third dependencies.

### Installation ###

1. Clone the MultiMarkersBasedAR repository

	```
	git clone https://github.com/imbinwang/MultiMarkersBasedAR.git
	```

2. We'll call the directory that you cloned MultiMarkersBasedAR into MultiMarkersBasedAR_ROOT. You should print the marker files in `MultiMarkersBasedAR_ROOT/data` and calibrate your camera in advance. 

3. Modify the maker size and camera paramters in the file `MultiMarkersBasedAR_ROOT/src/main.cpp`

	```
	//load camera intrinsic parameters
	float fx = 840.83909392801706;
	float fy = 840.83909392801706;
	float cx = 319.5;
	float cy = 239.5;
	float distortionCoeff[5] = {-0.0073528715923502968, 1.3968282968421968, 
		0., 0.,-9.3220236317679284};
	CameraCalibration camCalib(fx,fy,cx,cy,distortionCoeff);

	// marker location configuration. Set the location of object w.r.t the marker origin point.
	float objOriginZ = 1.5f; //this is for model box
	cv::Size2f markerSize = cv::Size2f(5, 5); // the real size of marker

	float obj2marker213Translation[3] = {11.3f, 6.8f, objOriginZ};// the translation of obj with respect to maker coodinate system, measured in advance
	float obj2marker231Translation[3] = {-11.3f, 6.5f, objOriginZ};
	float obj2marker561Translation[3] = {11.5f, -7.5f, objOriginZ};
	float obj2marker411Translation[3] = {-11.3f, -7.6f, objOriginZ};

	//load obj file
	std::string objFilePath = "data/box.obj";
	GLMmodel *objModel = glmReadOBJ(const_cast<char*>(objFilePath.c_str()));
	```

4. Run and the recorded video and pose file will be found in `MultiMarkersBasedAR_ROOT/data`.


