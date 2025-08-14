#pragma once
#include <iostream>
#include <vector>
#include "json/json.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

#ifdef _WIN32
#define DVSENSE_API __declspec(dllexport)
#else
#define DVSENSE_API
#endif // _WIN32

struct DVSENSE_API CameraParams
{
	cv::Mat k;
	cv::Mat r;
	cv::Mat t;
};

class DVSENSE_API CalibrateThroughFile
{
public:
	CalibrateThroughFile(std::string config_file):
		config_file_(config_file)
	{
		readParams();
	}

	~CalibrateThroughFile();

	cv::Mat getDvsToApsH(double);

	cv::Mat getApsToDvsH(double);

	cv::Mat warpImage(const cv::Mat&, const cv::Mat&, cv::Size); 

private:
	std::string config_file_;
	CameraParams dvs_params_;
	CameraParams aps_params_;

	cv::Mat plane_normal_ = (cv::Mat_<double>(3, 1) << 0, 0, 1);	

	void readParams();

};

