#pragma once
#include <iostream>
#include <vector>
#include "json/json.hpp"
#include "opencv2/opencv.hpp"

#ifdef _WIN32
#define DVSENSE_API __declspec(dllexport)
#else
#define DVSENSE_API
#endif // _WIN32

struct DVSENSE_API CameraParams
{
	cv::Mat camera_matrix;
	cv::Mat rotation;
	cv::Mat translation;
};

class DVSENSE_API CalibrateThroughFile
{
public:
	CalibrateThroughFile(std::string config_file);

	~CalibrateThroughFile();

	cv::Mat getDvsToApsHomographyMatrix(double);

	cv::Mat getApsToDvsHomographyMatrix(double);

	cv::Mat warpImage(const cv::Mat&, const cv::Mat&, cv::Size); 

private:
	void readParams();

	std::string config_file_;
	CameraParams dvs_params_;
	CameraParams aps_params_;
	cv::Mat plane_normal_;
};

