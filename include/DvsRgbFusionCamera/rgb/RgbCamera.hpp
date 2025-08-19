#pragma once

#ifdef _WIN32
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#else 
#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#endif

class RgbCamera {
public:
	virtual ~RgbCamera() = default;

	virtual bool findCamera(std::vector<std::string>& serial_numbers) = 0;

	virtual bool openCamera(std::string serial_number) = 0;

	virtual int startCamera() = 0;

	virtual bool isConnect() = 0;

	virtual int getWidth() = 0;

	virtual int getHeight() = 0;

	virtual void stopCamera() = 0;

	virtual int destroyCamera() = 0;

	virtual bool getNewRgbFrame(cv::Mat& output_frame) = 0;

	static std::unique_ptr<RgbCamera> create(float fps);

};


