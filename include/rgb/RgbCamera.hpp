#pragma once
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

class RgbCamera {
public:
	virtual ~RgbCamera() = default;

	virtual bool findCamera() = 0;

	virtual int startCamera() = 0;

	virtual int getWidth() = 0;

	virtual int getHeight() = 0;

	virtual void stopCamera() = 0;

	virtual bool getNewRgbFrame(cv::Mat& output_frame) = 0;

	static std::unique_ptr<RgbCamera> create(float fps);

};


