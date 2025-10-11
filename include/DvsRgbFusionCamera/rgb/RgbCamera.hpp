#pragma once

#include <opencv2/core.hpp>
#include <DvsenseDriver/camera/DvsCameraManager.hpp>

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

	virtual bool getNewRgbFrame(dvsense::ApsFrame& output_frame) = 0;

	virtual bool openExternalTrigger() = 0;

	template<typename RGBCameraType>
	static std::unique_ptr<RgbCamera> create(float fps)
	{
		return std::make_unique<RGBCameraType>(fps);
	}

};


