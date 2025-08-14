#pragma once
#include <iostream>
#include <memory>
#include <thread>
#include <deque>
#include <mutex>
#include <chrono>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <fstream>
#include "boost/filesystem.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include <DvsenseDriver/camera/DvsCameraManager.hpp>
#include "DvsenseHal/camera/DvsCameraUtils.hpp"
#include "DvsenseHal/EventStream/RawEventStreamFormat.hpp"
#include "DvsenseBase/EventBase/EventTypes.hpp"
#include "DvsenseHal/camera/tools/CameraTool.h"


using NewTriggerInCallback = std::function<void(const dvsense::EventTriggerIn& begin)>;

namespace dvsense 
{
	class DvsEventCamera
	{
	public:
		DvsEventCamera();

		~DvsEventCamera();

		const dvsense::CameraDescription getDvsDesc()
		{
			return camera_desc_;
		}

		bool findCamera(std::vector<dvsense::CameraDescription> &);

		bool openCamera(dvsense::CameraDescription);

		int startCamera();

		void stopCamera();

		uint32_t registerEventCallback(const dvsense::EventsStreamHandleCallback& callback);
		bool removeEventCallback(uint32_t callback_id);

		void startRecording(std::string output_dir = "");
		void stopRecording();

		uint32_t addTriggerInCallback(NewTriggerInCallback);

		bool removeTriggerInCallback(uint32_t callback_id);

		uint16_t getDvsWidth();

		uint16_t getDvsHeight();

		std::shared_ptr<dvsense::CameraTool> getTool(dvsense::ToolType type);


	private:
		dvsense::CameraDevice dvs_camera_;

		dvsense::DvsCameraManager camera_manager_;

		std::map<uint32_t, const NewTriggerInCallback> new_trigger_in_callbacks_;
		uint32_t trigger_in_callback_id_num_ = 0;

		std::thread trigger_thread_;

		void openDvsTriggerIn();

		dvsense::CameraDescription camera_desc_;

	};
}

