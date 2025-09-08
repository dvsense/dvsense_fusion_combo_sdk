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

#include <opencv2/core.hpp>

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

		bool isConnect();

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
        void openDvsTriggerIn();

		dvsense::CameraDevice dvs_camera_;

		dvsense::DvsCameraManager camera_manager_;

		uint32_t trigger_in_callback_id_num_;

		std::thread trigger_thread_;

		dvsense::CameraDescription camera_desc_;

		bool dvs_camera_running_;

	};
}

