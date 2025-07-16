#pragma once
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include "DvsEventCamera.hpp"
#include "RgbCamera.hpp"
#include "ApsTypes.hpp"
#include "DataToVideo.hpp"

#ifdef _WIN32
#define DVSENSE_API __declspec(dllexport)
#else
#define DVSENSE_API
#endif // _WIN32


class DVSENSE_API DvsRgbFusionCamera
{
public:
	DvsRgbFusionCamera(float fps = 30);

	~DvsRgbFusionCamera();

	const bool isConnected();

	const dvsense::CameraDescription getDvsDesc()
	{
		return dvs_camera_->getDvsDesc();
	}

	uint32_t addEventsStreamHandleCallback(const dvsense::EventsStreamHandleCallback& callback);

	bool removeEventsStreamHandleCallback(uint32_t callback_id);

	uint32_t addTriggerInCallback(const NewTriggerInCallback& newTriggerInCallback);

	bool removeTriggerInCallback(uint32_t callback_id);

	int addApsFrameCallback(const FrameCallback& frameCallback);

	bool removeApsFrameCallback(uint32_t callback_id);

	void addEvents(dvsense::EventIterator_t begin, dvsense::EventIterator_t end);

	std::shared_ptr<dvsense::Event2DVector> getEvents();

	void reset();

	int start();

	int startRecording(std::string output_dir = "");

	int stop();

	int stopRecording();

	uint16_t getWidth();

	uint16_t getHeight();

private:
	std::shared_ptr<dvsense::DvsEventCamera> dvs_camera_;
	std::unique_ptr<RgbCamera> rgb_camera_;
	std::vector<dvsense::CameraDescription> dvs_camera_descs_;

	float aps_fps_;
	std::mutex event_buffer_mutex_;
	std::mutex frame_buffer_mutex_;

	std::map<int, const FrameCallback> frame_callbacks_;

	int frame_callback_id_num_ = 0;

	std::shared_ptr<dvsense::Event2DVector> event_buffer_;

	std::string getCurrentTime();

	// ----- sync -----
	void extTriggerSyncCallback();

	bool findCamera();
	bool openCamera();

	dvsense::CameraDescription camera_desc_;

	std::shared_ptr<DataToVideo> aps_to_mp4_;
	std::shared_ptr<DataToVideo> aps_decoder_;
	bool aps_is_recording_ = false;
	dvsense::TimeStamp aps_save_ts_offset_;

};