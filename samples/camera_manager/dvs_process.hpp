#ifndef __DVS_PROCESS__
#define __DVS_PROCESS__

#include <memory>
#include <vector>
#include <mutex>
#include <iostream>
#include <thread>
#include <deque>
#include <chrono>
#include <functional>
#include <condition_variable>
#include <atomic>
#include "boost/filesystem.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

#include <DvsenseDriver/camera/DvsCameraManager.hpp>
#include <DvsenseDriver/camera/DvsCameraManager.hpp>
#include <DvsenseHal/camera/DvsCameraUtils.hpp>
#include "DvsenseDriver/FileReader/DvsFileReader.h"

#ifdef _WIN32
#ifdef DVS_PROCESS_EXPORT
#define DVS_API __declspec(dllexport)
#else
#define DVS_API __declspec(dllimport)
#endif
#else
#define DVS_API DLLEXPORT
#define DLLEXPORT __attribute__((visibility("default")))
#endif

class EventAnalyzer {
public:
	cv::Mat img, img_swap;
	uint64_t total_events = 0;
	std::mutex m;

	// Gray
	//cv::Vec3b color_bg = cv::Vec3b(0x70, 0x70, 0x70);
	//cv::Vec3b color_on = cv::Vec3b(0xbf, 0xbc, 0xb4);
	//cv::Vec3b color_off = cv::Vec3b(0x40, 0x3d, 0x33);

	cv::Vec3b color_bg = cv::Vec3b(0x00, 0x00, 0x00);
	cv::Vec3b color_on = cv::Vec3b(0xff, 0x00, 0xff);
	cv::Vec3b color_off = cv::Vec3b(0x00, 0xff, 0x00);

	void setup_display(const int width, const int height) {
		img = cv::Mat(height, width, CV_8UC3);
		img_swap = cv::Mat(height, width, CV_8UC3);
		img.setTo(color_bg);
	}

	// Called from main Thread
	void get_display_frame(cv::Mat& display) {
		// Swap images
		{
			std::unique_lock<std::mutex> lock(m);
			std::swap(img, img_swap);
			img.setTo(color_bg);
		}
		img_swap.copyTo(display);
	}

	// Called from decoding Thread
	void process_events(const dvsense::Event2D* begin, const dvsense::Event2D* end) {
		std::unique_lock<std::mutex> lock(m);
		uint32_t event_count = 0;
		for (auto it = begin; it != end; ++it) {
			img.at<cv::Vec3b>(it->y, it->x) = (it->polarity) ? color_on : color_off;
			event_count++;
		}
	}
};

class DVS_API DvsProcess
{
public:
	DvsProcess(const int height, const int width, const int max_count = 10);
	~DvsProcess();

	void add_events(dvsense::EventIterator_t begin, dvsense::EventIterator_t end);

	void get_histogram(cv::Mat& output_frame);

	void reset();

private:
	const int width_;
	const int height_;
	int max_count_;

	std::mutex event_buffer_mutex_;
	std::shared_ptr<dvsense::Event2DVector> event_buffer_;

	EventAnalyzer event_analyzer;
};

#endif // !__DVS_PROCESS__