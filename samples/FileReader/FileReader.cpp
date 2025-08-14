#include <queue>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include "DvsenseDriver/FileReader/DvsFileReader.h"
#include "DvsenseBase/logging/logger.hh"
#include "CalibrateThroughFile.hpp"

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

int main() {
	cv::VideoCapture cap("D:/FusionCamera/test_datas/fusion-2025-08-14_15_14_35.mp4", cv::CAP_FFMPEG);
	std::string event_file_path = "D:/FusionCamera/test_datas/fusion-2025-08-14_15_14_35.raw";
	if (!cap.isOpened()) {
		std::cerr << "Error: Could not open video file." << std::endl;
		return -1;
	} 

	dvsense::DvsFile reader = dvsense::DvsFileReader::createFileReader(event_file_path);
	if (!reader->loadFile())
	{
		LOG_ERROR("Load file failed!");
		return -1;
	}

	bool is_calibrator = true;

	std::unique_ptr<CalibrateThroughFile> calibrator = std::make_unique<CalibrateThroughFile>("D:/FusionCamera/dvsense_fusion_combo_sdk/calibration_result.json");
	cv::Mat H = calibrator->getApsToDvsH(600);

	EventAnalyzer event_analyzer;
	int dvs_width = reader->getWidth();
	int dvs_height = reader->getHeight();
	event_analyzer.setup_display(dvs_width, dvs_height);

	const int fps = 60; // event-based cameras do not have a frame rate, but we need one for visualization
	const int wait_time = static_cast<int>(std::round(1.f / fps * 1000)); // how long we should wait between two frames

	const std::string window_name = "DVSense File Viewer";
	cv::namedWindow(window_name, cv::WINDOW_GUI_EXPANDED);
	cv::resizeWindow(window_name, reader->getWidth(), reader->getHeight());

	// ----------------- Event processing and show -----------------

	dvsense::TimeStamp offset_timestamp;
	reader->getStartTimeStamp(offset_timestamp);

	cv::Mat new_dvs_frame;
	cv::Mat new_aps_frame;
	cv::Mat frame;
	while (cap.read(frame)) {
		dvsense::TimeStamp pts = cap.get(cv::CAP_PROP_POS_MSEC) * 1000;
		dvsense::TimeStamp current_ts = pts + offset_timestamp;

		std::shared_ptr<dvsense::Event2DVector> events = reader->getNTimeEventsGivenStartTimeStamp(current_ts, wait_time * 1000);
		event_analyzer.process_events(events->data(), events->data() + events->size());
		if (reader->reachedEndOfEvents())
		{
			break;
		}
		event_analyzer.get_display_frame(new_dvs_frame);

		if (!is_calibrator) 
		{
			int y_start = (frame.rows - new_dvs_frame.rows) / 2;
			int x_start = (frame.cols - new_dvs_frame.cols) / 2;
			frame(cv::Rect(x_start, y_start, new_dvs_frame.cols, new_dvs_frame.rows)).copyTo(new_aps_frame);
			new_aps_frame.setTo(cv::Scalar(0, 0, 0), new_dvs_frame);
			new_aps_frame = new_aps_frame + new_dvs_frame;		
		}
		else
		{
			cv::Mat img_warped = calibrator->warpImage(frame, H, cv::Size(dvs_width, dvs_height));
			img_warped.setTo(cv::Scalar(0, 0, 0), new_dvs_frame);
			new_aps_frame = img_warped + new_dvs_frame;
		}

		cv::imshow(window_name, new_aps_frame);

		int key = cv::waitKey(wait_time);
		if ((key & 0xff) == 'q' || (key & 0xff) == 27) {
			break;
		}
	}

	cap.release();
	cv::destroyAllWindows();
}