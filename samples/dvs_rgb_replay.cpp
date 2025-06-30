#include <queue>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "DvsenseDriver/FileReader/DvsFileReader.h"
#include "DvsenseBase/logging/logger.hh"

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
	std::queue<dvsense::TimeStamp> timestamps;
	std::ifstream file("D:/FusionCamera/test_datas/fusion-202506251931.txt");
	if (!file.is_open()) {
		std::cerr << "Error: Cannot open file!" << std::endl;
		return 1;
	}

	dvsense::TimeStamp ts;
	while (file >> ts) {
		timestamps.push(ts);  // 逐行读取并入队
	}
	file.close();

	cv::VideoCapture cap("D:/FusionCamera/test_datas/fusion-202506251931.mp4");
	if (!cap.isOpened()) {
		std::cerr << "Error: Could not open video file." << std::endl;
		return -1;
	}

	// ----------------- Program description -----------------

	//if (argc < 2) {
	//	std::cerr << "Usage: " << argv[0] << " <event_file_path>" << std::endl;
	//	return 1;
	//}

	const std::string short_program_desc(
		"Simple viewer to stream events from an event file, using the SDK driver API.\n");
	std::string long_program_desc(short_program_desc +
		"Press 'q' or Escape key to leave the program.\n");
	std::cout << long_program_desc << std::endl;

	// ----------------- Event file initialization -----------------

	//std::string event_file_path = argv[1];
	std::string event_file_path = "D:/FusionCamera/test_datas/fusion-202506251931.raw";
	std::cout << "Event file path: " << event_file_path << std::endl;

	dvsense::DvsFile reader = dvsense::DvsFileReader::createFileReader(event_file_path);
	if (!reader->loadFile())
	{
		LOG_ERROR("Load file failed!");
		return -1;
	}

	EventAnalyzer event_analyzer;
	event_analyzer.setup_display(reader->getWidth(), reader->getHeight());

	const int fps = 100; // event-based cameras do not have a frame rate, but we need one for visualization
	const int wait_time = static_cast<int>(std::round(1.f / fps * 1000)); // how long we should wait between two frames
	cv::Mat display;                                                      // frame where events will be accumulated
	const std::string window_name = "DVSense File Viewer";
	cv::namedWindow(window_name, cv::WINDOW_GUI_EXPANDED);
	cv::resizeWindow(window_name, reader->getWidth(), reader->getHeight());

	// ----------------- Event processing and show -----------------

	dvsense::TimeStamp start_timestamp, end_timestamp, offset_timestamp;
	reader->getStartTimeStamp(start_timestamp);
	reader->getEndTimeStamp(end_timestamp);
	LOG_INFO("File start ts:%u", start_timestamp);
	LOG_INFO("File end ts:%u", end_timestamp);
	uint64_t max_events = 0;
	reader->getMaxEvents(max_events);
	bool stop_application = false;
	if (!timestamps.empty()) {
		offset_timestamp = timestamps.front() - start_timestamp;
		LOG_INFO("offset_timestamp ts:%u", offset_timestamp);
	}
	cv::Mat rgb_frame;
	while (!timestamps.empty() && !stop_application) {
		cap >> rgb_frame; 
		if (rgb_frame.empty()) {
			std::cout << "End of video." << std::endl;
			break;
		}

		cv::Size target_size(1280, 720);
		resize(rgb_frame, rgb_frame, target_size, 0, 0, cv::INTER_LINEAR);
		dvsense::TimeStamp current_ts = timestamps.front();
		timestamps.pop();
		std::shared_ptr<dvsense::Event2DVector> events = reader->getNTimeEventsGivenStartTimeStamp(current_ts- offset_timestamp, 20000);
		event_analyzer.process_events(events->data(), events->data() + events->size());
		if (reader->reachedEndOfEvents())
		{
			//replay
			std::cout << "Playback finished, replaying." << std::endl;
		}
		event_analyzer.get_display_frame(display);
		cv::Mat grayImage;
		cv::cvtColor(display, grayImage, cv::COLOR_BGR2GRAY);
		cv::Mat mask;
		threshold(grayImage, mask, 1, 255, cv::THRESH_BINARY);
		display.copyTo(rgb_frame, mask);
		//rgb_frame.setTo(0, mask);  
		
		if (true) {
			//cv::imshow(window_name, display);
			cv::imshow(window_name, rgb_frame);
		}

		// If user presses `q` key, exit loop and stop application
		int key = cv::waitKey(wait_time);
		if ((key & 0xff) == 'q' || (key & 0xff) == 27) {
			stop_application = true;
			std::cout << "q pressed, exiting." << std::endl;
		}
		else if ((key & 0xff) == 'c') {
			// cvt mp4 test
			if (reader->exportEventDataToVideo(start_timestamp, end_timestamp, "test.mp4"))
			{
				std::cout << "export video success!" << std::endl;
			}
		}
	}
	
	cv::destroyAllWindows();
}