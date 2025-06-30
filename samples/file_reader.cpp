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
	std::string event_file_path = "D:/FusionCamera/test_datas/fusion-20250624-1.raw";
	std::cout << "Event file path: " << event_file_path << std::endl;

	dvsense::DvsFile reader = dvsense::DvsFileReader::createFileReader(event_file_path);
	if (!reader->loadFile())
	{
		LOG_ERROR("Load file failed!");
		return -1;
	}

	EventAnalyzer event_analyzer;
	event_analyzer.setup_display(reader->getWidth(), reader->getHeight());

	const int fps = 30; // event-based cameras do not have a frame rate, but we need one for visualization
	const int wait_time = static_cast<int>(std::round(1.f / fps * 1000)); // how long we should wait between two frames
	cv::Mat display;                                                      // frame where events will be accumulated
	const std::string window_name = "DVSense File Viewer";
	cv::namedWindow(window_name, cv::WINDOW_GUI_EXPANDED);
	cv::resizeWindow(window_name, reader->getWidth(), reader->getHeight());

	// ----------------- Event processing and show -----------------

	dvsense::TimeStamp start_timestamp, end_timestamp;
	reader->getStartTimeStamp(start_timestamp);
	reader->getEndTimeStamp(end_timestamp);
	LOG_INFO("File start ts:%u", start_timestamp);
	LOG_INFO("File end ts:%u", end_timestamp);
	uint64_t max_events = 0;
	reader->getMaxEvents(max_events);
	dvsense::TimeStamp get_time = start_timestamp;
	bool stop_application = false;
	while (!stop_application) {
		//Control the acquisition time and display frame rate to determine the playback rate
		std::shared_ptr<dvsense::Event2DVector> events = reader->getNTimeEventsGivenStartTimeStamp(get_time, 20000);
		get_time += 20000;
		event_analyzer.process_events(events->data(), events->data() + events->size());

		if (reader->reachedEndOfEvents())
		{
			//replay
			get_time = start_timestamp;
			std::cout << "Playback finished, replaying." << std::endl;
			reader->seekTime(get_time);
		}
		event_analyzer.get_display_frame(display);
		if (!display.empty()) {
			cv::imshow(window_name, display);
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