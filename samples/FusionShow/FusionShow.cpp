#include "DvsRgbFusionCamera/camera_manager/DvsRgbFusionCamera.hpp"
#include "DvsRgbCalib/CalibrateThroughFile.hpp"

// purple - magenta
cv::Vec3b color_bg = cv::Vec3b(0x00, 0x00, 0x00);
cv::Vec3b color_on = cv::Vec3b(0xff, 0x00, 0xff);
cv::Vec3b color_off = cv::Vec3b(0x00, 0xff, 0x00);

// Gray
//cv::Vec3b color_bg = cv::Vec3b(0x70, 0x70, 0x70);
//cv::Vec3b color_on = cv::Vec3b(0xbf, 0xbc, 0xb4);
//cv::Vec3b color_off = cv::Vec3b(0x40, 0x3d, 0x33);


int main(int argc, char* argv[])
{
	std::string data_save_path = "./data";
	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " <data_save_path>" << std::endl;
	}
	else {
	    data_save_path = argv[1];	
	}

	bool is_calibration_active = true;

	std::unique_ptr<CalibrateThroughFile> calibrator = std::make_unique<CalibrateThroughFile>("D:/FusionCamera/dvsense_fusion_combo_sdk/calibration_result.json");
	cv::Mat H = calibrator->getApsToDvsHomographyMatrix(600);

	std::queue<cv::Mat> image_display_queue;
	std::mutex display_image_mutex;

	std::unique_ptr<DvsRgbFusionCamera> fusionCamera = std::make_unique<DvsRgbFusionCamera>(60);

	std::vector<dvsense::CameraDescription> dvs_serials;
	std::vector<std::string> rgb_serials;
	if (!fusionCamera->findCamera(dvs_serials, rgb_serials))
	{
		std::cout << "fusionCamera find failed!" << std::endl;
		return 0;
	}
	DvsRgbCameraSerial dvs_rgb_camera_serial;
	dvs_rgb_camera_serial.dvs_serial_number = dvs_serials[0];
	dvs_rgb_camera_serial.rgb_serial_number = rgb_serials[0];
	if (!fusionCamera->openCamera(dvs_rgb_camera_serial))
	{
		std::cout << "fusionCamera open failed!" << std::endl;
		return 0;
	}
	if (!fusionCamera->isConnected()) 
	{
		std::cout << "fusionCamera is not connected" << std::endl;
		return 0;
	}
	// dvsense::CameraDescription camera_desc = fusionCamera->getDvsDesc();

	uint16_t dvs_width = fusionCamera->getWidth(dvsense::DVS_STREAM);
	uint16_t dvs_height = fusionCamera->getHeight(dvsense::DVS_STREAM);

	cv::Mat display;
	std::mutex dvs_frame_mutex;

	cv::Mat new_dvs_frame(dvs_height, dvs_width, CV_8UC3, cv::Scalar(0, 0, 0));
	fusionCamera->addEventsStreamHandleCallback(
		[&new_dvs_frame, &dvs_frame_mutex](const dvsense::EventIterator_t begin, const dvsense::EventIterator_t end) {
			std::unique_lock<std::mutex> lock(dvs_frame_mutex);
			for (auto it = begin; it != end; ++it) {
				new_dvs_frame.at<cv::Vec3b>(it->y, it->x) = (it->polarity) ? color_on : color_off;			
			}
		}
	);

	fusionCamera->addApsFrameCallback(
		[&is_calibration_active, &calibrator, &H, &image_display_queue, &new_dvs_frame, &dvs_width, &dvs_height, &dvs_frame_mutex, &display_image_mutex](const dvsense::ApsFrame& rgbframe)
		{
			if (rgbframe.getDataSize() != 0 && image_display_queue.empty())
			{
				const uint8_t* external_data = rgbframe.data();
				int aps_width = rgbframe.width();
				int aps_height = rgbframe.height();

				cv::Mat reconstructed_image(aps_height, aps_width, CV_8UC3, const_cast<void*>(static_cast<const void*>(external_data)));

				static cv::Mat aps_resize;

				if (!is_calibration_active)
				{

					static int new_dvs_width = dvs_width * 1.215;
					static int new_dvs_height = dvs_height * 1.215;

					int y_start = (aps_height - new_dvs_height) / 2;
					int x_start = (aps_width - new_dvs_width) / 2;
					aps_resize = reconstructed_image(cv::Rect(x_start, y_start, new_dvs_width, new_dvs_height));
					cv::resize(aps_resize, aps_resize, cv::Size(dvs_width, dvs_height), cv::INTER_AREA);
					{
						std::unique_lock<std::mutex> lock(dvs_frame_mutex);
						aps_resize.setTo(cv::Scalar(0, 0, 0), new_dvs_frame);
						aps_resize = aps_resize + new_dvs_frame;
						new_dvs_frame.setTo(cv::Scalar(0, 0, 0));
					}
					std::unique_lock<std::mutex> lock(display_image_mutex);
					image_display_queue.emplace(aps_resize);
				}
				else
				{
					cv::Mat img_warped = calibrator->warpImage(reconstructed_image, H, cv::Size(dvs_width, dvs_height));
					{
						std::unique_lock<std::mutex> lock(dvs_frame_mutex);
						img_warped.setTo(cv::Scalar(0, 0, 0), new_dvs_frame);
						img_warped = img_warped + new_dvs_frame;
						new_dvs_frame.setTo(cv::Scalar(0, 0, 0));
					}
					std::unique_lock<std::mutex> lock(display_image_mutex);
					image_display_queue.emplace(img_warped);
				}
			}
			else
			{
				std::unique_lock<std::mutex> lock(dvs_frame_mutex);
				new_dvs_frame.setTo(cv::Scalar(0, 0, 0));
			}

		}
	);

	fusionCamera->start(dvsense::FUSION_STREAM);

	const int fps = 30; // event-based cameras do not have a frame rate, but we need one for visualization
	const int wait_time = static_cast<int>(std::round(1.f / fps * 1000)); // how long we should wait between two frames 

	const std::string window_name = "DVSense File Viewer";
	cv::namedWindow(window_name, cv::WINDOW_GUI_EXPANDED);
	cv::resizeWindow(window_name, 1280, 720);
	// ----------------- Event processing and show -----------------
	bool is_recording = false;
	bool stop_application = false;
	while (!stop_application) {
		if (!image_display_queue.empty()) {
			std::unique_lock<std::mutex> lock(display_image_mutex);
			cv::Mat new_frame = image_display_queue.front();
			cv::imshow(window_name, new_frame);
			image_display_queue.pop();
		}	
		int key = cv::waitKey(wait_time);

		if ((key & 0xff) == 'q' || (key & 0xff) == 27) {
			stop_application = true;
			break;
		}

		if ((key & 0xff) == ' ') {
			if (!is_recording)
			{
				fusionCamera->startRecording(data_save_path);
			}
			if (is_recording)
			{
				fusionCamera->stopRecording();
			}
			is_recording = !is_recording;
		}
	}
	if (is_recording)
	{
		fusionCamera->stopRecording();
	}

	fusionCamera->stop();
	fusionCamera->destroy();
	cv::destroyAllWindows();
	return 0;
}