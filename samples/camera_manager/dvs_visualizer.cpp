#define DVS_VISUALIZER_EXPORT

#include <dvs_visualizer.hpp>
#include <cstdint>

DvsVisualizer::DvsVisualizer(const int height, const int width, const int max_count) : 
	height_(height), width_(width), max_count_(max_count)
{
	event_buffer_ = std::make_unique<EventBuffer>(height_ * width_ * 2, 0);
    time_surface_ = cv::Mat(height_, width_, CV_8UC1, cv::Scalar(1));
}

//void DvsVisualizer::update_events(const dvsense::Event2D* begin, const dvsense::Event2D* end) {
//	int index = 0;
//	std::unique_lock<std::mutex> lock(event_buffer_mutex_);
//	for (auto& event = begin; event != end; event++) {
//		index = (event->y * width_ + event->x) * 2 + event->polarity;
//		if (event_buffer_->at(index) < max_count_) {
//			event_buffer_->at(index)++;
//		}
//	}
//}

void DvsVisualizer::update_events(const Metavision::EventCD* begin, const Metavision::EventCD* end) {
	int index = 0;
	std::unique_lock<std::mutex> lock(event_buffer_mutex_);
	for (auto& event = begin; event != end; event++) {
		index = (event->y * width_ + event->x) * 2 + event->p;
		if (event_buffer_->at(index) < max_count_) {
			event_buffer_->at(index)++;
		}
	}
}
//void DvsVisualizer::update_events(const dvsense::Event2DVector &events) {
//    int index = 0;
//	std::unique_lock<std::mutex> lock(event_buffer_mutex_);
//	for (auto& event : events) {
//		index = (event.y * width_ + event.x) * 2 + event.polarity;
//		if (event_buffer_->at(index) < max_count_) {
//			event_buffer_->at(index)++;
//		}
//	}   
//}

//void DvsVisualizer::update_events(dvsense::EventIterator_t begin, dvsense::EventIterator_t end) {
//    int index = 0;
//	std::unique_lock<std::mutex> lock(event_buffer_mutex_);
//	for (auto& event = begin; event != end; event++) {
//		index = (event->y * width_ + event->x) * 2 + event->polarity;
//		if (event_buffer_->at(index) < max_count_) {
//			event_buffer_->at(index)++;
//		}
//	}
//}

void DvsVisualizer::visualize_frame(
	const cv::Mat& input_frame, cv::Mat& output_frame
) {
	output_frame = input_frame.clone();
	visualize_events_on_frame(output_frame);
}

void DvsVisualizer::visualize_frame(
	const cv::Mat& input_frame, cv::Mat& output_frame, cv::Point2f offset
) {
	output_frame = input_frame.clone();
	visualize_events_on_frame(output_frame, offset);
}

void DvsVisualizer::update_time_surface(const dvsense::EventIterator_t begin, const dvsense::EventIterator_t end) {
    std::unique_lock<std::mutex> lock(time_surface_mutex_);
    for (auto event = begin; event != end; event++) {
        if(event->polarity) {
            time_surface_.at<uint8_t>(event->y, event->x) = 255;
        } else {
            time_surface_.at<uint8_t>(event->y, event->x) = 0;
        }
    }
}

void DvsVisualizer::get_time_surface(cv::Mat &output_frame) {
    std::unique_lock<std::mutex> lock(time_surface_mutex_);
    output_frame = time_surface_.clone();
}

void DvsVisualizer::merge_time_surface(const cv::Mat &input_frame, cv::Mat &output_frame) {
    std::unique_lock<std::mutex> lock(time_surface_mutex_);
    output_frame = input_frame.clone();
    int cut_pixel_cnt = (output_frame.rows - height_) / 2;
	output_frame = output_frame(cv::Rect(0, cut_pixel_cnt, width_, height_));
    // output_frame.setTo(cv::Scalar(128), time_surface_);

    for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
            int y_rectified = y;
			int x_rectified = x;
            if (y_rectified < 0 || x_rectified < 0) continue;

			if (time_surface_.at<uint8_t>(y, x) == 255) {
				output_frame.at<cv::Vec3b>(y_rectified, x_rectified) =
					cv::Vec3b(255, 255, 255);
			} else if (time_surface_.at<uint8_t>(y, x) == 0) {
				output_frame.at<cv::Vec3b>(y_rectified, x_rectified) =
					cv::Vec3b(0, 0, 0);
			}
		}
	}
}

void DvsVisualizer::reset() {
	std::unique_lock<std::mutex> lock(event_buffer_mutex_);
	std::fill(event_buffer_->begin(), event_buffer_->end(), 0);
    time_surface_.setTo(cv::Scalar(1));
}

void DvsVisualizer::gen_undistort_map(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs) {
    cv::Mat map1, map2;
    //cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), camera_matrix, cv::Size(width_, height_), CV_16SC2, map1, map2);

}

void DvsVisualizer::get_histogram(cv::Mat& output_event_frame) {
	std::unique_lock<std::mutex> lock(event_buffer_mutex_);

	output_event_frame = cv::Mat(height_, width_, CV_8UC3, cv::Vec3b(0, 0, 0));
	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			int base_idx = (y * width_ + x) * 2;
			int negative_event_cnt = event_buffer_->at(base_idx);
			int positive_event_cnt = event_buffer_->at(base_idx + 1);
			if (positive_event_cnt > 0 && positive_event_cnt >= negative_event_cnt) {
				output_event_frame.at<cv::Vec3b>(y, x) =
					cv::Vec3b(0, 0, uint8_t(255.0 * (positive_event_cnt - negative_event_cnt) / max_count_));
			}
			else if (negative_event_cnt > 0 && negative_event_cnt > positive_event_cnt) {
				output_event_frame.at<cv::Vec3b>(y, x) =
					cv::Vec3b(uint8_t(255.0 * (negative_event_cnt - positive_event_cnt) / max_count_), 0, 0);
			}
		}
	}
}

void DvsVisualizer::visualize_events_on_frame(cv::Mat& canvas) {
	int cut_pixel_cnt = (canvas.rows - height_) / 2;
	canvas = canvas(cv::Rect(0, cut_pixel_cnt, width_, height_));
	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			int base_idx = (y * width_ + x) * 2;
			int negative_event_cnt = event_buffer_->at(base_idx);
			int positive_event_cnt = event_buffer_->at(base_idx + 1);
			if (positive_event_cnt > 0 && positive_event_cnt >= negative_event_cnt) {
				canvas.at<cv::Vec3b>(y, x) =
					cv::Vec3b(0, 0, uint8_t(255.0 * (positive_event_cnt - negative_event_cnt) / max_count_));
			}
			else if (negative_event_cnt > 0 && negative_event_cnt > positive_event_cnt) {
				canvas.at<cv::Vec3b>(y, x) =
					cv::Vec3b(uint8_t(255.0 * (negative_event_cnt - positive_event_cnt) / max_count_), 0, 0);
			}
		}
	}
}

void DvsVisualizer::visualize_events_on_frame(cv::Mat& canvas, cv::Point2f offset) {
	int cut_pixel_cnt = (canvas.rows - height_) / 2;
	canvas = canvas(cv::Rect(0, cut_pixel_cnt, width_, height_));

	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			int base_idx = (y * width_ + x) * 2;
			int negative_event_cnt = event_buffer_->at(base_idx);
			int positive_event_cnt = event_buffer_->at(base_idx + 1);
			int y_rectified = y - offset.y;
			int x_rectified = x - offset.x;
			if (y_rectified < 0 || x_rectified < 0 || y_rectified > height_ || x_rectified > width_) continue;
			if (positive_event_cnt > 0 && positive_event_cnt >= negative_event_cnt) {
				canvas.at<cv::Vec3b>(y_rectified, x_rectified) =
					cv::Vec3b(0, 0, uint8_t(255.0 * (positive_event_cnt - negative_event_cnt) / max_count_));
			}
			else if (negative_event_cnt > 0 && negative_event_cnt > positive_event_cnt) {
				canvas.at<cv::Vec3b>(y_rectified, x_rectified) =
					cv::Vec3b(uint8_t(255.0 * (negative_event_cnt - positive_event_cnt) / max_count_), 0, 0);
			}
		}
	}
}

DvsVisualizer::~DvsVisualizer()
{
}