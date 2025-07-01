#ifndef __DVS_VISUALIZER__
#define __DVS_VISUALIZER__

#include <memory>
#include <vector>
#include <mutex>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "metavision/sdk/base/events/event_cd.h"

#include <DvsenseDriver/camera/DvsCameraManager.hpp>

#include <DvsenseDriver/camera/DvsCameraManager.hpp>
#include <DvsenseHal/camera/DvsCameraUtils.hpp>

#ifdef _WIN32
#ifdef DVS_VISUALIZER_EXPORT
#define DVS_API __declspec(dllexport)
#else
#define DVS_API __declspec(dllimport)
#endif
#else
#define DVS_API DLLEXPORT
#define DLLEXPORT __attribute__((visibility("default")))
#endif

class DVS_API DvsVisualizer
{
public:
	DvsVisualizer(const int height, const int width, const int max_count=10);
	~DvsVisualizer();
	void update_events(const Metavision::EventCD* begin, const Metavision::EventCD* end);
	void update_events(const dvsense::Event2D* begin, const dvsense::Event2D* end);
    //void update_events(const dvsense::Event2DVector &events);
    //void update_events(dvsense::EventIterator_t begin, dvsense::EventIterator_t end);
	void visualize_frame(const cv::Mat &input_frame, cv::Mat &output_frame);
	void visualize_frame(const cv::Mat &input_frame, cv::Mat &output_frame, cv::Point2f offset);
    /*void update_time_surface(const Metavision::EventCD* begin, const Metavision::EventCD* end);*/
	void update_time_surface(const dvsense::EventIterator_t begin, const dvsense::EventIterator_t end);
    void get_time_surface(cv::Mat &output_frame);
    void get_histogram(cv::Mat &output_frame);
    void merge_time_surface(const cv::Mat &input_frame, cv::Mat &output_frame);
	void reset();
    void gen_undistort_map(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs);

	using EventBuffer = std::vector<uint16_t>;

private:
	const int width_;
	const int height_;
	int max_count_;

    std::mutex time_surface_mutex_;
    cv::Mat time_surface_;
	
	std::mutex event_buffer_mutex_;
	std::unique_ptr<EventBuffer> event_buffer_;

	void visualize_events_on_frame(cv::Mat& canvas);
    void visualize_events_on_frame(cv::Mat& canvas, cv::Point2f offset);
};

#endif // !__DVS_VISUALIZER__