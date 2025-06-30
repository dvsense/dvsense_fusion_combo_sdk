#define DVS_PROCESS_EXPORT

#include "dvs_process.hpp"

DvsProcess::DvsProcess(const int height, const int width, const int max_count) :
	height_(height), width_(width), max_count_(max_count)
{
    event_buffer_ = std::make_shared<dvsense::Event2DVector>();
    event_analyzer.setup_display(width_, height_);
}

DvsProcess::~DvsProcess() 
{
}

void DvsProcess::add_events(dvsense::EventIterator_t begin, dvsense::EventIterator_t end) 
{
    //if (!event_buffer_) {
    //    throw std::runtime_error("event_buffer_ is null!");
    //}

    size_t new_events_count = std::distance(begin, end);
    //std::cout << "new_events_count" << new_events_count << std::endl;

    std::lock_guard<std::mutex> lock(event_buffer_mutex_);
    event_buffer_->insert(event_buffer_->end(), begin, end);
}

void DvsProcess::get_histogram(cv::Mat& output_frame)
{
    //std::cout << "event_buffer_ size"<< event_buffer_->size() << std::endl;
    event_analyzer.process_events(event_buffer_->data(), event_buffer_->data() + event_buffer_->size());
    event_analyzer.get_display_frame(output_frame);
    event_buffer_->clear();
}

void DvsProcess::reset() {
    event_buffer_->clear();
}