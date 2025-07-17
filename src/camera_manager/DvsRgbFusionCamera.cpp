#include "DvsRgbFusionCamera.hpp"


DvsRgbFusionCamera::DvsRgbFusionCamera(float aps_fps): aps_fps_(aps_fps)
{
    event_buffer_ = std::make_shared<dvsense::Event2DVector>();
    dvs_camera_ = std::make_shared<dvsense::DvsEventCamera>();
    rgb_camera_ = RgbCamera::create(aps_fps_);
    extTriggerSyncCallback();
}
DvsRgbFusionCamera::~DvsRgbFusionCamera()
{
}

const bool DvsRgbFusionCamera::isConnected()
{
    bool ret = findCamera();
    if (!ret)
    {
        return false;
    }
    ret = openCamera();
    if (!ret)
    {
        return false;
    }
    return true;
}

bool DvsRgbFusionCamera::findCamera()
{
	bool ret = dvs_camera_->findCamera(dvs_camera_descs_);
    if (!ret) 
    {
        return false;
    }
    ret = rgb_camera_->findCamera();
    if (!ret)
    {
        return false;
    }
	return true;
}

bool DvsRgbFusionCamera::openCamera()
{
	bool ret = dvs_camera_->openCamera(dvs_camera_descs_[0]);
    if (!ret)
    {
        return false;
    }
	return true;
}

int DvsRgbFusionCamera::start()
{
    dvs_camera_->startCamera();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    rgb_camera_->startCamera();


    int aps_width = rgb_camera_->getWidth();
    int aps_height = rgb_camera_->getHeight();
    aps_to_mp4_ = std::make_shared<DataToVideo>();
    aps_to_mp4_->setConverterFmt(AV_PIX_FMT_BGR24, AV_PIX_FMT_YUV420P);
    aps_to_mp4_->setConverterFrameSize(aps_height, aps_width, aps_height, aps_width);
    if (aps_to_mp4_->initVideoConverter() < 0)
    {
        aps_to_mp4_.reset();
        aps_is_recording_ = false;
    }
    addApsFrameCallback(
        [this](const RgbFrame& rgb_frame) {
            if (aps_is_recording_) {
                if (rgb_frame.getDataSize() != 0)
                {
                    aps_to_mp4_->rgbToVideo(rgb_frame.data(), rgb_frame.exposure_start_timestamp - aps_save_ts_offset_);
                }
            }
            else
            {
                aps_save_ts_offset_ = rgb_frame.exposure_start_timestamp;
            }
        });

    return 0;
}

int DvsRgbFusionCamera::stop()
{
	dvs_camera_->stopCamera();
    rgb_camera_->stopCamera();
    return 0;
}

uint32_t DvsRgbFusionCamera::addEventsStreamHandleCallback(const dvsense::EventsStreamHandleCallback& callback) {
    return dvs_camera_->registerEventCallback(callback);
}

bool DvsRgbFusionCamera::removeEventsStreamHandleCallback(uint32_t callback_id)
{
    return dvs_camera_->removeEventCallback(callback_id);
}

uint32_t DvsRgbFusionCamera::addTriggerInCallback(const NewTriggerInCallback& newTriggerInCallback)
{
    return dvs_camera_->addTriggerInCallback(newTriggerInCallback);
}
bool DvsRgbFusionCamera::removeTriggerInCallback(uint32_t callback_id)
{
    return dvs_camera_->removeTriggerInCallback(callback_id);
}

//void DvsRgbFusionCamera::addEvents(dvsense::EventIterator_t begin, dvsense::EventIterator_t end)
//{
//    std::lock_guard<std::mutex> lock(event_buffer_mutex_);
//    /*event_buffer_->insert(event_buffer_->end(), begin, end);*/
//    event_buffer_->insert(event_buffer_->end(), begin, end);
//}
//
//std::shared_ptr<dvsense::Event2DVector> DvsRgbFusionCamera::getEvents()
//{
//	//std::lock_guard<std::mutex> lock(event_buffer_mutex_);
//	//return std::make_shared<dvsense::Event2DVector>(*event_buffer_); // Éî¿½±´
//    return event_buffer_; 
//}
//
//void DvsRgbFusionCamera::reset() {
//	//std::lock_guard<std::mutex> lock(event_buffer_mutex_);
//    event_buffer_->clear();
//}


std::string DvsRgbFusionCamera::getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm local_time;
    localtime_s(&local_time, &now_time);

    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y%m%d%H%M");

    return oss.str();
}

int DvsRgbFusionCamera::startRecording(std::string output_dir) {
    boost::filesystem::path output_dir_path(output_dir);
    if (!boost::filesystem::exists(output_dir_path)) {
        if (!boost::filesystem::create_directory(output_dir_path)) {
            std::cerr << "Directory creation failed" << std::endl;
        }
    }
    std::string current_time = getCurrentTime();

    std::string dvs_file_path = (output_dir_path / ("fusion-" + current_time + ".raw")).string();
    std::cout << dvs_file_path << std::endl;
    dvs_camera_->startRecording(dvs_file_path);

    aps_to_mp4_->setOutputFile((output_dir_path / ("fusion-" + current_time + ".mp4")).string());
    aps_is_recording_ = true;
    std::cout << "Start Recording, Saving to " << output_dir << std::endl;
    return 0;
}
int DvsRgbFusionCamera::stopRecording() {
    std::cout << "Stop Recording." << std::endl;
    dvs_camera_->stopRecording();

    aps_is_recording_ = false;
    aps_to_mp4_->flushAndCloseVideo();

    return 0;
}



void DvsRgbFusionCamera::extTriggerSyncCallback()
{
    dvs_camera_->addTriggerInCallback(
        [this](const dvsense::EventTriggerIn* begin) {

            // only rising edges are accurate
            if (begin->polarity == 0) return;

            cv::Mat newFrame;

            while (!rgb_camera_->getNewRgbFrame(newFrame)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            RgbFrame rgb_frame(newFrame.cols, newFrame.rows, begin->timestamp, begin->timestamp+5, newFrame.data, newFrame.total() * newFrame.elemSize());
            if (rgb_frame.getDataSize() != 0)
            {
                for (const auto& callback : frame_callbacks_) {
                    callback.second(rgb_frame);
                }                
            }
        }
    );
}

int DvsRgbFusionCamera::addApsFrameCallback(const FrameCallback& frameCallback) {
    frame_callbacks_.insert(
        { frame_callback_id_num_, frameCallback }
    );
    frame_callback_id_num_++;
    return frame_callback_id_num_;
}

bool DvsRgbFusionCamera::removeApsFrameCallback(uint32_t callback_id) {
    auto it = frame_callbacks_.find(callback_id);
    if (it != frame_callbacks_.end()) {
        frame_callbacks_.erase(it); 
        return true;
    }
    return false;
}

uint16_t DvsRgbFusionCamera::getWidth()
{
    return dvs_camera_->getWidth();
}

uint16_t DvsRgbFusionCamera::getHeight()
{
    return dvs_camera_->getHeight();
}
