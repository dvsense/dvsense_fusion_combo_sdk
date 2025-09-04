#include "DvsRgbFusionCamera/camera_manager/DvsRgbFusionCamera.hpp"
#include "DvsenseBase/logging/logger.hh"

DvsRgbFusionCamera::DvsRgbFusionCamera(float aps_fps): aps_fps_(aps_fps)
{
    event_buffer_ = std::make_shared<dvsense::Event2DVector>();
    dvs_camera_ = std::make_shared<dvsense::DvsEventCamera>();
    rgb_camera_ = RgbCamera::create(aps_fps_);
}
DvsRgbFusionCamera::~DvsRgbFusionCamera()
{
}

bool DvsRgbFusionCamera::findCamera(std::vector<dvsense::CameraDescription>& dvs_camera_descs, std::vector<std::string>& aps_serial_numbers)
{
	bool ret = dvs_camera_->findCamera(dvs_camera_descs);
    if (!ret) 
    {
        return false;
    }
    ret = rgb_camera_->findCamera(aps_serial_numbers);
    if (!ret)
    {
        return false;
    }
	return true;
}

bool DvsRgbFusionCamera::openCamera(DvsRgbCameraSerial dvs_rgb_serial_number)
{
    bool ret = rgb_camera_->openCamera(dvs_rgb_serial_number.rgb_serial_number);
    if (!ret)
    {
        return false;
    }
	ret = dvs_camera_->openCamera(dvs_rgb_serial_number.dvs_serial_number);
    if (!ret)
    {
        return false;
    }

    extTriggerSyncCallback();

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
    recording_frame_callback_id_ = addApsFrameCallback(
        [this](const dvsense::ApsFrame& rgb_frame) {
            if (aps_is_recording_) {
                if (save_frame_num_ == 0)
                {
                    aps_save_ts_offset_ = rgb_frame.exposure_end_timestamp;
                    sync_json_["aps_offset_timestamp"] = aps_save_ts_offset_;
                    if (json_file_.is_open()) {
                        json_file_ << sync_json_.dump(4);
                    }

                }
                if (rgb_frame.getDataSize() != 0)
                {
                    aps_to_mp4_->rgbToVideo(rgb_frame.data(), rgb_frame.exposure_end_timestamp - aps_save_ts_offset_);
                }
                save_frame_num_++;
            }
            else
            {
                save_frame_num_ = 0;
            }
        });
	return true;
}

const bool DvsRgbFusionCamera::isConnected()
{
    bool ret;
    ret = dvs_camera_->isConnect();
    if (!ret) 
    {
        std::cout << "DVS camera is not connected" << std::endl;
        return false;
    }
    ret = rgb_camera_->isConnect();
    if (!ret)
    {
        std::cout << "APS camera is not connected" << std::endl;
        return false;
    }
    return true;
}

int DvsRgbFusionCamera::start(dvsense::STREAM_TYPE type)
{
    switch (type) {
    case dvsense::DVS_STREAM: 
    {
        ext_trigger_sync_running_ = false;
        dvs_camera_->startCamera();
        return 0;
    }
    case dvsense::APS_STREAM:
    {
        std::cout << "Error: Standalone RGB camera operation is not supported." << std::endl;
        return -1;
    }
    case dvsense::FUSION_STREAM:
    {
        ext_trigger_sync_running_ = false;
        dvs_camera_->startCamera();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ext_trigger_sync_running_ = true;
        rgb_camera_->startCamera();
        return 0;
    }
    default:
        throw std::invalid_argument("Unknown STREAM_TYPE");
    }

    return 0;
}

int DvsRgbFusionCamera::stop(dvsense::STREAM_TYPE type)
{
    switch (type) {
    case dvsense::DVS_STREAM:
    {
        std::cout << "Error: Standalone DVS camera operation is not supported." << std::endl;
        return -1;
    }
    case dvsense::APS_STREAM:
    {
        ext_trigger_sync_running_ = false;
        rgb_camera_->stopCamera();
        return 0;
    }
    case dvsense::FUSION_STREAM:
    {
        ext_trigger_sync_running_ = false;
        rgb_camera_->stopCamera();
        dvs_camera_->stopCamera();
        return 0;
    }
    default:
        throw std::invalid_argument("Unknown STREAM_TYPE");
    }
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

std::string DvsRgbFusionCamera::getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H_%M_%S", std::localtime(&time));
    return std::string(buffer);
}

int DvsRgbFusionCamera::startRecording(std::string output_dir) {
    boost::filesystem::path output_dir_path(output_dir);
    if (!boost::filesystem::exists(output_dir_path)) {
        if (!boost::filesystem::create_directory(output_dir_path)) {
            std::cerr << "Directory creation failed" << std::endl;
        }
    }
    std::string current_time = getCurrentTime();

    std::string json_path = (output_dir_path / ("fusion-" + current_time + ".json")).string();
    json_file_.open(json_path);

    std::string dvs_file_path = (output_dir_path / ("fusion-" + current_time + ".raw")).string();
    std::string aps_file_path = (output_dir_path / ("fusion-" + current_time + ".mp4")).string();
    sync_json_["aps_file_path"] = aps_file_path;
    sync_json_["dvs_file_path"] = dvs_file_path;

    dvs_camera_->startRecording(dvs_file_path);
    aps_to_mp4_->setOutputFile(aps_file_path);
    aps_is_recording_ = true;
    std::cout << "Start Recording, Saving to " << output_dir << std::endl;
    return 0;
}
int DvsRgbFusionCamera::stopRecording() {
    std::cout << "Stop Recording." << std::endl;
    dvs_camera_->stopRecording();

    aps_is_recording_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    aps_to_mp4_->flushAndCloseVideo();

    json_file_.close();

    return 0;
}



void DvsRgbFusionCamera::extTriggerSyncCallback()
{
    ext_trigger_sync_callback_id_ = dvs_camera_->addTriggerInCallback(
        [this](const dvsense::EventTriggerIn& begin) {          
            static uint64_t rise_trigger_timestamp = 0;
            if (begin.polarity == 1) 
            {
                rise_trigger_timestamp = begin.timestamp;
                return;            
            }

            if (ext_trigger_sync_running_)
            {
                cv::Mat new_frame;
                while (!rgb_camera_->getNewRgbFrame(new_frame) && ext_trigger_sync_running_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
               
                dvsense::ApsFrame rgb_frame(new_frame.cols, new_frame.rows, rise_trigger_timestamp, begin.timestamp, new_frame.data, new_frame.total() * new_frame.elemSize());
                if (rgb_frame.getDataSize() != 0)
                {
                    for (const auto& callback : frame_callbacks_) {
                        callback.second(rgb_frame);
                    }                
                }
            }
        }
    );
}

int DvsRgbFusionCamera::addApsFrameCallback(const FrameCallback& frameCallback) {
    frame_callback_id_num_++;
    frame_callbacks_.insert(
        { frame_callback_id_num_, frameCallback }
    );
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

uint16_t DvsRgbFusionCamera::getWidth(dvsense::STREAM_TYPE type)
{
    switch (type) {
    case dvsense::DVS_STREAM:
        return dvs_camera_->getDvsWidth();
    case dvsense::APS_STREAM:
        return rgb_camera_->getWidth();
    case dvsense::FUSION_STREAM:
        return dvs_camera_->getDvsWidth();
    default:
        throw std::invalid_argument("Unknown STREAM_TYPE");
    }
}

uint16_t DvsRgbFusionCamera::getHeight(dvsense::STREAM_TYPE type)
{
    switch (type) {
    case dvsense::DVS_STREAM:
        return dvs_camera_->getDvsHeight();
    case dvsense::APS_STREAM:
        return rgb_camera_->getHeight();
    case dvsense::FUSION_STREAM:
        return dvs_camera_->getDvsHeight();
    default:
        throw std::invalid_argument("Unknown STREAM_TYPE");
    }
}

const std::shared_ptr<dvsense::CameraTool> DvsRgbFusionCamera::getTool(dvsense::ToolType type)
{
    return dvs_camera_->getTool(type);
}

int DvsRgbFusionCamera::destroy()
{
    dvs_camera_.reset();
    int ret = rgb_camera_->destroyCamera();
    rgb_camera_.reset();
    return ret;
}
