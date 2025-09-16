#include "DvsRgbFusionCamera/CameraManager/DvsRgbFusionCamera.hpp"
#include "DvsenseBase/logging/logger.hh"
#include "DvsRgbFusionCamera/rgb/hik/HikCamera.hpp"

template<typename RGBCameraType>
DvsRgbFusionCamera<RGBCameraType>::DvsRgbFusionCamera(float aps_fps): aps_fps_(aps_fps)
{
    frame_callback_id_num_ = 0;
    ext_trigger_sync_callback_id_ = 0;
    recording_frame_callback_id_ = 0;
    ext_trigger_sync_running_ = false;
    save_frame_num_ = 0;
    aps_is_recording_ = false;
    dvs_camera_ = std::make_shared<dvsense::DvsEventCamera>();
    rgb_camera_ = RgbCamera::create<RGBCameraType>(aps_fps_);
}

template<typename RGBCameraType>
DvsRgbFusionCamera<RGBCameraType>::~DvsRgbFusionCamera()
{
}

template<typename RGBCameraType>
bool DvsRgbFusionCamera<RGBCameraType>::findCamera(std::vector<dvsense::CameraDescription>& dvs_camera_descs, std::vector<std::string>& aps_serial_numbers)
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

template<typename RGBCameraType>
bool DvsRgbFusionCamera<RGBCameraType>::openCamera(DvsRgbCameraSerial dvs_rgb_serial_number)
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
            std::unique_lock<std::mutex> lock(recording_mutex_);
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

template<typename RGBCameraType>
const bool DvsRgbFusionCamera<RGBCameraType>::isConnected()
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

template<typename RGBCameraType>
int DvsRgbFusionCamera<RGBCameraType>::start(dvsense::STREAM_TYPE type)
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


template<typename RGBCameraType>
int DvsRgbFusionCamera<RGBCameraType>::stop(dvsense::STREAM_TYPE type)
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
        //if (save_to_mp4_thread_.joinable()) save_to_mp4_thread_.join();
        return 0;
    }
    case dvsense::FUSION_STREAM:
    {
        ext_trigger_sync_running_ = false;
        rgb_camera_->stopCamera();
        dvs_camera_->stopCamera();
        /*if (save_to_mp4_thread_.joinable()) save_to_mp4_thread_.join();*/
        return 0;
    }
    default:
        throw std::invalid_argument("Unknown STREAM_TYPE");
    }
    return 0;
}

template<typename RGBCameraType>
uint32_t DvsRgbFusionCamera<RGBCameraType>::addEventsStreamHandleCallback(const dvsense::EventsStreamHandleCallback& callback) {
    return dvs_camera_->registerEventCallback(callback);
}

template<typename RGBCameraType>
bool DvsRgbFusionCamera<RGBCameraType>::removeEventsStreamHandleCallback(uint32_t callback_id)
{
    return dvs_camera_->removeEventCallback(callback_id);
}

template<typename RGBCameraType>
uint32_t DvsRgbFusionCamera<RGBCameraType>::addTriggerInCallback(const NewTriggerInCallback& newTriggerInCallback)
{
    return dvs_camera_->addTriggerInCallback(newTriggerInCallback);
}

template<typename RGBCameraType>
bool DvsRgbFusionCamera<RGBCameraType>::removeTriggerInCallback(uint32_t callback_id)
{
    return dvs_camera_->removeTriggerInCallback(callback_id);
}

template<typename RGBCameraType>
std::string DvsRgbFusionCamera<RGBCameraType>::getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H_%M_%S", std::localtime(&time));
    return std::string(buffer);
}

template<typename RGBCameraType>
int DvsRgbFusionCamera<RGBCameraType>::startRecording(std::string output_dir) {
    std::string current_time = getCurrentTime();

    std::string json_path;
    if (!output_dir.empty() ) {
        json_path = output_dir + "/fusion-" + current_time + ".json";
    }

    std::cout << "json_path" << json_path << std::endl;
    json_file_.open(json_path);
    if (!json_file_.is_open()) {
        std::cerr << "Fatal Error: Failed to open JSON file: " << json_path << std::endl;
        std::exit(EXIT_FAILURE);
    }


    std::string dvs_file_path = output_dir + "/fusion-" + current_time + ".raw";
    std::string aps_file_path = output_dir + "/fusion-" + current_time + ".mp4";
    sync_json_["aps_file_path"] = aps_file_path;
    sync_json_["dvs_file_path"] = dvs_file_path;

    dvs_camera_->startRecording(dvs_file_path);
    aps_to_mp4_->setOutputFile(aps_file_path);
    std::unique_lock<std::mutex> lock(recording_mutex_);
    aps_is_recording_ = true;
    std::cout << "Start Recording, Saving to " << output_dir << std::endl;
    return 0;
}

template<typename RGBCameraType>
int DvsRgbFusionCamera<RGBCameraType>::stopRecording() {
    std::cout << "Stop Recording." << std::endl;
    dvs_camera_->stopRecording();

    std::unique_lock<std::mutex> lock(recording_mutex_);
    aps_is_recording_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    aps_to_mp4_->flushAndCloseVideo();

    json_file_.close();

    return 0;
}

template<typename RGBCameraType>
void DvsRgbFusionCamera<RGBCameraType>::extTriggerSyncCallback()
{
    ext_trigger_sync_callback_id_ = dvs_camera_->addTriggerInCallback(
        [this](const dvsense::EventTriggerIn& begin) {          
            static uint64_t rise_trigger_timestamp = 0;
            if (begin.polarity == 0) 
            {
                rise_trigger_timestamp = begin.timestamp;
                return;            
            }

            if (ext_trigger_sync_running_)
            {
                dvsense::ApsFrame new_frame(0, 0);
                while (!rgb_camera_->getNewRgbFrame(new_frame) && ext_trigger_sync_running_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

                new_frame.exposure_start_timestamp = rise_trigger_timestamp;
                new_frame.exposure_end_timestamp = begin.timestamp;
                if (new_frame.getDataSize() != 0)
                {
                    for (const auto& callback : frame_callbacks_) {
                        callback.second(new_frame);
                    }
                }
            }
        }
    );
}


template<typename RGBCameraType>
int DvsRgbFusionCamera<RGBCameraType>::addApsFrameCallback(const FrameCallback& frameCallback) {
    frame_callback_id_num_++;
    frame_callbacks_.insert(
        { frame_callback_id_num_, frameCallback }
    );
    return frame_callback_id_num_;
}

template<typename RGBCameraType>
bool DvsRgbFusionCamera<RGBCameraType>::removeApsFrameCallback(uint32_t callback_id) {
    auto it = frame_callbacks_.find(callback_id);
    if (it != frame_callbacks_.end()) {
        frame_callbacks_.erase(it); 
        return true;
    }
    return false;
}

template<typename RGBCameraType>
uint16_t DvsRgbFusionCamera<RGBCameraType>::getWidth(dvsense::STREAM_TYPE type)
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

template<typename RGBCameraType>
uint16_t DvsRgbFusionCamera<RGBCameraType>::getHeight(dvsense::STREAM_TYPE type)
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

template<typename RGBCameraType>
const std::shared_ptr<dvsense::CameraTool> DvsRgbFusionCamera<RGBCameraType>::getTool(dvsense::ToolType type)
{
    return dvs_camera_->getTool(type);
}

template<typename RGBCameraType>
int DvsRgbFusionCamera<RGBCameraType>::destroy()
{
    dvs_camera_.reset();
    int ret = rgb_camera_->destroyCamera();
    rgb_camera_.reset();
    return ret;
}

template class DvsRgbFusionCamera<HikCamera>;