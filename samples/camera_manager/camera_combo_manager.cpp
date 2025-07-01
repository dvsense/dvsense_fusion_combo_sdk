#define DVSENSE_APSFRAME_EXPORTS
#include "camera_combo_manager.hpp"
#include "DvsenseBase/logging/logger.hh"

namespace dvsense
{
    int CameraComboManager::find_aps_camera() {
        MV_CC_DEVICE_INFO_LIST mvs_device_info_list;
        memset(&mvs_device_info_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &mvs_device_info_list);
        if (ret != MV_OK) {
            std::cout << "MV_CC_EnumDevices fail! ret = " << ret << std::endl;
            return -1;
        }
        // print out the device information
        if (mvs_device_info_list.nDeviceNum > 0) {
            for (unsigned int i = 0; i < mvs_device_info_list.nDeviceNum; i++) {
                MV_CC_DEVICE_INFO* p_device_info = mvs_device_info_list.pDeviceInfo[i];
                if (p_device_info == nullptr) {
                    break;
                }
                std::cout << "[device " << i << "]:" << std::endl;
                std::cout << "UserDefinedName: " << p_device_info->SpecialInfo.stUsb3VInfo.chUserDefinedName << std::endl;
                std::cout << "Serial Number: " << p_device_info->SpecialInfo.stUsb3VInfo.chSerialNumber << std::endl;
                std::cout << "Device Number: " << p_device_info->SpecialInfo.stUsb3VInfo.nDeviceNumber << std::endl << std::endl;
            }
        }
        else {
            std::cout << "Find No Devices!" << std::endl;
            return -1;
        }

        std::cout << "Use the first device to create handle" << std::endl;
        ret = MV_CC_CreateHandle(&aps_camera_handle_, mvs_device_info_list.pDeviceInfo[0]);
        if (ret != MV_OK) {
            std::cout << "MV_CC_CreateHandle fail! ret = " << ret << std::endl;
            return -1;
        }

        return 0;
    }

    void CameraComboManager::buffer_to_mat(
        cv::Mat& frame
    ) {
        bool isMono;  // Mono or not
        switch (frame_out_.stFrameInfo.enPixelType) {
        case PixelType_Gvsp_Mono1p:
        case PixelType_Gvsp_Mono2p:
        case PixelType_Gvsp_Mono4p:
        case PixelType_Gvsp_Mono8:
        case PixelType_Gvsp_Mono8_Signed:
        case PixelType_Gvsp_Mono10:
        case PixelType_Gvsp_Mono10_Packed:
        case PixelType_Gvsp_Mono12:
        case PixelType_Gvsp_Mono12_Packed:
        case PixelType_Gvsp_Mono14:
        case PixelType_Gvsp_Mono16:
            isMono = true;
            break;
        default:
            isMono = false;
            break;
        }
        if (isMono) {
            frame = cv::Mat(
                frame_out_.stFrameInfo.nHeight, frame_out_.stFrameInfo.nWidth, 0,
                frame_out_.pBufAddr).clone();
        }
        else {
            // convert to bgr
            unsigned int dstBufSize =
                frame_out_.stFrameInfo.nHeight * frame_out_.stFrameInfo.nWidth * 3 + 2048;
            unsigned char* pDstData = (unsigned char*)malloc(dstBufSize);

            MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam = { 0 };
            stConvertParam.nWidth = frame_out_.stFrameInfo.nWidth;
            stConvertParam.nHeight = frame_out_.stFrameInfo.nHeight;
            stConvertParam.pSrcData = const_cast<unsigned char*>(frame_out_.pBufAddr);
            stConvertParam.nSrcDataLen = frame_out_.stFrameInfo.nFrameLen;
            stConvertParam.enSrcPixelType = frame_out_.stFrameInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            stConvertParam.pDstBuffer = pDstData;
            stConvertParam.nDstBufferSize = dstBufSize;
            MV_CC_ConvertPixelTypeEx(aps_camera_handle_, &stConvertParam);;
            frame = cv::Mat(frame_out_.stFrameInfo.nHeight, frame_out_.stFrameInfo.nWidth, 16,
                pDstData).clone();
            free(pDstData);
            pDstData = NULL;
        }
    }

    int CameraComboManager::get_next_frame(cv::Mat& frame) {
        int ret = MV_CC_GetImageBuffer(aps_camera_handle_, &frame_out_, 1000);
        if (ret != MV_OK) {
            std::cout << "MV_CC_GetOneFrameTimeout fail! ret = " << std::to_string(ret) << std::endl;
            frame = cv::Mat(frame_out_.stFrameInfo.nHeight, frame_out_.stFrameInfo.nWidth, CV_8UC1);
            return -1;
        }

        buffer_to_mat(frame);

        ret = MV_CC_FreeImageBuffer(aps_camera_handle_, &frame_out_);
        if (ret != MV_OK) {
            std::cout << "MV_CC_ReleaseImageBuffer fail! ret = " << ret << std::endl;
            frame = cv::Mat(frame_out_.stFrameInfo.nHeight, frame_out_.stFrameInfo.nWidth, CV_8UC1);
            return -1;
        }
        return 0;
    }

    int CameraComboManager::start_aps_camera() {
        int ret = MV_CC_OpenDevice(aps_camera_handle_);
        if (ret != MV_OK) {
            std::cout << "MV_CC_OpenDevice fail! ret = 0x" << std::hex << ret << std::endl;
            return -1;
        }
        // set the trigger mode to off
        ret = MV_CC_SetEnumValue(aps_camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
        //Set HB mode to off
        ret = MV_CC_SetEnumValue(aps_camera_handle_, "ImageCompressionMode", 0);

        // set frame rate to 30 FPS
        ret = MV_CC_SetFloatValue(aps_camera_handle_, "AcquisitionFrameRate", fps_);
        std::cout << "frame rate is " << fps_ << std::endl;
        if (ret != MV_OK) {
            std::cout << "Frame rate set failed fail! ret = " << ret << std::endl;
            return -1;
        }

        ret = MV_CC_SetBoolValue(aps_camera_handle_, "AcquisitionFrameRateEnable", true);
        if (ret != MV_OK) {
            std::cout << "Frame rate enable fail! ret = " << ret << std::endl;
            return -1;
        }
        // flip X
        //ret = MV_CC_SetBoolValue(aps_camera_handle_, "ReverseX", true);

        //Auto Exposure
        ret = MV_CC_SetIntValue(aps_camera_handle_, "AutoExposureTimeUpperLimit", 20000);
        ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "ExposureAuto", "Continuous");

        //// Gain
        ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "GainAuto", "Continuous");

        //// Trigger settings
        ret = MV_CC_SetEnumValue(aps_camera_handle_, "LineSelector", 1);
        if (ret != MV_OK) {
            std::cout << "MV_CC_SetEnumValue LineSelector fail! ret = " << ret << std::endl;
        }

        ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "LineMode", "Strobe");
        if (ret != MV_OK) {
            std::cout << "LineMode fail! ret = " << ret << std::endl;
        }

        ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "LineSource", "ExposureStartActive");
        if (ret != MV_OK) {
            std::cout << "MV_CC_SetEnumValue LineSource fail! ret = " << ret << std::endl;
        }

        ret = MV_CC_SetBoolValue(aps_camera_handle_, "StrobeEnable", true);
        if (ret != MV_OK) {
            std::cout << "MV_CC_SetEnable fail! ret = " << ret << std::endl;
        }

        ret = MV_CC_StartGrabbing(aps_camera_handle_);
        if (ret != MV_OK) {
            std::cout << "MV_CC_StartGrabbing fail! ret = " << ret << std::endl;
            return -1;
        }

        is_grab_image_thread_running = true;
        grab_frame_thread = std::thread(
            [this]() {
                while (is_grab_image_thread_running) {
                    cv::Mat frame_image;
                    int ret = get_next_frame(frame_image);
                    if (ret == 0) {
                        {          
                            std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
                            raw_frame_ = std::make_unique<ApsMFrame>(
                                frame_image, 0, 0
                            );
                            aps_frames_.emplace(frame_image);
                            //for (const auto& callback : new_frame_callbacks) {
                            //    callback.second(*raw_frame_);
                            //}
                            LOG_INFO("aps num: %d", aps_num_++);
                        }
                        frame_ready_condition_.notify_one( );
                    }
                    else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                }
            }
        );
        return 0;
    }

    void CameraComboManager::stop_aps_camera() {
        int ret = 0;
        ret = MV_CC_StopGrabbing(aps_camera_handle_);
        if (ret != 0) std::cout << "MV_CC_StopGrabbing failed, error code: " << ret << std::endl;
        ret = MV_CC_CloseDevice(aps_camera_handle_);
        if (ret != 0) std::cout << "MV_CC_CloseDevice failed, error code: " << ret << std::endl;
        ret = MV_CC_DestroyHandle(aps_camera_handle_);
        if (ret != 0) std::cout << "MV_CC_DestroyHandle failed, error code: " << ret << std::endl;
        aps_camera_handle_ = nullptr;
        std::cout << "stop camera" << std::endl;
    }

    bool CameraComboManager::get_aps_frame(cv::Mat& output_frame) {
        if (aps_frames_.empty()) {
            return false;
        }
        aps_frames_.front().copyTo(output_frame);
        aps_frames_.pop();
        return true;
    }

// ---------------------- DVS ---------------------
    int CameraComboManager::find_dvs_camera() {
        //Metavision::DeviceDiscovery::SystemList available_sources = Metavision::DeviceDiscovery::list_available_sources_local();
        dvsense::DvsCameraManager cameraManager;
        std::vector<dvsense::CameraDescription> cameraDescs = cameraManager.getCameraDescs();
        //camera_ = cameraManager.openCamera(cameraDescs[0].serial);

        if (cameraDescs.empty()) {
            std::cout << "No available sources" << std::endl;
            return -1;
        }

        std::cout << cameraDescs.size() << " camera(s) found" << std::endl;
        std::cout << "Serial Number: " << cameraDescs.front().serial << std::endl;

        // init the camera
        dvs_camera_ = cameraManager.openCamera(cameraDescs[0].serial);
        // enable trigger in
        // iggerIn>()->enable(Metavision::I_TriggerIn::Channel::Main);
        std::shared_ptr<dvsense::CameraTool> trigger_in = dvs_camera_->getTool(dvsense::ToolType::TOOL_TRIGGER_IN);
        trigger_in->setParam("enable", true);

        std::shared_ptr<dvsense::CameraTool> erc_tool = dvs_camera_->getTool(dvsense::ToolType::TOOL_EVENT_RATE_CONTROL);
        bool ret = erc_tool->setParam("max_event_rate", 1);
        ret = erc_tool->setParam("enable", true);
        return 0;

        //Metavision::DeviceDiscovery::SystemList available_sources = Metavision::DeviceDiscovery::list_available_sources_local();

        //if (available_sources.empty()) {
        //    std::cout << "No available sources" << std::endl;
        //    return -1;
        //}

        //std::cout << available_sources.size() << " camera(s) found" << std::endl;
        //std::cout << "Serial Number: " << available_sources.front().serial_ << std::endl;

        //// init the camera
        //dvs_camera_ = std::make_unique<Metavision::Camera>(
        //    Metavision::Camera::from_serial(available_sources.front().serial_)
        //);
        //// enable trigger in
        //dvs_camera_->get_device().get_facility<Metavision::I_TriggerIn>()->enable(Metavision::I_TriggerIn::Channel::Main);

        //return 0;
    }

int CameraComboManager::start_dvs_camera() {
    //dvs_camera_->ext_trigger().add_callback(
    //    [this](const Metavision::EventExtTrigger* begin, const Metavision::EventExtTrigger* end) {
    //        //static uint64_t trigger_cnt = 0;
    //        //std::cout << "ext trigger sync callback: " << begin->t << " polarity: " << begin->p 
    //        //    << " trigger_num: " << trigger_cnt++ / 2 << std::endl;
    //        ext_trigger_sync_callback(begin, end);
    //    }
    //);
    dvs_camera_->addTriggerInCallback(
        [this](const dvsense::EventTriggerIn begin) {
            //std::cout << "ext trigger sync callback" << std::endl;
            ext_trigger_sync_callback(&begin);
        }
    );
    dvs_camera_->start();
    return 0;
}

void CameraComboManager::stop_dvs_camera() {   
    dvs_camera_->stop();
}

//int CameraComboManager::register_event_callback(const Metavision::EventsCDCallback& callback) {  
//    return dvs_camera_->cd().add_callback(callback);
//}
int CameraComboManager::register_event_callback(const dvsense::EventsStreamHandleCallback& callback) {
    return dvs_camera_->addEventsStreamHandleCallback(callback);
}

// ---------------------- sync control -------------------------

void CameraComboManager::start_camera(std::string output_dir){
    
    if (!output_dir.empty()) {
        boost::filesystem::path output_dir_path(output_dir);
        if (!boost::filesystem::exists(output_dir_path)) {
            if (!boost::filesystem::create_directory(output_dir_path)) {
                std::cerr << "Directory creation failed" << std::endl;
            }
        }
        start_dvs_camera();

        std::string dvs_file_path_ = (output_dir_path / "fusion-20250624.raw").string();
        //dvs_camera_->startRecording(dvs_file_path_);

        int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
        //cv::Size frameSize(1280, 1024);  
        cv::Size frameSize(2048, 1200);
        std::string aps_file_path_ = (output_dir_path / "fusion-20250624.mp4").string();
        video_wirter_ = std::make_unique<cv::VideoWriter>(
            aps_file_path_, fourcc, 100.0, frameSize
        );
        //   VideoWriter
        if (!video_wirter_->isOpened()) {
            std::cerr << "video_wirter_ is not open\n";
        }

        std::cout << "Writing APS data to " << aps_file_path_
            << ", DVS data to " << std::endl;
        register_frame_callback(
            [this, output_dir_path](const ApsMFrame &frame){
                if (video_wirter_ != nullptr && video_wirter_->isOpened()) {
                    video_wirter_->write(frame.frame);
                    //cv::imwrite(output_dir_path.string() + "/aps.png", frame.frame);
                    std::cout << "write a frame" << std::endl;
                }
                   
            }
        );

        start_aps_camera();

        std::cout << "Saving to " << output_dir << std::endl;
    } else {
        //stop_aps_camera();

        //std::this_thread::sleep_for(std::chrono::seconds(1));
        // start dvs camera




        start_dvs_camera();
        //dvs_camera_->startRecording("./0627.raw");

        //std::this_thread::sleep_for(std::chrono::seconds(8));

        // start aps camera
        start_aps_camera();
    }
    std::cout << "Camera started" << std::endl;
}

std::string CameraComboManager::get_current_time() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm local_time;
    localtime_s(&local_time, &now_time); 

    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y%m%d%H%M");

    return oss.str();
}

void CameraComboManager::start_recoreding(std::string output_dir) {
        boost::filesystem::path output_dir_path(output_dir);
        if (!boost::filesystem::exists(output_dir_path)) {
            if (!boost::filesystem::create_directory(output_dir_path)) {
                std::cerr << "Directory creation failed" << std::endl;
            }
        }
        std::string current_time = get_current_time();
        std::string trigger_file_path = (output_dir_path / ("fusion-" + current_time + ".txt")).string();
        trigger_file_.open(trigger_file_path);
        if (!trigger_file_) {  
            return;
        }
        trigger_write_status_ = true;

        std::string dvs_file_path = (output_dir_path / ("fusion-" + current_time + ".raw")).string();
        /*dvs_camera_->start_recording(dvs_file_path);*/
        dvs_camera_->startRecording(dvs_file_path);

        int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
        //cv::Size frameSize(1280, 1024);  
        cv::Size frameSize(2048, 1200);
        std::string aps_file_path = (output_dir_path / ("fusion-" + current_time + ".mp4")).string();
        video_wirter_ = std::make_unique<cv::VideoWriter>(
            aps_file_path, fourcc, 50.0, frameSize
        );
        //   VideoWriter
        if (!video_wirter_->isOpened()) {
            std::cerr << "video_wirter_ is not open\n";
        }

        std::cout << "Writing APS data to " << aps_file_path
            << ", DVS data to " << dvs_file_path << std::endl;
        register_frame_callback(
            [this, output_dir_path](const ApsMFrame& frame) {
                if (video_wirter_ != nullptr && video_wirter_->isOpened()) {
                    video_wirter_->write(frame.frame);
                    //cv::imwrite(output_dir_path.string() + "/aps.png", frame.frame);
                    //std::cout << "write a frame" << std::endl;
                }

            }
        );
        //start_aps_camera();
        std::cout << "Start Recording, Saving to " << output_dir << std::endl;
}
void CameraComboManager::stop_recoreding() {
    std::cout << "Stop Recording." << std::endl;
    /*dvs_camera_->stop_recording();*/
    dvs_camera_->stopRecording();
    if (video_wirter_ != nullptr) {
        video_wirter_->release();
        video_wirter_.reset();
    } 
    trigger_write_status_ = false;
    trigger_file_.close();
}

void CameraComboManager::ext_trigger_sync_callback(
    //const Metavision::EventExtTrigger* begin, 
    //const Metavision::EventExtTrigger* end
    const dvsense::EventTriggerIn* begin
) {
    // only rising edges are accurate

    if (begin->polarity == 0) return;

    LOG_INFO("aps buffer length: %d ", aps_frames_.size());
    LOG_INFO("trigger num: %d ", trigger_num_++ );
    std::unique_ptr<ApsMFrame> frame;
    {
        
        /*while (aps_frames_.empty()) {*/
        while (aps_frames_.empty()) {
            //std::cout << "empty buffer;" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            //frame_ready_condition_.wait(lock, [this] {return raw_frame_.get() != nullptr; });
        }
        std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
        //std::cout << "get one frame from buffer;" << std::endl;
        frame = std::make_unique<ApsMFrame>(aps_frames_.front());
        aps_frames_.pop();
    }

    Timestamp t = begin->timestamp;
    //std::cout << t << std::endl;
    /////////Ð´Èëtriggerin
    //if(trigger_write_status_)
    //{
    //    trigger_file_ << std::to_string(t) << '\n'; 
    //}
    ///////½áÊøÐ´Èë
    //if (!aps_frames_.empty()) {
    //    std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
    //    //std::cout << "get one frame from buffer;" << std::endl;
    //    frame = std::make_unique<ApsMFrame>(aps_frames_.front());
    //    aps_frames_.pop();
    //    frame->exposure_start_timestamp = t;
    //    for (const auto& callback : new_frame_callbacks) {
    //        callback.second(*frame);
    //    }
    //}
    frame->exposure_start_timestamp = t;
    for (const auto& callback : new_frame_callbacks) {
        callback.second(*frame);
    }
    this->last_timestamp_ = begin->timestamp;
}

void CameraComboManager::stop_camera() {
    is_grab_image_thread_running = false;
    if (grab_frame_thread.joinable()) grab_frame_thread.join();

    stop_dvs_camera();
    //dvs_camera_->stopRecording();
    stop_aps_camera();

    if (video_wirter_ != nullptr) {
        video_wirter_->release();
        video_wirter_.reset();
        //dvs_camera_->stopRecording();
    }
}

} // namespace dvsense
