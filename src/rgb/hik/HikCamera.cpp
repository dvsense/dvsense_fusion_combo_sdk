#include "hik/HikCamera.hpp"

HikCamera::~HikCamera()
{
}

bool HikCamera::findCamera() {
    MV_CC_DEVICE_INFO_LIST mvs_device_info_list;
    memset(&mvs_device_info_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &mvs_device_info_list);
    if (ret != MV_OK) {
        std::cout << "MV_CC_EnumDevices fail! ret = " << ret << std::endl;
        return false;
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
        return false;
    }

    std::cout << "Use the first device to create handle" << std::endl;
    ret = MV_CC_CreateHandle(&aps_camera_handle_, mvs_device_info_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        std::cout << "MV_CC_CreateHandle fail! ret = " << ret << std::endl;
        return false;
    }

    ret = MV_CC_OpenDevice(aps_camera_handle_);
    if (ret != MV_OK) {
        std::cout << "MV_CC_OpenDevice fail! ret = 0x" << std::hex << ret << std::endl;
        return -1;
    }

    ret = MV_CC_SetImageNodeNum(aps_camera_handle_, 100);
    if (ret != MV_OK) {
        std::cout << "Set image node num failed fail! ret = " << ret << std::endl;
        return -1;
    }

    // set the trigger mode to off
    ret = MV_CC_SetEnumValue(aps_camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    //Set HB mode to off
    ret = MV_CC_SetEnumValue(aps_camera_handle_, "ImageCompressionMode", 0);

    // set frame rate to 30 FPS
    ret = MV_CC_SetFloatValue(aps_camera_handle_, "AcquisitionFrameRate", fps_);
    std::cout << "Aps frame fps is: " << fps_ << std::endl;
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
    ret = MV_CC_SetIntValue(aps_camera_handle_, "AutoExposureTimeUpperLimit", 15000);
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

    return true;
}

void HikCamera::bufferToMat(
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
        //frame = cv::Mat(frame_out_.stFrameInfo.nHeight, frame_out_.stFrameInfo.nWidth, 16,
        //    pDstData).clone();
        cv::Mat(frame_out_.stFrameInfo.nHeight, frame_out_.stFrameInfo.nWidth, 16, pDstData).copyTo(frame);
        free(pDstData);
        pDstData = NULL;
    }
}

int HikCamera::getNextFrame(FrameAndDrop& frame_and_drops) {
    int ret = MV_CC_GetImageBuffer(aps_camera_handle_, &frame_out_, 1000);
    if (ret != MV_OK) {
        std::cout << "MV_CC_GetOneFrameTimeout fail! ret = " << std::to_string(ret) << std::endl;
        frame_and_drops.frame = cv::Mat(frame_out_.stFrameInfo.nHeight, frame_out_.stFrameInfo.nWidth, CV_8UC1);
        return -1;
    }
    else
    {
        unsigned long long current_frame_id = frame_out_.stFrameInfo.nFrameNum;
        //std::cout << "currentFrameID: " << current_frame_id << std::endl;
        frame_and_drops.drop_frame_num = current_frame_id - last_frame_id_ - 1;
        if (frame_and_drops.drop_frame_num > 0)
        {
            std::cout << "Aps drop a frame" << std::endl;
        }
        last_frame_id_ = current_frame_id;
    }

    bufferToMat(frame_and_drops.frame);

    ret = MV_CC_FreeImageBuffer(aps_camera_handle_, &frame_out_);
    if (ret != MV_OK) {
        std::cout << "MV_CC_ReleaseImageBuffer fail! ret = " << ret << std::endl;
        frame_and_drops.frame = cv::Mat(frame_out_.stFrameInfo.nHeight, frame_out_.stFrameInfo.nWidth, CV_8UC1);
        return -1;
    }
    return 0;
}

int HikCamera::startCamera() {
    int ret = MV_CC_StartGrabbing(aps_camera_handle_);
    if (ret != MV_OK) {
        std::cout << "MV_CC_StartGrabbing fail! ret = " << ret << std::endl;
        return -1;
    }
    
    is_grab_image_thread_running_ = true;

    grab_frame_thread_ = std::thread(
        [this]() {
            static int frame_num = 0;
            
            while (is_grab_image_thread_running_) {
                FrameAndDrop new_frame_drop;
                int ret = getNextFrame(new_frame_drop);
                if (ret == 0) {
                    {
                        std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
                        aps_frames_drop_.emplace(new_frame_drop);
                    }
                }
                else {
                    std::cout << "when getNextFrame, ret is -1" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        }
    );
    return 0;
}

int HikCamera::getWidth()
{
    MVCC_INTVALUE_EX stIntValue = { 0 };
    MV_CC_GetIntValueEx(aps_camera_handle_, "Width", &stIntValue);
    return stIntValue.nCurValue;
}

int HikCamera::getHeight()
{
    MVCC_INTVALUE_EX stIntValue = { 0 };
    MV_CC_GetIntValueEx(aps_camera_handle_, "Height", &stIntValue);
    return stIntValue.nCurValue;
}

void HikCamera::stopCamera() {
    is_grab_image_thread_running_ = false;
    if (grab_frame_thread_.joinable()) grab_frame_thread_.join();
    int ret = 0;
    ret = MV_CC_StopGrabbing(aps_camera_handle_);
    if (ret != 0) std::cout << "MV_CC_StopGrabbing failed, error code: " << ret << std::endl;
}

int HikCamera::destroyCamera() {
    is_grab_image_thread_running_ = false;
    if (grab_frame_thread_.joinable()) grab_frame_thread_.join();
    int ret = 0;
    MV_CC_StopGrabbing(aps_camera_handle_);
    ret = MV_CC_CloseDevice(aps_camera_handle_);
    if (ret != 0) std::cout << "MV_CC_CloseDevice failed, error code: " << ret << std::endl;
    ret = MV_CC_DestroyHandle(aps_camera_handle_);
    if (ret != 0) std::cout << "MV_CC_DestroyHandle failed, error code: " << ret << std::endl;
    aps_camera_handle_ = nullptr;
    return ret;
}

bool HikCamera::getNewRgbFrame(cv::Mat& output_frame) {
    if (!private_buffer_frames_.empty()) {
        //private_buffer_frames_.front().copyTo(output_frame);
        output_frame = private_buffer_frames_.front();
        private_buffer_frames_.pop();
        return true;
    }

    if (aps_frames_drop_.empty())
    {
        return false;
    }
    std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
    FrameAndDrop frame_drops = aps_frames_drop_.front();
    if (frame_drops.drop_frame_num == 0)
    {
        //frame_drops.frame.copyTo(output_frame);
        output_frame = frame_drops.frame;
    }
    else
    {
        output_frame = cv::Mat();
        for (int i = 0; i < frame_drops.drop_frame_num - 1; i++)
        {
            private_buffer_frames_.emplace();
        }
        private_buffer_frames_.emplace(frame_drops.frame);
    }
    aps_frames_drop_.pop();

    //std::cout << "triggerin aps_frames size: " << aps_frames_drop_.size() << std::endl;
    return true;
}