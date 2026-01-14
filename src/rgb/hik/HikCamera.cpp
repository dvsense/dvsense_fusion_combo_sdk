#include "DvsRgbFusionCamera/rgb/hik/HikCamera.hpp"
#include <iostream>

HikCamera::HikCamera(float fps) : fps_(fps) {
    is_grab_image_thread_running_ = false;
    frame_callback_id_num_ = 0;
    last_frame_id_ = -1;
    int ret = MV_CC_Initialize();
    if (ret != MV_OK) {
        std::cout << "MV_CC_Initialize fail! ret = " << ret << std::endl;
    }
}

HikCamera::~HikCamera()
{
}


bool HikCamera::isConnect()
{
    return MV_CC_IsDeviceConnected(aps_camera_handle_);
}

bool HikCamera::findCamera(std::vector<std::string>& serial_numbers) {
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
            std::cout << i + 1 << " aps camera(s) found" << std::endl;
            std::cout << "Serial Number: " << p_device_info->SpecialInfo.stUsb3VInfo.chSerialNumber << std::endl;
            serial_numbers.push_back(reinterpret_cast<const char*>(p_device_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
        }
    }
    else {
        std::cout << "Find No Devices!" << std::endl;
        return false;
    }
    return true;
}

bool HikCamera::openCamera(std::string serial_number) {
    MV_CC_DEVICE_INFO_LIST mvs_device_info_list;
    memset(&mvs_device_info_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &mvs_device_info_list);
    if (ret != MV_OK) {
        std::cout << "MV_CC_EnumDevices fail! ret = " << ret << std::endl;
        return false;
    }
    // print out the device information
    int input_serial_number = -1;
    if (mvs_device_info_list.nDeviceNum > 0) {
        for (unsigned int i = 0; i < mvs_device_info_list.nDeviceNum; i++) {
            MV_CC_DEVICE_INFO* p_device_info = mvs_device_info_list.pDeviceInfo[i];
            if (p_device_info == nullptr) {
                break;
            }
            std::string hik_number = reinterpret_cast<const char*>(p_device_info->SpecialInfo.stUsb3VInfo.chSerialNumber);
            if (hik_number == serial_number) 
            {
                input_serial_number = i;
                break;
            }
        }
        if (input_serial_number == -1)
        {
            std::cout << "Aps camera input serial number no find!" << std::endl;
            return false;
        } 
    }
    else {
        std::cout << "Find No Devices!" << std::endl;
        return false;
    }

    ret = MV_CC_CreateHandle(&aps_camera_handle_, mvs_device_info_list.pDeviceInfo[input_serial_number]);
    if (ret != MV_OK) {
        std::cout << "MV_CC_CreateHandle fail! ret = " << ret << std::endl;
        return false;
    }

    ret = MV_CC_OpenDevice(aps_camera_handle_);
    if (ret != MV_OK) {
        std::cout << "MV_CC_OpenDevice fail! ret = 0x" << std::hex << ret << std::endl;
        return -1;
    }

    ret = MV_CC_SetImageNodeNum(aps_camera_handle_, 50);
    if (ret != MV_OK) {
        std::cout << "Set image node num failed fail! ret = " << ret << std::endl;
        return -1;
    }

    // set the trigger mode to off
    ret = MV_CC_SetEnumValue(aps_camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    //Set HB mode to off
    ret = MV_CC_SetEnumValue(aps_camera_handle_, "ImageCompressionMode", 0);

    // set frame rate
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
    int auto_exposure_time = 1000000 / fps_;
    ret = MV_CC_SetIntValue(aps_camera_handle_, "AutoExposureTimeUpperLimit", auto_exposure_time);
    ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "ExposureAuto", "Continuous");

    //// Gain
    ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "GainAuto", "Continuous");

    //// Trigger settings
    ret = MV_CC_SetEnumValue(aps_camera_handle_, "LineSelector", 1);
    if (ret != MV_OK) {
        std::cout << "MV_CC_SetEnumValue LineSelector fail! ret = " << ret << std::endl;
    }

    ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "LineMode", "Strobe");
    
    ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "LineSource", "ExposureStartActive");
    if (ret != MV_OK) {
        std::cout << "MV_CC_SetEnumValue LineSource fail! ret = " << ret << std::endl;
    }

    ret = MV_CC_SetBoolValue(aps_camera_handle_, "StrobeEnable", true);
    if (ret != MV_OK) {
        std::cout << "MV_CC_SetEnable fail! ret = " << ret << std::endl;
    }

    // Here setting the width is for better compatibility when using ffmpeg which could not handle the width that is not divisible by 4 in some cases.
    int64_t set_width = int(getWidth() / 4) * 4;
    MV_CC_SetIntValueEx(aps_camera_handle_, "Width", set_width);

    return true;
}

bool HikCamera::openExternalTrigger()
{
    bool ret = MV_CC_SetEnumValue(aps_camera_handle_, "AcquisitionMode", 2);
    ret = MV_CC_SetEnumValue(aps_camera_handle_, "TriggerSource", 0);
    ret = MV_CC_SetEnumValue(aps_camera_handle_, "TriggerMode", 1);
    ret = MV_CC_SetCommandValue(aps_camera_handle_, "AcquisitionStart");

    //Set HB mode to off
    ret = MV_CC_SetEnumValue(aps_camera_handle_, "ImageCompressionMode", 0);
    // set frame rate
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

    //// Gain
    ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "GainAuto", "Continuous");

    //// Trigger settings
    ret = MV_CC_SetEnumValue(aps_camera_handle_, "LineSelector", 1);
    if (ret != MV_OK) {
        std::cout << "MV_CC_SetEnumValue LineSelector fail! ret = " << ret << std::endl;
    }

    ret = MV_CC_SetEnumValueByString(aps_camera_handle_, "LineMode", "Strobe");

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
    dvsense::ApsFrame& rgb_frame
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
        // TODO: handle mono case
    }
    else {
        // convert to bgr
        MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam = {
            frame_out_.stFrameInfo.nWidth,                      // nWidth
            frame_out_.stFrameInfo.nHeight,                     // nHeight
            frame_out_.stFrameInfo.enPixelType,                 // enPixelType
            const_cast<unsigned char*>(frame_out_.pBufAddr),    // pSrcData
            frame_out_.stFrameInfo.nFrameLen,                   // nSrcDataLen
            PixelType_Gvsp_BGR8_Packed,                         // enDstPixelType
            //pDstData,                                         // pDstBuffer
            rgb_frame.data(),
            0,                                                  // nDstLen
            rgb_frame.getDataSize(),                                         // nDstBufferSize
            0
        };
        MV_CC_ConvertPixelTypeEx(aps_camera_handle_, &stConvertParam);
    }
}

int HikCamera::getNextFrame(dvsense::ApsFrame& rgb_frame, int& drop_frame_num) {
    int ret = MV_CC_GetImageBuffer(aps_camera_handle_, &frame_out_, 1000);
    if (ret != MV_OK) {
        rgb_frame = dvsense::ApsFrame(0, 0);//size
        return -1;
    }
    else
    {
        unsigned long long current_frame_id = frame_out_.stFrameInfo.nFrameNum;
        drop_frame_num = current_frame_id - last_frame_id_ - 1;
        if (drop_frame_num > 0)
        {
            std::cout << "Warning! Aps drop a frame." << std::endl;
        }
        last_frame_id_ = current_frame_id;
    }

    bufferToMat(rgb_frame);

    ret = MV_CC_FreeImageBuffer(aps_camera_handle_, &frame_out_);
    if (ret != MV_OK) {
        std::cout << "MV_CC_ReleaseImageBuffer fail! ret = " << ret << std::endl;
        rgb_frame = dvsense::ApsFrame(0, 0);
        return -1;
    }
    return 0;
}

int HikCamera::startCamera() {
    //MV_CC_SetBoolValue(aps_camera_handle_, "LineInverter", false);
    last_frame_id_ = -1;
    frames_buffer_ = std::queue<dvsense::ApsFrame>();
    int aps_width = getWidth();
    int aps_height = getHeight();
    //Ԥ��
    for (int i = 0; i < 3; i++) 
    {
        frames_buffer_.emplace(dvsense::ApsFrame(0, 0));
    }

    is_grab_image_thread_running_ = true;
    MVCC_ENUMVALUE stEnumValue = { 0 };
    MV_CC_GetEnumValue(aps_camera_handle_, "TriggerMode", &stEnumValue);
    if (stEnumValue.nCurValue == 0)
    {
        bool ret = MV_CC_StartGrabbing(aps_camera_handle_);
        if (ret != MV_OK) {
            std::cout << "MV_CC_StartGrabbing fail! ret = " << ret << std::endl;
            return -1;
        }
        while (is_grab_image_thread_running_)
        {
            int drop_nums;
            dvsense::ApsFrame rgb_frame(aps_width, aps_height);
            int ret = getNextFrame(rgb_frame, drop_nums);
            if (ret == 0)
            {
                int ret = MV_CC_StopGrabbing(aps_camera_handle_);
                break;
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    frames_buffer_ = std::queue<dvsense::ApsFrame>();

    bool ret = MV_CC_StartGrabbing(aps_camera_handle_);
    grab_frame_thread_ = std::thread(
        [this, aps_width, aps_height]() {
            while (is_grab_image_thread_running_) {

                int drop_nums;
                dvsense::ApsFrame rgb_frame(aps_width, aps_height);
                int ret = getNextFrame(rgb_frame, drop_nums);
                if (ret == 0) {
                    {
                        std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
                        for (int i = 0; i < drop_nums; i++)
                        {
                            frames_buffer_.emplace(dvsense::ApsFrame(0, 0));
                        }
                        frames_buffer_.emplace(rgb_frame);
                    }
                }
                else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        }
    );
    return 0;
}

int HikCamera::getWidth()
{
    MVCC_INTVALUE_EX stIntValue {0, 0, 0, 0, 0};
    MV_CC_GetIntValueEx(aps_camera_handle_, "Width", &stIntValue);
    return stIntValue.nCurValue;
}

int HikCamera::getHeight()
{
    MVCC_INTVALUE_EX stIntValue = {0, 0, 0, 0, 0};
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

bool HikCamera::getNewRgbFrame(dvsense::ApsFrame& output_frame) {
    std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
    if (frames_buffer_.empty())
    {
        return false;
    }
    else
    {
        output_frame = frames_buffer_.front();
        frames_buffer_.pop();
        return true;
    }
    return true;
}