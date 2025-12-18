#include "DvsRgbFusionCamera/rgb/daheng/DahengCamera.hpp"
#include <iostream>
#include <chrono>
#include <cstring>

DahengCamera::DahengCamera(float fps)
    : fps_(fps),
      is_grab_image_thread_running_(false),
      frame_callback_id_num_(0),
      last_frame_id_(-1),
      device_handle_(nullptr),
      ds_handle_(nullptr),
      payload_size_(0),
      rgb_image_buf_(nullptr),
      raw8_image_buf_(nullptr),
      color_filter_supported_(false),
      color_filter_type_(GX_COLOR_FILTER_NONE) {
    GXCloseLib();
    GX_STATUS st = GXInitLib();
    if (st != GX_STATUS_SUCCESS) {
        std::cout << "GXInitLib fail, status = " << st << std::endl;
    }
}

DahengCamera::~DahengCamera() {
    GXCloseLib();
    // 不在析构中强制 destroyCamera，交由调用方控制，与 Hik 行为保持一致
}

bool DahengCamera::isConnect() {
    // Galaxy 无直接“是否连接”API，这里以 device_handle_ 是否非空近似判断
    return device_handle_ != nullptr;
}

bool DahengCamera::findCamera(std::vector<std::string>& serial_numbers) {
    serial_numbers.clear();

    GX_STATUS st = GX_STATUS_SUCCESS;
    uint32_t dev_num = 0;
    st = GXUpdateAllDeviceList(&dev_num, 1000);
    if (st != GX_STATUS_SUCCESS) {
        std::cout << "GXUpdateAllDeviceList fail, status = " << st << std::endl;
        return false;
    }
    if (dev_num == 0) {
        std::cout << "Find No Devices!" << std::endl;
        return false;
    }

    // 为了避免依赖“获取基础信息数组”的接口差异，这里逐个打开读取序列号
    for (uint32_t i = 1; i <= dev_num; ++i) {
        GX_DEVICE_INFO stDeviceInfo;
        GXGetDeviceInfo(i, &stDeviceInfo);

        if (stDeviceInfo.emDevType == GX_DEVICE_CLASS_UNKNOWN) {
            continue;
        }
        else if(stDeviceInfo.emDevType == GX_DEVICE_CLASS_U3V) {
            serial_numbers.emplace_back((char*)stDeviceInfo.DevInfo.stU3VDevInfo.chSerialNumber);
        }
        else if(stDeviceInfo.emDevType = GX_DEVICE_CLASS_GEV) {
            serial_numbers.emplace_back((char*)stDeviceInfo.DevInfo.stGEVDevInfo.chSerialNumber);
        }
        else if(stDeviceInfo.emDevType = GX_DEVICE_CLASS_CXP) {
            serial_numbers.emplace_back((char*)stDeviceInfo.DevInfo.stCXPDevInfo.chSerialNumber);
        }
        else if(stDeviceInfo.emDevType = GX_DEVICE_CLASS_USB2) {
            serial_numbers.emplace_back((char*)stDeviceInfo.DevInfo.stUSBDevInfo.chSerialNumber);
        }
        else {
            continue;
        }
    }

    return !serial_numbers.empty();
}

bool DahengCamera::openCamera(std::string serial_number) {
    GX_STATUS st = GX_STATUS_SUCCESS;

    // 枚举并打开匹配序列号的设备
    {
        uint32_t dev_num = 0;
        st = GXUpdateAllDeviceList(&dev_num, 1000);
        if (st != GX_STATUS_SUCCESS || dev_num == 0) {
            std::cout << "GXUpdateAllDeviceList fail or no device, status=" << st << std::endl;
            return false;
        }

        auto tryOpen = [](uint32_t i, GX_DEVICE_INFO& info, GX_DEV_HANDLE& handle) -> bool {
            GX_STATUS st = GXOpenDeviceByIndex(i, &handle);
            if (st != GX_STATUS_SUCCESS) {
                std::cout << "GXOpenDeviceByIndex fail, status = " << st << std::endl;
                return false;
            }

            return true;
        };

        bool opened = false;
        for (uint32_t i = 1; i <= dev_num; ++i) {
            GX_DEVICE_INFO stDeviceInfo;
            GXGetDeviceInfo(i, &stDeviceInfo);
            if(stDeviceInfo.emDevType == GX_DEVICE_CLASS_UNKNOWN) {
                continue;
            }
            else if(stDeviceInfo.emDevType == GX_DEVICE_CLASS_U3V) {
                std::string sn = std::string((char*)stDeviceInfo.DevInfo.stU3VDevInfo.chSerialNumber);
                if(sn == serial_number) {
                    opened = tryOpen(i, stDeviceInfo, device_handle_);
                    break;
                } else {
                    continue;
                }
            }
            else if(stDeviceInfo.emDevType = GX_DEVICE_CLASS_GEV) {
                std::string sn = std::string((char*)stDeviceInfo.DevInfo.stGEVDevInfo.chSerialNumber);
                if(sn == serial_number) {
                    opened = tryOpen(i, stDeviceInfo, device_handle_);
                    break;
                } else {
                    continue;
                }
            }
            else if(stDeviceInfo.emDevType = GX_DEVICE_CLASS_CXP) {
                std::string sn = std::string((char*)stDeviceInfo.DevInfo.stCXPDevInfo.chSerialNumber);
                if(sn == serial_number) {
                    opened = tryOpen(i, stDeviceInfo, device_handle_);
                    break;
                } else {
                    continue;
                }
            }
            else if(stDeviceInfo.emDevType = GX_DEVICE_CLASS_USB2) {
                std::string sn = std::string((char*)stDeviceInfo.DevInfo.stUSBDevInfo.chSerialNumber);
                if(sn == serial_number) {
                    opened = tryOpen(i, stDeviceInfo, device_handle_);
                    break;
                } else {
                    continue;
                }
            }
            else {
                continue;
            }
        }
        if (!opened) {
            std::cout << "Gx camera input serial number not found!" << std::endl;
            return false;
        }
    }

    // 设备信息与彩色能力
    {
        GX_NODE_ACCESS_MODE am = GX_NODE_ACCESS_MODE_NA;
        st = GXGetNodeAccessMode(device_handle_, "PixelColorFilter", &am);
        if (st == GX_STATUS_SUCCESS) {
            color_filter_supported_ = (am == GX_NODE_ACCESS_MODE_WO || am == GX_NODE_ACCESS_MODE_RO || am == GX_NODE_ACCESS_MODE_RW);
            if (color_filter_supported_) {
                GX_ENUM_VALUE ev{};
                st = GXGetEnumValue(device_handle_, "PixelColorFilter", &ev);
                if (st == GX_STATUS_SUCCESS) {
                    color_filter_type_ = ev.stCurValue.nCurValue;
                }
            }
        }
    }

    // 数据流与负载
    {
        uint32_t dsNum = 0;
        st = GXGetDataStreamNumFromDev(device_handle_, &dsNum);
        if (st != GX_STATUS_SUCCESS || dsNum < 1) {
            std::cout << "GXGetDataStreamNumFromDev fail or zero" << std::endl;
            return false;
        }
        st = GXGetDataStreamHandleFromDev(device_handle_, 1, &ds_handle_);
        if (st != GX_STATUS_SUCCESS) {
            std::cout << "GXGetDataStreamHandleFromDev fail" << std::endl;
            return false;
        }
        st = GXGetPayLoadSize(ds_handle_, &payload_size_);
        if (st != GX_STATUS_SUCCESS) {
            std::cout << "GXGetPayLoadSize fail" << std::endl;
            return false;
        }
    }

    // 基本参数对齐（与 Hik 行为相近）
    trySetEnumByString("AcquisitionMode", "Continuous");
    trySetEnumByString("TriggerMode", "Off");

    // 帧率设置（不同 SDK 节点名略有差异，这里尽力覆盖）
    trySetEnumByString("AcquisitionFrameRateMode", "On");
    trySetFloat("AcquisitionFrameRate", fps_);
    std::cout << "Gx frame fps is: " << fps_ << std::endl;

    // 自动曝光/增益/白平衡，与 Hik 对齐
    float auto_exposure_time = 1000000.0 / fps_;
    trySetEnumByString("ExposureAuto", "Continuous");
    trySetFloat("AutoExposureTimeMin", auto_exposure_time * 0.5);
    trySetFloat("AutoExposureTimeMax", auto_exposure_time * 0.85);
    trySetEnumByString("GainAuto", "Continuous");
    trySetEnumByString("BalanceWhiteAuto", "Once"); // 或 "Continuous"，视需求

    // Trigger out
    trySetEnumByString("LineSelector", "Line1");
    trySetEnumByString("LineMode", "Output");
    trySetEnumByString("LineSource", "ExposureActive");

    GX_FLOAT_VALUE stFloatValue;
    auto emStatus = GXGetFloatValue(device_handle_, "ExposureTime", &stFloatValue);
    double dExposureTime = stFloatValue.dCurValue;
    std::cout << "Current exposure time: " << dExposureTime << std::endl;

    // 使宽度可被 4 整除（与 Hik 兼容 ffmpeg 的考虑一致）
    {
        int64_t w = 0;
        if (tryGetInt("Width", w) && w > 0) {
            int64_t set_w = (w / 4) * 4;
            if (set_w != w) {
                trySetInt("Width", set_w);
            }
        }
    }

    // 分配转换缓存
    preForAcquisition();

    return true;
}

bool DahengCamera::openExternalTrigger() {
    // Galaxy 与 Hik 节点不同：尽量开启外触发（Line0 上升沿），其余参数尽量对齐
    trySetEnumByString("AcquisitionMode", "Continuous");

    // TriggerMode On, Source Line0（不同机型可为 Line1/Line2/Ext）
    trySetEnumByString("TriggerMode", "On");
    trySetEnumByString("TriggerSource", "Line0");
    trySetEnumByString("TriggerActivation", "RisingEdge"); // 若不支持可忽略

    // Trigger out
    trySetEnumByString("LineSelector", "Line1");
    trySetEnumByString("LineMode", "Output");
    trySetEnumByString("LineSource", "ExposureActive");

    // 帧率节点与自动增益
    trySetEnumByString("GainAuto", "Continuous");
    trySetEnumByString("BalanceWhiteAuto", "Once");
    trySetEnumByString("ExposureAuto", "Off"); // 外触发时通常手动曝光，必要时可调整
    trySetEnumByString("AcquisitionFrameRateMode", "On");
    trySetFloat("AcquisitionFrameRate", fps_);
    std::cout << "Gx frame fps is: " << fps_ << std::endl;

    return true;
}

int DahengCamera::bufferToMat(PGX_FRAME_BUFFER pFrameBuffer, dvsense::ApsFrame& rgb_frame) {
    if (!pFrameBuffer || !rgb_image_buf_) return -1;

    VxInt32 dxst = DX_OK;

    // 按 Bayer 位深转换到 RGB24
    switch (pFrameBuffer->nPixelFormat) {
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8: {
            dxst = DxRaw8toRGB24Ex(
                (unsigned char*)pFrameBuffer->pImgBuf,
                rgb_image_buf_,
                pFrameBuffer->nWidth,
                pFrameBuffer->nHeight,
                RAW2RGB_NEIGHBOUR,
                DX_PIXEL_COLOR_FILTER(color_filter_type_),
                false,
                DX_RGB_CHANNEL_ORDER::DX_ORDER_BGR
            );
            if (dxst != DX_OK) {
                std::cout << "DxRaw8toRGB24 Failed: " << dxst << std::endl;
                return -1;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12: {
            if (!raw8_image_buf_) return -1;
            dxst = DxRaw16toRaw8(
                (unsigned char*)pFrameBuffer->pImgBuf,
                raw8_image_buf_,
                pFrameBuffer->nWidth,
                pFrameBuffer->nHeight,
                DX_BIT_2_9
            );
            if (dxst != DX_OK) {
                std::cout << "DxRaw16toRaw8 Failed: " << dxst << std::endl;
                return -1;
            }
            dxst = DxRaw8toRGB24Ex(
                raw8_image_buf_,
                rgb_image_buf_,
                pFrameBuffer->nWidth,
                pFrameBuffer->nHeight,
                RAW2RGB_NEIGHBOUR,
                DX_PIXEL_COLOR_FILTER(color_filter_type_),
                false,
                DX_RGB_CHANNEL_ORDER::DX_ORDER_BGR
            );
            if (dxst != DX_OK) {
                std::cout << "DxRaw8toRGB24 Failed: " << dxst << std::endl;
                return -1;
            }
            break;
        }
        default:
            std::cout << "Unsupported PixelFormat: " << pFrameBuffer->nPixelFormat << std::endl;
            return -1;
    }

    // Galaxy 转换结果为 RGB24，这里做通道交换到 BGR8 以对齐 Hik 封装输出
    const size_t pixels = static_cast<size_t>(pFrameBuffer->nWidth) * static_cast<size_t>(pFrameBuffer->nHeight);
    unsigned char* dst = rgb_frame.data();
    if (!dst) return -1;

    // 若 ApsFrame 的缓冲大小不足，这里假定构造时已分配 w*h*3
    memcpy(dst, rgb_image_buf_, pixels * 3 * sizeof(uint8_t));

    return 0;
}

int DahengCamera::getNextFrame(dvsense::ApsFrame& rgb_frame, int& drop_frame_num) {
    if (!device_handle_) return -1;

    GX_STATUS st = GX_STATUS_SUCCESS;
    PGX_FRAME_BUFFER pFB = nullptr;
    st = GXDQBuf(device_handle_, &pFB, 200);
    if (st != GX_STATUS_SUCCESS) {
        if (st != GX_STATUS_TIMEOUT) {
            std::cout << "GXDQBuf fail, status = " << st << std::endl;
        }
        return -1;
    }

    int ret = -1;
    if (pFB->nStatus == GX_FRAME_STATUS_SUCCESS) {
        // 计算丢帧
        unsigned long long cur_id = pFB->nFrameID;
        drop_frame_num = (last_frame_id_ < 0) ? 0 : static_cast<int>(cur_id - static_cast<unsigned long long>(last_frame_id_) - 1ULL);
        if (drop_frame_num > 0) {
            std::cout << "Warning! Gx drop a frame." << std::endl;
        }
        last_frame_id_ = static_cast<long long>(cur_id);

        ret = bufferToMat(pFB, rgb_frame);
    } else {
        std::cout << "<Abnormal Acquisition: code " << pFB->nStatus << ">" << std::endl;
    }

    GX_STATUS stQ = GXQBuf(device_handle_, pFB);
    if (stQ != GX_STATUS_SUCCESS) {
        std::cout << "GXQBuf fail, status = " << stQ << std::endl;
    }

    return (ret == 0) ? 0 : -1;
}

int DahengCamera::startCamera() {
    if (!device_handle_) return -1;

    last_frame_id_ = -1;
    frames_buffer_ = std::queue<dvsense::ApsFrame>();

    // 预推入空帧（与 Hik 行为一致）
    //for (int i = 0; i < 3; ++i) {
    //    frames_buffer_.emplace(dvsense::ApsFrame(0, 0));
    //}

    // 开流
#ifdef _WIN32
    GX_STATUS st = GXSetCommandValue(device_handle_, "AcquisitionStart");
#else
    GX_STATUS st = GXStreamOn(device_handle_);
#endif
    if (st != GX_STATUS_SUCCESS) {
        std::cout << "GXStreamOn fail, status = " << st << std::endl;
        return -1;
    }

    // 获取尺寸以便为 ApsFrame 分配
    int aps_width = getWidth();
    int aps_height = getHeight();

    is_grab_image_thread_running_ = true;
    grab_frame_thread_ = std::thread([this, aps_width, aps_height]() {
        while (is_grab_image_thread_running_) {
            int drop_nums = 0;
            dvsense::ApsFrame rgb_frame(aps_width, aps_height);
            int ret = getNextFrame(rgb_frame, drop_nums);
            if (ret == 0) {
                std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
                for (int i = 0; i < drop_nums; ++i) {
                    frames_buffer_.emplace(dvsense::ApsFrame(0, 0));
                }
                frames_buffer_.emplace(rgb_frame);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    });

    return 0;
}

int DahengCamera::getWidth() {
    int64_t v = 0;
    return (tryGetInt("Width", v) ? static_cast<int>(v) : 0);
}

int DahengCamera::getHeight() {
    int64_t v = 0;
    return (tryGetInt("Height", v) ? static_cast<int>(v) : 0);
}

void DahengCamera::stopCamera() {
    is_grab_image_thread_running_ = false;
    if (grab_frame_thread_.joinable()) grab_frame_thread_.join();

    if (device_handle_) {
#ifdef _WIN32
        GX_STATUS st = GXSetCommandValue(device_handle_, "AcquisitionStop");
#else
        GX_STATUS st = GXStreamOff(device_handle_);
#endif
        if (st != GX_STATUS_SUCCESS) {
            std::cout << "GXStreamOff failed, status = " << st << std::endl;
        }
    }
}

int DahengCamera::destroyCamera() {
    stopCamera();

    unPreForAcquisition();

    if (device_handle_) {
        GX_STATUS st = GXCloseDevice(device_handle_);
        if (st != GX_STATUS_SUCCESS) {
            std::cout << "GXCloseDevice failed, status = " << st << std::endl;
        }
        device_handle_ = nullptr;
        ds_handle_ = nullptr;
    }

    GX_STATUS st2 = GXCloseLib();
    if (st2 != GX_STATUS_SUCCESS) {
        std::cout << "GXCloseLib failed, status = " << st2 << std::endl;
    }
    return static_cast<int>(st2);
}

bool DahengCamera::getNewRgbFrame(dvsense::ApsFrame& output_frame) {
    std::unique_lock<std::mutex> lock(frame_buffer_mutex_);
    if (frames_buffer_.empty()) {
        return false;
    } else {
        output_frame = frames_buffer_.front();
        frames_buffer_.pop();
        return true;
    }
}

// 缓冲管理
void DahengCamera::preForAcquisition() {
    if (payload_size_ == 0) return;
    if (!rgb_image_buf_) rgb_image_buf_ = new unsigned char[payload_size_ * 3];
    if (!raw8_image_buf_) raw8_image_buf_ = new unsigned char[payload_size_];
}

void DahengCamera::unPreForAcquisition() {
    if (raw8_image_buf_) {
        delete[] raw8_image_buf_;
        raw8_image_buf_ = nullptr;
    }
    if (rgb_image_buf_) {
        delete[] rgb_image_buf_;
        rgb_image_buf_ = nullptr;
    }
}

// 工具函数（容错设置）
void DahengCamera::trySetEnumByString(const char* key, const char* val) {
    if (!device_handle_) return;

    GX_STATUS st = GXSetEnumValueByString(device_handle_, key, val);
    (void)st; // 静默失败
}
void DahengCamera::trySetFloat(const char* key, double val) {
    if (!device_handle_) return;
    GX_STATUS st = GXSetFloatValue(device_handle_, key, val);
    (void)st;
}
void DahengCamera::trySetInt(const char* key, int64_t val) {
    if (!device_handle_) return;
    GX_STATUS st = GXSetIntValue(device_handle_, key, val);
    (void)st;
}
void DahengCamera::trySetBool(const char* key, bool val)
{
    if (!device_handle_) return;
    GX_STATUS st = GXSetBoolValue(device_handle_, key, val);
    (void)st;
}
bool DahengCamera::tryGetInt(const char* key, int64_t& out) {
    if (!device_handle_) return false;
    GX_INT_VALUE v{};
    GX_STATUS st = GXGetIntValue(device_handle_, key, &v);
    if (st == GX_STATUS_SUCCESS) {
        out = v.nCurValue;
        return true;
    }
    return false;
}