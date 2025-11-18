#pragma once

#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>


#ifdef _WIN32
#include "GalaxyIncludes.h"
#else
#include "GxIAPI.h"
#endif

#include "DxImageProc.h"
#include "DvsRgbFusionCamera/rgb/RgbCamera.hpp"

class DahengCamera : public RgbCamera 
{

public:
    explicit DahengCamera(float fps);
    ~DahengCamera();

    // 设备状态
    bool isConnect();

    // 枚举设备（返回序列号列表）
    bool findCamera(std::vector<std::string>& serial_numbers);

    // 打开指定序列号设备，并进行参数配置（尽量与 Hik 对齐）
    bool openCamera(std::string serial_number);

    // 开启外触发（尽量模拟 Hik 行为，节点名因 SDK 差异做了兼容处理）
    bool openExternalTrigger();

    // 启动采集线程，持续抓取图像并压入队列
    int startCamera();

    // 停止采集线程与取流
    void stopCamera();

    // 关闭设备与库（与 Hik 接口保持一致）
    int destroyCamera();

    // 从队列取出最新一帧（与 Hik 接口一致）
    bool getNewRgbFrame(dvsense::ApsFrame& output_frame);

    // 图像宽高
    int getWidth();
    int getHeight();

private:
    // 从相机取下一帧并转换为 BGR8，返回 0 成功，-1 失败；返回丢帧数
    int getNextFrame(dvsense::ApsFrame& rgb_frame, int& drop_frame_num);

    // 将 PGX_FRAME_BUFFER 转为 BGR8 写入 ApsFrame（内部做 Bayer->RGB 与通道交换）
    int bufferToMat(PGX_FRAME_BUFFER pFrameBuffer, dvsense::ApsFrame& rgb_frame);

    // 分配/释放转换缓冲
    void preForAcquisition();
    void unPreForAcquisition();

    // 尝试设置/读取节点的工具函数（容错不抛异常）
    void trySetEnumByString(const char* key, const char* val);
    void trySetFloat(const char* key, double val);
    void trySetInt(const char* key, int64_t val);
    bool tryGetInt(const char* key, int64_t& out);

private:
    // 基本状态
    float fps_;
    bool is_grab_image_thread_running_;
    int frame_callback_id_num_;
    long long last_frame_id_;

    // Galaxy 句柄
    GX_DEV_HANDLE device_handle_;
    GX_DS_HANDLE  ds_handle_;
    uint32_t payload_size_;

    // 转换缓存
    unsigned char* rgb_image_buf_; // payload*3
    unsigned char* raw8_image_buf_; // payload

    // 彩色图像相关
    bool color_filter_supported_;
    int64_t color_filter_type_;

    // 抓帧线程与缓冲
    std::thread grab_frame_thread_;
    std::queue<dvsense::ApsFrame> frames_buffer_;
    std::mutex frame_buffer_mutex_;
};