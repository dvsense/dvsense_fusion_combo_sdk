#pragma once
#include <opencv2/core.hpp>
#include "MvCameraControl.h"

#include <atomic>
#include <thread>
#include <queue>
#include "DvsRgbFusionCamera/rgb/RgbCamera.hpp"


class HikCamera: public RgbCamera
{
public:
    HikCamera(float fps);
	~HikCamera();

    bool findCamera(std::vector<std::string>& serial_numbers) override;

    bool openCamera(std::string serial_number) override;

    int startCamera() override;

    bool isConnect() override;

    void stopCamera() override;

    bool getNewRgbFrame(cv::Mat& output_frame) override;

    int getWidth() override;

    int getHeight() override;

    int destroyCamera() override;

private:
    struct FrameAndDrop
    {
        cv::Mat frame;
        int drop_frame_num;
    };
    void bufferToMat(cv::Mat& frame);
    int getNextFrame(FrameAndDrop& frame_and_drops);

    void* aps_camera_handle_ = nullptr;
    MV_FRAME_OUT frame_out_ = {};
    std::atomic<bool> is_grab_image_thread_running_;
    std::thread grab_frame_thread_;
    std::mutex frame_buffer_mutex_;
    std::queue<cv::Mat> frames_buffer_;
    float fps_;
 
    int frame_callback_id_num_;
    int64_t last_frame_id_;
};
