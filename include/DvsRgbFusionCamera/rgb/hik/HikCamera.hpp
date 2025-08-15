#pragma once
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include "MvCameraControl.h"

#include <atomic>
#include <thread>
#include "DvsRgbFusionCamera/rgb/RgbCamera.hpp"

struct FrameAndDrop
{
    cv::Mat frame;
    int drop_frame_num;
};

class HikCamera: public RgbCamera
{
public:
    HikCamera(float fps): fps_(fps)  {
        int ret = MV_CC_Initialize();
        if (ret != MV_OK) {
            std::cout << "MV_CC_Initialize fail! ret = " << ret << std::endl;
        }
    }
	~HikCamera();

    bool findCamera() override;

    int startCamera() override;

    bool isConnect() override;

    void stopCamera() override;

    bool getNewRgbFrame(cv::Mat& output_frame) override;

    int getWidth() override;

    int getHeight() override;

    int destroyCamera() override;

private:
    void* aps_camera_handle_ = nullptr;
    MV_FRAME_OUT frame_out_ = {};
    std::atomic<bool> is_grab_image_thread_running_ = false;
    std::thread grab_frame_thread_;
    std::queue<cv::Mat> private_buffer_frames_;
    std::mutex frame_buffer_mutex_;
    float fps_;
 
    int frame_callback_id_num_ = 0;


    void bufferToMat(cv::Mat& frame);
    int getNextFrame(FrameAndDrop& frame_and_drops);

    unsigned long long last_frame_id_ = -1;
    std::queue<FrameAndDrop> aps_frames_drop_;




};
