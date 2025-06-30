//#ifndef __CAMERA_COMBO_MANAGER_HPP__
//#define __CAMERA_COMBO_MANAGER_HPP__
#pragma once

#include <iostream>
#include <memory>
#include <thread>
#include <deque>
#include <mutex>
#include <chrono>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <fstream>
#include "boost/filesystem.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include "mvs/MvCameraControl.h"

#include "metavision/sdk/driver/camera.h"
#include "metavision/hal/device/device_discovery.h"
#include <metavision/hal/facilities/i_trigger_in.h>
#include "metavision/sdk/driver/ext_trigger.h"
//
//#include <DvsenseDriver/camera/DvsCameraManager.hpp>
//#include <DvsenseHal/camera/DvsCameraUtils.hpp>


#ifdef _WIN32
#ifdef DVS_VISUALIZER_EXPORT
#define DVS_API __declspec(dllexport)
#else
#define DVS_API __declspec(dllexport)
#endif
#else
#define DVS_API DLLEXPORT
#define DLLEXPORT __attribute__((visibility("default")))
#endif

namespace dvsense{
    using Timestamp = uint64_t;
    struct DVS_API ApsMFrame {
        cv::Mat frame;
        Timestamp exposure_start_timestamp;
        Timestamp exposure_end_timestamp;
        ApsMFrame(
            cv::Mat& frame, Timestamp exposure_start_timestamp = 0, Timestamp exposure_end_timestamp = 0
        ):frame(frame), exposure_start_timestamp(exposure_start_timestamp), exposure_end_timestamp(exposure_end_timestamp){}
    };    
    using NewFrameCallback = std::function<void(const ApsMFrame &frame)>;

class DVS_API CameraComboManager {
public:
    CameraComboManager(float fps = 30){
        fps_ = fps;
        int ret = MV_CC_Initialize();
        if (ret != MV_OK){
            std::cout << "MV_CC_Initialize fail! ret = " << ret << std::endl;
        }
        trigger_write_status_ = false;
        find_aps_camera();
        find_dvs_camera();
        aps_num_ = 0;
        trigger_num_ = 0;
    }

    ~CameraComboManager(){
        stop_camera();
    }

    void start_camera(std::string output_dir = "");
    void stop_camera();
    std::string get_current_time();
    void start_recoreding(std::string output_dir = "");
    void stop_recoreding();
    bool get_aps_frame(cv::Mat& output_frame);

    int register_frame_callback(const NewFrameCallback &new_frame_callback) {
        new_frame_callbacks.insert(
            { frame_callback_id_num_, new_frame_callback }
        );
        frame_callback_id_num_++;
        return frame_callback_id_num_;
    }

    int register_event_callback(const Metavision::EventsCDCallback& callback);
    //int register_event_callback(const dvsense::EventsStreamHandleCallback& callback);

private:
    // ----- APS camera -----
    void *aps_camera_handle_ = nullptr;
    int find_aps_camera();
    MV_FRAME_OUT frame_out_ = {0};
    MV_FRAME_OUT_INFO_EX frame_out_info_ex_;
    std::mutex frame_buffer_mutex_;
    std::condition_variable frame_ready_condition_;

    std::unique_ptr<ApsMFrame> raw_frame_;

    std::atomic<bool> is_grab_image_thread_running = false;
    std::thread grab_frame_thread;

    std::unique_ptr<cv::VideoWriter> video_wirter_;
    std::string aps_file_path_;

    float fps_=30;

    int start_aps_camera();
    void stop_aps_camera();

    void buffer_to_mat(cv::Mat &frame);
    int get_next_frame(cv::Mat& frame);

    // ----- DVS camera -----
    std::unique_ptr<Metavision::Camera> dvs_camera_;
    //dvsense::CameraDevice dvs_camera_;

    int find_dvs_camera();
    int start_dvs_camera();
    void stop_dvs_camera();

    // ----- sync -----
    void ext_trigger_sync_callback(
        const Metavision::EventExtTrigger* begin,
        const Metavision::EventExtTrigger* end
        //const dvsense::EventTriggerIn* begin
        //const dvsense::EventTriggerIn* end
    );
    uint64_t last_timestamp_ = 0;

    int frame_callback_id_num_ = 0;
    std::map<int, const NewFrameCallback> new_frame_callbacks;
    std::ofstream trigger_file_;
    bool trigger_write_status_;

    int aps_num_;
    int trigger_num_;
    std::queue<cv::Mat> aps_frames_;
};

} // namespace dvsense

//#endif // __CAMERA_COMBO_MANAGER_HPP__