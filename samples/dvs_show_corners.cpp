#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>

#include <opencv2/highgui.hpp>

#include <DvsenseDriver/camera/DvsCameraManager.hpp>
#include "camera_manager/dvs_visualizer.hpp"
#include "calibration.hpp"
#include "calib_utils.hpp"



#define FRAME_COUNT 100
#define INTERVAL 100


int main()
{
    // Start-up visualization
    constexpr int timespan = 33333;
    DvsVisualizer dvs_visualizer{ 720, 1280 };

    dvsense::TimeStamp ts = 0;
    dvsense::DvsCameraManager camera_manager;
    std::vector<dvsense::CameraDescription> camera_descs = camera_manager.getCameraDescs();

    dvsense::CameraDevice camera = camera_manager.openCamera(camera_descs[0].serial);

    cv::Mat img_to_show;
    std::mutex img_to_show_mutex;

    camera->addEventsStreamHandleCallback([&dvs_visualizer, &img_to_show, &img_to_show_mutex, &ts, &timespan](dvsense::EventIterator_t begin, dvsense::EventIterator_t end) {
        dvs_visualizer.update_events(begin, end);
        dvsense::TimeStamp end_time = (end-1)->timestamp;
        if(end_time - ts > timespan) {
            std::unique_lock<std::mutex> lock(img_to_show_mutex);
            dvs_visualizer.get_histogram(img_to_show);
            dvs_visualizer.reset();
            lock.unlock();
            ts = end_time;
        }
    });

    camera->start();

    std::mutex chessboard_mutex;
    cv::Mat chessboard_base = dvsense::calib_utils::get_chessboard(5, 5, 90);
    cv::Mat black = cv::Mat(chessboard_base.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat chessboard = chessboard_base;

    bool show_chessboard = true;
    std::atomic<bool> stop_chessboard_thread(false);
    std::thread chessboard_thread([&chessboard, &chessboard_base, &black, &chessboard_mutex, &show_chessboard, &stop_chessboard_thread]() {
        while(!stop_chessboard_thread) {
            std::unique_lock<std::mutex> lock(chessboard_mutex);
            if (show_chessboard) {
                chessboard = chessboard_base;
            } else {
                chessboard = black;
            }
            show_chessboard = !show_chessboard;
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(INTERVAL));
        }
    });

    cv::namedWindow("Corners");
    cv::namedWindow("Chessboard");
    std::cout << "Starting visualization, press ESC to quit" << std::endl;
    while (true) {
        std::unique_lock<std::mutex> lock(img_to_show_mutex);
        std::vector<cv::Point2f> corners_dvs;
        if(img_to_show.empty()) {
            continue;
        }
        if(img_to_show.type() != CV_8UC1) {
            cv::cvtColor(img_to_show, img_to_show, cv::COLOR_BGR2GRAY);
        }
        bool found_dvs = cv::findChessboardCornersSB(img_to_show, cv::Size(5, 5), corners_dvs, cv::CALIB_CB_NORMALIZE_IMAGE);
        if(found_dvs) {
            cv::drawChessboardCorners(img_to_show, cv::Size(5, 5), corners_dvs, found_dvs);
        }

        cv::imshow("Corners", img_to_show);
        lock.unlock();
        std::unique_lock<std::mutex> lock_chessboard(chessboard_mutex);
        cv::imshow("Chessboard", chessboard);
        lock_chessboard.unlock();
        if (cv::waitKey(33) == 27) {
            stop_chessboard_thread = true;
            break;
        }
    }

    chessboard_thread.join();
    camera->stop();
    cv::destroyAllWindows();
    return 0;
}