#include <calibration.hpp>
#include <calib_utils.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <DvsenseDriver/camera/DvsCameraManager.hpp>
#include "camera_manager/dvs_visualizer.hpp"
#include <thread>
#include <chrono>
#include <atomic>

#define INTERVAL 100
#define FRAME_COUNT 100

#define CHESSBOARD_ROW 5
#define CHESSBOARD_COL 5
#define CHESSBOARD_SIZE 90

int main()
{
    DvsVisualizer dvs_visualizer{720, 1280};
    std::vector<cv::Mat> dvs_frames(FRAME_COUNT, cv::Mat(720, 1280, CV_8UC1));
    dvsense::DvsCameraManager camera_manager;
    std::vector<dvsense::CameraDescription> camera_descs = camera_manager.getCameraDescs();

    dvsense::CameraDevice camera = camera_manager.openCamera(camera_descs[0].serial);

    camera->setBatchEventsTime(33333);
    auto width = camera->getWidth();
    auto height = camera->getHeight();


    bool in_collection = false;
    bool in_visualization = true;
    cv::Mat img(height, width, CV_8UC3);
    std::cout << "start visualization" << std::endl;
    std::cout << "press 'c' to collect frames, press 'q' to stop collection and start calibration" << std::endl;
    camera->start();
    int count = 0;
    cv::namedWindow("dvs_img", cv::WINDOW_NORMAL);
    cv::namedWindow("chessboard", cv::WINDOW_AUTOSIZE);
    std::mutex chessboard_mutex;
    cv::Mat chessboard_base = dvsense::calib_utils::get_chessboard(CHESSBOARD_ROW, CHESSBOARD_COL, CHESSBOARD_SIZE);
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
    chessboard_thread.detach();

    while(in_visualization) {
        dvsense::Event2DVector events;
        bool ret = camera->getNextBatch(events);
        if(ret) {
            dvs_visualizer.update_events(events);
        }

        dvs_visualizer.get_histogram(img);
        dvs_visualizer.reset();
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

        if(in_collection) {
            dvs_frames[count % FRAME_COUNT] = img.clone();
            count++;
        }

        cv::imshow("dvs_img", img);
        std::unique_lock<std::mutex> lock(chessboard_mutex);
        cv::imshow("chessboard", chessboard);
        lock.unlock();
        char key = cv::waitKey(1);
        switch (key)
        {
        case 'c':
            in_collection = true;
            std::cout << "collection started" << std::endl;
            break;
        case 'q':
            in_collection = false;
            in_visualization = false;
            stop_chessboard_thread = true;
            break;
        default:
            break;
        }
    }
    camera->stop();
    cv::destroyAllWindows();

    std::cout << "collection is done, waiting for analysis..." << std::endl;

    float square_size_mm = dvsense::calib_utils::get_square_size(CHESSBOARD_SIZE);
    dvsense::DvsCalibration calib(width, height, CHESSBOARD_ROW, CHESSBOARD_COL, square_size_mm, square_size_mm, 1, false);
    
    count = std::min(count, FRAME_COUNT);
    for(int i = 0; i < count; i++) {
        bool success = calib.insert_frame(dvs_frames[i], 0);
        if(!success) {
            dvs_frames.erase(dvs_frames.begin() + i);
            i--;
            count--;
        }
    }

    int index = 0;
    std::cout << "frame count: " << count << std::endl;
    std::cout << "select frames for calibration" << std::endl;
    std::cout << "press 'd' to delete frame, press 'q' to move to calibration, press 'n' to next frame, press 'p' to previous frame" << std::endl;
    cv::namedWindow("chessboard result", cv::WINDOW_NORMAL);
    while(true && count > 0) {
        cv::Mat canvas;
        calib.draw_chessboard_corners(dvs_frames[index], canvas, 0, index);
        cv::putText(canvas, std::to_string(index + 1) + "/" + std::to_string(count), 
                    cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, 
                    cv::Scalar(255, 255, 255), 2);
        cv::imshow("chessboard result", canvas);
        char key = cv::waitKey(0); // Changed from 1 to 0 to wait for keyboard input
        if (key == 'd') {
            calib.delete_frame(index);
            dvs_frames.erase(dvs_frames.begin() + index);
            count--;
            index = std::min(index, count - 1);
            if(count == 0) {
                break;
            }
        }
        if (key == 'q') {
            break;
        }
        if (key == 'n') {
            index++;
            index = index % count;
        }
        if (key == 'p') {
            index--;
            index = (index + count) % count;
        }
    }

    if(count > 0) {
        double error = calib.calibrate_single();
        std::cout << "Reprojection error: " << error << std::endl;
    } else {
        std::cout << "No frames selected for calibration" << std::endl;
    }
    
    cv::destroyAllWindows();

    calib.save_calibration_result_to_json("calibration_result.json");

    return 0;
}
