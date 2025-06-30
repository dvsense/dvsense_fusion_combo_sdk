#include <iostream>
#include <fstream>
#include <memory>
#include <opencv2/highgui.hpp>
#include "camera_manager/camera_combo_manager.hpp"
#include "camera_manager/dvs_visualizer.hpp"
#include "camera_manager/dvs_process.hpp"
//#include "calib_utils.hpp"
#include "metavision/sdk/base/events/event_cd.h"
#include "calibration.hpp"
#include <thread>
#include <atomic>

#define INTERVAL 100
#define FRAME_COUNT 50

#define CHESSBOARD_ROW 5
#define CHESSBOARD_COL 5
#define CHESSBOARD_SIZE 90

#define WIDTH 1280
#define HEIGHT 720

int main(int argc, char* argv[]) {
    //start-up visualization    
    DvsVisualizer dvs_visualizer{ HEIGHT, WIDTH };
    DvsProcess dvs_process{ HEIGHT, WIDTH };
    std::unique_ptr<dvsense::CameraComboManager> ccm = std::make_unique<dvsense::CameraComboManager>(50);
    cv::Mat img_to_show(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    std::mutex img_to_show_mutex;

    //std::vector<cv::Mat> dvs_frames(FRAME_COUNT, cv::Mat(HEIGHT, WIDTH, CV_8UC3));
    //std::vector<cv::Mat> aps_frames(FRAME_COUNT, cv::Mat(HEIGHT, WIDTH, CV_8UC3));
    //dvs_frames.reserve(FRAME_COUNT);
    //aps_frames.reserve(FRAME_COUNT);
    std::queue<cv::Mat> dvs_frames;
    std::queue<cv::Mat> aps_frames;
    int count = 0;
    int actual_count = 0;
    bool in_visualization = true;
    bool in_collection = true;

    ccm->register_frame_callback(
        [&ccm, &img_to_show, &img_to_show_mutex, &dvs_visualizer, &dvs_process, &aps_frames, &dvs_frames, &count, &in_collection, &actual_count](const dvsense::ApsMFrame &aps_frame) {
            if(in_collection) {
                cv::Mat new_dvs_frame;
                dvs_visualizer.get_histogram(new_dvs_frame);
                dvs_visualizer.reset();

                cv::Mat new_aps_frame;
                int pixel_cnt = (aps_frame.frame.rows - HEIGHT) / 2;
                aps_frame.frame(cv::Rect(0, pixel_cnt, WIDTH, HEIGHT)).copyTo(new_aps_frame);

                {
                    std::unique_lock<std::mutex> lock(img_to_show_mutex);
                    new_aps_frame.copyTo(img_to_show);
                    img_to_show.setTo(cv::Scalar(0, 0, 0), new_dvs_frame);
                    img_to_show = img_to_show + new_dvs_frame;

                    actual_count++;
                    count++;
                    count = count % FRAME_COUNT;
                }
                

            } else {
                cv::Mat dvs_frame;
                dvs_visualizer.get_histogram(dvs_frame);
                dvs_visualizer.reset();
                int pixel_cnt = (aps_frame.frame.rows - HEIGHT) / 2;
                cv::Mat rgb_frame = aps_frame.frame(cv::Rect(0, pixel_cnt, WIDTH, HEIGHT)).clone();

                std::unique_lock<std::mutex> lock(img_to_show_mutex);
                img_to_show = rgb_frame;
                img_to_show.setTo(cv::Scalar(0, 0, 0), dvs_frame);
                img_to_show = img_to_show+dvs_frame;
                //img_to_show = dvs_frame;
            }

            cv::putText(img_to_show, std::to_string(std::min(actual_count, FRAME_COUNT)) + "/" + std::to_string(FRAME_COUNT), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        }
    );

    //ccm->register_event_callback(
    //    [&dvs_visualizer, &dvs_process](const dvsense::EventIterator_t begin, const dvsense::EventIterator_t end) {
    //        dvs_visualizer.update_events(begin, end);

    //        static uint64_t last_timestamp = 0;
    //        for (auto iter = begin; iter < end; iter++) {
    //            if (iter->timestamp - last_timestamp > 200)
    //                std::cout << "last_timestamp: " <<last_timestamp << " timestamp: " << iter->timestamp 
    //                << " diff: " << iter->timestamp - last_timestamp << std::endl;
    //            last_timestamp = iter->timestamp;
    //        }
    //    }
    //);
    ccm->register_event_callback(
        [&dvs_visualizer](const Metavision::EventCD* begin, const Metavision::EventCD* end) {
            dvs_visualizer.update_events(begin, end);
            static uint64_t last_timestamp = 0;
            for (auto iter = begin; iter < end; iter++) {
                if (iter->t - last_timestamp > 200)
                    std::cout << "last_timestamp: " <<last_timestamp << " timestamp: " << iter->t 
                    << " diff: " << iter->t - last_timestamp << std::endl;
                last_timestamp = iter->t;
            }
   
        }
    );
    //std::mutex chessboard_mutex;
    //cv::Mat chessboard_base = dvsense::calib_utils::get_chessboard(CHESSBOARD_ROW, CHESSBOARD_COL, CHESSBOARD_SIZE);
    //cv::Mat black = cv::Mat(chessboard_base.size(), CV_8UC1, cv::Scalar(0));
    //cv::Mat chessboard = chessboard_base;
    //bool show_chessboard = true;
    //std::atomic<bool> stop_chessboard_thread(false);
    //std::thread chessboard_thread([&chessboard, &chessboard_base, &black, &chessboard_mutex, &show_chessboard, &stop_chessboard_thread]() {
    //    while(!stop_chessboard_thread) {
    //        std::unique_lock<std::mutex> lock(chessboard_mutex);
    //        if (show_chessboard) {
    //            chessboard = chessboard_base;
    //        } else {
    //            chessboard = black;
    //        }
    //        show_chessboard = !show_chessboard;
    //        lock.unlock();
    //        std::this_thread::sleep_for(std::chrono::milliseconds(INTERVAL));
    //    }
    //});
    //chessboard_thread.detach();

    //ccm->start_camera("D:/FusionCamera/test_datas");
    ccm->start_camera();

    std::string save_path;  
    std::ifstream config_file("config.txt"); 
    if (!config_file.is_open()) {
        std::cerr << "Error: Could not open config.txt!" << std::endl;
        return 1;  
    }
    std::getline(config_file, save_path);
    config_file.close();  
    //ccm->start_recoreding(save_path);


    cv::namedWindow("Merged Frame");
    //cv::namedWindow("Chessboard");

    //std::cout << "Press 'c' to start collection, press 'q' to stop and move to calibration" << std::endl;

    while(in_visualization) {
        cv::imshow("Merged Frame", img_to_show);
        //cv::imshow("Chessboard", chessboard);
        char key = cv::waitKey(30);

        if (key == 'b') {
            ccm->start_recoreding(save_path);
        }
        if (key == 'e') {
            ccm->stop_recoreding();
        }

        if(key == 'q') {
            in_visualization = false;
            in_collection = false;
            //stop_chessboard_thread
            break;
        }
        if(key == 'c') {
            std::cout << (in_collection ? "Collection stopped" : "Collection started") << std::endl;
            in_collection = !in_collection;
        }
    }

    ccm->stop_camera();
    ccm.reset();
    cv::destroyAllWindows();

    //std::cout << "collection is done, waiting for analysis..." << std::endl;

    //float square_size_mm = dvsense::calib_utils::get_square_size(CHESSBOARD_SIZE);
    //dvsense::DvsCalibration calib(WIDTH, HEIGHT, CHESSBOARD_ROW, CHESSBOARD_COL, square_size_mm, square_size_mm, 1, true);
    
    //count = std::min(actual_count, FRAME_COUNT);

    //std::atomic<int> index_atomic(0);
    //std::vector<std::thread> threads;
    //std::vector<int> delete_index;
    //for(int i = 0; i < 8; i++) {
    //    threads.emplace_back([&dvs_frames, &aps_frames, &count, &index_atomic, &delete_index, &calib]() {
    //        while(true) {
    //            int idx = index_atomic.fetch_add(1);
    //            if(idx >= count) {
    //                break;
    //            }
    //            cv::cvtColor(dvs_frames[idx], dvs_frames[idx], cv::COLOR_BGR2GRAY);
    //            bool success = calib.insert_frames(dvs_frames[idx], aps_frames[idx], idx, idx);
    //            if(!success) {
    //                delete_index.push_back(idx);
    //            }
    //        }
    //    });
    //}

    //for(auto& thread : threads) {
    //    thread.join();
    //}

    //calib.sort_frames();

    //// Sort indices in descending order to avoid invalidating later indices
    //std::sort(delete_index.begin(), delete_index.end(), std::greater<int>());
    //for(int idx : delete_index) {
    //    dvs_frames.erase(dvs_frames.begin() + idx);
    //    aps_frames.erase(aps_frames.begin() + idx);
    //    count--;
    //}

    //int index = 0;
    //std::cout << "frame count: " << count << std::endl;
    //std::cout << "select frames for calibration" << std::endl;
    //std::cout << "press 'd' to delete frame, press 'q' to move to calibration, press 'n' to next frame, press 'p' to previous frame" << std::endl;

    //cv::namedWindow("chessboard result aps", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("chessboard result dvs", cv::WINDOW_AUTOSIZE);

    //while(true && count > 0) {
    //    cv::Mat canvas_dvs, canvas_aps;
    //    calib.draw_chessboard_corners(dvs_frames[index], canvas_dvs, 0, index);
    //    calib.draw_chessboard_corners(aps_frames[index], canvas_aps, 1, index);
    //    cv::putText(canvas_dvs, std::to_string(index + 1) + "/" + std::to_string(count), 
    //                cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, 
    //                cv::Scalar(255, 255, 255), 2);
    //    cv::putText(canvas_aps, std::to_string(index + 1) + "/" + std::to_string(count), 
    //                cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, 
    //                cv::Scalar(255, 255, 255), 2);
    //    cv::imshow("chessboard result aps", canvas_aps);
    //    cv::imshow("chessboard result dvs", canvas_dvs);
    //    char key = cv::waitKey(0); // Changed from 1 to 0 to wait for keyboard input
    //    if (key == 'd') {
    //        calib.delete_frame(index);
    //        dvs_frames.erase(dvs_frames.begin() + index);
    //        aps_frames.erase(aps_frames.begin() + index);
    //        count--;
    //        index = std::min(index, count - 1);
    //        if(count == 0) {
    //            break;
    //        }
    //    }
    //    if (key == 'q') {
    //        break;
    //    }
    //    if (key == 'n') {
    //        index++;
    //        index = index % count;
    //    }
    //    if (key == 'p') {
    //        index--;
    //        index = (index + count) % count;
    //    }
    //}

    //cv::destroyAllWindows();

    //std::cout << "selection is done, waiting for calibration..." << std::endl;

    //double error_stereo = calib.calibrate_stereo();
    //std::cout << "Calibration stereo error: " << error_stereo << std::endl;

    //double error0 = calib.get_reprojection_error(0);
    //double error1 = calib.get_reprojection_error(1);
    //std::cout << "Reprojection error of camera 0: " << error0 << std::endl;
    //std::cout << "Reprojection error of camera 1: " << error1 << std::endl;

    //cv::Mat R, T;
    //calib.get_extrinsic_matrix(0, R, T);
    //std::cout << "Rotation matrix of camera 0: " << R << std::endl;
    //std::cout << "Translation vector of camera 0: " << T << std::endl;

    //double error_affine = calib.calibrate_affine();
    //std::cout << "Calibration affine error: " << error_affine << std::endl;
    //cv::Mat r, t;
    //double scale;
    //calib.get_affine_matrix(0, r, t, scale);
    //std::cout << "Rotation-affine matrix of camera 0: " << r << std::endl;
    //std::cout << "Translation-affine vector of camera 0: " << t << std::endl;
    //std::cout << "Scale to camera 1: " << scale << std::endl;

    //cv::Mat affine_matrix;
    //calib.get_affine_matrix(0, affine_matrix);
    //std::cout << "Affine matrix of camera 0: " << affine_matrix << std::endl;

    //// Apply calibration    
    //cv::Mat camera_matrix_0, dist_coeffs_0;
    //calib.get_intrinsic_matrix(0, camera_matrix_0, dist_coeffs_0);
    //std::cout << "Camera matrix of camera 0: " << camera_matrix_0 << std::endl;
    //std::cout << "Distortion coefficients of camera 0: " << dist_coeffs_0 << std::endl;

    //cv::Mat camera_matrix_1, dist_coeffs_1;
    //calib.get_intrinsic_matrix(1, camera_matrix_1, dist_coeffs_1);
    //std::cout << "Camera matrix of camera 1: " << camera_matrix_1 << std::endl;
    //std::cout << "Distortion coefficients of camera 1: " << dist_coeffs_1 << std::endl;

    //calib.save_calibration_result_to_json("calibration_result_dvs_rgb.json");

    //// Start-up visualization for calibrated camera
    //ccm = std::make_unique<dvsense::CameraComboManager>(30);
    //dvs_visualizer.reset();

    ////ccm->register_frame_callback(
    ////    [&img_to_show, &img_to_show_mutex, &dvs_visualizer, &affine_matrix, &camera_matrix_0, &dist_coeffs_0, &camera_matrix_1, &dist_coeffs_1](const dvsense::ApsFrame &aps_frame) {
    ////        cv::Mat dvs_frame, dvs_frame_undistorted;
    ////        dvs_visualizer.get_histogram(dvs_frame);
    ////        // cv::undistort(dvs_frame, dvs_frame_undistorted, camera_matrix_0, dist_coeffs_0);

    ////        // cv::warpAffine(dvs_frame_undistorted, dvs_frame, affine_matrix, dvs_frame.size());
    ////        cv::warpAffine(dvs_frame, dvs_frame, affine_matrix, dvs_frame.size());
    ////        dvs_visualizer.reset();
    ////        int pixel_cnt = (aps_frame.frame.rows - HEIGHT) / 2;
    ////        cv::Mat rgb_frame = aps_frame.frame(cv::Rect(0, pixel_cnt, WIDTH, HEIGHT)).clone();
    ////        cv::Mat rgb_frame_undistorted;
    ////        // cv::undistort(rgb_frame, rgb_frame_undistorted, camera_matrix_1, dist_coeffs_1);

    ////        std::unique_lock<std::mutex> lock(img_to_show_mutex);
    ////        img_to_show = rgb_frame;
    ////        img_to_show.setTo(cv::Scalar(0, 0, 0), dvs_frame);
    ////        img_to_show = img_to_show + dvs_frame;
    ////    }
    ////);

    //ccm->register_event_callback(
    //    [&dvs_visualizer](const Metavision::EventCD* begin, const Metavision::EventCD* end) {
    //        dvs_visualizer.update_events(begin, end);
    //    }
    //);

    //ccm->start_camera();
    //std::cout << "Showing the calibrated frame, press 'q' to stop" << std::endl;
    //cv::namedWindow("Calibrated Frame");
    //while(true) {
    //    cv::imshow("Calibrated Frame", img_to_show);
    //    if(cv::waitKey(15) == 'q') {
    //        break;
    //    }
    //}

    //ccm->stop_camera();

    return 0;
}