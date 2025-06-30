#include "calib_utils.hpp"
//#include <X11/Xlib.h>
//#include <X11/Xutil.h>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

namespace dvsense{
namespace calib_utils{
    //float get_square_size(int square_size_pixel) {
    //        // 打开默认的显示
    //    Display* display = XOpenDisplay(nullptr);
    //    if (display == nullptr) {
    //        std::cerr << "Unable to open X display." << std::endl;
    //        return 1;
    //    }

    //    // 获取屏幕的ID
    //    int screen = DefaultScreen(display);

    //    // 获取屏幕的物理尺寸（毫米）
    //    float width_mm = DisplayWidthMM(display, screen);
    //    float height_mm = DisplayHeightMM(display, screen);

    //    // 获取屏幕的分辨率（像素）
    //    float width_px = DisplayWidth(display, screen);
    //    float height_px = DisplayHeight(display, screen);

    //    // 计算 DPI
    //    float dpi_x = (width_px / static_cast<float>(width_mm)) * 25.4; // 1 英寸 = 25.4 毫米
    //    float dpi_y = (height_px / static_cast<float>(height_mm)) * 25.4;

    //    float dpi = std::sqrt(width_px * width_px + height_px * height_px) / 
    //                std::sqrt(width_mm * width_mm / 25.4 / 25.4 + height_mm * height_mm / 25.4 / 25.4);
    //    // 关闭显示
    //    XCloseDisplay(display);

    //    return square_size_pixel * 25.4 / dpi;
    //}

    //cv::Mat get_chessboard(int corner_rows, int corner_cols, int square_size_pixel) {
    //    int padding = square_size_pixel * 2;
    //    cv::Mat chessboard((corner_rows+1) * square_size_pixel + 2 * padding, (corner_cols+1) * square_size_pixel + 2 * padding, CV_8UC1, cv::Scalar(255));
    //    for(int i = 0; i < corner_rows+1; i++) {
    //        for(int j = 0; j < corner_cols+1; j++) {
    //            if((i + j) % 2 == 1) {
    //                cv::rectangle(chessboard, cv::Rect(j * square_size_pixel + padding, 
    //                                                   i * square_size_pixel + padding, 
    //                                                   square_size_pixel, 
    //                                                   square_size_pixel), 
    //                                                   cv::Scalar(0), -1);
    //            }
    //        }
    //    }
    //    return chessboard;
    //}
}
}