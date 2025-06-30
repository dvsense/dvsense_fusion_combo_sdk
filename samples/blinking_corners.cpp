//#include "calib_utils.hpp"
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/core.hpp>
//#include <iostream>
//
//int main() {
//    // Size of each square (keeping original square size)
//    int squareSize = 90;
//
//    float square_size_mm = dvsense::calib_utils::get_square_size(squareSize);
//    std::cout << "square_size_mm: " << square_size_mm << std::endl;
//
//    std::string window_name = "Chessboard";
//    int interval = 100;
//    cv::Mat chessboard = dvsense::calib_utils::get_chessboard(5, 5, squareSize);
//    cv::Mat black = cv::Mat(chessboard.size(), CV_8UC1, cv::Scalar(0));
//    bool show_chessboard = true;
//    
//    while(true) {
//        if (show_chessboard) {
//            cv::imshow(window_name, chessboard);
//        } else {
//            cv::imshow(window_name, black);
//        }
//        show_chessboard = !show_chessboard;
//        if(cv::waitKey(interval) == 27) break;
//    }
//
//    cv::destroyAllWindows();
//    return 0;
//}
