#include <string>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>

namespace dvsense{
namespace calib_utils{
    /// @brief get the square size in mm
    /// @param square_size_pixel 
    /// @note please run this function when the scaling of the screen is 100% or it does not return the correct value, 
    ///       and the returned value could be a little different from the real value, you can adjust it by yourself
    /// @return the square size in mm
    float get_square_size(int square_size_pixel);

    /// @brief blinking the chessboard on the screen, the chessboard will be surrounded by a white border
    /// @param corner_rows the number of rows of corners of the chessboard
    /// @param corner_cols the number of cols of corners of the chessboard
    /// @param square_size_pixel the size of each square in pixel
    /// @return the chessboard image
    //cv::Mat get_chessboard(int corner_rows, int corner_cols, int square_size_pixel);
}
}