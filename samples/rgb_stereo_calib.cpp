#include <calibration.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
//#include <unistd.h>

int main()
{
    
    dvsense::DvsCalibration calib(1280, 720, 4, 7, 30.0, 30.0, 2, true);

    for(int i = 0; i < 4; i++) {
        char filename1[256], filename2[256];

        sprintf(filename1, "samples/rgb_frames/synched/%d.png", i);
        sprintf(filename2, "samples/rgb_frames/synched/%d.png", i+4);
        std::cout << "Processing " << filename1 << " and " << filename2 << std::endl;
        cv::Mat img1 = cv::imread(filename1);
        cv::Mat img2 = cv::imread(filename2);

        bool success = calib.insert_frames(img1, img2);
        if (!success) {
            std::cerr << "Error inserting frame" << std::endl;
            return -1;
        }
    }

    double error = calib.calibrate_stereo();
    std::cout << "Reprojection error: " << error << std::endl;

    double error0 = calib.get_reprojection_error(0);
    double error1 = calib.get_reprojection_error(1);
    std::cout << "Reprojection error for camera 0: " << error0 << std::endl;
    std::cout << "Reprojection error for camera 1: " << error1 << std::endl;

    cv::Mat R, T;
    cv::Point2f estimated_pixel_shift;
    calib.get_extrinsic_matrix(0, R, T);
    std::cout << "Rotation matrix for camera 0: " << R << std::endl;
    std::cout << "Translation vector for camera 0: " << T << std::endl;

    return 0;
}