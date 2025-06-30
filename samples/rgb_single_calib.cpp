#include <calibration.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
//#include <unistd.h>

int main()
{
    dvsense::DvsCalibration calib0(1280, 720, 4, 7, 30.0, 30.0, 1, false);
    dvsense::DvsCalibration calib1(1280, 720, 4, 7, 30.0, 30.0, 1, false);
    for(int i = 0; i < 4; i++) {
        char filename1[256];
        char filename2[256];
        sprintf(filename1, "samples/rgb_frames/D2/camera0_%d.png", i);
        sprintf(filename2, "samples/rgb_frames/J2/camera1_%d.png", i);
        std::cout << "Processing " << filename1 << std::endl;
        std::cout << "Processing " << filename2 << std::endl;
        cv::Mat img1 = cv::imread(filename1);
        cv::Mat img2 = cv::imread(filename2);
        bool success = calib0.insert_frame(img1, 0);
        if (!success) {
            std::cerr << "Error inserting frame" << std::endl;
            return -1;
        }
        success = calib1.insert_frame(img2, 0);
        if (!success) {
            std::cerr << "Error inserting frame" << std::endl;
            return -1;
        }
    }

    double error0 = calib0.calibrate_single(0);
    double error1 = calib1.calibrate_single(0);

    std::cout << "Reprojection error for camera 0: " << error0 << std::endl;
    std::cout << "Reprojection error for camera 1: " << error1 << std::endl;

    return 0;
}