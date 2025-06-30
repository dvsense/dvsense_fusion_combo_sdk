//#include "calibration.hpp"
//
//int main() {
//    std::filesystem::path current_path = std::filesystem::current_path();
//    std::string file_path = "samples/calib_data/calibration_result_dvs_rgb.json";
//    std::cout << "Current working directory: " << current_path << std::endl;
//    dvsense::DvsCalibration calib;
//    calib.load_calibration_result_from_json(file_path);
//    calib.save_calibration_result_to_json("samples/calib_data/calibration_result_dvs_rgb_new.json");
//
//    return 0;
//}