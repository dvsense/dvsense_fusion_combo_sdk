//#include <memory>
//#include <vector>
//#include <functional>
//#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
//#include <opencv2/imgproc.hpp>
//#include <iostream>
//#include <string>
//#include <fstream>
//#include <nlohmann/json.hpp>
//
//namespace dvsense
//{
//using Corners = std::vector<cv::Point2f>;
//using ObjPts = std::vector<cv::Point3f>;
//
//struct CalibData
//{
//    std::vector<Corners> corners_vec;
//    cv::Mat camera_matrix;                  // the camera matrix (3x3)
//    cv::Mat dist_coeffs;
//    std::vector<cv::Mat> tvecs;        
//    std::vector<cv::Mat> rvecs;
//    cv::Mat R;                              // the rotation matrix (3x3)
//    cv::Mat T;                              // the translation vector (3x1)
//    bool is_calibrated;
//    // cv::Point2f estimated_pixel_shift;
//    double scale;                           // the scale to the other camera
//    cv::Mat r;                              // the rotation matrix (2x2) to the other camera
//    cv::Mat t;                              // the translation vector (2x1) to the other camera
//    std::vector<int> insert_index;          // the index of the frames inserted
//};
//
//class DvsCalibration
//{
//public:
//    DvsCalibration(uint16_t width, 
//                   uint16_t height, 
//                   uint16_t rows, 
//                   uint16_t cols,
//                   double square_width,
//                   double square_height,
//                   uint16_t image_num=0, 
//                   bool is_two_cameras=false);
//    DvsCalibration();
//    ~DvsCalibration();
//
//    /// @brief insert a frame into the calibration process
//    /// @param frame, the frame to be inserted
//    /// @param cam_id, the camera id, only 0 and 1 are supported
//    /// @return whether the frame is successfully inserted (if no valid chess board is detected, return false)
//    bool insert_frame(const cv::Mat& frame, uint8_t cam_id);
//
//    /// @brief insert a frame into the calibration process, add a specific index for convinience of threading
//    /// @param frame, the frame to be inserted
//    /// @param cam_id, the camera id, only 0 and 1 are supported
//    /// @param index, the index of the frame to be inserted
//    /// @return whether the frame is successfully inserted (if no valid chess board is detected, return false)
//    bool insert_frame(const cv::Mat& frame, uint8_t cam_id, int index);
//
//    /// @brief delete a frame from the calibration process
//    /// @param cam_id, the camera id, only 0 and 1 are supported
//    /// @param index, the index of the frame to be deleted, if index is -1, delete the last frame
//    /// @return whether the frame is successfully deleted
//    bool delete_frame(int index);
//
//    /// @brief insert a pair of frames into the calibration process
//    /// @param frame_cam_0 the frame from cam0 to be inserted
//    /// @param frame_cam_1 the frame from cam1 to be inserted
//    /// @return whether the frames are successfully inserted (if no valid chess board is detected, return false)
//    bool insert_frames(const cv::Mat& frame_cam_0, const cv::Mat& frame_cam_1);
//
//    /// @brief insert a pair of frames into the calibration process, add a specific index for convinience of threading
//    /// @param frame_cam_0 the frame from cam0 to be inserted
//    /// @param frame_cam_1 the frame from cam1 to be inserted
//    /// @param index_0 the index of the frames from cam0 to be inserted
//    /// @param index_1 the index of the frames from cam1 to be inserted
//    /// @return whether the frames are successfully inserted (if no valid chess board is detected, return false)
//    bool insert_frames(const cv::Mat& frame_cam_0, const cv::Mat& frame_cam_1, int index_0, int index_1);
//
//    /// @brief sort the frames based on the index
//    /// @return whether the frames are successfully sorted
//    bool sort_frames();
//
//    /// @brief Calibrate the camera intrinsics
//    /// @return reprojection error
//    double calibrate_single(uint8_t cam_id);
//
//    /// @brief Calibrate the camera intrinsics (overloaded for single camera, default to camera 0)
//    /// @return reprojection error
//    double calibrate_single();
//
//    // /// @brief filter the corner for stereo calibration using RANSAC to filter out outliers
//    // /// @note you should be careful about using this function because this function only aims at compounded camera, 
//    // /// where two cameras share the same view but they have a slight translation. Under such condition, the shift bewteen the same corners
//    // /// of the two cameras should have nearly the same value.
//    // /// @param tolerance, the tolerance pixel error of corners
//    // /// @param iter_num, the number of iterations of RANSAC
//    // /// @param sample_ratio, the ratio of the sample size to the total size
//    // /// @return whether the corners are valid
//    // bool filter_corners(double tolerance, int iter_num, float sample_ratio=0.1);
//
//    /// @brief Calibrate the stereo camera intrinsics
//    /// @return reprojection error
//    double calibrate_stereo();
//
//    /// @brief Calibrate the affine camera extrinsics (assume two cameras only differ this way)
//    /// @note you should be careful about using this function because this function only aims at compounded camera, 
//    /// where two cameras share the same view but they have a slight translation / rotation / scaling.
//    /// @return reprojection error
//    double calibrate_affine();
//
//    // /// @brief estimate the pixel shift of the id-th camerae
//    // /// @param frames_cam_2, the frames of camera 2
//    // /// @return the estimated pixel shift
//    // cv::Point2f estimate_pixel_shift(std::vector<cv::Mat>& frames_cam_1, std::vector<cv::Mat>& frames_cam_2);
//
//    /// @brief Draw the latest chessboard corners on the frame
//    /// @param frame 
//    /// @param cam_id 
//    void draw_chessboard_corners(cv::Mat& frame, cv::Mat& canvas, uint8_t cam_id, int corner_idx=-1);
//
//    /// @brief Draw the undistorted chessboard corners on the frame
//    /// @param frame 
//    /// @param canvas 
//    /// @param cam_id 
//    /// @param corner_idx 
//    void draw_undistorted_chessboard_corners(cv::Mat& frame, cv::Mat& canvas, uint8_t cam_id, int corner_idx=-1);
//
//    /// @brief After calibration, get the reprojection error
//    /// @param cam_id 
//    /// @return the reprojection error, -1 if the camera is not calibrated or out of bounds, -2 if the data is not valid
//    double get_reprojection_error(uint8_t cam_id);
//
//    /// @brief reset the calibration process
//    /// @param is_two_cameras whether the calibration is based on stereo
//    /// @return whether the restart is successful
//    bool restart(bool is_two_cameras);
//
//    /// @brief get the calibration data of the id-th camera
//    /// @param cam_id 
//    /// @param camera_matrix 
//    /// @param dist_coeffs 
//    /// @return whether the data is successfully retrieved
//    bool get_intrinsic_matrix(uint8_t cam_id, cv::Mat& camera_matrix, cv::Mat& dist_coeffs);
//
//    /// @brief get the extrinsic matrix of the id-th camera
//    /// @param cam_id, the camera id, only 0 and 1 are supported
//    /// @param R, the rotation matrix
//    /// @param T, the translation vector
//    /// @return whether the data is successfully retrieved
//    bool get_extrinsic_matrix(uint8_t cam_id, cv::Mat& R, cv::Mat& T);
//
//    /// @brief get the affine matrix of the id-th camera
//    /// @param cam_id, the camera id, only 0 and 1 are supported
//    /// @param r, the rotation matrix
//    /// @param t, the translation vector
//    /// @param scale, the scale to the other camera
//    /// @return whether the data is successfully retrieved
//    bool get_affine_matrix(uint8_t cam_id, cv::Mat& r, cv::Mat& t, double& scale);
//
//    /// @brief get the affine matrix of the id-th camera (2x3)
//    /// @param cam_id 
//    /// @param affine_matrix 
//    /// @return whether the data is successfully retrieved
//    bool get_affine_matrix(uint8_t cam_id, cv::Mat& affine_matrix);
//
//    // /// @brief get the estimated pixel shift of the id-th camera
//    // /// @note you should be careful about using this function because this function only aims at compounded camera, 
//    // /// where two cameras share the same view but they have a slight translation.
//    // /// @param cam_id, the camera id, only 0 and 1 are supported
//    // /// @param estimated_pixel_shift, the estimated pixel shift
//    // /// @return whether the data is successfully retrieved
//    // bool get_estimated_pixel_shift(uint8_t cam_id, cv::Point2f& estimated_pixel_shift);
//
//    /// @brief save the calibration result to a json file
//    /// @param file_path 
//    /// @return whether the data is successfully saved
//    bool save_calibration_result_to_json(const std::string& file_path);
//
//    /// @brief load the calibration result from a json file
//    /// @param file_path 
//    /// @return whether the data is successfully loaded
//    bool load_calibration_result_from_json(const std::string& file_path);
//
//    /// @brief transfer the matrix to json
//    /// @param mat 
//    /// @return the json data
//    static nlohmann::ordered_json transfer_mat_to_json(const cv::Mat& mat);
//
//    /// @brief transfer the json to matrix
//    /// @param json_data 
//    /// @return the matrix
//    static cv::Mat load_mat_from_json(const nlohmann::json& json_data);
//
//    /// @brief save the json to a file
//    /// @param json_data 
//    /// @param file_path 
//    /// @return whether the data is successfully saved
//    static bool save_json_to_file(const nlohmann::json& json_data, const std::string& file_path);
//
//    /// @brief load the json from a file
//    /// @param json_data 
//    /// @param file_path 
//    /// @return whether the data is successfully loaded
//    static bool load_json_from_file(nlohmann::json& json_data, const std::string& file_path);
//
//private:
//    // calibration and camera params
//    uint16_t width_;
//    uint16_t height_;
//    uint16_t rows_;
//    uint16_t cols_;
//    double square_width_;
//    double square_height_;
//    uint16_t image_num_;
//    bool is_two_cameras_;
//
//    // camera calibration related
//    std::vector<CalibData> calib_data_;
//    std::vector<ObjPts> obj_pts_vec_;
//    cv::Mat E_; // essential matrix 
//    cv::Mat F_; // fundamental matrix
//
//    // thread related
//    std::mutex insert_mutex_;
//};
//}