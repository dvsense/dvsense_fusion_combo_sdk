#include "calibration.hpp"
#include <filesystem>
// #include <unordered_set>

namespace dvsense
{
DvsCalibration::DvsCalibration() {}

DvsCalibration::DvsCalibration(uint16_t width, 
                               uint16_t height, 
                               uint16_t rows, 
                               uint16_t cols, 
                               double square_width, 
                               double square_height, 
                               uint16_t image_num, 
                               bool is_two_cameras)
: width_(width), 
  height_(height), 
  rows_(rows), 
  cols_(cols), 
  square_width_(square_width), 
  square_height_(square_height), 
  image_num_(image_num), 
  is_two_cameras_(is_two_cameras)
{
    if (is_two_cameras_) {
        calib_data_.resize(2);
    } else {
        calib_data_.resize(1);
    }

}

DvsCalibration::~DvsCalibration() {}

bool DvsCalibration::insert_frame(const cv::Mat& frame, uint8_t cam_id)
{
    if (cam_id >= calib_data_.size()) {
        std::cerr << "Error: insert_frame() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return false;
    }

    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame.clone();
    }

    Corners corners_cur_frame;
    bool found_corners = cv::findChessboardCornersSB(gray, cv::Size(cols_, rows_), corners_cur_frame, cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY);

    if (found_corners)
    {
        ObjPts obj_pts;
        for(int i = 0; i < rows_; i++) {
            for(int j = 0; j < cols_; j++) {
                obj_pts.push_back(cv::Point3f(j * square_width_, i * square_height_, 0.0));
            }
        }
        obj_pts_vec_.push_back(obj_pts);
        calib_data_[cam_id].corners_vec.push_back(corners_cur_frame);
    }
    return found_corners;
}

bool DvsCalibration::insert_frame(const cv::Mat& frame, uint8_t cam_id, int index)
{
    if (cam_id >= calib_data_.size()) {
        std::cerr << "Error: insert_frame() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return false;
    }

    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame.clone();
    }

    Corners corners_cur_frame;
    bool found_corners = cv::findChessboardCornersSB(gray, cv::Size(cols_, rows_), corners_cur_frame, cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ACCURACY);

    if (found_corners)
    {
        ObjPts obj_pts;
        for(int i = 0; i < rows_; i++) {
            for(int j = 0; j < cols_; j++) {
                obj_pts.push_back(cv::Point3f(j * square_width_, i * square_height_, 0.0));
            }
        }

        std::lock_guard<std::mutex> lock(insert_mutex_);
        calib_data_[cam_id].insert_index.push_back(index);
        obj_pts_vec_.push_back(obj_pts);
        calib_data_[cam_id].corners_vec.push_back(corners_cur_frame);
    }
    return found_corners;
}

bool DvsCalibration::delete_frame(int index)
{
    if((size_t)index >= calib_data_[0].corners_vec.size()) {
        std::cerr << "Error: delete_frame() is called, out of frame bounds" << std::endl;
        return false;
    }

    if(index == -1) {
        if(obj_pts_vec_.empty()) {
            std::cerr << "Error: delete_frame() is called, but the object points are empty" << std::endl;
            return false;
        }
        obj_pts_vec_.pop_back();
        calib_data_[0].corners_vec.pop_back();
        calib_data_[0].insert_index.pop_back();
        if(is_two_cameras_) {
            calib_data_[1].corners_vec.pop_back();
            calib_data_[1].insert_index.pop_back();
        }
    } else {
        obj_pts_vec_.erase(obj_pts_vec_.begin() + index);
        calib_data_[0].corners_vec.erase(calib_data_[0].corners_vec.begin() + index);
        calib_data_[0].insert_index.erase(calib_data_[0].insert_index.begin() + index);
        if(is_two_cameras_) {
            calib_data_[1].corners_vec.erase(calib_data_[1].corners_vec.begin() + index);
            calib_data_[1].insert_index.erase(calib_data_[1].insert_index.begin() + index);
        }
    }
    return true;
}

bool DvsCalibration::insert_frames(const cv::Mat& frame_cam_0, const cv::Mat& frame_cam_1)
{
    if (calib_data_.size() != 2) {
        std::cerr << "Error: insert_frames() is called, but the calibration is not stereo." << std::endl;
        return false;
    }

    cv::Mat gray_cam_0, gray_cam_1;
    if (frame_cam_0.channels() == 3) {
        cv::cvtColor(frame_cam_0, gray_cam_0, cv::COLOR_BGR2GRAY);
    } else {
        gray_cam_0 = frame_cam_0.clone();
    }

    if (frame_cam_1.channels() == 3) {
        cv::cvtColor(frame_cam_1, gray_cam_1, cv::COLOR_BGR2GRAY);
    } else {
        gray_cam_1 = frame_cam_1.clone();
    }

    Corners corners_cur_frame_0, corners_cur_frame_1;
    bool found_corners_0 = cv::findChessboardCornersSB(gray_cam_0, cv::Size(cols_, rows_), corners_cur_frame_0);
    bool found_corners_1 = cv::findChessboardCornersSB(gray_cam_1, cv::Size(cols_, rows_), corners_cur_frame_1);

    if (found_corners_0 && found_corners_1)
    {
        cv::cornerSubPix(gray_cam_0, corners_cur_frame_0, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        cv::cornerSubPix(gray_cam_1, corners_cur_frame_1, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        ObjPts obj_pts;
        for(int i = 0; i < rows_; i++) {
            for(int j = 0; j < cols_; j++) {
                obj_pts.push_back(cv::Point3f(j * square_width_, i * square_height_, 0.0));
            }
        }
        obj_pts_vec_.push_back(obj_pts);
        calib_data_[0].corners_vec.push_back(corners_cur_frame_0);
        calib_data_[1].corners_vec.push_back(corners_cur_frame_1);
    }
    return found_corners_0 && found_corners_1;
}

bool DvsCalibration::insert_frames(const cv::Mat& frame_cam_0, const cv::Mat& frame_cam_1, int index_0, int index_1)
{
    if (calib_data_.size() != 2) {
        std::cerr << "Error: insert_frames() is called, but the calibration is not stereo." << std::endl;
        return false;
    }

    cv::Mat gray_cam_0, gray_cam_1;
    if (frame_cam_0.channels() == 3) {
        cv::cvtColor(frame_cam_0, gray_cam_0, cv::COLOR_BGR2GRAY);
    } else {
        gray_cam_0 = frame_cam_0.clone();
    }

    if (frame_cam_1.channels() == 3) {
        cv::cvtColor(frame_cam_1, gray_cam_1, cv::COLOR_BGR2GRAY);
    } else {
        gray_cam_1 = frame_cam_1.clone();
    }

    Corners corners_cur_frame_0, corners_cur_frame_1;
    bool found_corners_0 = cv::findChessboardCornersSB(gray_cam_0, cv::Size(cols_, rows_), corners_cur_frame_0);
    bool found_corners_1 = cv::findChessboardCornersSB(gray_cam_1, cv::Size(cols_, rows_), corners_cur_frame_1);

    if (found_corners_0 && found_corners_1)
    {
        cv::cornerSubPix(gray_cam_0, corners_cur_frame_0, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        cv::cornerSubPix(gray_cam_1, corners_cur_frame_1, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        ObjPts obj_pts;
        for(int i = 0; i < rows_; i++) {
            for(int j = 0; j < cols_; j++) {
                obj_pts.push_back(cv::Point3f(j * square_width_, i * square_height_, 0.0));
            }
        }
        std::lock_guard<std::mutex> lock(insert_mutex_);
        obj_pts_vec_.push_back(obj_pts);
        calib_data_[0].corners_vec.push_back(corners_cur_frame_0);
        calib_data_[1].corners_vec.push_back(corners_cur_frame_1);
        calib_data_[0].insert_index.push_back(index_0);
        calib_data_[1].insert_index.push_back(index_1);
    }
    return found_corners_0 && found_corners_1;
}

bool DvsCalibration::sort_frames()
{
    if(calib_data_[0].insert_index.empty() || (is_two_cameras_ && calib_data_[1].insert_index.empty())) {
        std::cerr << "Error: sort_frames() is called, but the insert index is empty" << std::endl;
        return false;
    }

    if(calib_data_[0].insert_index.size() != calib_data_[0].corners_vec.size() || 
       (is_two_cameras_ && calib_data_[1].insert_index.size() != calib_data_[1].corners_vec.size())) {
        std::cerr << "Error: sort_frames() is called, but the insert index is not the same size as the corners vector" << std::endl;
        return false;
    }

    std::unique_lock<std::mutex> lock(insert_mutex_);
    // For each camera
    for(size_t cam = 0; cam < calib_data_.size(); cam++) {
        // Create vector of indices and positions
        std::vector<size_t> positions(calib_data_[cam].insert_index.size());
        for(size_t i = 0; i < positions.size(); i++) {
            positions[i] = i;
        }

        // Sort positions based on insert_index values
        std::sort(positions.begin(), positions.end(),
            [this, cam](size_t i1, size_t i2) {
                return calib_data_[cam].insert_index[i1] < calib_data_[cam].insert_index[i2];
            });

        // Create temporary vectors to store sorted data
        std::vector<Corners> sorted_corners = calib_data_[cam].corners_vec;
        std::vector<int> sorted_indices = calib_data_[cam].insert_index;

        // Rearrange data according to sorted positions
        for(size_t i = 0; i < positions.size(); i++) {
            calib_data_[cam].corners_vec[i] = sorted_corners[positions[i]];
            calib_data_[cam].insert_index[i] = sorted_indices[positions[i]];
        }
    }

    return true;
}

double DvsCalibration::calibrate_single(uint8_t cam_id)
{
    if (cam_id >= calib_data_.size()) {
        std::cerr << "Error: calibrate_single() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return -1;
    }

    if(calib_data_[cam_id].corners_vec.size() < image_num_) {
        std::cerr << "Error: calibrate_single() is called, but the number of images is less than the required number of images." << std::endl;
        return -1;
    }

    double reprojection_error = cv::calibrateCamera(obj_pts_vec_, 
                                                    calib_data_[cam_id].corners_vec, 
                                                    cv::Size(width_, height_), 
                                                    calib_data_[cam_id].camera_matrix, 
                                                    calib_data_[cam_id].dist_coeffs, 
                                                    calib_data_[cam_id].rvecs, 
                                                    calib_data_[cam_id].tvecs);
    calib_data_[cam_id].is_calibrated = true;
    return reprojection_error;
}

double DvsCalibration::calibrate_single()
{
    if (calib_data_.size() < 1) {
        std::cerr << "Error: calibrate_single() is called, but the calibration is not single." << std::endl;
        return -1;
    }

    return calibrate_single(0);
}

// bool DvsCalibration::filter_corners(double tolerance, int iter_num, float sample_ratio)
// {
//     if(calib_data_.size() != 2) {
//         std::cerr << "Error: filter_corners() is called, but the calibration is not stereo." << std::endl;
//         return false;
//     }

//     if(tolerance <= 0 || iter_num <= 0 || sample_ratio <= 0 || sample_ratio > 1) {
//         std::cerr << "Error: filter_corners() is called with invalid parameters" << std::endl;
//         return false;
//     }

//     std::vector<std::vector<cv::Point2f>> shift_corners_vec;
//     for(size_t i = 0; i < obj_pts_vec_.size(); i++) {
//         std::vector<cv::Point2f> shift_corners;
//         for(size_t j = 0; j < obj_pts_vec_[i].size(); j++) {
//             shift_corners.push_back(calib_data_[0].corners_vec[i][j] - calib_data_[1].corners_vec[i][j]);
//         }
//         shift_corners_vec.push_back(shift_corners);
//     }

//     int average_error = 0;
//     for(int iter = 0; iter < iter_num; iter++) {
//         size_t sample_size = 0;
//         for(size_t i = 0; i < obj_pts_vec_.size(); i++) {
//             sample_size += obj_pts_vec_[i].size();
//         }

//         sample_size = sample_size * sample_ratio;
//         if(sample_size <= 10) {
//             std::cout << "Warning: filter_corners() is called, but the corners are too few, the filtering result could be inaccurate" << std::endl;
//             return true;
//         }

//         cv::Point2f pixel_shift {0, 0};

//         // Randomly select corner
//         std::unordered_set<int> selected_indices;
//         while(selected_indices.size() < sample_size) {
//             int i = rand() % obj_pts_vec_.size();
//             int j = rand() % obj_pts_vec_[i].size();
//             if(selected_indices.count(j * cols_ + i)) {
//                 continue;
//             }
//             selected_indices.insert(j * cols_ + i);
//             pixel_shift.x += shift_corners_vec[i][j].x;
//             pixel_shift.y += shift_corners_vec[i][j].y;
//         }

//         pixel_shift.x /= sample_size;
//         pixel_shift.y /= sample_size;

//         for(size_t i = 0; i < obj_pts_vec_.size(); i++) {
//             for(size_t j = obj_pts_vec_[i].size() - 1; j >= 0; j--) {
//                 if(std::abs(shift_corners_vec[i][j].x - pixel_shift.x) > tolerance || std::abs(shift_corners_vec[i][j].y - pixel_shift.y) > tolerance) {
//                     obj_pts_vec_[i].erase(obj_pts_vec_[i].begin() + j);
//                     calib_data_[0].corners_vec[i].erase(calib_data_[0].corners_vec[i].begin() + j);
//                     calib_data_[1].corners_vec[i].erase(calib_data_[1].corners_vec[i].begin() + j);
//                     shift_corners_vec[i].erase(shift_corners_vec[i].begin() + j);
//                     j--;
//                     break;
//                 }
//             }
//         }
//     }
// }

double DvsCalibration::calibrate_stereo()
{
    if (calib_data_.size() != 2) {
        std::cerr << "Error: calibrate_stereo() is called, but the calibration is not stereo." << std::endl;
        return -1;
    }

    if(calib_data_[0].corners_vec.size() < image_num_ || calib_data_[1].corners_vec.size() < image_num_) {
        std::cerr << "Error: calibrate_stereo() is called, but the number of images is less than the required number of images." << std::endl;
        return -1;
    }

    if(calib_data_[0].corners_vec.size() != calib_data_[1].corners_vec.size()) {
        std::cerr << "Error: calibrate_stereo() is called, but the number of object points is not the same for both cameras." << std::endl;
        return -1;
    }

    if(!calib_data_[0].is_calibrated) {
        calibrate_single(0);
    }
    if(!calib_data_[1].is_calibrated) {
        calibrate_single(1);
    }

    double reprojection_error = cv::stereoCalibrate(obj_pts_vec_, 
                                                    calib_data_[0].corners_vec, 
                                                    calib_data_[1].corners_vec, 
                                                    calib_data_[0].camera_matrix, 
                                                    calib_data_[0].dist_coeffs, 
                                                    calib_data_[1].camera_matrix, 
                                                    calib_data_[1].dist_coeffs, 
                                                    cv::Size(width_, height_), 
                                                    calib_data_[0].R, 
                                                    calib_data_[0].T, 
                                                    E_,
                                                    F_,
                                                    cv::CALIB_FIX_INTRINSIC);

    // Calculate R and T for the second camera based on first camera's transformation
    calib_data_[1].R = calib_data_[0].R.t();
    calib_data_[1].T = -calib_data_[0].R.t() * calib_data_[0].T;

    return reprojection_error;
}

double DvsCalibration::calibrate_affine()
{
    if (calib_data_.size() != 2) {
        std::cerr << "Error: calibrate_affine() is called, but the calibration is not stereo." << std::endl;
        return -1;
    }

    if (calib_data_[0].corners_vec.empty() || calib_data_[1].corners_vec.empty()) {
        std::cerr << "Error: calibrate_affine() is called, but no corners detected." << std::endl;
        return -1;
    }

    if(calib_data_[0].corners_vec.size() != calib_data_[1].corners_vec.size()) {
        std::cerr << "Error: calibrate_affine() is called, but the number of object points is not the same for both cameras." << std::endl;
        return -1;
    }

    if(!calib_data_[0].is_calibrated) {
        calibrate_single(0);
    }
    if(!calib_data_[1].is_calibrated) {
        calibrate_single(1);
    }

    // Initialize optimization variables
    cv::Mat r = cv::Mat::eye(2, 2, CV_64F); // 2x2 rotation matrix
    cv::Mat t = cv::Mat::zeros(2, 1, CV_64F); // 2x1 translation vector
    double scale = 1.0;

    // Prepare data for optimization
    std::vector<cv::Point2f> src_pts;
    std::vector<cv::Point2f> dst_pts;
    for (size_t i = 0; i < calib_data_[0].corners_vec.size(); i++) {
        src_pts.insert(src_pts.end(), calib_data_[0].corners_vec[i].begin(), calib_data_[0].corners_vec[i].end());
        dst_pts.insert(dst_pts.end(), calib_data_[1].corners_vec[i].begin(), calib_data_[1].corners_vec[i].end());
    }

    // Use cv::estimateAffinePartial2D to get initial guess
    cv::Mat init_transform = cv::estimateAffinePartial2D(src_pts, dst_pts);
    if (!init_transform.empty()) {
        r = init_transform(cv::Rect(0, 0, 2, 2));
        t = init_transform(cv::Rect(2, 0, 1, 2));
        scale = sqrt(r.at<double>(0,0)*r.at<double>(0,0) + r.at<double>(0,1)*r.at<double>(0,1));
        r = r / scale;
    }

    // Calculate reprojection error
    double total_error = 0;
    for (size_t i = 0; i < src_pts.size(); i++) {
        cv::Mat p1 = (cv::Mat_<double>(2,1) << src_pts[i].x, src_pts[i].y);
        cv::Mat p2_pred = scale * (r * p1 + t);
        cv::Point2f pred_pt(p2_pred.at<double>(0), p2_pred.at<double>(1));
        cv::Point2f diff = pred_pt - dst_pts[i];
        total_error += sqrt(diff.x*diff.x + diff.y*diff.y);
    }
    double avg_error = total_error / src_pts.size();

    // Store results
    calib_data_[0].scale = scale;
    calib_data_[0].r = r;
    calib_data_[0].t = t;

    calib_data_[1].scale = 1.0 / scale;
    calib_data_[1].r = r.t();
    calib_data_[1].t = -r.t() * t;

    return avg_error;
}

// cv::Point2f DvsCalibration::estimate_pixel_shift(std::vector<cv::Mat>& frames_cam_0, std::vector<cv::Mat>& frames_cam_1)
// {
//     if(!is_two_cameras_) {
//         std::cerr << "Error: estimate_pixel_shift() is called, but the calibration is not based on stereo." << std::endl;
//         return cv::Point2f(0, 0);
//     }

//     if(!calib_data_[0].is_calibrated && !calib_data_[1].is_calibrated) {
//         std::cerr << "Error: estimate_pixel_shift() is called, but the cameras are not calibrated" << std::endl;
//         return cv::Point2f(0, 0);
//     }

//     if(frames_cam_0.size() != frames_cam_1.size()) {
//         std::cerr << "Error: estimate_pixel_shift() is called, but the number of frames is not the same for both cameras" << std::endl;
//         return cv::Point2f(0, 0);
//     }

//     cv::Point2f estimated_pixel_shift_sum {0, 0};
//     size_t corners_count = 0;
//     for(size_t i = 0; i < frames_cam_1.size(); i++) {
//         cv::Mat frame_cam_undistoted_0, frame_cam_undistoted_1;
//         cv::undistort(frames_cam_0[i], frame_cam_undistoted_0, calib_data_[0].camera_matrix, calib_data_[0].dist_coeffs);
//         cv::undistort(frames_cam_1[i], frame_cam_undistoted_1, calib_data_[1].camera_matrix, calib_data_[1].dist_coeffs);

//         if(frame_cam_undistoted_0.channels() == 3) {
//             cv::cvtColor(frame_cam_undistoted_0, frame_cam_undistoted_0, cv::COLOR_BGR2GRAY);
//         }
//         if(frame_cam_undistoted_1.channels() == 3) {
//             cv::cvtColor(frame_cam_undistoted_1, frame_cam_undistoted_1, cv::COLOR_BGR2GRAY);
//         }

//         Corners corners_cam_0, corners_cam_1;
//         bool found_corners_0 = cv::findChessboardCornersSB(frame_cam_undistoted_0, cv::Size(cols_, rows_), corners_cam_0);
//         bool found_corners_1 = cv::findChessboardCornersSB(frame_cam_undistoted_1, cv::Size(cols_, rows_), corners_cam_1);

//         if(found_corners_0 && found_corners_1) {
//             cv::cornerSubPix(frame_cam_undistoted_0, corners_cam_0, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
//             cv::cornerSubPix(frame_cam_undistoted_1, corners_cam_1, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

//             for(size_t j = 0; j < corners_cam_0.size(); j++) {
//                 estimated_pixel_shift_sum += corners_cam_0[j] - corners_cam_1[j];
//             }

//             corners_count += corners_cam_0.size();
//         }
//     }

//     calib_data_[0].estimated_pixel_shift.x = estimated_pixel_shift_sum.x / corners_count;
//     calib_data_[0].estimated_pixel_shift.y = estimated_pixel_shift_sum.y / corners_count;
//     calib_data_[1].estimated_pixel_shift = -calib_data_[0].estimated_pixel_shift;

//     return calib_data_[0].estimated_pixel_shift;
// }

void DvsCalibration::draw_chessboard_corners(cv::Mat& frame, cv::Mat& canvas, uint8_t cam_id, int corner_idx)
{
    if (cam_id >= calib_data_.size()) {
        std::cerr << "Error: draw_chessboard_corners() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return;
    }

    if(calib_data_[cam_id].corners_vec.empty()) {
        std::cerr << "Error: draw_chessboard_corners() is called, but the corners are not found" << std::endl;
        return;
    }

    if(frame.channels() == 1) {
        cv::cvtColor(frame, canvas, cv::COLOR_GRAY2BGR);
    } else {
        canvas = frame.clone();
    }

    if(corner_idx == -1) {
        for(size_t i = 0; i < calib_data_[cam_id].corners_vec.back().size(); i++) {
            cv::circle(canvas, calib_data_[cam_id].corners_vec.back()[i], 2, cv::Scalar(0, 0, 255), -1);
        }
    } else {
        for(size_t i = 0; i < calib_data_[cam_id].corners_vec[corner_idx].size(); i++) {
            cv::circle(canvas, calib_data_[cam_id].corners_vec[corner_idx][i], 2, cv::Scalar(0, 0, 255), -1);
        }
    }
}

void DvsCalibration::draw_undistorted_chessboard_corners(cv::Mat& frame, cv::Mat& canvas, uint8_t cam_id, int corner_idx)
{
    if (cam_id >= calib_data_.size()) {
        std::cerr << "Error: draw_undistorted_chessboard_corners() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return;
    }

    if(calib_data_[cam_id].corners_vec.empty()) {
        std::cerr << "Error: draw_undistorted_chessboard_corners() is called, but the corners are not found" << std::endl;
        return;
    }

    if(!calib_data_[cam_id].is_calibrated) {
        std::cerr << "Error: draw_undistorted_chessboard_corners() is called, but the camera is not calibrated" << std::endl;
        return;
    }

    cv::Mat temp;
    draw_chessboard_corners(frame, temp, cam_id, corner_idx);
    cv::undistort(temp, canvas, calib_data_[cam_id].camera_matrix, calib_data_[cam_id].dist_coeffs);
}

double DvsCalibration::get_reprojection_error(uint8_t cam_id)
{
    if (cam_id >= calib_data_.size()) {
        std::cerr << "Error: get_reprojection_error() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return -1;
    }

    if(!calib_data_[cam_id].is_calibrated) {
        std::cerr << "Error: get_reprojection_error() is called for camera " << cam_id << ", but the camera is not calibrated" << std::endl;
        return -1;
    }

    double total_error = 0.0;
    size_t total_points = 0;
    std::vector<cv::Point2f> projected_points;
    cv::Mat rvec, tvec;

    for (size_t i = 0; i < obj_pts_vec_.size(); i++) {
        // Estimate pose for this view
        bool solved = cv::solvePnP(obj_pts_vec_[i], 
                                 calib_data_[cam_id].corners_vec[i],
                                 calib_data_[cam_id].camera_matrix,
                                 calib_data_[cam_id].dist_coeffs,
                                 rvec,
                                 tvec);
        
        if (!solved) {
            std::cerr << "Failed to solve PnP for view " << i << std::endl;
            continue;
        }

        // Project 3D points to image plane
        cv::projectPoints(obj_pts_vec_[i],
                         rvec,
                         tvec,
                         calib_data_[cam_id].camera_matrix,
                         calib_data_[cam_id].dist_coeffs,
                         projected_points);

        // Calculate error
        double err = cv::norm(calib_data_[cam_id].corners_vec[i], projected_points, cv::NORM_L2);
        total_error += err * err;
        total_points += obj_pts_vec_[i].size();
    }

    return std::sqrt(total_error / total_points);
}

bool DvsCalibration::restart(bool is_two_cameras)
{
    calib_data_.clear();
    obj_pts_vec_.clear();

    if (is_two_cameras) {
        calib_data_.resize(2);
    } else {
        calib_data_.resize(1);
    }
    return true;
}

bool DvsCalibration::get_intrinsic_matrix(uint8_t cam_id, cv::Mat& camera_matrix, cv::Mat& dist_coeffs)
{
    if (cam_id >= calib_data_.size()) {
        std::cerr << "Error: get_calib_data() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return false;
    }

    calib_data_[cam_id].camera_matrix.copyTo(camera_matrix);
    calib_data_[cam_id].dist_coeffs.copyTo(dist_coeffs);
    return true;
}

bool DvsCalibration::get_extrinsic_matrix(uint8_t cam_id, cv::Mat& R, cv::Mat& T)
{
    if(!is_two_cameras_) {
        std::cerr << "Error: get_extrinsic_matrix() is called, but the calibration is not based on stereo." << std::endl;
        return false;
    }

    if (cam_id >= calib_data_.size()) {
        std::cerr << "Error: get_extrinsic_matrix() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return false;
    }

    R = calib_data_[cam_id].R;
    T = calib_data_[cam_id].T;

    return true;
}

bool DvsCalibration::get_affine_matrix(uint8_t cam_id, cv::Mat& r, cv::Mat& t, double& scale)
{
    if(!is_two_cameras_) {
        std::cerr << "Error: get_affine_matrix() is called, but the calibration is not based on stereo." << std::endl;
        return false;
    }

    if(cam_id >= calib_data_.size()) {
        std::cerr << "Error: get_affine_matrix() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return false;
    }

    r = calib_data_[cam_id].r;
    t = calib_data_[cam_id].t;
    scale = calib_data_[cam_id].scale;
    return true;
}

bool DvsCalibration::get_affine_matrix(uint8_t cam_id, cv::Mat& affine_matrix)
{
    if(!is_two_cameras_) {
        std::cerr << "Error: get_affine_matrix() is called, but the calibration is not based on stereo." << std::endl;
        return false;
    }

    if(cam_id >= calib_data_.size()) {
        std::cerr << "Error: get_affine_matrix() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
        return false;
    }

    affine_matrix = cv::Mat::zeros(2, 3, CV_64F);
    affine_matrix.at<double>(0, 0) = calib_data_[cam_id].r.at<double>(0, 0) * calib_data_[cam_id].scale;
    affine_matrix.at<double>(0, 1) = calib_data_[cam_id].r.at<double>(0, 1) * calib_data_[cam_id].scale;
    affine_matrix.at<double>(0, 2) = calib_data_[cam_id].t.at<double>(0);
    affine_matrix.at<double>(1, 0) = calib_data_[cam_id].r.at<double>(1, 0) * calib_data_[cam_id].scale;
    affine_matrix.at<double>(1, 1) = calib_data_[cam_id].r.at<double>(1, 1) * calib_data_[cam_id].scale;
    affine_matrix.at<double>(1, 2) = calib_data_[cam_id].t.at<double>(1);
    return true;
}

// bool DvsCalibration::get_estimated_pixel_shift(uint8_t cam_id, cv::Point2f& estimated_pixel_shift)
// {
//     if(!is_two_cameras_) {
//         std::cerr << "Error: get_estimated_pixel_shift() is called, but the calibration is not based on stereo." << std::endl;
//         return false;
//     }

//     if (cam_id >= calib_data_.size()) {
//         std::cerr << "Error: get_estimated_pixel_shift() is called for camera " << cam_id << ", out of camera bounds" << std::endl;
//         return false;
//     }

//     if(!calib_data_[cam_id].is_calibrated) {
//         std::cerr << "Error: get_estimated_pixel_shift() is called for camera " << cam_id << ", but the camera is not calibrated" << std::endl;
//         return false;
//     }

//     estimated_pixel_shift = calib_data_[cam_id].estimated_pixel_shift;
//     return true;
// }

bool DvsCalibration::save_calibration_result_to_json(const std::string& file_path)
{
    if(calib_data_.empty()) {
        std::cerr << "Error: save_calibration_result_to_json() is called, but the calibration data is empty" << std::endl;
        return false;
    }

    if(!calib_data_[0].is_calibrated || (is_two_cameras_ && !calib_data_[1].is_calibrated)) {
        std::cerr << "Error: save_calibration_result_to_json() is called, but the calibration is not completed" << std::endl;
        return false;
    }

    nlohmann::json calib_result;
    nlohmann::json calib_result0;
    {
        nlohmann::json camera_matrix = transfer_mat_to_json(calib_data_[0].camera_matrix);
        nlohmann::json dist_coeffs = transfer_mat_to_json(calib_data_[0].dist_coeffs);
        nlohmann::json R = transfer_mat_to_json(calib_data_[0].R);
        nlohmann::json T = transfer_mat_to_json(calib_data_[0].T);
        nlohmann::json r = transfer_mat_to_json(calib_data_[0].r);
        nlohmann::json t = transfer_mat_to_json(calib_data_[0].t);
        nlohmann::json scale = calib_data_[0].scale;

        calib_result0 = {
            {"camera_matrix", camera_matrix},
            {"dist_coeffs", dist_coeffs},
            {"rotation", R},
            {"translation", T},
            {"affine_rotation", r},
            {"affine_translation", t},
            {"affine_scale", scale}
        };
    }

    if(is_two_cameras_) {
        nlohmann::json calib_result1;
        {
            nlohmann::json camera_matrix = transfer_mat_to_json(calib_data_[1].camera_matrix);
            nlohmann::json dist_coeffs = transfer_mat_to_json(calib_data_[1].dist_coeffs);
            nlohmann::json R = transfer_mat_to_json(calib_data_[1].R);
            nlohmann::json T = transfer_mat_to_json(calib_data_[1].T);
            nlohmann::json r = transfer_mat_to_json(calib_data_[1].r);
            nlohmann::json t = transfer_mat_to_json(calib_data_[1].t);
            nlohmann::json scale = calib_data_[1].scale;

            calib_result1 = {
                {"camera_matrix", camera_matrix},
                {"dist_coeffs", dist_coeffs},
                {"rotation", R},
                {"translation", T},
                {"affine_rotation", r},
                {"affine_translation", t},
                {"affine_scale", scale}
            };
        }

        calib_result = {
            {"0", calib_result0},
            {"1", calib_result1}
        };

        return save_json_to_file(calib_result, file_path);
    } else {
        calib_result = {
            {"0", calib_result0}
        };

        return save_json_to_file(calib_result, file_path);
    }

    return false;
}

bool DvsCalibration::load_calibration_result_from_json(const std::string& file_path)
{
    nlohmann::json json_data;
    if(!load_json_from_file(json_data, file_path)) {
        return false;
    }

    if(json_data.empty()) {
        std::cerr << "Error: load_calibration_result_from_json() is called, but the json file is empty" << std::endl;
        return false;
    }

    if(!json_data.contains("0") && !json_data.contains("1")) {
        std::cerr << "Error: load_calibration_result_from_json() is called, but the json file does not contain any calibration data" << std::endl;
        return false;
    }

    calib_data_.clear();

    if(json_data.contains("1")) {
        is_two_cameras_ = true;
        calib_data_.resize(2);
    } else {
        is_two_cameras_ = false;
        calib_data_.resize(1);
    }

    if(json_data.contains("0")) {
        calib_data_[0].camera_matrix = load_mat_from_json(json_data["0"]["camera_matrix"]);
        calib_data_[0].dist_coeffs = load_mat_from_json(json_data["0"]["dist_coeffs"]);
        calib_data_[0].R = load_mat_from_json(json_data["0"]["rotation"]);
        calib_data_[0].T = load_mat_from_json(json_data["0"]["translation"]);
        calib_data_[0].r = load_mat_from_json(json_data["0"]["affine_rotation"]);
        calib_data_[0].t = load_mat_from_json(json_data["0"]["affine_translation"]);
        calib_data_[0].scale = json_data["0"]["affine_scale"].get<double>();
        calib_data_[0].is_calibrated = true;
    }

    if(json_data.contains("1")) {
        calib_data_[1].camera_matrix = load_mat_from_json(json_data["1"]["camera_matrix"]);
        calib_data_[1].dist_coeffs = load_mat_from_json(json_data["1"]["dist_coeffs"]);
        calib_data_[1].R = load_mat_from_json(json_data["1"]["rotation"]);
        calib_data_[1].T = load_mat_from_json(json_data["1"]["translation"]);
        calib_data_[1].r = load_mat_from_json(json_data["1"]["affine_rotation"]);
        calib_data_[1].t = load_mat_from_json(json_data["1"]["affine_translation"]);
        calib_data_[1].scale = json_data["1"]["affine_scale"].get<double>();
        calib_data_[1].is_calibrated = true;
    }


    return true;
}

nlohmann::ordered_json DvsCalibration::transfer_mat_to_json(const cv::Mat& mat)
{
    nlohmann::ordered_json json_data;
    json_data["rows"] = mat.rows;
    json_data["cols"] = mat.cols;
    json_data["type"] = mat.type();
    std::vector<double> mat_vector;
    if(mat.isContinuous()) {
        mat_vector.assign(mat.ptr<double>(), mat.ptr<double>() + mat.total());
    } else {
        for(int i = 0; i < mat.rows; i++) {
            mat_vector.insert(mat_vector.end(), mat.ptr<double>(i), mat.ptr<double>(i) + mat.cols);
        }
    }
    json_data["data"] = mat_vector;
    return json_data;
}

cv::Mat DvsCalibration::load_mat_from_json(const nlohmann::json& json_data)
{
    cv::Mat mat(json_data["rows"].get<int>(), json_data["cols"].get<int>(), json_data["type"].get<int>());
    std::vector<double> mat_vector = json_data["data"].get<std::vector<double>>();
    memcpy(mat.data, mat_vector.data(), mat_vector.size() * sizeof(double));
    return mat;
}

bool DvsCalibration::save_json_to_file(const nlohmann::json& json_data, const std::string& file_path) {
    try {
        // Create directories if they don't exist
        std::filesystem::path path(file_path);
        if (!path.parent_path().empty()) {
            std::filesystem::create_directories(path.parent_path());
        }

        // Open file with exception handling
        std::ofstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << file_path << " for writing" << std::endl;
            return false;
        }

        // Write JSON with pretty printing if requested

        file << json_data.dump(2);

        file.close();

        // Verify file was written
        if (!std::filesystem::exists(file_path)) {
            std::cerr << "Error: File " << file_path << " was not created" << std::endl;
            return false;
        }

        // Verify file size
        if (std::filesystem::file_size(file_path) == 0) {
            std::cerr << "Error: File " << file_path << " is empty" << std::endl;
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        std::cerr << "Error saving JSON: " << e.what() << std::endl;
        return false;
    }
}

bool DvsCalibration::load_json_from_file(nlohmann::json& json_data, const std::string& file_path) {
    try {
        // Check if file exists
        if (!std::filesystem::exists(file_path)) {
            std::cerr << "Error: File " << file_path << " does not exist" << std::endl;
            return false;
        }

        // Open and read file
        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << file_path << " for reading" << std::endl;
            return false;
        }

        // Parse JSON
        file >> json_data;
        file.close();
        return true;

    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error loading JSON: " << e.what() << std::endl;
        return false;
    }
}
} // namespace dvsense

