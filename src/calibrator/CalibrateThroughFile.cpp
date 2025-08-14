#include <fstream>
#include "CalibrateThroughFile.hpp"

using json = nlohmann::json;

CalibrateThroughFile::~CalibrateThroughFile()
{
}

void CalibrateThroughFile::readParams()
{
    std::ifstream input_file(config_file_);
    json camera_data;
    input_file >> camera_data;

    for (auto& [camera_id, camera_info] : camera_data.items()) {
        if (camera_info["camera_serial"] == "dvs")
        {
            std::vector<double> matrix_data = camera_info["camera_matrix"]["data"];
            dvs_params_.camera_matrix = cv::Mat(3, 3, CV_64F, matrix_data.data()).clone();

            std::vector<double> rotation_data = camera_info["rotation"]["1"]["data"];
            dvs_params_.rotation = cv::Mat(3, 3, CV_64F, rotation_data.data()).clone();

            std::vector<double> translation_data = camera_info["translation"]["1"]["data"];
            dvs_params_.translation = cv::Mat(3, 1, CV_64F, translation_data.data()).clone();
        }

        if (camera_info["camera_serial"] == "aps")
        {
            std::vector<double> matrix_data = camera_info["camera_matrix"]["data"];
            aps_params_.camera_matrix = cv::Mat(3, 3, CV_64F, matrix_data.data()).clone();

            std::vector<double> rotation_data = camera_info["rotation"]["0"]["data"];
            aps_params_.rotation = cv::Mat(3, 3, CV_64F, rotation_data.data()).clone();

            std::vector<double> translation_data = camera_info["translation"]["0"]["data"];
            aps_params_.translation = cv::Mat(3, 1, CV_64F, translation_data.data()).clone();
        }
    }


}

cv::Mat CalibrateThroughFile::getDvsToApsHomographyMatrix(double distance)
{
	// 计算单应性矩阵 H = K2 * (R + t*nᵀ/d) * K1⁻¹
	cv::Mat H = aps_params_.camera_matrix * (aps_params_.rotation + aps_params_.translation * plane_normal_.t() / distance) * dvs_params_.camera_matrix.inv();
	// 归一化矩阵
	H = H / H.at<double>(2, 2);

	return H;
}

cv::Mat CalibrateThroughFile::getApsToDvsHomographyMatrix(double distance)
{
    cv::Mat H = dvs_params_.camera_matrix * (aps_params_.rotation + aps_params_.translation * plane_normal_.t() / distance) * aps_params_.camera_matrix.inv();
    H = H / H.at<double>(2, 2);

    return H;
}


cv::Mat CalibrateThroughFile::warpImage(const cv::Mat& src, const cv::Mat& H, cv::Size dstSize) {
	cv::Mat dst;
	warpPerspective(src, dst, H, dstSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
	return dst;
}

