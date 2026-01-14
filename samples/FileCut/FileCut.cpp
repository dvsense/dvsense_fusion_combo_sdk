#include <queue>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include "DvsenseDriver/FileReader/DvsFileReader.h"
#include "DvsenseBase/logging/logger.hh"

using json = nlohmann::json;

int main(int argc, char* argv[]) {
	if (argc < 4) {
		std::cout << "ÓÃ·¨: " << argv[0] << " <input_json> <start_time> <end_time> <output_json>" << std::endl;
		return 1;
	}
	std::string input_json = argv[1];
	double start_time = std::atof(argv[2]);
	double end_time = std::atof(argv[3]);
	std::string output_json = argv[4];

	std::ifstream file(input_json);
	json data = json::parse(file);
	std::string event_file_path = data["dvs_file_path"];
	std::string aps_file_path = data["aps_file_path"];
	dvsense::TimeStamp offset_timestamp = data["aps_offset_timestamp"];

	file.close();

	std::string event_output_path;
	std::string aps_output_path;
	std::string json_output_path;
	if (output_json.empty())           //Use the default output path
	{
		std::string sub_directory = "output-cut-test/";
		event_output_path = event_file_path;
		size_t lastSlash = event_file_path.find_last_of("/");
		if (lastSlash != std::string::npos) {
			event_output_path.insert(lastSlash + 1, sub_directory);
		}
		std::cout << "event_output_path: " << event_output_path << std::endl;

		aps_output_path = aps_file_path;
		lastSlash = aps_file_path.find_last_of("/");
		if (lastSlash != std::string::npos) {
			aps_output_path.insert(lastSlash + 1, sub_directory);
		}
		std::cout << "aps_output_path" << aps_output_path << std::endl;

		json_output_path = input_json;
		lastSlash = input_json.find_last_of("/");
		if (lastSlash != std::string::npos) {
			json_output_path.insert(lastSlash + 1, sub_directory);
		}
		std::cout << "json_output_path" << json_output_path << std::endl;
	}
	else
	{ 
		event_output_path = output_json;
		event_output_path.replace(event_output_path.find("json"), 4, "raw");
		std::cout << "event_output_path: " << event_output_path << std::endl;

		aps_output_path = output_json;
		aps_output_path.replace(aps_output_path.find("json"), 4, "mp4");
		std::cout << "aps_output_path" << aps_output_path << std::endl;

		json_output_path = output_json;
		std::cout << "json_output_path" << json_output_path << std::endl;
	}

	dvsense::TimeStamp cut_start_timestamp = 0;
	dvsense::TimeStamp cut_end_timestamp = 0;

	cv::VideoCapture cap(aps_file_path, cv::CAP_FFMPEG);
	if (!cap.isOpened()) {
		std::cerr << "Error: Could not open video file." << std::endl;
		return -1;
	} 

	double aps_fps = cap.get(cv::CAP_PROP_FPS);
	const int event_analyzer_time = static_cast<int>(std::round(1.f / aps_fps * 1000));
	int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
	int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
	int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

	int start_frame = static_cast<int>(start_time * aps_fps);
	int end_frame = static_cast<int>(end_time * aps_fps);

	if (start_frame >= end_frame || end_frame > total_frames) {
		std::cerr << "Error: Invalid time range." << std::endl;
		return -1;
	}

	cv::VideoWriter writer;
	int fourcc = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
	writer.open(aps_output_path, fourcc, aps_fps, cv::Size(width, height));

	if (!writer.isOpened()) {
		std::cerr << "Error: Could not open video writer." << std::endl;
		return -1;
	}

	cv::Mat frame;
	cap.set(cv::CAP_PROP_POS_FRAMES, start_frame);
	cap.read(frame);
	dvsense::TimeStamp start_time_us = cap.get(cv::CAP_PROP_POS_MSEC) * 1000;
	cut_start_timestamp = offset_timestamp + start_time_us;

	int current_frame = start_frame;

	while (current_frame <= end_frame) {
		if (!cap.read(frame)) {
			std::cerr << "Error: Failed to read frame " << current_frame << std::endl;
			break;
		}
		writer.write(frame);
		current_frame++;
	}

	dvsense::TimeStamp end_time_us = cap.get(cv::CAP_PROP_POS_MSEC) * 1000;
	cut_end_timestamp = offset_timestamp + end_time_us;

	cap.release();
	writer.release();

	dvsense::DvsFile reader = dvsense::DvsFileReader::createFileReader(event_file_path);
	if (!reader->loadFile())
	{
		LOG_ERROR("Load file failed!");
		return -1;
	}

	bool ret = reader->extractEventData(cut_start_timestamp, cut_end_timestamp, event_output_path);
	if (!ret)
	{
		std::cout << "extractEventData failed!" << std::endl;
	}

	json j;
	j["dvs_file_path"] = event_output_path;
	j["aps_file_path"] = aps_output_path;
	j["aps_offset_timestamp"] = cut_start_timestamp;

	std::ofstream file_writer(json_output_path);
	file_writer << j.dump(4);
	file_writer.close();

	return 0;
}