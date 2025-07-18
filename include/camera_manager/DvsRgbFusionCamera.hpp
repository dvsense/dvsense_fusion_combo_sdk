#pragma once
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include "DvsEventCamera.hpp"
#include "RgbCamera.hpp"
#include "ApsTypes.hpp"
#include "DataToVideo.hpp"

#ifdef _WIN32
#define DVSENSE_API __declspec(dllexport)
#else
#define DVSENSE_API
#endif // _WIN32


class DVSENSE_API DvsRgbFusionCamera
{
public:
	/**
	 * \~english @brief Constructor
	 * \~english @param fps Camera frame rate
	 * \~chinese @brief 构造函数
	 * \~chinese @param fps 相机帧率
	 */
	DvsRgbFusionCamera(float fps = 30);

	~DvsRgbFusionCamera();

	/**
	 * \~english @brief Check if the camera is connected
	 * \~english @return true if connected, otherwise false
	 * \~chinese @brief 检查相机是否连接
	 * \~chinese @return 如果连接则返回true，否则返回false
	 */
	const bool isConnected();

	/**
	 * \~english @brief Get camera description
	 * \~english @return @ref CameraDescription
	 * \~chinese @brief 获取相机描述信息
	 * \~chinese @return @ref CameraDescription
	 */
	const dvsense::CameraDescription getDvsDesc()
	{
		return dvs_camera_->getDvsDesc();
	}

	/**
	 * \~english @brief Add a callback function to handle events
	 * \~english @param cb callback function
	 * \~english @return callback id With id you can @ref removeEventsStreamHandleCallback
	 * \~chinese @brief 添加一个回调函数来处理事件流
	 * \~chinese @param cb 回调函数
	 * \~chinese @return 回调函数id，使用此id可以调用 @ref removeEventsStreamHandleCallback 来移除回调函数
	 */
	uint32_t addEventsStreamHandleCallback(const dvsense::EventsStreamHandleCallback& callback);

	/**
	 * \~english @brief Remove a callback function by id
	 * \~english @param callback_id
	 * \~english @return true if removed successfully
	 * \~english @return false if there is no callback function with the corresponding id in the callback function list.
	 * \~chinese @brief ͨ通过id来删除一个回调函数
	 * \~chinese @param callback_id 回调函数id
	 * \~chinese @return 如果成功移除则返回true
	 * \~chinese @return 如果回调函数列表中没有对应id的回调函数，则返回false
	 */
	bool removeEventsStreamHandleCallback(uint32_t callback_id);

	/**
	 * \~english @brief Add a callback function to handle trigger in signal
	 * \~english @param cb callback function
	 * \~english @return callback id With id you can @ref removeTriggerInCallback
	 * \~chinese @brief 添加一个回调函数来处理触发信号
	 * \~chinese @param cb 回调函数
	 * \~chinese @return 回调函数id，使用此id可以调用 @ref removeTriggerInCallback 来移除回调函数
	 */
	uint32_t addTriggerInCallback(const NewTriggerInCallback& newTriggerInCallback);

	/**
	 * \~english @brief Remove a callback function by id
	 * \~english @param callback_id
	 * \~english @return true if removed successfully
	 * \~english @return false if there is no callback function with the corresponding id in the callback function list.
	 * \~chinese @brief ͨ通过id来删除一个回调函数
	 * \~chinese @param callback_id 回调函数id
	 * \~chinese @return 如果成功移除则返回true
	 * \~chinese @return 如果回调函数列表中没有对应id的回调函数，则返回false
	 */
	bool removeTriggerInCallback(uint32_t callback_id);

	/**
	 * \~english @brief Add a callback function to handle aps data
	 * \~english @param cb callback function
	 * \~english @return callback id With id you can @ref removeApsFrameCallback
	 * \~chinese @brief 添加一个回调函数来处理aps数据
	 * \~chinese @param cb 回调函数
	 * \~chinese @return 回调函数id，使用此id可以调用 @ref removeApsFrameCallback 来移除回调函数
	 */
	int addApsFrameCallback(const FrameCallback& frameCallback);

	/**
	 * \~english @brief Remove a callback function by id
	 * \~english @param callback_id
	 * \~english @return true if removed successfully
	 * \~english @return false if there is no callback function with the corresponding id in the callback function list.
	 * \~chinese @brief ͨ通过id来删除一个回调函数
	 * \~chinese @param callback_id 回调函数id
	 * \~chinese @return 如果成功移除则返回true
	 * \~chinese @return 如果回调函数列表中没有对应id的回调函数，则返回false
	 */
	bool removeApsFrameCallback(uint32_t callback_id);

	/**
	 * \~english @brief Start the camera
	 * \~english @return int 0 if success, otherwise return error code
	 * \~chinese @brief 开启相机
	 * \~chinese @return int 如果成功开启则返回0，否则返回错误代码
	 */
	int start();

	/**
	 * \~english @brief Start recording events
	 * \~english @param file_path
	 * \~english @return int 0 if success, otherwise return error code
	 * \~chinese @brief 开始录制事件流
	 * \~chinese @param file_path
	 * \~chinese @return int 如果成功录制则返回0，否则返回错误代码
	 */
	int startRecording(std::string output_dir = "");

	/**
	 * \~english @brief Stop the camera
	 * \~english @return int 0 if success, otherwise return error code
	 * \~chinese @brief ͣ停止相机
	 * \~chinese @return int 如果成功停止则返回0，否则返回错误代码
	 */
	int stop();

	/**
	 * \~english @brief Stop recording events
	 * \~english @return int 0 if success, otherwise return error code
	 * \~chinese @brief ͣ停止ֹ录制事件流
	 * \~chinese @return int 如果成功停止录制则返回0，否则返回错误代码
	 */
	int stopRecording();

	/**
	 * \~english @brief Get the Width of the camera sensor
	 * \~english @return uint16_t
	 * \~chinese @brief 获取相机传感器的宽度
	 * \~chinese @return uint16_t
	 */
	uint16_t getWidth();

	/**
	 * \~english @brief Get the Height of the camera sensor
	 * \~english @return uint16_t
	 * \~chinese @brief 获取相机传感器的高度
	 * \~chinese @return uint16_t
	 */
	uint16_t getHeight();

private:
	std::shared_ptr<dvsense::DvsEventCamera> dvs_camera_;
	std::unique_ptr<RgbCamera> rgb_camera_;
	std::vector<dvsense::CameraDescription> dvs_camera_descs_;

	float aps_fps_;
	std::mutex event_buffer_mutex_;
	std::mutex frame_buffer_mutex_;

	std::map<int, const FrameCallback> frame_callbacks_;

	int frame_callback_id_num_ = 0;

	std::shared_ptr<dvsense::Event2DVector> event_buffer_;

	std::string getCurrentTime();

	// ----- sync -----
	void extTriggerSyncCallback();

	bool findCamera();
	bool openCamera();

	dvsense::CameraDescription camera_desc_;

	std::shared_ptr<DataToVideo> aps_to_mp4_;
	std::shared_ptr<DataToVideo> aps_decoder_;
	bool aps_is_recording_ = false;
	dvsense::TimeStamp aps_save_ts_offset_;

};