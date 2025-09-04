#include "DvsRgbFusionCamera/camera_manager/DataToVideo.hpp"
#include "DvsenseBase/logging/logger.hh"
#include <string>
#include <chrono>
#if defined(__linux__) && (defined(__arm__) || defined(__aarch64__))
#include <libyuv.h>
#endif

	DataToVideo::DataToVideo()
	{
	}

	DataToVideo::~DataToVideo()
	{
		if (dst_frame_)
		{
			av_frame_free(&dst_frame_);
		}
		if (src_frame_)
		{
			av_frame_free(&src_frame_);
		}

		if (codec_ctx_)
		{
			avcodec_free_context(&codec_ctx_);
		}
		if (fmt_ctx_)
		{
			avformat_free_context(fmt_ctx_);
		}
		if (sws_ctx_)
		{
			sws_freeContext(sws_ctx_);
		}
	}
	
	void DataToVideo::setConverterFrameSize(int src_height, int src_width, int dst_height, int dst_width)
	{
		dst_height_ = dst_height;
		dst_width_ = dst_width;
		src_height_ = src_height;
		src_width_ = src_width;
	}

	void DataToVideo::setConverterFmt(AVPixelFormat src_fmt, AVPixelFormat dst_fmt)
	{
		if (sws_ctx_)
		{
			sws_freeContext(sws_ctx_);
			sws_ctx_ = nullptr;
		}
		// Initializes the scaling context.  SWS_POINT is a fast but low quality scaling algorithm
		sws_ctx_ = sws_getContext(src_width_, src_height_, src_fmt,
								  dst_width_, dst_height_, dst_fmt, SWS_POINT, nullptr, nullptr, nullptr);

		src_fmt_ = src_fmt;
		dst_fmt_ = dst_fmt;
	}

	int DataToVideo::nv12ToRGB(const unsigned char *nv12_data, unsigned char *rgb_data)
	{
#if defined(__linux__) && (defined(__arm__) || defined(__aarch64__))
		const uint8_t *y_plane = nv12_data;
		const uint8_t *uv_plane = nv12_data + src_width_ * src_height_;

		int res = libyuv::NV12ToRAW(
			y_plane, src_width_,	  // Y plane, stride
			uv_plane, src_width_,	  // UV plane, stride
			rgb_data, dst_width_ * 3, // RGB output, stride (3 bytes per pixel)
			src_width_, src_height_	  // width, height
		);
		if (res != 0)
		{
			LOG_ERROR("NV12ToRAW failed with error code: %d", res);
			return -1;
		}
#else
		const uint8_t *srcSlice[2] = {nv12_data, nv12_data + src_width_ * src_height_};
		int srcStride[2] = {src_width_, src_width_}; // NV12 Y  stride = width，UV stride = width
		uint8_t *dstSlice[1] = {rgb_data};
		int dstStride[1] = {dst_width_ * 3}; 
		int res = sws_scale(sws_ctx_, srcSlice, srcStride, 0, src_height_, dstSlice, dstStride);
#endif
		return res;
	}

	int DataToVideo::initVideoConverter()
	{
		av_codec_ = find_preferred_h264_encoder();
		if (!av_codec_)
		{
			LOG_ERROR("H.264 encoder not found.");
			return -1;
		}
		LOG_INFO("Using encoder: %s", av_codec_->name);

		codec_ctx_ = avcodec_alloc_context3(av_codec_);
		codec_ctx_->width = dst_width_;
		codec_ctx_->height = dst_height_;
		codec_ctx_->pix_fmt = dst_fmt_;
		codec_ctx_->time_base = timebase_;
		codec_ctx_->framerate = {framerate_, 1};
		codec_ctx_->gop_size = 1;
		codec_ctx_->keyint_min = 1;
		codec_ctx_->bit_rate = 60000000;

		if (avcodec_open2(codec_ctx_, av_codec_, nullptr) < 0)
		{
			LOG_ERROR("Could not open codec.");
			return -1;
		}

		dst_frame_ = av_frame_alloc();
		dst_frame_->width = dst_width_;
		dst_frame_->height = dst_height_;
		dst_frame_->format = dst_fmt_;
		if (av_frame_get_buffer(dst_frame_, 32) < 0)
		{
			LOG_ERROR("Could not allocate the video frame data");
			return -1;
		}

		src_frame_ = av_frame_alloc();
		src_frame_->format = src_fmt_;
		src_frame_->width = src_width_;
		src_frame_->height = src_height_;
		if (av_frame_get_buffer(src_frame_, 32) < 0)
		{
			LOG_ERROR("Could not allocate the video frame data");
			return -1;
		}

		jpeg_codec_ = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
		if (!jpeg_codec_)
		{
			LOG_ERROR("JPEG codec not found.");
			return -1;
		}

		jpeg_ctx_ = avcodec_alloc_context3(jpeg_codec_);
		if (!jpeg_ctx_)
		{
			LOG_ERROR("Could not allocate JPEG codec context.");
			return -1;
		}
		jpeg_ctx_->pix_fmt = AV_PIX_FMT_YUVJ420P;
		jpeg_ctx_->width = dst_width_;
		jpeg_ctx_->height = dst_height_;
		jpeg_ctx_->time_base = {1, 25};

		if (avcodec_open2(jpeg_ctx_, jpeg_codec_, nullptr) < 0)
		{
			LOG_ERROR("Could not open JPEG codec.");
			avcodec_free_context(&jpeg_ctx_);
			return -1;
		}

		// av_init_packet(&packet_);
		packet_ = *av_packet_alloc();
		packet_.data = nullptr;
		packet_.size = 0;

		return 0;
	}

	int DataToVideo::nv12ToVideo(unsigned char *nv12_data, int64_t ts)
	{
		if (src_fmt_ != AV_PIX_FMT_NV12)
		{
			LOG_ERROR("Sws scale fmt is not AV_PIX_FMT_NV12");
			return -1;
		}
		const uint8_t *src_slice[2] = {
			nv12_data,							 // Y plane
			nv12_data + src_width_ * src_height_ // UV plane
		};
		int src_stride[2] = {
			src_width_, // Y stride
			src_width_	// UV stride
		};

		uint8_t *dst_slice[4];
		int dst_stride[4];

		for (int i = 0; i < 4; ++i)
		{
			dst_slice[i] = dst_frame_->data[i];
			dst_stride[i] = dst_frame_->linesize[i];
		}

		sws_scale(sws_ctx_,
				  src_slice, src_stride,
				  0, src_height_,
				  dst_slice, dst_stride);
				  
		if(ts == -1)
		{
			src_frame_->pts = pts_;
			// set timestamp
			dst_frame_->pts = pts_; // 根据一帧的时间分辨率计算帧的真实时间戳，当前设置1000，即33毫秒一帧

			pts_ += 33333;
		}else{
			src_frame_->pts = ts;
			// set timestamp
			dst_frame_->pts = ts; 
			pts_ = ts;
		}
		
		// LOG_INFO("YUV frame timestamp: %d", dst_frame_->pts);
		int ret;
		if ((ret = avcodec_send_frame(codec_ctx_, dst_frame_)) < 0)
		{
			LOG_ERROR("Error sending a frame for encoding");
			return -1;
		}
		
		while (ret >= 0)
		{
			ret = avcodec_receive_packet(codec_ctx_, &packet_);
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
			{
				break;
			}
			else if (ret < 0)
			{
				//LOG_ERROR("Error during encoding");
				return -1;
			}

			av_packet_rescale_ts(&packet_, codec_ctx_->time_base, av_stream_->time_base);
			packet_.stream_index = av_stream_->index;
			av_interleaved_write_frame(fmt_ctx_, &packet_);
			av_packet_unref(&packet_);
		}
		return 0;
	}

	int DataToVideo::rgbToVideo(const unsigned char *rgb_data, int64_t ts)
	{
		if(src_fmt_ != AV_PIX_FMT_BGR24)
		{
			LOG_ERROR("Sws scale fmt is not AV_PIX_FMT_BGR24");
			return -1;
		}

		memcpy(src_frame_->data[0], rgb_data, src_height_ * src_frame_->linesize[0]);

		sws_scale(sws_ctx_, src_frame_->data, src_frame_->linesize, 0, src_height_, dst_frame_->data, dst_frame_->linesize);

		if(ts == -1)
		{
			src_frame_->pts = pts_;
			// set timestamp
			dst_frame_->pts = pts_; // 根据一帧的时间分辨率计算帧的真实时间戳，当前设置1000，即33毫秒一帧

			pts_ += 33333;
		}else{
			src_frame_->pts = ts;
			// set timestamp
			dst_frame_->pts = ts; 
			pts_ = ts;
		}

		int ret;
		if ((ret = avcodec_send_frame(codec_ctx_, dst_frame_)) < 0)
		{
			LOG_ERROR("Error sending a frame for encoding");
			return -1;
		}
		
		while (ret >= 0)
		{
			ret = avcodec_receive_packet(codec_ctx_, &packet_);
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
			{
				break;
			}
			else if (ret < 0)
			{
				LOG_ERROR("Error during encoding");
				return -1;
			}

			av_packet_rescale_ts(&packet_, codec_ctx_->time_base, av_stream_->time_base);
			packet_.stream_index = av_stream_->index;
			av_interleaved_write_frame(fmt_ctx_, &packet_);
			av_packet_unref(&packet_);
		}

		return 0;
	}

	int DataToVideo::rgbToJpeg(unsigned char *rgb_data, int64_t ts)
	{
		int ret = 0;
#if defined(__linux__) && (defined(__arm__) || defined(__aarch64__))
		ret = libyuv::RAWToI420(
			rgb_data,
			src_width_ * 3,								  // BGR24 每行字节数
			dst_frame_->data[0], dst_frame_->linesize[0], // Y plane
			dst_frame_->data[1], dst_frame_->linesize[1], // U plane
			dst_frame_->data[2], dst_frame_->linesize[2], // V plane
			src_width_,
			src_height_);
#else
		uint8_t *src_slice[1] = {rgb_data};
		int src_stride[1] = {src_width_ * 3}; // BGR24每行字节数

		sws_scale(sws_ctx_, src_slice, src_stride, 0, src_height_, dst_frame_->data, dst_frame_->linesize);

#endif
		if (ret != 0)
		{
			LOG_ERROR("libyuv::RAWToI420 failed: %d", ret);
			return -1;
		}

		ret = avcodec_send_frame(jpeg_ctx_, dst_frame_);
		if (ret < 0)
		{
			LOG_ERROR("Failed to send frame to JPEG encoder.");
			return -1;
		}

		ret = avcodec_receive_packet(jpeg_ctx_, &packet_);
		if (ret < 0)
		{
			LOG_ERROR("Failed to receive packet from JPEG encoder.");
			return -1;
		}
		std::string filename = jpeg_output_path_ + "/" + std::to_string(ts) + ".jpg";
		LOG_INFO("JPEG image saved as: %s", filename.c_str());

		FILE *out = fopen(filename.c_str(), "wb");
		if (!out)
		{
			LOG_ERROR("Could not open output JPEG file.");
			return -1;
		}
		fwrite(packet_.data, 1, packet_.size, out);
		fclose(out);
		return 0;
	}

	int DataToVideo::setOutputFile(const std::string &output_filename)
	{
		if (!codec_ctx_)
		{
			LOG_ERROR("setOutputFile: codec_ctx_ not initialized. Please call initVideoConverter() first.");
			return -1;
		}

		if (fmt_ctx_)
		{
			avformat_free_context(fmt_ctx_);
			fmt_ctx_ = nullptr;
		}
		if (av_stream_)
		{
			av_stream_ = nullptr;
		}

		if (avformat_alloc_output_context2(&fmt_ctx_, nullptr, "mp4", output_filename.c_str()) < 0 || !fmt_ctx_)
		{
			LOG_ERROR("Failed to allocate output format context.");
			return -1;
		}

		av_stream_ = avformat_new_stream(fmt_ctx_, nullptr);
		if (!av_stream_)
		{
			LOG_ERROR("Failed to create new stream.");
			return -1;
		}
		if (avcodec_parameters_from_context(av_stream_->codecpar, codec_ctx_) < 0)
		{
			LOG_ERROR("Failed to copy codec parameters to stream.");
			return -1;
		}
		av_stream_->time_base = codec_ctx_->time_base;

		if (avio_open(&fmt_ctx_->pb, output_filename.c_str(), AVIO_FLAG_WRITE) < 0)
		{
			LOG_ERROR("Could not open output file: %s", output_filename.c_str());
			return -1;
		}

		if (avformat_write_header(fmt_ctx_, nullptr) < 0)
		{
			LOG_ERROR("Failed to write video header.");
			return -1;
		}

		LOG_INFO("Output file set to: %s", output_filename.c_str());
		return 0;
	}

	int DataToVideo::setJpegOutputPath(const std::string &output_path)
	{
		jpeg_output_path_ = output_path;
		return 0;
	}

	int DataToVideo::flushAndCloseVideo()
	{
		if (!fmt_ctx_ || !codec_ctx_ || !av_stream_)
		{
			LOG_ERROR("flushAndCloseVideo: Missing format or codec context.");
			return -1;
		}
		
		int ret = avcodec_send_frame(codec_ctx_, nullptr);
		if (ret < 0)
		{
			LOG_ERROR("Failed to send flush frame to encoder.");
			return -1;
		}

		while (true)
		{
			ret = avcodec_receive_packet(codec_ctx_, &packet_);
			if (ret == AVERROR(EAGAIN))
			{
				continue;
			}
			else if (ret == AVERROR_EOF)
			{
				break;
			}
			else if (ret < 0)
			{
				LOG_ERROR("Error receiving packet during flush: %d", ret);
				return -1;
			}

			av_packet_rescale_ts(&packet_, codec_ctx_->time_base, av_stream_->time_base);
			packet_.stream_index = av_stream_->index;

			if (av_interleaved_write_frame(fmt_ctx_, &packet_) < 0)
			{
				LOG_ERROR("Failed to write frame during flush.");
			}
			av_packet_unref(&packet_);
		}

		av_write_trailer(fmt_ctx_);

		if (fmt_ctx_->pb)
		{
			avio_closep(&fmt_ctx_->pb);
		}

		avformat_free_context(fmt_ctx_);
		fmt_ctx_ = nullptr;
		av_stream_ = nullptr;
		avcodec_close(codec_ctx_);
		if (avcodec_open2(codec_ctx_, av_codec_, nullptr) < 0)
		{
			LOG_ERROR("Could not reopen codec after flush.");
			return -1;
		}
		LOG_INFO("Video flushed and closed successfully.");
		return 0;
	}

	const AVCodec *DataToVideo::find_preferred_h264_encoder()
	{
		static const char *preferred_encoders[] = {
			"libx264",
			"h264_mf", 
			nullptr};

		void *iter = NULL;
		const AVCodec *codec;

		for (int i = 0; preferred_encoders[i]; i++)
		{
			iter = NULL;
			while ((codec = av_codec_iterate(&iter)))
			{
				if (av_codec_is_encoder(codec) &&
					codec->id == AV_CODEC_ID_H264 &&
					strcmp(codec->name, preferred_encoders[i]) == 0)
				{
					return codec;
				}
			}
		}

		iter = NULL;
		while ((codec = av_codec_iterate(&iter)))
		{
			if (av_codec_is_encoder(codec) && codec->id == AV_CODEC_ID_H264)
			{
				return codec;
			}
		}

		return NULL;
	}
