    #ifndef __DataToVideo_HPP__
#define __DataToVideo_HPP__

extern "C" {
#include <libavformat/avformat.h>     
#include <libavcodec/avcodec.h>      
#include <libavutil/imgutils.h>       
#include <libswscale/swscale.h>      
#include <libavutil/error.h>
}
#include <string>

#ifdef _WIN32
#define DVSENSE_API __declspec(dllexport)
#else
#define DVSENSE_API
#endif // _WIN32

    class DVSENSE_API DataToVideo 
    {
    public:
        DataToVideo();

        ~DataToVideo();

        void setConverterFrameSize(int src_height, int src_width, int dst_height, int dst_width);

        void setConverterFmt(AVPixelFormat src_fmt, AVPixelFormat dst_fmt);
        

        int nv12ToRGB(const unsigned char *nv12_data, unsigned char *rgb_data);

        int initVideoConverter();

        int setOutputFile(const std::string &output_filename);

        int setJpegOutputPath(const std::string &output_path);

        int nv12ToVideo(unsigned char *nv12_data, int64_t ts = -1);

        int rgbToVideo(const unsigned char *rgb_data, int64_t ts = -1);

        int rgbToJpeg(unsigned char *rgb_data, int64_t ts);

        int flushAndCloseVideo();

    private:
        const AVCodec *find_preferred_h264_encoder();
        std::string output_filename_;

        AVFormatContext *fmt_ctx_ = nullptr;
        AVCodecContext *codec_ctx_ = nullptr;
        AVStream *av_stream_ = nullptr;
        const AVCodec *av_codec_ = nullptr;
        SwsContext *sws_ctx_ = nullptr;
        
        AVPixelFormat src_fmt_;
        AVPixelFormat dst_fmt_;

        int dst_width_ = 3840;
        int dst_height_ = 2160;
        int src_width_ = 3840;
        int src_height_ = 2160;
        int framerate_ = 30;
        AVRational timebase_ = {1, 1000000};
        int pts_ = 0;
        AVFrame *dst_frame_ = nullptr;
        AVFrame *src_frame_ = nullptr;
        AVPacket packet_;

        const AVCodec *jpeg_codec_ = nullptr;
        AVCodecContext *jpeg_ctx_ = nullptr;
        std::string jpeg_output_path_;
    };

#endif //!__DataToVideo_HPP__
