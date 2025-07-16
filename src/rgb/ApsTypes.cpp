#include "ApsTypes.hpp"
#include <string.h>
#include <memory>

RgbFrame::RgbFrame(int width, int height, dvsense::TimeStamp start_ts, dvsense::TimeStamp end_ts)
    : width_(width), height_(height),
        exposure_start_timestamp(start_ts), exposure_end_timestamp(end_ts),
        pixel_type_(ApsPixelType::BGR888),
        data_(new uint8_t[width * height * 3], std::default_delete<uint8_t[]>())
{
}

RgbFrame::RgbFrame(int width, int height, dvsense::TimeStamp start_ts, dvsense::TimeStamp end_ts,
                    uint8_t *external_data, size_t data_size,
                    bool own_data)
    : width_(width), height_(height),
        exposure_start_timestamp(start_ts), exposure_end_timestamp(end_ts),
        pixel_type_(ApsPixelType::BGR888)
{
    if (own_data)
    {
        size_t required_size = width * height * 3;
        if (data_size != required_size)
        {
            // LOG_ERROR("Data size mismatch in RgbFrame");
        }
        data_.reset(new uint8_t[required_size], std::default_delete<uint8_t[]>());
        memcpy(data_.get(), external_data, required_size);
    }
    else
    {
        data_.reset(external_data, [](uint8_t *) {});
    }
}

RgbFrame::RgbFrame(const RgbFrame &other)
    : width_(other.width_), height_(other.height_),
        exposure_start_timestamp(other.exposure_start_timestamp),
        exposure_end_timestamp(other.exposure_end_timestamp),
        pixel_type_(other.pixel_type_),
        data_(other.data_)
{
}

RgbFrame &RgbFrame::operator=(const RgbFrame &other)
{
    if (this == &other)
        return *this;
    width_ = other.width_;
    height_ = other.height_;
    exposure_start_timestamp = other.exposure_start_timestamp;
    exposure_end_timestamp = other.exposure_end_timestamp;
    pixel_type_ = other.pixel_type_;
    data_ = other.data_;
    return *this;
}

RgbFrame RgbFrame::clone() const
{
    RgbFrame new_frame(width_, height_, exposure_start_timestamp, exposure_end_timestamp);
    memcpy(new_frame.data_.get(), data_.get(), getDataSize());
    return new_frame;
}

uint8_t *RgbFrame::data() { return data_.get(); }

const uint8_t *RgbFrame::data() const { return data_.get(); }

size_t RgbFrame::getDataSize() const { return width_ * height_ * 3; }

int RgbFrame::width() const { return width_; }

int RgbFrame::height() const { return height_; }

ApsPixelType RgbFrame::pixelType() const { return pixel_type_; }