#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include <functional>
#include <memory>
#include <vector>
#include "DvsenseBase/Utils/TypeUtils.hpp"
#include "DvsenseBase/EventBase/EventTypes.hpp"

#ifdef _WIN32
#define DVSENSE_API __declspec(dllexport)
#else
#define DVSENSE_API
#endif // _WIN32

enum class ApsPixelType
{
    BGR888,
};

class DVSENSE_API RgbFrame
{
public:
    RgbFrame(int width = 3840, int height = 2160, dvsense::TimeStamp start_ts = 0, dvsense::TimeStamp end_ts = 0);

    RgbFrame(int width, int height, dvsense::TimeStamp start_ts, dvsense::TimeStamp end_ts,
        uint8_t* external_data, size_t data_size,
        bool own_data = false);

    RgbFrame(const RgbFrame& other);

    RgbFrame& operator=(const RgbFrame& other);

    RgbFrame clone() const;

    uint8_t* data();

    const uint8_t* data() const;

    size_t getDataSize() const;

    int width() const;

    int height() const;

    ApsPixelType pixelType() const;

    dvsense::TimeStamp exposure_start_timestamp;
    dvsense::TimeStamp exposure_end_timestamp;

private:
    int width_;
    int height_;
    ApsPixelType pixel_type_;
    std::shared_ptr<uint8_t[]> data_;
};

using FrameCallback = std::function<void(RgbFrame&)>;
using DsFusionDataCallback = std::function<void(RgbFrame&, dvsense::EventIterator_t begin, dvsense::EventIterator_t end)>;