#include "DvsRgbFusionCamera/rgb/RgbCamera.hpp"
#include "DvsRgbFusionCamera/rgb/hik/HikCamera.hpp"

template std::unique_ptr<RgbCamera> RgbCamera::create<HikCamera>(float fps);