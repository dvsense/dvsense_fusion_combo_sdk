#include "DvsRgbFusionCamera/rgb/RgbCamera.hpp"
#include "DvsRgbFusionCamera/rgb/hik/HikCamera.hpp"
#include "DvsRgbFusionCamera/rgb/daheng/DahengCamera.hpp"

template std::unique_ptr<RgbCamera> RgbCamera::create<HikCamera>(float fps);
template std::unique_ptr<RgbCamera> RgbCamera::create<DahengCamera>(float fps);