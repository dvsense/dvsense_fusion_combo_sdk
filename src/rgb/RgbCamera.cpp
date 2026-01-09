#include "DvsRgbFusionCamera/rgb/RgbCamera.hpp"

#ifdef USE_HIK_CAMERA
#include "DvsRgbFusionCamera/rgb/hik/HikCamera.hpp"
template std::unique_ptr<RgbCamera> RgbCamera::create<HikCamera>(float fps);
#endif

#ifdef USE_DAHENG_CAMERA
#include "DvsRgbFusionCamera/rgb/daheng/DahengCamera.hpp"
template std::unique_ptr<RgbCamera> RgbCamera::create<DahengCamera>(float fps);
#endif