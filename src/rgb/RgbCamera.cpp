#include "DvsRgbFusionCamera/rgb/RgbCamera.hpp"
#include "DvsRgbFusionCamera/rgb/hik/HikCamera.hpp"

//std::unique_ptr<RgbCamera> RgbCamera::create(float fps) 
//{
//	return std::make_unique<HikCamera>(fps);
//}

template std::unique_ptr<RgbCamera> RgbCamera::create<HikCamera>(float fps);