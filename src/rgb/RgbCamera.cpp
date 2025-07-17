#include "RgbCamera.hpp"
#include "hik/HikCamera.hpp"

std::unique_ptr<RgbCamera> RgbCamera::create(float fps) 
{
	std::cout << "create fps is " << fps << std::endl;
	return std::make_unique<HikCamera>(fps);
}