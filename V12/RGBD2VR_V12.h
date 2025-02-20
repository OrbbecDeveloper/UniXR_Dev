#pragma once
#ifdef RGBD2VR_EXPORTS
#define RGBD2VR_API __declspec(dllexport)
#else
#define RGBD2VR_API __declspec(dllimport)
#endif
#include <opencv2/opencv.hpp>

#define M_PI 3.1415926

extern "C" RGBD2VR_API int RGBD2VR12(ushort * Depdata, uchar * RGBdata, uchar * MaskPtr, int width, int height, double DBack, double k, int isRGBFill, double CameraF,
	uchar * ImgLPtr, uchar * ImgRPtr, uchar * ImgOutPtr);

