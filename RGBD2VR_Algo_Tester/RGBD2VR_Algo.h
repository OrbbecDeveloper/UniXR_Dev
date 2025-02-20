#pragma once
#include <opencv2/opencv.hpp>

#define M_PI 3.1415926


int RGBD2VR9(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr);
cv::Mat findMask(cv::Mat& depth, int width, int height);
int RGBD2VR10(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr);

int RGBD2VR11(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, int isRGBFill, double CameraF,
	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr);
////int RGBD2VR12(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, int isRGBFill, double CameraF,
////	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr);

cv::Mat SimpleFill(const cv::Mat& depth, const cv::Mat& Mask, int width, int height, int fTimes = 1);
