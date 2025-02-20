// RGBD2VR_Algo_Tester.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <string>
#include <chrono>
#include "RGBD2VR_Algo.h"
#include "cbf_windows.h"
#include <cuda_runtime.h>

int RGBD2VR3(const cv::Mat& depth, const cv::Mat& color, const cv::Mat& Mask, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
	cv::Mat& ImgL, cv::Mat& ImgR, cv::Mat& ImageOut/*, cv::Mat& DisparityL, cv::Mat& DisparityR, cv::Mat& Disparity_Full, cv::Mat& MvImgL, cv::Mat& MvImgR*/);
int RGBD2VR4(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr);
int RGBD2VR6(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr);
cv::Mat calculateIntensity(const cv::Mat& rgbImage);

#define isMEGA

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>

extern "C" void RGBD2VR11_CUDA(double* Depdata, unsigned char* RGBdata, unsigned char* MaskPtr, int width, int height,
	double max_val, double k, int isRGBFill, double CameraF, unsigned char* ImgLPtr, unsigned char* ImgRPtr, unsigned char* ImgOutPtr);

int main() {
	std::string main_folder = "D:/NanXu/Patent_VR/Data_05292023/Images/Input";
	std::string rgb_folder = main_folder + "/color";
	std::string depth_folder = main_folder + "/depth";
	std::string result_folder = main_folder + "/result";

	std::string rgb_filename = "7.png";
	std::string depth_filename = "7.png";
	std::string result_filename = "7.png";

	std::string rgb_image_path = rgb_folder + "/" + rgb_filename;
	std::string depth_image_path = depth_folder + "/" + depth_filename;
	std::string result_image_path = result_folder + "/" + result_filename;

	// Load the pair of images and measure time consumption
	cv::Mat color = cv::imread(rgb_image_path, cv::IMREAD_COLOR);
	cv::Mat depth = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);

	if (color.empty() || depth.empty()) {
		std::cerr << "Error loading image pair: " << rgb_image_path << ", " << depth_image_path << std::endl;
		return -1;
	}

	int width = color.cols;
	int height = color.rows;
	int PixelNum = width * height;
	double min_val, max_val;
	double k = 0.5;
	int isRGBFill = 3;
	double CameraF = 0;
	cv::minMaxLoc(depth, &min_val, &max_val);
	cv::Mat ImgL(color.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat ImgR(color.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat Mask = findMask(depth, width, height);
	cv::Mat ImageOut(color.rows, color.cols * 2, CV_8UC3, cv::Scalar(0, 0, 0));
	unsigned char* MaskPtr = Mask.data;
	unsigned char* RGBdata = color.data;
	double* Depdata = reinterpret_cast<double*>(depth.data);
	unsigned char* ImgLPtr = ImgL.data;
	unsigned char* ImgRPtr = ImgR.data;
	unsigned char* ImgOutPtr = ImageOut.data;

	// Allocate memory for GPU
	double* dev_Depdata;
	unsigned char* dev_RGBdata;
	unsigned char* dev_MaskPtr;
	unsigned char* dev_ImgLPtr;
	unsigned char* dev_ImgRPtr;
	unsigned char* dev_ImgOutPtr;

	cudaMalloc((void**)&dev_Depdata, width * height * sizeof(double));
	cudaMalloc((void**)&dev_RGBdata, width * height * 3 * sizeof(unsigned char));
	cudaMalloc((void**)&dev_MaskPtr, width * height * sizeof(unsigned char));
	cudaMalloc((void**)&dev_ImgLPtr, width * height * 3 * sizeof(unsigned char));
	cudaMalloc((void**)&dev_ImgRPtr, width * height * 3 * sizeof(unsigned char));
	cudaMalloc((void**)&dev_ImgOutPtr, width * height * 6 * sizeof(unsigned char));

	// Copy data from host to device
	cudaMemcpy(dev_Depdata, Depdata, width * height * sizeof(double), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_RGBdata, RGBdata, width * height * 3 * sizeof(unsigned char), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_MaskPtr, MaskPtr, width * height * sizeof(unsigned char), cudaMemcpyHostToDevice);

	auto start_time = std::chrono::high_resolution_clock::now();


#ifdef isMEGA
	cv::Mat Mask4Fill = cv::Mat::zeros(depth.size(), CV_8U);
	Mask4Fill.setTo(1, depth < 10);
	//uchar* Depdata2fill = (depth.data);
	cv::Mat Depth2Cvt = cv::Mat::zeros(depth.size(), CV_16F);
	cv::Mat Depth2Fill = cv::Mat::zeros(depth.size(), CV_8U);
	cv::divide(depth, max_val/255, Depth2Cvt);
	Depth2Cvt.convertTo(Depth2Fill, CV_8U);
	cv::Mat rgbTmp;
	cv::cvtColor(color, rgbTmp, cv::COLOR_BGR2GRAY);
	

	uchar* D2FPtr = Depth2Fill.data;
	uchar* M4FPtr = Mask4Fill.data;
	uchar* rgbTmpPtr = rgbTmp.data;
	double spaceSigmas[3] = { 12,5,8 };
	double rangeSigmas[3] = { 0.2, 0.08, 0.02 };


	cbf::cbf(D2FPtr, rgbTmpPtr, (bool*)M4FPtr, D2FPtr, spaceSigmas, rangeSigmas, width, height);
	cv::Mat DepthFilled = cv::Mat::zeros(depth.size(), CV_16U);
	ushort* DFDPtr = reinterpret_cast<ushort*>(DepthFilled.data);

	//cv::multiply(Depth2Fill, 255, depth);
	for (int i = 0; i < PixelNum; i++)
	{
		if (MaskPtr[i] == 0 && M4FPtr[i] == 1)
		{
			Depdata[i] = D2FPtr[i] * max_val / 255;
		}
		//else
		//	DFDPtr[i] = Depdata[i];
	}
	auto end_time1 = std::chrono::high_resolution_clock::now();
	auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - start_time).count();
	std::cout << "Depth Filling in " << duration1 << " ms" << std::endl;

#endif

	// Launch the CUDA kernel
	RGBD2VR11_CUDA(dev_Depdata, dev_RGBdata, dev_MaskPtr, width, height, max_val, k, isRGBFill, CameraF,
		dev_ImgLPtr, dev_ImgRPtr, dev_ImgOutPtr);

	// Copy the results back from device to host
	cudaMemcpy(ImgLPtr, dev_ImgLPtr, width * height * 3 * sizeof(unsigned char), cudaMemcpyDeviceToHost);
	cudaMemcpy(ImgRPtr, dev_ImgRPtr, width * height * 3 * sizeof(unsigned char), cudaMemcpyDeviceToHost);
	cudaMemcpy(ImgOutPtr, dev_ImgOutPtr, width * height * 6 * sizeof(unsigned char), cudaMemcpyDeviceToHost);

	auto end_time = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
	std::cout << "Process image pair in " << duration << " ms" << std::endl;

	// Free the allocated memory on the device
	cudaFree(dev_Depdata);
	cudaFree(dev_RGBdata);
	cudaFree(dev_MaskPtr);
	cudaFree(dev_ImgLPtr);
	cudaFree(dev_ImgRPtr);
	cudaFree(dev_ImgOutPtr);

	cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Image", ImageOut);
	cv::imwrite(result_image_path, ImageOut);

	// Wait for the user to press a key
	cv::waitKey(0);

	return 0;
}




int RGBD2VR3(const cv::Mat& depth, const cv::Mat& color, const cv::Mat& Mask, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
	cv::Mat& ImgL, cv::Mat& ImgR, cv::Mat& ImageOut/*, cv::Mat& DisparityL, cv::Mat& DisparityR, cv::Mat& Disparity_Full, cv::Mat& MvImgL, cv::Mat& MvImgR*/) 
{
	// DBack is the max distance;
	// CameraF original 350;
	// k left right ratio; default = 0.5

	//cv::Mat depth, color;
	//cv::resize(depth1, depth, cv::Size(width, height));
	//cv::resize(color1, color, cv::Size(width, height));

	int FWin = 1; // hole filling window size. 0 means no filling
	double cx = 535.0 / 1024 * width; //optical center of the depth camera
	double d = 10.0; // distance between VR screento eye;
	double EyeAngle = 90 * M_PI / 180.0; // Horizontal FoV of Oculus screen
	double CameraAngle = 75 * M_PI / 180.0; // Horizontal FoV of the camera which is used to take the depth
	double Baseline = 60.0; // distance between two eyes
	double WL = d * tan(EyeAngle / 2); // Oculus screen width
	if (k > 1)
		k = k / 10;

	double f;
	if (CameraF == 0)
		f = width / 2 / tan(CameraAngle / 2);
	else
		f = CameraF;

	cv::Mat ImgTagL = cv::Mat::zeros(height, width, CV_8U);
	cv::Mat ImgTagR = cv::Mat::zeros(height, width, CV_8U);
	uchar* ImgTagLPtr = ImgTagL.data;
	uchar* ImgTagRPtr = ImgTagR.data;

	uchar* MaskPtr = Mask.data;
	uchar* RGBdata = color.data;
	ushort* Depdata = reinterpret_cast<ushort*>(depth.data);
	uchar* ImgLPtr = ImgL.data;
	uchar* ImgRPtr = ImgR.data;
	uchar* ImgOutPtr = ImageOut.data;

	int indexR = 0;
	int index = 0;
	ushort DepPixel = 0;
	char r = 0, g = 0, b = 0;
	// left image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		for (int i = width - 1; i >= 0; i--) {
			index = indexR + i;
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 || D > 0) {
				double xI = d / D * (D * (i - cx) / f + Baseline * (1 - k));
				int I = static_cast<int>((1 + xI / WL) * width / 2);
				if (I > 0 && I < width) {
					int indexFill = j * width + I;
					int indexFill3 = indexFill * 3;
					ImgLPtr[indexFill3 + 0] = r;
					ImgLPtr[indexFill3 + 1] = g;
					ImgLPtr[indexFill3 + 2] = b;
					ImgTagLPtr[indexFill] = 1;
					if (isRGBFill) {
						int fill_start = std::max(I - FWin, 0);
						int fill_end = std::min(I + FWin, width - 1);
						for (int fill_i = fill_start; fill_i <= fill_end; ++fill_i) {
							indexFill = j * width + fill_i;
							if (ImgTagLPtr[indexFill] == 0) {
								ImgLPtr[indexFill * 3 + 0] = r;
								ImgLPtr[indexFill * 3 + 1] = g;
								ImgLPtr[indexFill * 3 + 2] = b;
								ImgTagLPtr[indexFill] = 1;
								//ImgTagL.at<uint8_t>(j, fill_i) = 1;
							}
						}
					}
					else {
						int indexFill = j * width + I;
						int indexFill3 = indexFill * 3;
						//ImgLPtr[indexFill3 + 0] = r;
						//ImgLPtr[indexFill3 + 1] = g;
						//ImgLPtr[indexFill3 + 2] = b;
						//ImgTagLPtr[indexFill] = 1;
						//ImgLPtr[indexFill3 + 0 - 3] = r;
						//ImgLPtr[indexFill3 + 1 - 3] = g;
						//ImgLPtr[indexFill3 + 2 - 3] = b;
						//ImgTagLPtr[indexFill - 3] = 1;
						ImgLPtr[indexFill3 + 0 + 3] = r;
						ImgLPtr[indexFill3 + 1 + 3] = g;
						ImgLPtr[indexFill3 + 2 + 3] = b;
						ImgTagLPtr[indexFill + 3] = 1;
					}
				}
			}
		}
	}



	// right image
	for (int j = 0; j < height; ++j) {
		indexR = j * width;
		for (int i = 0; i < width; ++i) {
			index = indexR + i;
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 || D > 0) {
				double xI = d / D * (D * (i - cx) / f - Baseline * k);
				int I = static_cast<int>((1 + xI / WL) * width / 2);
				if (I > 0 && I < width) {
					int indexFill = j * width + I;
					int indexFill3 = indexFill * 3;
					ImgRPtr[indexFill3 + 0] = r;
					ImgRPtr[indexFill3 + 1] = g;
					ImgRPtr[indexFill3 + 2] = b;
					ImgTagRPtr[indexFill] = 1;
					if (isRGBFill) {
						int fill_start = std::max(I - FWin, 1);
						int fill_end = std::min(I + FWin, width);
						for (int fill_i = fill_start; fill_i <= fill_end; ++fill_i) {
							indexFill = j * width + fill_i;
							indexFill3 = indexFill * 3;
							if (ImgTagRPtr[indexFill] == 0) {
								ImgRPtr[indexFill3 + 0] = r;
								ImgRPtr[indexFill3 + 1] = g;
								ImgRPtr[indexFill3 + 2] = b;
								ImgTagRPtr[indexFill] = 1;
							}
						}
					}
					else {
						ImgRPtr[indexFill3 + 0 + 3] = r;
						ImgRPtr[indexFill3 + 1 + 3] = g;
						ImgRPtr[indexFill3 + 2 + 3] = b;
						ImgTagRPtr[indexFill + 3] = 1;
					}
				}

			}
		}
	}
	for (int j = 0; j < height; ++j) {
		indexR = j * width;
		for (int i = 0; i < width; ++i) {
			index = indexR + i;
			ImgOutPtr[index * 3] = ImgLPtr[index * 3];
			ImgOutPtr[index * 3 + 1] = ImgLPtr[index * 3 + 1];
			ImgOutPtr[index * 3 + 2] = ImgLPtr[index * 3 + 2];
			ImgOutPtr[width + index * 3] = ImgRPtr[index * 3];
			ImgOutPtr[width + index * 3 + 1] = ImgRPtr[index * 3 + 1];
			ImgOutPtr[width + index * 3 + 2] = ImgRPtr[index * 3 + 2];
		}
	}

	//ImageOut = ImgL.clone();
	//cv::hconcat(ImgL, ImgR, ImageOut);

	return 1;
}

int RGBD2VR4(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr)
{
	// DBack is the max distance;
	// CameraF original 350;
	// k left right ratio; default = 0.5
	int FWin = 1; // hole filling window size. 0 means no filling
	double cx = 535.0 / 1024 * width; //optical center of the depth camera
	double d = 10.0; // distance between VR screento eye;
	double EyeAngle = 90 * M_PI / 180.0; // Horizontal FoV of Oculus screen
	double CameraAngle = 75 * M_PI / 180.0; // Horizontal FoV of the camera which is used to take the depth
	double Baseline = 60.0; // distance between two eyes
	double WL = d * tan(EyeAngle / 2); // Oculus screen width
	if (k > 1)
		k = k / 10;

	double f;
	if (CameraF == 0)
		f = width / 2 / tan(CameraAngle / 2);
	else
		f = CameraF;

	cv::Mat ImgTagL = cv::Mat::zeros(height, width, CV_8U);
	cv::Mat ImgTagR = cv::Mat::zeros(height, width, CV_8U);
	uchar* ImgTagLPtr = ImgTagL.data;
	uchar* ImgTagRPtr = ImgTagR.data;


	int indexR = 0;
	int index = 0;
	ushort DepPixel = 0;
	char r = 0, g = 0, b = 0;
	// left image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		for (int i = width - 1; i >= 0; i--) {
			index = indexR + i;
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 || D > 0) {
				double xI = d / D * (D * (i - cx) / f + Baseline * (1 - k));
				int I = static_cast<int>((1 + xI / WL) * width / 2);
				if (I > 0 && I < width) {
					int indexFill = j * width + I;
					int indexFill3 = indexFill * 3;
					ImgLPtr[indexFill3 + 0] = r;
					ImgLPtr[indexFill3 + 1] = g;
					ImgLPtr[indexFill3 + 2] = b;
					ImgTagLPtr[indexFill] = 1;
					if (isRGBFill) {
						int fill_start = std::max(I - FWin, 0);
						int fill_end = std::min(I + FWin, width - 1);
						for (int fill_i = fill_start; fill_i <= fill_end; ++fill_i) {
							indexFill = j * width + fill_i;
							if (ImgTagLPtr[indexFill] == 0) {
								ImgLPtr[indexFill3 + 0] = r;
								ImgLPtr[indexFill3 + 1] = g;
								ImgLPtr[indexFill3 + 2] = b;
								ImgTagLPtr[indexFill] = 1;
								//ImgTagL.at<uint8_t>(j, fill_i) = 1;
							}
						}
					}
					else {
						ImgLPtr[indexFill3 + 0 + 3] = r;
						ImgLPtr[indexFill3 + 1 + 3] = g;
						ImgLPtr[indexFill3 + 2 + 3] = b;
						ImgTagLPtr[indexFill + 3] = 1;
					}
				}
			}
		}
	}



	// right image
	for (int j = 0; j < height; ++j) {
		indexR = j * width;
		for (int i = 0; i < width; ++i) {
			index = indexR + i;
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 || D > 0) {
				double xI = d / D * (D * (i - cx) / f - Baseline * k);
				int I = static_cast<int>((1 + xI / WL) * width / 2);
				if (I > 0 && I < width) {
					int indexFill = j * width + I;
					int indexFill3 = indexFill * 3;
					ImgRPtr[indexFill3 + 0] = r;
					ImgRPtr[indexFill3 + 1] = g;
					ImgRPtr[indexFill3 + 2] = b;
					ImgTagRPtr[indexFill] = 1;
					if (isRGBFill) {
						int fill_start = std::max(I - FWin, 1);
						int fill_end = std::min(I + FWin, width);
						for (int fill_i = fill_start; fill_i <= fill_end; ++fill_i) {
							indexFill = j * width + fill_i;
							indexFill3 = indexFill * 3;
							if (ImgTagRPtr[indexFill] == 0) {
								ImgRPtr[indexFill3 + 0] = r;
								ImgRPtr[indexFill3 + 1] = g;
								ImgRPtr[indexFill3 + 2] = b;
								ImgTagRPtr[indexFill] = 1;
							}
						}
					}
					else {
						ImgRPtr[indexFill3 + 0 + 3] = r;
						ImgRPtr[indexFill3 + 1 + 3] = g;
						ImgRPtr[indexFill3 + 2 + 3] = b;
						ImgTagRPtr[indexFill + 3] = 1;
					}
				}

			}
		}
	}
	//for (int j = 0; j < height; ++j) {
	//	indexR = j * width;
	//	for (int i = 0; i < width; ++i) {
	//		index = indexR + i;
	//		ImgOutPtr[index * 3] = ImgLPtr[index * 3];
	//		ImgOutPtr[index * 3 + 1] = ImgLPtr[index * 3 + 1];
	//		ImgOutPtr[index * 3 + 2] = ImgLPtr[index * 3 + 2];
	//		ImgOutPtr[width * 3 + index * 3] = ImgRPtr[index * 3];
	//		ImgOutPtr[width * 3 + index * 3 + 1] = ImgRPtr[index * 3 + 1];
	//		ImgOutPtr[width * 3 + index * 3 + 2] = ImgRPtr[index * 3 + 2];
	//	}
	//}
	for (int j = 0; j < height; ++j) {
		int row_offset = j * width * 3; // Precompute row offset for efficiency
		for (int i = 0; i < width; ++i) {
			int indexL = row_offset + i * 3;
			int indexR = row_offset * 2 + (width + i) * 3;
			int indexO = row_offset * 2 + i * 3;
			ImgOutPtr[indexO] = ImgLPtr[indexL];
			ImgOutPtr[indexO + 1] = ImgLPtr[indexL + 1];
			ImgOutPtr[indexO + 2] = ImgLPtr[indexL + 2];

			ImgOutPtr[indexR] = ImgRPtr[indexL];
			ImgOutPtr[indexR + 1] = ImgRPtr[indexL + 1];
			ImgOutPtr[indexR + 2] = ImgRPtr[indexL + 2];
		}
	}
	//ImageOut = ImgL.clone();
	//cv::hconcat(ImgL, ImgR, ImageOut);

	return 1;
}

int RGBD2VR6(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr)
{
	// DBack is the max distance;
	// CameraF original 350;
	// k left right ratio; default = 0.5
	int FWin = 1; // hole filling window size. 0 means no filling
	double cx = 535.0 / 1024 * width; //optical center of the depth camera
	double d = 10.0; // distance between VR screento eye;
	double EyeAngle = 90 * M_PI / 180.0; // Horizontal FoV of Oculus screen
	double CameraAngle = 75 * M_PI / 180.0; // Horizontal FoV of the camera which is used to take the depth
	double Baseline = 60.0; // distance between two eyes
	double WL = d * tan(EyeAngle / 2); // Oculus screen width
	if (k > 1)
		k = k / 10;

	double f;
	if (CameraF == 0)
		f = width / 2 / tan(CameraAngle / 2);
	else
		f = CameraF;

	cv::Mat ImgTagL = cv::Mat::zeros(height, width, CV_8U);
	cv::Mat ImgTagR = cv::Mat::zeros(height, width, CV_8U);
	uchar* ImgTagLPtr = ImgTagL.data;
	uchar* ImgTagRPtr = ImgTagR.data;


	int indexR = 0;
	int index = 0;
	ushort DepPixel = 0;
	char r = 0, g = 0, b = 0;
	// left image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		for (int i = width - 1; i >= 0; i--) {
			index = indexR + i;
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 || D > 0) {
				double xI_l = d / D * (D * (i - cx) / f + Baseline * (1 - k));
				int I_l = static_cast<int>((1 + xI_l / WL) * width / 2);
				if (I_l > 0 && I_l < width) {
					int indexFill = j * width + I_l;
					int indexFill3 = indexFill * 3;

					int indexO = indexR * 3 * 2 + I_l * 3;
					ImgOutPtr[indexO] = r;
					ImgOutPtr[indexO + 1] = g;
					ImgOutPtr[indexO + 2] = b;



					//ImgLPtr[indexFill3 + 0] = r;
					//ImgLPtr[indexFill3 + 1] = g;
					//ImgLPtr[indexFill3 + 2] = b;
					ImgTagLPtr[indexFill] = 1;
					if (isRGBFill) {
						int fill_start = std::max(I_l - FWin, 0);
						int fill_end = std::min(I_l + FWin, width - 1);
						for (int fill_i = fill_start; fill_i <= fill_end; ++fill_i) {
							indexFill = j * width + fill_i;
							indexFill3 = indexFill * 3;
							int indexFill3_O = indexFill3 + j * width * 3;
							if (ImgTagLPtr[indexFill] == 0) {
								ImgOutPtr[indexFill3_O] = r;
								ImgOutPtr[indexFill3_O + 1] = g;
								ImgOutPtr[indexFill3_O + 2] = b;
								//ImgLPtr[indexFill3 + 0] = r;
								//ImgLPtr[indexFill3 + 1] = g;
								//ImgLPtr[indexFill3 + 2] = b;
								ImgTagLPtr[indexFill] = 1;
								//ImgTagL.at<uint8_t>(j, fill_i) = 1;
							}
						}
					}
					else {
						ImgOutPtr[indexO + 3] = r;
						ImgOutPtr[indexO + 1 + 3] = g;
						ImgOutPtr[indexO + 2 + 3] = b;
						//ImgLPtr[indexFill3 + 0 + 3] = r;
						//ImgLPtr[indexFill3 + 1 + 3] = g;
						//ImgLPtr[indexFill3 + 2 + 3] = b;
						ImgTagLPtr[indexFill + 3] = 1;
					}
				}
				double xI_r = d / D * (D * (i - cx) / f - Baseline * k);
				int I_r = static_cast<int>((1 + xI_r / WL) * width / 2);
				if (I_r > 0 && I_r < width) {
					int indexFill = j * width + I_r;
					int indexFill3 = indexFill * 3;

					int indexO = (indexR * 2 + I_r + width) * 3;
					ImgOutPtr[indexO] = r;
					ImgOutPtr[indexO + 1] = g;
					ImgOutPtr[indexO + 2] = b;


					//ImgRPtr[indexFill3 + 0] = r;
					//ImgRPtr[indexFill3 + 1] = g;
					//ImgRPtr[indexFill3 + 2] = b;
					ImgTagRPtr[indexFill] = 1;
					if (isRGBFill) {
						int fill_start = std::max(I_r - FWin, 1);
						int fill_end = std::min(I_r + FWin, width);
						for (int fill_i = fill_start; fill_i <= fill_end; ++fill_i) {
							indexFill = j * width + fill_i;
							indexFill3 = indexFill * 3;
							int indexFill3_O = indexFill3 + j * width * 3 + width * 3;
							if (ImgTagRPtr[indexFill] == 0) {
								ImgOutPtr[indexFill3_O] = r;
								ImgOutPtr[indexFill3_O + 1] = g;
								ImgOutPtr[indexFill3_O + 2] = b;
								//ImgRPtr[indexFill3 + 0] = r;
								//ImgRPtr[indexFill3 + 1] = g;
								//ImgRPtr[indexFill3 + 2] = b;
								ImgTagRPtr[indexFill] = 1;
							}
						}
					}
					else {
						//ImgRPtr[indexFill3 + 0 + 3] = r;
						//ImgRPtr[indexFill3 + 1 + 3] = g;
						//ImgRPtr[indexFill3 + 2 + 3] = b;
						ImgOutPtr[indexO] = r;
						ImgOutPtr[indexO + 1 + 3] = g;
						ImgOutPtr[indexO + 2 + 3] = b;
						ImgTagRPtr[indexFill + 3] = 1;
					}
				}

			}
		}
	}
	return 1;
}

cv::Mat calculateIntensity(const cv::Mat& rgbImage)
{
	// Create a grayscale image with the same dimensions as the input image
	cv::Mat intensityImage(rgbImage.size(), CV_8UC1);

	// Iterate over each pixel
	for (int row = 0; row < rgbImage.rows; ++row)
	{
		for (int col = 0; col < rgbImage.cols; ++col)
		{
			// Get the RGB pixel values
			cv::Vec3b rgbPixel = rgbImage.at<cv::Vec3b>(row, col);

			// Calculate the average intensity
			int intensity = (rgbPixel[0] + rgbPixel[1] + rgbPixel[2]) / 3;

			// Set the intensity value in the intensity image
			intensityImage.at<uchar>(row, col) = intensity;
		}
	}

	return intensityImage;
}
// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
