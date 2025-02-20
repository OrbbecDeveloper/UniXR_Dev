
#include "RGBD2VR_Algo.h"
#include <math.h>
#include <opencv2/opencv.hpp>


#define CheckLnR
//#define FillLnR

int RGBD2VR9(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
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

	//target image pixel taken tag
	cv::Mat ImgTagL = cv::Mat::zeros(height, width, CV_8U);
	cv::Mat ImgTagR = cv::Mat::zeros(height, width, CV_8U);
	uchar* ImgTagLPtr = ImgTagL.data;
	uchar* ImgTagRPtr = ImgTagR.data;
	//save the move target I
	cv::Mat MvImgL = cv::Mat::zeros(height, width, CV_16U);
	cv::Mat MvImgR = cv::Mat::zeros(height, width, CV_16U);
	uchar* MvImgLPtr = MvImgL.data;
	uchar* MvImgRPtr = MvImgR.data;


	int indexR = 0;
	int index = 0;
	ushort DepPixel = 0;
	char r = 0, g = 0, b = 0;
	// left image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		for (int i = width - 1; i >= 0; i--) {
			index = indexR + i;					//current working pixel index on original images
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

					MvImgLPtr[index] = I_l;

					//ImgLPtr[indexFill3 + 0] = r;
					//ImgLPtr[indexFill3 + 1] = g;
					//ImgLPtr[indexFill3 + 2] = b;
					ImgTagLPtr[indexFill] = 1;
					if (isRGBFill == 1) {
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

					if (isRGBFill == 3) {
						int FillWin = 2;
						int indexL = i - 1;
						int indexPre = 0;
						int Fill_Min_index = 0;
						int Fill_Max_index = i;
						int Fill_index = 0;
						do
						{
							indexL--;
						} while (indexL > 0 && MvImgLPtr[j * width + indexL] == 0);
						if (indexL == 0)
							indexPre = index;
						else
							indexPre = indexL;
						int Tar = MvImgLPtr[j * width + indexPre];
						if (I_l > i)
							if (I_l - Tar > 2)
							{
								Fill_Min_index = (Tar - 7 > 1) ? Tar - 7 : 1;
								Fill_Max_index = (I_l > width) ? width : I_l;
								Fill_index = Tar - 7;
							}
							else
							{
								Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
								Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
								Fill_index = I_l;
							}
						else
						{
							if (I_l - Tar > 2)
							{
								Fill_Min_index = (Tar > 1) ? Tar : 1;
								Fill_Max_index = (I_l > width) ? width : I_l;
								Fill_index = Tar - 7;
							}
							else
							{
								Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
								Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
								Fill_index = I_l;
							}

						}
						int ForFillIndexL = (j * width + Fill_index) * 3;
						r = RGBdata[ForFillIndexL + 0];
						g = RGBdata[ForFillIndexL + 1];
						b = RGBdata[ForFillIndexL + 2];

						for (int ToFillIndex = Fill_Min_index; ToFillIndex < Fill_Max_index; ToFillIndex++)
						{
							int ToFillIndexL = (j * width + ToFillIndex) * 3;
							ImgLPtr[ToFillIndexL + 0] = r;
							ImgLPtr[ToFillIndexL + 1] = g;
							ImgLPtr[ToFillIndexL + 2] = b;
							ImgTagLPtr[ToFillIndexL] = 1;
						}
						//ImgOutPtr[indexO + 3] = r;
						//ImgOutPtr[indexO + 1 + 3] = g;
						//ImgOutPtr[indexO + 2 + 3] = b;
						//ImgLPtr[indexFill3 + 0 + 3] = r;
						//ImgLPtr[indexFill3 + 1 + 3] = g;
						//ImgLPtr[indexFill3 + 2 + 3] = b;
						//ImgTagLPtr[indexFill + 3] = 1;
					}
				}
			}
		}
	}
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		for (int i = width - 1; i >= 0; i--) {
			index = indexR + i;					//current working pixel index on original images
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 || D > 0) {
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
								//ImgOutPtr[indexFill3_O] = r;
								//ImgOutPtr[indexFill3_O + 1] = g;
								//ImgOutPtr[indexFill3_O + 2] = b;
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
						//ImgOutPtr[indexO] = r;
						//ImgOutPtr[indexO + 1 + 3] = g;
						//ImgOutPtr[indexO + 2 + 3] = b;
						//ImgTagRPtr[indexFill + 3] = 1;
					}
				}

			}
		}
	}
	return 1;
}

int RGBD2VR10(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
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

	//target image pixel taken tag
	cv::Mat ImgTagL = cv::Mat::zeros(height, width, CV_8U);
	cv::Mat ImgTagR = cv::Mat::zeros(height, width, CV_8U);
	uchar* ImgTagLPtr = ImgTagL.data;
	uchar* ImgTagRPtr = ImgTagR.data;
	//save the move target I
	cv::Mat MvImgL = cv::Mat::zeros(height, width, CV_16U);
	cv::Mat MvImgR = cv::Mat::zeros(height, width, CV_16U);
	ushort* MvImgLPtr = reinterpret_cast<ushort*>(MvImgL.data);
	ushort* MvImgRPtr = reinterpret_cast<ushort*>(MvImgR.data);


	int indexR = 0;
	int index = 0;
	ushort DepPixel = 0;
	char r = 0, g = 0, b = 0;
	// left image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		//for (int i = width - 1; i >= 0; i--) {
		for (int i =0; i < width; i++) {
			if (i == 500 && j == 500)
			{
				i = i;
			}
			index = indexR + i;					//current working pixel index on original images
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

					MvImgLPtr[index] = I_l;   //跟原始深度彩色图同步，存放当前pixel被移动到了哪里

					//ImgLPtr[indexFill3 + 0] = r;
					//ImgLPtr[indexFill3 + 1] = g;
					//ImgLPtr[indexFill3 + 2] = b;
					ImgTagLPtr[indexFill] = 1;	//跟结果彩色图同步，1表示这个pixel已经有pixel入住了。
					//if (isRGBFill == 1) {
					//	int fill_start = std::max(I_l - FWin, 0);
					//	int fill_end = std::min(I_l + FWin, width - 1);
					//	for (int fill_i = fill_start; fill_i <= fill_end; ++fill_i) {
					//		indexFill = j * width + fill_i;
					//		indexFill3 = indexFill * 3;
					//		int indexFill3_O = indexFill3 + j * width * 3;
					//		if (ImgTagLPtr[indexFill] == 0) {
					//			ImgOutPtr[indexFill3_O] = r;
					//			ImgOutPtr[indexFill3_O + 1] = g;
					//			ImgOutPtr[indexFill3_O + 2] = b;
					//			ImgLPtr[indexFill3 + 0] = r;
					//			ImgLPtr[indexFill3 + 1] = g;
					//			ImgLPtr[indexFill3 + 2] = b;
					//			ImgTagLPtr[indexFill] = 1;
					//			//ImgTagL.at<uint8_t>(j, fill_i) = 1;
					//		}
					//	}
					//}

					if (isRGBFill == 1) {
						int FillWin = 2;
						int indexL = i - 1;
						int indexPre = 0;
						int Fill_Min_index = 0;
						int Fill_Max_index = i;
						int Fill_index = I_l;
						do
						{
							indexL--;
						} while (indexL > 0 && MvImgLPtr[j * width + indexL] == 0);
						if (indexL == 0)
							indexPre = i;
						else
							indexPre = indexL;
						int Tar = MvImgLPtr[j * width + indexPre];
						if (I_l > i)
							if (I_l - Tar > 2)
							{
								Fill_Min_index = (Tar - 7 > 1) ? Tar - 7 : 1;
								Fill_Max_index = (I_l > width) ? width : I_l;
								Fill_index = Tar - 7;
							}
							else
							{
								Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
								Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
								Fill_index = I_l;
							}
						else
						{
							if (I_l - Tar > 2)
							{
								Fill_Min_index = (Tar > 1) ? Tar : 1;
								Fill_Max_index = (I_l > width) ? width : I_l;
								Fill_index = Tar - 7;
							}
							else
							{
								Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
								Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
								Fill_index = I_l;
							}

						}
						int ForFillIndexL = (j * width *2 + Fill_index) * 3;
						r = ImgOutPtr[ForFillIndexL + 0];
						g = ImgOutPtr[ForFillIndexL + 1];
						b = ImgOutPtr[ForFillIndexL + 2];

						for (int ToFillIndex = Fill_Min_index; ToFillIndex < Fill_Max_index; ToFillIndex++)
						{
							int ToFillIndexL = (j * width + ToFillIndex) * 3;
							int ToFillIndexO = (j * width * 2 + ToFillIndex) * 3;
							ImgLPtr[ToFillIndexL + 0] = r;
							ImgLPtr[ToFillIndexL + 1] = g;
							ImgLPtr[ToFillIndexL + 2] = b;
							ImgOutPtr[ToFillIndexO] = r;
							ImgOutPtr[ToFillIndexO + 1] = g;
							ImgOutPtr[ToFillIndexO + 2] = b;
							ImgTagLPtr[j * width + ToFillIndex] = 1;
						}
						//ImgOutPtr[indexO + 3] = r;
						//ImgOutPtr[indexO + 1 + 3] = g;
						//ImgOutPtr[indexO + 2 + 3] = b;
						//ImgLPtr[indexFill3 + 0 + 3] = r;
						//ImgLPtr[indexFill3 + 1 + 3] = g;
						//ImgLPtr[indexFill3 + 2 + 3] = b;
						//ImgTagLPtr[indexFill + 3] = 1;
					}
				}
			}
		}
	}
	//right image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		for (int i = width - 1; i >= 0; i--) {
		//for (int i = 0; i < width; i++) {
			if (j == 256 && i == 1005)
			{
				i = i;
			}
			index = indexR + i;					//current working pixel index on original images
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 && D > 0) {
				double xI_r = d / D * (D * (i - cx) / f - Baseline * k);
				int I_r = static_cast<int>((1 + xI_r / WL) * width / 2);
				if (I_r > 0 && I_r < width) {
					int indexFill = indexR + I_r;
					int indexFill3 = indexFill * 3;

					int indexO = (indexR * 2 + I_r + width) * 3;
					ImgOutPtr[indexO + 0] = r;
					ImgOutPtr[indexO + 1] = g;
					ImgOutPtr[indexO + 2] = b;
					ImgRPtr[indexFill3 + 0] = r;
					ImgRPtr[indexFill3 + 1] = g;
					ImgRPtr[indexFill3 + 2] = b;

					MvImgRPtr[index] = I_r;

					ImgTagRPtr[indexFill] = 1;
					if (isRGBFill == 1) {
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
								ImgRPtr[indexFill3 + 0] = r;
								ImgRPtr[indexFill3 + 1] = g;
								ImgRPtr[indexFill3 + 2] = b;
								ImgTagRPtr[indexFill] = 1;
							}
						}
					}
					//if (isRGBFill == 3) {
					//	int FillWin = 1;
					//	int indexS = i + 1;
					//	int indexPre = 0;
					//	int Fill_Min_index = 0;
					//	int Fill_Max_index = i;
					//	int Fill_index = 0;
					//	do
					//	{
					//		indexS++;
					//	} while (indexS < width && MvImgRPtr[j * width + indexS] == 0);
					//	if (indexS == width)
					//		indexPre = i;
					//	else
					//		indexPre = indexS;
					//	int Tar = MvImgLPtr[j * width + indexPre];
					//	if (I_r > i)
					//		if (Tar - I_r > 2)
					//		{
					//			Fill_Min_index = (I_r  > 1) ? I_r : 1;
					//			Fill_Max_index = (Tar > width) ? width : Tar;
					//			Fill_index = Tar;
					//		}
					//		else
					//		{
					//			Fill_Min_index = (I_r - FillWin > 1) ? I_r - FillWin : 1;
					//			Fill_Max_index = (I_r + FillWin > width) ? width : I_r + FillWin;
					//			Fill_index = I_r;
					//		}
					//	else
					//	{
					//		if (Tar - I_r > 2)
					//		{
					//			Fill_Min_index = (I_r > 1) ? I_r : 1;
					//			Fill_Max_index = (Tar > width) ? width : Tar;
					//			Fill_index = Tar;
					//		}
					//		else
					//		{
					//			Fill_Min_index = (I_r - FillWin > 1) ? I_r - FillWin : 1;
					//			Fill_Max_index = (I_r + FillWin > width) ? width : I_r + FillWin;
					//			Fill_index = I_r;
					//		}

					//	}
					//	int ForFillIndexO = (j * width * 2 + width + Fill_index) * 3;
					//	r = ImgOutPtr[ForFillIndexO + 0];
					//	g = ImgOutPtr[ForFillIndexO + 1];
					//	b = ImgOutPtr[ForFillIndexO + 2];

					//	for (int ToFillIndex = Fill_Min_index; ToFillIndex < Fill_Max_index; ToFillIndex++)
					//	{
					//		int ToFillIndexR = (j * width + ToFillIndex) * 3;
					//		int ToFillIndexO = (j * width * 2 + width + ToFillIndex) * 3;

					//		ImgRPtr[ToFillIndexR + 0] = r;
					//		ImgRPtr[ToFillIndexR + 1] = g;
					//		ImgRPtr[ToFillIndexR + 2] = b;
					//		ImgOutPtr[ToFillIndexO + 0] = r;
					//		ImgOutPtr[ToFillIndexO + 1] = g;
					//		ImgOutPtr[ToFillIndexO + 2] = b;

					//		ImgTagRPtr[j * width + ToFillIndex] = 1;
					//	}
					//	//ImgOutPtr[indexO + 3] = r;
					//	//ImgOutPtr[indexO + 1 + 3] = g;
					//	//ImgOutPtr[indexO + 2 + 3] = b;
					//	//ImgLPtr[indexFill3 + 0 + 3] = r;
					//	//ImgLPtr[indexFill3 + 1 + 3] = g;
					//	//ImgLPtr[indexFill3 + 2 + 3] = b;
					//	//ImgTagLPtr[indexFill + 3] = 1;
					//}

				}

			}
		}
	}
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		for (int i = width - 1; i > 600; i--) {
			//for (int i = 0; i < width; i++) {
			index = indexR + i;					//current working pixel index on original images
			int indexOR = j * width * 2 + width + i;
			if (/*RGBdata[index * 3 + 0] != 0 && RGBdata[index * 3 + 1] != 0 && RGBdata[index * 3 + 2] != 0 &&*/ ImgRPtr[index * 3] == 0 && ImgRPtr[index * 3 + 1] == 0 && ImgRPtr[index * 3 + 2] == 0)
			{
					ImgRPtr[index * 3] = ImgRPtr[index * 3 + 3];
					ImgRPtr[index * 3 + 1] = ImgRPtr[index * 3 + 3 + 1];
					ImgRPtr[index * 3 + 2] = ImgRPtr[index * 3 + 3 + 2];
					ImgOutPtr[indexOR * 3] = ImgRPtr[index * 3 + 3];
					ImgOutPtr[indexOR * 3 + 1] = ImgRPtr[index * 3 + 3 + 1];
					ImgOutPtr[indexOR * 3 + 2] = ImgRPtr[index * 3 + 3 + 2];
			}
		}
	}
	return 1;
}

int RGBD2VR11(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, int isRGBFill, double CameraF,
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

	//target image pixel taken tag
	cv::Mat ImgTagL = cv::Mat::zeros(height, width, CV_16U);
	cv::Mat ImgTagR = cv::Mat::zeros(height, width, CV_16U);
	ushort* ImgTagLPtr = reinterpret_cast<ushort*>(ImgTagL.data);
	ushort* ImgTagRPtr = reinterpret_cast<ushort*>(ImgTagR.data);
	//save the move target I
	cv::Mat MvImgL = cv::Mat::zeros(height, width, CV_16U);
	cv::Mat MvImgR = cv::Mat::zeros(height, width, CV_16U);
	ushort* MvImgLPtr = reinterpret_cast<ushort*>(MvImgL.data);
	ushort* MvImgRPtr = reinterpret_cast<ushort*>(MvImgR.data);


	int indexR = 0;
	int index = 0;
	ushort DepPixel = 0;
	uchar r = 0, g = 0, b = 0;
	// left image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		//for (int i = width - 1; i >= 0; i--) {
		for (int i = 0; i < width; i++) {
			if (i == 654 && j == 378)
			{
				i = i;
			}
			index = indexR + i;					//current working pixel index on original images
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 && D > 0) {
				double xI_l = d / D * (D * (i - cx) / f + Baseline * (1 - k));
				int I_l = static_cast<int>((1 + xI_l / WL) * width / 2);
				if (I_l > 0 && I_l < width) {
					int indexFill = j * width + I_l;
					int indexFill3 = indexFill * 3;

					int indexO = indexR * 3 * 2 + I_l * 3;
					ImgOutPtr[indexO] = r;
					ImgOutPtr[indexO + 1] = g;
					ImgOutPtr[indexO + 2] = b;

					MvImgLPtr[index] = I_l;   //跟原始深度彩色图同步，存放当前pixel被移动到了哪里

#ifdef CheckLnR
					ImgLPtr[indexFill3 + 0] = r;
					ImgLPtr[indexFill3 + 1] = g;
					ImgLPtr[indexFill3 + 2] = b;
#endif // CheckLnR

					ImgTagLPtr[indexFill] = i;	//跟结果彩色图同步，i表示这个pixel已经有pixel入住，入住pixel的原始坐标是i。

					if (isRGBFill == 3) {
						int FillWin = 2;
						int indexL = i;
						int indexPre = 0;
						int Fill_Min_index = 0;
						int Fill_Max_index = i;
						int Fill_index = I_l;
						do
						{
							indexL--;
						} while (indexL > 0 && MvImgLPtr[j * width + indexL] == 0);
						if (indexL == 0)
							indexPre = i;
						else
							indexPre = indexL;
						int Tar = MvImgLPtr[j * width + indexPre];
						if (I_l > i)
							if (I_l - Tar > 2)
							{
								Fill_Min_index = (Tar - 3 > 1) ? Tar - 3 : 1;
								Fill_Max_index = (I_l > width) ? width : I_l;
								Fill_index = Tar - 7;
							}
							else
							{
								//Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
								//Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
								Fill_Min_index = Tar + 1;
								Fill_Max_index = I_l;
								Fill_index = I_l;
							}
						else
						{
							if (I_l - Tar > 2)
							{
								Fill_Min_index = (Tar > 1) ? Tar : 1;
								Fill_Max_index = (I_l > width) ? width : I_l;
								Fill_index = Tar - 3;
							}
							else
							{
								//Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
								//Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
								Fill_Min_index = Tar + 1;
								Fill_Max_index = I_l;
								Fill_index = I_l;
							}

						}
						int ForFillIndexL = (j * width * 2 + Fill_index) * 3;
						r = ImgOutPtr[ForFillIndexL + 0];
						g = ImgOutPtr[ForFillIndexL + 1];
						b = ImgOutPtr[ForFillIndexL + 2];

						for (int ToFillIndex = Fill_Min_index; ToFillIndex < Fill_Max_index; ToFillIndex++)
						{
							int ToFillIndexL = (j * width + ToFillIndex) * 3;
							int ToFillIndexO = (j * width * 2 + ToFillIndex) * 3;
#ifdef FillLnR

							ImgLPtr[ToFillIndexL + 0] = r;
							ImgLPtr[ToFillIndexL + 1] = g;
							ImgLPtr[ToFillIndexL + 2] = b;

#endif // CheckLnR
							ImgOutPtr[ToFillIndexO] = r;
							ImgOutPtr[ToFillIndexO + 1] = g;
							ImgOutPtr[ToFillIndexO + 2] = b;
							ImgTagLPtr[j * width + ToFillIndex] = 2000;//ImgTagLPtr[j * width + Fill_index];
						}
					}
				}
			}
		}
	}
	//right image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		for (int i = width - 1; i >= 0; i--) {
			//for (int i = 0; i < width; i++) {
			if (j == 338 && i == 1001)
			{
				i = i;
			}
			index = indexR + i;					//current working pixel index on original images
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 && D > 0) {
				double xI_r = d / D * (D * (i - cx) / f - Baseline * k);
				int I_r = static_cast<int>((1 + xI_r / WL) * width / 2);
				if (I_r > 0 && I_r < width) {
					int indexFill = indexR + I_r;
					int indexFill3 = indexFill * 3;

					int indexO = (indexR * 2 + I_r + width) * 3;
					ImgOutPtr[indexO + 0] = r;
					ImgOutPtr[indexO + 1] = g;
					ImgOutPtr[indexO + 2] = b;
#ifdef CheckLnR
					ImgRPtr[indexFill3 + 0] = r;
					ImgRPtr[indexFill3 + 1] = g;
					ImgRPtr[indexFill3 + 2] = b;
#endif // CheckLnR
					MvImgRPtr[index] = I_r;//跟原始深度彩色图同步，存放当前pixel被移动到了哪里

					ImgTagRPtr[indexFill] = i;//跟结果彩色图同步，i表示这个pixel已经有pixel入住，入住pixel的原始坐标是i。
					if (isRGBFill == 1) {
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
								ImgRPtr[indexFill3 + 0] = r;
								ImgRPtr[indexFill3 + 1] = g;
								ImgRPtr[indexFill3 + 2] = b;
								ImgTagRPtr[indexFill] = 1;
							}
						}
					}

					if (isRGBFill == 3) {
						int FillWin = 2;
						int indexR = i;
						int indexPre = 0;
						int Fill_Min_index = 0;
						int Fill_Max_index = i;
						int Fill_index = I_r;
						do
						{
							indexR++;
						} while (indexR < width && MvImgRPtr[j * width + indexR] == 0);
						if (indexR == width)
							indexPre = i;
						else
							indexPre = indexR;
						int Tar = MvImgRPtr[j * width + indexPre];
						if (I_r > i)
							if (Tar - I_r > 2)
							{
								Fill_Min_index = I_r + 1;// (I_r > 1) ? I_r : 1;
								Fill_Max_index = (Tar < width) ? Tar : width;
								Fill_index = Tar;
							}
							else
							{
								//Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
								//Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
								Fill_Min_index = I_r + 1;
								Fill_Max_index = Tar;
								Fill_index = I_r;
							}
						else
						{
							if (Tar - I_r > 2)
							{
								Fill_Min_index = I_r + 1;// (I_r > 1) ? I_r : 1;
								Fill_Max_index = (Tar + 3 < width) ? Tar + 3 : width;
								Fill_index = Fill_Max_index; // Tar + 3;
							}
							else
							{
								//Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
								//Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
								Fill_Min_index = I_r + 1;
								Fill_Max_index = Tar;
								Fill_index = I_r;
							}

						}
						int ForFillIndexR = (j * width * 2 + width + Fill_index) * 3;
						r = ImgOutPtr[ForFillIndexR + 0];
						g = ImgOutPtr[ForFillIndexR + 1];
						b = ImgOutPtr[ForFillIndexR + 2];

						for (int ToFillIndex = Fill_Min_index; ToFillIndex < Fill_Max_index; ToFillIndex++)
						{
							int ToFillIndexR = (j * width + ToFillIndex) * 3;
							int ToFillIndexO = (j * width * 2 + width + ToFillIndex) * 3;
#ifdef FillLnR

							ImgRPtr[ToFillIndexR + 0] = r;
							ImgRPtr[ToFillIndexR + 1] = g;
							ImgRPtr[ToFillIndexR + 2] = b;

#endif // CheckLnR
							ImgOutPtr[ToFillIndexO] = r;
							ImgOutPtr[ToFillIndexO + 1] = g;
							ImgOutPtr[ToFillIndexO + 2] = b;
							ImgTagRPtr[j * width + ToFillIndex] = 2000+ Fill_index;//ImgTagLPtr[j * width + Fill_index];
						}
					}


				}

			}
		}
	}
	return 1;
}
int RGBD2VR12(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, int isRGBFill, double CameraF,
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
	double scenThread = 200;
	if (k > 1)
		k = k / 10;

	double f;
	if (CameraF == 0)
		f = width / 2 / tan(CameraAngle / 2);
	else
		f = CameraF;

	//target image pixel taken tag
	cv::Mat ImgTagL = cv::Mat::zeros(height, width, CV_16U);
	cv::Mat ImgTagR = cv::Mat::zeros(height, width, CV_16U);
	ushort* ImgTagLPtr = reinterpret_cast<ushort*>(ImgTagL.data);
	ushort* ImgTagRPtr = reinterpret_cast<ushort*>(ImgTagR.data);
	//save the move target I
	cv::Mat MvImgL = cv::Mat::zeros(height, width, CV_16U);
	cv::Mat MvImgR = cv::Mat::zeros(height, width, CV_16U);
	ushort* MvImgLPtr = reinterpret_cast<ushort*>(MvImgL.data);
	ushort* MvImgRPtr = reinterpret_cast<ushort*>(MvImgR.data);


	int indexR = 0;
	int index = 0;
	ushort DepPixel = 0;
	uchar r = 0, g = 0, b = 0;
	// left image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		//for (int i = width - 1; i >= 0; i--) {
		for (int i = 0; i < width; i++) {
			//if (i == 654 && j == 378)
			//{
			//	i = i;
			//}
			index = indexR + i;					//current working pixel index on original images
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 && D > 0) {
				double xI_l = d / D * (D * (i - cx) / f + Baseline * (1 - k));
				int I_l = static_cast<int>((1 + xI_l / WL) * width / 2);
				if (I_l > 0 && I_l < width) {
					int indexFill = j * width + I_l;
					int indexFill3 = indexFill * 3;

					int indexO = indexR * 3 * 2 + I_l * 3;
					ImgOutPtr[indexO] = r;
					ImgOutPtr[indexO + 1] = g;
					ImgOutPtr[indexO + 2] = b;

					MvImgLPtr[index] = I_l;   //跟原始深度彩色图同步，存放当前pixel被移动到了哪里

#ifdef CheckLnR
					ImgLPtr[indexFill3 + 0] = r;
					ImgLPtr[indexFill3 + 1] = g;
					ImgLPtr[indexFill3 + 2] = b;
#endif // CheckLnR

					ImgTagLPtr[indexFill] = i;	//跟结果彩色图同步，i表示这个pixel已经有pixel入住，入住pixel的原始坐标是i。

//					if (isRGBFill == 3) {
//						int FillWin = 2;
//						int indexL = i;
//						int indexPre = 0;
//						int Fill_Min_index = 0;
//						int Fill_Max_index = i;
//						int Fill_index = I_l;
//						do
//						{
//							indexL--;
//						} while (indexL > 0 && MvImgLPtr[j * width + indexL] == 0);
//						if (indexL == 0)
//							indexPre = i;
//						else
//							indexPre = indexL;
//						int Tar = MvImgLPtr[j * width + indexPre];
//						if (I_l > i)
//							if (I_l - Tar > 2)
//							{
//								Fill_Min_index = (Tar - 3 > 1) ? Tar - 3 : 1;
//								Fill_Max_index = (I_l > width) ? width : I_l;
//								Fill_index = Tar - 7;
//							}
//							else
//							{
//								//Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
//								//Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
//								Fill_Min_index = Tar + 1;
//								Fill_Max_index = I_l;
//								Fill_index = I_l;
//							}
//						else
//						{
//							if (I_l - Tar > 2)
//							{
//								Fill_Min_index = (Tar > 1) ? Tar : 1;
//								Fill_Max_index = (I_l > width) ? width : I_l;
//								Fill_index = Tar - 3;
//							}
//							else
//							{
//								//Fill_Min_index = (I_l - FillWin > 1) ? I_l - FillWin : 1;
//								//Fill_Max_index = (I_l + FillWin > width) ? width : I_l + FillWin;
//								Fill_Min_index = Tar + 1;
//								Fill_Max_index = I_l;
//								Fill_index = I_l;
//							}
//
//						}
//						int ForFillIndexL = (j * width * 2 + Fill_index) * 3;
//						r = ImgOutPtr[ForFillIndexL + 0];
//						g = ImgOutPtr[ForFillIndexL + 1];
//						b = ImgOutPtr[ForFillIndexL + 2];
//
//						for (int ToFillIndex = Fill_Min_index; ToFillIndex < Fill_Max_index; ToFillIndex++)
//						{
//							int ToFillIndexL = (j * width + ToFillIndex) * 3;
//							int ToFillIndexO = (j * width * 2 + ToFillIndex) * 3;
//#ifdef FillLnR
//
//							ImgLPtr[ToFillIndexL + 0] = r;
//							ImgLPtr[ToFillIndexL + 1] = g;
//							ImgLPtr[ToFillIndexL + 2] = b;
//
//#endif // CheckLnR
//							ImgOutPtr[ToFillIndexO] = r;
//							ImgOutPtr[ToFillIndexO + 1] = g;
//							ImgOutPtr[ToFillIndexO + 2] = b;
//							ImgTagLPtr[j * width + ToFillIndex] = 2000;//ImgTagLPtr[j * width + Fill_index];
//						}
//					}
				}
			}
		}
	}

	int fillCount1 = 0;
	int fillCount2 = 0;
	int fillCount3 = 0;
	for (int j = 0; j < height; j++) {
		int indexRow = j * width;
		//for (int i = width - 1; i >= 0; i--) {
		int FillTagL = 0;
		int FillTagR = 0;
		int depTagL = 0;
		int depTagR = 0;
		int FillStartTag = 0;
		int FillTag = 0;
		for (int i = 1; i < width-1; i++) {
			//if (i == 654 && j == 378)
			//{
			//	i = i;
			//}
			index = indexRow + i;					//current working pixel index on original images
			if (FillTag == 0 && ImgTagLPtr[index] > 0 && ImgTagLPtr[index + 1] == 0)
			{
				FillTagL = index;
				depTagL = ImgTagLPtr[FillTagL] + indexRow;
				FillTag = 1;
			}
			if (FillTag == 1 && ImgTagLPtr[index] == 0 && ImgTagLPtr[index + 1] > 0)
			{
				FillTagR = index + 1;
				depTagR = ImgTagLPtr[FillTagR] + indexRow;
				FillTag = 2;
			}
			if (FillTag == 2)
			{
				FillTag = 0;
				/*				if ((Depdata[depTagL] - Depdata[depTagR]) > scenThread)
								{
									r = RGBdata[(depTagL-1) * 3 + 0];
									g = RGBdata[(depTagL-1) * 3 + 1];
									b = RGBdata[(depTagL-1) * 3 + 2];
									for (int fi = FillTagL + 1; fi < FillTagR; fi++)
									{
										int findex = (fi + indexRow) * 3;
										ImgOutPtr[findex + 0] = r;
										ImgOutPtr[findex + 1] = g;
										ImgOutPtr[findex + 2] = b;
										fillCount1++;

									}
								}
								else if ((Depdata[depTagR] - Depdata[depTagL]) > scenThread)
								{
									r = RGBdata[(depTagR+1) * 3 + 0];
									g = RGBdata[(depTagR+1) * 3 + 1];
									b = RGBdata[(depTagR+1) * 3 + 2];
									for (int fi = FillTagL + 1; fi < FillTagR; fi++)
									{
										int findex = (fi + indexRow) * 3;
										ImgOutPtr[findex + 0] = r;
										ImgOutPtr[findex + 1] = g;
										ImgOutPtr[findex + 2] = b;
										fillCount2++;

									}
								}
								else
				*/
				{
					for (int fi = FillTagL + 1; fi < FillTagR; fi++)
					{
						int findex = (fi + indexRow) * 3;
						double K = abs(double(depTagR) - double(depTagL)) / (double(FillTagR) - double(FillTagL));

						int rgbIndex = depTagL + K * (fi - FillTagL);
						ImgOutPtr[findex + 0] = RGBdata[(rgbIndex) * 3 + 0];
						ImgOutPtr[findex + 1] = RGBdata[(rgbIndex) * 3 + 1];
						ImgOutPtr[findex + 2] = RGBdata[(rgbIndex) * 3 + 2];
						fillCount3++;

					}
				}

			}
		}
	}

	//right image
	for (int j = 0; j < height; j++) {
		indexR = j * width;
		for (int i = width - 1; i >= 0; i--) {
			//for (int i = 0; i < width; i++) {
			if (j == 338 && i == 1001)
			{
				i = i;
			}
			index = indexR + i;					//current working pixel index on original images
			DepPixel = Depdata[index];
			double D = DepPixel;
			r = RGBdata[index * 3 + 0];
			g = RGBdata[index * 3 + 1];
			b = RGBdata[index * 3 + 2];
			if (MaskPtr[index] == 0 && D > 0) {
				double xI_r = d / D * (D * (i - cx) / f - Baseline * k);
				int I_r = static_cast<int>((1 + xI_r / WL) * width / 2);
				if (I_r > 0 && I_r < width) {
					int indexFill = indexR + I_r;
					int indexFill3 = indexFill * 3;

					int indexO = (indexR * 2 + I_r + width) * 3;
					ImgOutPtr[indexO + 0] = r;
					ImgOutPtr[indexO + 1] = g;
					ImgOutPtr[indexO + 2] = b;
#ifdef CheckLnR
					ImgRPtr[indexFill3 + 0] = r;
					ImgRPtr[indexFill3 + 1] = g;
					ImgRPtr[indexFill3 + 2] = b;
#endif // CheckLnR
					MvImgRPtr[index] = I_r;//跟原始深度彩色图同步，存放当前pixel被移动到了哪里

					ImgTagRPtr[indexFill] = i;//跟结果彩色图同步，i表示这个pixel已经有pixel入住，入住pixel的原始坐标是i。
				}

			}
		}
	}
	for (int j = 0; j < height; j++) {
		int indexRow = j * width;
		//for (int i = width - 1; i >= 0; i--) {
		int FillTagL = 0;
		int FillTagR = 0;
		int depTagL = 0;
		int depTagR = 0;
		int FillStartTag = 0;
		int FillTag = 0;
		for (int i = 0; i < width; i++)
		{
			index = indexRow + i;					//current working pixel index on original images
			if (FillTag == 0 && ImgTagRPtr[index] > 0 && ImgTagRPtr[index + 1] == 0)
			{
				FillTagL = index;
				depTagL = ImgTagRPtr[FillTagL] + indexRow;
				FillTag = 1;
			}
			if (FillTag == 1 && ImgTagRPtr[index] == 0 && ImgTagRPtr[index + 1] > 0)
			{
				FillTagR = index + 1;
				depTagR = ImgTagRPtr[FillTagR] + indexRow;
				FillTag = 2;
			}
			if (FillTag == 2)
			{
				FillTag = 0;

				for (int fi = FillTagL + 1; fi < FillTagR; fi++)
				{
					int findex = (fi + indexRow + width) * 3;
					double K = abs(double(depTagR) - double(depTagL)) / (double(FillTagR) - double(FillTagL));

					int rgbIndex = depTagL + K * (fi - FillTagL);
					ImgOutPtr[findex + 0] = RGBdata[(rgbIndex) * 3 + 0];
					ImgOutPtr[findex + 1] = RGBdata[(rgbIndex) * 3 + 1];
					ImgOutPtr[findex + 2] = RGBdata[(rgbIndex) * 3 + 2];
					fillCount3++;

				}


			}
		}
	}

	return 1;
}


cv::Mat findMask(cv::Mat& depth, int width, int height) 
{
	cv::Mat Mask = cv::Mat::zeros(height, width, CV_8U); // Assuming depth is CV_8U. Change accordingly.
	cv::Mat Tag = cv::Mat::zeros(height, width, CV_8U);
	unsigned short int* depthData = (unsigned short int*)depth.data;
	uchar* maskData = Mask.data;
	uchar* TagData = Tag.data;

	maskData[0] = 1;
	maskData[width - 1] = 1;
	maskData[(height - 1) * width] = 1;
	maskData[height * width - 1] = 1;
	int i = 0;
	int m = 0;
	for (i = 0; i < width / 2; i++) {
		for (m = 0; m < i+2; m++) {
			int x, y;
			int index = 0;
			x = i + 1;
			y = m;
			index = x * width + y;
			TagData[index] = 1;
			if (depthData[index] == 0 && maskData[index - width] == 1) {
				maskData[index] = 1;
			}

			x = m;
			y = i + 1;
			index = x * width + y;
			TagData[index] = 1;
			if (depthData[index] == 0 && maskData[index - 1] == 1) {
				maskData[index] = 1;
			}

			x = height - i - 1;
			y = m;
			index = x * width + y;
			TagData[index] = 1;
			if (depthData[index] == 0 && maskData[index + width] == 1) {
				maskData[index] = 1;
			}

			x = height - m - 1;
			y = i + 1;
			index = x * width + y;
			TagData[index] = 1;
			if (depthData[index] == 0 && maskData[index - 1] == 1) {
				maskData[index] = 1;
			}

			x = i + 1;
			y = width - m - 1;
			index = x * width + y;
			TagData[index] = 1;
			if (depthData[index] == 0 && maskData[index - width] == 1) {
				maskData[index] = 1;
			}

			x = m;
			y = width - i - 1;
			index = x * width + y;
			TagData[index] = 1;
			if (depthData[index] == 0 && maskData[index + 1] == 1) {
				maskData[index] = 1;
			}

			x = height - i - 1;
			y = width - m - 1;
			index = x * width + y;
			TagData[index] = 1;
			if (depthData[index] == 0 && maskData[index + width] == 1) {
				maskData[index] = 1;
			}

			x = height - m - 1;
			y = width - i - 1;
			index = x * width + y;
			TagData[index] = 1;
			if (depthData[index] == 0 && maskData[index + 1] == 1) {
				maskData[index] = 1;
			}
		}
	}

	return Mask;
}

cv::Mat SimpleFill(const cv::Mat& depth, const cv::Mat& Mask, int width, int height, int fTimes)
{
	int index = 0;
	//int indexR = 0;
	cv::Mat Depth2Fill = depth.clone();
	cv::Mat DepthFilled = depth.clone();
	ushort* DepTar = reinterpret_cast<ushort*>(DepthFilled.data);
	ushort* DepSrc = reinterpret_cast<ushort*>(Depth2Fill.data);
	const uchar* MaskPtr = Mask.data;
	int Adj[8] = { -width - 1, -width, -width + 1, -1, 1, width - 1, width, width + 1 };
	for (int ft = 0; ft < fTimes; ft++)
	{
		for (int j = 1; j < height - 1; j++)
		{
			//indexR = j * width;
			for (int i = 1; i < width - 1; i++)
			{
				
				if (i == 1242 && j == 312)
				{
					i = i;
				}

				
				index = j * width + i;
				if (MaskPtr[index] == 0 && DepSrc[index] == 0) {
					double depthSum = 0;
					//double depthAvg = 0;
					int depthCnt = 0;
					int Adjinx;
					for (int t = 0; t < 8; t++) {
						Adjinx = Adj[t] + index;
						if (DepSrc[Adjinx] != 0) {
							depthCnt++;
							depthSum += DepSrc[Adjinx];
						}
					}
					if (depthCnt > 0) {
						DepTar[index] = static_cast<ushort>(depthSum / double(depthCnt));
					}
				}
			}
		}
		DepthFilled.copyTo(Depth2Fill);
	}
	return DepthFilled;
}
