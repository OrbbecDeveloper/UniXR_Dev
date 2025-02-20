// RGBD2VR_Algo_Tester.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <iostream>
#include <string>
#include <chrono>
#include "RGBD2VR_Algo.h"
//#include <RGBD2VR_V12.h>

//#include <RGBD2VR_V1303_BkGlnd.h>
//#include <RGBD2VR_V14_BkGlnd.h>
#include "cbf_windows.h"
//#include <RGBD2VR_V15_BkGlnd.h>
#include <RGBD2VR_V16_BkGlnd.h>

#define isBLINK
#define isMEGA 
#define useBack
//#define noBack
#define	rgbShift
#define checkTime
#define HunmanTarget 1


#ifdef isBLINK

#include "useBlink.h"

#endif 


#include <RGBD2VR_V13.h>


cv::Mat calculateIntensity(const cv::Mat& rgbImage);
cv::Mat findMask2(cv::Mat& depth, int width, int height);


int main() {
	std::string main_folder = "F:/Patent_VR/07092024/200255000"; //MEGA 
	std::string bImgName = "F:/Patent_VR/Picked_Background/result/result_11.png"; //Background
	std::string face_folder = "D:/NanXu/Patent_VR/face"; //face
	std::string rgb_folder = main_folder + "/color/png";
	std::string depth_folder = main_folder + "/depth/png";
	std::string result_folder = main_folder + "/result";
	std::string color_name_pre = "color_";
	std::string depth_name_pre = "depth_";
	std::string result_name_pre = "result_";


		std::string rgb_filename = color_name_pre + "1.png";
		std::string depth_filename = depth_name_pre + "1.png";
		std::string result_filename = result_name_pre + "1.png";

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
		//target resolution 
		int DstWidth = 1920;
		int DstLeng = 1080;
		if (color.cols != DstWidth || color.rows != DstLeng)
			cv::resize(color, color, cv::Size(DstWidth, DstLeng));
		if (depth.cols != DstWidth || depth.rows != DstLeng)
			cv::resize(depth, depth, cv::Size(DstWidth, DstLeng));

		int width = color.cols;
		int height = color.rows;
		int PixelNum = width * height;
		double min_val, max_val;
		double k = 0.5;
		int isRGBFill = 3;
		double CameraF = 0;
		cv::minMaxLoc(depth, &min_val, &max_val);
		cv::Mat Mask = (depth <= 200)/255;
		cv::Mat ImageOut(DstLeng, DstWidth * 2, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat DstImage(DstLeng, DstWidth * 2, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat DepthFilled = cv::Mat::zeros(depth.size(), CV_16U);
		cv::Mat MegaMask = cv::Mat::zeros(depth.size(), CV_8U);		//only when FEMTO MEGA or Azure Kinect depth
		ushort* DFDPtr = reinterpret_cast<ushort*>(DepthFilled.data);

		uchar* MaskPtr;
		uchar* RGBdata = color.data;
		ushort* Depdata = reinterpret_cast<ushort*>(depth.data);
		uchar* ImgOutPtr = ImageOut.data;
		bool backTag = 0; // 是否使用背景图，1=使用
		cv::Mat backImg; //背景图
		uchar* bImgPtr;
		R2VProperties myProp;

		int ImageNum = 10;
		double Pupil_D = 58.0;
		double eyeShift = 0.0;
		double dThred = 150;
		double frontDepthEstimate;
		double DepAvg;

#ifdef useBack
			backTag = 1;
			backImg = cv::imread(bImgName);
			if (backImg.size() != ImageOut.size()) 
			{
				cv::resize(backImg, backImg, ImageOut.size());
			}
			bImgPtr = backImg.data;

#endif // use Background
		cv::Mat bUse (depth.size(), CV_8UC3, cv::Scalar(0, 0, 0)); //Marker W*H*3
		uchar* bUsePtr = bUse.data;
		
#ifdef isMEGA
		MegaMask = findMask2(depth, width, height);
#endif

#ifdef isBLINK
		cv::Mat faceGray;
		cv::Mat faceDepth;
		cv::Mat faceColor;

		if (HunmanTarget)
		{
			faceDepth = depth.clone();
			cv::cvtColor(color, faceGray, cv::COLOR_RGB2GRAY);
		}
		else
		{
			std::string faceFolder = "D:/NanXu/Patent_VR/face";
			std::string faceRGBImg = faceFolder + "/color/rgb001.png";
			std::string faceDepthImg = faceFolder + "/depth/depth001.png";

			faceColor = cv::imread(faceRGBImg, cv::IMREAD_COLOR);
			cvtColor(faceColor, faceGray, cv::COLOR_RGB2GRAY);
			faceDepth = cv::imread(faceDepthImg, cv::IMREAD_ANYDEPTH);
		}
		std::string blinkOrbbecFolder = "D:/NanXu/SDK/Blink/BlinkSDK_3.35.3_orbbec_external_files/";
		std::string blinkSDKFolder = "D:/NanXu/SDK/Blink/BlinkSDK_3.35.3/BlinkSDK_3.35.3/";
		auto video_path = blinkOrbbecFolder + "video";
		auto max_num_faces = MAX_NUM_OF_FACES;
		auto intrinsics = "intrinsics";
		std::string resources_dir = blinkSDKFolder + "resources";
		bool is_mirrored = 0;
		BlImageFormat image_format = BlImageFormat::Y8;
		float scale_factor = 1.0f;
		int fps = 30;
		std::string csv_dump_path = "";
		std::string video_dump_path = "";
		bool enable_ui = false;
		std::string license_user = "orbbec";
		std::string license_key = "BIGGS-OEPBF-FLIKS-OMEJM";
		blink::BlinkContext ctx(license_user.c_str(), license_key.c_str(), resources_dir.c_str());
		ctx.set_logger_file("log.txt");
		/**************************/
		/* Face Detector creation */
		/**************************/
		blink::FaceDetectorConfig face_detector_config;
		face_detector_config.set_detection_mode(IMAGE_MODE);
		face_detector_config.set_max_number_of_sources(NUM_OF_SOURCES);
		face_detector_config.set_max_num_of_faces(max_num_faces);
		blink::FaceDetector face_detector(ctx, face_detector_config);
		blink::FaceDetectorData face_detector_data;
		/*******************************/
		/* Head Pose Detector creation */
		/*******************************/
		blink::HeadPoseDetectorConfig config;
		blink::HeadPoseDetector head_pose_detector(ctx, config);
		blink::HeadPoseDetectorData head_pose_data;
		BlHeadPose head_pose;

		/*****************************************/
		/* 3D Eye Localization Detector creation */
		/*****************************************/
		blink::Landmarks3dDetectorConfig landmarks_3d_config;
		blink::Landmarks3dDetector landmarks_3d_detector(ctx, landmarks_3d_config);
		blink::Landmarks3dDetectorData landmarks_3d_data;

		/*******************************/
		/*   Sample Input / Output     */
		/*******************************/
		std::vector<BlDataType> data_to_output = { BlDataType::FACE_IDS, BlDataType::HEAD_POSE, BlDataType::EYE_CENTERS_3D, BlDataType::FACE_LANDMARKS_3D };
		BlResolution camera_resolution = { width, height };
		/******************/
		/* Run estimation */
		/******************/
		int active_user_id = -1;
		uint64_t frame_id = 0;
		FaceIdsOutput face_ids;
		blink::common::BlVector<BlPoint3d> eye_center_3d;
		


		auto m_intrinsics = blink_input::parse_intrinsics_yaml("D:/NanXu/SDK/Blink/BlinkSDK_3.35.3_orbbec_external_files/camera_parameters/conceptd-dev_intrinsics.yml");
		auto m_extrinsics = blink_input::parse_extrinsics_yaml("D:/NanXu/SDK/Blink/BlinkSDK_3.35.3_orbbec_external_files/camera_parameters/conceptd-dev_extrinsics.yml");


		blink::Image faceImage(faceGray.cols, faceGray.rows, (uint8_t*)faceGray.data, BlImageFormat::Y8, blink_utils::get_current_timestamp(), m_intrinsics[0], m_extrinsics, false, BlDeviceOrientation::BL_ORIENTATION_0);

		blink::common::BlVector<blink::Image> images(NUM_OF_SOURCES);
		images.at(0) = faceImage;
		bool isFace = true;

		try {
			face_detector.estimate(images, face_detector_data);
		}
		catch (const blink::exception::BlException& e)
		{
			isFace = false;
			std::cout << "[WARN] Failed to estimate, moving to the next frame: " << std::string(e.what()) << std::endl;

		}
		if (isFace)
		{
			cv::flip(faceGray, faceGray, 1);
			face_detector_data.get_face_ids(SOURCE_INDEX, image_format, face_ids);				//get face ids
			head_pose_detector.estimate(face_detector_data, head_pose_data);					//head pose estimate
			head_pose_data.get_head_pose(SOURCE_INDEX, face_ids[0], image_format, head_pose);	//get head pose
			landmarks_3d_detector.estimate(head_pose_data, landmarks_3d_data);					//land mark estimate
			landmarks_3d_data.get_3d_eye_centers(SOURCE_INDEX, 0, image_format, eye_center_3d);	//get 3D eye center
			double x1 = eye_center_3d.at(0).x;
			double y1 = eye_center_3d.at(0).y;
			double z1 = eye_center_3d.at(0).z;
			double x2 = eye_center_3d.at(1).x;
			double y2 = eye_center_3d.at(1).y;
			double z2 = eye_center_3d.at(1).z;
			Pupil_D = abs(sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2)));
			frontDepthEstimate = (z1 + z2) / 2;
		}


#endif // isBLINK



			DepthFilled = depth.clone();
			DepthFilled = SimpleFill(depth, MegaMask, width, height, 2);
			DFDPtr = reinterpret_cast<ushort*>(DepthFilled.data);

			Mask = (DepthFilled <= 200) / 255;
			Mask = Mask.mul(MegaMask);
			MaskPtr = MegaMask.data;



			cv::Scalar meanValue = cv::mean(DepthFilled);
			DepAvg = meanValue[0];
			frontDepthEstimate = DepAvg;
#ifdef isBLINK


#endif // isBlink
#ifdef rgbShift //if D2C is not accurate
			cv::Mat transMat = (cv::Mat_<double>(2, 3) << 1, 0, 4, 0, 1, 0);
			cv::warpAffine(color, color, transMat, color.size());
#endif // rgbShift
			

#ifdef useBack
			
			int checki, checkj;
			myProp.CameraFocus = CameraF;
			myProp.depthAverage = DepAvg;
			myProp.frontDepthEst = frontDepthEstimate; //800;
			myProp.depthWidth = depth.cols;
			myProp.depthHeight = depth.rows;
			myProp.rgbWidth = color.cols;
			myProp.rgbHeight = color.rows;
			myProp.useBackground = backTag;
			myProp.pupilDistance = Pupil_D; //58.0
			myProp.eyeShift = eyeShift;
			myProp.depthThred = dThred;
			myProp.cameraCX = 535.0;	//from camera calibration data;
			myProp.cameraHFoV = 75.0;
			myProp.eyeFoV = 90.0; //human eye FoV
			checki = width - 1355;
			checkj = 282;
			int stat = RGBD2VR_V16_BkGlnd(DFDPtr, RGBdata, MaskPtr, ImgOutPtr, bUsePtr, myProp,  bImgPtr);
#endif


			cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
			cv::imshow("Image", ImageOut);
			cv::imwrite(result_image_path, ImageOut);
			ImageOut.setTo(cv::Scalar(0,0,0));
	// Wait for the user to press a key
	cv::waitKey(0);


    return 0;
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


cv::Mat findMask2(cv::Mat& depth, int width, int height)
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
	for (m = 0; m < i + 2; m++) {
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

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
