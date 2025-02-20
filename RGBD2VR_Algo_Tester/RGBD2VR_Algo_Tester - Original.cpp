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


//int RGBD2VR3(const cv::Mat& depth, const cv::Mat& color, const cv::Mat& Mask, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
//	cv::Mat& ImgL, cv::Mat& ImgR, cv::Mat& ImageOut/*, cv::Mat& DisparityL, cv::Mat& DisparityR, cv::Mat& Disparity_Full, cv::Mat& MvImgL, cv::Mat& MvImgR*/);
//int RGBD2VR4(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
//	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr);
//int RGBD2VR6(ushort* Depdata, uchar* RGBdata, uchar* MaskPtr, int width, int height, double DBack, double k, bool isRGBFill, double CameraF,
//	uchar* ImgLPtr, uchar* ImgRPtr, uchar* ImgOutPtr);
cv::Mat calculateIntensity(const cv::Mat& rgbImage);
cv::Mat findMask2(cv::Mat& depth, int width, int height);
//cv::Mat SimpleFill(const cv::Mat& depth, const cv::Mat& Mask, int width, int height, int fTimes = 1);


int main() {
	//std::string main_folder = "D:/NanXu/Patent_VR/Depth_Filling/Data";
	std::string main_folder = "D:/NanXu/Patent_VR/09162022/OneDrive_2_9-19-2022"; //MEGA æ≠µ‰–‹
	//std::string main_folder = "D:/NanXu/Patent_VR/MEGA_06202024"; //MEGA »ÀœÒ
	//std::string main_folder = "F:/Patent_VR/07092024/200255000"; //MEGA »ÀœÒ
	//std::string main_folder = "D:/NanXu/Patent_VR/03132024/RGBD2VR/5791826"; //MEGA
	//std::string main_folder = "D:/NanXu/Patent_VR/Data_05292023/Images/Input";
	//std::string main_folder = "D:/NanXu/Patent_VR/Data_10242023/sn_AYKB73D0006";
	//std::string main_folder = "D:/NanXu/Patent_VR/Data_10272023/sn_AY8M63P003J_gemini_2L";
	//std::string main_folder = "D:/NanXu/Patent_VR/Data_10272023/sn_AYK593P000J_gemini_2XL";
	//std::string main_folder = "D:/NanXu/Patent_VR/Data_10272023/CapturedFrames_gemini_2L";
	//std::string main_folder = "D:/NanXu/Patent_VR/Data_11132023/g2xl_D2C_HW_1280x800";
	//std::string main_folder = "D:/NanXu/Patent_VR/V7Lab_Dat/test/HR/09_Book_Store"; //Õºø‚
	//std::string main_folder = "F:/Patent_VR/Picked_Background"; //±≥æ∞Õºø‚
	//std::string main_folder = "F:/Patent_VR/V7Lab_Dat"; //Õºø‚
	//std::string bImgName = "F:/Patent_VR/V7Lab_Dat/test/HR/09_Book_Store/result/result_1.png"; //±≥æ∞Õº
	//std::string bImgName = "F:/Patent_VR/V7Lab_Dat/result/result_1.png"; //±≥æ∞Õº
	std::string bImgName = "F:/Patent_VR/Picked_Background/result/result_11.png"; //±≥æ∞Õº
	std::string face_folder = "D:/NanXu/Patent_VR/face"; //»À¡≥Õº
	std::string rgb_folder = main_folder + "/color/png";
	std::string depth_folder = main_folder + "/depth/png";
	//std::string rgb_folder = main_folder + "/color";
	//std::string depth_folder = main_folder + "/depth";
	std::string result_folder = main_folder + "/result";
	std::string color_name_pre = "color_";
	std::string depth_name_pre = "depth_";
	std::string result_name_pre = "result_";
	//for (int i = 1; i < 388; i++) {


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
		//cv::Mat ImgL(color.size(), CV_8UC3, cv::Scalar(0, 0, 0));
		//cv::Mat ImgR(color.size(), CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat Mask = (depth <= 200)/255;
		cv::Mat ImageOut(DstLeng, DstWidth * 2, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat DstImage(DstLeng, DstWidth * 2, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat DepthFilled = cv::Mat::zeros(depth.size(), CV_16U);
		cv::Mat MegaMask = cv::Mat::zeros(depth.size(), CV_8U);		//only when FEMTO MEGA or Azure Kinect depth
		ushort* DFDPtr = reinterpret_cast<ushort*>(DepthFilled.data);

		uchar* MaskPtr;
		//bool* maskcbf = reinterpret_cast<bool*>(Mask.data);
		uchar* RGBdata = color.data;
		ushort* Depdata = reinterpret_cast<ushort*>(depth.data);
		//uchar* ImgLPtr = ImgL.data;
		//uchar* ImgRPtr = ImgR.data;
		uchar* ImgOutPtr = ImageOut.data;
		bool backTag = 0; //  «∑Ò π”√±≥æ∞Õº£¨1= π”√
		cv::Mat backImg; //±≥æ∞Õº
		uchar* bImgPtr;

#ifdef useBack
			backTag = 1;
			backImg = cv::imread(bImgName);
			if (backImg.size() != ImageOut.size()) 
			{
				cv::resize(backImg, backImg, ImageOut.size());
			}
			bImgPtr = backImg.data;

#endif // useBack
		cv::Mat bUse (depth.size(), CV_8UC3, cv::Scalar(0, 0, 0)); //±Íº«æÿ’Û W*H*3
		uchar* bUsePtr = bUse.data;
		//DepthFilled = depth.clone();
		//DepthFilled = SimpleFill(depth, Mask, width, height, 2);
		
#ifdef isMEGA
		MegaMask = findMask2(depth, width, height);//(depth <= 200);// (depth.size(), CV_8UC1, cv::Scalar(0));
		//MegaMask = cv::imread("D:/NanXu/Patent_VR/Mask/MEGA_Mask.png", -1);
		//cv::imwrite("D:/NanXu/Patent_VR/RGBD2VR_Algo_Tester/x64/Debug/MEGA_Mask.png", MegaMask);
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
		//auto m_intrinsics = blink_input::parse_intrinsics_yaml("D:/NanXu/SDK/Blink/BlinkSDK_3.35.3_orbbec_external_files/camera_parameters/conceptd-dev_intrinsics.yml");
		//auto m_extrinsics = blink_input::parse_intrinsics_yaml("D:/NanXu/SDK/Blink/BlinkSDK_3.35.3_orbbec_external_files/camera_parameters/conceptd-dev_extrinsics.yml");
		/******************/
		/* Run estimation */
		/******************/
		//cv::Mat frame = color.clone();
		int active_user_id = -1;
		uint64_t frame_id = 0;
		FaceIdsOutput face_ids;
		blink::common::BlVector<BlPoint3d> eye_center_3d;
		
		//blink::Image image(1920, 1080, faceColor.data, BlImageFormat::RGB, blink_utils::get_current_timestamp(), m_intrinsics[0], m_extrinsics, false, BlDeviceOrientation::BL_ORIENTATION_0);
		//blink::Image image(uint32_t(width), uint32_t(height), faceColor.data, BlImageFormat::RGB, blink_utils::get_current_timestamp(), m_intrinsics[0], m_extrinsics, false, BlDeviceOrientation::BL_ORIENTATION_0);


		auto m_intrinsics = blink_input::parse_intrinsics_yaml("D:/NanXu/SDK/Blink/BlinkSDK_3.35.3_orbbec_external_files/camera_parameters/conceptd-dev_intrinsics.yml");
		auto m_extrinsics = blink_input::parse_extrinsics_yaml("D:/NanXu/SDK/Blink/BlinkSDK_3.35.3_orbbec_external_files/camera_parameters/conceptd-dev_extrinsics.yml");

		//Image(
		//	uint32_t width,
		//	uint32_t height,
		//	uint8_t* data,
		//	BlImageFormat format,
		//	BlTimestampMs timestamp_ms,
		//	const BlIntrinsics& intrinsics,
		//	const BlExtrinsics& extrinsics,
		//	bool  is_mirrored,
		//	BlDeviceOrientation orientation);

		blink::Image faceImage(faceGray.cols, faceGray.rows, (uint8_t*)faceGray.data, BlImageFormat::Y8, blink_utils::get_current_timestamp(), m_intrinsics[0], m_extrinsics, false, BlDeviceOrientation::BL_ORIENTATION_0);

		blink::common::BlVector<blink::Image> images(NUM_OF_SOURCES);
		images.at(0) = faceImage;
		bool isFace = true;

		try {
			face_detector.estimate(images, face_detector_data);
		}
		catch (const blink::exception::BlException& e)
		{
			//end = std::chrono::steady_clock::now();
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
		}
		//eye_centers_3d_per_user[id] = eye_center_3d;


#endif // isBLINK

		R2VProperties myProp;

		auto start_time = std::chrono::high_resolution_clock::now();
		int ImageNum = 10;
		double Pupil_D = 58.0;
		double eyeShift = 0.0;
		double dThred = 150;
		double frontDepthEstimate;
		double DepAvg;

		for (int i = 1; i <= ImageNum; i++)
		{


			start_time = std::chrono::high_resolution_clock::now();

			rgb_filename = color_name_pre + std::to_string(i) + ".png";
			depth_filename = depth_name_pre + std::to_string(i) + ".png";
			result_filename = result_name_pre + std::to_string(i) + ".png";

			rgb_image_path = rgb_folder + "/" + rgb_filename;
			depth_image_path = depth_folder + "/" + depth_filename;
			result_image_path = result_folder + "/" + result_filename;

			// Load the pair of images and measure time consumption
			color = cv::imread(rgb_image_path, cv::IMREAD_COLOR);
			depth = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);


			if (color.empty() || depth.empty()) 
			{
				std::cerr << "Error loading image pair: " << rgb_image_path << ", " << depth_image_path << std::endl;
				return -1;
			}
			if (color.cols != DstWidth || color.rows != DstLeng)
				cv::resize(color, color, cv::Size(DstWidth, DstLeng));
			if (depth.cols != DstWidth || depth.rows != DstLeng)
				cv::resize(depth, depth, cv::Size(DstWidth, DstLeng));
			RGBdata = color.data;
			Depdata = reinterpret_cast<ushort*>(depth.data);

			DepthFilled = depth.clone();
			DepthFilled = SimpleFill(depth, MegaMask, width, height, 2);
			DFDPtr = reinterpret_cast<ushort*>(DepthFilled.data);

			Mask = (DepthFilled <= 200) / 255;
			Mask = Mask.mul(MegaMask);
			//MaskPtr = Mask.data;
			MaskPtr = MegaMask.data;

			auto end_time1 = std::chrono::high_resolution_clock::now();
			auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - start_time).count();
			std::cout << "Depth Filling in " << duration1 << " ms" << std::endl;


			cv::Scalar meanValue = cv::mean(DepthFilled);
			DepAvg = meanValue[0];
			frontDepthEstimate = DepAvg;
#ifdef isBLINK
			cvtColor(color, faceGray, cv::COLOR_RGB2GRAY);
			
			
			blink::Image faceImage(faceGray.cols, faceGray.rows, (uint8_t*)faceGray.data, BlImageFormat::Y8, blink_utils::get_current_timestamp(), m_intrinsics[0], m_extrinsics, false, BlDeviceOrientation::BL_ORIENTATION_0);
			//images.~BlVector();

			//blink::common::BlVector<blink::Image> images(NUM_OF_SOURCES);
			images.at(0) = faceImage;
			isFace = true;
			try {
				BlStatus BLstat = face_detector.estimate(images, face_detector_data);
			}
			catch (const blink::exception::BlException& e)
			{
				//end = std::chrono::steady_clock::now();
				isFace = false;
				std::cout << "[WARN] Failed to estimate, moving to the next frame: " << std::string(e.what()) << std::endl;

			}

			//cv::flip(faceGray, faceGray, 1);
			if (isFace)
			{

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

#endif // isBlink
#ifdef rgbShift
			//cv::Mat dst;
			// ∂®“Â∆Ω“∆æÿ’Û
			cv::Mat transMat = (cv::Mat_<double>(2, 3) << 1, 0, 4, 0, 1, 0);
			// ”¶”√∑¬…‰±‰ªª
			cv::warpAffine(color, color, transMat, color.size());
#endif // rgbShift
			start_time = std::chrono::high_resolution_clock::now();
#ifdef noBack
			int stat = RGBD2VR13(Depdata, RGBdata, MaskPtr, DstWidth, DstLeng, max_val, isRGBFill, CameraF,
				ImgLPtr, ImgRPtr, ImgOutPtr, Pupil_D, eyeShift);
#endif // noBack
			

#ifdef useBack
			
			int checki, checkj;
			myProp.CameraFocus = CameraF;
			myProp.depthAverage = DepAvg;
			myProp.frontDepthEst = 800;//frontDepthEstimate;
			myProp.depthWidth = depth.cols;
			myProp.depthHeight = depth.rows;
			myProp.rgbWidth = color.cols;
			myProp.rgbHeight = color.rows;
			myProp.useBackground = backTag;
			myProp.pupilDistance = 58.0;// Pupil_D;
			myProp.eyeShift = eyeShift;
			myProp.depthThred = dThred;
			myProp.cameraCX = 535.0;	//from camera calibration data;
			myProp.cameraHFoV = 75.0;
			myProp.eyeFoV = 90.0; //human eye FoV
			//int stat = RGBD2VR_V15_BkGlnd(DFDPtr, RGBdata, MaskPtr, ImgOutPtr, bUsePtr, myProp, bImgPtr);
			checki = width - 1355;
			checkj = 282;
			int stat = RGBD2VR_V16_BkGlnd(DFDPtr, RGBdata, MaskPtr, ImgOutPtr, bUsePtr, myProp,  checki,  checkj, bImgPtr);
#endif

			auto end_time = std::chrono::high_resolution_clock::now(); 
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
			std::cout << "Process image pair in " << duration << " ms" << std::endl;

			cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
			cv::imshow("Image", ImageOut);
			cv::imwrite(result_image_path, ImageOut);
			ImageOut.setTo(cv::Scalar(0,0,0));
		}
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
