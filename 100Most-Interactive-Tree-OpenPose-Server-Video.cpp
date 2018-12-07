// ------------------------- OpenPose C++ API Tutorial - Example 8 - XXXXXXXXXXXXX -------------------------
// If the user wants to learn to use the OpenPose library, we highly recommend to start with the
// examples in `examples/tutorial_api_cpp/`.
// This example summarizes all the functionality of the OpenPose library:
	// 1. Read folder of images / video / webcam  (`producer` module)
	// 2. Extract and render body keypoint / heatmap / PAF of that image (`pose` module)
	// 3. Extract and render face keypoint / heatmap / PAF of that image (`face` module)
	// 4. Save the results on disk (`filestream` module)
	// 5. Display the rendered pose (`gui` module)
	// Everything in a multi-thread scenario (`thread` module)
	// Points 2 to 5 are included in the `wrapper` module
// In addition to the previous OpenPose modules, we also need to use:
	// 1. `core` module:
		// For the Array<float> class that the `pose` module needs
		// For the Datum struct that the `thread` module sends between the queues
	// 2. `utilities` module: for the error & logging functions, i.e., op::error & op::log respectively
// This file should only be used for the user to take specific examples.

// Command-line user intraface
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>



/* Winsock server dependencies */

// Complete Winsock Server Code
// https://docs.microsoft.com/en-us/windows/desktop/winsock/complete-server-code#winsock-server-source-code

#undef UNICODE

#define WIN32_LEAN_AND_MEAN

// https://social.msdn.microsoft.com/Forums/vstudio/en-US/af431084-0b0c-45a7-bdcf-bdf4bf07afcb/help-including-winsock2h-gives-116-errors?forum=vcgeneral
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

// added by Chris
#include <iostream>
#include <process.h>
#include <string>

using namespace std;

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
// #pragma comment (lib, "Mswsock.lib")

//#define DEFAULT_BUFLEN 512
//#define DEFAULT_BUFLEN 65536
#define DEFAULT_BUFLEN 524288
//#define DEFAULT_PORT "27156"

/* end of Winsock server dependencies */



/* class declarations */

// If the user needs his own variables, he can inherit the op::Datum struct and add them in there.
// UserDatum can be directly used by the OpenPose wrapper because it inherits from op::Datum, just define
// WrapperT<std::vector<UserDatum>> instead of Wrapper (or equivalently WrapperT<std::vector<UserDatum>>)
struct UserDatum : public op::Datum
{
	bool boolThatUserNeedsForSomeReason;

	UserDatum(const bool boolThatUserNeedsForSomeReason_ = false) :
		boolThatUserNeedsForSomeReason{ boolThatUserNeedsForSomeReason_ }
	{}
};

// The W-classes can be implemented either as a template or as simple classes given
// that the user usually knows which kind of data he will move between the queues,
// in this case we assume a std::shared_ptr of a std::vector of UserDatum
class WUserOutput : public op::WorkerConsumer<std::shared_ptr<std::vector<UserDatum>>>
{
private:
	SOCKET _clientSocket = INVALID_SOCKET;
	bool _isPrintData = false;
	bool _isShowImage = true;
	string _tcpMsgDelimiter = "[TCP]";

public:
	void setClientSocket(SOCKET clientSocket);
	void setIsPrintData(bool isPrintData);
	void setIsShowImage(bool isShowImage);
	void printNumOfPeople(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr);
	
	int sendData(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr);
	void printData(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr);

	void initializationOnThread();
	void workConsumer(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr);
};

/* end of class declarations */


/* function declarations */

bool checkBoolCommandLineArgument(char *arg);

/* Winsock server declarations */
int initializeTcpServer(int port, SOCKET *listenSocket);
void closeDownTcpServer(SOCKET ListenSocket, SOCKET ClientSocket);
void constructOpenPoseWorkerWrapper
(
	op::WrapperStructPose wrapperStructPose,
	op::WrapperStructFace wrapperStructFace,
	op::WrapperStructHand wrapperStructHand,
	op::WrapperStructExtra wrapperStructExtra,
	op::WrapperStructInput wrapperStructInput,
	op::WrapperStructOutput wrapperStructOutput,
	bool workerOutputOnNewThread,
	std::shared_ptr<WUserOutput> wUserOutput,
	op::WrapperT<std::vector<UserDatum>> *opWrapperT
);
/* end of Winsock server declarations */

/* openpose declarations */
string getSimplifiedJsonFromPoseKeyPoints(op::Array<float> poseKeyPoints);
/* end of openpose declarations */

/* end of function declarations */


/* main functions */

int tutorialApiCpp8(string modelDirPath, string tcpMsgDelimiter, int portToUse,
	bool isShowImage, bool isPrintData)
{
	try
	{
		op::log("Starting OpenPose demo...", op::Priority::High);		

		// logging_level
		op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
			__LINE__, __FUNCTION__, __FILE__);
		op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
		op::Profiler::setDefaultX(FLAGS_profile_speed);
		// // For debugging
		// // Print all logging messages
		// op::ConfigureLog::setPriorityThreshold(op::Priority::None);
		// // Print out speed values faster
		// op::Profiler::setDefaultX(100);

		// Applying user defined configuration - GFlags to program variables
		// cameraSize
		const auto cameraSize = op::flagsToPoint(FLAGS_camera_resolution, "-1x-1");
		// outputSize
		const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
		// netInputSize
		const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
		// faceNetInputSize
		const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
		// handNetInputSize
		const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
		// producerType
		op::ProducerType producerType;
		std::string producerString;
		std::tie(producerType, producerString) = op::flagsToProducer(
			FLAGS_image_dir, FLAGS_video, FLAGS_ip_camera, FLAGS_camera, FLAGS_flir_camera, FLAGS_flir_camera_index);
		// poseModel
		const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
		// JSON saving
		if (!FLAGS_write_keypoint.empty())
			op::log("Flag `write_keypoint` is deprecated and will eventually be removed."
				" Please, use `write_json` instead.", op::Priority::Max);
		// keypointScale
		const auto keypointScale = op::flagsToScaleMode(FLAGS_keypoint_scale);
		// heatmaps to add
		const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
			FLAGS_heatmaps_add_PAFs);
		const auto heatMapScale = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
		// >1 camera view?
		const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
		// Enabling Google Logging
		const bool enableGoogleLogging = true;


		// Configure OpenPose wrapper					

		// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
		string modelDirToUse = modelDirPath + "/";
		const op::WrapperStructPose wrapperStructPose{
			!FLAGS_body_disable, netInputSize, outputSize, keypointScale, FLAGS_num_gpu, FLAGS_num_gpu_start,
			FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
			poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
			FLAGS_part_to_show, modelDirToUse, heatMapTypes, heatMapScale, FLAGS_part_candidates,
			(float)FLAGS_render_threshold, FLAGS_number_people_max, enableGoogleLogging };		
		// Face configuration (use op::WrapperStructFace{} to disable it)
		const op::WrapperStructFace wrapperStructFace{
			FLAGS_face, faceNetInputSize, op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
			(float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold };		
		// Hand configuration (use op::WrapperStructHand{} to disable it)
		const op::WrapperStructHand wrapperStructHand{
			FLAGS_hand, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range, FLAGS_hand_tracking,
			op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
			(float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };		
		// Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
		const op::WrapperStructExtra wrapperStructExtra{
			FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads };		
		// Producer (use default to disable any input)
		const op::WrapperStructInput wrapperStructInput{
			producerType, producerString, FLAGS_frame_first, FLAGS_frame_step, FLAGS_frame_last,
			FLAGS_process_real_time, FLAGS_frame_flip, FLAGS_frame_rotate, FLAGS_frames_repeat,
			cameraSize, FLAGS_camera_fps, FLAGS_camera_parameter_folder, !FLAGS_frame_keep_distortion,
			(unsigned int)FLAGS_3d_views };		
		// Consumer (comment or use default argument to disable any output)
		const auto displayMode = op::DisplayMode::NoDisplay;
		const bool guiVerbose = false;
		const bool fullScreen = false;
		const op::WrapperStructOutput wrapperStructOutput{
			displayMode, guiVerbose, fullScreen, FLAGS_write_keypoint,
			op::stringToDataFormat(FLAGS_write_keypoint_format), FLAGS_write_json, FLAGS_write_coco_json,
			FLAGS_write_coco_foot_json, FLAGS_write_coco_json_variant, FLAGS_write_images, FLAGS_write_images_format,
			FLAGS_write_video, FLAGS_camera_fps, FLAGS_write_heatmaps, FLAGS_write_heatmaps_format,
			FLAGS_write_video_adam, FLAGS_write_bvh, FLAGS_udp_host, FLAGS_udp_port };		



		/* setting up server */

		int iResult;
		SOCKET ListenSocket = INVALID_SOCKET;
		SOCKET ClientSocket = INVALID_SOCKET;

		iResult = initializeTcpServer(portToUse, &ListenSocket);
		if (iResult != 0)
		{
			return iResult;
		}

		/* end of setting up server */




		// TCP Winsock: accept one connection/client at a time
		// https://stackoverflow.com/questions/16686444/function-names-conflict-in-c
		op::log("Port: " + to_string(portToUse) + " is used.");
		op::log("Waiting for incoming socket...");
		while ((ClientSocket = accept(ListenSocket, NULL, NULL)))
		{			
			if (ClientSocket == INVALID_SOCKET)
			{
				printf("accept failed with error: %d\n", WSAGetLastError());
				/*closesocket(ListenSocket);
				WSACleanup();
				return 1;*/

				//closesocket(ClientSocket);
				continue;
			}

			const auto timerBegin = std::chrono::high_resolution_clock::now();


			// Initializing the user custom classes
			// GUI (Display)
			auto wUserOutput = std::make_shared<WUserOutput>();
			const auto workerOutputOnNewThread = true;

			op::WrapperT<std::vector<UserDatum>> *opWrapperT = new op::WrapperT<std::vector<UserDatum>>();
			constructOpenPoseWorkerWrapper
			(
				wrapperStructPose,
				wrapperStructFace,
				wrapperStructHand,
				wrapperStructExtra,
				wrapperStructInput,
				wrapperStructOutput,
				workerOutputOnNewThread,
				wUserOutput,
				opWrapperT
			);

			// https://docs.microsoft.com/en-us/cpp/cpp/how-to-create-and-use-shared-ptr-instances?view=vs-2017
			wUserOutput->setClientSocket(ClientSocket);
			wUserOutput->setIsPrintData(isPrintData);
			wUserOutput->setIsShowImage(isShowImage);

			// Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
			op::log("Starting thread(s)...", op::Priority::High);

			opWrapperT->exec();
			

			/* 
				!!!Important!!!
				As I called new op::WrapperT<std::vector<UserDatum>>() in every while loop,
				if I don't call ::delete, the memory usage will go up when a new connection is made,
				i.e. when the stuff inside the while() loop is executed once, 
				until it finally runs out of memory.
			*/
			::delete opWrapperT;
			opWrapperT = nullptr;


			// Measuring total time
			const auto now = std::chrono::high_resolution_clock::now();
			const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now - timerBegin).count()
				* 1e-9;
			const auto message = "OpenPose demo successfully finished. Total time: "
				+ std::to_string(totalTimeSec) + " seconds.";
			op::log(message, op::Priority::High);

			op::log("Port: " + to_string(portToUse) + " is used.");
			op::log("Waiting for incoming socket...");
		}

		closeDownTcpServer(ListenSocket, ClientSocket);

		std::cin.get();

		// Return successful message
		return 0;
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
		return -1;
	}
}

int main(int argc, char *argv[])
{
	// OpenPose
	// Parsing command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	// getting command line arguments

	// command line usage
	// --camera, --process_real_time and --keypoint_scale flag is for OpenPose
	// --keypoint_scale 3 to scale output keypoints in the range [0,1]
	// https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/demo_overview.md
	// isShowImage and isPrintData are boolean flag, 1 = true, 0 = false
	// e.g. 100Most-Interactive-Tree-OpenPose-Server-Video [TCP] 27156 isShowImage isPrintData --camera 0 --process_real_time --keypoint_scale 3
	string usageMsg = 
		"Usage: 100Most-Interactive-Tree-OpenPose-Server-Video modelDirPath tcpMsgDelimiter portToListen isShowImage isPrintData --camera 0 --process_real_time --keypoint_scale 3";
	if (argc != 6)
	{
		op::log(usageMsg);
		return 1;
	}
	string modelDirPath = string(argv[1]);
	string tcpMsgDelimiter = string(argv[2]);
	int portToUse = stoi(argv[3]);
	bool isShowImage = checkBoolCommandLineArgument(argv[4]);
	bool isPrintData = checkBoolCommandLineArgument(argv[5]);

	// Running tutorialApiCpp8
	return tutorialApiCpp8(modelDirPath, tcpMsgDelimiter, portToUse, isShowImage, isPrintData);
}

/* end of main functions */


/* WUserOutput class implementation */

// The W-classes can be implemented either as a template or as simple classes given
// that the user usually knows which kind of data he will move between the queues,
// in this case we assume a std::shared_ptr of a std::vector of UserDatum

// This worker will just read and return all the jpg files in a directory
void WUserOutput::setClientSocket(SOCKET clientSocket)
{
	_clientSocket = clientSocket;
}

void WUserOutput::setIsPrintData(bool isPrintData)
{
	_isPrintData = isPrintData;
}

void WUserOutput::setIsShowImage(bool isShowImage)
{
	_isShowImage = isShowImage;
}


int WUserOutput::sendData(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
{
	if (_clientSocket == INVALID_SOCKET)
	{
		op::log("Can't send data. Client socket is invalid.");
		return 1;
	}
	/*else
	{
		op::log("Client socket is valid. About to send data.");
	}*/

	const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
	string jsonPose = getSimplifiedJsonFromPoseKeyPoints(poseKeypoints);

	string msgToSend = jsonPose + _tcpMsgDelimiter;


	// send something
	int iSendResult;
	const char *sendbuf = msgToSend.c_str();
	int sendBufStrLen = (int)strlen(sendbuf);
	iSendResult = send(_clientSocket, sendbuf, sendBufStrLen, 0);
	if (iSendResult == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(_clientSocket);
		//WSACleanup();
		return 2;
	}

	if (_isPrintData)
	{
		op::log("Sent message: ");
		op::log(msgToSend);
	}

	return 0;
}

void WUserOutput::printData(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
{
	// Show in command line the resulting pose keypoints for body, face and hands
	op::log("\nKeypoints:");
	// Accesing each element of the keypoints
	const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
	op::log("Person pose keypoints:");
	for (auto person = 0; person < poseKeypoints.getSize(0); person++)
	{
		op::log("Person " + std::to_string(person) + " (x, y, score):");
		for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++)
		{
			std::string valueToPrint;
			for (auto xyscore = 0; xyscore < poseKeypoints.getSize(2); xyscore++)
			{
				valueToPrint += std::to_string(poseKeypoints[{person, bodyPart, xyscore}]) + " ";
			}
			op::log(valueToPrint);
		}
	}


	op::log(" ");
	// Alternative: just getting std::string equivalent
	op::log("Face keypoints: " + datumsPtr->at(0).faceKeypoints.toString());
	op::log("Left hand keypoints: " + datumsPtr->at(0).handKeypoints[0].toString());
	op::log("Right hand keypoints: " + datumsPtr->at(0).handKeypoints[1].toString());
	// Heatmaps
	const auto& poseHeatMaps = datumsPtr->at(0).poseHeatMaps;
	if (!poseHeatMaps.empty())
	{
		op::log("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", "
			+ std::to_string(poseHeatMaps.getSize(1)) + ", "
			+ std::to_string(poseHeatMaps.getSize(2)) + "]");
		const auto& faceHeatMaps = datumsPtr->at(0).faceHeatMaps;
		op::log("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", "
			+ std::to_string(faceHeatMaps.getSize(1)) + ", "
			+ std::to_string(faceHeatMaps.getSize(2)) + ", "
			+ std::to_string(faceHeatMaps.getSize(3)) + "]");
		const auto& handHeatMaps = datumsPtr->at(0).handHeatMaps;
		op::log("Left hand heatmaps size: [" + std::to_string(handHeatMaps[0].getSize(0)) + ", "
			+ std::to_string(handHeatMaps[0].getSize(1)) + ", "
			+ std::to_string(handHeatMaps[0].getSize(2)) + ", "
			+ std::to_string(handHeatMaps[0].getSize(3)) + "]");
		op::log("Right hand heatmaps size: [" + std::to_string(handHeatMaps[1].getSize(0)) + ", "
			+ std::to_string(handHeatMaps[1].getSize(1)) + ", "
			+ std::to_string(handHeatMaps[1].getSize(2)) + ", "
			+ std::to_string(handHeatMaps[1].getSize(3)) + "]");
	}
}

void WUserOutput::printNumOfPeople(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
{
	// Accesing each element of the keypoints
	const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
	int numOfPeople = poseKeypoints.getSize(0);
	op::log("People length: " + std::to_string(numOfPeople));
}


void WUserOutput::initializationOnThread() {}

void WUserOutput::workConsumer(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
{
	try
	{
		// User's displaying/saving/other processing here
		// datum.cvOutputData: rendered frame with pose or heatmaps
		// datum.poseKeypoints: Array<float> with the estimated pose
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			int iSendResult = this->sendData(datumsPtr);
			if (iSendResult != 0)
			{
				throw exception("Failed to send data: " + iSendResult);
			}

			if (_isPrintData)
			{
				printData(datumsPtr);
			}

			//printNumOfPeople(datumsPtr);

			// Display rendered output image
			if (_isShowImage)
			{
				cv::imshow("User worker GUI", datumsPtr->at(0).cvOutputData);
			}

			// Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
			const char key = (char)cv::waitKey(1);
			if (key == 27)  // ESC
				this->stop();
		}
	}
	catch (const std::exception& e)
	{
		this->stop();

		//op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
		op::log(e.what());
	}
}

/* end of WUserOutput class implementation */


/* function implementations */


bool checkBoolCommandLineArgument(char *arg)
{
	return string(arg) == "1";
}

/* Winsock server implementations */

int initializeTcpServer(int port, SOCKET *ListenSocket)
{
	// Complete Winsock Server Code
	// https://docs.microsoft.com/en-us/windows/desktop/winsock/complete-server-code#winsock-server-source-code
	WSADATA wsaData;
	int iResult;
	*ListenSocket = INVALID_SOCKET;
	struct addrinfo *result = NULL;
	struct addrinfo hints;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	// Resolve the server address and port	
	iResult = getaddrinfo(NULL, (to_string(port)).c_str(), &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	// Create a SOCKET for connecting to server
	*ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (*ListenSocket == INVALID_SOCKET) {
		printf("socket failed with error: %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return 1;
	}

	// Setup the TCP listening socket
	// https://stackoverflow.com/questions/16686444/function-names-conflict-in-c
	iResult = ::bind(*ListenSocket, result->ai_addr, (int)result->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
		printf("bind failed with error: %d\n", WSAGetLastError());
		freeaddrinfo(result);
		closesocket(*ListenSocket);
		WSACleanup();
		return 1;
	}

	freeaddrinfo(result);

	iResult = listen(*ListenSocket, SOMAXCONN);
	if (iResult == SOCKET_ERROR) {
		printf("listen failed with error: %d\n", WSAGetLastError());
		closesocket(*ListenSocket);
		WSACleanup();
		return 1;
	}

	return iResult;
}

void closeDownTcpServer(SOCKET ListenSocket, SOCKET ClientSocket)
{
	closesocket(ListenSocket);
	closesocket(ClientSocket);
	WSACleanup();
}

/* end of Winsock server implementations */


/* openpose implementations */

// https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md
// Result for BODY_25 (25 body parts consisting of COCO + foot)
const map<unsigned int, string> POSE_BODY_25_BODY_PARTS{
	{0,  "Nose"},
	{1,  "Neck"},
	{2,  "RShoulder"},
	{3,  "RElbow"},
	{4,  "RWrist"},
	{5,  "LShoulder"},
	{6,  "LElbow"},
	{7,  "LWrist"},
	{8,  "MidHip"},
	{9,  "RHip"},
	{10, "RKnee"},
	{11, "RAnkle"},
	{12, "LHip"},
	{13, "LKnee"},
	{14, "LAnkle"},
	{15, "REye"},
	{16, "LEye"},
	{17, "REar"},
	{18, "LEar"},
	{19, "LBigToe"},
	{20, "LSmallToe"},
	{21, "LHeel"},
	{22, "RBigToe"},
	{23, "RSmallToe"},
	{24, "RHeel"},
	{25, "Background"}
};

string getSimplifiedJsonFromPoseKeyPoints(op::Array<float> poseKeypoints)
{
	string jsonResult = "{\"people\":[";

	for (auto person = 0; person < poseKeypoints.getSize(0); person++)
	{
		// start of body
		jsonResult += "{";

		for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++)
		{
			// start of body part
			jsonResult += "\"" + POSE_BODY_25_BODY_PARTS.at(bodyPart) +
				"\":[";

			// poseKeypoints.getSize(2) - 1 because we ignore the 3rd coordinate for each pose key point, which is the confidence score
			for (auto xyscore = 0; xyscore < poseKeypoints.getSize(2) - 1; xyscore++)
			{
				//cout << i * numOfPoseKeyPointsPerBody + j * numOfNumbersPerPoseKeyPoint + k << endl;
				jsonResult += to_string(poseKeypoints[{person, bodyPart, xyscore}]) + ",";
			}

			// end of body part
			jsonResult = jsonResult.substr(0, jsonResult.length() - 1);
			jsonResult += "],";
		}

		// end of body
		jsonResult = jsonResult.substr(0, jsonResult.length() - 1);
		jsonResult += "},";
	}

	if (jsonResult != "")
	{
		jsonResult = jsonResult.substr(0, jsonResult.length() - 1);
		jsonResult += "]}";
	}

	return jsonResult;
}

void constructOpenPoseWorkerWrapper
(
	op::WrapperStructPose wrapperStructPose,
	op::WrapperStructFace wrapperStructFace,
	op::WrapperStructHand wrapperStructHand,
	op::WrapperStructExtra wrapperStructExtra,
	op::WrapperStructInput wrapperStructInput,
	op::WrapperStructOutput wrapperStructOutput,
	bool workerOutputOnNewThread,
	std::shared_ptr<WUserOutput> wUserOutput,
	op::WrapperT<std::vector<UserDatum>> *opWrapperT
)
{
	// OpenPose wrapper
	op::log("Configuring OpenPose...", op::Priority::High);

	// Add custom processing
	opWrapperT->setWorker(op::WorkerType::Output, wUserOutput, workerOutputOnNewThread);

	opWrapperT->configure(wrapperStructPose);
	opWrapperT->configure(wrapperStructFace);
	opWrapperT->configure(wrapperStructHand);
	opWrapperT->configure(wrapperStructExtra);
	opWrapperT->configure(wrapperStructInput);
	opWrapperT->configure(wrapperStructOutput);

	// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
	if (FLAGS_disable_multi_thread)
		opWrapperT->disableMultiThreading();
}

/* end of openpose implementations */

/* end of function implementations */