#include "stdafx.h"

#include "GlobalAppState.h"
#include "GlobalBundlingState.h"
#include "MatrixConversion.h"
#include "PoseHelper.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <string>

#include <conio.h>
#include "CustomSensor.h"



using namespace std;
using namespace web::http;
using namespace web::http::experimental::listener;
using namespace web;
using namespace utility::conversions;


CustomSensor::CustomSensor()
{
	//아이폰 깊이 센서가 멍청해서 상대값임.
	//30장씩만 부분 스켄하고 꺼버리게
	heebin_counter = 0;

	listener = new http_listener(U("http://192.168.0.21:3000"));        //Server URL, Port 지정, 내주소 적어

	//TODO : 센서값을 잘 넣어라
	long m_depthWidth = 640;
	long m_depthHeight = 480;
	long m_colorWidth = 640;
	long m_colorHeight = 480;
	
	RGBDSensor::init(
		m_depthWidth,
		m_depthHeight,
		m_colorWidth,
		m_colorHeight,
		1);

	//아이폰
	//float v00=596.432, v11= 596.432, v02=320.0, v12=240.0;

	float v00 = 583.0, v11 = 583.0, v02 = 320.0, v12 = 240.0;

	//모두 float넣기
	initializeDepthIntrinsics(
		v00,
		v11,
		v02,
		v12);

	//모두 float넣기
	initializeColorIntrinsics(
		v00,
		v11,
		v02,
		v12);

	mat4f m = mat4f::identity();

	initializeDepthExtrinsics(m);
	initializeColorExtrinsics(m);

}

CustomSensor::~CustomSensor()
{

}


void CustomSensor::createFirstConnected()
{
	//TODO
	//서버에서 값 받는거 연결을 설정하시오
	//m_receiving==1이면 문제없음, m_receiving==-1 이면 망한거


	//관리자 모드로 vs를 실행해야 켜지는 경향을 확인함



	try
	{
		listener->open().then([]() {
			cout << "\n start!!\n";
			}).wait();    //Server open

	}
	catch (const std::exception& e)
	{
		m_receiving = -1;
		printf("Error exception:%s\n", e.what());
	}



	listener->support(methods::GET, [](http_request req) {                        //support() 함수를 통해 GET방식 지정
		req.reply(status_codes::OK, U("hello world"));                        //Lamda방식으로 간단하게 구현.
		});

	listener->support(methods::POST, [this](http_request req) {                        //support() 함수를 통해 POST방식 지정

		req.extract_json().then([&req, this](json::value val) {
			if (val.is_null()) {
				req.reply(status_codes::BadRequest, U("No object in post data."));
			}
			else {

				//std::vector<unsigned char> rawDepthVec = from_base64(val[U("depth")].as_string());
				//std::vector<unsigned char> rawColorVec = from_base64(val[U("color")].as_string());

				cv::Mat img_dep = cv::imdecode(from_base64(val[U("depth")].as_string()), cv::IMREAD_ANYDEPTH);
				cv::Mat img_col = cv::imdecode(from_base64(val[U("color")].as_string()), cv::IMREAD_COLOR);
				cv::cvtColor(img_col, img_col, cv::COLOR_BGR2RGB);

				/* mat.type()의 리턴
				+--------+----+----+----+----+------+------+------+------+
				|        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
				+--------+----+----+----+----+------+------+------+------+
				| CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
				| CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
				| CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
				| CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
				| CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
				| CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
				| CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
				+--------+----+----+----+----+------+------+------+------+				
				*/
				

				/*
				cout << img_dep.type(); //0
				cout << img_col.type(); //16

				cout << img_dep.size(); // 1920 * 1440
				cout << img_col.size(); // 1920 * 1440

				*/
				//img_dep.convertTo(img_dep, CV_16U);
				//cout << img_dep.type(); //2

				//cv::resize(img_dep, img_dep, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
				//cv::resize(img_col, img_col, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

				//cv::imshow("dd", img_dep);
				//cv::imshow("cc", img_col);
				//cv::waitKey(1);

				
				//새로운 프레임 생성
				ml::SensorData::RGBDFrameCacheRead::FrameState* frameState
					= new ml::SensorData::RGBDFrameCacheRead::FrameState();

				int width = 640, height = 480;

				//cout << img_col.rows << img_col.cols << endl;
				//컬러 데이터 입력
				frameState->m_colorFrame = (vec3uc*)malloc(width * height * sizeof(vec3uc));
				//뎁스 데이터 입력
				frameState->m_depthFrame = (unsigned short*)malloc(width * height * sizeof(unsigned short)); // img_dep.data;
				
				//std::vector<unsigned char>::iterator colorIter = rawColorVec.begin();
				//std::vector<unsigned char>::iterator depthIter = rawDepthVec.begin();
				std::vector<cv::Mat> three_channels;
				cv::split(img_col, three_channels);

				for (int i = 0; i < height; i++) {
					for (int j = 0; j < width; j++) {

						/*
						unsigned char blue = *colorIter;
						colorIter++;
						unsigned char green = *colorIter;
						colorIter++;
						unsigned char red = *colorIter;
						colorIter++;
						unsigned char alpha = *colorIter;
						colorIter++;
						*/


						

						(frameState->m_colorFrame)[i * width + j].x =three_channels[0].at<uchar>(i, j);//red
						(frameState->m_colorFrame)[i * width + j].y =three_channels[1].at<uchar>(i, j);//green
						(frameState->m_colorFrame)[i * width + j].z =three_channels[2].at<uchar>(i, j);//blue

						
						unsigned short temp = img_dep.at<unsigned short>(i, j);

						unsigned short front_val = temp >> 8;
						unsigned short back_val = temp << 8;
						

						(frameState->m_depthFrame)[i * width + j] = back_val | front_val;



						/*
						unsigned short front = (unsigned short)*depthIter;
						depthIter++;
						unsigned short back = (unsigned short)*depthIter;
						depthIter++;
						frameState->m_depthFrame[i * width + j] = ((back << 8) & 0xFF00) | (front & 0xFF);
						*/

					}
				}

				

				frameState->m_bIsReady = true;

				this->m_data.push(frameState);


				//cout << "\r received frame, count : " <<this->m_data.size();

				
				req.reply(status_codes::OK, U("ok"));

			}
			}).wait();
		});



	
	/* 
	직접 데이터 로딩 실험 성공

	for (int i = 0; i < 300; i++) {
		
		std::string depth_location;
		depth_location.resize(100);
		sprintf((char*) depth_location.c_str(), "C:\\scan\\BundleFusion-master\\data\\out\\depth\\%d.png", i);
		
		std::string color_location;
		color_location.resize(100);
		sprintf((char*)color_location.c_str(), "C:\\scan\\BundleFusion-master\\data\\out\\color\\%d.jpg", i);


		cv::Mat img_dep = cv::imread(depth_location, cv::IMREAD_ANYDEPTH);
		cv::Mat img_col = cv::imread(color_location, cv::IMREAD_COLOR);
		

		img_dep.convertTo(img_dep, CV_16U);
		

		cv::resize(img_dep, img_dep, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
		cv::resize(img_col, img_col, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

		//cv::imshow("dd", img_dep);
		//cv::imshow("cc", img_col);
		//cv::waitKey(0);

		
		//새로운 프레임 생성
		ml::SensorData::RGBDFrameCacheRead::FrameState* frameState
			= new ml::SensorData::RGBDFrameCacheRead::FrameState();

		//컬러 데이터 입력
		frameState->m_colorFrame = (vec3uc*)malloc(img_col.rows * img_col.cols * sizeof(vec3uc));
		//뎁스 데이터 입력
		frameState->m_depthFrame = (unsigned short*)malloc(img_dep.rows * img_dep.cols * sizeof(unsigned short)); // img_dep.data;


		std::vector<cv::Mat> three_channels;
		cv::split(img_col, three_channels);

		for (int i = 0; i < img_dep.rows; i++) {
			for (int j = 0; j < img_dep.cols; j++) {
				(frameState->m_colorFrame)[i * img_col.cols + j].x = three_channels[0].at<uchar>(i, j);
				(frameState->m_colorFrame)[i * img_col.cols + j].y = three_channels[1].at<uchar>(i, j);
				(frameState->m_colorFrame)[i * img_col.cols + j].z = three_channels[2].at<uchar>(i, j);

				(frameState->m_depthFrame)[i * img_dep.cols + j] = img_dep.at<unsigned short>(i, j);

			}
		}
		
		frameState->m_bIsReady = true;
		this->m_data.push(frameState);

	}

	*/

	




	m_receiving = 1;
}

bool CustomSensor::processDepth()
{
	
	//데이터 로딩 테스트를 위함
	//직접 데이터 로딩 실험 성공
	//중간에 읽다가 9 누르고 esc 누르면 꺼지니까 그렇게 해
	/*
	if (m_data.empty()) {
		m_receiving = -1;
	}
	*/

	heebin_counter++;
	//heebin_counter > 30 ||
	if (GlobalAppState::get().s_playData && m_receiving==-1)
	{
		GlobalAppState::get().s_playData = false;
		//std::cout << "binary dump sequence complete - press space to run again" << std::endl;
		stopReceivingFrames();
		std::cout << "data receive complete - stopped receiving frames" << std::endl;
		
	}

	if (GlobalAppState::get().s_playData) {

		while (m_data.empty()) {
			//cout << "\rwait for data";
		}
		ml::SensorData::RGBDFrameCacheRead::FrameState* frameState
			= m_data.front();

		m_data.pop();

		/*
		* 버퍼에 너무 많이 데이터가 들어오면 stride 식으로 조금 죽여야 한다고 생각했어
		* 사실 이러면 안될 것 같아. 고민중
		while(m_data.size() > 10){
			m_data.pop();
		}
		*/



		//이것이 리턴하는게 실제 깊이 지도 값을 저장 할 곳이다.
		//이미지 가로 세로 크기만큼의 float 배열이다.
		//상위 클레스에서 가져오는 포인터이다.
		////상위 클래스에 있는 벡터 td::vector<float*> m_depthFloat;에 깊이 맵들이 들어있다.
		float* depth = getDepthFloat();

		//m_depthShift에 적합한 값을 넣어라
		//데이터는 1000이다. 근데 우린 1000 곱해야 할듯
		float m_depthShift = 1000.0f;
		//getDepthWidth, getDepthHeight은 부모의 attr. 초기 연결시 설정완료
		//m_depthFrame 배열은 2차원 깂이 0~65535으로 들어간 놈이다. 근데 사실 선언은 1차원인 배열의 포인터인 그런 모양
		for (unsigned int i = 0; i < getDepthWidth() * getDepthHeight(); i++) {
			if (frameState->m_depthFrame[i] == 0) depth[i] = -std::numeric_limits<float>::infinity();
			else depth[i] = (float)frameState->m_depthFrame[i] / m_depthShift;
		}


		//m_colorFrame 배열은 2차원 깂이 0~255 3개로 들어간 이미지 한장이다. 근데 사실 1차원인 그런 모양
		//컬러 데이터가 들어가고 uchar 3차원 벡터(여기서 정의된) 벡터이다.
		for (unsigned int i = 0; i < getColorWidth() * getColorHeight(); i++) {
			// 부모 클래스에서 알아서 초기화함, 부모의 attr.
			//vec4uc* m_colorRGBX = new vec4uc[m_colorWidth*m_colorHeight];
			// 현재 프레임의 컬러 정보를 가지고 있다.
			m_colorRGBX[i] = vec4uc(frameState->m_colorFrame[i]);
		}
		
		frameState->free();

		//RingbufIdx는 상위 클래스의 프레임 넘버를 이야기하는 것이다.
		//이를 다음으로 넘기는 작업을 수행한다.
		incrementRingbufIdx();

		return true;
	}
	else {
		return false;
	}
}

/*
ml::mat4f CustomSensor::getRigidTransform(int offset) const
{
	unsigned int idx = m_currFrame - 1 + offset;
	if (idx >= m_sensorData->m_frames.size()) throw MLIB_EXCEPTION("invalid trajectory index " + std::to_string(idx));
	const mat4f& transform = m_sensorData->m_frames[idx].getCameraToWorld();
	return transform;
	//return m_data.m_trajectory[idx];
}
*/



/*
void CustomSensor::saveToFile(const std::string& filename, const std::vector<mat4f>& trajectory) const
{
	const unsigned int numFrames = (unsigned int)std::min(trajectory.size(), m_sensorData->m_frames.size());
	for (unsigned int i = 0; i < numFrames; i++) {
		m_sensorData->m_frames[i].setCameraToWorld(trajectory[i]);
	}
	//fill in rest invalid
	mat4f invalidTransform; invalidTransform.setZero(-std::numeric_limits<float>::infinity());
	for (unsigned int i = (unsigned int)trajectory.size(); i < m_sensorData->m_frames.size(); i++) {
		m_sensorData->m_frames[i].setCameraToWorld(invalidTransform);
	}

	m_sensorData->saveToFile(filename);
}
*/

/*
우리가 데이터로 준건 참조용 궤적이고
얘네도 따로 궤적을 계산한다.
궤적을 누적할 필요는 없어보여서 제거토록 했다.

데이터파일에 있는 포즈 값을 계산된 값과 비교한다.

데이터파일에 있는 포즈 값은 GT에 불과하다.
처리할때 안쓴다. 쾅쾅

void CustomSensor::evaluateTrajectory(const std::vector<mat4f>& trajectory) const
{
	std::vector<mat4f> referenceTrajectory;
	for (const auto& f : m_sensorData->m_frames) referenceTrajectory.push_back(f.getCameraToWorld());
	const size_t numTransforms = std::min(trajectory.size(), referenceTrajectory.size());
	// make sure reference trajectory starts at identity
	mat4f offset = referenceTrajectory.front().getInverse();
	for (unsigned int i = 0; i < referenceTrajectory.size(); i++) referenceTrajectory[i] = offset * referenceTrajectory[i];

	const auto transErr = PoseHelper::evaluateAteRmse(trajectory, referenceTrajectory);
	std::cout << "*********************************" << std::endl;
	std::cout << "ate rmse = " << transErr.first << ", " << transErr.second << std::endl;
	std::cout << "*********************************" << std::endl;
	//{
	//	std::vector<mat4f> optTrajectory = trajectory;
	//	optTrajectory.resize(numTransforms);
	//	referenceTrajectory.resize(numTransforms);
	//	PoseHelper::saveToPoseFile("debug/opt.txt", optTrajectory);
	//	PoseHelper::saveToPoseFile("debug/gt.txt", referenceTrajectory);
	//}
}
*/

/*
void CustomSensor::getTrajectory(std::vector<mat4f>& trajectory) const
{
	trajectory.clear();
	if (!m_sensorData) return;
	trajectory.resize(m_sensorData->m_frames.size());
	for (unsigned int f = 0; f < m_sensorData->m_frames.size(); f++) {
		trajectory[f] = m_sensorData->m_frames[f].getCameraToWorld();
		if (trajectory[f][0] == -std::numeric_limits<float>::infinity())
			throw MLIB_EXCEPTION("ERROR invalid transform in reference trajectory");
	}
}
*/

//heebin
//추가 노트, 프레임 하나를 가지고 있는 구조체에 대하여

/*
* 한 순간의 정보를 가지고 있는 구조체, 매번 쓰고 직접 날려(delete 해)줄 것.

struct FrameState {
		FrameState() {
			m_bIsReady = false;
			m_colorFrame = NULL;
			m_depthFrame = NULL;
			m_timeStampDepth = 0;
			m_timeStampColor = 0;
		}
		~FrameState() {
			//NEEDS MANUAL FREE
		}
		void free() {
			if (m_colorFrame) {
				std::free(m_colorFrame);
				m_colorFrame = NULL;
			}
			if (m_depthFrame) {
				std::free(m_depthFrame);
				m_depthFrame = NULL;
			}
			m_timeStampDepth = 0;
			m_timeStampColor = 0;
		}
		bool			m_bIsReady;
		vec3uc*			m_colorFrame;
		unsigned short*	m_depthFrame;
		UINT64			m_timeStampDepth;
		UINT64			m_timeStampColor;
	};



//위의 구조체를 new 하는 방법
//ml::SensorData::RGBDFrameCacheRead::FrameState *frameState = new ml::SensorData::RGBDFrameCacheRead::FrameState();

//이미지 한장이 들어가있는 것이지요.

bool			m_bIsReady;
vec3uc* m_colorFrame;
//색 하나당 8비트 색을 요구하고 있음
unsigned short* m_depthFrame;
//16비트 깊이 맵을 요구하는 바임
UINT64			m_timeStampDepth;
UINT64			m_timeStampColor;

//위의 것들을 넣어야해




* 초기화 하는 방법
	fs.m_colorFrame = sensorData->decompressColorAlloc(frame);
	fs.m_depthFrame = sensorData->decompressDepthAlloc(frame);

 * 타임스탬프 값 넣기
	fs.m_timeStampDepth = frame.m_timeStampDepth;
	fs.m_timeStampColor = frame.m_timeStampColor;
	준비되면 TRUE
	fs.m_bIsReady = true;

*/
