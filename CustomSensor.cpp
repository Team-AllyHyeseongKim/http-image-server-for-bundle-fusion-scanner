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
	//������ ���� ������ ��û�ؼ� ��밪��.
	//30�徿�� �κ� �����ϰ� ��������
	heebin_counter = 0;

	listener = new http_listener(U("http://192.168.0.21:3000"));        //Server URL, Port ����, ���ּ� ����

	//TODO : �������� �� �־��
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

	//������
	//float v00=596.432, v11= 596.432, v02=320.0, v12=240.0;

	float v00 = 583.0, v11 = 583.0, v02 = 320.0, v12 = 240.0;

	//��� float�ֱ�
	initializeDepthIntrinsics(
		v00,
		v11,
		v02,
		v12);

	//��� float�ֱ�
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
	//�������� �� �޴°� ������ �����Ͻÿ�
	//m_receiving==1�̸� ��������, m_receiving==-1 �̸� ���Ѱ�


	//������ ���� vs�� �����ؾ� ������ ������ Ȯ����



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



	listener->support(methods::GET, [](http_request req) {                        //support() �Լ��� ���� GET��� ����
		req.reply(status_codes::OK, U("hello world"));                        //Lamda������� �����ϰ� ����.
		});

	listener->support(methods::POST, [this](http_request req) {                        //support() �Լ��� ���� POST��� ����

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

				/* mat.type()�� ����
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

				
				//���ο� ������ ����
				ml::SensorData::RGBDFrameCacheRead::FrameState* frameState
					= new ml::SensorData::RGBDFrameCacheRead::FrameState();

				int width = 640, height = 480;

				//cout << img_col.rows << img_col.cols << endl;
				//�÷� ������ �Է�
				frameState->m_colorFrame = (vec3uc*)malloc(width * height * sizeof(vec3uc));
				//���� ������ �Է�
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
	���� ������ �ε� ���� ����

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

		
		//���ο� ������ ����
		ml::SensorData::RGBDFrameCacheRead::FrameState* frameState
			= new ml::SensorData::RGBDFrameCacheRead::FrameState();

		//�÷� ������ �Է�
		frameState->m_colorFrame = (vec3uc*)malloc(img_col.rows * img_col.cols * sizeof(vec3uc));
		//���� ������ �Է�
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
	
	//������ �ε� �׽�Ʈ�� ����
	//���� ������ �ε� ���� ����
	//�߰��� �дٰ� 9 ������ esc ������ �����ϱ� �׷��� ��
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
		* ���ۿ� �ʹ� ���� �����Ͱ� ������ stride ������ ���� �׿��� �Ѵٰ� �����߾�
		* ��� �̷��� �ȵ� �� ����. �����
		while(m_data.size() > 10){
			m_data.pop();
		}
		*/



		//�̰��� �����ϴ°� ���� ���� ���� ���� ���� �� ���̴�.
		//�̹��� ���� ���� ũ�⸸ŭ�� float �迭�̴�.
		//���� Ŭ�������� �������� �������̴�.
		////���� Ŭ������ �ִ� ���� td::vector<float*> m_depthFloat;�� ���� �ʵ��� ����ִ�.
		float* depth = getDepthFloat();

		//m_depthShift�� ������ ���� �־��
		//�����ʹ� 1000�̴�. �ٵ� �츰 1000 ���ؾ� �ҵ�
		float m_depthShift = 1000.0f;
		//getDepthWidth, getDepthHeight�� �θ��� attr. �ʱ� ����� �����Ϸ�
		//m_depthFrame �迭�� 2���� ���� 0~65535���� �� ���̴�. �ٵ� ��� ������ 1������ �迭�� �������� �׷� ���
		for (unsigned int i = 0; i < getDepthWidth() * getDepthHeight(); i++) {
			if (frameState->m_depthFrame[i] == 0) depth[i] = -std::numeric_limits<float>::infinity();
			else depth[i] = (float)frameState->m_depthFrame[i] / m_depthShift;
		}


		//m_colorFrame �迭�� 2���� ���� 0~255 3���� �� �̹��� �����̴�. �ٵ� ��� 1������ �׷� ���
		//�÷� �����Ͱ� ���� uchar 3���� ����(���⼭ ���ǵ�) �����̴�.
		for (unsigned int i = 0; i < getColorWidth() * getColorHeight(); i++) {
			// �θ� Ŭ�������� �˾Ƽ� �ʱ�ȭ��, �θ��� attr.
			//vec4uc* m_colorRGBX = new vec4uc[m_colorWidth*m_colorHeight];
			// ���� �������� �÷� ������ ������ �ִ�.
			m_colorRGBX[i] = vec4uc(frameState->m_colorFrame[i]);
		}
		
		frameState->free();

		//RingbufIdx�� ���� Ŭ������ ������ �ѹ��� �̾߱��ϴ� ���̴�.
		//�̸� �������� �ѱ�� �۾��� �����Ѵ�.
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
�츮�� �����ͷ� �ذ� ������ �����̰�
��׵� ���� ������ ����Ѵ�.
������ ������ �ʿ�� ������� ������� �ߴ�.

���������Ͽ� �ִ� ���� ���� ���� ���� ���Ѵ�.

���������Ͽ� �ִ� ���� ���� GT�� �Ұ��ϴ�.
ó���Ҷ� �Ⱦ���. ����

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
//�߰� ��Ʈ, ������ �ϳ��� ������ �ִ� ����ü�� ���Ͽ�

/*
* �� ������ ������ ������ �ִ� ����ü, �Ź� ���� ���� ����(delete ��)�� ��.

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



//���� ����ü�� new �ϴ� ���
//ml::SensorData::RGBDFrameCacheRead::FrameState *frameState = new ml::SensorData::RGBDFrameCacheRead::FrameState();

//�̹��� ������ ���ִ� ��������.

bool			m_bIsReady;
vec3uc* m_colorFrame;
//�� �ϳ��� 8��Ʈ ���� �䱸�ϰ� ����
unsigned short* m_depthFrame;
//16��Ʈ ���� ���� �䱸�ϴ� ����
UINT64			m_timeStampDepth;
UINT64			m_timeStampColor;

//���� �͵��� �־����




* �ʱ�ȭ �ϴ� ���
	fs.m_colorFrame = sensorData->decompressColorAlloc(frame);
	fs.m_depthFrame = sensorData->decompressDepthAlloc(frame);

 * Ÿ�ӽ����� �� �ֱ�
	fs.m_timeStampDepth = frame.m_timeStampDepth;
	fs.m_timeStampColor = frame.m_timeStampColor;
	�غ�Ǹ� TRUE
	fs.m_bIsReady = true;

*/
