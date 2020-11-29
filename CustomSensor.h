#pragma once


/************************************************************************/
/* Reads sensor data files from .sens files                            */
/************************************************************************/

#include "GlobalAppState.h"
#include "RGBDSensor.h"
#include "stdafx.h"

#include<core.hpp>
#include<highgui.hpp>
#include<imgproc.hpp>
#include <imgcodecs.hpp>
#include <imgcodecs.hpp>

#include<iostream>
#include<fstream>
#include<sstream>
#include<string>

#include <cpprest/http_listener.h>
#include <cpprest/json.h>

using namespace std;
using namespace web::http;
using namespace web::http::experimental::listener;
using namespace web;
using namespace utility::conversions;


//namespace ml {
//	class SensorData;
//	class RGBDFrameCacheRead;
//}

class CustomSensor : public RGBDSensor
{
public:

	//! Constructor
	CustomSensor();

	//! Destructor; releases allocated ressources
	~CustomSensor();

	//! initializes the sensor
	void createFirstConnected();

	//! reads the next depth frame
	bool processDepth();


	bool processColor() {
		//everything done in process depth, since order is relevant (color must be read first)
		//읽는 순서가 중요하기 때문에, processDepth함수에서 다 해버림 
		return true;
	}

	std::string getSensorName() const {
		return "CustomSensor";
	}

	void stopReceivingFrames() { 

		listener->close();
		m_bIsReceivingFrames = false; 
	
	}

	//void evaluateTrajectory(const std::vector<mat4f>& trajectory) const;

private:
	int heebin_counter;
	int m_receiving;
	http_listener* listener;
	std::queue<ml::SensorData::RGBDFrameCacheRead::FrameState*> m_data;

};

