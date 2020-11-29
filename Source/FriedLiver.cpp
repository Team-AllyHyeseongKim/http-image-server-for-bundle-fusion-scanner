#include "stdafx.h"
#include "FriedLiver.h"







RGBDSensor* getRGBDSensor()
{
	static RGBDSensor* g_sensor = NULL;
	if (g_sensor != NULL)	return g_sensor;
	if (GlobalAppState::get().s_sensorIdx == 8) {
		g_sensor = new CustomSensor; // new SensorDataReader; // 
		return g_sensor;
	}

	throw MLIB_EXCEPTION("unkown sensor id " + std::to_string(GlobalAppState::get().s_sensorIdx));
	return NULL;
}


RGBDSensor* g_RGBDSensor = NULL;
CUDAImageManager* g_imageManager = NULL;
OnlineBundler* g_bundler = NULL;



void bundlingOptimization() {
	g_bundler->process(GlobalBundlingState::get().s_numLocalNonLinIterations, GlobalBundlingState::get().s_numLocalLinIterations,
		GlobalBundlingState::get().s_numGlobalNonLinIterations, GlobalBundlingState::get().s_numGlobalLinIterations);
	//g_bundler->resetDEBUG(false, false); // for no opt
}

void bundlingOptimizationThreadFunc() {

	DualGPU::get().setDevice(DualGPU::DEVICE_BUNDLING);

	bundlingOptimization();
}

void bundlingThreadFunc() {
	assert(g_RGBDSensor && g_imageManager);
	DualGPU::get().setDevice(DualGPU::DEVICE_BUNDLING);
	g_bundler = new OnlineBundler(g_RGBDSensor, g_imageManager);

	std::thread tOpt;

	while (1) {
		// opt
		//heebin
		//여기서 10개씩 묶어서 처리함
		

		if (g_RGBDSensor->isReceivingFrames()) {
			/*if (g_bundler->getCurrProcessedFrame() == 21) {
				StopScanningAndExit(true);
			}*/
			if (g_bundler->getCurrProcessedFrame() % 10 == 0) { // stop solve
				if (tOpt.joinable()) {
					tOpt.join();
				}
			}
			if (g_bundler->getCurrProcessedFrame() % 10 == 1) { // start solve
				MLIB_ASSERT(!tOpt.joinable());
				tOpt = std::thread(bundlingOptimizationThreadFunc);
			}
		}
		else { // stop then start solve
			if (tOpt.joinable()) {
				tOpt.join();
			}
			tOpt = std::thread(bundlingOptimizationThreadFunc);
		}
		//wait for a new input frame (LOCK IMAGE MANAGER)
		ConditionManager::lockImageManagerFrameReady(ConditionManager::Bundling);
		while (!g_imageManager->hasBundlingFrameRdy()) {
			ConditionManager::waitImageManagerFrameReady(ConditionManager::Bundling);
		}
		{
			ConditionManager::lockBundlerProcessedInput(ConditionManager::Bundling);
			while (g_bundler->hasProcssedInputFrame()) { //wait until depth sensing has confirmed the last one (WAITING THAT DEPTH SENSING RELEASES ITS LOCK)
				ConditionManager::waitBundlerProcessedInput(ConditionManager::Bundling);
			}
			{
				if (g_bundler->getExitBundlingThread()) {
					if (tOpt.joinable()) {
						tOpt.join();
					}
					ConditionManager::release(ConditionManager::Bundling);
					break;
				}
				g_bundler->processInput();						//perform sift and whatever
			}
			g_bundler->setProcessedInputFrame();			//let depth sensing know we have a frame (UNLOCK BUNDLING)
			ConditionManager::unlockAndNotifyBundlerProcessedInput(ConditionManager::Bundling);
		}
		g_imageManager->confirmRdyBundlingFrame();		//here it's processing with a new input frame  (GIVE DEPTH SENSING THE POSSIBLITY TO LOCK IF IT WANTS)
		ConditionManager::unlockAndNotifyImageManagerFrameReady(ConditionManager::Bundling);

		if (g_bundler->getExitBundlingThread()) {
			ConditionManager::release(ConditionManager::Bundling);
			break;
		}
	}
}


int main(int argc, char** argv)
{


	// Enable run-time memory check for debug builds.
#if defined(DEBUG) | defined(_DEBUG)
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	//_CrtSetBreakAlloc(15453);
#endif 

	try {
		//Read the global app state
		std::string fileNameDescGlobalApp = "zParametersDefault.txt";
		ParameterFile parameterFileGlobalApp(fileNameDescGlobalApp);
		GlobalAppState::getInstance().readMembers(parameterFileGlobalApp);

		//Read the global camera tracking state
		std::string fileNameDescGlobalBundling = "zParametersBundlingDefault.txt";
		ParameterFile parameterFileGlobalBundling(fileNameDescGlobalBundling);
		GlobalBundlingState::getInstance().readMembers(parameterFileGlobalBundling);

		DualGPU& dualGPU = DualGPU::get();	//needs to be called to initialize devices
		dualGPU.setDevice(DualGPU::DEVICE_RECONSTRUCTION);	//main gpu
		ConditionManager::init();

		g_RGBDSensor = getRGBDSensor();

		//init the input RGBD sensor
		if (g_RGBDSensor == NULL) throw MLIB_EXCEPTION("No RGBD sensor specified");
		g_RGBDSensor->createFirstConnected();


		g_imageManager = new CUDAImageManager(GlobalAppState::get().s_integrationWidth, GlobalAppState::get().s_integrationHeight,
			GlobalBundlingState::get().s_widthSIFT, GlobalBundlingState::get().s_heightSIFT, g_RGBDSensor, false);
#ifdef RUN_MULTITHREADED
		std::thread bundlingThread(bundlingThreadFunc);
		//waiting until bundler is initialized
		while (!g_bundler)	Sleep(0);
#else
		g_bundler = new OnlineBundler(g_RGBDSensor, g_imageManager);
#endif

		dualGPU.setDevice(DualGPU::DEVICE_RECONSTRUCTION);	//main gpu

		//start depthSensing render loop
		startDepthSensing(g_bundler, getRGBDSensor(), g_imageManager);

		//TimingLog::printAllTimings();
		//g_bundler->saveGlobalSiftManagerAndCacheToFile("debug/global");
		//if (GlobalBundlingState::get().s_recordSolverConvergence) g_bundler->saveConvergence("convergence.txt");
		//g_bundler->saveCompleteTrajectory("trajectory.bin");
		//g_bundler->saveSiftTrajectory("siftTrajectory.bin");
		//g_bundler->saveIntegrateTrajectory("intTrajectory.bin");
		//g_bundler->saveLogsToFile();

#ifdef RUN_MULTITHREADED 
		g_bundler->exitBundlingThread();

		g_imageManager->setBundlingFrameRdy();			//release all bundling locks
		g_bundler->confirmProcessedInputFrame();		//release all bundling locks
		ConditionManager::release(ConditionManager::Recon); // release bundling locks

		if (bundlingThread.joinable())	bundlingThread.join();	//wait for the bundling thread to return;
#endif
		SAFE_DELETE(g_bundler);
		SAFE_DELETE(g_imageManager);

		//ConditionManager::DEBUGRELEASE();

		//this is a bit of a hack due to a bug in std::thread (a static object cannot join if the main thread exists)
		auto* s = getRGBDSensor();
		SAFE_DELETE(s);

		std::cout << "DONE! <<press key to exit program>>" << std::endl;
		getchar();
	}
	catch (const std::exception& e)
	{
		MessageBoxA(NULL, e.what(), "Exception caught", MB_ICONERROR);
		exit(EXIT_FAILURE);
	}
	catch (...)
	{
		MessageBoxA(NULL, "UNKNOWN EXCEPTION", "Exception caught", MB_ICONERROR);
		exit(EXIT_FAILURE);
	}


	return 0;
}




//위에 있는놈이 진짜고, 이건 테스트용
/*
int main(int argc, char** argv)
{
	
	
	http_listener listener(U("http://192.168.0.21:3000"));        //Server URL, Port 지정, 내주소 적어

	//관리자 모드로 vs를 실행해야 켜지는 경향을 확인
	try
	{
		listener.open().then([&listener]() {
			cout << "\n start!!\n";
			}).wait();    //Server open

	}
	catch (const std::exception& e)
	{
		printf("Error exception:%s\n", e.what());
	}



	listener.support(methods::GET, [](http_request req) {                        //support() 함수를 통해 GET방식 지정
		req.reply(status_codes::OK, U("hello world"));                        //Lamda방식으로 간단하게 구현.
		});

	listener.support(methods::POST, [](http_request req) {                        //support() 함수를 통해 POST방식 지정
		
		req.extract_json().then([&req](json::value val) {
			if (val.is_null()) {
				req.reply(status_codes::BadRequest, U("No object in post data."));
			}
			else {
				
				cv::Mat img_dep = cv::imdecode(from_base64(val[U("depth")].as_string()), 1);
				cv::Mat img_col = cv::imdecode(from_base64(val[U("color")].as_string()), 1);

				//cv::imshow("dd", img);
				//cv::waitKey(0);
				req.reply(status_codes::OK, U("ok"));
			
			}
			}).wait();
		});



	while (true);

	listener.close();






	return 0;
}

*/