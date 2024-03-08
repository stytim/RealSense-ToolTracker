#pragma once

#include <vector>
#include <map>
#include <thread>
#include <cstdint>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "IRStructs.h"


// Cross-platform alternative for platform-specific code
#ifdef _WIN32
// Windows-specific includes and definitions
#elif defined(__APPLE__)
// MacOS-specific includes and definitions
#elif defined(__linux__)
// Linux-specific includes and definitions
#endif

class IRToolTracking; // Forward declaration

class IRToolTracker
{
public:
	IRToolTracker(IRToolTracking* pRealSenseToolTracking) {
			m_pRealSenseToolTracking = pRealSenseToolTracking;
		}


	void AddFrame(void* pAbImage, void* pDepth, uint32_t depthWidth, uint32_t depthHeight, cv::Mat _pose, double _timestamp);
	bool AddTool(cv::Mat3f spheres, float sphere_radius, std::string identifier, uint min_visible_spheres, float lowpass_rotation, float lowpass_position);
	bool RemoveTool(std::string identifier);
	bool RemoveAllTools();
	bool StartTracking();
	bool StartCalibration();
	void CalibrateTool();
	void StopCalibration();

	void SetThreshold(int threshold);
	void SetMinMaxSize(int min, int max);

	const cv::Mat& GetProcessedFrame();


	void StopTracking();
	inline bool IsTracking() { return m_bIsCurrentlyTracking; }
	inline bool IsCalibrating() { return m_bIsCurrentlyCalibrating; }

	cv::Mat GetToolTransform(std::string identifier);
	void TrackTools();
	inline std::vector<float> GetToolDefinition() { return markerPositions; }


private:

	bool ProcessFrame(AHATFrame* rawFrame, ProcessedAHATFrame& result);

	void TrackTool(IRTrackedTool &tool, ProcessedAHATFrame &frame, ToolResultContainer &result);

	void UnionSegmentation(ToolResultContainer* raw_solutions, int num_tools, ProcessedAHATFrame frame);

	cv::Mat MatchPointsKabsch(IRTrackedTool tool, ProcessedAHATFrame frame, std::vector<int> sphere_ids, std::vector<int> occluded_nodes);

	cv::Mat FlipTransformRightLeft(cv::Mat hololens_transform);

	void ConstructMap(cv::Mat3f spheres_xyz, int num_spheres, cv::Mat& result_map, std::vector<Side>& result_ordered_sides);


	bool m_bShouldStop = false;

	std::vector<IRTrackedTool> m_Tools;

	AHATFrame* m_CurrentFrame = nullptr;
	std::mutex m_MutexCurFrame;

	std::map<std::string, int> m_ToolIndexMapping;

	float m_fToleranceSide = 4.0f;
	float m_fToleranceAvg = 4.0f;

	bool m_bIsCurrentlyTracking = false;
	bool m_bIsCurrentlyCalibrating = false;

	std::shared_ptr<std::thread> m_TrackingThread;
	std::shared_ptr<std::thread> m_CalibrationThread;

	double m_lTrackedTimestamp = 0;

	std::mutex mtx_frames;
	cv::Mat m_ProcessedFrame;

	IRToolTracking* m_pRealSenseToolTracking;

	uchar m_Threshold = 100;
	int m_MinSize = 10;
	int m_MaxSize = 180;

	float m_fCalibrationSphereRadius = 6.5f;
	const int MAX_CALIBRATION_FRAMES = 100;
	int NUM_CALIBRATION_SPHERES = 4;

	std::vector<float> markerPositions;
};
