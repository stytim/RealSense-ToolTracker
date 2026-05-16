#pragma once

#include <vector>
#include <map>
#include <memory>
#include <thread>
#include <atomic>
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

	~IRToolTracker();


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

	cv::Mat GetProcessedFrame();


	void StopTracking();
	inline bool IsTracking() { return m_bIsCurrentlyTracking; }
	inline bool IsCalibrating() { return m_bIsCurrentlyCalibrating; }

	cv::Mat GetToolTransform(std::string identifier);
	void TrackTools();
	inline std::vector<float> GetToolDefinition() { return markerPositions; }


private:

	bool ProcessFrame(AHATFrame& rawFrame, ProcessedAHATFrame& result);

	void TrackTool(IRTrackedTool &tool, const ProcessedAHATFrame &frame, ToolResultContainer &result);

	void UnionSegmentation(std::vector<ToolResultContainer> &raw_solutions, const ProcessedAHATFrame &frame);

	cv::Mat MatchPointsKabsch(IRTrackedTool &tool, const ProcessedAHATFrame &frame, const std::vector<int> &sphere_ids, const std::vector<int> &occluded_nodes);

	void ConstructMap(cv::Mat3f spheres_xyz, int num_spheres, cv::Mat& result_map, std::vector<Side>& result_ordered_sides);


	std::atomic<bool> m_bShouldStop{false};

	std::vector<IRTrackedTool> m_Tools;

	std::unique_ptr<AHATFrame> m_CurrentFrame;
	std::mutex m_MutexCurFrame;

	std::map<std::string, int> m_ToolIndexMapping;

	float m_fToleranceSide = 4.0f;
	float m_fToleranceAvg = 4.0f;

	std::atomic<bool> m_bIsCurrentlyTracking{false};
	std::atomic<bool> m_bIsCurrentlyCalibrating{false};

	std::shared_ptr<std::thread> m_TrackingThread;
	std::shared_ptr<std::thread> m_CalibrationThread;

	double m_lTrackedTimestamp = 0;

	std::mutex mtx_frames;
	// Tracking-thread-only scratch buffer drawn into during each ProcessFrame/MatchPointsKabsch.
	cv::Mat m_WorkingFrame;
	// Published copy snapshot to viewers; only touched under mtx_frames.
	cv::Mat m_ProcessedFrame;

	void PublishWorkingFrame();

	IRToolTracking* m_pRealSenseToolTracking;

	uchar m_Threshold = 100;
	int m_MinSize = 10;
	int m_MaxSize = 180;

	float m_fCalibrationSphereRadius = 6.5f;
	const int MAX_CALIBRATION_FRAMES = 100;
	int NUM_CALIBRATION_SPHERES = 4;

	std::vector<float> markerPositions;
};
