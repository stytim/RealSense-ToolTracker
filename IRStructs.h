#pragma once

#include <vector>
#include <map>
#include <thread>
#include <cstdint>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "IRKalmanFilter.h"

struct Side
{
	int id_from{ 0 };
	int id_to{ 0 };
	float distance{ 0 };

	static bool compare(Side a, Side b) {
		if (a.distance < b.distance) {
			return true;
		}
		return false;
	}

};


struct AHATFrame {
	double timestamp;
	cv::Mat device_pose;
	cv::Mat cvAbImage;
	uint16_t* pDepth;
	uint32_t depthWidth;
	uint32_t depthHeight;
};

struct ProcessedAHATFrame
{
	double timestamp;
	cv::Mat device_pose;
	uint num_spheres;
	cv::Mat3f spheres_xyd;
	std::map<float, cv::Mat3f> spheres_xyz_per_mm;
	std::map<float, std::vector<Side>> ordered_sides_per_mm;
	std::map<float, cv::Mat> map_per_mm;
};

struct ToolResult
{
	int tool_id{ -1 };
	std::vector<int> sphere_ids;
	float error{ 0 };
	std::vector<int> occluded_nodes;
	float dist_to_prev{ 0 };
	static bool compare(ToolResult a, ToolResult b)
	{
		if (a.occluded_nodes.size() < b.occluded_nodes.size()) {
			return true;
		}
		else if (a.occluded_nodes.size() < b.occluded_nodes.size()) {
			return false;
		}
		//Results have same amount of spheres visisble
		if (a.dist_to_prev < b.dist_to_prev) {
			return true;
		}
		if (a.error < b.error) {
			return true;
		}
		return false;
	}
};

struct ToolResultContainer
{
	int tool_id{ -1 };
	std::vector<ToolResult> candidates;
};

struct Tool
{
	int numSpheres;
	std::vector<float> spherePositions;
	float sphereRadius;
	std::string toolName;
	bool isAdded;

	Tool() : numSpheres(4), sphereRadius(6.5f), toolName("Tool"), isAdded(false)
	{
		spherePositions.resize(numSpheres * 3, 0.0f);
	}
};

struct IRTrackedTool
{
	//Name of tool for easier access
	std::string identifier;

	//Sphere count, min 3, max n
	uint num_spheres;

	//Minimum visible spheres, min 3, max num_spheres
	uint min_visible_spheres;

	//sphere positions relative to tool origin
	cv::Mat3f spheres_xyz;
	float sphere_radius;

	//distances between spheres
	std::vector<Side> ordered_sides;
	cv::Mat map;

	//Kalman filtering
	std::vector<IRToolKalmanFilter> sphere_kalman_filters;

	//Low Pass Filter
	float lowpass_factor_rotation = 0.3f;
	float lowpass_factor_position = 0.6f;

	//Position of the tool in the world 
	cv::Mat cur_transform = cv::Mat::zeros(8, 1, CV_32F);
	cv::Vec3f cur_position_cheap{};
	std::vector<cv::Vec3f> unfiltered_sphere_positions;
	double timestamp{ 0 };

	bool tracking_finished = true;
};