#include "IRToolTrack.h"
#include "IRToolTracking.h"
#include <set>
#include <limits>


IRToolTracker::~IRToolTracker()
{
	m_bShouldStop = true;
	if (m_TrackingThread && m_TrackingThread->joinable()) {
		try { m_TrackingThread->join(); } catch (const std::system_error&) {}
	}
	if (m_CalibrationThread && m_CalibrationThread->joinable()) {
		try { m_CalibrationThread->join(); } catch (const std::system_error&) {}
	}
	// m_CurrentFrame is a unique_ptr; its destructor frees any pending frame.
}


// Compile-time toggles for the smoothing filters. Override with -D… if you
// want to disable a filter without editing the source.
#ifndef DISABLE_LOWPASS
#define DISABLE_LOWPASS 0
#endif
#ifndef DISABLE_KALMAN
#define DISABLE_KALMAN 0
#endif



cv::Mat IRToolTracker::GetToolTransform(std::string identifier)
{
	std::lock_guard<std::mutex> lock(m_ToolsMutex);
	if (m_Tools.size() == 0 || m_ToolIndexMapping.count(identifier) == 0)
		return cv::Mat::zeros(8, 1, CV_32F);

	auto it = m_ToolIndexMapping.find(identifier);
	int index = it->second;

	// Deep copy so callers (UDP/CSV/GUI threads) never alias the live buffer that
	// the tracking thread keeps writing, and so the visibility reset below does not
	// corrupt the tool's stored transform.
	cv::Mat transform = m_Tools.at(index).cur_transform.clone();
	if (m_lTrackedTimestamp != m_Tools.at(index).timestamp)
	{
		transform.at<float>(7, 0) = 0.f;
	}

	return transform;
}

void IRToolTracker::TrackTools()
{
	// m_bIsCurrentlyTracking is set true by StartTracking() before this thread starts,
	// and cleared on exit below, so callers observe the tracking state synchronously.
	while (!m_bShouldStop) {
		std::unique_ptr<AHATFrame> rawFrame;
		{
			std::lock_guard<std::mutex> lock(m_MutexCurFrame);
			rawFrame = std::move(m_CurrentFrame);
		}
		if (!rawFrame) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}

		ProcessedAHATFrame processedFrame;
		if (!ProcessFrame(*rawFrame, processedFrame)) {
			continue;
		}

		const int current_num_tools = static_cast<int>(m_Tools.size());
		std::vector<ToolResultContainer> raw_results(current_num_tools);

		for (int i = 0; i < current_num_tools; i++) {
			raw_results[i].tool_id = i;
			TrackTool(m_Tools.at(i), processedFrame, raw_results[i]);
		}

		UnionSegmentation(raw_results, processedFrame);
		PublishWorkingFrame();
	}
	m_bIsCurrentlyTracking = false;
}

void IRToolTracker::TrackTool(IRTrackedTool &tool, const ProcessedAHATFrame &frame, ToolResultContainer &result)
{
	if (frame.num_spheres < tool.min_visible_spheres) {
		//Not enough spheres for the tool are available
		return;
	}
	std::vector<Side> eligible_sides;
	

	const int radius_key = SphereRadiusKey(tool.sphere_radius);
	auto it_sides = frame.ordered_sides_per_mm.find(radius_key);
	auto it_map = frame.map_per_mm.find(radius_key);
	// No precomputed map for this tool's sphere radius in this frame: nothing to match.
	if (it_sides == frame.ordered_sides_per_mm.end() || it_map == frame.map_per_mm.end())
		return;
	std::vector<Side> frame_ordered_sides = it_sides->second;
	cv::Mat frame_map = it_map->second;

	//Find the set of eligible side to start with - aka sides that have similar length to first side of tool

	struct search_entry {
		std::vector<int> visited_nodes_frame;
		float combined_error{ 0 };
		int num_sides{ 0 };
		std::vector<int> occluded_nodes_tool;
	};

	std::vector<search_entry> search_list;

	float cur_side_length = tool.map.at<float>(0, 1);
	std::vector<int> hidden_nodes;
	int max_occluded_spheres = tool.num_spheres - tool.min_visible_spheres;

	for (int m = 0; m <= max_occluded_spheres; m++)
	{
		std::vector<int> hidden_nodes_inside;
		for (int k = m+1; k <= max_occluded_spheres+1; k++)
		{		
			cur_side_length = tool.map.at<float>(m, k);
			for (int i = 0; i < frame_ordered_sides.size(); i++) {
				Side s = frame_ordered_sides.at(i);
				if (cv::abs(s.distance - cur_side_length) < m_fToleranceSide) {
					eligible_sides.push_back(s);
				}
			}
			if (eligible_sides.size() == 0 && max_occluded_spheres == 0)
			{
				return;
			}
			if (eligible_sides.size() != 0)
			{
				for (int occl_nodes = m+1; occl_nodes < k; occl_nodes++)
				{
					hidden_nodes_inside.push_back(occl_nodes);
				}
				break;
			}
		}
		if (eligible_sides.size() == 0 && max_occluded_spheres == 0)
		{
			return;
		}
			
		if (eligible_sides.size() != 0)
		{
			std::vector<int> all_hidden_nodes;
			all_hidden_nodes.reserve(hidden_nodes.size() + hidden_nodes_inside.size());
			all_hidden_nodes.insert(all_hidden_nodes.end(), hidden_nodes.begin(), hidden_nodes.end());
			all_hidden_nodes.insert(all_hidden_nodes.end(), hidden_nodes_inside.begin(), hidden_nodes_inside.end());
			if (!(all_hidden_nodes.size() > max_occluded_spheres))
			{
				//From the start sides, add each direction to search queue
				for (Side s : eligible_sides)
				{
					search_entry forward{ std::vector<int>{s.id_from, s.id_to}, cv::abs(s.distance - cur_side_length), 1, std::vector<int>{all_hidden_nodes} };
					search_entry backward{ std::vector<int>{s.id_to, s.id_from}, cv::abs(s.distance - cur_side_length), 1, std::vector<int>{all_hidden_nodes} };
					search_list.push_back(backward);
					search_list.push_back(forward);
				}
			}

			
			eligible_sides.clear();
		}
		hidden_nodes.push_back(m);
	}

	while (search_list.size() > 0) {
		search_entry curr = search_entry{ search_list.back() };
		search_list.pop_back();

		if (curr.occluded_nodes_tool.size() <= (max_occluded_spheres) &&
			 curr.visited_nodes_frame.size() == (tool.num_spheres - curr.occluded_nodes_tool.size())) {
			ToolResult r{};
			r.error = curr.combined_error;
			r.sphere_ids = curr.visited_nodes_frame;
			r.occluded_nodes = curr.occluded_nodes_tool;

			if ((curr.combined_error / (curr.num_sides))<m_fToleranceAvg)
			{
				//r.dist_to_prev = 
				result.candidates.push_back(r);
			}
			continue;
		}

		for (int candidate_node_id = 0; candidate_node_id < frame.num_spheres; candidate_node_id++) {
			if (std::find(curr.visited_nodes_frame.begin(), curr.visited_nodes_frame.end(), candidate_node_id) != curr.visited_nodes_frame.end()) {
				//Already used this element
				continue;
			}
			bool exceeded_side_tolerance = false;
			float error_new = 0.f;
			int error_counter = 0;
			int tool_node_id = 0;
			for (int j = 0; j < curr.visited_nodes_frame.size(); j++) {
				//Account for occluded nodes
				while (std::find(curr.occluded_nodes_tool.begin(), curr.occluded_nodes_tool.end(), tool_node_id) != curr.occluded_nodes_tool.end()) {
					tool_node_id++;
				}
				int id1 = curr.visited_nodes_frame.at(j);
				int id2 = candidate_node_id;

				//Swap if id1 is bigger than id2 (we only build a triangular distance matrix)
				if (id1 > id2)
				{
					int temp = id1;
					id1 = id2;
					id2 = temp;
				}
				float error_side = cv::abs(frame_map.at<float>(id1, id2) - tool.map.at<float>(tool_node_id, curr.visited_nodes_frame.size()+curr.occluded_nodes_tool.size()));
				if (error_side > m_fToleranceSide) {
					exceeded_side_tolerance = true;
					break;
				}
				error_new += error_side;
				error_counter++;
				tool_node_id++;
			}
			if (exceeded_side_tolerance)
			{
				if (curr.occluded_nodes_tool.size() < (max_occluded_spheres))
				{
					std::vector<int> occluded_nodes_new = std::vector<int>(curr.occluded_nodes_tool);
					occluded_nodes_new.push_back(curr.visited_nodes_frame.size() + curr.occluded_nodes_tool.size());
					search_list.push_back(search_entry{ curr.visited_nodes_frame, curr.combined_error, curr.num_sides, occluded_nodes_new});
				}
				continue;
			}
				
			std::vector<int> searched_ids_new = std::vector<int>(curr.visited_nodes_frame);
			searched_ids_new.push_back(candidate_node_id);
			search_list.push_back(search_entry{ searched_ids_new, curr.combined_error + error_new, curr.num_sides + error_counter, curr.occluded_nodes_tool});
		}
	}
}



void IRToolTracker::UnionSegmentation(std::vector<ToolResultContainer> &raw_solutions, const ProcessedAHATFrame &frame) {
	std::vector<ToolResult> unique_solutions;
	const int num_tools = static_cast<int>(raw_solutions.size());
	for (int i = 0; i < num_tools; i++)
	{
		std::vector<ToolResult> &candidates = raw_solutions[i].candidates;
		if (candidates.empty())
			continue;

		std::sort(candidates.begin(), candidates.end(), &ToolResult::compare);

		for (ToolResult candidate : candidates)
		{
			candidate.tool_id = i;
			unique_solutions.push_back(std::move(candidate));
		}
	}

	std::sort(unique_solutions.begin(), unique_solutions.end(), &ToolResult::compare);

	std::vector<bool> claimed_tools(num_tools, false);
	std::set<int> used_spheres;

	// Collect the winning poses first (MatchPointsKabsch is heavy and must run without
	// the lock), then publish them all together under a single lock so readers
	// (GetToolTransform) observe the per-tool transforms/timestamps and
	// m_lTrackedTimestamp updated atomically — no torn read, no visibility glitch.
	std::vector<std::pair<int, cv::Mat>> committed;

	for (const ToolResult &current : unique_solutions)
	{
		if (claimed_tools[current.tool_id])
			continue;

		bool overlap = false;
		for (int sid : current.sphere_ids) {
			if (used_spheres.count(sid) > 0) {
				overlap = true;
				break;
			}
		}
		if (overlap)
			continue;

		cv::Mat result = MatchPointsKabsch(m_Tools[current.tool_id], frame, current.sphere_ids, current.occluded_nodes);
		if (result.at<float>(7, 0) == 1.f)
		{
			committed.emplace_back(current.tool_id, result.clone());
		}

		claimed_tools[current.tool_id] = true;
		used_spheres.insert(current.sphere_ids.begin(), current.sphere_ids.end());
	}

	{
		std::lock_guard<std::mutex> lock(m_ToolsMutex);
		for (auto &c : committed)
		{
			m_Tools.at(c.first).cur_transform = c.second;
			m_Tools.at(c.first).timestamp = frame.timestamp;
		}
		m_lTrackedTimestamp = frame.timestamp;
	}
}

cv::Mat IRToolTracker::MatchPointsKabsch(IRTrackedTool &tool, const ProcessedAHATFrame &frame, const std::vector<int> &sphere_ids, const std::vector<int> &occluded_nodes) {
	int num_points = static_cast<int>(tool.num_spheres) - static_cast<int>(occluded_nodes.size());
	// A degenerate match (no usable points) cannot yield a pose; report invisible.
	if (num_points <= 0)
		return cv::Mat::zeros(8, 1, CV_32F);

	auto it_spheres_xyz = frame.spheres_xyz_per_mm.find(SphereRadiusKey(tool.sphere_radius));
	if (it_spheres_xyz == frame.spheres_xyz_per_mm.end())
		return cv::Mat::zeros(8, 1, CV_32F);
	cv::Mat3f frame_spheres_xyz = it_spheres_xyz->second;

	cv::Mat p = cv::Mat(num_points, 3, CV_32F);
	cv::Mat q = cv::Mat(num_points, 3, CV_32F);
	cv::Vec3f p_center = cv::Vec3f(0.f);
	cv::Vec3f q_center = cv::Vec3f(0.f);

	cv::Mat hololens_pose_mm = frame.device_pose.clone();

	hololens_pose_mm.at<float>(0, 3) = hololens_pose_mm.at<float>(0, 3) * 1000.f;
	hololens_pose_mm.at<float>(1, 3) = hololens_pose_mm.at<float>(1, 3) * 1000.f;
	hololens_pose_mm.at<float>(2, 3) = hololens_pose_mm.at<float>(2, 3) * 1000.f;

	int tool_node_id = 0;
	for (int i = 0; i < num_points; i++) {
		while (std::find(occluded_nodes.begin(), occluded_nodes.end(), tool_node_id) != occluded_nodes.end()) {
			tool_node_id++;
		}
		cv::Vec3f sphere = tool.spheres_xyz.at<cv::Vec3f>(tool_node_id, 0);
		

		p.at<float>(i, 0) = sphere[0];
		p.at<float>(i, 1) = sphere[1];
		p.at<float>(i, 2) = sphere[2];
		p_center[0] += sphere[0];
		p_center[1] += sphere[1];
		p_center[2] += sphere[2];

		int frame_sphere_id = sphere_ids.at(i);
		cv::Vec3f sphere_frame = frame_spheres_xyz.at<cv::Vec3f>(frame_sphere_id, 0);

		cv::Mat sphere_frame_mat = cv::Mat(4, 1, CV_32F);
		sphere_frame_mat.at<float>(0, 0) = sphere_frame[0];
		sphere_frame_mat.at<float>(1, 0) = sphere_frame[1];
		sphere_frame_mat.at<float>(2, 0) = sphere_frame[2];
		sphere_frame_mat.at<float>(3, 0) = 1.f;

		//Copy translation 
		float uv[2] = { 0, 0};
		float xyz[3] = { 0, 0, 0 };
		xyz[0] = sphere_frame[0];
		xyz[1] = sphere_frame[1];
		xyz[2] = sphere_frame[2];

		m_pRealSenseToolTracking->ProjectPointToPixel(xyz,uv);
    	cv::Point center(uv[0], uv[1]);
		cv::drawMarker(m_WorkingFrame, center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);

		cv::Mat sphere_world_mat = hololens_pose_mm * sphere_frame_mat;
		cv::Vec3f sphere_world = cv::Vec3f(sphere_world_mat.at<float>(0, 0), sphere_world_mat.at<float>(1, 0), sphere_world_mat.at<float>(2, 0));

		//Filter the resulting world position
#if !DISABLE_KALMAN
		sphere_world = tool.sphere_kalman_filters.at(tool_node_id).FilterData(sphere_world);
#endif
		tool_node_id++;


		q.at<float>(i, 0) = sphere_world[0];
		q.at<float>(i, 1) = sphere_world[1];
		q.at<float>(i, 2) = sphere_world[2];
		q_center[0] += sphere_world[0];
		q_center[1] += sphere_world[1];
		q_center[2] += sphere_world[2];

	}

	p_center /= num_points;
	q_center /= num_points;

	cv::Mat p_residual = cv::Mat(p);
	cv::Mat q_residual = cv::Mat(q);
	for (int r = 0; r < p_residual.rows; ++r) {
		p_residual.at<float>(r, 0) -= p_center[0];
		p_residual.at<float>(r, 1) -= p_center[1];
		p_residual.at<float>(r, 2) -= p_center[2];
		q_residual.at<float>(r, 0) -= q_center[0];
		q_residual.at<float>(r, 1) -= q_center[1];
		q_residual.at<float>(r, 2) -= q_center[2];
	}

	//SVD
	cv::Mat cov = p_residual.t() * q_residual;
	cv::SVD svd(cov);

	//Find rotation
	double d = cv::determinant(svd.vt.t() * svd.u.t());

	if (d > 0)
		d = 1.0;
	else
		d = -1.0;

	cv::Mat I = cv::Mat::eye(3, 3, CV_32F);

	I.at<float>(2, 2) = (float)d;

	cv::Mat R = svd.vt.t() * I * svd.u.t();

	cv::Mat q_avgMat = cv::Mat(3, 1, CV_32F);
	cv::Mat p_avgMat = cv::Mat(3, 1, CV_32F);
	q_avgMat.at<float>(0, 0) = q_center[0];
	q_avgMat.at<float>(1, 0) = q_center[1];
	q_avgMat.at<float>(2, 0) = q_center[2];
	p_avgMat.at<float>(0, 0) = p_center[0];
	p_avgMat.at<float>(1, 0) = p_center[1];
	p_avgMat.at<float>(2, 0) = p_center[2];

	cv::Mat t = q_avgMat - (R * p_avgMat);


	//Build transformation matrix
	cv::Mat transform_matrix = cv::Mat::zeros(4, 4, CV_32F);

	R.copyTo(transform_matrix(cv::Rect_<float>(0, 0, 3, 3)));


	transform_matrix.at<float>(0, 3) = t.at<float>(0, 0) / 1000.f;
	transform_matrix.at<float>(1, 3) = t.at<float>(1, 0) / 1000.f;
	transform_matrix.at<float>(2, 3) = t.at<float>(2, 0) / 1000.f;
	transform_matrix.at<float>(3, 3) = 1.f;

	//Copy translation and convert mm to m
	cv::Vec3f position;
	position[0] = transform_matrix.at<float>(0, 3);
	position[1] = transform_matrix.at<float>(1, 3);
	position[2] = transform_matrix.at<float>(2, 3);


	//Create Quaternion
	cv::Vec4f quat;
	quat[3] = cv::sqrt(cv::max(0.f, 1.f + transform_matrix.at<float>(0, 0) + transform_matrix.at<float>(1, 1) + transform_matrix.at<float>(2, 2))) / 2.f;
	quat[0] = cv::sqrt(cv::max(0.f, 1.f + transform_matrix.at<float>(0, 0) - transform_matrix.at<float>(1, 1) - transform_matrix.at<float>(2, 2))) / 2.f;
	quat[1] = cv::sqrt(cv::max(0.f, 1.f - transform_matrix.at<float>(0, 0) + transform_matrix.at<float>(1, 1) - transform_matrix.at<float>(2, 2))) / 2.f;
	quat[2] = cv::sqrt(cv::max(0.f, 1.f - transform_matrix.at<float>(0, 0) - transform_matrix.at<float>(1, 1) + transform_matrix.at<float>(2, 2))) / 2.f;
	quat[0] *= (quat[0] * (transform_matrix.at<float>(2, 1) - transform_matrix.at<float>(1, 2))) >= 0.f ? 1.f : -1.f;
	quat[1] *= (quat[1] * (transform_matrix.at<float>(0, 2) - transform_matrix.at<float>(2, 0))) >= 0.f ? 1.f : -1.f;
	quat[2] *= (quat[2] * (transform_matrix.at<float>(1, 0) - transform_matrix.at<float>(0, 1))) >= 0.f ? 1.f : -1.f;

	Eigen::Quaternionf rotation(quat[3], quat[0], quat[1], quat[2]);

#if !DISABLE_LOWPASS
	{
		Eigen::Quaternionf rotation_old(tool.cur_transform.at<float>(6, 0), tool.cur_transform.at<float>(3, 0), tool.cur_transform.at<float>(4, 0), tool.cur_transform.at<float>(5, 0));
    	rotation = rotation_old.slerp(tool.lowpass_factor_rotation, rotation);

    	cv::Vec3f position_old{ tool.cur_transform.at<float>(0, 0) ,tool.cur_transform.at<float>(1, 0) ,tool.cur_transform.at<float>(2, 0) };
		position = tool.lowpass_factor_position*position+(1-tool.lowpass_factor_position)*position_old;
	}
#endif

		
	cv::Mat position_rotation = cv::Mat::zeros(8, 1, CV_32F);
	//Position in xyz
	position_rotation.at<float>(0, 0) = position[0];
	position_rotation.at<float>(1, 0) = position[1];
	position_rotation.at<float>(2, 0) = position[2];
	//Quaternion
	position_rotation.at<float>(3, 0) = rotation.x();
	position_rotation.at<float>(4, 0) = rotation.y();
	position_rotation.at<float>(5, 0) = rotation.z();
	position_rotation.at<float>(6, 0) = rotation.w();
	//Last float is used to determine visibility of the tool
	position_rotation.at<float>(7, 0) = 1.f;

	return position_rotation;


}

void IRToolTracker::ConstructMap(cv::Mat3f spheres_xyz, int num_spheres, cv::Mat& map, std::vector<Side>& ordered_sides)
{
	for (int i = 0; i < num_spheres; i++) {
		for (int j = i; j < num_spheres; j++) {
			float distance = cv::norm(spheres_xyz.at<cv::Vec3f>(i, 0) - spheres_xyz.at<cv::Vec3f>(j, 0));
			map.at<float>(i, j) = distance;
			if (i == j)
				continue;
			Side s{};
			s.distance = distance;
			s.id_from = i;
			s.id_to = j;
			if (ordered_sides.size() < 1)
				ordered_sides.push_back(s);
			else {
				if (distance >= ordered_sides.back().distance)
					ordered_sides.push_back(s);
				else if (distance <= ordered_sides.front().distance)
					ordered_sides.insert(ordered_sides.begin(), s);
				else {
					auto it = std::upper_bound(ordered_sides.cbegin(), ordered_sides.cend(), s, &Side::compare);
					ordered_sides.insert(it, s);
				}
			}

		}
	}
}

void IRToolTracker::AddFrame(void* pAbImage, void* pDepth, uint32_t depthWidth, uint32_t depthHeight, cv::Mat _pose, double _timestamp) {

	cv::Mat cvAbImage_origin(cv::Size(depthWidth, depthHeight), CV_8UC1, pAbImage);
	cv::Mat cvAbImage = cvAbImage_origin.clone();

	auto frame = std::make_unique<AHATFrame>();
	frame->timestamp = _timestamp;
	frame->device_pose = _pose;
	frame->cvAbImage = std::move(cvAbImage);
	frame->depthWidth = depthWidth;
	frame->depthHeight = depthHeight;
	frame->pDepth.resize(static_cast<std::size_t>(depthWidth) * depthHeight);
	std::memcpy(frame->pDepth.data(), pDepth, frame->pDepth.size() * sizeof(uint16_t));

	std::lock_guard<std::mutex> lock(m_MutexCurFrame);
	m_CurrentFrame = std::move(frame);
}

void IRToolTracker::SetThreshold(int threshold)
{
	m_Threshold = threshold;
}

cv::Mat IRToolTracker::GetProcessedFrame()
{
	std::lock_guard<std::mutex> lock(mtx_frames);
	if (m_ProcessedFrame.empty())
		return cv::Mat();
	return m_ProcessedFrame.clone();
}

void IRToolTracker::PublishWorkingFrame()
{
	if (m_WorkingFrame.empty())
		return;
	std::lock_guard<std::mutex> lock(mtx_frames);
	m_WorkingFrame.copyTo(m_ProcessedFrame);
}

void IRToolTracker::SetMinMaxSize(int min, int max)
{
	m_MinSize = min;
	m_MaxSize = max;
}


bool IRToolTracker::ProcessFrame(AHATFrame &rawFrame, ProcessedAHATFrame &result) {
	const int minSize = m_MinSize;
	const int maxSize = m_MaxSize;
	cv::Mat labels, stats, centroids;
	std::vector<float> irToolCenters;

	// Binary threshold: any IR pixel above m_Threshold becomes 255, everything else 0.
	// SIMD-vectorized — replaces a per-pixel forEach divide that was the hottest loop.
	cv::threshold(rawFrame.cvAbImage, rawFrame.cvAbImage, m_Threshold, 255, cv::THRESH_BINARY);
	cv::cvtColor(rawFrame.cvAbImage, m_WorkingFrame, cv::COLOR_GRAY2RGB);


	int areaCount = cv::connectedComponentsWithStats(rawFrame.cvAbImage, labels, stats, centroids, 8);
	for (int i = 1; i < areaCount; ++i)
	{
		auto area = stats.at<int32_t>(i, cv::CC_STAT_AREA);
		if (area <= maxSize && area >= minSize)
		{
			double _u = centroids.at<double>(i, 0);
			double _v = centroids.at<double>(i, 1);
			float uv[2] = { _u + 0.5, _v + 0.5 };
			float xy[2] = { 0, 0 };
			float depth = static_cast<float>(rawFrame.pDepth[rawFrame.depthWidth * static_cast<size_t>(_v) + static_cast<size_t>(_u)]);
			
			float uvd[3] = { uv[0], uv[1], depth };

			m_pRealSenseToolTracking->DeprojectPixelToPoint(uvd,xy);

			irToolCenters.push_back(xy[0]);
			irToolCenters.push_back(xy[1]);
			irToolCenters.push_back(depth);
		}
	}

	int irToolCentersSize = irToolCenters.size();

	cv::Mat3f spheres = cv::Mat3f(irToolCentersSize / 3, 1);
	int j_sphere = 0;
	for (int i_sphere = 0; i_sphere < irToolCentersSize; i_sphere += 3)
	{
		spheres.at<cv::Vec3f>(j_sphere, 0) = cv::Vec3f(irToolCenters.at(i_sphere), irToolCenters.at(i_sphere + 1), irToolCenters.at(i_sphere + 2));
		j_sphere++;
	}

	int num_spheres = spheres.size().height;

	if (num_spheres < 3) {
		// If there are fewer than 3 points visible, there is no tool to track.
		// Caller owns rawFrame via unique_ptr, so no manual cleanup needed.
		return false;
	}

	//Create 3d coordinates for every possible sphere size
	std::map<int, std::vector<Side>> ordered_sides_per_mm;
	std::map<int, cv::Mat> map_per_mm;
	std::map<int, cv::Mat3f> spheres_xyz_per_mm;


	if (!m_bIsCurrentlyCalibrating)
	{
		for (IRTrackedTool tool : m_Tools)
		{
			float cur_radius = tool.sphere_radius;
			const int radius_key = SphereRadiusKey(cur_radius);
			if (spheres_xyz_per_mm.find(radius_key) != spheres_xyz_per_mm.end()) {
				//We already created this map
				continue;
			}

			cv::Mat3f spheres_xyz = spheres.clone();

			spheres_xyz.forEach(
				[&](cv::Vec3f& xyz, const int* position) -> void {
					float norm = cv::norm(xyz);
					xyz = xyz / norm * (cur_radius + norm);
				}
			);


			//Construct map
			cv::Mat map(cv::Size(num_spheres, num_spheres), CV_32F);
			std::vector<Side> ordered_sides;

			ConstructMap(spheres_xyz, num_spheres, map, ordered_sides);

			ordered_sides_per_mm.insert({ radius_key, ordered_sides });
			map_per_mm.insert({ radius_key, map });
			spheres_xyz_per_mm.insert({ radius_key, spheres_xyz });

		}
	}
	else
	{
		float cur_radius = m_fCalibrationSphereRadius;
		const int radius_key = SphereRadiusKey(cur_radius);
		cv::Mat3f spheres_xyz = spheres.clone();
		spheres_xyz.forEach(
			[&](cv::Vec3f& xyz, const int* position) -> void {
				float norm = cv::norm(xyz);
				xyz = xyz / norm * (cur_radius + norm);

				//Copy translation 
				float uv[2] = { 0, 0};
				float mxyz[3] = { 0, 0, 0 };
				mxyz[0] = xyz[0];
				mxyz[1] = xyz[1];
				mxyz[2] = xyz[2];

				m_pRealSenseToolTracking->ProjectPointToPixel(mxyz,uv);

				// Convert point coordinates to cv::Point
				cv::Point center(uv[0], uv[1]);

				cv::drawMarker(m_WorkingFrame, center, cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 2);
			}
		);
		
		//Construct map
		cv::Mat map(cv::Size(4, 4), CV_32F);
		std::vector<Side> ordered_sides;

		ConstructMap(spheres_xyz, 4, map, ordered_sides);

		ordered_sides_per_mm.insert({ radius_key, ordered_sides });
		map_per_mm.insert({ radius_key, map });
		spheres_xyz_per_mm.insert({ radius_key, spheres_xyz });
	}


	result.timestamp = rawFrame.timestamp;
	result.device_pose = rawFrame.device_pose;
	result.num_spheres = static_cast<uint>(num_spheres);
	result.spheres_xyd = spheres.clone();
	result.spheres_xyz_per_mm = spheres_xyz_per_mm;
	result.ordered_sides_per_mm = ordered_sides_per_mm;
	result.map_per_mm = map_per_mm;

	return true;
}

bool IRToolTracker::AddTool(cv::Mat3f spheres, float sphere_radius, std::string identifier, uint min_visible_spheres, float lowpass_rotation, float lowpass_position)
{
	//Do we already have this tool?
	{
		std::lock_guard<std::mutex> lock(m_ToolsMutex);
		if (m_ToolIndexMapping.count(identifier) > 0)
			return false;
	}

	bool restartTracking = false;
	if (m_bIsCurrentlyTracking) {
		restartTracking = true;
		StopTracking();
	}

	//Create the tool
	IRTrackedTool tool{};
	tool.identifier = identifier;
	tool.num_spheres = spheres.size().height;
	tool.spheres_xyz = spheres;
	tool.sphere_radius = sphere_radius;
	tool.min_visible_spheres = std::max((uint)3, std::min(min_visible_spheres, tool.num_spheres));
	tool.lowpass_factor_position = lowpass_position;
	tool.lowpass_factor_rotation = lowpass_rotation;

	//Construct map
	cv::Mat map(cv::Size(tool.num_spheres, tool.num_spheres), CV_32F);
	std::vector<Side> ordered_sides;
	ConstructMap(spheres, tool.num_spheres, map, ordered_sides);
	tool.map = map.clone();
	tool.ordered_sides = ordered_sides;
	//One Kalman filter per sphere
	for (int i = 0; i < tool.num_spheres; i++) {
		tool.sphere_kalman_filters.push_back(IRToolKalmanFilter());
	}
	//Add to map so we can find the tool with name
	{
		std::lock_guard<std::mutex> lock(m_ToolsMutex);
		m_ToolIndexMapping.insert({ identifier, m_Tools.size() });
		m_Tools.push_back(tool);
	}
	std::cout<<"Added "<<identifier<<" with "<<tool.num_spheres<<" spheres"<<std::endl;

	if (restartTracking) {
		StartTracking();
	}
	return true;
}

bool IRToolTracker::RemoveTool(std::string identifier)
{
	//Do we even have this tool?
	{
		std::lock_guard<std::mutex> lock(m_ToolsMutex);
		if (m_ToolIndexMapping.count(identifier) == 0)
			return false;
	}

	bool restartTracking = false;
	if (m_bIsCurrentlyTracking) {
		restartTracking = true;
		StopTracking();
	}

	{
		std::lock_guard<std::mutex> lock(m_ToolsMutex);
		m_ToolIndexMapping.erase(identifier);
		std::vector<IRTrackedTool> oldTools(m_Tools);
		std::map<std::string, int> oldMapping(m_ToolIndexMapping);
		m_Tools.clear();
		m_ToolIndexMapping.clear();
		for (auto pair : oldMapping)
		{
			m_ToolIndexMapping.insert({ pair.first, m_Tools.size() });
			m_Tools.push_back(oldTools.at(pair.second));
		}
	}

	std::cout << "Removed " << identifier  << std::endl;

	if (restartTracking) {

		StartTracking();
	}
	return true;
}

bool IRToolTracker::RemoveAllTools()
{
	if (m_bIsCurrentlyTracking) {
		StopTracking();
	}
	{
		std::lock_guard<std::mutex> lock(m_ToolsMutex);
		m_Tools.clear();
		m_ToolIndexMapping.clear();
	}
	return true;
}

bool IRToolTracker::StartTracking() {
	if (m_bIsCurrentlyTracking || m_Tools.size() == 0)
		return false;
	m_bShouldStop = false;
	// Mark tracking active BEFORE spawning the thread. Otherwise there is a window
	// where IsTracking()==false while the thread is already reading m_Tools, during
	// which AddTool/RemoveTool would skip StopTracking() and mutate m_Tools out from
	// under the running tracking thread (use-after-free on vector reallocation).
	m_bIsCurrentlyTracking = true;
	m_TrackingThread = std::make_shared<std::thread>(&IRToolTracker::TrackTools, this);
	return true;
}

void IRToolTracker::StopTracking()
{
	m_bShouldStop = true;
	//Wait until thread shuts down
	JoinThread(m_TrackingThread);
}

bool IRToolTracker::StartCalibration()
{
	if (m_bIsCurrentlyCalibrating)
		return false;
	m_bShouldStop = false;
	m_bIsCurrentlyCalibrating = true;
	m_CalibrationThread = std::make_shared<std::thread>(&IRToolTracker::CalibrateTool, this);
	return true;
}

// Function to convert cv::Vec3f to Eigen::Vector3f
Eigen::Vector3f cvVec3fToEigen(const cv::Vec3f &cvVec)
{
	return Eigen::Vector3f(cvVec[0], cvVec[1], cvVec[2]);
}

Eigen::Vector3f calculateStdDev(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& mean) {
	if (points.empty())
		return Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
	Eigen::Vector3f sumSq(0, 0, 0);
	for (const auto& point : points) {
		Eigen::Vector3f diff = point - mean;
		sumSq += diff.cwiseProduct(diff);
	}
	return (sumSq / points.size()).cwiseSqrt();
}

Eigen::Vector3f calculateMean(const std::vector<Eigen::Vector3f>& points) {
	// Empty input would divide by zero; return NaN so the calibration is flagged
	// invalid by the caller instead of silently producing a (0,0,0) marker.
	if (points.empty())
		return Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
	Eigen::Vector3f sum(0, 0, 0);
	for (const auto& point : points) {
		sum += point;
	}
	return sum / points.size();
}

std::vector<Eigen::Vector3f> removeOutliers(const std::vector<Eigen::Vector3f>& points, float threshold) {
	Eigen::Vector3f mean = calculateMean(points);
	Eigen::Vector3f stdDev = calculateStdDev(points, mean);

	std::vector<Eigen::Vector3f> filteredPoints;
	for (const auto& point : points) {
		Eigen::Vector3f diff = point - mean;
		if (diff.cwiseAbs().cwiseQuotient(stdDev).maxCoeff() < threshold) {
			filteredPoints.push_back(point);
		}
	}
	return filteredPoints;
}

void IRToolTracker::CalibrateTool()
{
	std::vector<ProcessedAHATFrame> processedFrames;
	processedFrames.reserve(MAX_CALIBRATION_FRAMES);

	// Exit if asked to stop OR if the streaming pipeline terminated (e.g. no device
	// connected / device disconnected), otherwise this loop would spin forever
	// waiting for frames that will never arrive.
	while (!m_bShouldStop && !m_pRealSenseToolTracking->IsTerminated()) {
		std::unique_ptr<AHATFrame> rawFrame;
		{
			std::lock_guard<std::mutex> lock(m_MutexCurFrame);
			rawFrame = std::move(m_CurrentFrame);
		}
		if (!rawFrame) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}

		ProcessedAHATFrame processedFrame;
		if (!ProcessFrame(*rawFrame, processedFrame)) {
			continue;
		}
		processedFrames.push_back(processedFrame);
		PublishWorkingFrame();

		// If enough data is collected, perform calibration
		if (processedFrames.size() == MAX_CALIBRATION_FRAMES) {
			m_bShouldStop = true;
			break;
		}
	}

	// markerPositions is the calibration result consumed by the GUI. Clear it up front
	// so that every early-return below leaves an empty result, which the GUI treats as
	// "calibration unsuccessful".
	markerPositions.clear();

	const int calib_radius_key = SphereRadiusKey(m_fCalibrationSphereRadius);

	// No frames captured (no device / disconnected / stopped before any frame): there
	// is nothing to calibrate. Bail out instead of dereferencing processedFrames[0].
	if (processedFrames.empty())
	{
		m_bIsCurrentlyCalibrating = false;
		return;
	}

	// Perform calibration, loop through processedFrames
	std::vector<std::vector<Eigen::Vector3f>> markerPoints;

	// Define the number of calibration spheres from the first frame's detected blobs.
	auto it_first = processedFrames[0].spheres_xyz_per_mm.find(calib_radius_key);
	if (it_first == processedFrames[0].spheres_xyz_per_mm.end())
	{
		m_bIsCurrentlyCalibrating = false;
		return;
	}
	NUM_CALIBRATION_SPHERES = it_first->second.size().height;

	// A tool needs at least 3 spheres to define a coordinate frame, and we refuse to
	// calibrate more than MAX_CALIBRATION_SPHERES. Either case is reported as failure.
	if (NUM_CALIBRATION_SPHERES < 3 || NUM_CALIBRATION_SPHERES > MAX_CALIBRATION_SPHERES)
	{
		m_bIsCurrentlyCalibrating = false;
		return;
	}

	markerPoints.resize(NUM_CALIBRATION_SPHERES);
	for (ProcessedAHATFrame frame : processedFrames)
	{
		auto it_sides = frame.ordered_sides_per_mm.find(calib_radius_key);
		auto it_spheres_xyz = frame.spheres_xyz_per_mm.find(calib_radius_key);
		if (it_sides == frame.ordered_sides_per_mm.end() ||
			it_spheres_xyz == frame.spheres_xyz_per_mm.end())
			continue;
		std::vector<Side> frame_ordered_sides = it_sides->second;
		cv::Mat3f frame_spheres_xyz = it_spheres_xyz->second;

		// Only use frames whose detected-sphere count matches the reference frame;
		// otherwise the fixed-size indexing below (up to NUM_CALIBRATION_SPHERES) would
		// read past the end of a frame that detected fewer blobs.
		if (frame_spheres_xyz.size().height != NUM_CALIBRATION_SPHERES || frame_ordered_sides.empty())
			continue;

		//Side shortest = frame_ordered_sides.front();
    	Side longest = frame_ordered_sides.back();

		if (std::isnan(longest.distance))
		{
			continue;
		}

		int tempMarker1ID = longest.id_from;
		int tempMarker2ID = longest.id_to;

		Eigen::Vector3f tempMarker1 = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(tempMarker1ID));
		Eigen::Vector3f tempMarker2 = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(tempMarker2ID));
		Eigen::Vector3f lineVector = tempMarker2 - tempMarker1;

		std::vector<std::pair<int, float>> distanceToLine;
		for (int i = 0; i < NUM_CALIBRATION_SPHERES; ++i)
		{
			if (i != tempMarker1ID && i != tempMarker2ID)
			{
				Eigen::Vector3f marker = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(i));
				Eigen::Vector3f tempMarkerToMarker = marker - tempMarker1;
				float distance = (tempMarkerToMarker - tempMarkerToMarker.dot(lineVector) * lineVector / lineVector.squaredNorm()).norm();
				distanceToLine.push_back(std::make_pair(i, distance));
			}
		}

		// Sorting the distances
		std::sort(distanceToLine.begin(), distanceToLine.end(), [](const std::pair<int, float> &a, const std::pair<int, float> &b) {
			return a.second < b.second;
		});

		// Need at least one off-axis marker to resolve the third coordinate.
		if (distanceToLine.empty())
			continue;

		int marker3ID = distanceToLine.front().first;
		distanceToLine.erase(distanceToLine.begin());

		// Determine marker1 and marker2
		Eigen::Vector3f marker3 = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(marker3ID));
		float distToTempMarker1 = (marker3 - tempMarker1).norm();
		float distToTempMarker2 = (marker3 - tempMarker2).norm();

		int marker1ID = distToTempMarker1 < distToTempMarker2 ? tempMarker1ID : tempMarker2ID;
		int marker2ID = distToTempMarker1 < distToTempMarker2 ? tempMarker2ID : tempMarker1ID;

		std::vector<int> remainingMarkerIDs;
		for (auto& pair : distanceToLine) {
			remainingMarkerIDs.push_back(pair.first);
		}
		
		//int marker4ID = *remainingMarkerIDs.begin();

		std::vector<int> allMarkerIDs = {marker1ID, marker2ID, marker3ID};
		allMarkerIDs.insert(allMarkerIDs.end(), remainingMarkerIDs.begin(), remainingMarkerIDs.end());

		// Convert cv::Vec3f to Eigen::Vector3f
		Eigen::Vector3f marker1 = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(marker1ID));
		Eigen::Vector3f marker2 = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(marker2ID));

		// Define the tool coordinate system
		Eigen::Vector3f x_axis = (marker2 - marker1).normalized();
		Eigen::Vector3f y_axis_temp = (marker3 - marker1).normalized();
		Eigen::Vector3f z_axis = x_axis.cross(y_axis_temp).normalized();
		Eigen::Vector3f y_axis = z_axis.cross(x_axis);

		// Construct the rotation matrix, Eigen uses column major order lol
		Eigen::Matrix3f rotation; 
		rotation.row(0) = x_axis;
		rotation.row(1) = y_axis;
		rotation.row(2) = z_axis;

		// Construct the translation vector
		Eigen::Vector3f translation = -rotation * marker1;

		// Construct the transformation matrix
		Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
		transformation.block<3, 3>(0, 0) = rotation;
		transformation.block<3, 1>(0, 3) = translation;

		// Transform the marker positions
		for (int i = 0; i < allMarkerIDs.size(); ++i) {
			Eigen::Vector3f marker = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(allMarkerIDs[i]));
			Eigen::Vector4f marker_homogeneous(marker[0], marker[1], marker[2], 1.0f);
			Eigen::Vector4f marker_tool = transformation * marker_homogeneous;
			markerPoints[i].push_back(marker_tool.head<3>());
		}
	}

	processedFrames.clear();

	// Calculate the average marker positions
	markerPositions.clear();
	markerPositions.reserve(NUM_CALIBRATION_SPHERES * 3);
	markerPositions.push_back(0);
	markerPositions.push_back(0);
	markerPositions.push_back(0);
	//std::cout << "Marker 1: " << markerPositions[0] << ", " << markerPositions[1] << ", " << markerPositions[2] << std::endl;
	for (int i = 1; i < NUM_CALIBRATION_SPHERES; i++)
	{
		Eigen::Vector3f filteredMean = calculateMean(removeOutliers(markerPoints[i], 2.0f));
		
		float x, y, z;
		x = std::round(filteredMean[0] * 1000.0f) / 1000.0f;
		y = std::round(filteredMean[1] * 1000.0f) / 1000.0f;
		z = std::round(filteredMean[2] * 1000.0f) / 1000.0f;

		markerPositions.push_back((x == -0.0f) ? 0.0f : x);
		markerPositions.push_back((y == -0.0f) ? 0.0f : y);
		markerPositions.push_back((z == -0.0f) ? 0.0f : z);
		//std::cout << "Marker " << i+1 << ": " << markerPositions[i * 3] << ", " << markerPositions[i * 3 + 1] << ", " << markerPositions[i * 3 + 2] << std::endl;
	}

	m_bIsCurrentlyCalibrating = false;
}


void IRToolTracker::StopCalibration()
{
	m_bShouldStop = true;
	JoinThread(m_CalibrationThread);
}