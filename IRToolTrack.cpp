#include "IRToolTrack.h"
#include "IRToolTracking.h"

#include <set>


#define DISABLE_LOWPASS FALSE
#define DISABLE_KALMAN FALSE



cv::Mat IRToolTracker::GetToolTransform(std::string identifier)
{
	if (m_Tools.size() == 0 || m_ToolIndexMapping.count(identifier) == 0)
		return cv::Mat::zeros(8, 1, CV_32F);

	auto it = m_ToolIndexMapping.find(identifier);
	int index = it->second;

	cv::Mat transform = m_Tools.at(index).cur_transform;
	if (m_lTrackedTimestamp != m_Tools.at(index).timestamp)
	{
		transform.at<float>(7, 0) = 0.f;
	}

	return transform;
}

void IRToolTracker::TrackTools()
{
	while (!m_bShouldStop) {
		m_bIsCurrentlyTracking = true;
		m_MutexCurFrame.lock();
		if (m_CurrentFrame == nullptr) {
			m_MutexCurFrame.unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}
		
		//Copy pointer to frame
		AHATFrame* rawFrame = m_CurrentFrame;
		m_CurrentFrame = nullptr;
		m_MutexCurFrame.unlock();

		int current_num_tools = m_Tools.size();
		ToolResultContainer* raw_results = new ToolResultContainer[current_num_tools];

		ProcessedAHATFrame processedFrame;

		if (!ProcessFrame(rawFrame, processedFrame)) {
			continue;
		}
		
		//std::vector<std::thread> tool_track_threads(current_num_tools);

		for (int i = 0; i < current_num_tools; i++) {
			IRTrackedTool tool = m_Tools.at(i);
			if (!tool.tracking_finished)
				continue;

			ToolResultContainer result{ i, std::vector<ToolResult>() };
			
			//tool_track_threads.at(i) = std::thread(&IRToolTracker::TrackTool, this, tool, processedFrame, result);
			TrackTool(tool, processedFrame, result);
			raw_results[i] = result;
			//ProcessEnvFrame(processedFrame, result);
		}

		//for (auto & thread : tool_track_threads) {
		//	thread.join();
		//}
		UnionSegmentation(raw_results, current_num_tools, processedFrame);

		delete[] raw_results;
		//TODO: make sure i didnt create a memory leak here
	}
	m_bIsCurrentlyTracking = false;
}

void IRToolTracker::TrackTool(IRTrackedTool &tool, ProcessedAHATFrame &frame, ToolResultContainer &result)
{
	tool.tracking_finished = false;
	if (frame.num_spheres < tool.min_visible_spheres) {
		//Not enough spheres for the tool are available
		tool.tracking_finished = true;
		return;
	}
	std::vector<Side> eligible_sides;
	

	auto it_sides = frame.ordered_sides_per_mm.find(tool.sphere_radius);
	std::vector<Side> frame_ordered_sides = it_sides->second;

	auto it_map = frame.map_per_mm.find(tool.sphere_radius);
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
				tool.tracking_finished = true;
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
			tool.tracking_finished = true;
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
	tool.tracking_finished = true;
	return;
}



void IRToolTracker::UnionSegmentation(ToolResultContainer* raw_solutions, int num_tools, ProcessedAHATFrame frame) {
	int* tool_solutions = new int[num_tools];
	std::vector<ToolResult> unique_solutions;
	for (int i = 0; i < num_tools; i++)
	{
		tool_solutions[i] = 0;
		ToolResultContainer tool_results = raw_solutions[i];
		
		//std::cout << "Tool " << i << " has " << tool_results.candidates.size() << " candidates" << std::endl;
		if (tool_results.candidates.size() == 0)
			continue;

		std::vector<ToolResult> ordered_candidates = tool_results.candidates;
		std::sort(ordered_candidates.begin(), ordered_candidates.end(), &ToolResult::compare);

		for (ToolResult candidate : ordered_candidates)
		{
			candidate.tool_id = i;
			unique_solutions.push_back(candidate);
			tool_solutions[i]++;
		}
	}

	std::sort(unique_solutions.begin(), unique_solutions.end(), &ToolResult::compare);

	while (unique_solutions.size() > 0)
	{
		ToolResult current = unique_solutions.front();
		int cur_toolid = current.tool_id;
		unique_solutions.erase(unique_solutions.begin());
		cv::Mat result = MatchPointsKabsch(m_Tools[cur_toolid], frame, current.sphere_ids, current.occluded_nodes);
		if (result.at<float>(7, 0) == 1.f)
		{
			m_Tools.at(cur_toolid).cur_transform = result.clone();
			m_Tools.at(cur_toolid).timestamp = frame.timestamp;
		}

		std::vector<ToolResult> remaining_unique_solutions;
		for (ToolResult next_check : unique_solutions) {
			if (next_check.tool_id == cur_toolid)
				continue;

			bool used = false;
			for (auto cursphere : current.sphere_ids)
			{
				if (used)
				{
					break;
				}
				for (auto nexsphere : next_check.sphere_ids)
				{
					if (cursphere == nexsphere)
					{
						used = true;
						break;
					}

				}
			}
			// std::vector<int> intersection;
			//std::set_intersection(current.sphere_ids.begin(), current.sphere_ids.end(), next_check.sphere_ids.begin(), next_check.sphere_ids.end(), intersection.begin());
			//if (intersection.size() > 0)
			//	continue;
			if (used)
			{
				continue;
			}
			remaining_unique_solutions.push_back(next_check);
		}
		unique_solutions = remaining_unique_solutions;
	}
	delete[] tool_solutions;
	m_lTrackedTimestamp = frame.timestamp;
	return;
}

cv::Mat IRToolTracker::MatchPointsKabsch(IRTrackedTool tool, ProcessedAHATFrame frame, std::vector<int> sphere_ids, std::vector<int> occluded_nodes) {
	int num_points = tool.num_spheres-occluded_nodes.size();
	cv::Mat p = cv::Mat(num_points, 3, CV_32F);
	cv::Mat q = cv::Mat(num_points, 3, CV_32F);
	cv::Vec3f p_center = cv::Vec3f(0.f);
	cv::Vec3f q_center = cv::Vec3f(0.f);

	auto it_spheres_xyz = frame.spheres_xyz_per_mm.find(tool.sphere_radius);
	cv::Mat3f frame_spheres_xyz = it_spheres_xyz->second;

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
		sphere_frame_mat.at<float>(0, 1) = sphere_frame[1];
		sphere_frame_mat.at<float>(0, 2) = sphere_frame[2];
		sphere_frame_mat.at<float>(0, 3) = 1.f;

		//Copy translation 
		float uv[2] = { 0, 0};
		float xyz[3] = { 0, 0, 0 };
		xyz[0] = sphere_frame[0];
		xyz[1] = sphere_frame[1];
		xyz[2] = sphere_frame[2];

		m_pRealSenseToolTracking->ProjectPointToPixel(xyz,uv);
    	cv::Point center(uv[0], uv[1]);
		cv::drawMarker(m_ProcessedFrame, center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);

		cv::Mat sphere_world_mat = hololens_pose_mm * sphere_frame_mat;
		cv::Vec3f sphere_world = cv::Vec3f(sphere_world_mat.at<float>(0, 0), sphere_world_mat.at<float>(1, 0), sphere_world_mat.at<float>(2, 0));

		//Filter the resulting world position
#if !DEBUG_NO_FILTER && !DISABLE_KALMAN
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

	// transform_matrix = FlipTransformRightLeft(transform_matrix);

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

#if !DISABLE_LOWPASS && !DEBUG_NO_FILTER
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

cv::Mat IRToolTracker::FlipTransformRightLeft(cv::Mat transform_rhs)
{
	//Bring to unity coordinate system
	cv::Mat flipz = cv::Mat::ones(4, 4, CV_32F);
	flipz.at<float>(2, 0) = -1.f;
	flipz.at<float>(0, 2) = -1.f;
	flipz.at<float>(2, 1) = -1.f;
	flipz.at<float>(1, 2) = -1.f;
	flipz.at<float>(2, 3) = -1.f;
	cv::Mat transform_lhs = transform_rhs.mul(flipz);
	return transform_lhs;
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

	cv::Mat cvAbImage_origin(cv::Size(depthWidth, depthHeight), CV_8UC1, (void*)pAbImage);
	cv::Mat cvAbImage = cvAbImage_origin.clone();

	m_MutexCurFrame.lock();

	if (m_CurrentFrame != nullptr) {
		delete[] m_CurrentFrame->pDepth;
		delete m_CurrentFrame;
	}

	m_CurrentFrame = new AHATFrame { _timestamp, _pose, cvAbImage,  new uint16_t[depthWidth * depthHeight], depthWidth, depthHeight };
	memcpy(m_CurrentFrame->pDepth, pDepth, depthWidth * depthHeight * sizeof(uint16_t));

	m_MutexCurFrame.unlock();

}

void IRToolTracker::SetThreshold(int threshold)
{
	m_Threshold = threshold;
}

cv::Mat IRToolTracker::GetProcessedFrame()
{
	return m_ProcessedFrame.clone();
}

void IRToolTracker::SetMinMaxSize(int min, int max)
{
	m_MinSize = min;
	m_MaxSize = max;
}


bool IRToolTracker::ProcessFrame(AHATFrame* rawFrame, ProcessedAHATFrame &result) {
	uchar lowerLimit = m_Threshold;
	uchar upperLimit = lowerLimit + 1;// 256 * 20;
	int minSize = m_MinSize, maxSize = m_MaxSize;
	cv::Mat labels, stats, centroids;
	std::vector<float> irToolCenters;

	rawFrame->cvAbImage.forEach<uchar>(
		[&](uchar& ir, const int* position) -> void {
			ir = (std::clamp(ir, lowerLimit, upperLimit) - lowerLimit) / (upperLimit - lowerLimit)*255;
		}
	);
	
	rawFrame->cvAbImage.convertTo(rawFrame->cvAbImage, CV_8UC1);
	cv::cvtColor(rawFrame->cvAbImage.clone(), m_ProcessedFrame, cv::COLOR_GRAY2RGB);
	

	int areaCount = cv::connectedComponentsWithStats(rawFrame->cvAbImage, labels, stats, centroids, 8);
	for (int i = 1; i < areaCount; ++i)
	{
		auto area = stats.at<int32_t>(i, cv::CC_STAT_AREA);
		if (area <= maxSize && area >= minSize)
		{
			double _u = centroids.at<double>(i, 0);
			double _v = centroids.at<double>(i, 1);
			float uv[2] = { _u + 0.5, _v + 0.5 };
			float xy[2] = { 0, 0 };
			float depth = (static_cast<float>(rawFrame->pDepth[rawFrame->depthWidth * (uint16_t)_v + (uint16_t)_u]));
			
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
		//If theres less than 3 points visible, theres no tool to track
		//Free memory
		delete[] rawFrame->pDepth;
		delete rawFrame;
		return false;
	}

	//Create 3d coordinates for every possible sphere size
	std::map<float, std::vector<Side>> ordered_sides_per_mm;
	std::map<float, cv::Mat> map_per_mm;
	std::map<float, cv::Mat3f> spheres_xyz_per_mm;


	if (!m_bIsCurrentlyCalibrating)
	{
		for (IRTrackedTool tool : m_Tools)
		{
			float cur_radius = tool.sphere_radius;
			if (!(spheres_xyz_per_mm.find(cur_radius) == spheres_xyz_per_mm.end())) {
				//We already created this map
				continue;
			}

			cv::Mat3f spheres_xyz = spheres.clone();

			spheres_xyz.forEach(
				[&](cv::Vec3f& xyz, const int* position) -> void {
					float norm = cv::norm(xyz);
					xyz = xyz / norm * (cur_radius + norm);
					// xyz[2] = xyz[2] + cur_radius;
					// cv::Vec3f temp_vec(xyz[0], xyz[1], 1);
					// xyz = cv::Vec3f((temp_vec / cv::norm(temp_vec)) * xyz[2]);
				}
			);	


			//Construct map
			cv::Mat map(cv::Size(num_spheres, num_spheres), CV_32F);
			std::vector<Side> ordered_sides;

			ConstructMap(spheres_xyz, num_spheres, map, ordered_sides);

			ordered_sides_per_mm.insert({ cur_radius, ordered_sides });
			map_per_mm.insert({ cur_radius, map });
			spheres_xyz_per_mm.insert({ cur_radius, spheres_xyz });

		}
	}
	else
	{
		float cur_radius = m_fCalibrationSphereRadius;
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

				cv::drawMarker(m_ProcessedFrame, center, cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 2);
			}
		);
		
		//Construct map
		cv::Mat map(cv::Size(4, 4), CV_32F);
		std::vector<Side> ordered_sides;

		ConstructMap(spheres_xyz, 4, map, ordered_sides);

		ordered_sides_per_mm.insert({ cur_radius, ordered_sides });
		map_per_mm.insert({ cur_radius, map });
		spheres_xyz_per_mm.insert({ cur_radius, spheres_xyz });
	}


	result.timestamp = rawFrame->timestamp;
	result.device_pose = rawFrame->device_pose;
	result.num_spheres = static_cast<uint>(num_spheres);
	result.spheres_xyd = spheres.clone();
	result.spheres_xyz_per_mm = spheres_xyz_per_mm;
	result.ordered_sides_per_mm = ordered_sides_per_mm;
	result.map_per_mm = map_per_mm;


	//Free memory
	delete[] rawFrame->pDepth;
	delete rawFrame;

	return true;
}

bool IRToolTracker::AddTool(cv::Mat3f spheres, float sphere_radius, std::string identifier, uint min_visible_spheres, float lowpass_rotation, float lowpass_position)
{
	//Do we already have this tool?
	if (m_ToolIndexMapping.count(identifier) > 0)
		return false;

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
	m_ToolIndexMapping.insert({ identifier, m_Tools.size() });
	m_Tools.push_back(tool);
	std::cout<<"Added "<<identifier<<" with "<<tool.num_spheres<<" spheres"<<std::endl;

	if (restartTracking) {
		StartTracking();
	}
	return true;
}

bool IRToolTracker::RemoveTool(std::string identifier)
{
	//Do we even have this tool?
	if (m_ToolIndexMapping.count(identifier) == 0)
		return false;


	auto it = m_ToolIndexMapping.find(identifier);
	int index = it->second;

	bool restartTracking = false;
	if (m_bIsCurrentlyTracking) {
		restartTracking = true;
		StopTracking();
	}

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

	std::cout << "Removed " << identifier  << std::endl;

	if (restartTracking) {

		StartTracking();
	}
	return true;
}

bool IRToolTracker::RemoveAllTools()
{
	m_Tools.clear();
	m_ToolIndexMapping.clear();
	return true;
}

bool IRToolTracker::StartTracking() {
	if (m_bIsCurrentlyTracking || m_Tools.size() == 0)
		return false;
	m_bShouldStop = false;
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
	Eigen::Vector3f sumSq(0, 0, 0);
	for (const auto& point : points) {
		Eigen::Vector3f diff = point - mean;
		sumSq += diff.cwiseProduct(diff);
	}
	return (sumSq / points.size()).cwiseSqrt();
}

Eigen::Vector3f calculateMean(const std::vector<Eigen::Vector3f>& points) {
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

	while (!m_bShouldStop) {
		m_MutexCurFrame.lock();
		if (m_CurrentFrame == nullptr) {
			m_MutexCurFrame.unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}
		
		
		//Copy pointer to frame
		AHATFrame* rawFrame = m_CurrentFrame;
		m_CurrentFrame = nullptr;
		m_MutexCurFrame.unlock();

		ProcessedAHATFrame processedFrame;

		if (!ProcessFrame(rawFrame, processedFrame)) {
			continue;
		}
		processedFrames.push_back(processedFrame);

		// If enough data is collected, perform calibration
		if (processedFrames.size() == MAX_CALIBRATION_FRAMES) {
			m_bShouldStop = true;
			break;
		}
	}

	// Perform calibration, loop through processedFrames
	std::vector<std::vector<Eigen::Vector3f>> markerPoints;
	markerPoints.resize(NUM_CALIBRATION_SPHERES);
	for (ProcessedAHATFrame frame : processedFrames)
	{
		auto it_sides = frame.ordered_sides_per_mm.find(m_fCalibrationSphereRadius);
		std::vector<Side> frame_ordered_sides = it_sides->second;

		auto it_spheres_xyz = frame.spheres_xyz_per_mm.find(m_fCalibrationSphereRadius);
		cv::Mat3f frame_spheres_xyz = it_spheres_xyz->second;

		Side shortest = frame_ordered_sides.front();
    	Side longest = frame_ordered_sides.back();

		if (std::isnan(longest.distance) || std::isnan(shortest.distance))
		{
			continue; 
		}

		int marker1ID = -1;
		int marker2ID = -1;
		int marker3ID = -1;
		int marker4ID = -1;

		// Check if id_from or id_to of the longest side matches with any id of the shortest side
		if (longest.id_from == shortest.id_from || longest.id_from == shortest.id_to)
		{
			marker1ID = longest.id_from;
			marker2ID = longest.id_to;
			marker3ID = longest.id_from == shortest.id_from ? shortest.id_to : shortest.id_from;
		}
		else if (longest.id_to == shortest.id_from || longest.id_to == shortest.id_to)
		{
			marker1ID = longest.id_to;
			marker2ID = longest.id_from;
			marker3ID = longest.id_to == shortest.id_from ? shortest.id_to : shortest.id_from;
		}

		std::set<int> remainingMarkers{0,1,2,3};

		// Remove Marker 1, 2, and 3 IDs
		remainingMarkers.erase(marker1ID);
		remainingMarkers.erase(marker2ID);
		remainingMarkers.erase(marker3ID);
		marker4ID = *remainingMarkers.begin();

		// Convert cv::Vec3f to Eigen::Vector3f
		Eigen::Vector3f marker1 = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(marker1ID));
		Eigen::Vector3f marker2 = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(marker2ID));
		Eigen::Vector3f marker3 = cvVec3fToEigen(frame_spheres_xyz.at<cv::Vec3f>(marker3ID));

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
		// Marker 1
		Eigen::Vector4f marker1_homogeneous(marker1[0], marker1[1], marker1[2], 1.0f);
		Eigen::Vector4f marker1_tool = transformation * marker1_homogeneous;
		markerPoints[0].push_back(marker1_tool.head<3>());

		// Marker 2
		Eigen::Vector4f marker2_homogeneous(marker2[0], marker2[1], marker2[2], 1.0f);
		Eigen::Vector4f marker2_tool = transformation * marker2_homogeneous;
		markerPoints[1].push_back(marker2_tool.head<3>());

		// Marker 3
		Eigen::Vector4f marker3_homogeneous(marker3[0], marker3[1], marker3[2], 1.0f);
		Eigen::Vector4f marker3_tool = transformation * marker3_homogeneous;
		markerPoints[2].push_back(marker3_tool.head<3>());

		// Marker 4
		Eigen::Vector4f marker4_homogeneous(frame_spheres_xyz.at<cv::Vec3f>(marker4ID)[0], frame_spheres_xyz.at<cv::Vec3f>(marker4ID)[1], frame_spheres_xyz.at<cv::Vec3f>(marker4ID)[2], 1.0f);
		Eigen::Vector4f marker4_tool = transformation * marker4_homogeneous;
		markerPoints[3].push_back(marker4_tool.head<3>());
	}

	processedFrames.clear();

	// Calculate the average marker positions
	markerPositions.clear();
	markerPositions.reserve(NUM_CALIBRATION_SPHERES * 3);
	for (int i = 0; i < NUM_CALIBRATION_SPHERES; i++)
	{
		Eigen::Vector3f filteredMean = calculateMean(removeOutliers(markerPoints[i], 2.0f));
		
		float x, y, z;
		x = std::round(filteredMean[0] * 1000.0f) / 1000.0f;
		y = std::round(filteredMean[1] * 1000.0f) / 1000.0f;
		z = std::round(filteredMean[2] * 1000.0f) / 1000.0f;

		markerPositions.push_back((x == -0.0f) ? 0.0f : x);
		markerPositions.push_back((y == -0.0f) ? 0.0f : y);
		markerPositions.push_back((z == -0.0f) ? 0.0f : z);
		std::cout << "Marker " << i << ": " << markerPositions[i * 3] << ", " << markerPositions[i * 3 + 1] << ", " << markerPositions[i * 3 + 2] << std::endl;
	}

	m_bIsCurrentlyCalibrating = false;
}


void IRToolTracker::StopCalibration()
{
	m_bShouldStop = true;
	JoinThread(m_CalibrationThread);
}