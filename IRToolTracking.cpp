#include "IRToolTracking.h"
#include <iostream>
#include <algorithm>
#include <librealsense2/rsutil.h>

IRToolTracking::IRToolTracking() {
    // Constructor: Initialize any required variables or state
}

void IRToolTracking::initialize(int width, int height) {

    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cerr << "No RealSense devices found." << std::endl;
        Terminated = true;
        return;
    }

    dev = devices.front();
    std::string model_name = dev.get_info(RS2_CAMERA_INFO_NAME);
    if (model_name == "Intel RealSense D415") {
        irThreshold = 100;
        minSize = 10;
        maxSize = 240;
    }
    else if (model_name == "Intel RealSense D435") {
        irThreshold = 180;
        minSize = 10;
        maxSize = 270;
    }
    SetThreshold(irThreshold);
    SetMinMaxSize(minSize, maxSize);

    frame_width = width;
    frame_height = height;
    // Configure the RealSense pipeline
    config.enable_stream(RS2_STREAM_INFRARED, 1, frame_width, frame_height, RS2_FORMAT_Y8, 90);
    config.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, 90);

    Terminated = false;
}

void IRToolTracking::StartToolCalibration()
{
    if (m_IRToolTracker == nullptr)
    {
        m_IRToolTracker = std::make_shared<IRToolTracker>(this);
    }
    m_IRToolTracker->StartCalibration();
    Terminated = false;
}

void IRToolTracking::StopToolCalibration()
{
    if (m_IRToolTracker == nullptr)
        return;
    m_IRToolTracker->StopCalibration();
    Terminated = true;
}

void IRToolTracking::processStreams() {

    if (Terminated)
        return;
    // Start the pipeline
    try {
        profile = pipeline.start(config);
    } catch (const rs2::error &e) {
        std::cerr << "Error occurred during RealSense pipeline start." << std::endl;
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }
    // Warm up the device
    for (int i = 0; i < 50; i++) {
        pipeline.wait_for_frames();
    }

    // Continuously capture frames and process them
    while (!Terminated) {
        rs2::frameset frames = pipeline.wait_for_frames();

        rs2::frame ir_frame_left = frames.get_infrared_frame(1);
        rs2::frame depth_frame = frames.get_depth_frame();

        depth_frame = spat_filter.process(depth_frame); // Apply Spatial Filter
        depth_frame = temp_filter.process(depth_frame); // Apply Temporal Filter

        // Colorize the depth frame
        rs2::frame colorized_depth = color_map.colorize(depth_frame);
        cv::Mat colorized_depth_image(cv::Size(frame_width, frame_height), CV_8UC3, (void*)colorized_depth.get_data(), cv::Mat::AUTO_STEP);
        std::lock_guard<std::mutex> lock(mtx_frames);
        depthFrame = colorized_depth_image;

        // Get intrinsic parameters of the depth sensor if not already retrieved
        if (!intrinsics_found)
        {
            intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
            intrinsics_found = true;
            std::cout << "Intrinsic parameters retrieved" << std::endl;
        }

        // Get the timestamp of the current frame
        double timestamp = ir_frame_left.get_timestamp();
        long long frame_number = ir_frame_left.get_frame_number();

        // Convert RealSense frame to OpenCV matrix
        cv::Mat left_frame_image(cv::Size(frame_width, frame_height), CV_8UC1, (void*)ir_frame_left.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_frame_image(cv::Size(frame_width, frame_height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        
        if (m_IRToolTracker != nullptr && (m_IRToolTracker->IsTracking() || m_IRToolTracker->IsCalibrating()) && timestamp > m_latestTrackedFrame)
        {
            // Create a 4x4 identity matrix
            cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);
            m_IRToolTracker->AddFrame(left_frame_image.data, depth_frame_image.data, left_frame_image.cols, left_frame_image.rows, pose ,timestamp);
            m_latestTrackedFrame = timestamp;
        }
        
        trackingFrame = m_IRToolTracker->GetProcessedFrame();
    }
}

void IRToolTracking::shutdown() {
    // Clean up resources as necessary
    pipeline.stop();
    trackingFrame.release();
    depthFrame.release();
}

void IRToolTracking::StartToolTracking()
{
    if (m_IRToolTracker == nullptr)
    {
        m_IRToolTracker = std::make_shared<IRToolTracker>(this);
    }
    m_IRToolTracker->StartTracking();
    Terminated = false;
}

void IRToolTracking::StopToolTracking()
{
    if (m_IRToolTracker == nullptr)
        return;
    m_IRToolTracker->StopTracking();
    Terminated = true;
}

bool IRToolTracking::IsTrackingTools()
{
    if (m_IRToolTracker == nullptr)
    {
        return false;
    }
    return m_IRToolTracker->IsTracking();
}

void IRToolTracking::SetThreshold(int threshold)
{
    if (m_IRToolTracker == nullptr)
        return;
    irThreshold = threshold;
    m_IRToolTracker->SetThreshold(threshold);
}

int IRToolTracking::GetThreshold()
{
    return irThreshold;
}


void IRToolTracking::SetMinMaxSize(int min, int max)
{
    if (m_IRToolTracker == nullptr)
        return;
    minSize = min;
    maxSize = max;
    m_IRToolTracker->SetMinMaxSize(min, max);
}

void IRToolTracking::GetMinMaxSize(int &min, int &max)
{
    min = minSize;
    max = maxSize;
}

bool IRToolTracking::AddToolDefinition(int sphere_count, std::vector<float> sphere_positions, float sphere_radius, std::string identifier)
{
    return AddToolDefinition(sphere_count, sphere_positions, sphere_radius, identifier, sphere_count, 0.3f, 0.6f);
}

bool IRToolTracking::AddToolDefinition(int sphere_count, std::vector<float> sphere_positions, float sphere_radius, std::string identifier, int min_visible_spheres)
{
    return AddToolDefinition(sphere_count, sphere_positions, sphere_radius, identifier, min_visible_spheres, 0.3f, 0.6f);
}

bool IRToolTracking::AddToolDefinition(int sphere_count, std::vector<float> sphere_positions, float sphere_radius, std::string identifier, int min_visible_spheres, float lowpass_rotation, float lowpass_position)
{
    if (m_IRToolTracker == nullptr)
    {
        m_IRToolTracker = std::make_shared<IRToolTracker>(this);
    }
    //Minimum required spheres for a tool is 3
    if (sphere_count < 3) {
        return false;
    }
    if (sphere_positions.size() != 3 * sphere_count)
        return false;

    cv::Mat3f spheres = cv::Mat3f(sphere_count, 1);
    int j = 0;
    for (int i = 0; i < sphere_count; i++) {
        //Flip z and convert from unity meters to millimeters
        spheres.at<cv::Vec3f>(i, 0) = cv::Vec3f(sphere_positions[j], sphere_positions[j + 1], sphere_positions[j + 2]);
        j += 3;
    }
    return m_IRToolTracker->AddTool(spheres, sphere_radius, identifier, min_visible_spheres, std::clamp(lowpass_rotation, 0.f, 1.f), std::clamp(lowpass_position, 0.f, 1.f));
}

bool IRToolTracking::RemoveToolDefinition(std::string identifier)
{
    if (m_IRToolTracker == nullptr)
        return false;
    return m_IRToolTracker->RemoveTool(identifier);
}

bool IRToolTracking::RemoveAllToolDefinitions()
{
    if (m_IRToolTracker == nullptr)
        return false;
    return m_IRToolTracker->RemoveAllTools();
}

bool IRToolTracking::DeprojectPixelToPoint(float(&uvd)[3], float(&xy)[2])
{
    float point[3] = {0.f, 0.f, 0.f};
    float pixel[2] = {uvd[0], uvd[1]};
    float depth = uvd[2];
    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);
    xy[0] = point[0];
    xy[1] = point[1];

    return true;
}

bool IRToolTracking::ProjectPointToPixel(float(&xyz)[3], float(&uv)[2])
{
    float point[3] = {xyz[0], xyz[1], xyz[2]};
    float pixel[2] = {0.f, 0.f};
    rs2_project_point_to_pixel(pixel, &intrinsics, point);
    uv[0] = pixel[0];
    uv[1] = pixel[1];

    return true;
}

std::vector<float> IRToolTracking::GetToolTransform(std::string identifier)
{
    if (m_IRToolTracker == nullptr)
        return std::vector<float>(8, 0);

    cv::Mat transform = m_IRToolTracker->GetToolTransform(identifier);
    std::vector<float> array;
    if (transform.isContinuous()) {
        array.assign((float*)transform.data, (float*)transform.data + transform.total() * transform.channels());
    }
    else {
        for (int i = 0; i < transform.rows; ++i) {
            array.insert(array.end(), transform.ptr<float>(i), transform.ptr<float>(i) + transform.cols * transform.channels());
        }
    }
    return array;
}

bool IRToolTracking::IsCalibratingTool()
{
    if (m_IRToolTracker == nullptr)
    {
        return false;
    }
    return m_IRToolTracker->IsCalibrating();
}

std::vector<float> IRToolTracking::GetToolDefinition()
{
    if (m_IRToolTracker == nullptr)
        return std::vector<float>(12, -1);

    std::vector<float> array = m_IRToolTracker->GetToolDefinition();
    return array;
}

