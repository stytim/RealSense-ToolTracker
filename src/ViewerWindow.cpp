#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <filesystem>
#include <fstream>
#include <regex>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <nfd.h> 
#include <nlohmann/json.hpp>

#include "ViewerWindow.h"
#include "ROMParser.h"
#include "Log.h"

#include "AppIcon.inc"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"


namespace fs = std::filesystem;

// Height (px) reserved for the docked log console along the bottom edge of the
// window. The IR/Depth monitors are lifted by this amount so they never sit
// under it.
static constexpr float kLogConsoleHeight = 170.0f;

double GetCurrentUnixTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

// Base directory for app-managed data (tool definitions under Tools/, recorded
// CSVs). On macOS the app runs as a .app bundle whose working directory is "/",
// so relative paths like "Tools" would resolve under root and silently fail to
// load/save. Use a stable per-user location instead, so it behaves the same
// whether launched from Finder or a terminal. On Windows and Linux the working
// directory is kept unchanged (paths stay relative to ".").
static fs::path GetDataDirectory()
{
#if defined(__APPLE__)
    if (const char* home = std::getenv("HOME"); home && *home)
    {
        fs::path dir = fs::path(home) / "Library" / "Application Support" / "IR Tracking App";
        std::error_code ec;
        fs::create_directories(dir, ec);
        if (!ec)
        {
            return dir;
        }
    }
#endif
    return fs::path(".");
}

void ViewerWindow::SaveToolDefinition(const Tool &tool)
{
    fs::path toolsDir = GetDataDirectory() / "Tools";
    if (!fs::exists(toolsDir))
    {
        fs::create_directories(toolsDir); // Create the Tools directory if it doesn't exist
    }

    // Construct the filename using the tool name
    fs::path filePath = toolsDir / (tool.toolName + ".json");

    nlohmann::json toolJson;
    toolJson["numSpheres"] = tool.numSpheres;
    toolJson["spherePositions"] = tool.spherePositions;
    toolJson["sphereRadius"] = tool.sphereRadius;
    toolJson["toolName"] = tool.toolName;

    std::ofstream file(filePath);
    if (file)
    {
        file << toolJson.dump(4); // Save the JSON with indentation for readability
    }
    else
    {
        std::cerr << "Failed to save tool definition: " << tool.toolName << std::endl;
    }
}

bool ViewerWindow::LoadToolDefinition()
{
    fs::path toolDir = GetDataDirectory() / "Tools";
    if (!fs::exists(toolDir) || !fs::is_directory(toolDir))
    {
        std::cerr << "Tool directory not found." << std::endl;
        return false;
    }

    tools.clear(); // Clear existing tools
    for (const auto &entry : fs::directory_iterator(toolDir))
    {
        if (entry.path().extension() == ".json")
        {
            // Parse defensively: a malformed or hand-edited file must be skipped, not
            // crash startup. nlohmann throws on parse errors / missing keys / type
            // mismatches; catch per-file so one bad file doesn't take down the rest.
            try
            {
                std::ifstream file(entry.path());
                if (!file)
                {
                    std::cerr << "Failed to open " << entry.path() << std::endl;
                    continue;
                }

                nlohmann::json toolJson;
                file >> toolJson;

                Tool tool;
                tool.numSpheres = toolJson.at("numSpheres").get<int>();
                tool.spherePositions = toolJson.at("spherePositions").get<std::vector<float>>();
                tool.sphereRadius = toolJson.at("sphereRadius").get<float>();
                tool.toolName = toolJson.at("toolName").get<std::string>();

                // Reject inconsistent definitions (huge/negative counts, mismatched
                // arrays, non-finite radius) before they reach any resize/indexing.
                if (tool.numSpheres < MIN_SPHERES || tool.numSpheres > MAX_SPHERES ||
                    tool.spherePositions.size() != static_cast<size_t>(tool.numSpheres) * 3 ||
                    !std::isfinite(tool.sphereRadius) || tool.sphereRadius <= 0.f)
                {
                    std::cerr << "Skipping invalid tool definition: " << entry.path() << std::endl;
                    continue;
                }

                std::cout << "Loaded tool definition: " << tool.toolName << std::endl;
                tools.push_back(std::move(tool));
                numTools += 1;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Failed to parse " << entry.path() << ": " << e.what() << std::endl;
                continue;
            }
        }
    }

    if (tools.empty())
    {
        numTools = 1;
        return false;
    }
    
    return true;
}

void ViewerWindow::Initialize(const std::string& file) {

    recordedFile = file;
    const int init_result = glfwInit();
    if (init_result != GLFW_TRUE) {
        std::cerr <<"glfwInit failed" << std::endl;
        return;
    }

    // Decide GL+GLSL versions
    #if defined(IMGUI_IMPL_OPENGL_ES2)
        // GL ES 2.0 + GLSL 100
        const char* glsl_version = "#version 100";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    #elif defined(__APPLE__)
        // GL 3.2 + GLSL 150
        const char* glsl_version = "#version 150";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
    #else
        // GL 3.3 + GLSL 330
        const char* glsl_version = "#version 330";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
    #endif

    window = glfwCreateWindow( 1060, 900, "RealSense Tool Tracker", nullptr, nullptr );
    if (window == nullptr)
    {
        std::cerr <<"glfwCreateWindow failed" << std::endl;
        return;
    }

    GLFWimage icon;
    icon.pixels = stbi_load_from_memory(
        AppIcon_png,
        AppIcon_png_len,
        &icon.width,
        &icon.height,
        nullptr,
        4); // RGBA
    glfwSetWindowIcon(window, 1, &icon);
    free(icon.pixels);
        
    glfwMakeContextCurrent( window );
    glfwSwapInterval( 1 );

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL( window, true );
    ImGui_ImplOpenGL3_Init( glsl_version);

    // Set window close callback
    glfwSetWindowCloseCallback(window, [](GLFWwindow* window) {
        glfwSetWindowShouldClose(window, GL_TRUE); 
    });

    std::cout << "Data directory: " << GetDataDirectory().string() << std::endl;
    LoadToolDefinition();
    tracker.RemoveAllToolDefinitions();

    Terminated = false;
    Render();
}

Eigen::Matrix4f ViewerWindow::TrackingDataToMatrix(const TrackingData& data)
{
	Eigen::Quaternionf q(data.quaternion[3], data.quaternion[0], data.quaternion[1], data.quaternion[2]);
	Eigen::Vector3f t(data.position[0], data.position[1], data.position[2]);
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
	matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
	matrix.block<3, 1>(0, 3) = t;
    return matrix;
}

bool ViewerWindow::EnsureSocketInit()
{
    std::lock_guard<std::mutex> lock(socketInitMutex);
    if (socketInitCount == 0)
    {
        if (nanosockets_initialize()) // non-zero == failure
        {
            return false;
        }
    }
    ++socketInitCount;
    return true;
}

void ViewerWindow::ReleaseSocketInit()
{
    std::lock_guard<std::mutex> lock(socketInitMutex);
    if (socketInitCount > 0 && --socketInitCount == 0)
    {
        nanosockets_deinitialize();
    }
}

bool ViewerWindow::Connect(NanoSocket& _socket, NanoAddress& address, const char *host, int port, bool& _connected) {
    if (!EnsureSocketInit())
    {
        std::cerr << "Error initializing socket library" << std::endl;
        return false;
    }

    _socket = nanosockets_create(1024, 1024);
    if (_socket < 0)
    {
        std::cerr << "Failed to create a socket." << std::endl;
        ReleaseSocketInit();
        return false;
    }

    address.port = static_cast<uint16_t>(port);

    if (!host || std::strlen(host) == 0)
    {
        if (nanosockets_address_set_ip(&address, "127.0.0.1"))
        {
            std::cerr<<"Error setting default address"<<std::endl;
            nanosockets_destroy(&_socket);
            ReleaseSocketInit();
            return false;
        }
    }
    else
    {
        if (nanosockets_address_set_hostname(&address, host))
        {
            std::cerr << "Error setting hostname to " << host << std::endl;
            nanosockets_destroy(&_socket);
            ReleaseSocketInit();
            return false;
        }
    }

    _connected = true;
    return true;
}

void ViewerWindow::Disconnect(NanoSocket& _socket, bool& _connected)
{
    if (_connected)
    {
        nanosockets_destroy(&_socket);
        ReleaseSocketInit();
        _connected = false;
    }
}

std::vector<std::string> ViewerWindow::SnapshotToolNames()
{
    std::lock_guard<std::mutex> lock(toolsMutex);
    std::vector<std::string> names;
    names.reserve(tools.size());
    for (const auto &t : tools)
    {
        names.push_back(t.toolName);
    }
    return names;
}

void ViewerWindow::UdpReceiveThreadFunction()
{
    if (!Connect(receiveSocket, receiveAddress, "0.0.0.0", m_receiveport, m_receiveconnected))
    {
        std::cerr << "Failed to connect to server." << std::endl;
        multiEnabled = false;
        return;
    }

	receiveAddress.port = static_cast<uint16_t>(m_receiveport);
    
    if (nanosockets_bind(receiveSocket, &receiveAddress))
    {
		std::cerr << "Failed to bind to port " << m_port << std::endl;
        multiEnabled = false;
        Disconnect(receiveSocket, m_receiveconnected);
		return;
	}

    // Set the socket to non-blocking mode
    if (nanosockets_set_nonblocking(receiveSocket, 1) != NANOSOCKETS_STATUS_OK) {
        std::cerr << "Failed to set non-blocking mode" << std::endl;
        Disconnect(receiveSocket, m_receiveconnected);
        return;
    }


    std::vector<uint8_t> buffer(sizeof(TrackingData));
    while (multiEnabled)
    {
        // Wait up to 50 ms for a packet so the loop doesn't burn a core when idle.
        const int ready = nanosockets_poll(receiveSocket, 50);
        if (ready <= 0) {
            continue;
        }

        NanoAddress sender;
        const int received = nanosockets_receive(receiveSocket, &sender, buffer.data(), buffer.size());
        // Only accept a datagram that is exactly one TrackingData; a truncated or
        // oversized packet must not be reinterpreted as a valid struct.
        if (received == static_cast<int>(sizeof(TrackingData)))
        {
			TrackingData data;
			std::memcpy(&data, buffer.data(), sizeof(TrackingData));
			//std::cout << "Received tracking data for tool ID " << data.toolId << " from " << data.serialNumber <<std::endl;
            //print received address
            //char ip[16];
            //nanosockets_address_get_ip(&sender, ip, sizeof(ip));
            //std::cout << "Received from " << ip << ":" << sender.port << std::endl;

            if (tracker.IsTrackingTools())
            {
                std::vector<std::string> toolNames = SnapshotToolNames();
                for (size_t id = 0; id < toolNames.size(); ++id)
                {
                    if (data.toolId == static_cast<int>(id + 1))
                    {
                        // Always update extrinsics in case camera is moved
                        std::vector<float> tool_transform = tracker.GetToolTransform(toolNames[id]);
                        if (!tool_transform.empty() && !std::isnan(tool_transform[0]) && tool_transform[7] != 0)
                        {
                            // tool_transform to matrix
                            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                            transform.block<3, 3>(0, 0) = Eigen::Quaternionf(tool_transform[6], tool_transform[3], tool_transform[4], tool_transform[5]).toRotationMatrix();
                            transform.block<3, 1>(0, 3) = Eigen::Vector3f(tool_transform[0], tool_transform[1], tool_transform[2]);

                            extrinsics[data.serialNumber] =  transform * TrackingDataToMatrix(data).inverse();
                        }

                        // check if serial number is in extrinsics
                        if (extrinsics.find(data.serialNumber) == extrinsics.end())
                        {
							continue;
						}

                        Eigen::Matrix4f extrinsic = extrinsics[data.serialNumber];
                        Eigen::Matrix4f new_transform = extrinsic * TrackingDataToMatrix(data);

                        {
                            std::lock_guard<std::mutex> lock(secondaryDataMutex);
                            toolTransforms[id] = new_transform;
                        }
					}
                }
            }

		}
	}
    multiEnabled = false;
    std::cout << "Exiting UDP receive thread" << std::endl;
    Disconnect(receiveSocket, m_receiveconnected);
}

void ViewerWindow::UdpThreadFunction()
{
    if (!Connect(socket, sendAddress, ipAddress, m_port, m_connected))
    { 
        std::cerr << "Failed to connect to server." << std::endl;
        udpEnabled = false;
        return;
    }

    while (udpEnabled)
    {
        if (tracker.IsTrackingTools())
        {
            std::vector<std::string> toolNames = SnapshotToolNames();
            for (size_t id = 0; id < toolNames.size(); ++id)
            {
                std::vector<float> tool_transform = tracker.GetToolTransform(toolNames[id]);

                bool validPrimaryData = !tool_transform.empty() && !std::isnan(tool_transform[0]) && tool_transform[7] != 0;

                bool validSecondaryData = false;
                if (!validPrimaryData && multiEnabled)
                {
                    std::lock_guard<std::mutex> lock(secondaryDataMutex);
                    if (toolTransforms.find(id) != toolTransforms.end())
                    {
                        const Eigen::Matrix4f& secondaryData = toolTransforms[id];
                        Eigen::Quaternionf q(secondaryData.block<3, 3>(0, 0));
                        tool_transform = { secondaryData(0, 3), secondaryData(1, 3), secondaryData(2, 3),
                                          q.x(), q.y(), q.z(), q.w() };

                        // Consume only this tool's entry, not the whole map.
                        toolTransforms.erase(id);
                        validSecondaryData = true;
                    }
                }

                if (validPrimaryData || validSecondaryData)
                {
                    // Value-initialize so no uninitialized padding goes on the wire.
                    TrackingData data{};
                    std::copy(tool_transform.begin(), tool_transform.begin() + 3, data.position);
                    // GetToolTransform() returns 8 floats (XYZ, quaternion XYZW, visibility).
                    // Copy only the 4 quaternion components — copying through end() would
                    // write a 5th float past data.quaternion[4].
                    std::copy(tool_transform.begin() + 3, tool_transform.begin() + 7, data.quaternion);
                    data.toolId = static_cast<int>(id) + 1;                                        
                    data.timestamp = GetCurrentUnixTimestamp();
                    data.serialNumber = serialNumber;

                    if (m_connected) {
                        // Serialize data
                        std::vector<uint8_t> buffer(sizeof(TrackingData));
                        std::memcpy(buffer.data(), &data, sizeof(TrackingData));

                        // Send the serialized data
                        if (nanosockets_send(socket, &sendAddress, buffer.data(), buffer.size()) < 0)
                        {
                            std::cerr << "Failed to send tracking data for tool ID " << data.toolId << std::endl;
                        }
                    }
                }
            }
        }

        int sleepDurationMs = 1000 / std::max(frequency, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepDurationMs));
    }

    Disconnect(socket, m_connected);
}

void ViewerWindow::WriteToCSV()
{
    // Check if csvFileName exists, if so, add a number to the end of the filename.
    // A bare default filename is anchored to the app data directory; an absolute
    // path chosen via the "Save To" dialog is used as-is.
    fs::path basePath(csvFileName);
    if (basePath.is_relative())
    {
        basePath = GetDataDirectory() / basePath;
    }
    std::string stem = basePath.stem().string();
    std::string extension = basePath.extension().string();
    int count = 1;

    fs::path newFilePath = basePath;
    while (fs::exists(newFilePath)) {
        newFilePath = basePath.parent_path() / (stem + std::to_string(count) + extension);
        count++;
    }
    std::cout << "Saving to " << newFilePath << std::endl;

	std::ofstream file(newFilePath);
    if (!file)
    {
		std::cerr << "Failed to open csv" << std::endl;
		return;
	}

	file << "Serial Number,Tool ID,Timestamp,Position X,Position Y,Position Z,Quaternion X,Quaternion Y,Quaternion Z,Quaternion W\n";

    std::chrono::steady_clock::time_point start;
    bool timerStarted = false;

    while (csvEnabled)
    {
        if (tracker.IsTrackingTools())
        {
            if (!timerStarted)
            {
                start = std::chrono::steady_clock::now(); // Start the timer
                timerStarted = true;
            }
            else
            {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start);

                if (elapsed.count() >= duration) // Check if duration has been exceeded
                {
                    break; // Exit loop
                }
            }
            std::vector<std::string> toolNames = SnapshotToolNames();
            for (size_t id = 0; id < toolNames.size(); ++id)
            {
                std::vector<float> tool_transform = tracker.GetToolTransform(toolNames[id]);
                if (!tool_transform.empty() && !std::isnan(tool_transform[0]) && tool_transform[7] != 0)
                {
                    double unixTimestamp = GetCurrentUnixTimestamp();
                    file << std::fixed << std::setprecision(6) << serialNumber << "," << id + 1 << "," << unixTimestamp << ",";
                    for (size_t i = 0; i < tool_transform.size() - 1; ++i)
                    {
                        file << tool_transform[i];
                        if (i < tool_transform.size() - 2)
                        {
                            file << ",";
                        }
                    }
                    file << "\n";
                }
            }
        }

        int sleepDurationMs = 1000 / std::max(recordFrequency, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepDurationMs));
	}

	file.close();
    csvEnabled = false;
    finishedRecord = true;
}

void ViewerWindow::GetSerialNumber()
{
    std::regex re("\\b(\\d{12})\\b"); // Regular expression to match a 12-digit number
    std::smatch match;

    if (std::regex_search(currentDevice, match, re) && match.size() > 0) {
        std::string numberStr = match.str(0); // The first match in the string

        // Convert string to long long
        std::istringstream iss(numberStr);
        iss >> serialNumber;
    }
}

void ViewerWindow::Render() {

    // Generate textures
    glGenTextures(1, &texture);
    glGenTextures(1, &dtexture);

    ImGuiWindowFlags overlayFlags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                                    ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                                    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                    ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoScrollbar;

    tracker.queryDevices();
    auto& deviceList = tracker.getDeviceList();
    static int selectedDeviceIndex = deviceList.size() > 0 || recordedFile != "" ? 0: -1;
    currentDevice = deviceList.size() > 0 ? deviceList[0] : "No Device Available";
    GetSerialNumber();
    float windowWidth = 350.0f;
    // Main loop for GUI operations
    while (!Terminated && !glfwWindowShouldClose(window)) {
        

        glfwPollEvents();
        glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
        glClear( GL_COLOR_BUFFER_BIT );

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        
        ImGui::NewFrame();

        // Create ImGui window
        ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(windowWidth, 0.0f));
        ImGui::Begin("Device Selection", nullptr, overlayFlags);
        // Dropdown menu for devices
        if (ImGui::BeginCombo("Devices", currentDevice.c_str())) {
            for (int i = 0; i < static_cast<int>(deviceList.size()); i++) {
                bool isSelected = (currentDevice == deviceList[i]);
                if (ImGui::Selectable(deviceList[i].c_str(), isSelected)) {
                    currentDevice = deviceList[i];
                    selectedDeviceIndex = i;
                    std::cout << "Selected device: " << currentDevice << std::endl;
                    GetSerialNumber();
                }
                // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                if (isSelected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Refresh")) {
			tracker.queryDevices();
			deviceList = tracker.getDeviceList();
			if (deviceList.size() > 0) {
				currentDevice = deviceList[0];
				selectedDeviceIndex = 0;
                GetSerialNumber();
			}
            else
            {
                currentDevice = "No Device Available";
                selectedDeviceIndex = -1;
            }
		}

        ImGui::End();

        // Number of Tools Window
        ImGui::SetNextWindowPos(ImVec2(20, 60), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(windowWidth, 0.0f));
        ImGui::Begin("Tools to Track", nullptr, overlayFlags);
        ImGui::InputInt("Number of Tools", &numTools);
        ImGui::End();

        // Hold toolsMutex while we resize/edit `tools` so the UDP/CSV worker threads
        // never index it mid-modification. Released right after the calibration-result
        // block below.
        std::unique_lock<std::mutex> toolsLock(toolsMutex);

        // Adjust the size of the tools vector based on numTools. Clamp to a sane range:
        // an unbounded value from the InputInt would make tools.resize() allocate wildly.
        numTools = std::clamp(numTools, 1, MAX_TOOLS);
        if (static_cast<int>(tools.size()) > numTools)
        {
            // Unregister any tools that are about to be dropped so the tracker
            // doesn't keep matching ghosts.
            for (int i = numTools; i < static_cast<int>(tools.size()); ++i)
            {
                if (tools[i].isAdded)
                {
                    tracker.RemoveToolDefinition(tools[i].toolName);
                }
            }
        }
        if (static_cast<int>(tools.size()) != numTools)
        {
            tools.resize(numTools);
        }

        ImGui::SetNextWindowPos(ImVec2(20, 100), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(windowWidth, 0.0f));
        ImGui::Begin("Tool Definitions", nullptr, overlayFlags);

        isToolAdded = false;
        for (int toolIdx = 0; toolIdx < numTools; ++toolIdx)
        {
            tools[toolIdx].toolName = tools[toolIdx].toolName == "Tool" ? "Tool" + std::to_string(toolIdx + 1) : tools[toolIdx].toolName;
            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::CollapsingHeader(("Tool " + std::to_string(toolIdx + 1)).c_str()))
            {
                ImGui::InputInt(("Number of Spheres##" + std::to_string(toolIdx)).c_str(), &tools[toolIdx].numSpheres);
                ImGui::SetNextItemWidth(80);
                ImGui::InputFloat(("Sphere Radius##" + std::to_string(toolIdx)).c_str(), &tools[toolIdx].sphereRadius);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80);
                char tmpName[MAX_TOOL_NAME_LENGTH];
                strncpy(tmpName, tools[toolIdx].toolName.c_str(), MAX_TOOL_NAME_LENGTH - 1);
                tmpName[MAX_TOOL_NAME_LENGTH - 1] = '\0'; // Ensure null-termination
                ImGui::InputText(("Tool Name##" + std::to_string(toolIdx)).c_str(), tmpName, MAX_TOOL_NAME_LENGTH);
                tools[toolIdx].toolName = tmpName;
                tools[toolIdx].numSpheres = std::clamp(tools[toolIdx].numSpheres, MIN_SPHERES, MAX_SPHERES);
                tools[toolIdx].spherePositions.resize(static_cast<size_t>(tools[toolIdx].numSpheres) * 3);

                for (int i = 0; i < tools[toolIdx].numSpheres; ++i)
                {
                    std::ostringstream label;
                    label << "Sphere " << (i + 1) << " Position##" << toolIdx;
                    ImGui::InputFloat3(label.str().c_str(), &tools[toolIdx].spherePositions[i * 3]);
                }

                if (!tools[toolIdx].isAdded)
                {

                    if (ImGui::Button(("Add Tool Definition##" + std::to_string(toolIdx)).c_str()))
                    {
                        // Only persist/mark added if the tracker actually accepted it.
                        if (tracker.AddToolDefinition(tools[toolIdx].numSpheres, tools[toolIdx].spherePositions, tools[toolIdx].sphereRadius, tools[toolIdx].toolName))
                        {
                            SaveToolDefinition(tools[toolIdx]);
                            tools[toolIdx].isAdded = true;
                        }
                    }
                }
                else
                {
                    isToolAdded = true;
                    if (ImGui::Button(("Remove Tool Definition##" + std::to_string(toolIdx)).c_str()))
                    {
						tracker.RemoveToolDefinition(tools[toolIdx].toolName);
						tools[toolIdx].isAdded = false;
                        isToolAdded = false;
					}
				}

                if (!tools[toolIdx].isAdded)
                {
                    ImGui::SameLine();

                    // Calibration needs a live stream (a selected device or a recording).
                    // Without one, the old code still spawned worker threads and left the
                    // app wedged "calibrating" — and a second click overwrote a joinable
                    // std::thread, aborting the process. Also block re-entry while any
                    // session is already running.
                    const bool canCalibrate = (selectedDeviceIndex != -1 || recordedFile != "") &&
                                              !calibrationInitiated && !tracker.IsTrackingTools() &&
                                              !tracker.IsCalibratingTool();
                    ImGui::BeginDisabled(!canCalibrate);
                    if (ImGui::Button(("Calibrate Tool##" + std::to_string(toolIdx)).c_str()))
                    {
                        std::cout << "Calibrating tool " << toolIdx + 1 << std::endl;
                        // Ensure any previous streaming thread is fully joined before we
                        // replace the handle (destroying a joinable std::thread aborts).
                        JoinThread(processingThread);
                        if (recordedFile != "")
                        {
                            tracker.initializeFromFile(recordedFile);
                        }
                        else
                        {
                            tracker.initialize(selectedDeviceIndex, 848, 480);
                        }
                        tracker.StartToolCalibration();
                        calibrationInitiated = true;
                        toolId = toolIdx;
                        processingThread = std::make_shared<std::thread>([this]()
                            { this->tracker.processStreams(); });
                    }
                    ImGui::EndDisabled();

                    ImGui::SameLine();
                    if (ImGui::Button(("Load ROM##" + std::to_string(toolIdx)).c_str()))
                    {
                        NFD_Init();
                        nfdchar_t* path = nullptr;
                        nfdfilteritem_t filterItem[1] = { { "NDI rom file", "rom" }};
                        nfdresult_t result = NFD_OpenDialog(&path, filterItem, 1, NULL);
                        if (result == NFD_OKAY && path) {
                            ROMParser parser(path);
                            const int romMarkers = parser.GetNumMarkers();
                            const std::vector<float>& romPositions = parser.GetMarkerPositions();
                            // Only accept a ROM whose marker count is in range and whose
                            // position array matches; otherwise the next-frame resize and
                            // indexing would be inconsistent or oversized.
                            if (romMarkers >= MIN_SPHERES && romMarkers <= MAX_SPHERES &&
                                romPositions.size() == static_cast<size_t>(romMarkers) * 3)
                            {
                                tools[toolIdx].numSpheres = romMarkers;
                                tools[toolIdx].spherePositions = romPositions;
                            }
                            else
                            {
                                std::cerr << "Ignoring ROM with unsupported marker count: "
                                          << romMarkers << std::endl;
                            }
                            NFD_FreePath(path);
                        }
                        NFD_Quit();
                    }
                }
            }
        }

        ImGui::End();

        const bool finishCalibration = (!tracker.IsCalibratingTool() && calibrationInitiated);
        if (finishCalibration)
        {
            std::vector<float> tool_definition = tracker.GetToolDefinition();

            // Validate before accepting: a whole number of spheres in
            // [MIN_SPHERES, MAX_CALIB_SPHERES], every coordinate finite (no NaN/inf),
            // and not the "no result" sentinel. Anything else means calibration failed.
            // Note the lower bound matches MIN_SPHERES so an accepted count is never
            // bumped (and corrupted) by the per-frame sphere-count clamp above.
            bool validCalibration = false;
            const int numFloats = static_cast<int>(tool_definition.size());
            if (numFloats > 0 && numFloats % 3 == 0 && tool_definition[0] != -1)
            {
                const int sphereCount = numFloats / 3;
                if (sphereCount >= MIN_SPHERES && sphereCount <= MAX_CALIB_SPHERES)
                {
                    validCalibration = std::all_of(tool_definition.begin(), tool_definition.end(),
                                                   [](float v) { return std::isfinite(v); });
                }
            }

            if (validCalibration && toolId >= 0 && toolId < static_cast<int>(tools.size()))
            {
                tools[toolId].numSpheres = numFloats / 3;
                tools[toolId].spherePositions = tool_definition;
            }
            else
            {
                // Discard the result and prompt the user to recalibrate.
                showCalibrationError = true;
            }
        }

        // Done mutating `tools` for this frame; release before any blocking teardown
        // (JoinThread can block inside wait_for_frames up to the frame timeout, and we
        // must not stall the UDP/CSV workers on toolsMutex while it does).
        toolsLock.unlock();

        if (finishCalibration)
        {
            tracker.StopToolCalibration();
            JoinThread(processingThread);
            tracker.shutdown();
            calibrationInitiated = false;
        }

        if (isToolAdded && selectedDeviceIndex !=  -1)
        {
            ImGui::SetNextWindowPos(ImVec2(400, 100), ImGuiCond_FirstUseEver);
            ImGui::Begin("Tracking Control", nullptr, overlayFlags);
            if (!tracker.IsTrackingTools()) {
                if (ImGui::Button("Start Tracking")) {
                    // Make sure any previous streaming thread is fully joined before we
                    // replace the handle (destroying a joinable std::thread aborts).
                    JoinThread(processingThread);
                    if (recordedFile != "")
                    {
                        tracker.initializeFromFile(recordedFile);
                    }
                    else
                    {
                        tracker.initialize(selectedDeviceIndex, 848, 480);
                        tracker.getLaserPower(laserPower, minlasPower, maxlasPower);
                    }
                    tracker.StartToolTracking();
                    irThreshold = tracker.GetThreshold();
                    tracker.GetMinMaxSize(minPixelSize, maxPixelSize);  
                    processingThread = std::make_shared<std::thread>([this]() {
                    this->tracker.processStreams();
                });
                }
            } else {
                if (ImGui::Button("Stop Tracking")) {
                    tracker.StopToolTracking(); 
                    JoinThread(processingThread);
                    tracker.shutdown();               
                }
            }
            ImGui::End();
        }

        // Add a slider for setting the IR threshold
        ImGui::SetNextWindowPos(ImVec2(400, 20), ImGuiCond_FirstUseEver);
        ImGui::Begin("IR Threshold Control", nullptr, overlayFlags);
        // if not on Mac
        #if !defined(__APPLE__)
        {
            if(ImGui::SliderInt("Laser Power", &laserPower, minlasPower, maxlasPower))
            {
                tracker.setLaserPower(laserPower);
            }
        }
        #endif
        if(ImGui::SliderInt("IR Threshold", &irThreshold, 0, 255))
        {
            tracker.SetThreshold(irThreshold);
        }
        ImGui::SetNextItemWidth(100);
        if (ImGui::InputInt("Min Px", &minPixelSize))
        {
            tracker.SetMinMaxSize(minPixelSize, maxPixelSize);
        }
        ImGui::SetNextItemWidth(100);
        ImGui::SameLine();
        if (ImGui::InputInt("Max Px", &maxPixelSize))
        {
            tracker.SetMinMaxSize(minPixelSize, maxPixelSize);
        }
        ImGui::End();

        // Checkbox for enabling UDP
        ImGui::SetNextWindowSize(ImVec2(300, 0.0f));
        ImGui::SetNextWindowPos(ImVec2(740, 20), ImGuiCond_FirstUseEver);
        ImGui::Begin("UDP Settings", nullptr, overlayFlags);
        ImGui::SetNextItemWidth(110);
        ImGui::InputText("IP Address", ipAddress, sizeof(ipAddress));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("Port", &m_port, 0, 0, ImGuiInputTextFlags_CharsDecimal);
        // Keep within the valid UDP port range; the value is cast to uint16_t later.
        m_port = std::clamp(m_port, 1, 65535);
        ImGui::SetNextItemWidth(110);
        ImGui::InputInt("Frequency", &frequency);
        ImGui::SameLine();
        {
            bool udpEnabledTmp = udpEnabled.load();
            if (ImGui::Checkbox("UDP", &udpEnabledTmp))
            {
                udpEnabled.store(udpEnabledTmp);
                if (udpEnabledTmp)
                {
                    udpThread = std::make_shared<std::thread>(&ViewerWindow::UdpThreadFunction, this);
                }
                else
                {
                    JoinThread(udpThread);
                }
            }
        }
        ImGui::End();
        frequency = std::max(frequency, 1);

        ImGui::SetNextWindowSize(ImVec2(300, 0.0f));
        ImGui::SetNextWindowPos(ImVec2(740, 80), ImGuiCond_FirstUseEver);
        ImGui::Begin("Multi-Camera Settings", nullptr, overlayFlags);
        {
            bool multiEnabledTmp = multiEnabled.load();
            if (ImGui::Checkbox("Multi-Camera", &multiEnabledTmp))
            {
                multiEnabled.store(multiEnabledTmp);
                if (multiEnabledTmp)
                {
                    udpReceiveThread = std::make_shared<std::thread>(&ViewerWindow::UdpReceiveThreadFunction, this);
                }
                else
                {
                    JoinThread(udpReceiveThread);
                }
            }
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("Receive Port", &m_receiveport, 0, 0, ImGuiInputTextFlags_CharsDecimal);
        m_receiveport = std::clamp(m_receiveport, 1, 65535);
        ImGui::End();

        // GUI for CSV recording
        ImGui::SetNextWindowSize(ImVec2(300, 0.0f));
        ImGui::SetNextWindowPos(ImVec2(740, 120), ImGuiCond_FirstUseEver);
        ImGui::Begin("CSV Recording", nullptr, overlayFlags);
        ImGui::SetNextItemWidth(70);
        ImGui::InputInt("Frequency", &recordFrequency);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::InputInt("Duration", &duration);
        ImGui::SetNextItemWidth(110);
        if (ImGui::Button("Save To"))
        {
            NFD_Init();
            nfdchar_t* path = nullptr;
            nfdfilteritem_t filterItem[1] = { { "CSV file", "csv" } };
            nfdresult_t result = NFD_SaveDialog(&path, filterItem, 1, NULL, NULL);
            if (result == NFD_OKAY && path) {
                csvFileName = path;
                NFD_FreePath(path);
            }
            NFD_Quit();
        }
        ImGui::SameLine();
        {
            bool csvEnabledTmp = csvEnabled.load();
            if (ImGui::Checkbox("Record", &csvEnabledTmp))
            {
                csvEnabled.store(csvEnabledTmp);
                if (csvEnabledTmp)
                {
                    csvThread = std::make_shared<std::thread>(&ViewerWindow::WriteToCSV, this);
                }
                else
                {
                    JoinThread(csvThread);
                }
            }
        }
        ImGui::End();
        recordFrequency = std::min(std::max(recordFrequency, 1), 90);
        duration = std::max(duration, 1);

        if (finishedRecord)
        {
            JoinThread(csvThread);
            finishedRecord = false;
        }

        cv::Mat frame = tracker.getNextIRFrame();
        cv::Mat depth = tracker.getNextDepthFrame();

        if (!depth.empty() && (tracker.IsTrackingTools() || calibrationInitiated))
        {
            int windowWidth, windowHeight;
            glfwGetWindowSize(window, &windowWidth, &windowHeight);

            // Sit above the docked log console rather than under it, but never
            // off the top of the window if it has been resized very short.
            const float monitorY = std::max(0.0f,
                static_cast<float>(windowHeight) - 300.0f - kLogConsoleHeight);

            ImGui::SetNextWindowPos(ImVec2(20, monitorY));
            ImGui::Begin("IR Monitor", nullptr, overlayFlags);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.cols, frame.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, frame.data);
            ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(texture)), ImVec2(424, 240));
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2(480, monitorY));
            ImGui::Begin("Depth Monitor", nullptr, overlayFlags);
            glBindTexture(GL_TEXTURE_2D, dtexture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, depth.cols, depth.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, depth.data);
            ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(dtexture)), ImVec2(424, 240));
            ImGui::End();
        }

        if (tracker.IsTrackingTools())
        {
            ImGui::SetNextWindowPos(ImVec2(400, 200), ImGuiCond_FirstUseEver);
            ImGui::Begin("Tool Ttacking Results", nullptr, overlayFlags);

            for (size_t i = 0; i < tools.size(); ++i)
            {
                const auto &tool = tools[i];
                std::vector<float> tool_transform = tracker.GetToolTransform(tool.toolName);
                bool validPrimaryData = !tool_transform.empty() && !std::isnan(tool_transform[0]) && tool_transform[7] != 0;

                bool validSecondaryData = false;
                if (!validPrimaryData && multiEnabled)
                {
                    std::lock_guard<std::mutex> lock(secondaryDataMutex);
                    if (toolTransforms.find(i) != toolTransforms.end())
                    {
                        const Eigen::Matrix4f& secondaryData = toolTransforms[i];
                        Eigen::Quaternionf q(secondaryData.block<3, 3>(0, 0));
                        tool_transform = { secondaryData(0, 3), secondaryData(1, 3), secondaryData(2, 3),
                                          q.x(), q.y(), q.z(), q.w() };

                        // Consume only this tool's entry, not the whole map.
                        toolTransforms.erase(i);
                        validSecondaryData = true;
                    }
                }

                // Check if the tool transform is valid
                if (validPrimaryData || validSecondaryData)
                {
                    // Change text color based on the data source
                    if (validPrimaryData) {
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    }
                    else {
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 0.5f));
                    }

                    ImGui::Text("Tool %zu: %s", i + 1, tool.toolName.c_str()); // Tool number and name

                    // Display the position (XYZ)
                    ImGui::Text(" Position: X: %.3f, Y: %.3f, Z: %.3f",
                                tool_transform[0], tool_transform[1], tool_transform[2]);

                    // Display the quaternion (XYZW)
                    ImGui::Text(" Quaternion: X: %.3f, Y: %.3f, Z: %.3f, W: %.3f",
                                tool_transform[3], tool_transform[4], tool_transform[5], tool_transform[6]);

                    ImGui::PopStyleColor();
                    ImGui::Separator(); // Separate the data for each tool
                }

            }

            ImGui::End();
        }

        // Calibration failure popup: shown when a calibration produced an invalid
        // result (too many spheres, NaN/inf positions, or no data captured).
        if (showCalibrationError)
        {
            ImGui::OpenPopup("Calibration Failed");
            showCalibrationError = false;
        }
        {
            ImVec2 center = ImGui::GetMainViewport()->GetCenter();
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        }
        if (ImGui::BeginPopupModal("Calibration Failed", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Tool calibration unsuccessful.");
            ImGui::Text("Please recalibrate the tool.");
            ImGui::Separator();
            if (ImGui::Button("OK", ImVec2(120, 0)))
            {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        // Docked log console along the bottom edge: always visible, full width,
        // and re-pinned every frame so it tracks live window resizes.
        {
            int logWinWidth = 0, logWinHeight = 0;
            glfwGetWindowSize(window, &logWinWidth, &logWinHeight);
            const ImGuiWindowFlags logFlags =
                ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoFocusOnAppearing;
            ImGui::SetNextWindowPos(ImVec2(0.0f, logWinHeight - kLogConsoleHeight));
            ImGui::SetNextWindowSize(ImVec2(static_cast<float>(logWinWidth), kLogConsoleHeight));
            ImGui::Begin("Log", nullptr, logFlags);
            LogConsole::Get().DrawContents();
            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );

        glfwSwapBuffers( window );
    }

    StopRender();
    Terminated = true;
}

void ViewerWindow::StopRender() {
    glDeleteTextures(1, &texture);
    glDeleteTextures(1, &dtexture);
    ImGui_ImplGlfw_Shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

void ViewerWindow::Shutdown() {
    Terminated = true;
    udpEnabled = false;
    multiEnabled = false;
    csvEnabled = false;

    tracker.StopToolTracking();
    tracker.StopToolCalibration();
    JoinThread(processingThread);
    JoinThread(udpThread);
    JoinThread(udpReceiveThread);
    JoinThread(csvThread);

    // shutdown() is idempotent and guarded by pipeline_started, so always call it.
    // (Gating on IsTracking/IsCalibrating could miss the window where calibration
    // already finished but its completion teardown hadn't run, leaking the pipeline.)
    tracker.shutdown();
}