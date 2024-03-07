#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <filesystem>
#include <fstream>
#include <regex>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <nfd.h> 

#include "ViewerWindow.h"
#include "ROMParser.h"

#include "AppIcon.inc"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace fs = std::filesystem;

void ViewerWindow::SaveToolDefinition(const Tool &tool)
{
    fs::path toolsDir("Tools");
    if (!fs::exists(toolsDir))
    {
        fs::create_directories(toolsDir); // Create the Tools directory if it doesn't exist
    }

    // Construct the filename using the tool name
    fs::path filePath = toolsDir / (tool.toolName + ".bin");

    std::ofstream file(filePath, std::ios::out | std::ios::binary);
    if (file)
    {
        file.write(reinterpret_cast<const char *>(&tool.numSpheres), sizeof(tool.numSpheres));
        file.write(reinterpret_cast<const char *>(tool.spherePositions.data()), tool.spherePositions.size() * sizeof(float));
        file.write(reinterpret_cast<const char *>(&tool.sphereRadius), sizeof(tool.sphereRadius));
        file << tool.toolName << '\n'; // Writing tool name at the end of the file
    }
    else
    {
        std::cerr << "Failed to save tool definition: " << tool.toolName << std::endl;
    }
}

bool ViewerWindow::LoadToolDefinition()
{
    fs::path toolDir("Tools");
    if (!fs::exists(toolDir) || !fs::is_directory(toolDir))
    {
        std::cerr << "Tool directory not found." << std::endl;
        return false;
    }

    tools.clear(); // Clear existing tools
    for (const auto &entry : fs::directory_iterator(toolDir))
    {
        if (entry.path().extension() == ".bin")
        {
            std::ifstream file(entry.path(), std::ios::in | std::ios::binary);
            if (!file)
            {
                std::cerr << "Failed to open " << entry.path() << std::endl;
                continue; // Skip to the next file
            }

            Tool tool;
            file.read(reinterpret_cast<char *>(&tool.numSpheres), sizeof(tool.numSpheres));
            tool.spherePositions.resize(tool.numSpheres * 3);
            file.read(reinterpret_cast<char *>(tool.spherePositions.data()), tool.spherePositions.size() * sizeof(float));
            file.read(reinterpret_cast<char *>(&tool.sphereRadius), sizeof(tool.sphereRadius));
            std::getline(file, tool.toolName);
            if (!file)
            {
                std::cerr << "Failed to read data from " << entry.path() << std::endl;
                continue; // Skip to the next file
            }
            std::cout << "Loaded tool definition: " << tool.toolName << std::endl;
            tools.push_back(std::move(tool));
            numTools += 1;
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

    window = glfwCreateWindow( 1060, 720, "RealSense Tool Tracker", nullptr, nullptr );
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

bool ViewerWindow::Connect(NanoSocket& _socket, NanoAddress& address, const char *host, int port, bool& _connected) {
    if (udpEnabled != multiEnabled)
    {
        if (nanosockets_initialize())
        {
            std::cerr << "Error initializing socket library" << std::endl;
            return false;
        }
    }

    _socket = nanosockets_create(1024, 1024);
    if (socket < 0)
    {
        std::cerr << "Failed to create a socket." << std::endl;
        return false;
    }

    address.port = static_cast<uint16_t>(port);

    if (!host || std::strlen(host) == 0)
    {
        if (nanosockets_address_set_ip(&address, "127.0.0.1"))
        {
            std::cerr<<"Error setting default address"<<std::endl;
            return false;
        }
    }
    else
    {
        if (nanosockets_address_set_hostname(&address, host))
        {
            std::cerr << "Error setting hostname to " << host << std::endl;
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
        if (!multiEnabled && !udpEnabled)
        {
            nanosockets_deinitialize();
        }
        _connected = false;
    }
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


    while (multiEnabled)
    {
        NanoAddress sender;
		std::vector<uint8_t> buffer(sizeof(TrackingData));
        //toolTransforms.clear();
        if (nanosockets_receive(receiveSocket, &sender, buffer.data(), buffer.size()) > 0)
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
                for (size_t id = 0; id < tools.size(); ++id)
                {
                    if (data.toolId == static_cast<int>(id + 1))
                    {
                        // Always update extrinsics in case camera is moved
                        std::vector<float> tool_transform = tracker.GetToolTransform(tools[id].toolName);
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
            for (size_t id = 0; id < tools.size(); ++id)
            {
                const auto &tool = tools[id];
                std::vector<float> tool_transform = tracker.GetToolTransform(tool.toolName);

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

                        toolTransforms.clear();
                        validSecondaryData = true;
                    }
                }

                if (validPrimaryData || validSecondaryData)
                {
                    TrackingData data;
                    std::copy(tool_transform.begin(), tool_transform.begin() + 3, data.position); 
                    std::copy(tool_transform.begin() + 3, tool_transform.end(), data.quaternion); 
                    data.toolId = static_cast<int>(id) + 1;                                        
                    data.timestamp = glfwGetTime();
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

        int sleepDurationMs = 1000 / frequency; // Convert frequency to sleep duration in milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepDurationMs));
    }

    Disconnect(socket, m_connected);
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

        // Adjust the size of the tools vector based on numTools
        numTools = std::max(numTools, 1);
        if (static_cast<int>(tools.size()) != numTools)
        {
            tools.resize(numTools);
        }

        ImGui::SetNextWindowPos(ImVec2(20, 100), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(windowWidth, 0.0f));
        ImGui::Begin("Tool Definitions", nullptr, overlayFlags);

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
                tools[toolIdx].numSpheres = std::max(tools[toolIdx].numSpheres, 4);
                tools[toolIdx].spherePositions.resize(tools[toolIdx].numSpheres * 3);

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
                        tracker.AddToolDefinition(tools[toolIdx].numSpheres, tools[toolIdx].spherePositions, tools[toolIdx].sphereRadius, tools[toolIdx].toolName);
                        SaveToolDefinition(tools[toolIdx]);
                        tools[toolIdx].isAdded = true;
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

                    if (ImGui::Button(("Calibrate Tool##" + std::to_string(toolIdx)).c_str()))
                    {
                        std::cout << "Calibrating tool " << toolIdx + 1 << std::endl;
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

                    ImGui::SameLine();
                    if (ImGui::Button(("Load ROM##" + std::to_string(toolIdx)).c_str()))
                    {
                        NFD_Init();
                        nfdchar_t* path = nullptr;
                        nfdfilteritem_t filterItem[1] = { { "NDI rom file", "rom" }};
                        nfdresult_t result = NFD_OpenDialog(&path, filterItem, 1, NULL);
                        if (result == NFD_OKAY && path) {
                            ROMParser parser(path);
                            tools[toolIdx].numSpheres = parser.GetNumMarkers();
                            tools[toolIdx].spherePositions = parser.GetMarkerPositions();
                            NFD_FreePath(path);
                        }
                        NFD_Quit();
                    }
                }
            }
        }

        ImGui::End();

        if (!tracker.IsCalibratingTool() && calibrationInitiated)
        {
            std::vector<float> tool_definition = tracker.GetToolDefinition();
            if (!tool_definition.empty() && !std::isnan(tool_definition[0]) && tool_definition[0] != -1)
            {
                tools[toolId].spherePositions = tool_definition;
            }
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
        ImGui::SetNextItemWidth(110);
        ImGui::InputInt("Frequency", &frequency);
        ImGui::SameLine();
        if (ImGui::Checkbox("UDP", &udpEnabled))
        {
            if (udpEnabled)
            {
                udpThread = std::make_shared<std::thread>(&ViewerWindow::UdpThreadFunction, this);
            }
            else
            {
                udpEnabled = false;
                JoinThread(udpThread);
            }
        }
        ImGui::End();
        frequency = std::max(frequency, 1);

        ImGui::SetNextWindowSize(ImVec2(300, 0.0f));
        ImGui::SetNextWindowPos(ImVec2(740, 80), ImGuiCond_FirstUseEver);
        ImGui::Begin("Multi-Camera Settings", nullptr, overlayFlags);
        if (ImGui::Checkbox("Multi-Camera", &multiEnabled))
        {
            if (multiEnabled)
            {
                udpReceiveThread = std::make_shared<std::thread>(&ViewerWindow::UdpReceiveThreadFunction, this);
            }
            else
            {
                multiEnabled = false;
                JoinThread(udpReceiveThread);
            }
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("Receive Port", &m_receiveport, 0, 0, ImGuiInputTextFlags_CharsDecimal);
        ImGui::End();

        cv::Mat frame = tracker.getNextIRFrame();
        cv::Mat depth = tracker.getNextDepthFrame();

        if (!depth.empty() && (tracker.IsTrackingTools() || calibrationInitiated))
        {
            int windowWidth, windowHeight;
            glfwGetWindowSize(window, &windowWidth, &windowHeight);

            ImGui::SetNextWindowPos(ImVec2(20, windowHeight - 300));
            ImGui::Begin("IR Monitor", nullptr, overlayFlags);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.cols, frame.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, frame.data);
            ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(texture)), ImVec2(424, 240));
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2(480, windowHeight - 300));
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
            ImGui::SetNextWindowPos(ImVec2(400, 150), ImGuiCond_FirstUseEver);
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

                        toolTransforms.clear();
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
    tracker.StopToolTracking();
    JoinThread(processingThread);
    JoinThread(udpThread);
    JoinThread(udpReceiveThread);
    if (tracker.IsTrackingTools() || tracker.IsCalibratingTool())
        tracker.shutdown();
}