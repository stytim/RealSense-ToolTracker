#pragma once

#ifndef VIEWER_WINDOW_H
#define VIEWER_WINDOW_H

#include <GLFW/glfw3.h>
#include <vector>
#include <string>
#include <nanosockets.h> 
#include "IRToolTracking.h"

class ViewerWindow {

    struct TrackingData
    {
        long long serialNumber; // Serial number
        double timestamp;    // Timestamp
        float position[3];   // x, y, z
        float quaternion[4]; // x, y, z, w
        int toolId;          // Tool ID
    };

public:
    void Initialize(const std::string& file);
    void Shutdown();

    bool IsTerminated() const
    {
        return Terminated;
    }

private:
    void Render();
    void StopRender();
    void SaveToolDefinition(const Tool &tool);
    bool LoadToolDefinition();

    void UdpThreadFunction();
    void UdpReceiveThreadFunction();

    void WriteToCSV();

    bool Connect(NanoSocket& _socket, NanoAddress& address, const char* host, int port, bool& _connected);
    void Disconnect(NanoSocket& _socket, bool& _connected);

    void GetSerialNumber();
    Eigen::Matrix4f TrackingDataToMatrix(const TrackingData &data);


    std::atomic<bool> Terminated = ATOMIC_VAR_INIT(false);
    std::shared_ptr<std::thread> Thread;
    std::shared_ptr<std::thread> processingThread;

    std::shared_ptr<std::thread> udpThread;
    bool udpEnabled = false;

    std::shared_ptr<std::thread> udpReceiveThread;
    bool multiEnabled = false;

    std::shared_ptr<std::thread> csvThread;
    bool csvEnabled = false;

    std::map<long long, Eigen::Matrix4f> extrinsics;

    GLFWwindow* window = nullptr;
    GLuint texture = 0, dtexture = 0;
    std::vector<Tool> tools;
    std::map<int, Eigen::Matrix4f> toolTransforms;
    std::mutex secondaryDataMutex;

    int numTools = 0; // Default number of tools
    std::string toolName = "Tool1"; // Default tool name
    int toolId = 0; // Default tool id
    std::vector<float> spherePositions; 

    std::string toolDefinitionFileName = "tool_definition.bin";
    static const int MAX_TOOL_NAME_LENGTH = 20;

    std::string recordedFile = "";

    IRToolTracking tracker;
    int irThreshold = 100;
    int laserPower = 300;
    int minlasPower = 0;
    int maxlasPower = 360;
    int minPixelSize = 10;
    int maxPixelSize = 300;
    bool isToolAdded = false;
    bool calibrationInitiated = false;

    std::string currentDevice;
    long long serialNumber = 0;

    NanoSocket socket;
    NanoSocket receiveSocket;
    NanoAddress sendAddress = {};
    NanoAddress receiveAddress = {};
    bool m_connected;
    bool m_receiveconnected;
    char ipAddress[16] = "127.0.0.1"; 
    int m_port = 12345;
    int m_receiveport = 12345;
    int frequency = 100;
    int recordFrequency = 10;
    int duration = 20;
    std::string csvFileName = "tracking_data.csv";
    bool finishedRecord = false;
};

#endif // VIEWER_WINDOW_H
