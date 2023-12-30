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
        double timestamp;    // Timestamp
        float position[3];   // x, y, z
        float quaternion[4]; // x, y, z, w
        int toolId;          // Tool ID
    };

public:
    void Initialize();
    void Shutdown();

    bool IsTerminated() const
    {
        return Terminated;
    }

private:
    void ProcessInput();
    void Render();
    void StopRender();
    void SaveToolDefinition(const Tool &tool);
    bool LoadToolDefinition();

    void UdpThreadFunction();

    bool Connect(const char *host, int port);
    void Disconnect();


    std::atomic<bool> Terminated = ATOMIC_VAR_INIT(false);
    std::shared_ptr<std::thread> Thread;
    std::shared_ptr<std::thread> processingThread;

    std::shared_ptr<std::thread> udpThread;
    bool udpEnabled = false;

    GLFWwindow* window = nullptr;
    GLuint texture = 0, dtexture = 0;
    std::vector<Tool> tools;

    int numTools = 0; // Default number of tools
    int numSpheres = 4; // Default number of spheres
    float sphereRadius = 6.5f; // Default sphere radius
    std::string toolName = "Tool1"; // Default tool name
    int toolId = 0; // Default tool id
    std::vector<float> spherePositions; 

    std::string toolDefinitionFileName = "tool_definition.bin";
    static const int MAX_TOOL_NAME_LENGTH = 20;

    IRToolTracking tracker;
    int irThreshold = 100;
    int minPixelSize = 10;
    int maxPixelSize = 180;
    bool isToolAdded = false;
    bool calibrationInitiated = false;

    NanoSocket socket;
    NanoAddress address = {};
    bool m_connected;
    char ipAddress[16] = "127.0.0.1"; 
    int m_port = 12345;
    int frequency = 100;
};

#endif // VIEWER_WINDOW_H
