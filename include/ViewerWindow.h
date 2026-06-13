#pragma once

#ifndef VIEWER_WINDOW_H
#define VIEWER_WINDOW_H

#include <GLFW/glfw3.h>
#include <vector>
#include <string>
#include <atomic>
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
    // Make destruction safe regardless of how Render()/main() unwound: stop every
    // worker and join it, so no joinable std::thread is ever destroyed (which would
    // call std::terminate()).
    ~ViewerWindow() { Shutdown(); }

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
    // Reference-counted nanosockets init/deinit so the library is initialized exactly
    // once while one or more channels (send/receive) are active.
    bool EnsureSocketInit();
    void ReleaseSocketInit();

    // Thread-safe snapshot of the current tool names, so worker threads never index the
    // GUI-owned `tools` vector while it is being resized/edited on the GUI thread.
    std::vector<std::string> SnapshotToolNames();

    void GetSerialNumber();
    Eigen::Matrix4f TrackingDataToMatrix(const TrackingData &data);

    // Validation bounds.
    static constexpr int MAX_TOOLS = 32;          // upper bound for "Number of Tools"
    static constexpr int MIN_SPHERES = 3;         // lower bound for spheres per tool (matches the tracker's 3-sphere minimum)
    static constexpr int MAX_SPHERES = 20;        // safety cap for manual entry / ROM
    static constexpr int MAX_CALIB_SPHERES = 6;   // a calibration must not exceed this


    std::atomic<bool> Terminated = ATOMIC_VAR_INIT(false);
    std::shared_ptr<std::thread> Thread;
    std::shared_ptr<std::thread> processingThread;

    std::shared_ptr<std::thread> udpThread;
    std::atomic<bool> udpEnabled{false};

    std::shared_ptr<std::thread> udpReceiveThread;
    std::atomic<bool> multiEnabled{false};

    std::shared_ptr<std::thread> csvThread;
    std::atomic<bool> csvEnabled{false};

    std::map<long long, Eigen::Matrix4f> extrinsics;

    GLFWwindow* window = nullptr;
    GLuint texture = 0, dtexture = 0;
    std::vector<Tool> tools;
    // Guards `tools` (resized/edited by the GUI thread, read by UDP/CSV worker threads).
    std::mutex toolsMutex;
    std::map<int, Eigen::Matrix4f> toolTransforms;
    std::mutex secondaryDataMutex;

    // Set when a calibration produced an invalid result (too many spheres / NaN /
    // no data); triggers the "Tool calibration unsuccessful" popup on the next frame.
    bool showCalibrationError = false;

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
    bool m_connected = false;
    bool m_receiveconnected = false;
    // Reference count + guard for nanosockets_initialize()/deinitialize().
    std::mutex socketInitMutex;
    int socketInitCount = 0;
    char ipAddress[16] = "127.0.0.1"; 
    int m_port = 12345;
    int m_receiveport = 12345;
    int frequency = 100;
    int recordFrequency = 10;
    int duration = 20;
    std::string csvFileName = "tracking_data.csv";
    std::atomic<bool> finishedRecord{false};
};

#endif // VIEWER_WINDOW_H
