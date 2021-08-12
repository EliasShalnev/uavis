#pragma once

#include <gst/gst.h>

#include <thread>
#include <mutex>

struct Context
{
    
    GstElement* m_pipeline = NULL;
    GMainLoop* m_gMainLoop = NULL;
    std::pair<char*, size_t> m_currentFrame {NULL, 0};
    std::thread m_gstreamerThread;
    std::mutex m_mutex;
    uint16_t m_cameraPort;
};

class CameraHandle
{
public:
    CameraHandle(const uint16_t port);
    ~CameraHandle();

    bool isGMainLoopRunning();

    void saveFrame(uint32_t frameNum);

private:
    bool writeOutFile(const std::string& outfile, char* buf, const size_t& len);

private:
    Context m_context;

   
    const std::string m_frameDir;
};
