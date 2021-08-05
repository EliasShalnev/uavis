#pragma once


#include <gst/gst.h>

#include <condition_variable>
#include <thread>


class CameraHandle
{
public:
    CameraHandle() = default;
    ~CameraHandle();

    void saveCapture(guint32 frameNum);

    void initPipeline();

    bool isGMainLoopRunning(); 

private:
    GMainLoop* m_gMainLoop = NULL;
    GstElement* m_pipeline = NULL;
    std::mutex m_mutex;
};
