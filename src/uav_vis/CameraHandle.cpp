#include "uav_vis/CameraHandle.h"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ros/io.h>
#include <ros/this_node.h>


void initPipeline(Context* context);

GstFlowReturn newFrame_cb(GstElement* frameSink, Context* context);

void error_cb (GstBus* bus, GstMessage* msg, Context* context);

void eos_cb(GstBus* bus, GstMessage* msg, Context* context);


CameraHandle::CameraHandle() 
    : m_frameDir( getpwuid( getuid() )->pw_dir + ros::this_node::getNamespace() + "/frames" )
{
    m_context.m_gstreamerThread = std::thread(initPipeline, &m_context);

    while(CameraHandle::isGMainLoopRunning() == false)
    { 
        std::this_thread::sleep_for( std::chrono::seconds(1) );
    }
    m_context.m_gstreamerThread.detach();
}


CameraHandle::~CameraHandle() 
{
    /* GMainLoop supposed to be quited firstly to finish thread */
    g_main_loop_quit(m_context.m_gMainLoop);
    while(CameraHandle::isGMainLoopRunning() == true)
    {
        ROS_INFO_STREAM("Waiting...");
        std::this_thread::sleep_for( std::chrono::milliseconds(300) );
    }

    if(m_context.m_pipeline != NULL)
    {
        gst_element_set_state(m_context.m_pipeline, GST_STATE_NULL);
        gst_object_unref(m_context.m_pipeline);
    }

    auto [frameData, _] =  m_context.m_currentFrame;
    if(frameData != NULL) { delete[] frameData; }
}


bool CameraHandle::isGMainLoopRunning() 
{
    std::lock_guard<std::mutex> lk(m_context.m_mutex);
    if(m_context.m_gMainLoop == NULL) { return false; }
    return g_main_loop_is_running(m_context.m_gMainLoop);
}


void CameraHandle::saveFrame(uint32_t frameNum) 
{
    std::lock_guard<std::mutex> lk(m_context.m_mutex);

    /* set location property */
    std::string frameName("frame" + std::to_string(frameNum) + ".jpeg");
    std::string location = m_frameDir + '/' + frameName;

    auto [frameData, frameSize] = m_context.m_currentFrame;
    ROS_INFO_STREAM("******* " << frameSize);
    writeOutFile(location, frameData, frameSize);
}

bool CameraHandle::writeOutFile(const std::string& outfile, char* buf, const size_t& len)
{
    FILE* outputFile = fopen(outfile.c_str(), "wb");
	if(outputFile == NULL) {
		ROS_ERROR_STREAM("File could not be created: " << outfile);
        return false;
	}
	if(fwrite(buf, 1, len, outputFile) != len) { 
        ROS_ERROR_STREAM("Could not write to file: " << outfile);
    }

    fclose(outputFile);
}


void initPipeline(Context* context)
{
    /* Initialize GStreamer */
    gst_init (NULL, NULL);

    /* Pipeline launch */
    gchar* pipelineDescription = g_strdup_printf (
        "udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264"  
        "! rtph264depay ! avdec_h264 ! videoconvert ! jpegenc ! appsink name=frame_sink"
    );
    
    GError* error = NULL;
    context->m_pipeline = gst_parse_launch(pipelineDescription, &error);
    if (error != NULL)
    {
        ROS_ERROR_STREAM("could not construct pipeline: " << error->message);
        g_error_free(error);
        g_free(pipelineDescription);
        return;
    }
    g_free(pipelineDescription);

    /* get sink */
    GstElement* frameSink = gst_bin_get_by_name(GST_BIN(context->m_pipeline), "frame_sink");
    /* emit signal settings */
    g_object_set(frameSink, "emit-signals", TRUE, NULL);
    g_signal_connect(frameSink, "new-sample", G_CALLBACK(newFrame_cb), context);

    /* Instruct the bus to emit signals for each received message, and connect to the interesting signals */
    GstBus* bus = gst_element_get_bus( context->m_pipeline );
    gst_bus_add_signal_watch(bus);
    g_signal_connect (G_OBJECT (bus), "message::error", G_CALLBACK(error_cb), context);
    g_signal_connect (G_OBJECT (bus), "message::eos", G_CALLBACK(eos_cb), context);
    gst_object_unref(bus);

    /* Start playing pipeline */
    gst_element_set_state(context->m_pipeline, GST_STATE_PLAYING);

    /* Wait for state change */
    GstStateChangeReturn ret = gst_element_get_state(context->m_pipeline, NULL, NULL, -1);
    if(ret != GST_STATE_CHANGE_SUCCESS)
    {
        ROS_ERROR_STREAM("Unable to set the pipeline to the playing state.");
        return;
    }

    context->m_gMainLoop = g_main_loop_new (NULL, FALSE);
    g_main_loop_run(context->m_gMainLoop);
}


GstFlowReturn newFrame_cb(GstElement* frameSink, Context* context) 
{
    std::lock_guard<std::mutex> lk(context->m_mutex);
    GstSample* sample = NULL;

    /* Retrieve the buffer */
    g_signal_emit_by_name(frameSink, "pull-sample", &sample);
    if (!sample) { return GST_FLOW_ERROR; }

    GstBuffer* frameBuffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    gst_buffer_map (frameBuffer, &map, GST_MAP_READ);

    auto frameData = context->m_currentFrame.first;
    if(frameData != NULL) { delete[] frameData; }

    frameData = new char[map.size];
    // Copy image
    memmove(frameData, map.data, map.size);

    auto newPair = std::make_pair(frameData, map.size);
    context->m_currentFrame.swap(newPair);

    gst_buffer_unmap(frameBuffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}


/* This function is called when an error message is posted on the bus */
void error_cb (GstBus* bus, GstMessage* msg, Context* context)
{
    GError* err;
    gchar* debug_info;

    /* Print error details on the screen */
    gst_message_parse_error(msg, &err, &debug_info);
    ROS_ERROR("Error received from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
    ROS_ERROR("Debugging information: %s\n", debug_info ? debug_info : "none");
    g_clear_error(&err);
    g_free(debug_info);

    g_main_loop_quit(context->m_gMainLoop);
}


void eos_cb(GstBus* bus, GstMessage* msg, Context* context)
{
    ROS_WARN_STREAM("End of stream message is received.");
    g_main_loop_quit(context->m_gMainLoop);
}




// bool isGMainLoopRunning() 
// {
    // std::lock_guard<std::mutex> lk(context->m_mutex);
    // if(context->m_gMainLoop == NULL) { return false; }
    // return g_main_loop_is_running(context->m_gMainLoop);
// }




// GstFlowReturn newFrameCallback(GstElement *frameSink, Context* context);

// CameraHandle::CameraHandle() 
//     : m_frameDir( getpwuid( getuid() )->pw_dir + ros::this_node::getNamespace() + "/frames"  )
// { 
//     ROS_INFO_STREAM("Frame directory: " << m_frameDir);
// }




// void CameraHandle::saveFrame(guint32 frameNum) 
// {
//     std::lock_guard<std::mutex> lk(m_mutex);

//     /* set location property */
//     std::string frameName("frame" + std::to_string(frameNum) + ".jpeg");
//     std::string location = m_frameDir + '/' + frameName;

//     auto [frameData, frameSize] = m_CurrentFrame;
//     writeOutFile(location, frameData, frameSize);
// }


// void CameraHandle::initPipeline() 
// {
//     /* Initialize GStreamer */
//     gst_init (NULL, NULL);

//     /* Pipeline launch */
//     gchar* pipelineDescription = g_strdup_printf (
//         "udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264"  
//         "! rtph264depay ! avdec_h264 ! videoconvert ! jpegenc ! appsink name=frame_sink"
//     );
    
//     GError* error = NULL;
//     m_pipeline = gst_parse_launch(pipelineDescription, &error);
//     if (error != NULL)
//     {
//         ROS_ERROR_STREAM("could not construct pipeline: " << error->message);
//         g_error_free(error);
//         g_free(pipelineDescription);
//         return;
//     }
//     g_free(pipelineDescription);

//     /* get sink */
//     GstElement* frameSink = gst_bin_get_by_name(GST_BIN(m_pipeline), "frame_sink");

//     g_object_set(frameSink, "emit-signals", TRUE, NULL);
//     //TODO - may not correctly working 
//     g_signal_connect (frameSink, "new-sample", G_CALLBACK (&CameraHandle::newFrameCallback), NULL);

//     /* Instruct the bus to emit signals for each received message, and connect to the interesting signals */
//     // GstBus* bus = gst_element_get_bus( pipeline );
//     // gst_bus_add_signal_watch(bus);
//     // g_signal_connect (G_OBJECT (bus), "message::error", (GCallback)error_cb, NULL);
//     // // g_signal_connect (G_OBJECT (bus), "message::eos", (GCallback)error_cb, NULL);
//     // gst_object_unref(bus);


//     /* Start playing pipeline */
//     GstStateChangeReturn ret = gst_element_set_state(m_pipeline, GST_STATE_PLAYING);

//     /* Wait for state change */
//     ret = gst_element_get_state(m_pipeline, NULL, NULL, -1);
//     if(ret != GST_STATE_CHANGE_SUCCESS)
//     {
//         ROS_ERROR_STREAM("Unable to set the pipeline to the playing state.\n");
//         return;
//     }

//     m_gMainLoop = g_main_loop_new (NULL, FALSE);
//     g_main_loop_run(m_gMainLoop);
// }


// bool CameraHandle::isGMainLoopRunning() 
// {
//     std::lock_guard<std::mutex> lk(m_mutex);
//     if(m_gMainLoop == NULL) { return false; }
//     return g_main_loop_is_running(m_gMainLoop);
// }


// GstFlowReturn CameraHandle::newFrameCallback(GstElement *frameSink) 
// {
//     GstSample* sample = NULL;

//     /* Retrieve the buffer */
//     g_signal_emit_by_name (frameSink, "pull-sample", &sample);
//     if (!sample) { return GST_FLOW_ERROR; }

//     ROS_INFO_STREAM("New sample");
//     GstBuffer* frameBuffer = gst_sample_get_buffer(sample);
//     GstMapInfo map;
//     gst_buffer_map (frameBuffer, &map, GST_MAP_READ);

//     auto [frameData, frameSize] = m_CurrentFrame;
//     if(frameData != NULL) { delete[] frameData; }

//     frameData = new char[map.size];
//     // Copy image
//     memmove(frameData, map.data, map.size);
//     frameSize = map.size;
//     // writeOutFile("/home/elias/tmp/gstreamer_test/snapshot.jpeg", pRet, map.size);

//     gst_buffer_unmap (frameBuffer, &map);
//     gst_sample_unref(sample);

//     return GST_FLOW_OK;
// }


// bool CameraHandle::writeOutFile(const std::string& outfile, char* buf, const size_t& len) 
// {
//     FILE* outputFile = fopen(outfile.c_str(), "wb");
// 	if(outputFile == NULL) {
// 		ROS_ERROR_STREAM("File could not be created: " << outfile);
//         return false;
// 	}
// 	if(fwrite(buf, 1, len, outputFile) != len) { 
//         ROS_ERROR_STREAM("Could not write to file: " << outfile);
//     }

//     fclose(outputFile);
// }