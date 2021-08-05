#include "uav_vis/CameraHandle.h"

#include <ros/io.h>

CameraHandle::~CameraHandle() 
{
    if(m_pipeline != NULL)
    {
        gst_element_set_state(m_pipeline, GST_STATE_NULL);
        gst_object_unref(m_pipeline); 
    }
    //TODO - check is GMainLoop pointer supposed to be unref
}


void CameraHandle::saveCapture(guint32 frameNum) 
{
    std::lock_guard<std::mutex> lk(m_mutex);
    /* get sink */
    GstElement* frameSink = gst_bin_get_by_name(GST_BIN(m_pipeline), "multi_file_sink");
    ROS_INFO_STREAM("frame: " << frameNum);
    /* set location property */
    gchar* location = g_strdup_printf("/home/def/tmp/gstreamer-test/frame%u.jpeg", frameNum);
    g_object_set(frameSink, "location", location, NULL);
    g_free(location);
}


void CameraHandle::initPipeline() 
{
    /* Initialize GStreamer */
    gst_init (NULL, NULL);

    /* Pipeline launch */
    gchar* pipelineDescription = g_strdup_printf (
        "udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264"  
        "! rtph264depay ! avdec_h264 ! videoconvert ! jpegenc" 
        "! multifilesink name=multi_file_sink");
    GError* error = NULL;
    m_pipeline = gst_parse_launch(pipelineDescription, &error);
    if (error != NULL)
    {
        ROS_ERROR_STREAM("could not construct pipeline: %s\n", error->message);
        g_error_free(error);
        g_free(pipelineDescription);
        return;
    }
    g_free(pipelineDescription);

    /* Start playing pipeline */
    GstStateChangeReturn ret = gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
    if(ret != GST_STATE_CHANGE_ASYNC)
    {
        ROS_ERROR_STREAM("State change return should be: GST_STATE_CHANGE_ASYNC.\n");
        return;
    }
    /* Wait for state change */
    ret = gst_element_get_state(m_pipeline, NULL, NULL, -1);
    if(ret == GST_STATE_CHANGE_FAILURE)
    {
        ROS_ERROR_STREAM("Unable to set the pipeline to the playing state.\n");
        return;
    }

    m_gMainLoop = g_main_loop_new (NULL, FALSE);
    g_main_loop_run(m_gMainLoop);
}


bool CameraHandle::isGMainLoopRunning() 
{
    std::lock_guard<std::mutex> lk(m_mutex);
    if(m_gMainLoop == NULL) { return false; }
    return g_main_loop_is_running(m_gMainLoop);
}