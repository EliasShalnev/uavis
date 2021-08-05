#include <ros/init.h>



#include "uav_vis/UavVis.h"
#include "uav_vis/CameraHandle.h"


//"udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false"

//gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! queue ! autovideoconvert ! pngenc ! multifilesink location="frame%d.png"

const std::string nodeName = "uav_vis";

// GstFlowReturn new_frame(GstElement *frameSink, gpointer user_data)
// {
//   GstSample* sample;
  
//   /* Retrieve the buffer */
//   g_signal_emit_by_name (frameSink, "pull-sample", &sample);
//   if (!sample) { return GST_FLOW_ERROR; }


//   GstCaps* caps = gst_sample_get_caps (sample);
//   if (!caps) {
//     g_print ("could not get snapshot format\n");
//     exit (-1);
//   }
//   GstStructure* structure = gst_caps_get_structure(caps, 0);

//   /* we need to get the final caps on the buffer to get the size */
//   gint width, height;
//   gboolean res = gst_structure_get_int (structure, "width", &width);
//   res |= gst_structure_get_int (structure, "height", &height);
//   if (!res) {
//     g_print ("could not get snapshot dimension\n");
//     exit (-1);
//   }


//   GstBuffer* frameBuffer = gst_sample_get_buffer(sample);
//   GstMapInfo map;
//   if ( gst_buffer_map (frameBuffer, &map, GST_MAP_READ) ) 
//   {
//     auto pixbuf = gdk_pixbuf_new_from_data (map.data,
//            GDK_COLORSPACE_RGB, FALSE, 8, width, height,
//            GST_ROUND_UP_4 (width * 3), NULL, NULL);
//     GError *error = NULL;
//     /* save the pixbuf */
//     gdk_pixbuf_save (pixbuf, "/home/def/tmp/gstreamer-test/snapshot.png", "png", &error, NULL);
//     if (error != NULL)
//     {
//       g_print ("Error: %s\n", error->message);
//       g_error_free (error);
//     }

//     // g_print("%i\n", map.size);
//   }


//   gst_sample_unref (sample);

//   return GST_FLOW_OK;
// }

// /* This function is called when an error message is posted on the bus */
// static void error_cb (GstBus *bus, GstMessage *msg) {
//   GError *err;
//   gchar *debug_info;

//   /* Print error details on the screen */
//   gst_message_parse_error (msg, &err, &debug_info);
//   g_printerr ("Error received from element %s: %s\n", GST_OBJECT_NAME (msg->src), err->message);
//   g_printerr ("Debugging information: %s\n", debug_info ? debug_info : "none");
//   g_clear_error (&err);
//   g_free (debug_info);
//   // g_main_loop_quit (data->main_loop);
// }

// int main(int argc, char **argv)
// {
//   /* Initialize GStreamer */
//   gst_init (&argc, &argv);

//   gchar* desrc = g_strdup_printf (
//     "udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264"  
//     "! rtph264depay ! avdec_h264 ! videoconvert ! videoscale ! appsink name=frame_sink"
//   );

//   GError* error = NULL;
//   GstElement* pipeline = gst_parse_launch(desrc, &error);

//   if (error != NULL)
//   {
//     g_print ("could not construct pipeline: %s\n", error->message);
//     g_error_free (error);
//     return -1;
//   }

//   /* get sink */
//   GstElement* frameSink = gst_bin_get_by_name(GST_BIN(pipeline), "frame_sink");

//   /* set to PAUSED to make the first frame arrive in the sink */
//   GstStateChangeReturn ret = gst_element_set_state (pipeline, GST_STATE_PAUSED);
//   if(ret == GST_STATE_CHANGE_FAILURE)
//   {
//     g_printerr ("Unable to set the pipeline to the playing state.\n");
//     gst_object_unref (pipeline);
//     return -1;
//   }

//   g_object_set(frameSink, "emit-signals", TRUE, NULL);
//   g_signal_connect (frameSink, "new-sample", G_CALLBACK (new_frame), NULL);


//   /* Instruct the bus to emit signals for each received message, and connect to the interesting signals */
//   GstBus* bus = gst_element_get_bus( pipeline );
//   gst_bus_add_signal_watch(bus);
//   g_signal_connect (G_OBJECT (bus), "message::error", (GCallback)error_cb, NULL);
//   g_signal_connect (G_OBJECT (bus), "message::eos", (GCallback)error_cb, NULL);
//   gst_object_unref(bus);

//   /* Start playing the pipeline */
//   gst_element_set_state(pipeline, GST_STATE_PLAYING);

//   /* Create a GLib Main Loop and set it to run */
//   GMainLoop* main_loop = g_main_loop_new (NULL, FALSE);
//   g_main_loop_run(main_loop);

//   return 0;
// }


// /* This function is called when an error message is posted on the bus */
static void error_cb (GstBus *bus, GstMessage *msg)
{
  GError* err;
  gchar* debug_info;

  /* Print error details on the screen */
  gst_message_parse_error (msg, &err, &debug_info);
  g_printerr ("Error received from element %s: %s\n", GST_OBJECT_NAME (msg->src), err->message);
  g_printerr ("Debugging information: %s\n", debug_info ? debug_info : "none");
  g_clear_error (&err);
  g_free (debug_info);
}

int main(int argc, char **argv)
{
  // /* Initialize GStreamer */
  // gst_init (&argc, &argv);

  // gchar* desrc = g_strdup_printf (
  //   "udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264"  
  //   "! rtph264depay ! avdec_h264 ! videoconvert ! jpegenc ! multifilesink location=/home/def/tmp/gstreamer-test/frame%%d.jpeg");

  // GError* error = NULL;
  // GstElement* pipeline = gst_parse_launch(desrc, &error);
  // if (error != NULL)
  // {
  //   g_print ("could not construct pipeline: %s\n", error->message);
  //   // g_free(desrc);
  //   g_error_free (error);
  //   return -1;
  // }
  // // g_free(desrc);

  // /* Instruct the bus to emit signals for each received message, and connect to the interesting signals */
  // GstBus* bus = gst_element_get_bus( pipeline );
  // gst_bus_add_signal_watch(bus);
  // g_signal_connect (G_OBJECT (bus), "message::error", (GCallback)error_cb, NULL);

  // GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  // if(ret == GST_STATE_CHANGE_FAILURE)
  // {
  //   g_printerr ("Unable to set the pipeline to the playing state.\n");
  //   gst_object_unref (pipeline);
  //   return -1;
  // }

  // GMainLoop* main_loop = g_main_loop_new (NULL, FALSE);
  // g_main_loop_run(main_loop);
  
  // /* Free resources */
  // gst_object_unref (bus);
  // gst_element_set_state (pipeline, GST_STATE_NULL);
  // gst_object_unref (pipeline);
  // return 0;

    
    CameraHandle cameraHandle;
    std::thread gstreamerThread(&CameraHandle::initPipeline, &cameraHandle);
    gstreamerThread.detach();

    while(cameraHandle.isGMainLoopRunning() == false) { 
      std::this_thread::sleep_for( std::chrono::seconds(1) ); 
    }
  
    if( cameraHandle.isGMainLoopRunning() ) { ROS_INFO_STREAM("GMainLoop is running"); }
    else { ROS_ERROR_STREAM("GMainLoop is not running"); }

    for(int i=0; i<100; ++i)
    {
      cameraHandle.saveCapture(i);
      std::this_thread::sleep_for( std::chrono::milliseconds(100) );
    }
    // cameraHandle.saveCapture(2);
    /******************************/
    ros::init(argc, argv, nodeName);

    // UavVis uavvis(ros::this_node::getNamespace());

    ros::spin();
    return 0;
}
