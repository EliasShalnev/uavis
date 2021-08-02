#include <ros/init.h>

#include "uav_vis/UavVis.h"

#include <gst/gst.h>

//"udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false"

//gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! queue ! autovideoconvert ! pngenc ! multifilesink location="frame%d.png"

const std::string nodeName = "uav_vis";

GstFlowReturn new_frame(GstElement *frameSink, gpointer user_data)
{
  g_print("new frame");

  return GST_FLOW_OK;
}

/* This function is called when an error message is posted on the bus */
static void error_cb (GstBus *bus, GstMessage *msg) {
  GError *err;
  gchar *debug_info;

  /* Print error details on the screen */
  gst_message_parse_error (msg, &err, &debug_info);
  g_printerr ("Error received from element %s: %s\n", GST_OBJECT_NAME (msg->src), err->message);
  g_printerr ("Debugging information: %s\n", debug_info ? debug_info : "none");
  g_clear_error (&err);
  g_free (debug_info);

  // g_main_loop_quit (data->main_loop);
}

int main(int argc, char **argv)
{
  /* Initialize GStreamer */
  gst_init (&argc, &argv);

  gchar* desrc = g_strdup_printf (
    "udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264"  
    "! rtph264depay ! avdec_h264 ! queue ! autovideoconvert ! pngenc ! appsink name=frame_sink"
  );

  GError* error = NULL;
  GstElement* pipeline = gst_parse_launch(desrc, &error);

  if (error != NULL)
  {
    g_print ("could not construct pipeline: %s\n", error->message);
    g_error_free (error);
    exit (-1);
  }


  gst_element_set_state(pipeline, GST_STATE_PAUSED);
  GstElement* frameSink = gst_bin_get_by_name(GST_BIN(pipeline), "frame_sink");
  GstAppSink* appsink = (GstAppSink *) sink;
  gst_app_sink_set_max_buffers ( appsink, 2); // limit number of buffers queued
  gst_app_sink_set_drop( appsink, true ); // drop old buffers in queue when full

  // g_signal_connect (frameSink, "new-sample", G_CALLBACK (new_frame), NULL);

  // /* Instruct the bus to emit signals for each received message, and connect to the interesting signals */
  // GstBus* bus = gst_element_get_bus(pipeline);
  // gst_bus_add_signal_watch(bus);
  // g_signal_connect (G_OBJECT (bus), "message::error", (GCallback)error_cb, NULL);
  // gst_object_unref (bus);

  // /* Start playing the pipeline */
  // gst_element_set_state (pipeline, GST_STATE_PLAYING);

  // /* Create a GLib Main Loop and set it to run */
  // GMainLoop* main_loop = g_main_loop_new (NULL, FALSE);
  // g_main_loop_run(main_loop);

  return 0;
}



// int main(int argc, char **argv)
// {
//   GstElement* pipeline;
//   GstBus* bus;
//   GstMessage* msg;
//   // GstElement* app_sink;

//   /* Initialize GStreamer */
//   gst_init (&argc, &argv);

//   /* Build the pipeline */
//   pipeline =
//       gst_parse_launch
//       ("udpsrc port=5600 ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264"  
//        "! rtph264depay ! avdec_h264 ! queue ! autovideoconvert ! pngenc",
//       NULL);

//   /* Start playing */
//   gst_element_set_state (pipeline, GST_STATE_PLAYING);

//   /* Wait until error or EOS */
//   bus = gst_element_get_bus (pipeline);
//   msg =
//       gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
//       (GstMessageType) (GST_MESSAGE_ERROR | GST_MESSAGE_EOS) );

//   GMainLoop* main_loop = g_main_loop_new (NULL, FALSE);

//   g_main_loop_run(main_loop);
  
//   /* Free resources */
//   if (msg != NULL)
//     gst_message_unref (msg);
//   gst_object_unref (bus);
//   gst_element_set_state (pipeline, GST_STATE_NULL);
//   gst_object_unref (pipeline);
//   return 0;
//     /******************************/
//     // ros::init(argc, argv, nodeName);

//     // UavVis uavvis(ros::this_node::getNamespace());

//     // ros::spin();
//     // return 0;
// }
