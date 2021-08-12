#include <ros/init.h>

#include "uav_vis/UavVis.h"


//gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! queue ! autovideoconvert ! pngenc ! multifilesink location="frame%d.png"

const std::string nodeName = "uav_vis";


int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    UavVis uavvis( ros::this_node::getNamespace() );

    ros::spin();
    return 0;
}
