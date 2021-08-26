#include <ros/init.h>
#include <ros/param.h>

#include "uav_vis/Parameters.h"
#include "uav_vis/UavVis.h"


//gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! queue ! autovideoconvert ! pngenc ! multifilesink location="frame%d.png"

const std::string nodeName = "uav_vis";


int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    auto boardName = ros::this_node::getNamespace();

    if(boardName == "/")
    {
        ROS_ERROR_STREAM("uavis should be launched with namespace");
        return -1;
    }

    if( boardName.find("/scout") == std::string::npos)
    {
        ROS_ERROR_STREAM("uavis should be launched with scout<number> namespace");
        return -1;
    }
    
    auto parameters = Parameters::getInstance();
    parameters->parseArgs(argc, argv);
    ROS_INFO_STREAM("Boardname: " << boardName);
    parameters->print();

    UavVis uavvis(boardName);
    uavvis.startSimulation();

    ros::spin();
    return 0;
}

