#include <ros/init.h>

#include "uav_vis/UavVis.h"

const std::string nodeName = "uav_vis";

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    UavVis uavvis(ros::this_node::getNamespace());

    ros::spin();
    return 0;
}