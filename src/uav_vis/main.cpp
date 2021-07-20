#include <ros/init.h>

#include "uav_vis/TopicNameParser.h"

const std::string nodeName = "uav_vis";

int main(int argc, char **argv)
{
    // ros::init(argc, argv, nodeName);
    // ros::NodeHandle nh;

    TopicNameParser parser("/scout1/uav_com/bomber1/topic");

    for(auto segment = parser.begin(); segment != parser.end(); ++segment)
    {
        ROS_INFO_STREAM(*segment);
    }

    for(auto segment : parser)
    {
        ROS_INFO_STREAM(segment);
    }


    return 0;
}