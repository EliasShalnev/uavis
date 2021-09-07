#include <ros/init.h>
#include <ros/io.h>

#include "uav_vis/Parameters.h"
#include "uav_vis/UavVis.h"


const std::string nodeName = "uav_vis";


int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    std::string boardName = ros::this_node::getNamespace();
    if( boardName.find("bomber") != std::string::npos ) 
    {
        ROS_ERROR_STREAM("Unknown board name: " << boardName 
                         << ". It should be \"/bomber\".");
        return -1;
    }

    auto parameters = Parameters::getInstance();
    parameters->parseArgs(argc, argv);
    parameters->print();

    UavVis uavvis( parameters->getUavModel() );
    uavvis.startSimulation();

    ros::spin();
    return 0;
}

