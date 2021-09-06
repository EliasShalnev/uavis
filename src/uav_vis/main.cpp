#include <ros/init.h>

#include "uav_vis/Parameters.h"
#include "uav_vis/UavVis.h"


const std::string nodeName = "uav_vis";


int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    auto parameters = Parameters::getInstance();
    parameters->parseArgs(argc, argv);
    parameters->print();

    UavVis uavvis( parameters->getUavModel() );
    uavvis.startSimulation();

    ros::spin();
    return 0;
}

