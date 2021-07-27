#include "uav_vis/Target.h"


Target::Target(const TargetName& targetName) 
    : m_targetPoseTopicName('/'+targetName+"/local_position")
    , m_coordinates(m_nh, m_targetPoseTopicName, 10)
{ }


geometry_msgs::Point::ConstPtr Target::getCoordinates() const 
{
    return m_coordinates.getMessage();
}

