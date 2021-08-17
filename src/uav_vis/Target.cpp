#include "uav_vis/Target.h"


Target::Target(const TargetName& targetName)
    : m_targetName(targetName)
    , m_modelStates(m_nh, "/gazebo/model_states", 10)
{ }


bool Target::isActive() const
{
    for(auto model : m_modelStates.getMessage()->name)
    {
        if(model == m_targetName) { return true; }
    }
    return false;
}


geometry_msgs::Point::ConstPtr Target::getCoordinates() const 
{
    auto models = m_modelStates.getMessage()->name;
    for(unsigned int i=0; i<models.size(); ++i)
    {
        if(models[i] == m_targetName)
        {   
            auto pose = new geometry_msgs::Point(m_modelStates.getMessage()->pose[i].position);
            return geometry_msgs::Point::ConstPtr(pose);
        }
    }
    return nullptr;
}


geometry_msgs::Vector3::ConstPtr Target::getMovementSpeed() const
{
    auto models = m_modelStates.getMessage()->name;
    for(unsigned int i=0; i<models.size(); ++i)
    {
        if(models[i] == m_targetName)
        {   
            auto ms = new geometry_msgs::Vector3(m_modelStates.getMessage()->twist[i].linear);
            return geometry_msgs::Vector3::ConstPtr(ms);
        }
    }
    return nullptr;
}

