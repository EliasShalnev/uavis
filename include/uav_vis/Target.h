#pragma once

#include <string>
#include <memory>

#include <gazebo_msgs/ModelStates.h>

#include "uav_vis/SubMonitor.h"

class Target
{
public:
    using Ptr = std::shared_ptr<Target>;
    using ConstPtr = std::shared_ptr<const Target>;
    using TargetName = std::string;
    static constexpr char targetNamePrefix[] = "p3at";
    static constexpr uint32_t LENGTH = 4950; //мм
    static constexpr uint32_t WIDTH = 1970; //мм

public:
    Target(const TargetName& targetName);
    Target(const Target&) = delete;
    Target& operator=(const Target&) = delete;
    ~Target() = default;

    /**
     * @brief Target assume to be active while position coordinates is published 
     * 
     * @return true if active
     * @return false if isn't active 
     */
    bool isActive() const;

    geometry_msgs::Point::ConstPtr getCoordinates() const;

    geometry_msgs::Vector3::ConstPtr getMovementSpeed() const;

private:
    ros::NodeHandle m_nh;
    const std::string m_targetName;
    SubMonitor<gazebo_msgs::ModelStates> m_modelStates;
};
