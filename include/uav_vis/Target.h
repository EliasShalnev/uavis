#pragma once

#include <string>

// #include <geometry_msgs/Twist.h>

#include "uav_vis/SubMonitor.h"

class Target
{
public:
    using TargetName = std::string;

public:
    Target(ros::NodeHandle nh, const TargetName& targetName);
    Target(const Target&) = delete;
    Target& operator=(const Target&) = delete;
    ~Target();

private:
    TargetName m_targetName;
    // SubMonitor<geometry_msgs::Twist> m_coordinates;
};

