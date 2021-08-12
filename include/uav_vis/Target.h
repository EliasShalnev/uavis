#pragma once

#include <string>
#include <memory>

#include <geometry_msgs/Point.h>

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
    bool isActive() const { return m_coordinates.getNumPublishers() != 0; }

    geometry_msgs::Point::ConstPtr getCoordinates() const;

private:
    ros::NodeHandle m_nh;
    const std::string m_targetPoseTopicName;
    SubMonitor<geometry_msgs::Point> m_coordinates;
};
