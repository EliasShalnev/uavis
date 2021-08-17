#pragma once

#include <string>
#include <unordered_map>

#include <ros/node_handle.h>

#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>

#include "uav_vis/SubMonitor.h"
#include "uav_vis/Target.h"
#include "uav_vis/CameraHandle.h"


class UavVis
{
public:
    using BoardName = std::string;

public:
    UavVis(const BoardName& boardName);
    UavVis(const UavVis&) = delete;
    UavVis& operator=(const UavVis&) = delete;
    ~UavVis() = default;

public:
    geometry_msgs::PoseStamped::ConstPtr getCoordinates() const;

    bool isActive() { return m_uavCoordinates.getNumPublishers() != 0; }

private:
    void frameTimerCallback(const ros::TimerEvent &event);

    void simulateVis();

    /**
     * @brief checks if new target model is spawned and saves it in a targets store.
     * 
     * @param modelStates 
     */
    void checkRegisteredTargets(const gazebo_msgs::ModelStates::ConstPtr &modelStates);

    /**
    * @brief checks if targets is active. Unactive targets will be removed from 
    *        targets store.  
    */
    void removeUnregisteredTargets();

private:
    const BoardName m_boardName;
    ros::NodeHandle m_nh;

    //Camera fields
    CameraHandle m_cameraHandle;
    ros::Duration m_frameFreq;
    uint32_t m_frameNum {0};
    ros::Timer m_frameTimer;

    ros::Publisher m_targetCoordinatesPub;
    
    SubMonitor<geometry_msgs::PoseStamped> m_uavCoordinates;

    //Target fields
    ros::Subscriber m_modelStatesSub;
    std::unordered_map<Target::TargetName, Target::ConstPtr> m_targets;
};