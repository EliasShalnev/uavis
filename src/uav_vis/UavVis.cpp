#include "uav_vis/UavVis.h"

#include <ros/master.h>

#include "uavis/TargetCoordinates.h"

#include "uav_vis/TecVisionSim.h"
#include "uav_vis/Parameters.h"


UavVis::UavVis(const BoardName& boardName)
    : m_boardName(boardName)
    , m_nh(boardName)
    , m_frameFreq( Parameters::getInstance()->getFrameProcessingTime() )
    , m_frameTimer( m_nh.createTimer(m_frameFreq, &UavVis::frameTimerCallback, this) )
    , m_targetCoordinatesPub( m_nh.advertise<uavis::TargetCoordinates>("target_coordinates", 10) )
    , m_uavCoordinates(m_nh, "mavros/local_position/pose", 10)
    , m_modelStatesSub( m_nh.subscribe<gazebo_msgs::ModelStates>
                        ("/gazebo/model_states", 10, &UavVis::checkRegisteredTargets, this) )
{ }


geometry_msgs::PoseStamped::ConstPtr UavVis::getCoordinates() const
{
    return m_uavCoordinates.getMessage();
}


void UavVis::frameTimerCallback(const ros::TimerEvent& event) 
{
    simulateVis();
}


void UavVis::simulateVis() 
{
    auto start = std::chrono::high_resolution_clock::now();
    //TODO - fix checkTarget() should be invoked 3 sec after save frame 
    if( !isActive() ) { 
        ROS_WARN_STREAM(m_boardName << " isn't active.");
        return;
    }
    
    removeUnregisteredTargets();

    uavis::TargetCoordinates msg;
    msg.frameNum = ++m_frameNum;

    auto uavCoord = getCoordinates();

    TecVisionSim tecVisionSim;

    for(auto [targetName, target] : m_targets)
    {
        auto targetCoord = target->getCoordinates();

        ROS_INFO_STREAM("Target: " << targetName);
        if( !tecVisionSim.checkTarget(uavCoord, targetCoord) ) { continue; } //проверка возможности обнаружения ЦО
        if( !tecVisionSim.generateSecondKindError() ) { ROS_ERROR_STREAM("False negative");  continue; } //симуляция ошибки первого рода 

        msg.coordinates.emplace_back(*targetCoord);

        auto movementSpeed = target->getMovementSpeed();
        msg.speed.emplace_back(*movementSpeed);
    }
    m_cameraHandle.saveFrame(msg.frameNum); //сохранение кадра в каталоге

    m_targetCoordinatesPub.publish(msg);
    
    auto stop = std::chrono::high_resolution_clock::now();

    auto simulationTime = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    ROS_INFO_STREAM( "Simul time " << simulationTime.count() << "ms" );
}


void UavVis::checkRegisteredTargets(const gazebo_msgs::ModelStates::ConstPtr& modelStates) 
{
    for(auto model : modelStates->name)
    {
        if( model.find(Target::targetNamePrefix) == std::string::npos ) { continue; }
        if( m_targets.find(model) != m_targets.end() ) { continue; }

        ROS_INFO_STREAM("New target \"" << model << "\" was founded.");
        m_targets.emplace( model, new Target(model) );
    }
}


void UavVis::removeUnregisteredTargets() 
{
    for(auto it = m_targets.begin(); it != m_targets.end(); )
    {
        Target::TargetName targetName = it->first;

        if( !it->second->isActive() )
        {
            it = m_targets.erase(it);
            
            ROS_INFO_STREAM("Target \"" << targetName << "\" was removed.");
        }
        else { ++it; }
    }
}

