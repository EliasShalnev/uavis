#include "uav_vis/TecVisionSim.h"

#include <random>

#include "uav_vis/Parameters.h"


TecVisionSim::TecVisionSim() 
    : m_firstKindError( Parameters::getInstance()->getFirstKindError() )
    , m_secondKindError( Parameters::getInstance()->getSecondKindError() )
{ }


bool TecVisionSim::checkTarget(const UavCoordinates& uavCoord, 
                               const TargetCoordinates& targetCoord) const
{
    if( !isTargetInCam(uavCoord, targetCoord) ) { return false; }

    auto ppm = eval_ppm(uavCoord);
    if(ppm >= min_ppm) { ROS_INFO_STREAM("ppm:" << ppm); }
    else { ROS_ERROR_STREAM( "ppm = " << ppm << " should be " << min_ppm); }
    return ppm >= min_ppm;
}


bool TecVisionSim::generateFirstKindError() const
{
    auto randomVal = getRandom();
    auto firstKindError = static_cast<uint8_t>(m_firstKindError*100);
    if(randomVal <= firstKindError) { return true; }
    else { return false; }
}


bool TecVisionSim::generateSecondKindError() const
{
    auto randomVal = getRandom();
    auto secondKindError = static_cast<uint8_t>(m_secondKindError*100);
    if(randomVal <= secondKindError) { return true; }
    else { return false; }
}


inline bool TecVisionSim::isTargetInCam(const UavCoordinates& uavCoord, 
                                        const TargetCoordinates& targetCoord) const 
{
    const auto height = uavCoord->pose.position.z;
    const auto A_x = CameraParameters::S_x*( (height/CameraParameters::f - 1)/CameraParameters::R );
    const auto A_y = CameraParameters::S_y*( (height/CameraParameters::f  - 1)/CameraParameters::R );

    const auto target_x = targetCoord->x;
    const auto target_y = targetCoord->y;

    const auto uav_x = uavCoord->pose.position.x;
    const auto uav_y = uavCoord->pose.position.y;

    const auto min_x = uav_x - A_x/2;
    const auto min_y = uav_y - A_y/2;

    const auto max_x = uav_x + A_x/2;
    const auto max_y = uav_y + A_y/2;

    if( (min_y < target_y) && (target_y < max_y) && 
        (min_y < target_y) && (target_y < max_y) )
    {
        ROS_INFO_STREAM( "target_x = " << target_x << 
                         " min_x = " << min_x << 
                         " max_x = " << max_x);
        ROS_INFO_STREAM("target_y = " << target_y << 
                        " min_y = " << min_y << 
                        " max_y = " << max_y);
    } else {
        ROS_ERROR_STREAM( "target_x = " << target_x << 
                          " min_x = " << min_x << 
                          " max_x = " << max_x);
        ROS_ERROR_STREAM("target_y = " << target_y << 
                         " min_y = " << min_y << 
                         " max_y = " << max_y); 
    }

    return (min_y < target_y) && (target_y < max_y) && 
           (min_y < target_y) && (target_y < max_y);
}


inline double TecVisionSim::evalDistance(const UavCoordinates& uavCoord, 
                                         const TargetCoordinates& targetCoord) const
{
    const double uavX = uavCoord->pose.position.x;
    const double targetX = targetCoord->x;
    const double deltaX = uavX-targetX;

    const double uavY = uavCoord->pose.position.y;
    const double targetY = targetCoord->y;
    const double deltaY = uavX-targetY;

    const double uavHeight = uavCoord->pose.position.z;

    const double meters = std::sqrt( ( std::pow(deltaX, 2) + std::pow(deltaY, 2) + 
                                       std::pow(uavHeight, 2) ) );
    return meters;
}


inline double TecVisionSim::eval_ppm(const UavCoordinates& uavCoord) const
{
    const auto height = uavCoord->pose.position.z;
    return CameraParameters::R/(height/CameraParameters::f - 1);
}


inline int TecVisionSim::getRandom() const 
{
    std::random_device rd;
    std::mt19937 gen( rd() );
    std::uniform_int_distribution distrib(1, 100);

    return distrib(gen);
}
