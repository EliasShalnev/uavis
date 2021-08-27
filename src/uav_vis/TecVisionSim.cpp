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

    auto ppm = eval_ppm(uavCoord, targetCoord);
    if(ppm >= min_ppm) { ROS_INFO_STREAM("ppm: " << ppm); }
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
    const auto uavHeight = uavCoord->z;
    const auto targetHeight = targetCoord->z;
    const auto heightDiff = uavHeight - targetHeight;
    if(heightDiff < 0) { 
        ROS_ERROR_STREAM("UAV height is lesser than target");
        ROS_ERROR_STREAM(" UAV height: " << uavHeight << " Target height: " << targetHeight);
        return false;
    }

    const auto temp = (heightDiff/CameraParameters::f - 1)/CameraParameters::R;
    const auto A_x = CameraParameters::S_x*(temp);
    const auto A_y = CameraParameters::S_y*(temp);

    const auto target_x = targetCoord->x;
    const auto target_y = targetCoord->y;

    const auto uav_x = uavCoord->x;
    const auto uav_y = uavCoord->y;

    const auto min_x = uav_x - A_x/2;
    const auto min_y = uav_y - A_y/2;

    const auto max_x = uav_x + A_x/2;
    const auto max_y = uav_y + A_y/2;

    ROS_INFO_STREAM("UAV x = " << uav_x << " UAV y = " << uav_y << " UAV z = " << uavHeight);

    if( (min_x < target_x) && (target_x < max_x) && 
        (min_y < target_y) && (target_y < max_y) )
    {
        ROS_INFO_STREAM( "target_x = " << target_x << 
                         " min_x = " << min_x << 
                         " max_x = " << max_x);
        ROS_INFO_STREAM("target_y = " << target_y << 
                        " min_y = " << min_y << 
                        " max_y = " << max_y);
        ROS_INFO_STREAM("target_z = " << targetHeight);
    } else {
        ROS_ERROR_STREAM( "target_x = " << target_x << 
                          " min_x = " << min_x << 
                          " max_x = " << max_x);
        ROS_ERROR_STREAM("target_y = " << target_y << 
                         " min_y = " << min_y << 
                         " max_y = " << max_y);
        ROS_ERROR_STREAM("target_z = " << targetHeight);
    }

    return (min_x < target_x) && (target_x < max_x) && 
           (min_y < target_y) && (target_y < max_y);
}


inline double TecVisionSim::evalDistance(const UavCoordinates& uavCoord, 
                                         const TargetCoordinates& targetCoord) const
{
    const double uavX = uavCoord->x;
    const double targetX = targetCoord->x;
    const double deltaX = uavX-targetX;

    const double uavY = uavCoord->y;
    const double targetY = targetCoord->y;
    const double deltaY = uavX-targetY;

    const double uavHeight = uavCoord->z;

    const double meters = std::sqrt( ( std::pow(deltaX, 2) + 
                                       std::pow(deltaY, 2) + 
                                       std::pow(uavHeight, 2) ) );
    return meters;
}


inline double TecVisionSim::eval_ppm(const UavCoordinates& uavCoord,
                                     const TargetCoordinates& targetCoord) const
{
    const auto uavHeight = uavCoord->z;
    const auto targetHeight = targetCoord->z;
    const auto heightDiff = uavHeight - targetHeight;
    if(heightDiff < 0) { 
        ROS_ERROR_STREAM("UAV height is lesser than target");
        ROS_ERROR_STREAM(" UAV height: " << uavHeight << " Target height: " << targetHeight);
        return false;
    }
    return CameraParameters::R/(heightDiff/CameraParameters::f - 1);
}


inline int TecVisionSim::getRandom() const 
{
    std::random_device rd;
    std::mt19937 gen( rd() );
    std::uniform_int_distribution distrib(1, 100);

    return distrib(gen);
}
