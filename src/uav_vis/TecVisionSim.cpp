#include "uav_vis/TecVisionSim.h"

#include <random>


bool TecVisionSim::checkTarget(const UavCoordinates& uavCoord, 
                               const TargetCoordinates& targetCoord) const
{
    // ROS_INFO_STREAM(evalP(uavCoord, targetCoord) << " " << minP);
    return evalP(uavCoord, targetCoord) >= minP && isTargetInCam(uavCoord, targetCoord);
}


bool TecVisionSim::generateFirstKindError() const
{
    auto randomVal = getRandom();
    ROS_INFO_STREAM(randomVal);
    if(randomVal <= firstKindError) { return true; }
    else { return false; }
}


bool TecVisionSim::generateSecondKindError() const
{
    auto randomVal = getRandom();
    if(randomVal <= secondKindError) { return true; }
    else { return false; }
}


inline bool TecVisionSim::isTargetInCam(const UavCoordinates& uavCoord, 
                                        const TargetCoordinates& targetCoord) const 
{
    const auto d = evalDistance(uavCoord, targetCoord);
    const auto A_x = CameraParameters::S_x*( (d/CameraParameters::f - 1)/CameraParameters::R );
    const auto A_y = CameraParameters::S_y*( (d/CameraParameters::f - 1)/CameraParameters::R );

    auto target_x = targetCoord->x * 1000;
    auto target_y = targetCoord->y * 1000;

    return target_x < A_x && target_y < A_y;
}


inline double TecVisionSim::evalDistance(const UavCoordinates& uavCoord, 
                                         const TargetCoordinates& targetCoord) const
{
    double uavX = uavCoord->pose.position.x;
    double targetX = targetCoord->x;
    double deltaX = uavX-targetX;

    double uavY = uavCoord->pose.position.y;
    double targetY = targetCoord->y;
    double deltaY = uavX-targetY;

    double uavHeight = uavCoord->pose.position.z;

    double meters = std::sqrt( ( std::pow(deltaX, 2) + std::pow(deltaY, 2) + 
                                 std::pow(uavHeight, 2) ) );
    return meters * 1000;
}


inline double TecVisionSim::evalP(const UavCoordinates& uav, const TargetCoordinates& target) const
{
    double d = evalDistance(uav, target);
    return (Target::LENGTH*CameraParameters::R)/(d/CameraParameters::f - 1);
}


inline int TecVisionSim::getRandom() const 
{
    std::random_device rd;
    std::mt19937 gen( rd() );
    std::uniform_int_distribution distrib(1, 100);

    return distrib(gen);
}
