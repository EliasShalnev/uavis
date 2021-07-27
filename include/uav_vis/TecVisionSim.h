#pragma once

#include "uav_vis/UavVis.h"
#include "uav_vis/Target.h"


namespace CameraParameters
{
    constexpr uint8_t f=35; //фокусное расстояние (мм)
    constexpr uint32_t S_x = 6400; //разрешение матрицы по горизонтали (пикс)
    constexpr uint32_t S_y = 4800; //разрешение матрицы по вертикали (пикс)
    constexpr uint32_t S = S_x*S_y; //размер фотосенсора в пикселях
    constexpr uint8_t L_x = 32; //линейный размер области фотосенсора по горизонтали (мм)
    constexpr uint8_t L_y = 24; //линейный размер области фотосенсора по вертикали (мм)
    // const uint32_t L = L_x*L_y; //линейный размер чувствительной области фотосенсора
    constexpr double R = S_x/L_x; //(пикс/мм) плотность пикселов в матрице
} 


class TecVisionSim
{
    using UavCoordinates = geometry_msgs::PoseStamped::ConstPtr;
    using TargetCoordinates = geometry_msgs::Point::ConstPtr;

public:
    TecVisionSim() = default;
    ~TecVisionSim() = default;

    bool checkTarget(const UavCoordinates& uavCoord, const TargetCoordinates& targetCoord) const;
    bool generateFirstKindError() const;
    bool generateSecondKindError() const;

private:
    inline bool isTargetInCam(const UavCoordinates& uavCoord, const TargetCoordinates& targetCoord) const;
    inline double evalDistance(const UavCoordinates& uavCoord, const TargetCoordinates& targetCoord) const; //дистанция между БпЛА и ЦО (мм)
    inline double evalP(const UavCoordinates& uavCoord, const TargetCoordinates& targetCoord) const; //размер уменьшенного изображения в пикселях
    inline int getRandom() const ;

private:

    const double minP = 10; //(пикс/мм)

    static constexpr uint8_t firstKindError = 15; //вероятность ошибки первого рода
    static constexpr uint8_t secondKindError = 75; //веростность ошибки второго рода 
};
