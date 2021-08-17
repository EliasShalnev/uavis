#pragma once

#include "uav_vis/UavVis.h"
#include "uav_vis/Target.h"


namespace CameraParameters
{
    constexpr double f=0.035; //фокусное расстояние (м)
    constexpr uint32_t S_x = 6400; //разрешение матрицы по горизонтали (пикс)
    constexpr uint32_t S_y = 4800; //разрешение матрицы по вертикали (пикс)
    constexpr uint32_t S = S_x*S_y; //размер фотосенсора в пикселях
    constexpr double L_x = 0.0223; //линейный размер области фотосенсора по горизонтали (м)
    constexpr double L_y = 0.0149; //линейный размер области фотосенсора по вертикали (м)
    // const uint32_t L = L_x*L_y; //линейный размер чувствительной области фотосенсора
    constexpr double R = S_x/(L_x); //плотность пикселов в матрице (пикс/м)
} 


class TecVisionSim
{
    using UavCoordinates = geometry_msgs::PoseStamped::ConstPtr;
    using TargetCoordinates = geometry_msgs::Point::ConstPtr;

public:
    TecVisionSim();
    TecVisionSim(const TecVisionSim&) = delete;
    TecVisionSim& operator=(const TecVisionSim&) = delete;
    ~TecVisionSim() = default;

    bool checkTarget(const UavCoordinates& uavCoord, const TargetCoordinates& targetCoord) const;
    bool generateFirstKindError() const;
    bool generateSecondKindError() const;

private:
    inline bool isTargetInCam(const UavCoordinates& uavCoord, 
                              const TargetCoordinates& targetCoord) const;
    inline double evalDistance(const UavCoordinates& uavCoord, 
                               const TargetCoordinates& targetCoord) const; //дистанция между БпЛА и ЦО (м)
    inline double eval_ppm(const UavCoordinates& uavCoord, 
                        const TargetCoordinates& targetCoord) const; //размер уменьшенного изображения (пикс/метр)
    inline int getRandom() const ;

private:
    const double min_ppm = 10; //(пикс/метр) 

    const uint8_t m_firstKindError; //вероятность ошибки первого рода
    const uint8_t m_secondKindError; //веростность ошибки второго рода 
};
