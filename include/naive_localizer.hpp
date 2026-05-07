#pragma once

#include <vector>
#include "map.hpp"
#include "lidar.hpp"

class NaiveLocalizer
{
public:
    NaiveLocalizer(const Map& map, Lidar& lidar, 
                   double pos_step = 0.2, double theta_step = 0.1);

    void update(const std::vector<double>& actual_scan);

    Pose get_estimated_pose() const;

private:
    const Map& map_;
    Lidar& lidar_;
    
    double pos_step_;   // Дискретизация пространства (метры)
    double theta_step_; // Дискретизация угла (радианы)
    
    Pose estimated_pose_{0.0, 0.0, 0.0};
};