#pragma once

#include <random>

#include "odometry.hpp"
#include "lidar.hpp"
#include "amcl.hpp"
#include "map.hpp"

class Robot {
public:
    Robot(double start_x, double start_y, double start_theta, const Map& map);

    void move_relative(double forward_dist, double dtheta);

    void update_sensors();

    void update_amcl();

    Pose get_real_pos     () const { return real_pose_; }
    Pose get_intended_pos () const { return odom_.get_pose(); }
    Pose get_amcl_pose    () const { return amcl_.get_estimated_pose(); }

    const Lidar& get_lidar () const { return lidar_; }
    const std::vector<Particle>& get_particles() const { return amcl_.get_particles(); }

private:
    Pose real_pose_; 
    Odometry odom_;
    Lidar lidar_;
    AMCL amcl_;
    std::mt19937 gen_;

    Pose last_amcl_pose_;
};