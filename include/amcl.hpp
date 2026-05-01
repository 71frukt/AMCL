#pragma once

#include <vector>
#include <random>
#include <cmath>

#include "map.hpp"
#include "odometry.hpp"
#include "lidar.hpp"

struct Particle
{
    Pose pose;
    double weight = 1.0;
};

class AMCL
{
public:
    AMCL(int num_particles, const Map& map, Lidar& lidar);

    void initialize_particles();
    void predict(double distance, double dtheta);
    void update(const std::vector<double>& actual_scan);
    void resample();
    Pose get_estimated_pose() const;

    const std::vector<Particle>& get_particles() const { return particles_; }

private:
    const Map& map_;
    Lidar& lidar_;
    std::vector<Particle> particles_;
    std::mt19937 gen_;

    void normalize_weights_();
};