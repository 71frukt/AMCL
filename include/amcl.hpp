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
    size_t n_particles_;
    std::mt19937 gen_;

    double w_slow_ = 0.0;
    double w_fast_ = 0.0;
    const double alpha_slow_ = 0.001;
    const double alpha_fast_ = 0.1;

    void normalize_weights_();
    double calculate_cloud_dispersion_();
    Particle generate_random_particle_();

};
