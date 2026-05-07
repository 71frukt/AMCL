#pragma once

#include <vector>
#include <random>
#include <cmath>

#include "map.hpp"
#include "odometry.hpp"
#include "lidar.hpp"
#include "amcl.hpp"


class MCL
{
public:
    MCL(int num_particles, const Map& map, Lidar& lidar);

    void update(double odom_distance, double odom_dtheta, const std::vector<double>& actual_scan);
    
    Pose get_estimated_pose() const;

    const std::vector<Particle>& get_particles() const { return particles_; }
    
    size_t get_particle_count() const { return particles_.size(); }

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

    void initialize_particles_();
    void predict_(double distance, double dtheta);
    void update_weights_(const std::vector<double>& actual_scan);
    void resample_();
    void normalize_weights_();
    double calculate_cloud_dispersion_();
    Particle generate_random_particle_();
};
