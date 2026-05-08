#include "mcl.hpp"
#include <numbers>
#include <cmath>
#include <algorithm>

MCL::MCL(int num_particles, const Map& map, Lidar& lidar)
    : map_(map)
    , lidar_(lidar)
    , gen_(std::random_device{}())
    , n_particles_(num_particles)
    , w_slow_(0.0)
    , w_fast_(0.0)
{
    particles_.resize(num_particles);
    initialize_particles_();
}

void MCL::initialize_particles_()
{
    std::uniform_real_distribution<double> dist_x(0, map_.width()  * map_.resolution());
    std::uniform_real_distribution<double> dist_y(0, map_.height() * map_.resolution());
    std::uniform_real_distribution<double> dist_theta(-std::numbers::pi, std::numbers::pi);

    for (auto& p : particles_)
    {
        do
        {
            p.pose = {dist_x(gen_), dist_y(gen_), dist_theta(gen_)};
        }
        while (map_.is_wall_at_physical(p.pose.x, p.pose.y));

        p.weight = 1.0 / n_particles_;
    }
}

void MCL::predict_(double distance, double dtheta)
{
    std::normal_distribution<double> noise_dist(0.0, 0.1 * std::abs(distance) + 0.01);
    std::normal_distribution<double> noise_theta(0.0, 0.1 * std::abs(dtheta) + 0.02);

    for (auto& p : particles_)
    {
        double p_dist = distance + noise_dist(gen_);
        double p_dtheta = dtheta + noise_theta(gen_);

        p.pose.theta = normalize_angle(p.pose.theta + p_dtheta);
        p.pose.x += p_dist * std::cos(p.pose.theta);
        p.pose.y += p_dist * std::sin(p.pose.theta);
    }
}

void MCL::update_weights_(const std::vector<double>& actual_scan)
{
    double dispersion = calculate_cloud_dispersion_();
    
    double sigma = 0.5 + std::min(1.5, dispersion * 0.3);
    double two_sigma_sq = 2.0 * sigma * sigma;

    double weight_sum = 0.0;

    for (auto& p : particles_)
    {
        if (map_.is_wall_at_physical(p.pose.x, p.pose.y))
        {
            p.weight = 0.0;
            continue;
        }

        std::vector<double> p_scan = lidar_.simulate_scan(p.pose);
        double sse = 0.0;

        for (size_t i = 0; i < actual_scan.size(); ++i)
        {
            double diff = actual_scan[i] - p_scan[i];
            sse += diff * diff;
        }

        p.weight = std::exp(-sse / two_sigma_sq);
        weight_sum += p.weight;
    }

    double w_avg = weight_sum / n_particles_;

    if (w_slow_ == 0.0) w_slow_ = w_avg;
    if (w_fast_ == 0.0) w_fast_ = w_avg;

    w_slow_ += alpha_slow_ * (w_avg - w_slow_);
    w_fast_ += alpha_fast_ * (w_avg - w_fast_);

    normalize_weights_();
}

void MCL::update(double odom_distance, double odom_dtheta, const std::vector<double>& actual_scan)
{
    predict_(odom_distance, odom_dtheta);
    update_weights_(actual_scan);
    resample_();
}

void MCL::resample_()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(n_particles_);

    // Вероятность вброса случайной частицы (Augmented MCL)
    // Защита от деления на ноль для w_slow_
    double safe_w_slow = std::max(w_slow_, 1e-9);
    double p_random = std::max(0.0, 1.0 - w_fast_ / safe_w_slow);

    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    
    // Low-variance sampler
    double inv_n = 1.0 / n_particles_;
    double r = unit_dist(gen_) * inv_n;
    double c = particles_[0].weight;
    size_t i = 0;

    std::normal_distribution<double> jitter_pos(0.0, 0.03);
    std::normal_distribution<double> jitter_rot(0.0, 0.02);

    for (size_t m = 0; m < n_particles_; ++m)
    {
        if (unit_dist(gen_) < p_random)
        {
            new_particles.push_back(generate_random_particle_());
        }
        else
        {
            double u = r + m * inv_n;
            while (u > c && i < n_particles_ - 1)
            {
                i++;
                c += particles_[i].weight;
            }

            Particle p = particles_[i];
            
            p.pose.x += jitter_pos(gen_);
            p.pose.y += jitter_pos(gen_);
            p.pose.theta += jitter_rot(gen_);
            
            p.weight = inv_n;
            new_particles.push_back(p);
        }
    }

    particles_ = std::move(new_particles);
}

Pose MCL::get_estimated_pose() const
{
    if (particles_.empty())
    {
        return Pose{0.0, 0.0, 0.0};
    }

    auto best_it = std::max_element(particles_.begin(), particles_.end(),
        [](const Particle& a, const Particle& b) 
        {
            return a.weight < b.weight;
        });
        
    const Particle& best_p = *best_it;

    const double cluster_radius = 0.5; 
    
    Pose est{0.0, 0.0, 0.0};
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    double weight_sum = 0.0;

    for (const auto& p : particles_)
    {
        double dx = p.pose.x - best_p.pose.x;
        double dy = p.pose.y - best_p.pose.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist <= cluster_radius)
        {
            est.x += p.pose.x * p.weight;
            est.y += p.pose.y * p.weight;
            sin_sum += std::sin(p.pose.theta) * p.weight;
            cos_sum += std::cos(p.pose.theta) * p.weight;
            weight_sum += p.weight;
        }
    }

    if (weight_sum > 0.0)
    {
        est.x /= weight_sum;
        est.y /= weight_sum;
        est.theta = std::atan2(sin_sum, cos_sum);
    }
    else
    {
        est = best_p.pose; 
    }

    return est;
}

void MCL::normalize_weights_()
{
    double sum = 0.0;
    for (const auto& p : particles_)
    {
        sum += p.weight;
    }

    if (sum > 0.0)
    {
        for (auto& p : particles_)
        {
            p.weight /= sum;
        }
    }
    else
    {
        initialize_particles_();
    }
}

double MCL::calculate_cloud_dispersion_()
{
    if (particles_.empty()) return 0.0;

    double mean_x = 0.0;
    double mean_y = 0.0;

    for (const auto& p : particles_)
    {
        mean_x += p.pose.x;
        mean_y += p.pose.y;
    }

    mean_x /= n_particles_;
    mean_y /= n_particles_;

    double total_dist = 0.0;
    for (const auto& p : particles_)
    {
        double dx = p.pose.x - mean_x;
        double dy = p.pose.y - mean_y;
        total_dist += std::sqrt(dx * dx + dy * dy);
    }

    return total_dist / n_particles_;
}

Particle MCL::generate_random_particle_()
{
    double phys_width = map_.width() * map_.resolution();
    double phys_height = map_.height() * map_.resolution();

    std::uniform_real_distribution<double> dist_x(0.0, phys_width);
    std::uniform_real_distribution<double> dist_y(0.0, phys_height);
    std::uniform_real_distribution<double> dist_theta(0.0, 2.0 * std::numbers::pi);

    Particle p;
    p.weight = 1.0 / n_particles_;

    const int max_attempts = 1000;
    int attempts = 0;

    while (attempts < max_attempts)
    {
        p.pose.x = dist_x(gen_);
        p.pose.y = dist_y(gen_);
        p.pose.theta = dist_theta(gen_);

        if (!map_.is_wall_at_physical(p.pose.x, p.pose.y))
        {
            return p;
        }
        attempts++;
    }

    p.pose.x = phys_width / 2.0;
    p.pose.y = phys_height / 2.0;
    p.pose.theta = 0.0;
    return p;
}