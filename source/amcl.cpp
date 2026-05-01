#include "amcl.hpp"
#include <numbers>

AMCL::AMCL(int num_particles, const Map& map, Lidar& lidar)
    : map_(map)
    , lidar_(lidar)
    , gen_(std::random_device{}())
{
    particles_.resize(num_particles);
    initialize_particles();
}

void AMCL::initialize_particles()
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

        p.weight = 1.0 / particles_.size();
    }
}

void AMCL::predict(double distance, double dtheta)
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

void AMCL::update(const std::vector<double>& actual_scan)
{
    double sigma = 2.0;

    for (auto& p : particles_)
    {
        if (map_.is_wall_at_physical(p.pose.x, p.pose.y))
        {
            p.weight = 0.0;
            continue;
        }

        std::vector<double> p_scan = lidar_.simulate_scan(p.pose);

        double error = 0.0;
        for (size_t i = 0; i < actual_scan.size(); ++i)
        {
            double diff = actual_scan[i] - p_scan[i];
            error += diff * diff;
        }

        error /= actual_scan.size();

        p.weight = std::exp(-error / (2.0 * sigma * sigma));
    }

    normalize_weights_();
}

void AMCL::resample()
{
    std::vector<double> weights;
    weights.reserve(particles_.size());

    for (const auto& p : particles_)
    {
        weights.push_back(p.weight);
    }

    std::discrete_distribution<int> dist(weights.begin(), weights.end());
    std::vector<Particle> new_particles;
    new_particles.reserve(particles_.size());

    for (size_t i = 0; i < particles_.size(); ++i)
    {
        new_particles.push_back(particles_[dist(gen_)]);
    }

    particles_ = std::move(new_particles);
}

Pose AMCL::get_estimated_pose() const
{
    Pose est{0, 0, 0};
    double sin_sum = 0.0;
    double cos_sum = 0.0;

    for (const auto& p : particles_)
    {
        est.x += p.pose.x;
        est.y += p.pose.y;
        sin_sum += std::sin(p.pose.theta);
        cos_sum += std::cos(p.pose.theta);
    }

    est.x /= particles_.size();
    est.y /= particles_.size();
    est.theta = std::atan2(sin_sum, cos_sum);
    return est;
}


void AMCL::normalize_weights_()
{
    double sum = 0.0;
    for (const auto& p : particles_)
    {
        sum += p.weight;
    }

    if (sum > 1e-9)
    {
        for (auto& p : particles_)
        {
            p.weight /= sum;
        }
    }
    else
    {
        initialize_particles();
    }
}
