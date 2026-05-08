#include "amcl.hpp"
#include <numbers>
#include <unordered_set>
#include <tuple>
#include <cmath>
#include <algorithm>

namespace 
{
    struct BinHash 
    {
        size_t operator()(const std::tuple<int, int, int>& t) const 
        {
            auto h1 = std::hash<int>{}(std::get<0>(t));
            auto h2 = std::hash<int>{}(std::get<1>(t));
            auto h3 = std::hash<int>{}(std::get<2>(t));
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

AMCL::AMCL(int min_particles, int max_particles, const Map& map, Lidar& lidar)
    : map_(map)
    , lidar_(lidar)
    , gen_(std::random_device{}())
    , min_particles_(min_particles)
    , max_particles_(max_particles)
    , w_slow_(0.0)
    , w_fast_(0.0)
{
    particles_.resize(max_particles);
    initialize_particles_();
}

void AMCL::initialize_particles_()
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

void AMCL::predict_(double distance, double dtheta)
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

void AMCL::update_weights_(const std::vector<double>& actual_scan)
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

    double w_avg = weight_sum / particles_.size();

    if (w_slow_ == 0.0) w_slow_ = w_avg;
    if (w_fast_ == 0.0) w_fast_ = w_avg;

    w_slow_ += alpha_slow_ * (w_avg - w_slow_);
    w_fast_ += alpha_fast_ * (w_avg - w_fast_);

    normalize_weights_();
}


void AMCL::update(double odom_distance, double odom_dtheta, const std::vector<double>& actual_scan)
{
    predict_(odom_distance, odom_dtheta);
    
    update_weights_(actual_scan);
    
    resample_();
}

void AMCL::resample_()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(max_particles_);

    std::unordered_set<std::tuple<int, int, int>, BinHash> occupied_bins;

    double p_random = std::max(0.0, 1.0 - w_fast_ / w_slow_);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    
    double inv_n = 1.0 / particles_.size();
    double r = unit_dist(gen_) * inv_n;
    double c = particles_[0].weight;
    int i = 0;

    std::normal_distribution<double> jitter_pos(0.0, 0.03);
    std::normal_distribution<double> jitter_rot(0.0, 0.02);

    int k = 0;
    int n_req = min_particles_; 
    int m = 0;

    while (new_particles.size() < max_particles_ && 
          (new_particles.size() < n_req || new_particles.size() < min_particles_))
    {
        Particle p;

        if (unit_dist(gen_) < p_random)
        {
            p = generate_random_particle_();
        }
        else
        {
            double u = r + m * inv_n;
            while (u > c && i < particles_.size() - 1)
            {
                i++;
                c += particles_[i].weight;
            }

            p = particles_[i];
            
            p.pose.x += jitter_pos(gen_);
            p.pose.y += jitter_pos(gen_);
            p.pose.theta += jitter_rot(gen_);
            
            p.weight = 1.0; 
            m++;
        }

        int bin_x = static_cast<int>(p.pose.x / bin_size_x_);
        int bin_y = static_cast<int>(p.pose.y / bin_size_y_);
        int bin_theta = static_cast<int>(p.pose.theta / bin_size_theta_);
        auto bin = std::make_tuple(bin_x, bin_y, bin_theta);

        if (occupied_bins.find(bin) == occupied_bins.end())
        {
            occupied_bins.insert(bin);
            k++;
            
            if (k > 1)
            {
                double a = 1.0 - 2.0 / (9.0 * (k - 1));
                double b = kld_z_ * std::sqrt(2.0 / (9.0 * (k - 1)));
                n_req = static_cast<int>( (k - 1) / (2.0 * kld_err_) * std::pow(a + b, 3) );
            }
        }

        new_particles.push_back(p);
    }

    double new_weight = 1.0 / new_particles.size();
    for (auto& np : new_particles)
    {
        np.weight = new_weight;
    }

    particles_ = std::move(new_particles);
}

// void AMCL::resample()
// {
//     std::vector<Particle> new_particles;
//     new_particles.reserve(n_particles_);

//     // Вычисляем вероятность вброса случайной частицы
//     // Если w_fast значительно меньше w_slow, p_random растет
//     double p_random = std::max(0.0, 1.0 - w_fast_ / w_slow_);

//     std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    
//     // Настройка Low-variance sampler
//     double inv_n = 1.0 / n_particles_;
//     double r = unit_dist(gen_) * inv_n;
//     double c = particles_[0].weight;
//     int i = 0;

//     // Параметры джиттера (раздувания облака)
//     std::normal_distribution<double> jitter_pos(0.0, 0.03);
//     std::normal_distribution<double> jitter_rot(0.0, 0.02);

//     for (int m = 0; m < n_particles_; ++m)
//     {
//         if (unit_dist(gen_) < p_random)
//         {
//             // Вбрасываем абсолютно новую гипотезу в случайное место
//             new_particles.push_back(generate_random_particle_());
//         }
//         else
//         {
//             // Стандартный систематический ресемплинг
//             double u = r + m * inv_n;
//             while (u > c && i < n_particles_ - 1)
//             {
//                 i++;
//                 c += particles_[i].weight;
//             }

//             Particle p = particles_[i];
            
//             // Накладываем шум на клонов, чтобы облако не схлопывалось
//             p.pose.x += jitter_pos(gen_);
//             p.pose.y += jitter_pos(gen_);
//             p.pose.theta += jitter_rot(gen_);
            
//             p.weight = inv_n;
//             new_particles.push_back(p);
//         }
//     }

//     particles_ = std::move(new_particles);
// }

Pose AMCL::get_estimated_pose() const
{
    if (particles_.empty())
    {
        return Pose{0.0, 0.0, 0.0};
    }

    // 1. Находим частицу с максимальным весом (эпицентр самой вероятной гипотезы)
    auto best_it = std::max_element(particles_.begin(), particles_.end(),
        [](const Particle& a, const Particle& b) 
        {
            return a.weight < b.weight;
        });
        
    const Particle& best_p = *best_it;

    // 2. Усредняем только те частицы, которые принадлежат этому кластеру.
    // Радиус кластера зависит от масштабов карты (например, 0.5 метра)
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

        // Отсекаем равномерно распределенный шум и альтернативные гипотезы
        if (dist <= cluster_radius)
        {
            est.x += p.pose.x * p.weight;
            est.y += p.pose.y * p.weight;
            sin_sum += std::sin(p.pose.theta) * p.weight;
            cos_sum += std::cos(p.pose.theta) * p.weight;
            weight_sum += p.weight;
        }
    }

    // Нормализуем по сумме весов кластера
    if (weight_sum > 0.0)
    {
        est.x /= weight_sum;
        est.y /= weight_sum;
        est.theta = std::atan2(sin_sum, cos_sum);
    }
    else
    {
        // Fallback: если что-то пошло не так, возвращаем просто лучшую гипотезу
        est = best_p.pose; 
    }

    return est;
}

void AMCL::normalize_weights_()
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

double AMCL::calculate_cloud_dispersion_()
{
    if (particles_.empty())
    {
        return 0.0;
    }

    double mean_x = 0.0;
    double mean_y = 0.0;

    for (const auto& p : particles_)
    {
        mean_x += p.pose.x;
        mean_y += p.pose.y;
    }

    mean_x /= particles_.size();
    mean_y /= particles_.size();

    double total_dist = 0.0;
    for (const auto& p : particles_)
    {
        double dx = p.pose.x - mean_x;
        double dy = p.pose.y - mean_y;
        total_dist += std::sqrt(dx * dx + dy * dy);
    }

    return total_dist / particles_.size();
}

Particle AMCL::generate_random_particle_()
{
    double phys_width = map_.width() * map_.resolution();
    double phys_height = map_.height() * map_.resolution();

    std::uniform_real_distribution<double> dist_x(0.0, phys_width);
    std::uniform_real_distribution<double> dist_y(0.0, phys_height);
    std::uniform_real_distribution<double> dist_theta(0.0, 2.0 * std::numbers::pi);

    Particle p;
    p.weight = 1.0 / particles_.size();

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




