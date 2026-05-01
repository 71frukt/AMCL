#include "amcl.hpp"
#include <numbers>

AMCL::AMCL(int num_particles, const Map& map, Lidar& lidar)
    : map_(map)
    , lidar_(lidar)
    , gen_(std::random_device{}())
    , w_slow_(0.0)
    , w_fast_(0.0)
    , n_particles_(num_particles)
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
    // 1. Оцениваем разброс облака для динамической настройки точности
    double dispersion = calculate_cloud_dispersion_();
    
    // 2. Адаптивная сигма: от 0.5 (точное ведение) до 2.0 (глобальный поиск)
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

void AMCL::resample()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(n_particles_);

    // Вычисляем вероятность вброса случайной частицы
    // Если w_fast значительно меньше w_slow, p_random растет
    double p_random = std::max(0.0, 1.0 - w_fast_ / w_slow_);

    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    
    // Настройка Low-variance sampler
    double inv_n = 1.0 / n_particles_;
    double r = unit_dist(gen_) * inv_n;
    double c = particles_[0].weight;
    int i = 0;

    // Параметры джиттера (раздувания облака)
    std::normal_distribution<double> jitter_pos(0.0, 0.03);
    std::normal_distribution<double> jitter_rot(0.0, 0.02);

    for (int m = 0; m < n_particles_; ++m)
    {
        if (unit_dist(gen_) < p_random)
        {
            // Вбрасываем абсолютно новую гипотезу в случайное место
            new_particles.push_back(generate_random_particle_());
        }
        else
        {
            // Стандартный систематический ресемплинг
            double u = r + m * inv_n;
            while (u > c && i < n_particles_ - 1)
            {
                i++;
                c += particles_[i].weight;
            }

            Particle p = particles_[i];
            
            // Накладываем шум на клонов, чтобы облако не схлопывалось
            p.pose.x += jitter_pos(gen_);
            p.pose.y += jitter_pos(gen_);
            p.pose.theta += jitter_rot(gen_);
            
            p.weight = inv_n;
            new_particles.push_back(p);
        }
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

    // Возвращаем жесткую проверку на ноль. 
    // Если все частицы смотрят не туда, экспонента выдаст нули, sum станет 0.0,
    // и произойдет корректный сброс облака по всей карте.
    if (sum > 0.0)
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
    // Физические размеры карты в метрах
    double phys_width = map_.width() * map_.resolution();
    double phys_height = map_.height() * map_.resolution();

    std::uniform_real_distribution<double> dist_x(0.0, phys_width);
    std::uniform_real_distribution<double> dist_y(0.0, phys_height);
    std::uniform_real_distribution<double> dist_theta(0.0, 2.0 * M_PI);

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

    // Fallback: безопасное положение в центре карты или начале координат
    p.pose.x = phys_width / 2.0;
    p.pose.y = phys_height / 2.0;
    p.pose.theta = 0.0;
    return p;
}