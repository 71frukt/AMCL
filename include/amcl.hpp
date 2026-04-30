#pragma once

#include <vector>
#include <random>
#include <cmath>
#include "map.hpp"
#include "odometry.hpp"

struct Particle
{
    Pose pose;
    double weight = 1.0;
};

class AMCL {
public:
    AMCL(int num_particles, const Map& map)
        : map_(map), gen_(std::random_device{}()) 
    {
        particles_.resize(num_particles);
        initialize_particles();
    }

    void initialize_particles()
    {
        std::uniform_real_distribution<double> dist_x(0, map_.width() * map_.resolution());
        std::uniform_real_distribution<double> dist_y(0, map_.height() * map_.resolution());
        std::uniform_real_distribution<double> dist_theta(-std::numbers::pi, std::numbers::pi);

        for (auto& p : particles_)
        {
            do {
                p.pose = {dist_x(gen_), dist_y(gen_), dist_theta(gen_)};
            } 
            while (map_.is_wall_at_physical(p.pose.x, p.pose.y));
            
            p.weight = 1.0 / particles_.size();
        }
    }

    // 1. Предсказание: перемещение частиц на основе одометрии с добавлением шума
    void predict(double distance, double dtheta)
    {
        std::normal_distribution<double> noise_dist(0.0, 0.1 * std::abs(distance) + 0.01);
        std::normal_distribution<double> noise_theta(0.0, 0.1 * std::abs(dtheta) + 0.02);

        for (auto& p : particles_)
        {
            // 1. Добавляем шум к перемещению конкретной частицы
            double p_dist = distance + noise_dist(gen_);
            double p_dtheta = dtheta + noise_theta(gen_);

            // 2. Обновляем угол гипотезы
            p.pose.theta = normalize_angle(p.pose.theta + p_dtheta);

            // 3. Перемещаем частицу вдоль её собственного направления
            // Это критично: каждая частица "едет" туда, куда смотрит она сама
            p.pose.x += p_dist * std::cos(p.pose.theta);
            p.pose.y += p_dist * std::sin(p.pose.theta);
        }
    }

    // 2. Обновление: расчет весов на основе сравнения скана лидара
    void update(const std::vector<double>& actual_scan) {
        double sigma = 0.5; // Параметр доверия сенсору

        for (auto& p : particles_) {
            if (map_.is_wall_at_physical(p.pose.x, p.pose.y)) {
                p.weight = 0.0;
                continue;
            }

            // Симулируем скан для частицы (упрощенно по 8 направлениям)
            std::vector<double> p_scan = simulate_scan(p.pose, actual_scan.size());
            
            double error = 0.0;
            for (size_t i = 0; i < actual_scan.size(); ++i) {
                double diff = actual_scan[i] - p_scan[i];
                error += diff * diff;
            }
            
            // Вес через Гауссовское распределение ошибки
            p.weight = std::exp(-error / (2 * sigma * sigma));
        }

        normalize_weights();
    }

    // 3. Ресемплинг: замена маловероятных частиц копиями вероятных
    void resample() {
        std::vector<double> weights;
        for (const auto& p : particles_) weights.push_back(p.weight);

        std::discrete_distribution<int> dist(weights.begin(), weights.end());
        std::vector<Particle> new_particles;
        new_particles.reserve(particles_.size());

        for (size_t i = 0; i < particles_.size(); ++i) {
            new_particles.push_back(particles_[dist(gen_)]);
        }
        particles_ = std::move(new_particles);
    }

    // Оценка положения: средневзвешенное состояние всех частиц
    Pose get_estimated_pose() const {
        Pose est{0, 0, 0};
        double sin_sum = 0, cos_sum = 0;

        for (const auto& p : particles_) {
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

    const std::vector<Particle>& get_particles() const { return particles_; }

private:
    const Map& map_;
    std::vector<Particle> particles_;
    std::mt19937 gen_;

    void normalize_weights() {
        double sum = 0;
        for (const auto& p : particles_) sum += p.weight;
        if (sum > 0) {
            for (auto& p : particles_) p.weight /= sum;
        } else {
            initialize_particles();
        }
    }

    // Трассировка лучей для частицы (аналогично лидару робота)
    std::vector<double> simulate_scan(const Pose& p, int num_beams) {
        std::vector<double> scan(num_beams);
        double angle_step = 2.0 * std::numbers::pi / num_beams;
        for (int i = 0; i < num_beams; ++i) {
            double angle = normalize_angle(p.theta + i * angle_step);
            double range = 0;
            while (range < 15.0) {
                range += 0.2;
                if (map_.is_wall_at_physical(p.x + range * std::cos(angle), 
                                            p.y + range * std::sin(angle))) break;
            }
            scan[i] = range;
        }
        return scan;
    }
};