#pragma once

#include <random>

#include "odometry.hpp"
#include "lidar.hpp"
#include "amcl.hpp"
#include "map.hpp"

class Robot {
public:
    Robot(double start_x, double start_y, double start_theta, const Map& map)
        : real_pose_{start_x, start_y, start_theta},
          odom_(real_pose_),
          lidar_(real_pose_, map, 72),
          amcl_(500, map),
          gen_(std::random_device{}())
    {}

    void move_relative(double forward_dist, double dtheta)
    {
        real_pose_.theta = normalize_angle(real_pose_.theta + dtheta);
        real_pose_.x += forward_dist * std::cos(real_pose_.theta);
        real_pose_.y += forward_dist * std::sin(real_pose_.theta);

        const double base_noise_dist = 0.02; // 2 см ошибки всегда
        const double base_noise_angle = 0.05; // ~3 градуса ошибки всегда при любом действии
        
        const double a_dist = 0.15; // 15% ошибка от пути
        const double a_angle = 0.20; // 20% ошибка от угла

        // Дисперсия: теперь она никогда не будет нулевой при совершении действия
        double std_dev_v = base_noise_dist + a_dist * std::abs(forward_dist);
        double std_dev_w = base_noise_angle + a_angle * std::abs(dtheta);

        std::normal_distribution<double> noise_v(0.0, std_dev_v);
        std::normal_distribution<double> noise_w(0.0, std_dev_w);

        // Генерируем зашумленные данные "энкодеров"
        double noisy_forward = forward_dist + noise_v(gen_);
        double noisy_dtheta = dtheta + noise_w(gen_);

        // 3. Обновление одометрии (Внутренняя логика робота)
        // ВАЖНО: Одометрия живет своей жизнью. У нее свой theta, к которому прибавляется шум.
        Pose current_odom = odom_.get_pose();
        double new_odom_theta = normalize_angle(current_odom.theta + noisy_dtheta);
        
        // Ошибка в направлении (theta) теперь напрямую влияет на то, куда "уедет" синий круг
        double dx = noisy_forward * std::cos(new_odom_theta);
        double dy = noisy_forward * std::sin(new_odom_theta);

        // В apply_delta передаем разницу углов, которую "намерил" энкодер
        odom_.apply_delta(dx, dy, noisy_dtheta);

        amcl_.predict(noisy_forward, noisy_dtheta);
    }

    void update_sensors()
    {
        lidar_.update();
    }

    void update_amcl()
    {
        amcl_.update(lidar_.get_distances());
        amcl_.resample();
    }

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
};