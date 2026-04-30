#pragma once

#include <random>

#include "odometry.hpp"

class Robot {
public:
    Robot(double start_x, double start_y, double start_theta)
        : real_pose_{start_x, start_y, start_theta},
          odom_(real_pose_),
          gen_(std::random_device{}())
    {}

    // Перемещение робота в абсолютную точку на карте с генерацией шума энкодеров
    void move(double target_x, double target_y, double target_theta)
    {
        double dx = target_x - real_pose_.x;
        double dy = target_y - real_pose_.y;
        double dtheta = target_theta - real_pose_.theta;

        real_pose_ = {target_x, target_y, target_theta};
        real_pose_.theta = normalize_angle(real_pose_.theta);

        std::normal_distribution<double> noise_lin(0.0, std::abs(dx + dy) * 0.05); // 5% ошибка
        std::normal_distribution<double> noise_ang(0.0, std::abs(dtheta) * 0.05);

        double noisy_dx = dx + noise_lin(gen_);
        double noisy_dy = dy + noise_lin(gen_);
        double noisy_dtheta = dtheta + noise_ang(gen_);

        odom_.apply_delta(noisy_dx, noisy_dy, noisy_dtheta);
    }

    // Относительное перемещение (вперед/назад, поворот) для обработки ввода с клавиатуры
    void move_relative(double forward_dist, double dtheta)
    {
        double target_theta = normalize_angle(real_pose_.theta + dtheta);
        double target_x = real_pose_.x + forward_dist * std::cos(target_theta);
        double target_y = real_pose_.y + forward_dist * std::sin(target_theta);

        move(target_x, target_y, target_theta);
    }

    Pose GetIntendedPos() const
    {
        return odom_.get_pose();
    }

    Pose GetRealPos() const
    {
        return real_pose_;
    }

private:
    Pose real_pose_; 
    Odometry odom_;
    std::mt19937 gen_;
};