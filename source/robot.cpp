#include "robot.hpp"
#include <cmath>

Robot::Robot(double start_x, double start_y, double start_theta, const Map& map)
    : real_pose_{start_x, start_y, start_theta}
    , odom_(real_pose_)
    , lidar_(real_pose_, map, 72)
    , amcl_(200, 1000, map, lidar_)
    , gen_(std::random_device{}())
    // Запоминаем позу одометрии, на которой был прошлый апдейт AMCL
    , last_amcl_odom_pose_{start_x, start_y, start_theta}
{}

void Robot::move_relative(double forward_dist, double dtheta)
{
    // 1. Истинное движение (Ground Truth)
    real_pose_.theta = normalize_angle(real_pose_.theta + dtheta);
    real_pose_.x += forward_dist * std::cos(real_pose_.theta);
    real_pose_.y += forward_dist * std::sin(real_pose_.theta);

    // 2. Модель шума энкодеров
    const double base_noise_dist = 0.02; 
    const double base_noise_angle = 0.05; 
    const double a_dist = 0.15; 
    const double a_angle = 0.20; 

    double std_dev_v = base_noise_dist + a_dist * std::abs(forward_dist);
    double std_dev_w = base_noise_angle + a_angle * std::abs(dtheta);

    std::normal_distribution<double> noise_v(0.0, std_dev_v);
    std::normal_distribution<double> noise_w(0.0, std_dev_w);

    double noisy_forward = forward_dist + noise_v(gen_);
    double noisy_dtheta = dtheta + noise_w(gen_);

    // 3. Обновление одометрии
    Pose current_odom = odom_.get_pose();
    double new_odom_theta = normalize_angle(current_odom.theta + noisy_dtheta);
    
    double dx = noisy_forward * std::cos(new_odom_theta);
    double dy = noisy_forward * std::sin(new_odom_theta);

    odom_.apply_delta(dx, dy, noisy_dtheta);

    // ВАЖНО: Мы больше не вызываем amcl_.predict() здесь. 
    // Движение частиц произойдет внутри единого метода update.
}

void Robot::update_sensors()
{
    lidar_.update();
}

void Robot::update_amcl()
{
    Pose current_odom = odom_.get_pose();
    
    // Вычисляем, сколько робот прошел и прокрутился С МОМЕНТА ПОСЛЕДНЕГО АПДЕЙТА AMCL
    double dx = current_odom.x - last_amcl_odom_pose_.x;
    double dy = current_odom.y - last_amcl_odom_pose_.y;
    
    double dist_moved = std::sqrt(dx * dx + dy * dy);
    double dtheta_moved = normalize_angle(current_odom.theta - last_amcl_odom_pose_.theta);

    // Условие срабатывания: 10 см или ~5 градусов
    if (dist_moved > 0.1 || std::abs(dtheta_moved) > 0.087)
    {
        // Вызываем единый инкапсулированный метод.
        // Он сам сделает внутри predict_ (на dist_moved), update_weights_ и resample_
        amcl_.update(dist_moved, dtheta_moved, lidar_.get_distances());
        
        // Сбрасываем точку отсчета для следующей дельты
        last_amcl_odom_pose_ = current_odom;
    }
}