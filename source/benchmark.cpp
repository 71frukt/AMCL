#include "benchmark.hpp"
#include "odometry.hpp"
#include "amcl.hpp"
#include "naive_localizer.hpp"

#include <fstream>
#include <iostream>
#include <random>
#include <cmath>
#include <chrono>

Benchmark::Benchmark(const Map& map, int min_particles, int max_particles)
    : map_(map)
    , lidar_(Pose{8.0, 8.0, 0.0}, map)
    , amcl_(min_particles, max_particles, map_, lidar_)
    , mcl_( max_particles, map_, lidar_)
    , naive_localizer_(map_, lidar_, 0.4, 0.2)

    , min_particles_(min_particles)
    , max_particles_(max_particles)   
{
}

void Benchmark::run_simulation(int steps, const std::string& csv_filepath)
{
    std::ofstream file(csv_filepath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open " << csv_filepath << std::endl;
        return;
    }

    file << "step,gt_x,gt_y,odom_x,odom_y,amcl_x,amcl_y,mcl_x,mcl_y,naive_x,naive_y,"
         << "err_odom,err_amcl,err_mcl,err_naive,time_amcl_us,time_mcl_us,time_naive_us\n";

    Pose gt_pose{8.0, 8.0, 0.0};
    Odometry blind_odom(gt_pose);

    std::mt19937 gen(42); // Seed 42 гарантирует, что хаотичный маршрут будет повторяться при перезапусках
    std::normal_distribution<double> noise_odom_dist(0.0, 0.05);
    std::normal_distribution<double> noise_odom_theta(0.0, 0.02);
    std::normal_distribution<double> noise_lidar(0.0, 0.03);      

    // --- НОВЫЕ ГЕНЕРАТОРЫ ДЛЯ ХАОТИЧНОГО ДВИЖЕНИЯ ---
    std::uniform_real_distribution<double> rand_turn_speed(0.1, 0.4); // Скорость вращения (рад/шаг)
    std::uniform_int_distribution<int> rand_turn_duration(5, 15);     // Сколько шагов крутиться
    std::uniform_int_distribution<int> rand_direction(0, 1);          // 0 - влево, 1 - вправо

    // Переменные состояния робота
    int turn_steps_remaining = 0;
    double current_turn_cmd  = 0.0;

    auto is_wall_ahead = [&](double current_x, double current_y, double current_theta, double dist) {
        double next_x = current_x + dist * std::cos(current_theta);
        double next_y = current_y + dist * std::sin(current_theta);
        
        int grid_x = static_cast<int>(next_x / map_.resolution());
        int grid_y = static_cast<int>(next_y / map_.resolution());
        
        if (grid_x < 0 || grid_x >= map_.width() || grid_y < 0 || grid_y >= map_.height()) {
            return true;
        }
        
        const auto& grid = map_.get_data(); 
        return grid[grid_y * map_.width() + grid_x] == 1;
    };

    for (int i = 0; i < steps; ++i)
    {
        std::cout << i << std::endl;

        double cmd_dist = 0.0;
        double cmd_theta = 0.0;

        // --- ЛОГИКА ХАОТИЧНОГО ИССЛЕДОВАНИЯ ---
        if (turn_steps_remaining > 0) {
            // Робот находится в процессе разворота
            cmd_dist = 0.0;
            cmd_theta = current_turn_cmd;
            turn_steps_remaining--;
        } 
        else if (is_wall_ahead(gt_pose.x, gt_pose.y, gt_pose.theta, 0.6)) {
            // Уперлись в стену (смотрим на 0.6м вперед). Генерируем новый маневр.
            double dir = (rand_direction(gen) == 0) ? 1.0 : -1.0;
            current_turn_cmd = dir * rand_turn_speed(gen);
            turn_steps_remaining = rand_turn_duration(gen);
            
            cmd_dist = 0.0;
            cmd_theta = current_turn_cmd;
            turn_steps_remaining--;
        } 
        else {
            // Путь свободен, едем прямо (добавил микро-дрейф 0.01 для изгиба траектории)
            cmd_dist = 0.2;
            cmd_theta = 0.01; 
        }

        // 1. Истинное движение (GT)
        gt_pose.theta = normalize_angle(gt_pose.theta + cmd_theta);
        double dx_gt = cmd_dist * std::cos(gt_pose.theta);
        double dy_gt = cmd_dist * std::sin(gt_pose.theta);
        gt_pose.x += dx_gt;
        gt_pose.y += dy_gt;

        // 2. Симуляция показаний одометрии (с шумом)
        double measured_dist = cmd_dist + (cmd_dist > 0.0 ? noise_odom_dist(gen) : 0.0);
        double measured_theta = cmd_theta + noise_odom_theta(gen);

        Pose current_odom = blind_odom.get_pose();
        double heading = current_odom.theta + measured_theta;
        double dx_odom = measured_dist * std::cos(heading);
        double dy_odom = measured_dist * std::sin(heading);
        blind_odom.apply_delta(dx_odom, dy_odom, measured_theta);

        // 3. Симуляция показаний лидара
        std::vector<double> ideal_scan = lidar_.simulate_scan(gt_pose); 
        std::vector<double> noisy_scan = ideal_scan;
        for (auto& ray : noisy_scan)
        {
            if (ray > 0 && ray < 100.0) 
            {
                ray += noise_lidar(gen);
            }
        }

        auto start_amcl = std::chrono::high_resolution_clock::now();
        amcl_.update(measured_dist, measured_theta, noisy_scan);
        auto end_amcl = std::chrono::high_resolution_clock::now();

        auto start_mcl = std::chrono::high_resolution_clock::now();
        mcl_.update(measured_dist, measured_theta, noisy_scan);
        auto end_mcl = std::chrono::high_resolution_clock::now();

        auto start_naive = std::chrono::high_resolution_clock::now();
        naive_localizer_.update(noisy_scan);
        auto end_naive = std::chrono::high_resolution_clock::now();

        Pose p_odom  = blind_odom      .get_pose();
        Pose p_amcl  = amcl_           .get_estimated_pose();
        Pose p_mcl   = mcl_            .get_estimated_pose();
        Pose p_naive = naive_localizer_.get_estimated_pose();

        double err_odom  = std::hypot(p_odom.x  - gt_pose.x, p_odom.y  - gt_pose.y);
        double err_amcl  = std::hypot(p_amcl.x  - gt_pose.x, p_amcl.y  - gt_pose.y);
        double err_mcl   = std::hypot(p_mcl.x   - gt_pose.x, p_mcl.y   - gt_pose.y);
        double err_naive = std::hypot(p_naive.x - gt_pose.x, p_naive.y - gt_pose.y);
        
        auto t_amcl  = std::chrono::duration_cast<std::chrono::microseconds>(end_amcl  - start_amcl) .count();
        auto t_mcl   = std::chrono::duration_cast<std::chrono::microseconds>(end_mcl   - start_mcl)  .count();
        auto t_naive = std::chrono::duration_cast<std::chrono::microseconds>(end_naive - start_naive).count();

        file << i << ","
             << gt_pose.x << "," << gt_pose.y << ","
             << p_odom.x  << "," << p_odom.y  << ","
             << p_amcl.x  << "," << p_amcl.y  << ","
             << p_mcl.x   << "," << p_mcl.y   << ","
             << p_naive.x << "," << p_naive.y << ","
             << err_odom  << "," << err_amcl  << "," << err_mcl << "," << err_naive << ","
             << t_amcl    << "," << t_mcl     << "," << t_naive << "\n";
    }

    file.close();
    std::cout << "Benchmark complete. Data saved to " << csv_filepath << "\n";
}