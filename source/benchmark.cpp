#include "benchmark.hpp"
#include <fstream>
#include <iostream>
#include <random>
#include <cmath>
#include "benchmark.hpp"
#include "odometry.hpp"
#include <fstream>
#include <iostream>
#include <random>
#include <cmath>

Benchmark::Benchmark(const Map& map, int num_particles)
    : map_(map), n_particles_(num_particles)
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

    file << "step,gt_x,gt_y,gt_theta,odom_x,odom_y,odom_theta,amcl_x,amcl_y,amcl_theta,err_odom,err_amcl\n";

    Pose gt_pose{8.0, 8.0, 0.0};
    Odometry blind_odom(gt_pose);

    Lidar lidar(gt_pose, map_);
    AMCL amcl(n_particles_, map_, lidar);
    
    amcl.initialize_particles();

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
    double current_turn_cmd = 0.0;

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
        std::vector<double> ideal_scan = lidar.simulate_scan(gt_pose); 
        std::vector<double> noisy_scan = ideal_scan;
        for (auto& ray : noisy_scan)
        {
            if (ray > 0 && ray < 100.0) 
            {
                ray += noise_lidar(gen);
            }
        }

        // 4. Обновление AMCL
        amcl.predict(measured_dist, measured_theta); 
        amcl.update(noisy_scan);
        amcl.resample();

        Pose amcl_pose = amcl.get_estimated_pose();
        Pose final_odom = blind_odom.get_pose();

        double err_odom = std::hypot(final_odom.x - gt_pose.x, final_odom.y - gt_pose.y);
        double err_amcl = std::hypot(amcl_pose.x - gt_pose.x, amcl_pose.y - gt_pose.y);

        file << i << ","
             << gt_pose.x << "," << gt_pose.y << "," << gt_pose.theta << ","
             << final_odom.x << "," << final_odom.y << "," << final_odom.theta << ","
             << amcl_pose.x << "," << amcl_pose.y << "," << amcl_pose.theta << ","
             << err_odom << "," << err_amcl << "\n";
    }

    file.close();
    std::cout << "Benchmark complete. Data saved to " << csv_filepath << "\n";
}