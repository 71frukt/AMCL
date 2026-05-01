#pragma once

#include <vector>
#include <cmath>
#include <numbers>
#include "map.hpp"
#include "odometry.hpp"

class Lidar {
public:
    Lidar(const Pose& real_pose, const Map& map, int num_beams = 36)
        : real_pose_(real_pose)
        , map_(map)
        , num_beams_(num_beams) 
    {
        distances_.resize(num_beams_, 0.0);
    }

    void update()
    {
        distances_ = simulate_scan(real_pose_);
    }

    std::vector<double> simulate_scan(Pose pose)
    {
        double angle_step = 2.0 * std::numbers::pi / num_beams_;
        double max_range = 15.0; // Максимальная дистанция лидара в метрах
        double step_size = 0.05; // Шаг проверки луча (5 см)

        std::vector<double> distances(num_beams_);

        for (int i = 0; i < num_beams_; ++i)
        {
            double beam_angle = normalize_angle(pose.theta + i * angle_step);
            double current_range = 0.0;
            bool hit = false;

            while (current_range < max_range)
            {
                current_range += step_size;
                double check_x = pose.x + current_range * std::cos(beam_angle);
                double check_y = pose.y + current_range * std::sin(beam_angle);

                if (map_.is_wall_at_physical(check_x, check_y))
                {
                    hit = true;
                    break;
                }
            }
            
            distances[i] = current_range;
        }

        return distances;
    }

    const std::vector<double>& get_distances() const { return distances_; }
    int get_num_beams() const { return num_beams_; }

private:
    const Pose& real_pose_;
    const Map& map_;
    int num_beams_;
    std::vector<double> distances_;
};