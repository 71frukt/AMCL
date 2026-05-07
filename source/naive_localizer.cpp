#include "naive_localizer.hpp"
#include <limits>
#include <cmath>
#include <numbers>

NaiveLocalizer::NaiveLocalizer(const Map& map, Lidar& lidar, double pos_step, double theta_step)
    : map_(map)
    , lidar_(lidar)
    , pos_step_(pos_step)
    , theta_step_(theta_step)
{
}

void NaiveLocalizer::update(const std::vector<double>& actual_scan)
{
    double min_sse = std::numeric_limits<double>::max();
    Pose best_pose = estimated_pose_;

    double map_width_m = map_.width() * map_.resolution();
    double map_height_m = map_.height() * map_.resolution();

    for (double x = 0.0; x < map_width_m; x += pos_step_)
    {
        for (double y = 0.0; y < map_height_m; y += pos_step_)
        {
            if (map_.is_wall_at_physical(x, y))
            {
                continue;
            }

            for (double theta = -std::numbers::pi; theta < std::numbers::pi; theta += theta_step_)
            {
                Pose test_pose{x, y, theta};
                
                std::vector<double> simulated_scan = lidar_.simulate_scan(test_pose);
                
                double current_sse = 0.0;
                for (size_t i = 0; i < actual_scan.size(); ++i)
                {
                    double diff = actual_scan[i] - simulated_scan[i];
                    current_sse += diff * diff;
                }

                if (current_sse < min_sse)
                {
                    min_sse = current_sse;
                    best_pose = test_pose;
                }
            }
        }
    }

    estimated_pose_ = best_pose;
}

Pose NaiveLocalizer::get_estimated_pose() const
{
    return estimated_pose_;
}