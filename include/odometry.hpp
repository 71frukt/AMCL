#include <vector>
#include <cmath>
#include <numbers>

struct Pose
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// Нормализация угла в диапазон [-pi, pi]
inline double normalize_angle(double angle)
{
    while (angle >  std::numbers::pi) angle -= 2.0 * std::numbers::pi;
    while (angle < -std::numbers::pi) angle += 2.0 * std::numbers::pi;
    return angle;
}

class Odometry
{
public:
    explicit Odometry(const Pose& start_pose) : current_pose_(start_pose) {}

    void apply_delta(double dx, double dy, double dtheta)
    {
        current_pose_.theta = normalize_angle(current_pose_.theta + dtheta);
        current_pose_.x += dx;
        current_pose_.y += dy;
    }

    Pose get_pose() const { return current_pose_; }

private:
    Pose current_pose_;
};
