#pragma once

#include <vector>
#include <cstddef>

class Map
{
public:
    Map(size_t width, size_t height, double resolution = 1.0)
        : width_      (width)
        , height_     (height)
        , resolution_ (resolution)
        , data_       (width * height, 0)
    {}

    void set_data(const std::vector<int>& grid_data)
    {
        if (grid_data.size() == data_.size()) {
            data_ = grid_data;
        }
    }

    bool is_wall_at_grid(int grid_x, int grid_y) const
    {
        if (grid_x < 0 || grid_x >= static_cast<int>(width_) ||
            grid_y < 0 || grid_y >= static_cast<int>(height_)) {
            return true; // Выход за границы карты считается стеной
        }
        return data_[grid_y * width_ + grid_x] == 1;
    }

    // Проверка наличия стены по физическим (double) координатам
    bool is_wall_at_physical(double x, double y) const {
        int grid_x = static_cast<int>(x / resolution_);
        int grid_y = static_cast<int>(y / resolution_);
        return is_wall_at_grid(grid_x, grid_y);
    }

    size_t width()      const { return width_; }
    size_t height()     const { return height_; }
    double resolution() const { return resolution_; }

private:
    size_t width_;
    size_t height_;
    double resolution_; // Физический размер одной ячейки (например, 1.0 метр)
    std::vector<int> data_; // 0 - пустота, 1 - стена
};