#pragma once

#include "map.hpp"
#include "amcl.hpp"
#include "lidar.hpp"
#include <string>

class Benchmark
{
public:
    Benchmark(const Map& map, int min_particles, int max_particles);
    
    // Запуск симуляции по круговой/квадратной траектории
    void run_simulation(int steps, const std::string& csv_filepath);

private:
    const Map& map_;

    const int min_particles_ = 500;
    const int max_particles_ = 5000;
};