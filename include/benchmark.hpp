#pragma once

#include "map.hpp"
#include "amcl.hpp"
#include "lidar.hpp"
#include <string>

class Benchmark
{
public:
    Benchmark(const Map& map, int num_particles);
    
    // Запуск симуляции по круговой/квадратной траектории
    void run_simulation(int steps, const std::string& csv_filepath);

private:
    const Map& map_;
    int n_particles_;
};