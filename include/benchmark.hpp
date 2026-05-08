#pragma once

#include <string>

#include "map.hpp"
#include "naive_localizer.hpp"
#include "amcl.hpp"
#include "mcl.hpp"


class Benchmark
{
public:
    Benchmark(const Map& map, int min_particles, int max_particles);
    
    // Запуск симуляции по круговой/квадратной траектории
    void run_simulation(int steps, const std::string& csv_filepath);

private:
    const Map& map_;
    Lidar lidar_;
    
    AMCL           amcl_;
    MCL            mcl_;
    NaiveLocalizer naive_localizer_;

    const int min_particles_ = 500;
    const int max_particles_ = 5000;
};