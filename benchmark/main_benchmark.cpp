#include "map.hpp"
#include "benchmark.hpp"
#include "maps/map1.hpp"

int main()
{
    // Инициализация карты (подставь свой конструктор/загрузчик)
    const Map& map = maps::GetMap1();
    
    // Инициализация бенчмарка - 2500 частиц
    Benchmark bench(map, 200, 2500);
    
    // Симулируем 500 шагов, результат кладем в корень запуска
    bench.run_simulation(307, "benchmark_results.csv");

    return 0;
}