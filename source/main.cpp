#include "game_manager.hpp"

int main() {
    // Настройки экрана
    const int window_size = 800; // 800x800 пикселей
    
    // Так как карта 16x16 метров, а окно 800px, то 800 / 16 = 50 пикселей на метр
    const float pixels_per_meter = 50.0f;

    GameManager game(window_size, window_size, pixels_per_meter);
    game.run();

    return 0;
}