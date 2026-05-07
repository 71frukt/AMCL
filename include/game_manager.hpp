#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include "amcl.hpp"
#include "robot.hpp"
#include "map.hpp"

class GameManager {
public:
    GameManager(int window_width, int window_height, float pixels_per_meter);

    void run();

private:
    sf::RenderWindow window_;
    float ppm_;
    Map map_;
    Robot robot_;

private:
    void init_map_()
    {
        std::vector<int> grid = {
            1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
            1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,
            1,0,0,0,0,0,1,0,0,1,1,1,1,0,0,1,
            1,0,1,1,0,0,1,0,0,1,0,0,1,0,0,1,
            1,0,1,1,0,0,0,0,0,1,0,0,1,0,0,1,
            1,0,0,0,0,0,0,0,0,1,1,0,1,0,0,1,
            1,1,1,1,1,0,0,1,1,1,1,0,1,0,0,1,
            1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,
            1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,
            1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,1,
            1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,
            1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,
            1,0,1,1,1,1,1,0,0,0,0,1,0,0,0,1,
            1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,
            1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
            1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
        };
        map_.set_data(grid);
    }

    void process_events_();

    std::pair<double, double> handle_input_(double dt);

    void draw_map_();
    void draw_lidar_();
    void draw_particles_();
    void draw_robot_(const Pose& pose, sf::Color color, bool is_outline = false);
    
    void render_();

    sf::Font font_;
    sf::Text text_particle_count_;
};