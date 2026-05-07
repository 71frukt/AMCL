#include "game_manager.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <iostream>

GameManager::GameManager(int window_width, int window_height, float pixels_per_meter)
    : window_(sf::VideoMode(window_width, window_height), "AMCL Particle Filter MVP"),
        ppm_(pixels_per_meter),
        map_(16, 16, 1.0),
        robot_(2.0, 2.0, 0.0, map_)
{
    window_.setFramerateLimit(60);
    init_map_();

    if (!font_.loadFromFile("fonts/Ubuntu-C.ttf"))
    {
        std::cerr << "Failed to load font!\n";
    }

    else
    {
        text_particle_count_.setFont(font_);
        text_particle_count_.setCharacterSize(20); // Размер шрифта
        text_particle_count_.setFillColor(sf::Color::Yellow); // Цвет текста
        text_particle_count_.setPosition(10.f, 10.f); // Отступ от левого верхнего угла
    }
}

void GameManager::run()
{
    sf::Clock clock;
    robot_.update_sensors();
    
    while (window_.isOpen())
    {
        sf::Time dt = clock.restart();

        process_events_();
        
        auto [d_fwd, d_th] = handle_input_(dt.asSeconds());

        robot_.update_sensors();

        if (d_fwd != 0.0 || d_th != 0.0)
        {
            robot_.update_amcl();
        }

        render_();
    }
}

void GameManager::process_events_()
{
    sf::Event event;
    while (window_.pollEvent(event))
    {
        if (event.type == sf::Event::Closed)
        {
            window_.close();
        }
    }
}

std::pair<double, double> GameManager::handle_input_(double dt)
{
    const double linear_speed = 3.0;
    const double angular_speed = 2.5;
    double d_fwd = 0.0, d_th = 0.0;

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) d_th  = -angular_speed * dt;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) d_th  =  angular_speed * dt;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) d_fwd =  linear_speed  * dt;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) d_fwd = -linear_speed  * dt;

    if (d_fwd != 0.0 || d_th != 0.0)
    {
        Pose current = robot_.get_real_pos();
        double target_theta = current.theta + d_th;
        double target_x = current.x + d_fwd * std::cos(target_theta);
        double target_y = current.y + d_fwd * std::sin(target_theta);

        if (!map_.is_wall_at_physical(target_x, target_y))
        {
            robot_.move_relative(d_fwd, d_th);
            return {d_fwd, d_th};
        }
    }
    return {0.0, 0.0};
}


void GameManager::draw_map_()
{
    sf::RectangleShape wall_shape(sf::Vector2f(map_.resolution() * ppm_, map_.resolution() * ppm_));
    wall_shape.setFillColor(sf::Color(50, 50, 50));

    for (size_t y = 0; y < map_.height(); ++y)
    {
        for (size_t x = 0; x < map_.width(); ++x)
        {
            if (map_.is_wall_at_grid(x, y))
            {
                wall_shape.setPosition(x * map_.resolution() * ppm_, y * map_.resolution() * ppm_);
                window_.draw(wall_shape);
            }
        }
    }
}



void GameManager::draw_lidar_()
{
    const Lidar& lidar = robot_.get_lidar();
    const auto& dists = lidar.get_distances();
    Pose rp = robot_.get_real_pos();
    int num_beams = lidar.get_num_beams();
    double angle_step = 2.0 * std::numbers::pi / num_beams;

    for (int i = 0; i < num_beams; ++i)
    {
        double angle = normalize_angle(rp.theta + i * angle_step);
        double d = dists[i];

        float hit_x = (rp.x + d * std::cos(angle)) * ppm_;
        float hit_y = (rp.y + d * std::sin(angle)) * ppm_;

        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(rp.x * ppm_, rp.y * ppm_), sf::Color(0, 150, 255, 75)),
            sf::Vertex(sf::Vector2f(hit_x, hit_y), sf::Color(0, 150, 255, 75))
        };
        window_.draw(line, 2, sf::Lines);

        sf::CircleShape dot(3.5f);
        dot.setOrigin(3.5f, 3.5f);
        dot.setPosition(hit_x, hit_y);
        dot.setFillColor(sf::Color(0, 100, 255, 200));
        window_.draw(dot);
    }
}

void GameManager::draw_particles_()
{
    const auto& particles = robot_.get_particles();
    for (const auto& p : particles)
    {
        sf::CircleShape p_shape(1.5f);
        p_shape.setOrigin(1.5f, 1.5f);
        p_shape.setPosition(p.pose.x * ppm_, p.pose.y * ppm_);
        p_shape.setFillColor(sf::Color(50, 200, 50, 180));
        window_.draw(p_shape);
    }

    std::string info = "Particles: " + std::to_string(robot_.get_particles().size());
    text_particle_count_.setString(info);
    window_.draw(text_particle_count_);
}

void GameManager::draw_robot_(const Pose& pose, sf::Color color, bool is_outline)
{
    float robot_radius_m = 0.25f;
    sf::CircleShape shape(robot_radius_m * ppm_);
    shape.setOrigin(robot_radius_m * ppm_, robot_radius_m * ppm_);
    shape.setPosition(pose.x * ppm_, pose.y * ppm_);

    if (is_outline)
    {
        shape.setOutlineColor(color);
        shape.setOutlineThickness(2.0f);
        shape.setFillColor(sf::Color::Transparent);
    }
    else
    {
        shape.setFillColor(color);
        
        sf::VertexArray direction_line(sf::Lines, 2);
        direction_line[0].position = sf::Vector2f(pose.x * ppm_, pose.y * ppm_);
        direction_line[0].color = sf::Color::Black;
        direction_line[1].position = sf::Vector2f(
            (pose.x + std::cos(pose.theta) * robot_radius_m * 1.5) * ppm_,
            (pose.y + std::sin(pose.theta) * robot_radius_m * 1.5) * ppm_
        );
        direction_line[1].color = sf::Color::Black;
        window_.draw(direction_line);
    }
    
    window_.draw(shape);
}

void GameManager::render_()
{
    window_.clear(sf::Color(240, 240, 240));

    draw_map_();
    draw_lidar_();

    draw_robot_(robot_.get_real_pos(), sf::Color::Red);
    draw_robot_(robot_.get_intended_pos(), sf::Color(0, 0, 255, 100));
    draw_robot_(robot_.get_amcl_pose(), sf::Color::Green, true);

    draw_particles_();

    window_.display();
}