#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include "robot.hpp"
#include "map.hpp"

class GameManager {
public:
    GameManager(int window_width, int window_height, float pixels_per_meter)
        : window_(sf::VideoMode(window_width, window_height), "AMCL Particle Filter MVP"),
          ppm_(pixels_per_meter),
          map_(16, 16, 1.0), // Карта 16x16 метров, ячейка = 1 метр
          robot_(2.0, 2.0, 0.0) // Робот стартует в координатах (2м, 2м)
    {
        window_.setFramerateLimit(60);
        init_map();
    }

    void run() {
        sf::Clock clock;
        while (window_.isOpen()) {
            sf::Time dt = clock.restart();

            process_events();
            handle_input(dt.asSeconds());
            
            // В будущем здесь будет вызов: robot_.run_amcl();

            render();
        }
    }

private:
    sf::RenderWindow window_;
    float ppm_; // Масштаб: пикселей на метр
    Map map_;
    Robot robot_;

    void init_map() {
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

    void process_events() {
        sf::Event event;
        while (window_.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window_.close();
            }
        }
    }

    void handle_input(double dt) {
        const double linear_speed = 3.0;  // м/с
        const double angular_speed = 2.5; // рад/с

        double d_forward = 0.0;
        double d_theta = 0.0;

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) d_theta = -angular_speed * dt;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) d_theta = angular_speed * dt;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) d_forward = linear_speed * dt;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) d_forward = -linear_speed * dt;

        if (d_forward != 0.0 || d_theta != 0.0) {
            Pose current = robot_.GetRealPos();
            
            double target_theta = current.theta + d_theta;
            double target_x = current.x + d_forward * std::cos(target_theta);
            double target_y = current.y + d_forward * std::sin(target_theta);

            // Проверка коллизии: не даем роботу въехать в стену
            if (!map_.is_wall_at_physical(target_x, target_y)) {
                robot_.move_relative(d_forward, d_theta);
            }
        }
    }

    void render() {
        window_.clear(sf::Color(240, 240, 240)); // Светло-серый фон

        // 1. Отрисовка карты
        sf::RectangleShape wall_shape(sf::Vector2f(map_.resolution() * ppm_, map_.resolution() * ppm_));
        wall_shape.setFillColor(sf::Color(50, 50, 50)); // Темно-серые стены

        for (size_t y = 0; y < map_.height(); ++y) {
            for (size_t x = 0; x < map_.width(); ++x) {
                if (map_.is_wall_at_grid(x, y)) {
                    wall_shape.setPosition(x * map_.resolution() * ppm_, y * map_.resolution() * ppm_);
                    window_.draw(wall_shape);
                }
            }
        }

        // 2. Отрисовка истинного робота (красный круг)
        Pose true_pose = robot_.GetRealPos();
        float robot_radius_m = 0.25f; // Физический радиус робота 25 см
        
        sf::CircleShape robot_shape(robot_radius_m * ppm_);
        robot_shape.setOrigin(robot_radius_m * ppm_, robot_radius_m * ppm_);
        robot_shape.setPosition(true_pose.x * ppm_, true_pose.y * ppm_);
        robot_shape.setFillColor(sf::Color::Red);
        window_.draw(robot_shape);

        // Линия направления (вектор theta)
        sf::VertexArray direction_line(sf::Lines, 2);
        direction_line[0].position = sf::Vector2f(true_pose.x * ppm_, true_pose.y * ppm_);
        direction_line[0].color = sf::Color::Black;
        direction_line[1].position = sf::Vector2f(
            (true_pose.x + std::cos(true_pose.theta) * robot_radius_m * 1.5) * ppm_,
            (true_pose.y + std::sin(true_pose.theta) * robot_radius_m * 1.5) * ppm_
        );
        direction_line[1].color = sf::Color::Black;
        window_.draw(direction_line);


        // 3. Отрисовка одометрии (синий полупрозрачный круг - "призрак")
        // Наглядно показывает, как накапливается ошибка!
        Pose odom_pose = robot_.GetIntendedPos();
        sf::CircleShape odom_shape(robot_radius_m * ppm_);
        odom_shape.setOrigin(robot_radius_m * ppm_, robot_radius_m * ppm_);
        odom_shape.setPosition(odom_pose.x * ppm_, odom_pose.y * ppm_);
        odom_shape.setFillColor(sf::Color(0, 0, 255, 100)); // Полупрозрачный синий
        window_.draw(odom_shape);

        // Линия направления одометрии (вектор theta)
        sf::VertexArray odom_direction_line(sf::Lines, 2);
        odom_direction_line[0].position = sf::Vector2f(odom_pose.x * ppm_, odom_pose.y * ppm_);
        odom_direction_line[0].color = sf::Color::Black;
        odom_direction_line[1].position = sf::Vector2f(
            (odom_pose.x + std::cos(odom_pose.theta) * robot_radius_m * 1.5) * ppm_,
            (odom_pose.y + std::sin(odom_pose.theta) * robot_radius_m * 1.5) * ppm_
        );
        odom_direction_line[1].color = sf::Color::Black;
        window_.draw(odom_direction_line);

        window_.display();
    }
};