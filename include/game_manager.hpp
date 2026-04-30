#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include "amcl.hpp"
#include "robot.hpp"
#include "map.hpp"

class GameManager {
public:
    GameManager(int window_width, int window_height, float pixels_per_meter)
        : window_(sf::VideoMode(window_width, window_height), "AMCL Particle Filter MVP"),
          ppm_(pixels_per_meter),
          map_(16, 16, 1.0), // Карта 16x16 метров, ячейка = 1 метр
          robot_(2.0, 2.0, 0.0, map_) // Робот стартует в координатах (2м, 2м)
    {
        window_.setFramerateLimit(60);
        init_map();
    }

    void run()
    {
        sf::Clock clock;
        robot_.update_sensors();
        
        while (window_.isOpen())
        {
            sf::Time dt = clock.restart();

            process_events();
            
            auto [d_fwd, d_th] = handle_input(dt.asSeconds());

            robot_.update_sensors();

            if (d_fwd != 0.0 || d_th != 0.0)
            {
                robot_.update_amcl();
            }

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

    std::pair<double, double> handle_input(double dt)
    {
        const double linear_speed = 3.0;
        const double angular_speed = 2.5;
        double d_fwd = 0.0, d_th = 0.0;

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) d_th  = -angular_speed * dt;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) d_th  =  angular_speed * dt;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) d_fwd =  linear_speed  * dt;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) d_fwd = -linear_speed  * dt;

        if (d_fwd != 0.0 || d_th != 0.0) {
            Pose current = robot_.get_real_pos();
            double target_theta = current.theta + d_th;
            double target_x = current.x + d_fwd * std::cos(target_theta);
            double target_y = current.y + d_fwd * std::sin(target_theta);

            if (!map_.is_wall_at_physical(target_x, target_y)) {
                robot_.move_relative(d_fwd, d_th);
                return {d_fwd, d_th};
            }
        }
        return {0.0, 0.0};
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

        // 4. Отрисовка лидара
        const Lidar& lidar = robot_.get_lidar();
        const auto& dists = lidar.get_distances();
        Pose rp = robot_.get_real_pos();
        int num_beams = lidar.get_num_beams();
        double angle_step = 2.0 * std::numbers::pi / num_beams;

        for (int i = 0; i < num_beams; ++i)
        {
            double angle = normalize_angle(rp.theta + i * angle_step);
            double d = dists[i];

            // Конечная точка луча
            float hit_x = (rp.x + d * std::cos(angle)) * ppm_;
            float hit_y = (rp.y + d * std::sin(angle)) * ppm_;

            // Рисуем луч (прозрачно-синяя линия)
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(rp.x * ppm_, rp.y * ppm_), sf::Color(0, 150, 255, 75)),
                sf::Vertex(sf::Vector2f(hit_x, hit_y), sf::Color(0, 150, 255, 75))
            };
            window_.draw(line, 2, sf::Lines);

            // Рисуем точку попадания (синяя точка)
            sf::CircleShape dot(3.5f);
            dot.setOrigin(2.0f, 2.0f);
            dot.setPosition(hit_x, hit_y);
            dot.setFillColor(sf::Color(0, 100, 255, 200));
            window_.draw(dot);
        }

        // 2. Отрисовка истинного робота (красный круг)
        Pose true_pose = robot_.get_real_pos();
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
        Pose odom_pose = robot_.get_intended_pos();
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


        // 5. Отрисовка частиц AMCL
        const auto& particles = robot_.get_particles();
        for (const auto& p : particles) {
            sf::CircleShape p_shape(1.5f); // Крошечная точка
            p_shape.setPosition(p.pose.x * ppm_, p.pose.y * ppm_);
            p_shape.setFillColor(sf::Color(50, 200, 50, 180)); // Зеленый с прозрачностью
            window_.draw(p_shape);
        }

        // 6. Отрисовка предполагаемого положения (результат AMCL)
        Pose est = robot_.get_amcl_pose();
        sf::CircleShape est_shape(robot_radius_m * ppm_);
        est_shape.setOrigin(robot_radius_m * ppm_, robot_radius_m * ppm_);
        est_shape.setPosition(est.x * ppm_, est.y * ppm_);
        est_shape.setOutlineColor(sf::Color::Green);
        est_shape.setOutlineThickness(2.0f);
        est_shape.setFillColor(sf::Color::Transparent);
        window_.draw(est_shape);


        window_.display();
    }
};