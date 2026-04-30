cmake_minimum_required(VERSION 3.15)
project(amcl_game LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(SFML COMPONENTS graphics window system REQUIRED)

add_executable(amcl_game
    source/main.cpp    
)

target_include_directories(amcl_game
    PRIVATE
        include
)

target_link_libraries(amcl_game
    PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
)
