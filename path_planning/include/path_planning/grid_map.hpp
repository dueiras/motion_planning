#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>

struct Obstacle {
    std::vector<std::pair<int, int>> vertices;
    int size;
};

struct Point {
    double x, y;
};

struct IntPoint {
    int x, y;
};

class GridMap : public rclcpp::Node {
public:
    GridMap();

private:
    void loadMapConfig(const std::string& file_path);
    Point vertice_to_point(const std::pair<int, int>& vertice);
    bool is_obstacle(const Point& point, const Obstacle& obstacle);
    void inflate_point(const Point& point);
    void generateOccupancyGrid();
    void publishMap();
    void publishObstacles();

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    int map_width_, map_height_;
    double resolution_, inflate_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> map_msg_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr obs_timer_;
    std::vector<std::vector<int>> grid_;
    std::vector<Obstacle> obstacles_;
    YAML::Node config;
};

#endif // GRID_MAP_HPP
