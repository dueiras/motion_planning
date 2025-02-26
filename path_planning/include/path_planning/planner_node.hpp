#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "path_planning/a_star.hpp"
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include "path_planning/srv/compute_path.hpp"

class PathPlanning : public rclcpp::Node {
public:
    PathPlanning();
    void publish_path();
    void path_markers(visualization_msgs::msg::MarkerArray& marker_array);
    void goal_markers(visualization_msgs::msg::MarkerArray& marker_array);
    AStar::Vec start_, goal_;
    AStar::Planning planner;

private:
    bool path_found;
    double resolution;
    visualization_msgs::msg::MarkerArray marker_array_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
    rclcpp::Client<path_planning::srv::ComputePath>::SharedPtr path_client_;
    std::vector<std::vector<int>> grid_;
    void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    std::vector<AStar::Vec> path_;
    void load_map_config(const std::string& file_path);
    bool get_path();

};

#endif // PLANNER_HPP