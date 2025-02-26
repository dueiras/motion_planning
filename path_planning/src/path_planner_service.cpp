#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "path_planning/a_star.hpp"
#include "path_planning/srv/compute_path.hpp"

class PathPlannerService : public rclcpp::Node
{
public:
    PathPlannerService() : Node("path_planner_service")
    {
        service_ = this->create_service<path_planning::srv::ComputePath>(
            "compute_path",
            std::bind(&PathPlannerService::compute_path_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Path Planner Service is ready.");
    }

private:
    void compute_path_callback(
        const std::shared_ptr<path_planning::srv::ComputePath::Request> request,
        std::shared_ptr<path_planning::srv::ComputePath::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received path planning request.");

        int width = request->grid.info.width;
        int height = request->grid.info.height;
        AStar::Vec start = {(int) request->start.x, (int) request->start.y};
        AStar::Vec goal = {(int) request->goal.x, (int) request->goal.y};

        grid_.assign(height, std::vector<int>(width, -1));

        // Fill grid with occupancy values from message
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                grid_[y][x] = request->grid.data[y * width + x];  // Row-major order
            }
        }

        planner.set_grid(grid_);
        AStar::Vec world_size = {width, height};
        planner.set_world_size(world_size);
        path_ = planner.find_path(start, goal);

        if (path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Not able to compute path");
            response->success = false;
        }
        else {
            nav_msgs::msg::Path path;
            geometry_msgs::msg::PoseStamped waypoint;

            for(long unsigned int i = 0; i < path_.size(); i++) {
                waypoint.pose.position.x = path_[i].x;
                waypoint.pose.position.y = path_[i].y;
                path.poses.push_back(waypoint);
            }
    
            response->path = path;
            response->success = true;
    
            RCLCPP_INFO(this->get_logger(), "Path planning completed.");
        }
    }

    rclcpp::Service<path_planning::srv::ComputePath>::SharedPtr service_;
    AStar::Planning planner;
    std::vector<AStar::Vec> path_;
    std::vector<std::vector<int>> grid_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerService>());
    rclcpp::shutdown();
    return 0;
}
