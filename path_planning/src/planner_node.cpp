#include "path_planning/planner_node.hpp"

PathPlanning::PathPlanning() : Node("path_planning_node") {
    //declare_parameter<std::string>("config_file", "");
    //std::string config_file = get_parameter("config_file").as_string();
    //load_map_config(config_file);
    //
    //map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    //    "/occupancy_grid", 1, 
    //    std::bind(&PathPlanning::callback, this, std::placeholders::_1)
    //);
    //
    path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/path", 10);
    
    path_found = false;
    std::vector<std::vector<int>> grid_;
    grid_ = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0},
             {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
            };

    path_client_ = this->create_client<path_planning::srv::ComputePath>("compute_path");
    while (!path_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(),"Waiting for the path to be computed...");
    }
}

void PathPlanning::load_map_config(const std::string& file_path) {
    YAML::Node config = YAML::LoadFile(file_path);
    resolution = config["map"]["resolution"].as<double>();
    std::vector<int> start = config["path"]["start"].as<std::vector<int>>();
    std::vector<int> goal = config["path"]["goal"].as<std::vector<int>>();
    
    start_ = {static_cast<int>(start[0]/resolution), static_cast<int>(start[1]/resolution)};
    goal_ = {static_cast<int>(goal[0]/resolution), static_cast<int>(goal[1]/resolution)};
}

void PathPlanning::callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!path_found) {
        int width = msg->info.width;
        int height = msg->info.height;

        RCLCPP_DEBUG(this->get_logger(), "Received Occupancy Grid: %dx%d", width, height);

        auto request = std::make_shared<path_planning::srv::ComputePath::Request>();
        auto start_msg = geometry_msgs::msg::Point();
        auto goal_msg = geometry_msgs::msg::Point();

        start_msg.x = start_.x;
        start_msg.y = start_.y;
        goal_msg.x = goal_.x;
        goal_msg.y = goal_.y;
        request->grid = *msg;
        request->start = start_msg;
        request->goal = goal_msg;

        auto result = path_client_->async_send_request(request);
        auto response = result.get();
        // Wait for the result.
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Success computing path");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service ComputePath");    
        }
        
        path_found = true;
    }
    publish_path();
}

void PathPlanning::goal_markers(visualization_msgs::msg::MarkerArray& marker_array) {
    // publish start
    visualization_msgs::msg::Marker sph_marker;
    sph_marker.header.frame_id = "map";
    sph_marker.header.stamp = now();
    sph_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sph_marker.action = visualization_msgs::msg::Marker::ADD;
    sph_marker.ns = "points";
    sph_marker.scale.x = 0.5;
    sph_marker.scale.y = 0.5;
    sph_marker.scale.z = 0.5;

    // publish start
    sph_marker.id = 0;
    sph_marker.color.r = 0.05;
    sph_marker.color.g = 0.90;
    sph_marker.color.b = 0.05;
    sph_marker.color.a = 0.99;
    sph_marker.pose.position.x = start_.x * resolution;
    sph_marker.pose.position.y = start_.y * resolution;
    RCLCPP_DEBUG(this->get_logger(), "Start: {%f, %f}", start_.x* resolution, start_.y* resolution);
    marker_array.markers.push_back(sph_marker);

    // publish goal
    sph_marker.id = 1;
    sph_marker.color.r = 0.90;
    sph_marker.color.g = 0.01;
    sph_marker.color.b = 0.01;
    sph_marker.color.a = 0.99;
    sph_marker.pose.position.x = goal_.x * resolution;
    sph_marker.pose.position.y = goal_.y * resolution;
    RCLCPP_DEBUG(this->get_logger(), "Goal: {%f, %f}", goal_.x* resolution, goal_.y* resolution);
    marker_array.markers.push_back(sph_marker);   
}

void PathPlanning::path_markers(visualization_msgs::msg::MarkerArray& marker_array) {
    // publish path markers
    int id = 0; 
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "path";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.15;  // Line width

    marker.color.r = 0.97;
    marker.color.g = 0.62;
    marker.color.b = 0.11;
    marker.color.a = 0.99;
    for (const auto& vec : path_) {
        marker.id = id++;
        geometry_msgs::msg::Point p;
        p.x = vec.x* resolution;
        p.y = vec.y* resolution;
        RCLCPP_DEBUG(this->get_logger(), "Path: {%f, %f}", vec.x* resolution, vec.y* resolution);
        marker.points.push_back(p);
    }
    marker_array.markers.push_back(marker);    
}

void PathPlanning::publish_path() { 
    // Add markers to marker_array and publish
    visualization_msgs::msg::MarkerArray marker_array;
    goal_markers(marker_array);
    //path_markers(marker_array);
    path_pub_->publish(marker_array);    
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
