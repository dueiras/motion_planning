#include "path_planning/grid_map.hpp"

GridMap::GridMap() : Node("grid_map_node") {
    declare_parameter<std::string>("config_file", "");
    std::string config_file = get_parameter("config_file").as_string();

    loadMapConfig(config_file);
    generateOccupancyGrid();

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 1);

    map_msg_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    map_timer_ = create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&GridMap::publishMap, this)
    );

    obs_timer_ = create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&GridMap::publishObstacles, this)
    );
}

void GridMap::loadMapConfig(const std::string& file_path) {
    // load yaml file and apply resolution to map
    config = YAML::LoadFile(file_path);
    map_width_ = config["map"]["width"].as<int>();
    map_height_ = config["map"]["height"].as<int>();
    resolution_ = config["map"]["resolution"].as<double>();
    inflate_ = config["map"]["inflate"].as<double>();
    inflate_ = inflate_ / resolution_;

    map_width_ = static_cast<int>(static_cast<double>(map_width_) / resolution_);
    map_height_ = static_cast<int>(static_cast<double>(map_height_) / resolution_);

    grid_.assign(map_height_, std::vector<int>(map_width_, 0));

    for (const auto& obs : config["obstacles"]) {
        Obstacle obstacle;
        obstacle.size = 0;
        for (const auto& vertex : obs["vertices"]) {
            int x = static_cast<int>(vertex[0].as<double>() / resolution_);
            int y = static_cast<int>(vertex[1].as<double>() / resolution_);

            // Ensure obstacles don't go out of bounds
            x = std::clamp(x, 0, map_width_ - 1);
            y = std::clamp(y, 0, map_height_ - 1);

            obstacle.vertices.emplace_back(x, y);
            obstacle.size ++;
        }
        obstacles_.emplace_back(obstacle);
    }
}

Point GridMap::vertice_to_point(const std::pair<int, int>& vertice) {
    return {static_cast<double>(vertice.first), static_cast<double>(vertice.second)};
}

void GridMap::inflate_point(const Point& point) {
    auto [x,y] = point;
    int inflate = std::ceil(inflate_);
    // Inflate surrounding cells
    for (int dx = -inflate; dx <= inflate; ++dx) {
        for (int dy = -inflate; dy <= inflate; ++dy) {
            int nx = x + dx, ny = y + dy;
            if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                if (grid_[ny][nx] < 100) {
                    grid_[ny][nx] = 50;
                }                
            }
        }
    }
}

bool GridMap::is_obstacle(const Point& point, const Obstacle& obstacle) {
    // check if a point is inside an obstacle
    // main idea is to check whether a horizontal line from point
    // crosses an obstacle more than once
    // source: https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
    int num_vertices = obstacle.size;
    int x = point.x, y = point.y;
    bool inside = false;
 
    Point p1 = vertice_to_point(obstacle.vertices[0]), p2;
 
    // loop each edge in obstacle
    for (int i = 1; i <= num_vertices; i++) {
        // get next point
        p2 = vertice_to_point(obstacle.vertices[i % num_vertices]);
 
        // check if point is above minimum y coordinate 
        if (y >= std::min(p1.y, p2.y)) {
            // check if point is below maximum y coordinate
            if (y < std::max(p1.y, p2.y)) {
                // check if the point is to the left of the
                // maximum x coordinate of the edge
                if (x < std::max(p1.x, p2.x)) {
                    // calculate the x-intersection of the
                    // line connecting the point to the edge
                    double x_intersection = (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
 
                    // check if point is on same line as the edge or to the left of x-intersection
                    if (p1.x == p2.x || x < x_intersection) {
                        inside = !inside;
                    }
                }
            }
        }
 
        // store the current point as the first point for next iteration
        p1 = p2;
    }
 
    return inside;
}

void GridMap::generateOccupancyGrid() {
    // occupy grid with obstacles
    // iterate only in obstacle's bounding box instead of entire map
    for (const Obstacle& obs : obstacles_) {
        // find min and max points of obstacle
        IntPoint point_min{std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
        IntPoint point_max{std::numeric_limits<int>::lowest(), std::numeric_limits<int>::lowest()};

        for (auto& [x_v, y_v] : obs.vertices) {
            if (x_v < point_min.x) point_min.x = x_v;
            if (y_v < point_min.y) point_min.y = y_v;
            if (x_v > point_max.x) point_max.x = x_v;
            if (y_v > point_max.y) point_max.y = y_v;
        }

        // iterate through all points in the bounding box
        for (int x = point_min.x-1; x <= point_max.x+1; ++x) {
            for (int y = point_min.y-1; y <= point_max.y+1; ++y) {
                Point p{(double) x, (double) y};
                if (is_obstacle(p, obs)) {
                    grid_[y][x] = 100;  // Mark as occupied
                    inflate_point(p);
                }
            }
        }
    }
}

void GridMap::publishMap() {
    // publish occupancygrid

    map_msg_->header.stamp = now();
    if (map_msg_->data.empty()){
        RCLCPP_INFO(this->get_logger(), "Generating map message");
        map_msg_->header.frame_id = "map";
        map_msg_->info.resolution = resolution_;
        map_msg_->info.width = map_width_;
        map_msg_->info.height = map_height_;
        map_msg_->info.origin.position.x = 0.0;
        map_msg_->info.origin.position.y = 0.0;
        map_msg_->info.origin.position.z = 0.0;
        map_msg_->info.origin.orientation.x = 0.0;
        map_msg_->info.origin.orientation.y = 0.0;
        map_msg_->info.origin.orientation.z = 0.0;
        map_msg_->info.origin.orientation.w = 1.0;
        map_msg_->data.resize(map_width_ * map_height_); 
        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                map_msg_->data[y * map_width_ + x] = grid_[y][x];
            }
        }
    }

    map_pub_->publish(*map_msg_);
}

void GridMap::publishObstacles() {
    // publish obstacle markers
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto& obs : obstacles_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "obstacles";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;  // Line width

        marker.color.r = 1.0;
        marker.color.a = 1.0;

        for (const auto& [x, y] : obs.vertices) {
            geometry_msgs::msg::Point p;
            p.x = x * resolution_;
            p.y = y * resolution_;
            RCLCPP_DEBUG(this->get_logger(), "Obstacles: {%f, %f}", x * resolution_, y * resolution_);
            marker.points.push_back(p);
        }

        marker.points.push_back(marker.points.front()); // Close the shape
        marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridMap>());
    rclcpp::shutdown();
    return 0;
}
