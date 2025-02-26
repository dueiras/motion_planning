#include "path_planning/a_star.hpp"
#include <math.h>

using namespace std::placeholders;

bool AStar::Vec::operator == (const Vec& coordinates_) 
{
    return ((x == coordinates_.x) && (y == coordinates_.y));
}

AStar::Node::Node(Vec coord_, Node* parent_)
{
    parent = parent_;
    coordinates = coord_;
    G = 0;
    H = 0;
}

unsigned int AStar::Node::get_score()
{
    return G + H;
}


AStar::Vec AStar::Heuristic::get_delta(Vec start_, Vec goal_)
{
    return {abs(start_.x - goal_.x), abs(start_.y - goal_.y)};
}

unsigned int AStar::Heuristic::euclidean(Vec start_, Vec goal_)
{   
    // times 10 so diagonal values can also be (int)
    // 10 and 14 instead of 1.0 and sqrt(2)=1.4 
    auto delta = get_delta(start_, goal_);
    return static_cast<unsigned int>(10 * sqrt(pow(delta.x,2)+pow(delta.y,2)));
}

unsigned int AStar::Heuristic::manhattan(Vec start_, Vec goal_)
{   
    // times 10 so diagonal values can also be (int)
    auto delta = get_delta(start_, goal_);
    return static_cast<unsigned int>(10 * (delta.x+delta.y));
}

AStar::Planning::Planning()
{
    set_heuristic(&Heuristic::euclidean);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Planning::set_grid(std::vector<std::vector<int>> grid) {
    grid_ = grid;
}

void AStar::Planning::set_world_size(Vec world_size_)
{
    world_size = world_size_;
}

void AStar::Planning::set_heuristic(std::function<unsigned int(Vec, Vec)> heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2); // first and second argument of func
}

bool AStar::Planning::is_collision(Vec coordinates_)
{
    // the grid contains int from 0 to 100 for the probability of being occupied
    if (coordinates_.x >= world_size.x || coordinates_.x < 0 ||
        coordinates_.y >= world_size.y || coordinates_.x < 0) {
            return (true);
    }
    if (grid_[coordinates_.y][coordinates_.x] > 0) {
        return (true);
    }
    return (false);
}

AStar::Node* AStar::Planning::find_node(std::vector<Node*> nodes_, Vec coordinates_)
{
    for (auto node: nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Planning::release_nodes(std::vector<Node*> nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

std::vector<AStar::Vec> AStar::Planning::find_path(Vec start_, Vec goal_)
{
    std::vector<Vec> path;

    Node *current = nullptr;
    std::vector<Node*> open_set, closed_set;
    open_set.reserve(100);
    closed_set.reserve(100);
    open_set.push_back(new Node(start_));

    if (is_collision(start_)){
        std::cout << "Collision on Start point..." << "\n";
        return path;
    }

    else if (is_collision(goal_)){
        std::cout << "Collision on End point..." << "\n";
        return path;
    }
    while (!open_set.empty()) {
        auto current_it = open_set.begin();
        current = *current_it;

        for (auto it = open_set.begin(); it != open_set.end(); it++) {
            auto node = *it;
            if (node->get_score() <= current->get_score()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == goal_) {
            break;
        }

        closed_set.push_back(current);
        open_set.erase(current_it);

        // diagonal movement has 8 directions
        for (unsigned int i = 0; i < 8; ++i) {
            Vec new_coordinates(current->coordinates + direction[i]);
            if (find_node(closed_set, new_coordinates)) {
                continue;
            }
            if (is_collision(new_coordinates)) {
                continue;
            }

            // first 4 directions are horizontal and vertical,
            // last 4 are diagnoal ~ 10 * sqrt(2) ~ 14
            unsigned int total_cost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = find_node(open_set, new_coordinates);
            if (successor == nullptr) {
                successor = new Node(new_coordinates, current);
                successor->G = total_cost;
                successor->H = heuristic(successor->coordinates, goal_);
                open_set.push_back(successor);
            }
            else if (total_cost < successor->G) {
                successor->parent = current;
                successor->G = total_cost;
            }
        }
    } // while

    std::cout << "I finished computing the path\n"; 
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }
    std::cout << "I wrote the path\n"; 

    release_nodes(open_set);
    release_nodes(closed_set);

    std::cout << "I cleared the sets\n";

    return path;
}