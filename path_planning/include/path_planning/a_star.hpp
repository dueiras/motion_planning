#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <set>
#include <functional>
#include <iostream>

namespace AStar
{
    struct Vec
    {
        int x,y;

        bool operator == (const Vec& coordinates_);
        friend Vec operator +(const Vec& left_, const Vec& right_){
            return {left_.x + right_.x, left_.y + right_.y};
        }

    };

    struct Node
    {
        unsigned int G, H;
        Vec coordinates;
        Node *parent;

        Node(Vec coord_, Node *parent_ = nullptr);
        unsigned int get_score();
    };

    class Heuristic 
    {
        static Vec get_delta(Vec start_, Vec goal_);

    public:
        static unsigned int manhattan(Vec start_, Vec goal_);
        static unsigned int euclidean(Vec start_, Vec goal_);
    };

    class Planning
    {
        bool is_collision(Vec coordinates_);
        Node* find_node(std::vector<Node*> nodes_, Vec coordinates_);
        void release_nodes(std::vector<Node*> nodes_);

    public:
        Planning();
        void set_world_size(Vec world_size_);
        void set_grid(std::vector<std::vector<int>> grid);
        void set_heuristic(std::function<unsigned int(Vec, Vec)> heuristic_);
        std::vector<Vec> find_path(Vec start_, Vec goal_);

    private:
        std::vector<std::vector<int>> grid_;
        std::function<unsigned int(Vec, Vec)> heuristic;
        std::vector<Vec> direction, wall;
        Vec world_size;
        
    };
}

#endif // ASTAR_HPP