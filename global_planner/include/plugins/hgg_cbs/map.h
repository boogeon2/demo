#ifndef MAP_H
#define MAP_H

#include "iostream"
#include "string"
#include "algorithm"
#include "sstream"
#include "fstream"
#include "vector"
#include "unordered_map"
#include "tinyxml2.h"
#include "const.h"
#include "structs.h"
#include "config.h"
#include "yaml-cpp/yaml.h"

class Map
{
private:
    std::vector<std::vector<int>> grid;
    std::vector<gNode> nodes;
    std::vector<Edge> edges;
    std::vector<Obstacle> obstacles;
    std::vector<std::vector<Node>> valid_moves;
    // Before changing obs_type --------------------------------------------------------------------
    std::unordered_map<std::pair<int, int>, ObstacleInfo, PairHash> obstacle_info;
    // Before changing obs_type --------------------------------------------------------------------

    // After changing obs_type ---------------------------------------------------------------------
    std::unordered_map<std::pair<int, int>, ObsEdgeInfo, PairHash> obstacle_edge_info;
    // After changing obs_type ---------------------------------------------------------------------
    int height, width, size;
    int connectedness;
    double agent_size;
    bool map_is_roadmap;
    bool check_line(int x1, int y1, int x2, int y2);
    bool get_grid(const char *FileName);
    bool get_roadmap(const char *FileName);

public:
    Map() {}
    Map(double size, int k)
    {
        agent_size = size;
        connectedness = k;
    }
    Map(double size)
    {
        agent_size = size;
        map_is_roadmap = true;
    }
    ~Map() {}
    int get_size() const { return size; }
    bool get_map(std::string &map_file_name);
    int get_edge_size() const { return edges.size(); }
    int get_node_size() const { return nodes.size(); }
    bool is_roadmap() const { return map_is_roadmap; }
    bool cell_is_obstacle(int i, int j) const;
    int get_width() const { return width; }
    gNode get_gNode(int id) const
    {
        if (id < int(nodes.size()))
            return nodes[id];
        return gNode();
    }
    Edge get_edge(int id) const
    {
        if (id < int(edges.size()))
            return edges[id];
        return Edge();
    }
    Edge get_edge(int entry, int exit) const;
    // Before changing obs_type --------------------------------------------------------------------
    bool add_obstacle_info(const ObstacleInfo &obs_info);
    // Before changing obs_type --------------------------------------------------------------------

    // After changing obs_type ---------------------------------------------------------------------
    bool add_obs_edge_info(const ObsEdgeInfo &obs_edge_info);
    // After changing obs_type ---------------------------------------------------------------------
    void add_obstacle(Obstacle obs) { obstacles.emplace_back(obs); }
    std::vector<Obstacle> get_obstacles() const { return obstacles; }
    // Before changing obs_type --------------------------------------------------------------------
    const std::unordered_map<std::pair<int, int>, ObstacleInfo, PairHash> get_obstacle_info() const { return obstacle_info; }
    // Before changing obs_type --------------------------------------------------------------------

    // After changing obs_type ---------------------------------------------------------------------
    const std::unordered_map<std::pair<int, int>, ObsEdgeInfo, PairHash> get_obs_edge_info() const { return obstacle_edge_info; }
    // After changing obs_type ---------------------------------------------------------------------
    double get_i(int id) const;
    double get_j(int id) const;
    int get_id(double i, double j) const;
    std::vector<Node> get_valid_moves(int id) const;
    void print_map();
    void printPPM();
};

#endif // MAP_H
