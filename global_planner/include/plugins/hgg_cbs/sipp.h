#ifndef SIPP_H
#define SIPP_H
#include "structs.h"
#include "map.h"
#include "heuristic.h"
#include "pdtask.h"
#include "unordered_map"
#include "tuple"
#include "map"
#include "set"
#include "unordered_set"

class SIPP
{
    // Modification Details - last update: 2023.02.19 (jw)
    // 1. add 'loop' input parameter to find_path() function
public:
    SIPP() {}
    ~SIPP() {}

    void init_predefined_info(const std::vector<Node_info> &node_info, const std::unordered_map<std::pair<int, int>, Intermotion, PairHash> &inter_motion, const std::unordered_map<std::tuple<int, int, int>, Innermotion, TupleHash> &inner_motion, const std::unordered_map<std::tuple<int, int, int>, double, TupleHash> &uh_val);
    void update_heuristic_info(const std::unordered_map<std::tuple<int, int, int>, double, TupleHash> &uh_val) { uh_value = uh_val; }
    Path find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values);
    Path update_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values);
    Path replan_initial_path(int agent_id, PDTask &task, const Map &map, Heuristic &h_values, const sPath &prev_path, double current_time);
    Path replan_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values);

private:
    Agent agent;
    ReplanAgent rAgent;
    std::vector<Path> find_partial_path(std::vector<Node> starts, std::vector<Node> goals, const Map &map, Heuristic &h_values, double max_f = CN_INFINITY);
    std::vector<Path> update_partial_path(std::vector<Node> starts, std::vector<Node> goals, const Map &map, Heuristic &h_values, double max_f = CN_INFINITY);
    std::vector<Path> replan_partial_path(std::vector<Node> starts, std::vector<Node> goals, const Map &map, Heuristic &h_values, double max_f = CN_INFINITY);
    Path add_part(Path result, Path part);
    void find_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values, Node goal);
    void update_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values, std::vector<Node> goals);
    void replan_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values, std::vector<Node> goals);
    void add_open(Node newNode);
    Node find_min();
    double dist(const Node &a, const Node &b);
    double custom_cost(const Node &a, const Node &b);
    int get_direction(const Node &a, const Node &b);
    std::vector<Node> reconstruct_path(Node curNode);
    std::vector<Node> reconstruct_custom_path(Node curNode);
    void make_constraints(std::list<Constraint> &cons);
    void update_constraints(std::list<Constraint> &cons);
    void clear();
    void add_collision_interval(int id, std::pair<double, double> interval);
    void update_collision_interval(const Constraint &cons);
    void add_move_constraint(Move move);
    void update_intersection_constraint(const Constraint &cons);
    void update_cross_constraint(const Constraint &cons);
    void update_move_constraint(const Constraint &cons);
    std::vector<Node> get_endpoints(int node_id, double node_i, double node_j, double t1, double t2);
    std::vector<Node> get_custom_endpoints(Node n);
    double check_endpoint(Node start, Node goal);

    std::unordered_map<int, Node> close;
    std::unordered_map<std::pair<int, int>, Node, PairHash> custom_close;       // store <id, type>
    std::unordered_map<std::tuple<int, int, int>, Node, TupleHash> label_close; // store <id, type*NODE_SIZE + orientation, label>
    std::list<Node> open;
    std::unordered_map<int, std::pair<double, bool>> visited;
    std::unordered_map<std::pair<int, int>, std::pair<double, bool>, PairHash> custom_visited;       // store <id, type>
    std::unordered_map<std::tuple<int, int, int>, std::pair<double, bool>, TupleHash> label_visited; // store <id, type*NODE_SIZE + orientation, label>
    std::map<std::pair<int, int>, std::vector<Move>> constraints;                                    // stores sets of constraints associated with moves

    std::unordered_map<int, std::vector<std::pair<double, double>>> collision_intervals;                                     // stores sets of collision intervals associated with cells
    std::unordered_map<std::tuple<int, int, int>, std::vector<std::pair<double, double>>, TupleHash> intersection_intervals; // key value is std::make_tuple(id, type, orientation)
    std::map<std::tuple<int, int, int>, std::vector<Move>> intersection_constraints;                                         // key value is std::make_tuple(id, orientation1, orientation2)
    std::map<std::tuple<int, int, int>, std::vector<Move>> cross_constraints;                                                // key value is std::make_tuple(cur_id, cur_type, new_id)
    std::map<std::tuple<int, int, int, int>, std::vector<Move>> move_constraints;                                            // key value is std::make_tuple(cur_id, cur_type, new_id, new_type)
    std::vector<Move> landmarks;
    Path path;

    // Before changing obs_type --------------------------------------------------------------------
    std::unordered_map<std::pair<int, int>, ObstacleInfo, PairHash> obs_info; // Key: {entry node idx, exit node idx}, Value: {ObstacleInfo}
    // Before changing obs_type --------------------------------------------------------------------

    // After changing obs_type ---------------------------------------------------------------------
    std::unordered_map<std::pair<int, int>, ObsEdgeInfo, PairHash> obs_edge_info; // Key: {entry node idx, exit node idx}, Value: {ObstacleEdgeInfo}
    // After changing obs_type ---------------------------------------------------------------------

    std::vector<Node_info> node_info;
    std::unordered_map<std::pair<int, int>, Intermotion, PairHash> inter_motion;
    std::unordered_map<std::tuple<int, int, int>, Innermotion, TupleHash> inner_motion;
    std::unordered_map<std::tuple<int, int, int>, double, TupleHash> uh_value;
    double start_time = 0.0;
};

#endif // SIPP_H
