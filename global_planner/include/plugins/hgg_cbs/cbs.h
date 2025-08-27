#ifndef CBS_H
#define CBS_H
#include "chrono"
#include "random"
#include "structs.h"
#include "map.h"
#include "pdtask.h"
#include "config.h"
#include "sipp.h"
#include "heuristic.h"

class CBS
{
public:
    CBS() {}
    bool update_root(const Map &map, PDTask &task);
    bool replan_root(const Map &map, PDTask &task, const std::vector<sPath> &prev_paths, std::vector<double> progressed_time);

    bool check_intersection_conflict(Move move1, Move move2);
    bool check_cross_conflict(sNode a1, sNode a2, sNode b1, sNode b2);
    bool isSingleLane(sNode n);
    bool isSingleLane(Move m);
    double get_hl_heuristic(const std::list<Conflict> &conflicts);
    Constraint get_replan_constraint(int agent, Move move1, Move move2);

    Solution update_solution(const Map &map, PDTask &task, const Config &cfg, int capacity);
    Solution replan_solution(const Map &map, PDTask &task, const Config &cfg, const std::vector<sPath> &prev_paths, std::vector<double> progressed_time);
    std::list<Constraint> get_constraints(CBS_Node *node, int agent_id = -1);
    Conflict check_replan_paths(const sPath &pathA, const sPath &pathB);
    std::vector<Conflict> get_all_replan_conflicts(const std::vector<sPath> &paths, int id);

    void replan_new_conflicts(const Map &map, const PDTask &task, CBS_Node &node, std::vector<sPath> &paths, const sPath &path,
                              const std::list<Conflict> &conflicts, const std::list<Conflict> &semicard_conflicts, const std::list<Conflict> &cardinal_conflicts,
                              int &low_level_searches, int &low_level_expanded);
    double get_cost(CBS_Node node, int agent_id);
    std::vector<sPath> get_paths(CBS_Node *node, unsigned int agents_size);

    Conflict get_conflict(std::list<Conflict> &conflicts);
    void parse_node_info(const YAML::Node &info);
    void parse_guide_motion(const YAML::Node &inner);
    void summarize_conflicts(int num_agent, const std::list<Conflict> &conflicts, const std::list<Conflict> &semi, const std::list<Conflict> &cardi);
    void generate_random_obstacles(Map &map, int num_obs);
    // Before changing obs_type --------------------------------------------------------------------
    void add_obstacle(Map &map, Obstacle obs);
    // Before changing obs_type --------------------------------------------------------------------

    // After changing obs_type ---------------------------------------------------------------------
    void register_obstacle(Map &map, std::vector<Obstacle> new_obstacles);
    // After changing obs_type ---------------------------------------------------------------------
    std::vector<Eigen::Vector2d> interpolatePoints(const std::vector<Eigen::Vector2d> &pts, int pointsPerSegment);
    Eigen::MatrixXd generalizedBezierPoints(const std::vector<Eigen::Vector2d> &pts, double step);
    std::vector<timedPoint> generatePPTrajectory(const std::vector<Eigen::Vector2d> &pts, int interp_num, double bezier_step, double v_min = 0.1, double v_ref = 0.6, double w_max = 2.0, double dt = 0.1, double lookahead_dist = 0.25);
    CBS_Tree tree;
    SIPP planner;
    Solution solution;
    Solution new_solution;
    Heuristic h_values;
    Config config;
    const Map *map;
    std::vector<Node_info> node_info;
    std::unordered_map<std::pair<int, int>, Intermotion, PairHash> inter_motion;
    std::unordered_map<std::tuple<int, int, int>, Innermotion, TupleHash> inner_motion;
    std::unordered_map<std::tuple<int, int, int, int, int>, std::vector<std::pair<double, double>>, IntersectionHash> inner_collision_interval;
};

#endif // CBS_H
