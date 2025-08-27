#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "structs.h"
#include "const.h"
#include "vector"
#include "queue"
#include "unordered_map"
#include "map.h"

typedef multi_index_container<
    Node,
    indexed_by<
        // ordered_non_unique<tag<cost>, BOOST_MULTI_INDEX_MEMBER(Open_Elem, double, cost)>,
        ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, double, g)>,
        hashed_unique<BOOST_MULTI_INDEX_MEMBER(Node, int, id)>>>
    Open_Container;

class Heuristic
{
    using INTERMOTION = std::unordered_map<std::pair<int, int>, Intermotion, PairHash>;
    using INNERMOTION = std::unordered_map<std::tuple<int, int, int>, Innermotion, TupleHash>;
    using UVALUE = std::unordered_map<std::tuple<int, int, int>, double, TupleHash>;

    std::vector<std::vector<double>> h_values;
    UVALUE uh_values; //<curr_node_id, curr_node_type*NODE_SIZE + curr_node_orientation, target_node_id> (orientation is no larger than node size)
    Open_Container open;

    Node find_min();
    double dist(const Node &a, const Node &b) { return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2)); }

public:
    Heuristic() {}
    void init(unsigned int size, unsigned int agents);
    void count(const Map &map, Agent agent);
    void update(const Map &map, const Config &cfg, const std::vector<Node_info> &info, const INTERMOTION &inter_motion, const INNERMOTION &inner_motion);
    double custom_dist(const Node &a, const Node &b, const Map &map, const std::vector<Node_info> &info, const INTERMOTION &inter_motion, const INNERMOTION &inner_motion);
    double intersection_g(const Map &map, int curr, int next, int nextnext);
    unsigned int get_size() const { return h_values[0].size(); }
    double get_value(int id_node, int id_agent) { return h_values[id_node][id_agent]; }
    UVALUE get_uvalue() { return uh_values; }
    UVALUE *uvalue() { return &uh_values; }
};

#endif // HEURISTIC_H
