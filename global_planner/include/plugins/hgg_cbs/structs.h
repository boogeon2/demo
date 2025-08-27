#ifndef STRUCTS_H
#define STRUCTS_H

#include "math.h"
#include "vector"
#include "list"
#include "iostream"
#include "chrono"
#include "const.h"
#include "memory"
#include "set"
#include "iterator"
#include "boost/multi_index_container.hpp"
#include "boost/multi_index/ordered_index.hpp"
#include "boost/multi_index/hashed_index.hpp"
#include "boost/multi_index/member.hpp"
#include "boost/multi_index/composite_key.hpp"
#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"

using boost::multi_index_container;
using namespace boost::multi_index;

//////////////////////////////////////////////
// Structures that did not require changes //
//////////////////////////////////////////////

struct gNode
{
    int id;
    double i;
    double j;
    std::vector<int> neighbors;
    gNode(double i_ = -1, double j_ = -1) : i(i_), j(j_) {}
    ~gNode() { neighbors.clear(); }
};

struct Edge
{
    int id;
    int entry, exit;
    Edge(int id_ = -1, int entry_ = -1, int exit_ = -1) : id(id_), entry(entry_), exit(exit_) {}
};

struct timedPoint
{
    Eigen::Vector2d position;
    double t;
    timedPoint(double _x, double _y, double _t)
        : position(_x, _y), t(_t) {}
};

struct Obstacle
{
    int id;          // index of obstacle
    int ent, ext;    // corridor_ent_id, corridor_ext_id
    double ox, oy;   // center of the obstacle x,y location
    double ow, oh;   // obstacle width and height (assumed rectangular)
    bool registered; // whether this obstacle is registered in 'obstacle_info'
    Obstacle(int id_ = -1, int ent_ = -1, int ext_ = -1, double ox_ = -1.0, double oy_ = -1.0, double ow_ = -1.0, double oh_ = -1.0, bool registered_ = false)
        : id(id_), ent(ent_), ext(ext_), ox(ox_), oy(oy_), ow(ow_), oh(oh_), registered(registered_) {}
};

struct ObstacleInfo
{
    int id;               // obstacle id
    int entry, exit;      // entry and exit id of the edge
    int obs_type;         // 0: avoidable, 1: impassable, 2: obstacle exists in the opposite lane
    Eigen::Vector2d xy;   // obstacle location
    double width, length; // obstacle configuration
    std::vector<timedPoint> entry_traj, exit_traj;
    std::vector<timedPoint> avoid_traj;
    std::vector<timedPoint> uturn_traj;
    std::pair<double, double> avoid_interval, uturn_interval;
    ObstacleInfo(int id_ = -1, int entry_ = -1, int exit_ = -1, int obs_type_ = -1, double x_ = 0.0, double y_ = 0.0, double width_ = 0.0, double length_ = 0.0)
        : id(id_), entry(entry_), exit(exit_), obs_type(obs_type_), xy(x_, y_), width(width_), length(length_)
    {
        avoid_interval = std::make_pair(0.0, 0.0);
        uturn_interval = std::make_pair(0.0, 0.0);
    }
    void display_info()
    {
        std::cout << "[id entry exit obs_type]-->[" << id << ", " << entry << ", " << exit << ", " << obs_type << "]" << std::endl;
        std::cout << "entry_traj:\n";
        for (std::size_t i = 0; i < entry_traj.size(); ++i)
            std::cout << "[" << i << "] " << entry_traj[i].position(0) << ", " << entry_traj[i].position(1) << ", " << entry_traj[i].t << std::endl;
        std::cout << "avoid_traj:\n";
        for (std::size_t i = 0; i < avoid_traj.size(); ++i)
            std::cout << "[" << i << "] " << avoid_traj[i].position(0) << ", " << avoid_traj[i].position(1) << ", " << avoid_traj[i].t << std::endl;
        std::cout << "exit_traj:\n";
        for (std::size_t i = 0; i < exit_traj.size(); ++i)
            std::cout << "[" << i << "] " << exit_traj[i].position(0) << ", " << exit_traj[i].position(1) << ", " << exit_traj[i].t << std::endl;
        std::cout << "uturn_traj:\n";
        for (std::size_t i = 0; i < uturn_traj.size(); ++i)
            std::cout << "[" << i << "] " << uturn_traj[i].position(0) << ", " << uturn_traj[i].position(1) << ", " << uturn_traj[i].t << std::endl;
        std::cout << "avoid_interval: [" << avoid_interval.first << ", " << avoid_interval.second << "]" << std::endl;
    }
};

struct ObsEdgeInfo
{
    int id;                          // obstacle edge id
    int entry, exit;                 // entry and exit id of the edge
    int edge_type;                   // 0: normal, 1: lane1 blocked & unavoidable, 2: lane1 blocked & avoidable, etc.
    std::vector<Obstacle> obstacles; // obstacles on the edge

    std::pair<std::vector<timedPoint>, std::vector<timedPoint>> entry_traj, exit_traj;
    std::pair<std::vector<timedPoint>, std::vector<timedPoint>> avoid_traj;
    std::pair<std::vector<timedPoint>, std::vector<timedPoint>> uturn_traj;
    std::pair<std::pair<double, double>, std::pair<double, double>> avoid_interval, uturn_interval;
    ObsEdgeInfo(int id_ = -1, int entry_ = -1, int exit_ = -1, int edge_type_ = -1, std::vector<Obstacle> obstacles_ = std::vector<Obstacle>())
        : id(id_), entry(entry_), exit(exit_), edge_type(edge_type_), obstacles(obstacles_),
          entry_traj({{}, {}}), exit_traj({{}, {}}),
          avoid_traj({{}, {}}), uturn_traj({{}, {}}),
          avoid_interval({{0.0, 0.0}, {0.0, 0.0}}), uturn_interval({{0.0, 0.0}, {0.0, 0.0}})
    {
    }
};

struct Node
{
    int id, label;
    int type;        // -1: default, 0: station, 1: entrance, 2: exit, 3: avoidance start, 4: u-turn start, 5: holding point (edge)
    int orientation; // road id at the node (default: 0)
    double f, g, i, j;
    double offset = 0.0;
    Node *parent;
    int parent_id;
    std::pair<double, double> interval;
    int interval_id;
    int task_id = -1;
    int node_pd_type = -1; // 0: station, 1: pickup, 2: delivery
    bool new_assigned = false;
    Node(int _id = -1, double _f = -1, double _g = -1, double _i = -1, double _j = -1, Node *_parent = nullptr, double begin = -1, double end = -1)
        : id(_id), label(0), orientation(0), f(_f), g(_g), i(_i), j(_j), parent(_parent), parent_id(0),
          interval(std::make_pair(begin, end)), interval_id(0)
    {
    }
    bool operator<(const Node &other) const { return g < other.g; }
};

struct Agent
{
    double start_i, start_j, goal_i, goal_j;
    int start_id, goal_id;
    int id;
    double size;
    std::vector<std::tuple<int, int, int, int, std::pair<double, double>>> task_list; // (task id, pickup id, delivery id, status, interval)
    int initial_id = -1;
    int initial_orientation = -1;
    Node start_node;
    std::vector<Node> goal_nodes;
    bool new_goal_assigned = false;
    Agent(int s_id = -1, int g_id = -1, int _id = -1)
        : start_id(s_id), goal_id(g_id), id(_id)
    {
    }
};

struct Position
{
    double i, j, t;
    Position(double _i = -1, double _j = -1, double _t = -1)
        : i(_i), j(_j), t(_t)
    {
    }
    Position(const Node &node) : i(node.i), j(node.j), t(node.g) {}
};

struct Path
{
    std::vector<Node> nodes;
    double cost;
    int agentID;
    int expanded;
    Path(std::vector<Node> _nodes = std::vector<Node>(), double _cost = -1, int _agentID = -1)
        : nodes(_nodes), cost(_cost), agentID(_agentID), expanded(0)
    {
    }
};

//////////////////////////////////////////////
// 수정된 부분: sNode (초기화 순서 재정렬)      //
//////////////////////////////////////////////

struct sNode
{
    int id;
    int label;
    int orientation;
    int type; // -1: default, 0: station, 1: entrance, 2: exit, 3: avoidance start, 4: detour start, 5: between nodes (edge)
    double g;
    double i;
    double j; // i 값이 -1이면 edge 상에 있다는 의미
    double offset;

    sNode(int id_ = -1, int label_ = 0, double g_ = -1, double i_ = -1, double j_ = -1)
        : id(id_), label(label_), orientation(0), type(-1), g(g_), i(i_), j(j_), offset(0.0)
    {
    }

    sNode(const Node &n)
        : id(n.id), label(n.label), orientation(n.orientation), type(n.type), g(n.g), i(n.i), j(n.j), offset(n.offset)
    {
    }

    bool operator==(const sNode &other) const
    {
        return (id == other.id and type == other.type and orientation == other.orientation);
    }
};

//////////////////////////////////////////////
// 수정된 부분: sPath (복사 대입 연산자 수정)   //
//////////////////////////////////////////////

struct sPath
{
    std::vector<sNode> nodes;
    double cost;
    int agentID;
    int expanded;

    sPath(std::vector<sNode> _nodes = std::vector<sNode>(), double _cost = -1, int _agentID = -1)
        : nodes(_nodes), cost(_cost), agentID(_agentID), expanded(0)
    {
    }

    // 복사 생성자는 기본값 사용
    sPath(const sPath &other) = default;

    // Path 타입으로부터의 대입 연산자
    sPath &operator=(const Path &path)
    {
        cost = path.cost;
        agentID = path.agentID;
        expanded = path.expanded;
        nodes.clear();
        for (const auto &n : path.nodes)
            nodes.push_back(sNode(n));
        return *this;
    }

    // sPath 타입에 대한 대입 연산자
    sPath &operator=(const sPath &path)
    {
        if (this != &path)
        {
            cost = path.cost;
            agentID = path.agentID;
            expanded = path.expanded;
            nodes = path.nodes;
        }
        return *this;
    }
};

struct ReplanAgent
{
    int id;
    sNode start, goal;
    ReplanAgent(int _id = -1, sNode _s = sNode(), sNode _g = sNode())
        : id(_id), start(_s), goal(_g)
    {
    }
};

//////////////////////////////////////////////
// 수정된 부분: Constraint (초기화 및 미사용 제거) //
//////////////////////////////////////////////

struct Constraint
{
    int agent;
    double t1, t2; // 이동 시작 금지 시간 구간
    int id1, id2;
    int type1, type2;               // -1: default, 0: station, 1: entrance, 2: exit, 3: avoidance start, 4: detour start, 5: between nodes
    int orientation1, orientation2; // -1: default, 0: south, 1: west, 2: north, 3: east (시계방향)
    bool positive;

    Constraint(int _agent = -1, double _t1 = -1, double _t2 = -1, int _id1 = -1, int _id2 = -1, bool _positive = false)
        : agent(_agent), t1(_t1), t2(_t2), id1(_id1), id2(_id2),
          type1(-1), type2(-1), orientation1(-1), orientation2(-1),
          positive(_positive)
    {
    }

    Constraint(int _agent, int _id1, int _id2, int _type1, int _type2, int _ori1, int _ori2, double cons_t1, double cons_t2)
        : agent(_agent), t1(cons_t1), t2(cons_t2), id1(_id1), id2(_id2),
          type1(_type1), type2(_type2), orientation1(_ori1), orientation2(_ori2),
          positive(false)
    {
    }

    friend std::ostream &operator<<(std::ostream &os, const Constraint &con)
    {
        os << con.agent << " " << con.t1 << " " << con.t2 << " \n";
        return os;
    }

    bool operator<(const Constraint &other) const
    {
        return t1 < other.t1;
    }
};

//////////////////////////////////////////////
// 수정된 부분: Move (초기화 순서 재정렬 및 미사용 제거) //
//////////////////////////////////////////////

struct Move
{
    double t1, t2;
    double i1, j1, i2, j2;
    int id1, id2;
    int type1, type2;
    int orientation1, orientation2;
    double offset1, offset2;

    // 기본 생성자
    Move(double _t1 = -1, double _t2 = -1, int _id1 = -1, int _id2 = -1)
        : t1(_t1), t2(_t2),
          i1(-1.0), j1(-1.0), i2(-1.0), j2(-1.0),
          id1(_id1), id2(_id2),
          type1(-1), type2(-1),
          orientation1(-1), orientation2(-1),
          offset1(0.0), offset2(0.0)
    {
    }

    // Constraint로부터의 생성자
    Move(const Constraint &con)
        : t1(con.t1), t2(con.t2),
          i1(-1.0), j1(-1.0), i2(-1.0), j2(-1.0),
          id1(con.id1), id2(con.id2),
          type1(con.type1), type2(con.type2),
          orientation1(con.orientation1), orientation2(con.orientation2),
          offset1(0.0), offset2(0.0)
    {
    }

    // sNode 두 개로부터의 생성자 (초기화 순서를 멤버 선언 순서에 맞춤)
    Move(sNode a, sNode b)
        : t1(a.g), t2(b.g),
          i1(a.i), j1(a.j), i2(b.i), j2(b.j),
          id1(a.id), id2(b.id),
          type1(a.type), type2(b.type),
          orientation1(a.orientation), orientation2(b.orientation),
          offset1(a.offset), offset2(b.offset)
    {
    }

    bool operator<(const Move &other) const
    {
        if (id1 < other.id1)
            return true;
        else if (id1 > other.id1)
            return false;
        else if (id2 < other.id2)
            return true;
        else
            return false;
    }

    bool isEdge() const
    {
        return !(id1 == id2 and type1 == type2 and orientation1 == orientation2);
    }
};

struct Step
{
    int i;
    int j;
    int id;
    double cost;
    Step(int _i = 0, int _j = 0, int _id = 0, double _cost = -1.0)
        : i(_i), j(_j), id(_id), cost(_cost)
    {
    }
};

struct Conflict
{
    int agent1, agent2;
    double t;
    Move move1, move2;
    double overcost;
    Conflict(int _agent1 = -1, int _agent2 = -1, Move _move1 = Move(), Move _move2 = Move(), double _t = 1e9)
        : agent1(_agent1), agent2(_agent2), t(_t), move1(_move1), move2(_move2), overcost(0)
    {
    }
    bool operator<(const Conflict &other) const
    {
        return overcost < other.overcost;
    }
    int classify_type()
    {
        bool inter1 = (move1.type1 == 1 and move1.type2 == 2);
        bool inter2 = (move2.type1 == 1 and move2.type2 == 2);
        if (inter1 and inter2)
            return 0;
        bool edge1 = move1.isEdge();
        bool edge2 = move2.isEdge();
        if (edge1 and edge2)
            return 1;
        else if ((edge1 and !edge2) or (!edge1 and edge2))
            return 2;
        else if (!edge1 and !edge2)
            return 3;
        return 4;
    }
};

struct PairHash
{
    std::size_t operator()(const std::pair<int, int> &p) const
    {
        std::size_t h1 = std::hash<int>()(p.first);
        std::size_t h2 = std::hash<int>()(p.second);
        return h1 ^ (h2 << 1);
    }
};

struct TupleHash
{
    std::size_t operator()(const std::tuple<int, int, int> &t) const
    {
        std::size_t h1 = std::hash<int>()(std::get<0>(t));
        std::size_t h2 = std::hash<int>()(std::get<1>(t));
        std::size_t h3 = std::hash<int>()(std::get<2>(t));
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct IntersectionHash
{
    std::size_t operator()(const std::tuple<int, int, int, int, int> &t) const
    {
        std::size_t h1 = std::hash<int>()(std::get<0>(t));
        std::size_t h2 = std::hash<int>()(std::get<1>(t));
        std::size_t h3 = std::hash<int>()(std::get<2>(t));
        std::size_t h4 = std::hash<int>()(std::get<3>(t));
        std::size_t h5 = std::hash<int>()(std::get<4>(t));
        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3) ^ (h5 << 4);
    }
};

struct Road
{
    int id;
    double exit_angle;
    Eigen::Vector2d exit_point;
    Eigen::Vector2d entry_point;
    int connected_wp_id;
    Road(int _id, double _exit_angle, double _exit_x, double _exit_y, double _entry_x, double _entry_y, int _conn_wp)
        : id(_id), exit_angle(_exit_angle), exit_point(_exit_x, _exit_y), entry_point(_entry_x, _entry_y), connected_wp_id(_conn_wp)
    {
    }
};

struct Node_info
{
    int id, type;
    double entry_angle;
    std::vector<Road> roads;
};

struct Intermotion
{
    int s_id, e_id;
    double time_cost;
    std::vector<timedPoint> paths;
    Intermotion(int _s, int _e)
        : s_id(_s), e_id(_e)
    {
    }
};

struct Innermotion
{
    int i_id, s_id, e_id;
    double time_cost;
    std::vector<timedPoint> paths;
    Innermotion(int _i, int _s, int _e)
        : i_id(_i), s_id(_s), e_id(_e)
    {
    }
};

struct CBS_Node
{
    std::vector<sPath> paths;
    CBS_Node *parent;
    Constraint constraint;
    Constraint positive_constraint;
    int id;
    std::string id_str;
    double cost;
    double h;
    unsigned int conflicts_num;
    unsigned int total_cons;
    unsigned int low_level_expanded;
    std::list<Conflict> conflicts;
    std::list<Conflict> semicard_conflicts;
    std::list<Conflict> cardinal_conflicts;
    CBS_Node(std::vector<sPath> _paths = {}, CBS_Node *_parent = nullptr, Constraint _constraint = Constraint(), double _cost = 0, int _conflicts_num = 0, int total_cons_ = 0)
        : paths(_paths), parent(_parent), constraint(_constraint), cost(_cost), conflicts_num(_conflicts_num), total_cons(total_cons_)
    {
        low_level_expanded = 0;
        h = 0;
        conflicts.clear();
        semicard_conflicts.clear();
        cardinal_conflicts.clear();
    }
    ~CBS_Node()
    {
        parent = nullptr;
        paths.clear();
        conflicts.clear();
        semicard_conflicts.clear();
        cardinal_conflicts.clear();
    }
};

struct Open_Elem
{
    CBS_Node *tree_pointer;
    int id;
    double cost;
    unsigned int cons_num;
    unsigned int conflicts_num;

    Open_Elem(CBS_Node *_tree_pointer = nullptr, int _id = -1, double _cost = -1, unsigned int _cons_num = 0, unsigned int _conflicts_num = 0)
        : tree_pointer(_tree_pointer), id(_id), cost(_cost), cons_num(_cons_num), conflicts_num(_conflicts_num)
    {
    }
    ~Open_Elem()
    {
        tree_pointer = nullptr;
    }
};

struct cost
{
};
struct id
{
};

typedef multi_index_container<
    Open_Elem,
    indexed_by<
        ordered_non_unique<composite_key<Open_Elem,
                                         BOOST_MULTI_INDEX_MEMBER(Open_Elem, double, cost),
                                         BOOST_MULTI_INDEX_MEMBER(Open_Elem, unsigned int, conflicts_num),
                                         BOOST_MULTI_INDEX_MEMBER(Open_Elem, unsigned int, cons_num)>,
                           composite_key_compare<std::less<double>, std::less<int>, std::greater<int>>>,
        hashed_unique<tag<id>, BOOST_MULTI_INDEX_MEMBER(Open_Elem, int, id)>>>
    CT_container;

struct Focal_Elem
{
    int id;
    unsigned int conflicts_num;
    unsigned int constraints;
    double cost;
    Focal_Elem(int id_ = -1, unsigned int conflicts_num_ = 0, unsigned int constraints_ = 0, double cost_ = 0)
        : id(id_), conflicts_num(conflicts_num_), constraints(constraints_), cost(cost_)
    {
    }
    bool operator<(const Focal_Elem &other) const
    {
        if (conflicts_num < other.conflicts_num)
            return true;
        else if (conflicts_num > other.conflicts_num)
            return false;
        else if (constraints > other.constraints)
            return true;
        else if (constraints < other.constraints)
            return false;
        else if (cost < other.cost)
            return true;
        else
            return false;
    }
};

struct conflicts_num
{
};
struct constraints_num
{
};

typedef multi_index_container<
    Focal_Elem,
    indexed_by<
        ordered_non_unique<tag<conflicts_num>, identity<Focal_Elem>>,
        hashed_unique<tag<id>, BOOST_MULTI_INDEX_MEMBER(Focal_Elem, int, id)>>>
    Focal_container;

typedef multi_index_container<
    Node,
    indexed_by<
        ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, double, g)>,
        hashed_non_unique<BOOST_MULTI_INDEX_MEMBER(Node, int, id)>>>
    Open_List;

class CBS_Tree
{
    std::list<CBS_Node> tree;
    Focal_container focal;
    CT_container container;
    double focal_weight;
    int open_size;
    std::set<int> closed;

public:
    CBS_Tree()
    {
        open_size = 0;
        focal_weight = 1.0;
    }
    unsigned int get_size() { return tree.size(); }
    void clear()
    {
        tree.clear();
        focal.clear();
        container.clear();
        closed.clear();
    }
    void set_focal_weight(double weight) { focal_weight = weight; }
    int get_open_size() { return open_size; }
    void add_node(CBS_Node node)
    {
        tree.push_back(node);
        container.insert(Open_Elem(&tree.back(), node.id, node.cost, node.total_cons, node.conflicts_num));
        open_size++;
        if (focal_weight > 1.0)
            if (container.get<0>().begin()->cost * focal_weight > node.cost)
                focal.insert(Focal_Elem(node.id, node.conflicts_num, node.total_cons, node.cost));
    }
    CBS_Node *get_front()
    {
        open_size--;
        if (focal_weight > 1.0)
        {
            double cost = container.get<0>().begin()->cost;
            if (focal.empty())
                update_focal(cost);
            auto min = container.get<1>().find(focal.get<0>().begin()->id);
            focal.get<0>().erase(focal.get<0>().begin());
            auto pointer = min->tree_pointer;
            container.get<1>().erase(min);
            if (container.get<0>().begin()->cost > cost + 1e-9)
                update_focal(cost);
            return pointer;
        }
        else
        {
            auto pointer = container.get<0>().begin()->tree_pointer;
            container.get<0>().erase(container.get<0>().begin());
            return pointer;
        }
    }
    void update_focal(double cost)
    {
        auto it0 = container.get<0>().begin();
        auto it1 = container.get<0>().upper_bound(cost * focal_weight + 1e-9);
        for (auto it = it0; it != it1; ++it)
            focal.insert(Focal_Elem(it->id, it->conflicts_num, it->cons_num, it->cost));
    }
    std::vector<sPath> get_paths(CBS_Node node, int size)
    {
        std::vector<sPath> paths(size);
        while (node.parent != nullptr)
        {
            if (paths.at(node.paths.begin()->agentID).nodes.empty())
                paths.at(node.paths.begin()->agentID) = *node.paths.begin();
            node = *node.parent;
        }
        for (unsigned int i = 0; i < node.paths.size(); i++)
            if (paths.at(i).nodes.empty())
                paths.at(i) = node.paths.at(i);
        return paths;
    }
};

struct Solution
{
    bool found;
    double flowtime;
    double makespan;
    double check_time;
    double init_cost;
    int constraints_num;
    int max_constraints;
    int high_level_expanded;
    int high_level_generated;
    int low_level_expansions;
    double low_level_expanded;
    int cardinal_solved;
    int semicardinal_solved;
    std::chrono::duration<double> time;
    std::chrono::duration<double> init_time;
    std::vector<sPath> paths;
    Solution(double _flowtime = -1, double _makespan = -1, std::vector<sPath> _paths = {})
        : flowtime(_flowtime), makespan(_makespan), paths(_paths)
    {
        init_cost = -1;
        constraints_num = 0;
        low_level_expanded = 0;
        low_level_expansions = 0;
        cardinal_solved = 0;
        semicardinal_solved = 0;
        max_constraints = 0;
    }
    ~Solution()
    {
        paths.clear();
        found = false;
    }
};

class Vector2D
{
public:
    double i, j;
    Vector2D(double _i = 0.0, double _j = 0.0) : i(_i), j(_j) {}
    inline Vector2D operator+(const Vector2D &vec) { return Vector2D(i + vec.i, j + vec.j); }
    inline Vector2D operator-(const Vector2D &vec) { return Vector2D(i - vec.i, j - vec.j); }
    inline Vector2D operator-() { return Vector2D(-i, -j); }
    inline Vector2D operator/(const double &num) { return Vector2D(i / num, j / num); }
    inline Vector2D operator*(const double &num) { return Vector2D(i * num, j * num); }
    inline double operator*(const Vector2D &vec) { return i * vec.i + j * vec.j; }
    inline void operator+=(const Vector2D &vec)
    {
        i += vec.i;
        j += vec.j;
    }
    inline void operator-=(const Vector2D &vec)
    {
        i -= vec.i;
        j -= vec.j;
    }
};

class Point
{
public:
    double i, j;
    Point(double _i = 0.0, double _j = 0.0) : i(_i), j(_j) {}
    Point operator-(Point &p) { return Point(i - p.i, j - p.j); }
    int operator==(Point &p) { return (i == p.i) and (j == p.j); }
    int classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1; // LEFT;
        if (sa < 0.0)
            return 2; // RIGHT;
        if ((a.i * b.i < 0.0) or (a.j * b.j < 0.0))
            return 3; // BEHIND;
        if ((a.i * a.i + a.j * a.j) < (b.i * b.i + b.j * b.j))
            return 4; // BEYOND;
        if (pO == p2)
            return 5; // ORIGIN;
        if (p1 == p2)
            return 6; // DESTINATION;
        return 7;     // BETWEEN;
    }
};

#endif // STRUCTS_H
