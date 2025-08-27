#include "cbs.h"

bool CBS::update_root(const Map &map, PDTask &task)
{
    CBS_Node root;
    tree.set_focal_weight(config.focal_weight);
    sPath path;
    for (int i = 0; i < int(task.get_agents_size()); i++)
    {
        Agent agent = task.get_agent(i);
        path = planner.replan_path(agent, map, {}, h_values);
        if (path.cost < 0)
            return false;
        root.paths.push_back(path);
        agent.new_goal_assigned = false;
        for (auto &goal : agent.goal_nodes)
            goal.new_assigned = false;
        task.update_agent_info(i, agent);
        root.cost += path.cost;
    }
    std::cout << "initial cost for all paths is " << root.cost << std::endl;
    root.low_level_expanded = 0;
    root.parent = nullptr;
    root.id = 1;
    root.id_str = "1";
    auto conflicts = get_all_replan_conflicts(root.paths, -1);
    root.conflicts_num = conflicts.size();
    for (auto conflict : conflicts)
    {
        if (!config.use_cardinal)
            root.conflicts.push_back(conflict);
        else
        {
            auto pathA = planner.replan_path(task.get_agent(conflict.agent1), map, {get_replan_constraint(conflict.agent1, conflict.move1, conflict.move2)}, h_values);
            auto pathB = planner.replan_path(task.get_agent(conflict.agent2), map, {get_replan_constraint(conflict.agent2, conflict.move2, conflict.move1)}, h_values);
            if (pathA.cost > root.paths[conflict.agent1].cost and pathB.cost > root.paths[conflict.agent2].cost)
            {
                conflict.overcost = std::min(pathA.cost - root.paths[conflict.agent1].cost, pathB.cost - root.paths[conflict.agent2].cost);
                root.cardinal_conflicts.push_back(conflict);
            }
            else if (pathA.cost > root.paths[conflict.agent1].cost or pathB.cost > root.paths[conflict.agent2].cost)
                root.semicard_conflicts.push_back(conflict);
            else
                root.conflicts.push_back(conflict);
        }
    }
    solution.init_cost = root.cost;
    tree.add_node(root);
    return true;
}
bool CBS::replan_root(const Map &map, PDTask &task, const std::vector<sPath> &prev_paths, std::vector<double> progressed_time)
{
    CBS_Node root;
    tree.set_focal_weight(config.focal_weight);
    sPath path;
    for (int i = 0; i < int(task.get_agents_size()); i++)
    {
        Agent agent = task.get_agent(i);
        path = planner.replan_initial_path(i, task, map, h_values, prev_paths[i], progressed_time[i]);
        std::cout << "replan root of agent[" << agent.id << "] with path size-> " << path.nodes.size() << " and cost-> " << path.cost << std::endl;
        if (path.cost < 0)
            return false;
        root.paths.push_back(path);
        root.cost += path.cost;
        // agent.new_goal_assigned = false;
        // task.update_agent_info(i, agent);
    }
    root.low_level_expanded = 0;
    root.parent = nullptr;
    root.id = 1;
    root.id_str = "1";
    auto conflicts = get_all_replan_conflicts(root.paths, -1);
    root.conflicts_num = conflicts.size();
    for (auto conflict : conflicts)
    {
        if (!config.use_cardinal)
            root.conflicts.push_back(conflict);
        else
        {
            auto pathA = planner.replan_path(task.get_agent(conflict.agent1), map, {get_replan_constraint(conflict.agent1, conflict.move1, conflict.move2)}, h_values);
            auto pathB = planner.replan_path(task.get_agent(conflict.agent2), map, {get_replan_constraint(conflict.agent2, conflict.move2, conflict.move1)}, h_values);
            if (pathA.cost > root.paths[conflict.agent1].cost and pathB.cost > root.paths[conflict.agent2].cost)
            {
                conflict.overcost = std::min(pathA.cost - root.paths[conflict.agent1].cost, pathB.cost - root.paths[conflict.agent2].cost);
                root.cardinal_conflicts.push_back(conflict);
            }
            else if (pathA.cost > root.paths[conflict.agent1].cost or pathB.cost > root.paths[conflict.agent2].cost)
                root.semicard_conflicts.push_back(conflict);
            else
                root.conflicts.push_back(conflict);
        }
    }
    new_solution.init_cost = root.cost;
    std::cout << "replan root cost is " << root.cost << std::endl;
    tree.add_node(root);
    return true;
}

bool CBS::check_intersection_conflict(Move move1, Move move2)
{

    auto it = inner_collision_interval.find({move1.id1, move1.orientation1, move1.orientation2, move2.orientation1, move2.orientation2});
    if (it != inner_collision_interval.end())
    {
        auto intervals = inner_collision_interval.find({move1.id1, move1.orientation1, move1.orientation2, move2.orientation1, move2.orientation2})->second;
        double time_diff = move2.t1 - move1.t1;
        for (auto interval : intervals)
        {
            if (interval.first < time_diff and time_diff < interval.second)
                return true;
        }
    }
    else
    {
        int n_m11 = node_info[move1.id1].roads[move1.orientation1].connected_wp_id;
        int n_m12 = node_info[move1.id1].roads[move1.orientation2].connected_wp_id;
        int n_m21 = node_info[move2.id1].roads[move2.orientation1].connected_wp_id;
        int n_m22 = node_info[move2.id1].roads[move2.orientation2].connected_wp_id;
        std::cout << "m11:[" << n_m11 << "," << node_info[n_m11].type << "] and m12:[" << n_m12 << "," << node_info[n_m12].type << "]\n"
                  << "m21:[" << n_m21 << "," << node_info[n_m21].type << "] and m22:[" << n_m22 << "," << node_info[n_m22].type << "]" << std::endl;

        std::cout << "there is no inner collision interval between [(" << move1.id1 << "," << move1.type1 << "," << move1.orientation1 << "), (" << move1.id2 << "," << move1.type2 << "," << move1.orientation2 << ")] and ["
                  << "(" << move2.id1 << "," << move2.type1 << "," << move2.orientation1 << "), (" << move2.id2 << "," << move2.type2 << "," << move2.orientation2 << ")] check this value is valid" << std::endl;
    }
    return false;
}

bool CBS::check_cross_conflict(sNode a1, sNode a2, sNode b1, sNode b2)
{
    int next_a = node_info[a1.id].roads[a1.orientation].connected_wp_id;
    int next_b = node_info[b1.id].roads[b1.orientation].connected_wp_id;
    // int obs_id1 = -1, obs_id2 = -1;
    // auto obs_it1 = map->get_obstacle_info().find({a1.id, next_a});
    // if(obs_it1 != map->get_obstacle_info().end())
    //     obs_id1 = obs_it1->second.id;
    // auto obs_it2 = map->get_obstacle_info().find({b1.id, next_b});
    // if(obs_it2 != map->get_obstacle_info().end())
    //     obs_id2 = obs_it2->second.id;

    // if(obs_id1 == obs_id2 and obs_id1 != -1)    //share same obstacle
    if (next_a == b1.id and next_b == a1.id)
    {
        if (a2.g > b1.g + CN_EPSILON and b2.g > a1.g + CN_EPSILON)
            return true;
    }
    return false;
}

bool CBS::isSingleLane(sNode n)
{
    if (node_info[n.id].type == 0)
    {
        return true;
    }
    else if (node_info[n.id].roads[n.orientation].entry_point == node_info[n.id].roads[n.orientation].exit_point)
    {
        return true;
    }
    return false;
}
bool CBS::isSingleLane(Move m)
{
    if (node_info[m.id1].type == 0 or node_info[m.id2].type == 0)
    {
        return true;
    }
    else if (node_info[m.id2].roads[m.orientation2].entry_point == node_info[m.id2].roads[m.orientation2].exit_point) // we only check move id2 at the intersection move
    {
        return true;
    }
    return false;
}

double CBS::get_hl_heuristic(const std::list<Conflict> &conflicts)
{
    if (conflicts.empty() or config.hlh_type == 0)
        return 0;
    else if (config.hlh_type == 1)
    {
        std::cout << "Now hlh-type 1 is deprecated" << std::endl;
        return 0;
    }
    else
    {
        double h_value(0);
        std::vector<std::tuple<double, int, int>> values;
        values.reserve(conflicts.size());
        std::set<int> used;
        for (auto c : conflicts)
            values.push_back(std::make_tuple(c.overcost, c.agent1, c.agent2));
        std::sort(values.begin(), values.end(), std::greater<std::tuple<double, int, int>>());
        for (auto v : values)
        {
            if (used.find(get<1>(v)) != used.end() or used.find(get<2>(v)) != used.end())
                continue;
            h_value += get<0>(v);
            used.insert(get<1>(v));
            used.insert(get<2>(v));
        }
        return h_value;
    }
}

Constraint CBS::get_replan_constraint(int agent, Move move1, Move move2)
{
    double safe_time = config.safe_time + 1.5; // we increase safe_time

    bool interA = (move1.type1 == 1 and move1.type2 == 2);
    bool interB = (move2.type1 == 1 and move2.type2 == 2);

    bool crossA = (move1.type1 == 3 and move1.type2 == 5);
    bool crossB = (move2.type1 == 3 and move2.type2 == 5);

    if (interA and interB)
    {
        if (isSingleLane(move1) and isSingleLane(move2))
        {
            // return Constraint(agent, move1.id1, move1.id2, move1.type1, move1.type2, move1.orientation1, move1.orientation2, move1.t1, move2.t2 + 0.1);
            return Constraint(agent, move1.id1, move1.id2, move1.type1, move1.type2, move1.orientation1, move1.orientation2, move1.t1, move2.t2 + safe_time);
        }

        auto it = inner_collision_interval.find({move1.id1, move1.orientation1, move1.orientation2, move2.orientation1, move2.orientation2});
        if (it != inner_collision_interval.end())
        {
            auto intervals = inner_collision_interval.find({move1.id1, move1.orientation1, move1.orientation2, move2.orientation1, move2.orientation2})->second;
            double time_diff = move2.t1 - move1.t1;
            double hold_time = 0.0;
            for (const auto &interval : intervals)
            {
                if (interval.first < time_diff and time_diff < interval.second)
                {
                    double delta = time_diff - interval.first + safe_time;
                    if (delta > hold_time)
                        hold_time = delta;
                }
            }
            return Constraint(agent, move1.id1, move1.id2, move1.type1, move1.type2, move1.orientation1, move1.orientation2, move1.t1, move1.t1 + hold_time);
        }
        else
        {
            std::cout << "there is no inner collision interval between [" << move1.id1 << "," << move1.orientation1 << "," << move1.orientation2 << "] and ["
                      << move2.id1 << "," << move2.orientation1 << "," << move2.orientation2 << "] check this value is valid" << std::endl;
            return Constraint();
        }
    }
    else if (crossA and crossB and (move1.id1 != move2.id1))
    {
        return Constraint(agent, move1.id1, move1.id2, move1.type1, move1.type2, move1.orientation1, move1.orientation2, move1.t1, move2.t2 + 0.1);
    }
    else
    {
        bool edgeA = move1.isEdge();
        bool edgeB = move2.isEdge();
        double hold_time = 0.0;

        if (edgeA and !edgeB) // Edge-Vertex Constraint
        {
            if (move2.id1 == move1.id1 and move2.type1 == move1.type1 and move2.orientation1 == move1.orientation1)
                hold_time = move2.t2 - move1.t1 + safe_time; // hold_time = move2.t2 - move1.t1+0.1;
            else if (move2.id1 == move1.id2 and move2.type1 == move1.type2 and move2.orientation1 == move1.orientation2)
                hold_time = move2.t2 + safe_time - move1.t2;
            else
                hold_time = move2.t2 - move1.t1 + safe_time; // hold_time = move2.t2 - move1.t1+0.1;
            return Constraint(agent, move1.id1, move1.id2, move1.type1, move1.type2, move1.orientation1, move1.orientation2, move1.t1, move1.t1 + hold_time);
        }
        else if (!edgeA and edgeB) // Vertex-Edge Constraint
        {
            if (move1.id1 == move2.id1 and move1.type1 == move2.type1 and move1.orientation1 == move2.orientation1)
                hold_time = move2.t1 + safe_time - move1.t1;
            else if (move1.id1 == move2.id2 and move1.type1 == move2.type2 and move1.orientation1 == move2.orientation2)
                hold_time = move2.t2 - move1.t1 + safe_time; // hold_time = move2.t2 - move1.t1+0.1;
            else
                hold_time = move2.t2 - move1.t1 + safe_time; // hold_time = move2.t2 - move1.t1+0.1;
            return Constraint(agent, move1.id1, move1.id2, move1.type1, move1.type2, move1.orientation1, move1.orientation2, move1.t1, move1.t1 + hold_time);
        }
        else if (edgeA and edgeB) // Edge-Edge Constraint
        {
            if (move1.id1 == move2.id1 and move1.type1 == move2.type1 and move1.orientation1 == move2.orientation1 and move1.id2 == move2.id2 and move1.type2 == move2.type2 and move1.orientation2 == move2.orientation2) // same edge
            {
                return Constraint(agent, move1.id1, move1.id2, move1.type1, move1.type2, move1.orientation1, move1.orientation2, move1.t1, move2.t1 + safe_time);
            }
            else if (move1.id2 == move2.id1 and move1.type2 == move2.type1 and move1.orientation2 == move2.orientation1) // target - source
            {
                hold_time = move2.t1 + safe_time - move1.t2;
                return Constraint(agent, move1.id1, move1.id2, move1.type1, move1.type2, move1.orientation1, move1.orientation2, move1.t1, move1.t1 + hold_time);
            }
            else if (move1.id1 == move2.id2 and move1.type1 == move2.type2 and move1.orientation1 == move2.orientation2) // source - target
            {
                hold_time = move2.t2 + safe_time - move1.t1; // Incorrect value(24.01.15 jw)
                // hold_time = move2.t2 - move1.t1+0.1;    //CHECK THIS VALUE IS CORRECT (24.01.12 jw) --> even if move1.type1 == 0(swapping conflict), the same hold time is required
                return Constraint(agent, move1.id1, move1.id2, move1.type1, move1.type2, move1.orientation1, move1.orientation2, move1.t1, move1.t1 + hold_time);
            }
        }
    }
}

Conflict CBS::get_conflict(std::list<Conflict> &conflicts)
{
    auto best_it = conflicts.begin();
    for (auto it = conflicts.begin(); it != conflicts.end(); it++)
    {
        if (it->overcost > 0)
        {
            if (best_it->overcost < it->overcost or (fabs(best_it->overcost - it->overcost) < CN_EPSILON and best_it->t < it->t))
                best_it = it;
        }
        else if (best_it->t < it->t)
            best_it = it;
    }

    Conflict conflict = *best_it;
    conflicts.erase(best_it);
    return conflict;
}

void CBS::parse_node_info(const YAML::Node &info)
{
    int num_info = info["intersections"].size() + info["endpoints"].size();
    if (num_info != map->get_node_size())
        throw std::runtime_error("Graph node size is not equal to node information size");
    node_info.resize(num_info);

    const YAML::Node i_info = info["intersections"];
    for (const auto &i : i_info)
    {
        int idx = i["id"].as<int>();
        node_info[idx].id = idx;
        node_info[idx].type = 1;
        for (const auto &r : i["roads"])
        {
            Road ro(r["id"].as<int>(), r["exit_angle"].as<double>(), r["exit_point"][0].as<double>(), r["exit_point"][1].as<double>(), r["entry_point"][0].as<double>(), r["entry_point"][1].as<double>(), r["connected_waypoint"].as<int>());
            node_info[idx].roads.push_back(ro);
        }
    }

    const YAML::Node e_info = info["endpoints"];
    for (const auto &e : e_info)
    {
        int idx = e["id"].as<int>();
        node_info[idx].id = idx;
        node_info[idx].type = 0;
        node_info[idx].entry_angle = e["entry_angle"].as<double>();
    }
    std::cout << "successfully parsing node info" << std::endl;
}

void CBS::parse_guide_motion(const YAML::Node &guide)
{
    auto inter_guide = guide["inter_motion_paths"];
    for (const auto &t : inter_guide)
    {
        Intermotion int_m(t["start_waypoint"].as<int>(), t["end_waypoint"].as<int>());

        const YAML::Node path = t["paths"];
        for (const auto &p : path)
        {
            int_m.paths.emplace_back(p[0].as<double>(), p[1].as<double>(), p[2].as<double>());
        }
        int_m.time_cost = int_m.paths.back().t;
        inter_motion.insert({{std::make_pair(int_m.s_id, int_m.e_id)}, {int_m}});
    }

    auto inner_guide = guide["inner_motion_paths"];
    for (const auto &n : inner_guide)
    {
        Innermotion inn_m(n["waypoint"].as<int>(), n["start_road"].as<int>(), n["end_road"].as<int>());

        const YAML::Node path = n["paths"];
        for (const auto &p : path)
        {
            inn_m.paths.emplace_back(p[0].as<double>(), p[1].as<double>(), p[2].as<double>());
        }
        inn_m.time_cost = inn_m.paths.back().t;
        inner_motion.insert({{std::make_tuple(inn_m.i_id, inn_m.s_id, inn_m.e_id)}, {inn_m}});
    }

    std::cout << "successfully parsing guide motion" << std::endl;
}

void CBS::summarize_conflicts(int num, const std::list<Conflict> &conflicts, const std::list<Conflict> &semi, const std::list<Conflict> &cardi)
{
    std::vector<int> num_conflicts(num);
    std::vector<int> type_conflicts(5);
    std::unordered_map<std::pair<int, int>, int, PairHash> conflict_loc;

    for (auto c : conflicts)
    {
        num_conflicts[c.agent1] += 1;
        num_conflicts[c.agent2] += 1;
        type_conflicts[c.classify_type()] += 1;
        if (!conflict_loc.empty())
        {
            auto iter = conflict_loc.find({c.move1.id1, c.move1.id2});
            if (iter == conflict_loc.end())
            {
                conflict_loc.insert({{c.move1.id1, c.move1.id2}, 1});
            }
            else
            {
                iter->second += 1;
            }
        }
        else
        {
            conflict_loc.insert({{c.move1.id1, c.move1.id2}, 1});
        }
    }
    for (auto s : semi)
    {
        num_conflicts[s.agent1] += 1;
        num_conflicts[s.agent2] += 1;
        type_conflicts[s.classify_type()] += 1;
        if (!conflict_loc.empty())
        {
            auto iter = conflict_loc.find({s.move1.id1, s.move1.id2});
            if (iter == conflict_loc.end())
            {
                conflict_loc.insert({{s.move1.id1, s.move1.id2}, 1});
            }
            else
            {
                iter->second += 1;
            }
        }
        else
        {
            conflict_loc.insert({{s.move1.id1, s.move1.id2}, 1});
        }
    }
    for (auto ca : cardi)
    {
        num_conflicts[ca.agent1] += 1;
        num_conflicts[ca.agent2] += 1;
        type_conflicts[ca.classify_type()] += 1;
        if (!conflict_loc.empty())
        {
            auto iter = conflict_loc.find({ca.move1.id1, ca.move1.id2});
            if (iter == conflict_loc.end())
            {
                conflict_loc.insert({{ca.move1.id1, ca.move1.id2}, 1});
            }
            else
            {
                iter->second += 1;
            }
        }
        else
        {
            conflict_loc.insert({{ca.move1.id1, ca.move1.id2}, 1});
        }
    }
    std::cout << "conflicts for each agent: [";
    for (auto nc : num_conflicts)
        std::cout << nc << ", ";
    std::cout << "]";

    std::cout << " and type: [";
    for (auto tc : type_conflicts)
        std::cout << tc << ", ";
    std::cout << "]" << std::endl;
    std::cout << "conflict loc: \n";
    for (auto loc : conflict_loc)
        std::cout << "[" << loc.first.first << ", " << loc.first.second << ", " << loc.second << "]" << std::endl;
}

Solution CBS::update_solution(const Map &map, PDTask &task, const Config &cfg, int capacity)
{
    std::cout << "Updating solution..." << std::endl;
    config = cfg;
    this->map = &map;
    h_values.init(map.get_size(), task.get_agents_size());
    auto h_t1 = std::chrono::high_resolution_clock::now();
    int cardinal_solved = 0, semicardinal_solved = 0;
    // for (int i = 0; i < int(task.get_agents_size()); i++)
    // {
    //     Agent agent = task.get_agent(i);
    //     h_values.count(map, agent);
    // }
    // auto h_t2 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - h_t1);
    // std::cout << "Time to get h_val: " << h_t2.count() << std::endl;

    auto uh_t1 = std::chrono::high_resolution_clock::now();
    h_values.update(map, cfg, node_info, inter_motion, inner_motion);
    auto uh_t2 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - uh_t1);
    std::cout << "Time to get uh_val: " << uh_t2.count() << std::endl;

    planner.init_predefined_info(node_info, inter_motion, inner_motion, h_values.get_uvalue());
    task.get_h_val(h_values.get_uvalue());

    std::vector<int> empty_labels;
    std::vector<double> time_remaining(task.get_agents_size(), 0.0);
    task.online_task_assigner(node_info, empty_labels, capacity, time_remaining);

    auto t = std::chrono::high_resolution_clock::now();
    if (!this->update_root(map, task))
        return solution;
    solution.init_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    std::cout << "init_time: " << solution.init_time.count() << std::endl;
    solution.found = true;
    CBS_Node node;
    std::chrono::duration<double> time_spent;
    int expanded(1);
    double time(0);
    std::list<Conflict> conflicts;
    Conflict conflict;
    int low_level_searches(0);
    int low_level_expanded(0);
    int id = 2;
    do
    {
        auto parent = tree.get_front();
        node = *parent;
        node.cost -= node.h;
        parent->conflicts.clear();
        parent->cardinal_conflicts.clear();
        parent->semicard_conflicts.clear();
        auto paths = get_paths(&node, task.get_agents_size());

        auto time_now = std::chrono::high_resolution_clock::now();
        conflicts = node.conflicts;
        auto cardinal_conflicts = node.cardinal_conflicts;
        auto semicard_conflicts = node.semicard_conflicts;
        if (conflicts.empty() and semicard_conflicts.empty() and cardinal_conflicts.empty())
        {
            break;
        }
        if (!cardinal_conflicts.empty())
        {
            conflict = get_conflict(cardinal_conflicts);
            cardinal_solved++;
        }
        else if (!semicard_conflicts.empty())
        {
            conflict = get_conflict(semicard_conflicts);
            semicardinal_solved++;
        }
        else
            conflict = get_conflict(conflicts);
        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
        time += time_spent.count();
        expanded++;

        std::list<Constraint> constraintsA = get_constraints(&node, conflict.agent1);

        Constraint constraintA(get_replan_constraint(conflict.agent1, conflict.move1, conflict.move2));
        constraintsA.push_back(constraintA);
        sPath pathA;
        pathA = planner.replan_path(task.get_agent(conflict.agent1), map, constraintsA, h_values);
        low_level_searches++;
        low_level_expanded += pathA.expanded;

        std::list<Constraint> constraintsB = get_constraints(&node, conflict.agent2);

        Constraint constraintB(get_replan_constraint(conflict.agent2, conflict.move2, conflict.move1));
        constraintsB.push_back(constraintB);
        sPath pathB;
        pathB = planner.replan_path(task.get_agent(conflict.agent2), map, constraintsB, h_values);
        low_level_searches++;
        low_level_expanded += pathB.expanded;

        CBS_Node right({pathA}, parent, constraintA, node.cost + pathA.cost - get_cost(node, conflict.agent1), 0, node.total_cons + 1);
        CBS_Node left({pathB}, parent, constraintB, node.cost + pathB.cost - get_cost(node, conflict.agent2), 0, node.total_cons + 1);
        Constraint positive;
        bool inserted = false;

        right.id_str = node.id_str + "0";
        left.id_str = node.id_str + "1";
        right.id = id++;
        left.id = id++;
        if (pathA.cost > 0)
        {
            time_now = std::chrono::high_resolution_clock::now();
            replan_new_conflicts(map, task, right, paths, pathA, conflicts, semicard_conflicts, cardinal_conflicts, low_level_searches, low_level_expanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if (right.cost > 0)
            {
                right.h = get_hl_heuristic(right.cardinal_conflicts);
                right.cost += right.h;
                tree.add_node(right);
            }
        }
        if (pathB.cost > 0)
        {
            time_now = std::chrono::high_resolution_clock::now();
            replan_new_conflicts(map, task, left, paths, pathB, conflicts, semicard_conflicts, cardinal_conflicts, low_level_searches, low_level_expanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if (left.cost > 0)
            {
                left.h = get_hl_heuristic(left.cardinal_conflicts);
                left.cost += left.h;
                tree.add_node(left);
            }
        }
        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
        if (time_spent.count() > config.timelimit)
        {
            solution.found = false;
            break;
        }
    } while (tree.get_open_size() > 0);
    solution.paths = get_paths(&node, task.get_agents_size());
    solution.flowtime = node.cost;
    solution.low_level_expansions = low_level_searches;
    solution.low_level_expanded = double(low_level_expanded) / std::max(low_level_searches, 1);
    solution.high_level_expanded = expanded;
    solution.high_level_generated = int(tree.get_size());
    for (auto path : solution.paths)
        solution.makespan = (solution.makespan > path.cost) ? solution.makespan : path.cost;
    solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.check_time = time;
    solution.cardinal_solved = cardinal_solved;
    solution.semicardinal_solved = semicardinal_solved;

    return solution;
}

Solution CBS::replan_solution(const Map &map, PDTask &task, const Config &cfg, const std::vector<sPath> &prev_paths, std::vector<double> progressed_time)
{
    std::cout << "replanning starts" << std::endl;
    tree = CBS_Tree();
    new_solution = Solution();
    int cardinal_solved = 0, semicardinal_solved = 0;

    // std::cout<<"update h_values"<<std::endl;
    // h_values.update(map, cfg, node_info, inter_motion, inner_motion);
    // planner.update_heuristic_info(h_values.get_uvalue());

    auto t = std::chrono::high_resolution_clock::now();
    if (!this->replan_root(map, task, prev_paths, progressed_time))
        return new_solution;
    new_solution.init_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    new_solution.found = true;
    CBS_Node node;
    std::chrono::duration<double> time_spent;
    int expanded(1);
    double time(0);
    std::list<Conflict> conflicts;
    Conflict conflict;
    int low_level_searches(0);
    int low_level_expanded(0);
    int id = 2;
    do
    {
        auto parent = tree.get_front();
        node = *parent;
        node.cost -= node.h;
        parent->conflicts.clear();
        parent->cardinal_conflicts.clear();
        parent->semicard_conflicts.clear();
        auto paths = get_paths(&node, task.get_agents_size());

        auto time_now = std::chrono::high_resolution_clock::now();
        conflicts = node.conflicts;
        auto cardinal_conflicts = node.cardinal_conflicts;
        auto semicard_conflicts = node.semicard_conflicts;

        if (conflicts.empty() and semicard_conflicts.empty() and cardinal_conflicts.empty())
        {
            break;
        }
        if (!cardinal_conflicts.empty())
        {
            conflict = get_conflict(cardinal_conflicts);
            cardinal_solved++;
        }
        else if (!semicard_conflicts.empty())
        {
            conflict = get_conflict(semicard_conflicts);
            semicardinal_solved++;
        }
        else
            conflict = get_conflict(conflicts);
        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
        time += time_spent.count();
        expanded++;

        std::list<Constraint> constraintsA = get_constraints(&node, conflict.agent1);
        Constraint constraintA(get_replan_constraint(conflict.agent1, conflict.move1, conflict.move2));
        constraintsA.push_back(constraintA);

        sPath pathA;
        pathA = planner.replan_path(task.get_agent(conflict.agent1), map, constraintsA, h_values);
        if (pathA.cost < 0)
        {
            std::cout << "we fail to find pathA(agent:" << conflict.agent1 << ")" << std::endl;
            new_solution.found = false;
            break;
        }
        low_level_searches++;
        low_level_expanded += pathA.expanded;

        std::list<Constraint> constraintsB = get_constraints(&node, conflict.agent2);
        Constraint constraintB = get_replan_constraint(conflict.agent2, conflict.move2, conflict.move1);
        constraintsB.push_back(constraintB);

        sPath pathB;
        pathB = planner.replan_path(task.get_agent(conflict.agent2), map, constraintsB, h_values);
        if (pathB.cost < 0)
        {
            std::cout << "we fail to find pathB(agent:" << conflict.agent2 << ")" << std::endl;
            new_solution.found = false;
            break;
        }

        low_level_searches++;
        low_level_expanded += pathB.expanded;

        CBS_Node right({pathA}, parent, constraintA, node.cost + pathA.cost - get_cost(node, conflict.agent1), 0, node.total_cons + 1);
        CBS_Node left({pathB}, parent, constraintB, node.cost + pathB.cost - get_cost(node, conflict.agent2), 0, node.total_cons + 1);
        Constraint positive;
        bool inserted = false;

        right.id_str = node.id_str + "0";
        left.id_str = node.id_str + "1";
        right.id = id++;
        left.id = id++;
        if (pathA.cost > 0)
        {
            time_now = std::chrono::high_resolution_clock::now();
            replan_new_conflicts(map, task, right, paths, pathA, conflicts, semicard_conflicts, cardinal_conflicts, low_level_searches, low_level_expanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if (right.cost > 0)
            {
                right.h = get_hl_heuristic(right.cardinal_conflicts);
                right.cost += right.h;
                tree.add_node(right);
            }
        }
        if (pathB.cost > 0)
        {
            time_now = std::chrono::high_resolution_clock::now();
            replan_new_conflicts(map, task, left, paths, pathB, conflicts, semicard_conflicts, cardinal_conflicts, low_level_searches, low_level_expanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if (left.cost > 0)
            {
                left.h = get_hl_heuristic(left.cardinal_conflicts);
                left.cost += left.h;
                tree.add_node(left);
            }
        }
        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
        if (time_spent.count() > config.timelimit)
        {
            new_solution.found = false;
            break;
        }
    } while (tree.get_open_size() > 0);

    new_solution.paths = get_paths(&node, task.get_agents_size());
    new_solution.flowtime = node.cost;
    new_solution.low_level_expansions = low_level_searches;
    new_solution.low_level_expanded = double(low_level_expanded) / std::max(low_level_searches, 1);
    new_solution.high_level_expanded = expanded;
    new_solution.high_level_generated = int(tree.get_size());
    for (auto path : new_solution.paths)
        new_solution.makespan = (new_solution.makespan > path.cost) ? new_solution.makespan : path.cost;
    new_solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    new_solution.check_time = time;
    new_solution.cardinal_solved = cardinal_solved;
    new_solution.semicardinal_solved = semicardinal_solved;

    if (new_solution.found)
    {
        solution = Solution();
        solution = new_solution;
        return new_solution;
    }
    std::cout << "We failed replanning solution." << std::endl;
    solution.found = false;
    return solution;
}

void CBS::replan_new_conflicts(const Map &map, const PDTask &task, CBS_Node &node, std::vector<sPath> &paths, const sPath &path,
                               const std::list<Conflict> &conflicts, const std::list<Conflict> &semicard_conflicts, const std::list<Conflict> &cardinal_conflicts,
                               int &low_level_searches, int &low_level_expanded)
{
    auto oldpath = paths[path.agentID];
    paths[path.agentID] = path;
    auto new_conflicts = get_all_replan_conflicts(paths, path.agentID);
    paths[path.agentID] = oldpath;
    std::list<Conflict> conflictsA({}), semicard_conflictsA({}), cardinal_conflictsA({});
    for (auto c : conflicts)
        if (c.agent1 != path.agentID and c.agent2 != path.agentID)
            conflictsA.push_back(c);
    for (auto c : semicard_conflicts)
        if (c.agent1 != path.agentID and c.agent2 != path.agentID)
            semicard_conflictsA.push_back(c);
    for (auto c : cardinal_conflicts)
        if (c.agent1 != path.agentID and c.agent2 != path.agentID)
            cardinal_conflictsA.push_back(c);
    if (!config.use_cardinal)
    {
        node.conflicts = conflictsA;
        for (auto n : new_conflicts)
            node.conflicts.push_back(n);
        node.cardinal_conflicts.clear();
        node.semicard_conflicts.clear();
        node.conflicts_num = node.conflicts.size();
        return;
    }
    for (auto c : new_conflicts)
    {
        std::list<Constraint> constraintsA, constraintsB;
        if (path.agentID == c.agent1)
        {
            constraintsA = get_constraints(&node, c.agent1);
            constraintsA.push_back(get_replan_constraint(c.agent1, c.move1, c.move2));
            auto new_pathA = planner.replan_path(task.get_agent(c.agent1), map, constraintsA, h_values);
            constraintsB = get_constraints(&node, c.agent2);
            constraintsB.push_back(get_replan_constraint(c.agent2, c.move2, c.move1));
            auto new_pathB = planner.replan_path(task.get_agent(c.agent2), map, constraintsB, h_values);
            double old_cost = get_cost(node, c.agent2);

            if (new_pathA.cost < 0 and new_pathB.cost < 0)
            {
                node.cost = -1;
                return;
            }
            else if (new_pathA.cost < 0)
            {
                c.overcost = new_pathB.cost - old_cost;
                cardinal_conflictsA.push_back(c);
            }
            else if (new_pathB.cost < 0)
            {
                c.overcost = new_pathA.cost - path.cost;
                cardinal_conflictsA.push_back(c);
            }
            else if (new_pathA.cost > path.cost and new_pathB.cost > old_cost)
            {
                c.overcost = std::min(new_pathA.cost - path.cost, new_pathB.cost - old_cost);
                cardinal_conflictsA.push_back(c);
            }
            else if (new_pathA.cost > path.cost or new_pathB.cost > old_cost)
                semicard_conflictsA.push_back(c);
            else
                conflictsA.push_back(c);
            low_level_searches += 2;
            low_level_expanded += (new_pathA.expanded + new_pathB.expanded);
        }
        else
        {
            constraintsA = get_constraints(&node, c.agent2);
            constraintsA.push_back(get_replan_constraint(c.agent2, c.move2, c.move1));
            auto new_pathA = planner.replan_path(task.get_agent(c.agent2), map, constraintsA, h_values);
            constraintsB = get_constraints(&node, c.agent1);
            constraintsB.push_back(get_replan_constraint(c.agent1, c.move1, c.move2));
            auto new_pathB = planner.replan_path(task.get_agent(c.agent1), map, constraintsB, h_values);
            double old_cost = get_cost(node, c.agent1);

            if (new_pathA.cost < 0 and new_pathB.cost < 0)
            {
                node.cost = -1;
                return;
            }
            else if (new_pathA.cost < 0)
            {
                c.overcost = new_pathB.cost - old_cost;
                cardinal_conflictsA.push_back(c);
            }
            else if (new_pathB.cost < 0)
            {
                c.overcost = new_pathA.cost - path.cost;
                cardinal_conflictsA.push_back(c);
            }
            else if (new_pathA.cost > path.cost and new_pathB.cost > old_cost)
            {
                c.overcost = std::min(new_pathA.cost - path.cost, new_pathB.cost - old_cost);
                cardinal_conflictsA.push_back(c);
            }
            else if (new_pathA.cost > path.cost or new_pathB.cost > old_cost)
                semicard_conflictsA.push_back(c);
            else
                conflictsA.push_back(c);
            low_level_searches += 2;
            low_level_expanded += (new_pathA.expanded + new_pathB.expanded);
        }
    }

    node.conflicts = conflictsA;
    node.semicard_conflicts = semicard_conflictsA;
    node.cardinal_conflicts = cardinal_conflictsA;
    node.conflicts_num = conflictsA.size() + semicard_conflictsA.size() + cardinal_conflictsA.size();
    return;
}

std::list<Constraint> CBS::get_constraints(CBS_Node *node, int agent_id)
{
    CBS_Node *curNode = node;
    std::list<Constraint> constraints(0);
    while (curNode->parent != nullptr)
    {
        if (agent_id < 0 or curNode->constraint.agent == agent_id)
            constraints.push_back(curNode->constraint);
        if (curNode->positive_constraint.agent == agent_id)
            constraints.push_back(curNode->positive_constraint);
        curNode = curNode->parent;
    }
    return constraints;
}

Conflict CBS::check_replan_paths(const sPath &pathA, const sPath &pathB)
{
    // In corridor, two robots can't conflict unless they share the same node or edge. (You don't need to calculate the distance to check conflict)
    // So, Move(a, a+1) collide only when (a.id) or (a+1.id) are equal to (b.id) or (b+1.id) except for moves at intersection or invading the opposite lane
    // In intersection, we can use ctable and check pathB
    unsigned int a(0), b(0);
    auto nodesA = pathA.nodes;
    auto nodesB = pathB.nodes;

    double safe_time = config.safe_time;

    bool inSingleLaneA = false, inSingleLaneB = false;
    std::vector<std::tuple<int, sNode, double, double>> singleA, singleB; // entry_start_path_index, node, entry_time, exit_time
    for (std::size_t i = 0; i < nodesA.size() - 1; ++i)
    {
        if (inSingleLaneA and nodesA[i].type == 1 and !(nodesA[i] == nodesA[i + 1]))
        {
            if (std::get<1>(singleA.back()).id == nodesA[i].id and std::get<3>(singleA.back()) < 0)
            {
                inSingleLaneA = false;
                std::get<3>(singleA.back()) = nodesA[i].g;
            }
        }
        if (nodesA[i + 1].type == 2 and !(nodesA[i] == nodesA[i + 1]) and isSingleLane(nodesA[i + 1]))
        {
            // if(nodesA[i+1].label+1 < nodesA.back().label)    //240415 bug fix
            {
                inSingleLaneA = true;
                singleA.push_back({i + 1, nodesA[i + 1], nodesA[i + 1].g, -1});
            }
        }
    }
    for (std::size_t i = 0; i < nodesB.size() - 1; ++i)
    {
        if (inSingleLaneB and nodesB[i].type == 1 and !(nodesB[i] == nodesB[i + 1]))
        {
            if (std::get<1>(singleB.back()).id == nodesB[i].id and std::get<3>(singleB.back()) < 0)
            {
                inSingleLaneB = false;
                std::get<3>(singleB.back()) = nodesB[i].g;
            }
        }
        if (nodesB[i + 1].type == 2 and !(nodesB[i] == nodesB[i + 1]) and isSingleLane(nodesB[i + 1]))
        {
            // if(nodesB[i+1].label+1 < nodesB.back().label)
            {
                inSingleLaneB = true;
                singleB.push_back({i + 1, nodesB[i + 1], nodesB[i + 1].g, -1});
            }
        }
    }

    while (a < nodesA.size() - 1 or b < nodesB.size() - 1)
    {
        if (nodesA[a].g >= 0 and nodesB[b].g >= 0)
        {
            bool crossA = (nodesA[a].type == 3 and nodesA[a + 1].type == 5);
            bool crossB = (nodesB[b].type == 3 and nodesB[b + 1].type == 5);

            bool interA = (nodesA[a].type == 1 and nodesA[a + 1].type == 2);
            bool interB = (nodesB[b].type == 1 and nodesB[b + 1].type == 2);

            for (auto &sa : singleA)
            {
                if (std::get<0>(sa) == a + 1)
                {
                    for (auto &sb : singleB)
                    {
                        auto na = std::get<1>(sb);
                        auto nb = std::get<1>(sa);
                        if (na == nb or (node_info[na.id].roads[na.orientation].connected_wp_id == nb.id and node_info[nb.id].roads[nb.orientation].connected_wp_id == na.id))
                        {
                            if (std::get<2>(sa) <= std::get<3>(sb) and std::get<2>(sb) <= std::get<3>(sa))
                            {
                                Move move1(nodesA[a], nodesA[a + 1]);
                                move1.t2 = std::get<3>(sa);
                                Move move2(nodesB[std::get<0>(sb) - 1], nodesB[std::get<0>(sb)]);
                                move2.t2 = std::get<3>(sb);
                                return Conflict(pathA.agentID, pathB.agentID, move1, move2);
                            }
                        }
                    }
                }
            }
            for (auto &sb : singleB)
            {
                if (std::get<0>(sb) == b + 1)
                {
                    for (auto &sa : singleA)
                    {
                        auto na = std::get<1>(sb);
                        auto nb = std::get<1>(sa);
                        // if(std::get<1>(sb) == std::get<1>(sa))
                        if (na == nb or (node_info[na.id].roads[na.orientation].connected_wp_id == nb.id and node_info[nb.id].roads[nb.orientation].connected_wp_id == na.id))
                        {
                            if (std::get<2>(sa) <= std::get<3>(sb) and std::get<2>(sb) <= std::get<3>(sa))
                            {
                                Move move1(nodesA[std::get<0>(sa) - 1], nodesA[std::get<0>(sa)]);
                                move1.t2 = std::get<3>(sa);
                                Move move2(nodesB[b], nodesB[b + 1]);
                                move2.t2 = std::get<3>(sb);
                                return Conflict(pathA.agentID, pathB.agentID, move1, move2);
                            }
                        }
                    }
                }
            }
            if (interA and interB)
            {
                if (nodesA[a].id == nodesB[b].id)
                    if (check_intersection_conflict(Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1])))
                        return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
            }
            else if (crossA and crossB and !(nodesA[a] == nodesB[b]))
            {
                if (check_cross_conflict(nodesA[a], nodesA[a + 1], nodesB[b], nodesB[b + 1]))
                    return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
            }
            else
            {
                bool edgeA = !(nodesA[a] == nodesA[a + 1]);
                bool edgeB = !(nodesB[b] == nodesB[b + 1]);

                if (edgeA and !edgeB) // Edge-Vertex conflict
                {
                    if (nodesB[b] == nodesA[a])
                    {
                        if (nodesA[a].g + safe_time > nodesB[b].g and nodesA[a].g < nodesB[b + 1].g)
                            return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                    }
                    else if (nodesB[b] == nodesA[a + 1])
                    {
                        if (nodesA[a + 1].g > nodesB[b].g and nodesA[a + 1].g < nodesB[b + 1].g + safe_time)
                            return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                    }
                    else if (isSingleLane(nodesB[b]))
                    {
                        if (nodesB[b].id == nodesA[a].id and nodesB[b].orientation == nodesA[a].orientation)
                        {
                            if (nodesA[a].g + safe_time > nodesB[b].g and nodesA[a].g < nodesB[b + 1].g)
                                return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                        }
                        else if (nodesB[b].id == nodesA[a + 1].id and nodesB[b].orientation == nodesA[a + 1].orientation)
                        {
                            if (nodesA[a + 1].g > nodesB[b].g and nodesA[a + 1].g < nodesB[b + 1].g + safe_time)
                                return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                        }
                    }
                }
                else if (!edgeA and edgeB) // Vertex-Edge conflict
                {
                    if (nodesA[a] == nodesB[b])
                    {
                        if (nodesB[b].g + safe_time > nodesA[a].g and nodesB[b].g < nodesA[a + 1].g)
                            return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                    }
                    else if (nodesA[a] == nodesB[b + 1])
                    {
                        if (nodesB[b + 1].g > nodesA[a].g and nodesB[b + 1].g < nodesA[a + 1].g + safe_time)
                            return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                    }
                    else if (isSingleLane(nodesA[a]))
                    {
                        if (nodesA[a].id == nodesB[b].id and nodesA[a].orientation == nodesB[b].orientation)
                        {
                            if (nodesB[b].g + safe_time > nodesA[a].g and nodesB[b].g < nodesA[a + 1].g)
                                return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                        }
                        else if (nodesA[a].id == nodesB[b + 1].id and nodesA[a].orientation == nodesB[b + 1].orientation)
                        {
                            if (nodesB[b + 1].g > nodesA[a].g and nodesB[b + 1].g < nodesA[a + 1].g + safe_time)
                                return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                        }
                    }
                }
                else if (edgeA and edgeB) // Edge-Edge conflict
                {
                    if (nodesA[a] == nodesB[b] and nodesA[a + 1] == nodesB[b + 1]) // sharing both source target (following)
                    {
                        double diff1 = nodesA[a].g - nodesB[b].g;
                        double diff2 = nodesA[a + 1].g - nodesB[b + 1].g;
                        if (fabs(diff1) < safe_time or fabs(diff2) < safe_time) // assume two robots are moving with constant speed
                            return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                    }
                    else if (nodesA[a + 1] == nodesB[b]) // sharing target - source
                    {
                        double diff = nodesA[a + 1].g - nodesB[b].g;
                        if (fabs(diff) < safe_time)
                            return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                    }
                    else if (nodesA[a] == nodesB[b + 1]) // sharing source - target
                    {
                        double diff = nodesA[a].g - nodesB[b + 1].g;
                        if (fabs(diff) < safe_time)
                            return Conflict(pathA.agentID, pathB.agentID, Move(nodesA[a], nodesA[a + 1]), Move(nodesB[b], nodesB[b + 1]), std::min(nodesA[a].g, nodesB[b].g));
                    }
                }
                // Vertex-Vertex conflict is ignored
            }
        }
        if (a == nodesA.size() - 1)
            b++;
        else if (b == nodesB.size() - 1)
            a++;
        else if (fabs(nodesA[a + 1].g - nodesB[b + 1].g) < CN_EPSILON)
        {
            a++;
            b++;
        }
        else if (nodesA[a + 1].g < nodesB[b + 1].g)
            a++;
        else if (nodesB[b + 1].g - CN_EPSILON < nodesA[a + 1].g)
            b++;
    }
    return Conflict();
}

std::vector<Conflict> CBS::get_all_replan_conflicts(const std::vector<sPath> &paths, int id)
{
    std::vector<Conflict> conflicts;
    if (id < 0)
        for (unsigned int i = 0; i < paths.size(); i++)
            for (unsigned int j = i + 1; j < paths.size(); j++)
            {
                Conflict conflict = check_replan_paths(paths[i], paths[j]);
                if (conflict.agent1 >= 0)
                    conflicts.push_back(conflict);
            }
    else
    {
        for (unsigned int i = 0; i < paths.size(); i++)
        {
            if (int(i) == id)
                continue;
            Conflict conflict = check_replan_paths(paths[i], paths[id]);
            if (conflict.agent1 >= 0)
                conflicts.push_back(conflict);
        }
    }
    return conflicts;
}

double CBS::get_cost(CBS_Node node, int agent_id)
{
    while (node.parent != nullptr)
    {
        if (node.paths.begin()->agentID == agent_id)
            return node.paths.begin()->cost;
        node = *node.parent;
    }
    return node.paths.at(agent_id).cost;
}

std::vector<sPath> CBS::get_paths(CBS_Node *node, unsigned int agents_size)
{
    CBS_Node *curNode = node;
    std::vector<sPath> paths(agents_size);
    while (curNode->parent != nullptr)
    {
        if (paths.at(curNode->paths.begin()->agentID).cost < 0)
            paths.at(curNode->paths.begin()->agentID) = *curNode->paths.begin();
        curNode = curNode->parent;
    }
    for (unsigned int i = 0; i < agents_size; i++)
        if (paths.at(i).cost < 0)
            paths.at(i) = curNode->paths.at(i);
    return paths;
}

void CBS::generate_random_obstacles(Map &map, int obs_num)
{
    std::vector<std::pair<int, int>> candidate_edges;
    std::vector<std::pair<int, int>> selected_edges;
    for (auto &im : inter_motion)
    {
        // if(node_info[im.first.first].type == 1 and node_info[im.first.second].type == 1 and im.second.paths.size() > 10 and im.second.paths.back().t > 10)
        if (node_info[im.first.first].type == 1 and node_info[im.first.second].type == 1)
        {
            candidate_edges.emplace_back(im.first.first, im.first.second);
        }
    }
    int num = obs_num;
    if (num > candidate_edges.size())
    {
        std::cout << "Obstacle Number exceeds the size of candidate edges = " << candidate_edges.size() << std::endl;
        num = candidate_edges.size();
    }
    std::random_device rd;
    std::mt19937 gen(rd());

    std::shuffle(candidate_edges.begin(), candidate_edges.end(), gen);
    auto isSameCorridor = [](const std::pair<int, int> &edge1, const std::pair<int, int> &edge2)
    {
        return edge1 == edge2 or (edge1.first == edge2.second and edge1.second == edge2.first);
    };
    int idx = 0;
    selected_edges.emplace_back(213, 207);
    selected_edges.emplace_back(140, 169);
    selected_edges.emplace_back(213, 203); // you can force to generate an obstacle at certain edges
    while (selected_edges.size() < num and idx < candidate_edges.size())
    {
        if (selected_edges.empty())
        {
            selected_edges.emplace_back(candidate_edges[idx]);
            ++idx;
            continue;
        }
        if (!selected_edges.empty())
        {
            bool already_selected = false;
            for (std::size_t i = 0; i < selected_edges.size(); ++i)
            {
                if (isSameCorridor(candidate_edges[idx], selected_edges[i]))
                {
                    already_selected = true;
                    break;
                }
            }
            if (already_selected)
            {
                ++idx;
                continue;
            }
            selected_edges.emplace_back(candidate_edges[idx]);
            ++idx;
        }
    }

    for (std::size_t i = 0; i < selected_edges.size(); ++i)
    {
        int ent = selected_edges[i].first;
        int ext = selected_edges[i].second;
        auto &temp_motion = inter_motion.at({ent, ext}).paths;

        // std::uniform_int_distribution<> dist(4, temp_motion.size()-5);
        std::uniform_int_distribution<> dist(0, temp_motion.size() - 1);
        int idx = dist(gen);
        double start_t = temp_motion.front().t;
        double end_t = temp_motion.back().t;
        // while(temp_motion.at(idx).t < start_t+3.0 or temp_motion.at(idx).t > end_t-3.0)
        // while(temp_motion.at(idx).t < start_t+config.safe_time*2 or temp_motion.at(idx).t > end_t-config.safe_time*2)
        //     idx = dist(gen); // SJ Commented because it reduces the options of obstacle placement
        double ox = temp_motion.at(idx).position(0);
        double oy = temp_motion.at(idx).position(1);

        std::cout << "Obstacle is added --> [" << map.get_obstacles().size() << "," << ent << "," << ext << ","
                  << ox << "," << oy << "," << "idx->" << idx << "]" << std::endl;
        if (ent == 64 and ext == 66) // for real-time uturn test
            map.add_obstacle(Obstacle(map.get_obstacles().size(), ent, ext, temp_motion.at(idx).position(0), temp_motion.at(idx).position(1), 0.5, 2.0, false));
        else
            map.add_obstacle(Obstacle(map.get_obstacles().size(), ent, ext, ox, oy, 0.4, 0.4, false));
    }
}

// Before changing obs_type ------------------------------------------------------------------------
void CBS::add_obstacle(Map &map, Obstacle obs)
{

    // Notice:
    // Assuming all obstacles are in the form of a box
    // Obstacle structure: <corridor_ent_id, corridor_ext_id, obs_x, obs_y, obs_width, obs_height>
    // Obstacle x, y is the center of the box
    int id = obs.id;
    int ent = obs.ent;
    int ext = obs.ext;
    double ox = obs.ox;
    double oy = obs.oy;
    double ow = obs.ow;
    double oh = obs.oh;

    if (map.get_obstacle_info().find({ent, ext}) != map.get_obstacle_info().end())
    {
        // The obstacle is already registered in the current map
        return;
    }

    auto forward_motion = inter_motion.at({ent, ext}).paths;
    auto opposite_motion = inter_motion.at({ext, ent}).paths;
    bool avoidable = true;

    std::cout << "Add obstacle start! obstacle info is [" << ent << ", " << ext << ", " << ox << ", " << oy << ", " << ow << ", " << oh << "]" << std::endl;

    auto circle_rectangle_collision = [&](double cx, double cy, double cr, double rx, double ry, double rw, double rh)
    {
        double testX = cx;
        double testY = cy;

        // when the rx, ry are the center of the rectangle
        if (cx < rx - 0.5 * rw)
            testX = rx - 0.5 * rw;
        else if (cx > rx + 0.5 * rw)
            testX = rx + 0.5 * rw;

        if (cy < ry - 0.5 * rh)
            testY = ry - 0.5 * rh;
        else if (cy > ry + 0.5 * rh)
            testY = ry + 0.5 * rh;

        // when the rx, ry are the left corner of the rectangle///
        //  if (cx < rx)            testX = rx;
        //  else if (cx > rx+rw)    testX = rx+rw;

        // if (cy < ry)            testY = ry;
        // else if (cy > ry+rh)    testY = ry+rh;
        /////////////////////////////////////////////////////////

        double dist = sqrt(pow(cx - testX, 2) + pow(cy - testY, 2));

        if (dist <= cr)
        {
            return true;
        }
        return false;
    };

    std::pair<int, timedPoint> min_tp1 = std::make_pair(forward_motion.size() - 1, forward_motion.back());
    std::pair<int, timedPoint> max_tp1 = std::make_pair(0, forward_motion.front());
    bool collide1 = false;
    for (std::size_t i = 0; i < forward_motion.size(); ++i)
    {
        double px = forward_motion[i].position(0);
        double py = forward_motion[i].position(1);
        if (circle_rectangle_collision(px, py, config.agent_size, ox, oy, ow, oh))
        {
            collide1 = true;
            if (forward_motion[i].t < min_tp1.second.t)
                min_tp1 = std::make_pair(i, forward_motion[i]);
            if (forward_motion[i].t > max_tp1.second.t)
                max_tp1 = std::make_pair(i, forward_motion[i]);
        }
    }

    std::pair<int, timedPoint> min_tp2 = std::make_pair(opposite_motion.size() - 1, opposite_motion.back());
    std::pair<int, timedPoint> max_tp2 = std::make_pair(0, opposite_motion.front());
    bool collide2 = false;
    for (std::size_t i = 0; i < opposite_motion.size(); ++i)
    {
        double px = opposite_motion[i].position(0);
        double py = opposite_motion[i].position(1);
        if (circle_rectangle_collision(px, py, config.agent_size, ox, oy, ow, oh))
        {
            collide2 = true;
            if (opposite_motion[i].t < min_tp2.second.t)
                min_tp2 = std::make_pair(i, opposite_motion[i]);
            if (opposite_motion[i].t > max_tp2.second.t)
                max_tp2 = std::make_pair(i, opposite_motion[i]);
        }
    }
    std::cout << "ent->ext edge collide? " << collide1 << "\next->ent edge collide? " << collide2 << std::endl;
    if (collide1 and !collide2) // generate avoidance motion for ent->ext edge
    {
        std::cout << "Obstacle partially blocks one lane: [" << map.get_obstacles().size() << ", " << ent << ", " << ext << ", " << min_tp1.second.t << ", " << max_tp1.second.t << "]" << std::endl;
        // if(min_tp1.first < 3 or max_tp1.first +2 >= forward_motion.size()-1)   //there is no room to avoid this obstacle when the robot enter or exit the edge.
        if (min_tp1.second.t < config.safe_time or forward_motion.back().t - max_tp1.second.t < config.safe_time)
        {
            std::cout << "There is no room to avoid this obstacle in the lane. So we consider this edge as a blocked corridor" << std::endl;
            avoidable = false;
        }
        else
        {
            int op1 = -1;
            int op2 = -1;
            int op3 = -1;
            int start_idx1 = min_tp1.first - 1;
            int end_idx1 = max_tp1.first + 1;
            while (forward_motion[min_tp1.first].t - forward_motion[start_idx1].t < config.safe_time)
                --start_idx1;
            while (forward_motion[end_idx1].t - forward_motion[max_tp1.first].t < config.safe_time)
                ++end_idx1;
            Eigen::Vector2d v1 = forward_motion[min_tp1.first - 1].position - forward_motion[min_tp1.first - 2].position;
            Eigen::Vector2d v2 = forward_motion[max_tp1.first + 3].position - forward_motion[max_tp1.first + 2].position;
            // Eigen::Vector2d v1 = forward_motion[v1_idx].position - forward_motion[v1_idx-1].position;
            // Eigen::Vector2d v2 = forward_motion[v2_idx+1].position - forward_motion[v2_idx].position;
            double minDist1 = CN_INFINITY;
            double minDist2 = CN_INFINITY;
            double minDist3 = CN_INFINITY;
            for (int om = 0; om < opposite_motion.size(); ++om)
            {
                auto op = opposite_motion[om].position;
                Eigen::Vector2d v3 = op - forward_motion[min_tp1.first - 2].position;
                Eigen::Vector2d v4 = op - forward_motion[max_tp1.first + 2].position;
                if (v1.dot(v3) > 0 and v3.norm() < minDist1)
                {
                    op1 = om;
                    minDist1 = v3.norm();
                }
                if (v2.dot(v4) < 0 and v4.norm() < minDist2)
                {
                    op2 = om;
                    minDist2 = v4.norm();
                }
                if (v1.dot(v3) < 0 and v3.norm() < minDist3)
                {
                    op3 = om;
                    minDist3 = v3.norm();
                }
            }
            // Interpolate and generate trajectory
            //  avoidance waypoints
            //  [ forward[min_tp1.first-3], forward[min_tp1.first-2], opposite[op1] ~ opposite[op2], forward[max_tp1.first+2], forward[max_tp1.first+3] ]
            //
            //  u-turn waypoints
            //  [ forward[min_tp1.first-3], forward[min_tp1.first-2], opposite[op3], opposite[op3+1] ]

            std::vector<Eigen::Vector2d> avoid_wps, uturn_wps;

            // avoid_wps.push_back(forward_motion[min_tp1.first-3].position);
            // avoid_wps.push_back(forward_motion[min_tp1.first-2].position);
            avoid_wps.push_back(forward_motion[start_idx1 - 1].position);
            avoid_wps.push_back(forward_motion[start_idx1].position);
            uturn_wps = avoid_wps;
            for (std::size_t i = 0; i <= op1 - op2; ++i)
            {
                avoid_wps.push_back(opposite_motion[op1 - i].position);
            }
            // avoid_wps.push_back(forward_motion[max_tp1.first+2].position);
            // avoid_wps.push_back(forward_motion[max_tp1.first+3].position);
            avoid_wps.push_back(forward_motion[end_idx1].position);
            avoid_wps.push_back(forward_motion[end_idx1 + 1].position);

            uturn_wps.push_back(opposite_motion[op3].position);
            uturn_wps.push_back(opposite_motion[op3 + 1].position);

            auto avoid_traj = generatePPTrajectory(avoid_wps, 10, 0.002, 0.1, 0.6, 2.5, 0.1, config.agent_size * 0.5);
            int start_idx2 = op2 - 1;
            int end_idx2 = op1 + 1;
            while (opposite_motion[op2].t - opposite_motion[start_idx2].t < config.safe_time and start_idx2 - 2 >= 0)
                --start_idx2;
            while (opposite_motion[end_idx2].t - opposite_motion[op1].t < config.safe_time and end_idx2 + 2 < opposite_motion.size())
                ++end_idx2;
            // std::vector<timedPoint> avoid_traj2(opposite_motion.begin()+op2-1, opposite_motion.begin()+op1+2);
            std::vector<timedPoint> avoid_traj2(opposite_motion.begin() + start_idx2 - 1, opposite_motion.begin() + end_idx2 + 2);
            double offset = avoid_traj2.front().t;
            for (std::size_t i = 0; i < avoid_traj2.size(); ++i)
                avoid_traj2[i].t -= offset;
            // collision intervals between type 3 and type 5
            std::pair<double, double> avoid_interval1 = std::make_pair(0.0, avoid_traj.back().t);
            std::pair<double, double> avoid_interval2 = std::make_pair(0.0, avoid_traj2.back().t);

            // std::vector<timedPoint> avoid_remaining1(forward_motion.begin() + max_tp1.first+3, forward_motion.end());
            // std::vector<timedPoint> avoid_remaining2(opposite_motion.begin()+op1+1, opposite_motion.end());
            std::vector<timedPoint> avoid_remaining1(forward_motion.begin() + end_idx1 + 1, forward_motion.end());
            std::vector<timedPoint> avoid_remaining2(opposite_motion.begin() + end_idx2 + 1, opposite_motion.end());

            for (std::size_t i = 0; i < avoid_remaining1.size(); ++i)
                avoid_remaining1[i].t -= avoid_remaining1.front().t;
            for (std::size_t i = 0; i < avoid_remaining2.size(); ++i)
                avoid_remaining2[i].t -= avoid_remaining2.front().t;

            // Notice:
            // I believe that separating 'avoid_traj' from 'avoid_remaining' is more feasible.
            // Additionally, generating 'uturn_traj' is unnecessary when the robot can avoid the obstacle.
            // Therefore, although I have written some code for the 'uturn' motion, it has not been tested yet.

            // auto uturn_traj = generatePPTrajectory(uturn_wps, 10, 0.002, 0.1, 0.6, 2.5, 0.1, config.agent_size*0.5);
            // collision intervals between type 4 and type 5
            // std::pair<double, double> uturn_interval1 = std::make_pair(0.0, uturn_traj.back().t);
            // std::pair<double, double> uturn_interval2 = std::make_pair(opposite_motion[op3-1].t - opposite_motion[op2-1].t, opposite_motion[op3+2].t - opposite_motion[op2-1].t);

            // std::vector<timedPoint> uturn_remaining(opposite_motion.begin() + op3+1, opposite_motion.end());
            // offset = uturn_traj.back().t + (uturn_traj.back().position-uturn_remaining.front().position).norm()/config.trVel;
            // for(std::size_t i=0; i < uturn_remaining.size(); ++i)
            // {
            //     uturn_traj.emplace_back(uturn_remaining[i].position(0), uturn_remaining[i].position(1), uturn_remaining[i].t+offset);
            // }

            ObstacleInfo info1(id, ent, ext, 0, ox, oy, ow, oh);
            // std::copy(forward_motion.begin(), forward_motion.begin() + min_tp1.first-2, std::back_inserter(info1.entry_traj));
            std::copy(forward_motion.begin(), forward_motion.begin() + start_idx1, std::back_inserter(info1.entry_traj));
            std::copy(avoid_traj.begin(), avoid_traj.end(), std::back_inserter(info1.avoid_traj));
            std::copy(avoid_remaining1.begin(), avoid_remaining1.end(), std::back_inserter(info1.exit_traj));
            info1.avoid_interval = avoid_interval1;

            // std::copy(uturn_traj.begin(), uturn_traj.end(), std::back_inserter(info1.uturn_traj));
            // info1.uturn_interval = uturn_inverval1;
            std::cout << "ObstacleInfo1 is as below" << std::endl;
            info1.display_info();
            map.add_obstacle_info(info1);

            ObstacleInfo info2(id, ext, ent, 2, ox, oy, ow, oh);
            // std::copy(opposite_motion.begin(), opposite_motion.begin() + op2, std::back_inserter(info2.entry_traj));
            std::copy(opposite_motion.begin(), opposite_motion.begin() + start_idx2, std::back_inserter(info2.entry_traj));
            std::copy(avoid_traj2.begin(), avoid_traj2.end(), std::back_inserter(info2.avoid_traj));
            // std::copy(opposite_motion.begin()+op2-1, opposite_motion.begin()+op1+2, std::back_inserter(info2.avoid_traj));
            std::copy(avoid_remaining2.begin(), avoid_remaining2.end(), std::back_inserter(info2.exit_traj));
            info2.avoid_interval = avoid_interval2;
            // info2.uturn_interval = uturn_interval2;

            std::cout << "ObstacleInfo2 is as below" << std::endl;
            info2.display_info();
            map.add_obstacle_info(info2);
        }
    }
    else if (!collide1 and collide2) // generate avoidance motion for ext->ent edge
    {
        std::cout << "Obstacle partially blocks one lane: [" << map.get_obstacles().size() << ", " << ext << ", " << ent << ", " << min_tp2.second.t << ", " << max_tp2.second.t << "]" << std::endl;
        // if(min_tp2.first < 3 or max_tp2.first +2 >= opposite_motion.size()-1)
        if (min_tp2.second.t < config.safe_time or opposite_motion.back().t - max_tp2.second.t < config.safe_time)
        {
            std::cout << "There is no room to avoid this obstacle in the lane. So we consider this edge as a blocked corridor" << std::endl;
            avoidable = false;
        }
        else
        {
            int fp1 = -1;
            int fp2 = -1;
            int fp3 = -1;
            double minDist1 = CN_INFINITY;
            double minDist2 = CN_INFINITY;
            double minDist3 = CN_INFINITY;
            int start_idx1 = min_tp2.first - 1;
            int end_idx1 = max_tp2.first + 1;
            while (opposite_motion[min_tp2.first].t - opposite_motion[start_idx1].t < config.safe_time)
                --start_idx1;
            while (opposite_motion[end_idx1].t - opposite_motion[max_tp2.first].t < config.safe_time)
                ++end_idx1;
            Eigen::Vector2d v1 = opposite_motion[min_tp2.first - 1].position - opposite_motion[min_tp2.first - 2].position;
            Eigen::Vector2d v2 = opposite_motion[max_tp2.first + 3].position - opposite_motion[max_tp2.first + 2].position;
            for (int fm = 0; fm < forward_motion.size(); ++fm)
            {
                auto fp = forward_motion[fm].position;
                Eigen::Vector2d v3 = fp - opposite_motion[min_tp2.first - 2].position;
                Eigen::Vector2d v4 = fp - opposite_motion[max_tp2.first + 2].position;
                if (v1.dot(v3) > 0 and v3.norm() < minDist1)
                {
                    fp1 = fm;
                    minDist1 = v3.norm();
                }
                if (v2.dot(v4) < 0 and v4.norm() < minDist2)
                {
                    fp2 = fm;
                    minDist2 = v4.norm();
                }
                if (v1.dot(v3) < 0 and v3.norm() < minDist3)
                {
                    fp3 = fm;
                    minDist3 = v3.norm();
                }
            }
            // Interpolate and generate trajectory
            //  avoidance waypoints
            //  [ opposite[min_tp2.first-3], opposite[min_tp2.first-2], forward[fp1] ~ forward[fp2], opposite[max_tp2.first+2], opposite[max_tp2.first+3] ]
            //
            //  u-turn waypoints
            //  [ opposite[min_tp1.first-3], opposite[min_tp1.first-2], forward[op3], forward[op3+1] ]

            std::vector<Eigen::Vector2d> avoid_wps, uturn_wps;

            // avoid_wps.push_back(opposite_motion[min_tp2.first-3].position);
            // avoid_wps.push_back(opposite_motion[min_tp2.first-2].position);
            avoid_wps.push_back(opposite_motion[start_idx1 - 1].position);
            avoid_wps.push_back(opposite_motion[start_idx1].position);
            uturn_wps = avoid_wps;
            for (std::size_t i = 0; i <= fp1 - fp2; ++i)
            {
                avoid_wps.push_back(forward_motion[fp1 - i].position);
            }
            // avoid_wps.push_back(opposite_motion[max_tp2.first+2].position);
            // avoid_wps.push_back(opposite_motion[max_tp2.first+3].position);
            avoid_wps.push_back(opposite_motion[end_idx1].position);
            avoid_wps.push_back(opposite_motion[end_idx1 + 1].position);

            uturn_wps.push_back(forward_motion[fp3].position);
            uturn_wps.push_back(forward_motion[fp3 + 1].position);

            auto avoid_traj = generatePPTrajectory(avoid_wps, 10, 0.002, 0.1, config.trVel, 2.5, 0.1, config.agent_size * 0.5);
            int start_idx2 = fp2 - 1;
            int end_idx2 = fp1 + 1;
            while (forward_motion[fp2].t - forward_motion[start_idx2].t < config.safe_time and start_idx2 - 2 >= 0)
                --start_idx2;
            while (forward_motion[end_idx2].t - forward_motion[fp1].t < config.safe_time and end_idx2 + 2 < forward_motion.size())
                ++end_idx2;
            // std::vector<timedPoint> avoid_traj2(forward_motion.begin()+fp2-1, forward_motion.begin()+fp1+2);
            std::vector<timedPoint> avoid_traj2(forward_motion.begin() + start_idx2 - 1, forward_motion.begin() + end_idx2 + 2);
            double offset = avoid_traj2.front().t;
            for (std::size_t i = 0; i < avoid_traj2.size(); ++i)
                avoid_traj2[i].t -= offset;

            // collision intervals between type 3 and type 5
            std::pair<double, double> avoid_interval1 = std::make_pair(0.0, avoid_traj.back().t);
            std::pair<double, double> avoid_interval2 = std::make_pair(0.0, avoid_traj2.back().t);

            // std::vector<timedPoint> avoid_remaining1(opposite_motion.begin() +max_tp2.first+3, opposite_motion.end());
            // std::vector<timedPoint> avoid_remaining2(forward_motion.begin()+fp1+1, forward_motion.end());
            std::vector<timedPoint> avoid_remaining1(opposite_motion.begin() + end_idx1 + 1, opposite_motion.end());
            std::vector<timedPoint> avoid_remaining2(forward_motion.begin() + end_idx2 + 1, forward_motion.end());

            for (std::size_t i = 0; i < avoid_remaining1.size(); ++i)
                avoid_remaining1[i].t -= avoid_remaining1.front().t;
            for (std::size_t i = 0; i < avoid_remaining2.size(); ++i)
                avoid_remaining2[i].t -= avoid_remaining2.front().t;

            // auto uturn_traj = generatePPTrajectory(uturn_wps, 10, 0.002, 0.1, config.trVel, 2.5, 0.1, config.agent_size*0.5);
            // //collision intervals between type 4 and type 5
            // std::pair<double, double> uturn_interval1 = std::make_pair(0.0, uturn_traj.back().t);
            // std::pair<double, double> uturn_interval2 = std::make_pair(forward_motion[fp3-1].t - forward_motion[fp2-1].t, forward_motion[fp3+2].t - forward_motion[fp2-1].t);

            // std::vector<timedPoint> uturn_remaining(forward_motion.begin() + fp3+1, forward_motion.end());
            // offset = uturn_traj.back().t + (uturn_traj.back().position-uturn_remaining.front().position).norm()/config.trVel;
            // for(std::size_t i=0; i < uturn_remaining.size(); ++i)
            // {
            //     uturn_traj.emplace_back(uturn_remaining[i].position(0), uturn_remaining[i].position(1), uturn_remaining[i].t+offset);
            // }

            ObstacleInfo info1(id, ext, ent, 0, ox, oy, ow, oh);
            // std::copy(opposite_motion.begin(), opposite_motion.begin() + min_tp2.first-2, std::back_inserter(info1.entry_traj));
            std::copy(opposite_motion.begin(), opposite_motion.begin() + start_idx1, std::back_inserter(info1.entry_traj));
            std::copy(avoid_traj.begin(), avoid_traj.end(), std::back_inserter(info1.avoid_traj));
            std::copy(avoid_remaining1.begin(), avoid_remaining1.end(), std::back_inserter(info1.exit_traj));
            info1.avoid_interval = avoid_interval1;

            std::cout << "ObstacleInfo1 is as below" << std::endl;
            info1.display_info();
            // std::copy(uturn_traj.begin(), uturn_traj.end(), std::back_inserter(info1.uturn_traj));
            // info1.uturn_interval = uturn_inverval1;
            map.add_obstacle_info(info1);

            ObstacleInfo info2(id, ent, ext, 2, ox, oy, ow, oh);
            // std::copy(forward_motion.begin(), forward_motion.begin() + fp2, std::back_inserter(info2.entry_traj));
            std::copy(forward_motion.begin(), forward_motion.begin() + start_idx2, std::back_inserter(info2.entry_traj));
            std::copy(avoid_traj2.begin(), avoid_traj2.end(), std::back_inserter(info2.avoid_traj));
            // std::copy(forward_motion.begin()+fp2-1, forward_motion.begin()+fp1+2, std::back_inserter(info2.exit_traj));
            std::copy(avoid_remaining2.begin(), avoid_remaining2.end(), std::back_inserter(info2.exit_traj));
            info2.avoid_interval = avoid_interval2;
            // info2.uturn_interval = uturn_interval2;
            std::cout << "ObstacleInfo2 is as below" << std::endl;
            info2.display_info();
            map.add_obstacle_info(info2);
        }
    }
    else if (collide1 and collide2) // no other choice than u-turns
    {
        std::cout << "Obstacle blocks both lanes: [" << map.get_obstacles().size() << ", " << ent << " <--> " << ext << ", ("
                  << min_tp1.second.t << ", " << max_tp1.second.t << "), (" << min_tp2.second.t << ", " << max_tp2.second.t << ")]" << std::endl;
        // if(min_tp1.first < 3 or max_tp1.first+2 >= forward_motion.size()-1 or min_tp2.first < 3 or max_tp2.first +2 >= opposite_motion.size()-1)
        if (min_tp1.second.t < config.safe_time or forward_motion.back().t - max_tp1.second.t < config.safe_time or min_tp2.second.t < config.safe_time or opposite_motion.back().t - max_tp2.second.t < config.safe_time)
        {
            std::cout << "There is no room to uturn. So we consider this edge as a blocked corridor" << std::endl;
            avoidable = false;
        }
        else
        {
            int fp1 = -1;
            int op1 = -1;
            double minDist1 = CN_INFINITY;
            double minDist2 = CN_INFINITY;

            int start_idx1 = min_tp1.first - 1;
            int start_idx2 = min_tp2.first - 1;
            while (forward_motion[min_tp1.first].t - forward_motion[start_idx1].t < config.safe_time)
                --start_idx1;
            while (opposite_motion[min_tp2.first].t - opposite_motion[start_idx2].t < config.safe_time)
                --start_idx2;

            Eigen::Vector2d v1 = forward_motion[min_tp1.first - 1].position - forward_motion[min_tp1.first - 2].position;
            Eigen::Vector2d v2 = opposite_motion[min_tp2.first - 1].position - opposite_motion[min_tp2.first - 2].position;
            for (int om = 0; om < opposite_motion.size(); ++om)
            {
                auto op = opposite_motion[om].position;
                Eigen::Vector2d v3 = op - forward_motion[min_tp1.first - 2].position;
                if (v1.dot(v3) < 0 and v3.norm() < minDist1)
                {
                    op1 = om;
                    minDist1 = v3.norm();
                }
            }
            for (int fm = 0; fm < forward_motion.size(); ++fm)
            {
                auto fp = forward_motion[fm].position;
                Eigen::Vector2d v4 = fp - opposite_motion[min_tp2.first - 2].position;
                if (v2.dot(v4) < 0 and v4.norm() < minDist2)
                {
                    fp1 = fm;
                    minDist2 = v4.norm();
                }
            }
            std::vector<Eigen::Vector2d> uturn_wps1, uturn_wps2;
            // uturn_wps1.push_back(forward_motion[min_tp1.first-3].position);
            // uturn_wps1.push_back(forward_motion[min_tp1.first-2].position);
            uturn_wps1.push_back(forward_motion[start_idx1 - 1].position);
            uturn_wps1.push_back(forward_motion[start_idx1].position);
            uturn_wps1.push_back(opposite_motion[op1].position);
            uturn_wps1.push_back(opposite_motion[op1 + 1].position);

            // uturn_wps2.push_back(opposite_motion[min_tp2.first-3].position);
            // uturn_wps2.push_back(opposite_motion[min_tp2.first-2].position);
            uturn_wps2.push_back(opposite_motion[start_idx2 - 1].position);
            uturn_wps2.push_back(opposite_motion[start_idx2].position);
            uturn_wps2.push_back(forward_motion[fp1].position);
            uturn_wps2.push_back(forward_motion[fp1 + 1].position);

            std::vector<timedPoint> uturn_remaining1(opposite_motion.begin() + op1 + 1, opposite_motion.end());
            std::vector<timedPoint> uturn_remaining2(forward_motion.begin() + fp1 + 1, forward_motion.end());
            auto uturn_traj1 = generatePPTrajectory(uturn_wps1, 10, 0.002, 0.1, config.trVel, 2.5, 0.1, config.agent_size * 0.5);
            double offset = uturn_traj1.back().t - uturn_remaining1.front().t;
            for (std::size_t i = 1; i < uturn_remaining1.size(); ++i)
            {
                uturn_traj1.emplace_back(uturn_remaining1[i].position(0), uturn_remaining1[i].position(1), uturn_remaining1[i].t + offset);
            }

            auto uturn_traj2 = generatePPTrajectory(uturn_wps2, 10, 0.002, 0.1, config.trVel, 2.5, 0.1, config.agent_size * 0.5);
            offset = uturn_traj2.back().t - uturn_remaining2.front().t;
            for (std::size_t i = 1; i < uturn_remaining2.size(); ++i)
            {
                uturn_traj2.emplace_back(uturn_remaining2[i].position(0), uturn_remaining2[i].position(1), uturn_remaining2[i].t + offset);
            }
            ObstacleInfo info1(id, ent, ext, 1, ox, oy, ow, oh);
            // std::copy(forward_motion.begin(), forward_motion.begin() + min_tp1.first-2, std::back_inserter(info1.entry_traj));
            std::copy(forward_motion.begin(), forward_motion.begin() + start_idx1, std::back_inserter(info1.entry_traj));
            std::copy(uturn_traj1.begin(), uturn_traj1.end(), std::back_inserter(info1.uturn_traj));
            std::cout << "ObstacleInfo1 is as below" << std::endl;
            info1.display_info();
            map.add_obstacle_info(info1);

            ObstacleInfo info2(id, ext, ent, 1, ox, oy, ow, oh);
            // std::copy(opposite_motion.begin(), opposite_motion.begin() + min_tp2.first-2, std::back_inserter(info2.entry_traj));
            std::copy(opposite_motion.begin(), opposite_motion.begin() + start_idx2, std::back_inserter(info2.entry_traj));
            std::copy(uturn_traj2.begin(), uturn_traj2.end(), std::back_inserter(info2.uturn_traj));
            std::cout << "ObstacleInfo2 is as below" << std::endl;
            info2.display_info();
            map.add_obstacle_info(info2);
        }
    }
    if (!avoidable)
    {
        std::cout << "Current obstacle is too close to the intersection. I will ignore such cases for now." << std::endl;
    }
}
// Before changing obs_type ------------------------------------------------------------------------

// After changing obs_type -------------------------------------------------------------------------
void CBS::register_obstacle(Map &map, std::vector<Obstacle> new_obstacles)
{
    auto circle_rectangle_collision = [&](double cx, double cy, double cr, double rx, double ry, double rw, double rh)
    {
        double testX = cx;
        double testY = cy;

        // when the rx, ry are the center of the rectangle
        if (cx < rx - 0.5 * rw)
            testX = rx - 0.5 * rw;
        else if (cx > rx + 0.5 * rw)
            testX = rx + 0.5 * rw;

        if (cy < ry - 0.5 * rh)
            testY = ry - 0.5 * rh;
        else if (cy > ry + 0.5 * rh)
            testY = ry + 0.5 * rh;

        // when the rx, ry are the left corner of the rectangle///
        //  if (cx < rx)            testX = rx;
        //  else if (cx > rx+rw)    testX = rx+rw;

        // if (cy < ry)            testY = ry;
        // else if (cy > ry+rh)    testY = ry+rh;
        /////////////////////////////////////////////////////////

        double dist = sqrt(pow(cx - testX, 2) + pow(cy - testY, 2));

        if (dist <= cr)
        {
            return true;
        }
        return false;
    };

    std::unordered_map<std::pair<int, int>, std::vector<Obstacle>, PairHash> obstacles_in_edge;
    for (auto &new_obs : new_obstacles)
    {
        int ent = std::min(new_obs.ent, new_obs.ext); // Ensure smaller value is first
        int ext = std::max(new_obs.ent, new_obs.ext); // Ensure larger value is second

        // Use operator[] to insert a default vector if the key doesn't exist
        obstacles_in_edge[{ent, ext}].push_back(new_obs);
    }

    int id = map.get_obstacle_info().size();

    for (auto edge : obstacles_in_edge)
    {
        // Notice:
        // Assuming all obstacles are in the form of a box
        // Obstacle structure: <corridor_ent_id, corridor_ext_id, obs_x, obs_y, obs_width, obs_height>
        // Obstacle x, y is the center of the box
        // edge.first: <corridor_ent_id, corridor_ext_id>
        // edge.second: vector of obstacles in the edge

        int ent = edge.first.first;
        int ext = edge.first.second;

        auto forward_motion = inter_motion.at({ent, ext}).paths;
        auto opposite_motion = inter_motion.at({ext, ent}).paths;

        std::pair<int, timedPoint> min_tp1 = std::make_pair(forward_motion.size() - 1, forward_motion.back());
        std::pair<int, timedPoint> max_tp1 = std::make_pair(0, forward_motion.front());

        bool collide1 = false;
        for (int i = 0; i < forward_motion.size(); ++i)
        {
            double px = forward_motion[i].position(0);
            double py = forward_motion[i].position(1);
            for (auto obs : edge.second)
            {
                if (circle_rectangle_collision(px, py, config.agent_size, obs.ox, obs.oy, obs.ow, obs.oh))
                {
                    collide1 = true;
                    if (forward_motion[i].t < min_tp1.second.t)
                        min_tp1 = std::make_pair(i, forward_motion[i]);
                    if (forward_motion[i].t > max_tp1.second.t)
                        max_tp1 = std::make_pair(i, forward_motion[i]);
                }
            }
        }

        std::pair<int, timedPoint> min_tp2 = std::make_pair(opposite_motion.size() - 1, opposite_motion.back());
        std::pair<int, timedPoint> max_tp2 = std::make_pair(0, opposite_motion.front());

        bool collide2 = false;
        for (int i = 0; i < opposite_motion.size(); ++i)
        {
            double px = opposite_motion[i].position(0);
            double py = opposite_motion[i].position(1);
            for (auto obs : edge.second)
            {
                if (circle_rectangle_collision(px, py, config.agent_size, obs.ox, obs.oy, obs.ow, obs.oh))
                {
                    collide2 = true;
                    if (opposite_motion[i].t < min_tp2.second.t)
                        min_tp2 = std::make_pair(i, opposite_motion[i]);
                    if (opposite_motion[i].t > max_tp2.second.t)
                        max_tp2 = std::make_pair(i, opposite_motion[i]);
                }
            }
        }

        std::cout << "ent->ext edge collide? " << collide1 << "\next->ent edge collide? " << collide2 << std::endl;

        if (collide1 and !collide2) // lane1 is blocked by obstacle
        {
            std::cout << "Obstacle partially blocks one lane: [" << map.get_obstacles().size() << ", " << ent << ", " << ext << ", " << min_tp1.second.t << ", " << max_tp1.second.t << "]" << std::endl;
            if (min_tp1.second.t < config.safe_time or forward_motion.back().t - max_tp1.second.t < config.safe_time)
            {
                std::cout << "There is no room to avoid this obstacle in the lane. Case 1" << std::endl;
                for (auto &motion : inter_motion.at({ent, ext}).paths)
                {
                    if (motion.t > 0.0)
                        motion.t = CN_INFINITY;
                }
                for (auto &motion : forward_motion)
                {
                    if (motion.t > 0.0)
                        motion.t = CN_INFINITY;
                }
                inter_motion.at({ent, ext}).time_cost = CN_INFINITY;

                ObsEdgeInfo info(id, ent, ext, 1, edge.second);
                map.add_obs_edge_info(info);
            }
            else
            {
                int op1 = -1;
                int op2 = -1;
                int op3 = -1;
                int start_idx1 = min_tp1.first - 1;
                int end_idx1 = max_tp1.first + 1;
                while (forward_motion[min_tp1.first].t - forward_motion[start_idx1].t < config.safe_time)
                    --start_idx1;
                while (forward_motion[end_idx1].t - forward_motion[max_tp1.first].t < config.safe_time)
                    ++end_idx1;
                Eigen::Vector2d v1 = forward_motion[min_tp1.first - 1].position - forward_motion[min_tp1.first - 2].position;
                Eigen::Vector2d v2 = forward_motion[max_tp1.first + 3].position - forward_motion[max_tp1.first + 2].position;
                // Eigen::Vector2d v1 = forward_motion[v1_idx].position - forward_motion[v1_idx-1].position;
                // Eigen::Vector2d v2 = forward_motion[v2_idx+1].position - forward_motion[v2_idx].position;
                double minDist1 = CN_INFINITY;
                double minDist2 = CN_INFINITY;
                double minDist3 = CN_INFINITY;
                for (int om = 0; om < opposite_motion.size(); ++om)
                {
                    auto op = opposite_motion[om].position;
                    Eigen::Vector2d v3 = op - forward_motion[min_tp1.first - 2].position;
                    Eigen::Vector2d v4 = op - forward_motion[max_tp1.first + 2].position;
                    if (v1.dot(v3) > 0 and v3.norm() < minDist1)
                    {
                        op1 = om;
                        minDist1 = v3.norm();
                    }
                    if (v2.dot(v4) < 0 and v4.norm() < minDist2)
                    {
                        op2 = om;
                        minDist2 = v4.norm();
                    }
                    if (v1.dot(v3) < 0 and v3.norm() < minDist3)
                    {
                        op3 = om;
                        minDist3 = v3.norm();
                    }
                }
                // Interpolate and generate trajectory
                //  avoidance waypoints
                //  [ forward[min_tp1.first-3], forward[min_tp1.first-2], opposite[op1] ~ opposite[op2], forward[max_tp1.first+2], forward[max_tp1.first+3] ]
                //
                //  u-turn waypoints
                //  [ forward[min_tp1.first-3], forward[min_tp1.first-2], opposite[op3], opposite[op3+1] ]

                std::vector<Eigen::Vector2d> avoid_wps, uturn_wps;

                // avoid_wps.push_back(forward_motion[min_tp1.first-3].position);
                // avoid_wps.push_back(forward_motion[min_tp1.first-2].position);
                avoid_wps.push_back(forward_motion[start_idx1 - 1].position);
                avoid_wps.push_back(forward_motion[start_idx1].position);
                uturn_wps = avoid_wps;
                for (std::size_t i = 0; i <= op1 - op2; ++i)
                {
                    avoid_wps.push_back(opposite_motion[op1 - i].position);
                }
                // avoid_wps.push_back(forward_motion[max_tp1.first+2].position);
                // avoid_wps.push_back(forward_motion[max_tp1.first+3].position);
                avoid_wps.push_back(forward_motion[end_idx1].position);
                avoid_wps.push_back(forward_motion[end_idx1 + 1].position);

                uturn_wps.push_back(opposite_motion[op3].position);
                uturn_wps.push_back(opposite_motion[op3 + 1].position);

                auto avoid_traj = generatePPTrajectory(avoid_wps, 10, 0.002, 0.1, 0.6, 2.5, 0.1, config.agent_size * 0.5);
                int start_idx2 = op2 - 1;
                int end_idx2 = op1 + 1;
                while (opposite_motion[op2].t - opposite_motion[start_idx2].t < config.safe_time and start_idx2 - 2 >= 0)
                    --start_idx2;
                while (opposite_motion[end_idx2].t - opposite_motion[op1].t < config.safe_time and end_idx2 + 2 < opposite_motion.size())
                    ++end_idx2;
                // std::vector<timedPoint> avoid_traj2(opposite_motion.begin()+op2-1, opposite_motion.begin()+op1+2);
                std::vector<timedPoint> avoid_traj2(opposite_motion.begin() + start_idx2 - 1, opposite_motion.begin() + end_idx2 + 2);
                double offset = avoid_traj2.front().t;
                for (std::size_t i = 0; i < avoid_traj2.size(); ++i)
                    avoid_traj2[i].t -= offset;
                // collision intervals between type 3 and type 5
                std::pair<double, double> avoid_interval1 = std::make_pair(0.0, avoid_traj.back().t);
                std::pair<double, double> avoid_interval2 = std::make_pair(0.0, avoid_traj2.back().t);

                // std::vector<timedPoint> avoid_remaining1(forward_motion.begin() + max_tp1.first+3, forward_motion.end());
                // std::vector<timedPoint> avoid_remaining2(opposite_motion.begin()+op1+1, opposite_motion.end());
                std::vector<timedPoint> avoid_remaining1(forward_motion.begin() + end_idx1 + 1, forward_motion.end());
                std::vector<timedPoint> avoid_remaining2(opposite_motion.begin() + end_idx2 + 1, opposite_motion.end());

                for (std::size_t i = 0; i < avoid_remaining1.size(); ++i)
                    avoid_remaining1[i].t -= avoid_remaining1.front().t;
                for (std::size_t i = 0; i < avoid_remaining2.size(); ++i)
                    avoid_remaining2[i].t -= avoid_remaining2.front().t;

                // Notice:
                // I believe that separating 'avoid_traj' from 'avoid_remaining' is more feasible.
                // Additionally, generating 'uturn_traj' is unnecessary when the robot can avoid the obstacle.
                // Therefore, although I have written some code for the 'uturn' motion, it has not been tested yet.

                // auto uturn_traj = generatePPTrajectory(uturn_wps, 10, 0.002, 0.1, 0.6, 2.5, 0.1, config.agent_size*0.5);
                // collision intervals between type 4 and type 5
                // std::pair<double, double> uturn_interval1 = std::make_pair(0.0, uturn_traj.back().t);
                // std::pair<double, double> uturn_interval2 = std::make_pair(opposite_motion[op3-1].t - opposite_motion[op2-1].t, opposite_motion[op3+2].t - opposite_motion[op2-1].t);

                // std::vector<timedPoint> uturn_remaining(opposite_motion.begin() + op3+1, opposite_motion.end());
                // offset = uturn_traj.back().t + (uturn_traj.back().position-uturn_remaining.front().position).norm()/config.trVel;
                // for(std::size_t i=0; i < uturn_remaining.size(); ++i)
                // {
                //     uturn_traj.emplace_back(uturn_remaining[i].position(0), uturn_remaining[i].position(1), uturn_remaining[i].t+offset);
                // }

                if (start_idx1 < 1 or end_idx1 >= forward_motion.size() - 1)
                {
                    std::cout << "Obstacle is too close to the edge. Case 1" << std::endl;
                    for (auto &motion : inter_motion.at({ent, ext}).paths)
                    {
                        if (motion.t > 0.0)
                            motion.t = CN_INFINITY;
                    }
                    for (auto &motion : forward_motion)
                    {
                        if (motion.t > 0.0)
                            motion.t = CN_INFINITY;
                    }
                    inter_motion.at({ent, ext}).time_cost = CN_INFINITY;

                    ObsEdgeInfo info(id, ent, ext, 1, edge.second);
                }
                else
                {
                    std::cout << "There is room to avoid this obstacle in the lane. Case 2" << std::endl;
                    ObsEdgeInfo info(id, ent, ext, 2, edge.second);

                    std::copy(forward_motion.begin(), forward_motion.begin() + start_idx1, std::back_inserter(info.entry_traj.first));
                    std::copy(avoid_traj.begin(), avoid_traj.end(), std::back_inserter(info.avoid_traj.first));
                    std::copy(avoid_remaining1.begin(), avoid_remaining1.end(), std::back_inserter(info.exit_traj.first));
                    info.avoid_interval.first = avoid_interval1;

                    std::copy(opposite_motion.begin(), opposite_motion.begin() + start_idx2, std::back_inserter(info.entry_traj.second));
                    std::copy(avoid_traj2.begin(), avoid_traj2.end(), std::back_inserter(info.avoid_traj.second));
                    std::copy(avoid_remaining2.begin(), avoid_remaining2.end(), std::back_inserter(info.exit_traj.second));
                    info.avoid_interval.second = avoid_interval2;

                    map.add_obs_edge_info(info);
                }
            }
        }
        else if (!collide1 and collide2) // lane2 is blocked by obstacle
        {
            std::cout << "Obstacle partially blocks one lane: [" << map.get_obstacles().size() << ", " << ext << ", " << ent << ", " << min_tp2.second.t << ", " << max_tp2.second.t << "]" << std::endl;
            if (min_tp2.second.t < config.safe_time or opposite_motion.back().t - max_tp2.second.t < config.safe_time)
            {
                std::cout << "There is no room to avoid this obstacle in the lane. Case 3" << std::endl;
                for (auto &motion : inter_motion.at({ext, ent}).paths)
                {
                    if (motion.t > 0.0)
                        motion.t = CN_INFINITY;
                }
                for (auto &motion : opposite_motion)
                {
                    if (motion.t > 0.0)
                        motion.t = CN_INFINITY;
                }
                inter_motion.at({ext, ent}).time_cost = CN_INFINITY;

                ObsEdgeInfo info(id, ext, ent, 3, edge.second);
            }
            else
            {
                int fp1 = -1;
                int fp2 = -1;
                int fp3 = -1;
                double minDist1 = CN_INFINITY;
                double minDist2 = CN_INFINITY;
                double minDist3 = CN_INFINITY;
                int start_idx1 = min_tp2.first - 1;
                int end_idx1 = max_tp2.first + 1;
                while (opposite_motion[min_tp2.first].t - opposite_motion[start_idx1].t < config.safe_time)
                    --start_idx1;
                while (opposite_motion[end_idx1].t - opposite_motion[max_tp2.first].t < config.safe_time)
                    ++end_idx1;
                Eigen::Vector2d v1 = opposite_motion[min_tp2.first - 1].position - opposite_motion[min_tp2.first - 2].position;
                Eigen::Vector2d v2 = opposite_motion[max_tp2.first + 3].position - opposite_motion[max_tp2.first + 2].position;
                for (int fm = 0; fm < forward_motion.size(); ++fm)
                {
                    auto fp = forward_motion[fm].position;
                    Eigen::Vector2d v3 = fp - opposite_motion[min_tp2.first - 2].position;
                    Eigen::Vector2d v4 = fp - opposite_motion[max_tp2.first + 2].position;
                    if (v1.dot(v3) > 0 and v3.norm() < minDist1)
                    {
                        fp1 = fm;
                        minDist1 = v3.norm();
                    }
                    if (v2.dot(v4) < 0 and v4.norm() < minDist2)
                    {
                        fp2 = fm;
                        minDist2 = v4.norm();
                    }
                    if (v1.dot(v3) < 0 and v3.norm() < minDist3)
                    {
                        fp3 = fm;
                        minDist3 = v3.norm();
                    }
                }
                // Interpolate and generate trajectory
                //  avoidance waypoints
                //  [ opposite[min_tp2.first-3], opposite[min_tp2.first-2], forward[fp1] ~ forward[fp2], opposite[max_tp2.first+2], opposite[max_tp2.first+3] ]
                //
                //  u-turn waypoints
                //  [ opposite[min_tp1.first-3], opposite[min_tp1.first-2], forward[op3], forward[op3+1] ]

                std::vector<Eigen::Vector2d> avoid_wps, uturn_wps;

                // avoid_wps.push_back(opposite_motion[min_tp2.first-3].position);
                // avoid_wps.push_back(opposite_motion[min_tp2.first-2].position);
                avoid_wps.push_back(opposite_motion[start_idx1 - 1].position);
                avoid_wps.push_back(opposite_motion[start_idx1].position);
                uturn_wps = avoid_wps;
                for (int i = 0; i <= fp1 - fp2; ++i)
                {
                    avoid_wps.push_back(forward_motion[fp1 - i].position);
                }
                // avoid_wps.push_back(opposite_motion[max_tp2.first+2].position);
                // avoid_wps.push_back(opposite_motion[max_tp2.first+3].position);
                avoid_wps.push_back(opposite_motion[end_idx1].position);
                avoid_wps.push_back(opposite_motion[end_idx1 + 1].position);

                uturn_wps.push_back(forward_motion[fp3].position);
                uturn_wps.push_back(forward_motion[fp3 + 1].position);

                auto avoid_traj = generatePPTrajectory(avoid_wps, 10, 0.002, 0.1, config.trVel, 2.5, 0.1, config.agent_size * 0.5);
                int start_idx2 = fp2 - 1;
                int end_idx2 = fp1 + 1;
                while (forward_motion[fp2].t - forward_motion[start_idx2].t < config.safe_time and start_idx2 - 2 >= 0)
                    --start_idx2;
                while (forward_motion[end_idx2].t - forward_motion[fp1].t < config.safe_time and end_idx2 + 2 < forward_motion.size())
                    ++end_idx2;
                // std::vector<timedPoint> avoid_traj2(forward_motion.begin()+fp2-1, forward_motion.begin()+fp1+2);
                std::vector<timedPoint> avoid_traj2(forward_motion.begin() + start_idx2 - 1, forward_motion.begin() + end_idx2 + 2);
                double offset = avoid_traj2.front().t;
                for (std::size_t i = 0; i < avoid_traj2.size(); ++i)
                    avoid_traj2[i].t -= offset;

                // collision intervals between type 3 and type 5
                std::pair<double, double> avoid_interval1 = std::make_pair(0.0, avoid_traj.back().t);
                std::pair<double, double> avoid_interval2 = std::make_pair(0.0, avoid_traj2.back().t);

                // std::vector<timedPoint> avoid_remaining1(opposite_motion.begin() +max_tp2.first+3, opposite_motion.end());
                // std::vector<timedPoint> avoid_remaining2(forward_motion.begin()+fp1+1, forward_motion.end());
                std::vector<timedPoint> avoid_remaining1(opposite_motion.begin() + end_idx1 + 1, opposite_motion.end());
                std::vector<timedPoint> avoid_remaining2(forward_motion.begin() + end_idx2 + 1, forward_motion.end());

                for (std::size_t i = 0; i < avoid_remaining1.size(); ++i)
                    avoid_remaining1[i].t -= avoid_remaining1.front().t;
                for (std::size_t i = 0; i < avoid_remaining2.size(); ++i)
                    avoid_remaining2[i].t -= avoid_remaining2.front().t;

                // auto uturn_traj = generatePPTrajectory(uturn_wps, 10, 0.002, 0.1, config.trVel, 2.5, 0.1, config.agent_size*0.5);
                // //collision intervals between type 4 and type 5
                // std::pair<double, double> uturn_interval1 = std::make_pair(0.0, uturn_traj.back().t);
                // std::pair<double, double> uturn_interval2 = std::make_pair(forward_motion[fp3-1].t - forward_motion[fp2-1].t, forward_motion[fp3+2].t - forward_motion[fp2-1].t);

                // std::vector<timedPoint> uturn_remaining(forward_motion.begin() + fp3+1, forward_motion.end());
                // offset = uturn_traj.back().t + (uturn_traj.back().position-uturn_remaining.front().position).norm()/config.trVel;
                // for(std::size_t i=0; i < uturn_remaining.size(); ++i)
                // {
                //     uturn_traj.emplace_back(uturn_remaining[i].position(0), uturn_remaining[i].position(1), uturn_remaining[i].t+offset);
                // }

                if (start_idx1 < 1 or end_idx1 >= opposite_motion.size() - 1)
                {
                    std::cout << "Obstacle is too close to the edge. Case 3" << std::endl;
                    for (auto &motion : inter_motion.at({ent, ext}).paths)
                    {
                        if (motion.t > 0.0)
                            motion.t = CN_INFINITY;
                    }
                    for (auto &motion : forward_motion)
                    {
                        if (motion.t > 0.0)
                            motion.t = CN_INFINITY;
                    }
                    inter_motion.at({ent, ext}).time_cost = CN_INFINITY;

                    ObsEdgeInfo info(id, ext, ent, 3, edge.second);
                }
                else
                {
                    std::cout << "There is room to avoid this obstacle in the lane. Case 4" << std::endl;
                    ObsEdgeInfo info(id, ext, ent, 4, edge.second);

                    std::copy(opposite_motion.begin(), opposite_motion.begin() + start_idx1, std::back_inserter(info.entry_traj.first));
                    std::copy(avoid_traj.begin(), avoid_traj.end(), std::back_inserter(info.avoid_traj.first));
                    std::copy(avoid_remaining1.begin(), avoid_remaining1.end(), std::back_inserter(info.exit_traj.first));
                    info.avoid_interval.first = avoid_interval1;

                    std::copy(forward_motion.begin(), forward_motion.begin() + start_idx2, std::back_inserter(info.entry_traj.second));
                    std::copy(avoid_traj2.begin(), avoid_traj2.end(), std::back_inserter(info.avoid_traj.second));
                    std::copy(avoid_remaining2.begin(), avoid_remaining2.end(), std::back_inserter(info.exit_traj.second));
                    info.avoid_interval.second = avoid_interval2;

                    map.add_obs_edge_info(info);
                }
            }
        }
        else if (collide1 and collide2) // both lanes are blocked by obstacle
        {
            std::cout << "Obstacle blocks both lanes: [" << map.get_obstacles().size() << ", " << ent << " <--> " << ext << ", ("
                      << min_tp1.second.t << ", " << max_tp1.second.t << "), (" << min_tp2.second.t << ", " << max_tp2.second.t << ")]" << std::endl;
            if (min_tp1.second.t < config.safe_time or forward_motion.back().t - max_tp1.second.t < config.safe_time or min_tp2.second.t < config.safe_time or opposite_motion.back().t - max_tp2.second.t < config.safe_time) // 양 방향 중 하나라도 유턴 불가능
            {
                std::cout << "There is no room to uturn for the robot in the lanes. Case 5" << std::endl;

                for (auto &motion : inter_motion.at({ent, ext}).paths)
                {
                    if (motion.t > 0.0)
                        motion.t = CN_INFINITY;
                }
                for (auto &motion : forward_motion)
                {
                    if (motion.t > 0.0)
                        motion.t = CN_INFINITY;
                }
                inter_motion.at({ent, ext}).time_cost = CN_INFINITY;

                for (auto &motion : inter_motion.at({ext, ent}).paths)
                {
                    if (motion.t > 0.0)
                        motion.t = CN_INFINITY;
                }
                for (auto &motion : opposite_motion)
                {
                    if (motion.t > 0.0)
                        motion.t = CN_INFINITY;
                }
                inter_motion.at({ext, ent}).time_cost = CN_INFINITY;

                ObsEdgeInfo info(id, ent, ext, 5, edge.second);
                map.add_obs_edge_info(info);
            }
            else // 양 방향 모두 유턴 가능
            {
                int fp1 = -1;
                int op1 = -1;
                double minDist1 = CN_INFINITY;
                double minDist2 = CN_INFINITY;

                int start_idx1 = min_tp1.first - 1;
                int start_idx2 = min_tp2.first - 1;
                while (forward_motion[min_tp1.first].t - forward_motion[start_idx1].t < config.safe_time)
                    --start_idx1;
                while (opposite_motion[min_tp2.first].t - opposite_motion[start_idx2].t < config.safe_time)
                    --start_idx2;

                Eigen::Vector2d v1 = forward_motion[min_tp1.first - 1].position - forward_motion[min_tp1.first - 2].position;
                Eigen::Vector2d v2 = opposite_motion[min_tp2.first - 1].position - opposite_motion[min_tp2.first - 2].position;
                for (int om = 0; om < opposite_motion.size(); ++om)
                {
                    auto op = opposite_motion[om].position;
                    Eigen::Vector2d v3 = op - forward_motion[min_tp1.first - 2].position;
                    if (v1.dot(v3) < 0 and v3.norm() < minDist1)
                    {
                        op1 = om;
                        minDist1 = v3.norm();
                    }
                }
                for (int fm = 0; fm < forward_motion.size(); ++fm)
                {
                    auto fp = forward_motion[fm].position;
                    Eigen::Vector2d v4 = fp - opposite_motion[min_tp2.first - 2].position;
                    if (v2.dot(v4) < 0 and v4.norm() < minDist2)
                    {
                        fp1 = fm;
                        minDist2 = v4.norm();
                    }
                }
                std::vector<Eigen::Vector2d> uturn_wps1, uturn_wps2;
                // uturn_wps1.push_back(forward_motion[min_tp1.first-3].position);
                // uturn_wps1.push_back(forward_motion[min_tp1.first-2].position);
                uturn_wps1.push_back(forward_motion[start_idx1 - 1].position);
                uturn_wps1.push_back(forward_motion[start_idx1].position);
                uturn_wps1.push_back(opposite_motion[op1].position);
                uturn_wps1.push_back(opposite_motion[op1 + 1].position);

                // uturn_wps2.push_back(opposite_motion[min_tp2.first-3].position);
                // uturn_wps2.push_back(opposite_motion[min_tp2.first-2].position);
                uturn_wps2.push_back(opposite_motion[start_idx2 - 1].position);
                uturn_wps2.push_back(opposite_motion[start_idx2].position);
                uturn_wps2.push_back(forward_motion[fp1].position);
                uturn_wps2.push_back(forward_motion[fp1 + 1].position);

                std::vector<timedPoint> uturn_remaining1(opposite_motion.begin() + op1 + 1, opposite_motion.end());
                std::vector<timedPoint> uturn_remaining2(forward_motion.begin() + fp1 + 1, forward_motion.end());
                auto uturn_traj1 = generatePPTrajectory(uturn_wps1, 10, 0.002, 0.1, config.trVel, 2.5, 0.1, config.agent_size * 0.5);
                double offset = uturn_traj1.back().t - uturn_remaining1.front().t;
                for (std::size_t i = 1; i < uturn_remaining1.size(); ++i)
                {
                    uturn_traj1.emplace_back(uturn_remaining1[i].position(0), uturn_remaining1[i].position(1), uturn_remaining1[i].t + offset);
                }

                auto uturn_traj2 = generatePPTrajectory(uturn_wps2, 10, 0.002, 0.1, config.trVel, 2.5, 0.1, config.agent_size * 0.5);
                offset = uturn_traj2.back().t - uturn_remaining2.front().t;
                for (std::size_t i = 1; i < uturn_remaining2.size(); ++i)
                {
                    uturn_traj2.emplace_back(uturn_remaining2[i].position(0), uturn_remaining2[i].position(1), uturn_remaining2[i].t + offset);
                }

                ObsEdgeInfo info(id, ent, ext, 6, edge.second);

                std::copy(forward_motion.begin(), forward_motion.begin() + start_idx1, std::back_inserter(info.entry_traj.first));
                std::copy(uturn_traj1.begin(), uturn_traj1.end(), std::back_inserter(info.uturn_traj.first));

                std::copy(opposite_motion.begin(), opposite_motion.begin() + start_idx2, std::back_inserter(info.entry_traj.second));
                std::copy(uturn_traj2.begin(), uturn_traj2.end(), std::back_inserter(info.uturn_traj.second));

                map.add_obs_edge_info(info);
            }
        }
        id++;
    }
}
// After changing obs_type -------------------------------------------------------------------------

std::vector<Eigen::Vector2d> CBS::interpolatePoints(const std::vector<Eigen::Vector2d> &pts, int num_additional_pts)
{
    // Interpolates the given waypoint vector
    std::vector<Eigen::Vector2d> interpPts;

    if (pts.size() < 2)
    {
        return pts;
    }
    for (std::size_t i = 0; i < pts.size() - 1; ++i)
    {
        const auto &PA = pts[i];
        const auto &PB = pts[i + 1];
        for (int j = 0; j <= num_additional_pts; ++j)
        {
            double t = j / static_cast<double>(num_additional_pts);
            Eigen::Vector2d P = (1 - t) * PA + t * PB;
            interpPts.push_back(P);
        }
        if (i < pts.size() - 2)
        {
            interpPts.pop_back(); // avoid duplicating the last point of each segment except for the final;
        }
    }
    return interpPts;
}

Eigen::MatrixXd CBS::generalizedBezierPoints(const std::vector<Eigen::Vector2d> &pts, double step)
{
    int n = pts.size() - 1;
    std::vector<double> sigma(pts.size());

    auto factorial = [&](int n)
    {
        double result = 1;
        for (int i = 1; i <= n; ++i)
        {
            result *= i;
        }
        return result;
    };

    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        sigma[i] = factorial(n) / (factorial(i) * factorial(n - i));
    }

    std::vector<Eigen::VectorXd> UB;
    for (double u = 0; u <= 1; u += step)
    {
        Eigen::VectorXd UB_row(n + 1);
        for (int d = 1; d <= n + 1; ++d)
        {
            UB_row(d - 1) = sigma[d - 1] * std::pow(1 - u, n + 1 - d) * std::pow(u, d - 1);
        }
        UB.push_back(UB_row);
    }
    Eigen::MatrixXd l(UB.size(), n + 1);
    for (std::size_t i = 0; i < UB.size(); ++i)
    {
        l.row(i) = UB[i];
    }

    Eigen::MatrixXd ptsMatrix(pts.size(), 2);
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        ptsMatrix.row(i) = pts[i];
    }
    Eigen::MatrixXd c = l * ptsMatrix;

    return c;
}

std::vector<timedPoint> CBS::generatePPTrajectory(const std::vector<Eigen::Vector2d> &pts, int interp_num, double bezier_step, double v_min, double v_ref, double w_max, double dt, double lookahead_dist)
{
    std::vector<timedPoint> traj;

    // auto interpPts = interpolatePoints(pts, interp_num);
    auto interpPts = pts;
    // std::cout<<"interPts"<<std::endl;
    // for(auto ip: interpPts)
    // {
    //     std::cout<<ip(0)<<","<<ip(1)<<std::endl;
    // }
    Eigen::MatrixXd path = generalizedBezierPoints(interpPts, bezier_step);

    if (path.rows() < 2)
    {
        std::cout << "Invalid path size (path.rows() is smaller than 2)" << std::endl;
        return traj;
    }

    double th_i = std::atan2(path(1, 1) - path(0, 1), path(1, 0) - path(0, 0));
    Eigen::Vector3d curr_p(path(0, 0), path(0, 1), th_i);
    double dist2goal = (path.row(path.rows() - 1) - path.row(0)).norm();
    int idx = 1;
    double curr_t = 0.0;
    traj.emplace_back(curr_p(0), curr_p(1), curr_t);

    double curr_v = v_ref;
    while (dist2goal > curr_v * dt)
    {
        Eigen::Vector2d pp(path.row(idx)(0), path.row(idx)(1));
        double nextp_dist = (pp - curr_p.head<2>()).norm();
        Eigen::Vector2d lookahead_pt;

        if (nextp_dist < lookahead_dist)
        {
            if (idx < path.rows() - 1)
            {
                idx++;
                continue;
            }
            else
            {
                lookahead_pt = path.row(idx).head<2>();
            }
        }
        else
        {
            lookahead_pt = curr_p.head<2>() + (pp - curr_p.head<2>()) / nextp_dist * lookahead_dist;
        }
        double alpha = std::atan2(lookahead_pt(1) - curr_p(1), lookahead_pt(0) - curr_p(0)) - curr_p(2);
        double kappa = fabs(2 * std::sin(alpha) / lookahead_dist);
        if (kappa > w_max / v_ref)
        {
            curr_v = std::min(v_min, v_ref / kappa);
        }
        else
        {
            curr_v = v_ref;
        }
        double curr_w = 2 * curr_v * std::sin(alpha) / lookahead_dist;

        curr_p(0) += dt * curr_v * std::cos(curr_p(2));
        curr_p(1) += dt * curr_v * std::sin(curr_p(2));
        curr_p(2) += dt * curr_w;

        Eigen::Vector2d pp_end(path.row(path.rows() - 1).head<2>());
        dist2goal = (curr_p.head<2>() - pp_end).norm();
        curr_t += dt;
        // if(traj.empty() or (curr_p.head<2>() - traj.back().position).norm() > config.agent_size *0.5)
        //     traj.emplace_back(curr_p(0), curr_p(1), curr_t);
        traj.emplace_back(curr_p(0), curr_p(1), curr_t);
        if (curr_t > 10000)
        {
            traj.clear();
            std::cout << "Generating PP trajectory over 10000(s). Maybe the trajectory is diverged." << std::endl;
            return traj;
        }
    }

    // Modify the last position if the trajectory does not reached yet
    if ((traj.back().position - pts.back()).norm() != 0)
    {
        traj.back().position(0) = pts.back()(0);
        traj.back().position(1) = pts.back()(1);
        // Adding the last position might inadvertently cause the generation of lateral or backward motion. Therefore, we just modify the last value.
        //  traj.emplace_back(pts.back()(0), pts.back()(1), traj.back().t+(traj.back().position-pts.back()).norm()/config.trVel);
    }

    return traj;
}
