#include "sipp.h"

void SIPP::clear()
{
    open.clear();
    close.clear();
    custom_close.clear();
    collision_intervals.clear();
    intersection_intervals.clear();
    landmarks.clear();
    constraints.clear();
    move_constraints.clear();
    intersection_constraints.clear();
    cross_constraints.clear();
    visited.clear();
    custom_visited.clear();
    label_visited.clear();
    path.cost = -1;
}
void SIPP::init_predefined_info(const std::vector<Node_info> &node_information, const std::unordered_map<std::pair<int, int>, Intermotion, PairHash> &guide_inter_motion, const std::unordered_map<std::tuple<int, int, int>, Innermotion, TupleHash> &guide_inner_motion, const std::unordered_map<std::tuple<int, int, int>, double, TupleHash> &uh_val)
{
    node_info = node_information;
    inter_motion = guide_inter_motion;
    inner_motion = guide_inner_motion;
    uh_value = uh_val;
}

double SIPP::dist(const Node &a, const Node &b)
{
    return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}
double SIPP::custom_cost(const Node &a, const Node &b)
{
    // Caculate the traversal time of the robot moving from node a to node b
    if (a.id == b.id and a.type == b.type and a.orientation == b.orientation) // wait
    {
        return 0.0;
    }
    else if (a.id == b.id and a.type == 1 and b.type == 2) // intersection
    {
        return (inner_motion.find({a.id, a.orientation, b.orientation})->second.time_cost - a.offset); // add offset
    }

    // Before changing obs_type --------------------------------------------------------------------
    else if (a.type >= 2 and !obs_info.empty() and obs_info.find({a.id, node_info[a.id].roads[a.orientation].connected_wp_id}) != obs_info.end())
    {

        auto it = obs_info.find({a.id, node_info[a.id].roads[a.orientation].connected_wp_id});
        if (it->second.obs_type == 0 or it->second.obs_type == 2)
        {
            if (a.type == 2 and b.type == 3)
            {
                return (it->second.entry_traj.back().t - a.offset);
            }
            else if (a.type == 3 and b.type == 5)
            {
                return (it->second.avoid_traj.back().t - a.offset);
            }
            else if (a.type == 5 and b.type == 1)
            {
                return (it->second.exit_traj.back().t - a.offset);
            }
            else
            {
                std::cout << "Invalid case. This must be a bug, so check custom_cost for avoidance motion" << std::endl;
            }
        }
        else if (it->second.obs_type == 1)
        {
            if (a.type == 2 and b.type == 4)
            {
                return (it->second.entry_traj.back().t - a.offset);
            }
            else if (a.type == 4 and b.type == 1)
            {
                return (it->second.uturn_traj.back().t - a.offset);
            }
            else
            {
                std::cout << "Invalid case. This must be a bug, so check custom_cost for u-turn motion" << std::endl;
            }
        }
        // auto it = obs_info.find({a.id, node_info[a.id].roads[a.orientation].connected_wp_id});
        // if(it == obs_info.end() or it->second.obs_type != 1)
        //     std::cout<<"this can't happen, since u-turn is not allowed unless the robot is in the blocked corridor"<<std::endl;
        // u-turn cost = offset + 90+90 turn
        // return u-turn costs;
    }
    // Before changing obs_type --------------------------------------------------------------------

    // After changing obs_type ---------------------------------------------------------------------

    // After changing obs_type ---------------------------------------------------------------------
    else
    {
        double cost = inter_motion.find({a.id, b.id})->second.time_cost - a.offset;
        if (node_info[a.id].type == 0) // robot should turn 180 deg after exiting endpoint
            cost += M_PI / CN_ROTVEL;
        return cost;
    }
}
int SIPP::get_direction(const Node &c, const Node &n)
{
    int dir = -1;
    if (node_info[c.id].type == 0)
        return 0;
    const auto road_info = node_info[c.id].roads;
    for (int i = 0; i < int(road_info.size()); i++)
    {
        if (road_info[i].connected_wp_id == n.id)
        {
            return i;
        }
    }
    if (dir == -1)
    {
        std::cout << "get direction can't classify exit orientation(road id)" << std::endl;
        std::cout << "current node[id i j ori]: [" << c.id << ", " << c.i << ", " << c.j << ", " << c.orientation << "] next node[id i j ori]: ["
                  << n.id << ", " << n.i << ", " << n.j << ", " << n.orientation << "] " << std::endl;
    }

    return dir;
}

void SIPP::find_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values, Node goal)
{
    Node newNode;
    std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
    for (auto move : valid_moves)
    {
        newNode.i = move.i;
        newNode.j = move.j;
        newNode.id = move.id;
        double cost = dist(curNode, newNode);
        newNode.g = curNode.g + cost;
        std::vector<std::pair<double, double>> intervals(0);
        auto colls_it = collision_intervals.find(newNode.id);
        if (colls_it != collision_intervals.end())
        {
            std::pair<double, double> interval = {0, CN_INFINITY};
            for (unsigned int i = 0; i < colls_it->second.size(); i++)
            {
                interval.second = colls_it->second[i].first;
                intervals.push_back(interval);
                interval.first = colls_it->second[i].second;
            }
            interval.second = CN_INFINITY;
            intervals.push_back(interval);
        }
        else
            intervals.push_back({0, CN_INFINITY});
        auto cons_it = constraints.find({curNode.id, newNode.id});
        int id(0);
        for (auto interval : intervals)
        {
            newNode.interval_id = id;
            id++;
            auto it = visited.find(newNode.id + newNode.interval_id * map.get_size());
            if (it != visited.end())
                if (it->second.second)
                    continue;
            if (interval.second < newNode.g)
                continue;
            if (interval.first > newNode.g)
                newNode.g = interval.first;
            if (cons_it != constraints.end())
                for (unsigned int i = 0; i < cons_it->second.size(); i++)
                    if (newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 and newNode.g - cost < cons_it->second[i].t2)
                        newNode.g = cons_it->second[i].t2 + cost;
            newNode.interval = interval;
            if (newNode.g - cost > curNode.interval.second or newNode.g > newNode.interval.second)
                continue;
            if (it != visited.end())
            {
                if (it->second.first - CN_EPSILON < newNode.g)
                    continue;
                else
                    it->second.first = newNode.g;
            }
            else
                visited.insert({newNode.id + newNode.interval_id * map.get_size(), {newNode.g, false}});
            if (goal.id == agent.goal_id) // perfect heuristic is known
                newNode.f = newNode.g + h_values.get_value(newNode.id, agent.id);
            else
            {
                double h = sqrt(pow(goal.i - newNode.i, 2) + pow(goal.j - newNode.j, 2));
                for (unsigned int i = 0; i < h_values.get_size(); i++) // differential heuristic with pivots placed to agents goals
                    h = std::max(h, fabs(h_values.get_value(newNode.id, i) - h_values.get_value(goal.id, i)));
                newNode.f = newNode.g + h;
            }
            succs.push_back(newNode);
        }
    }
}
void SIPP::update_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values, std::vector<Node> goals)
{
    Node newNode;
    std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
    for (auto move : valid_moves)
    {
        if (curNode.type == 1)
        {
            newNode.i = curNode.i;
            newNode.j = curNode.j;
            newNode.id = curNode.id; // id is identical unless robot moves through the next intersection
            newNode.type = 2;
            newNode.orientation = get_direction(curNode, move);

            int prev_id = node_info[curNode.id].roads[curNode.orientation].connected_wp_id;
            int next_id = node_info[newNode.id].roads[newNode.orientation].connected_wp_id;
            if (node_info[prev_id].type == 0 and node_info[next_id].type == 0 and node_info[curNode.id].roads.size() > 15 and next_id != goals[curNode.label].id)
                continue;
            if (curNode.orientation == newNode.orientation) // u-turn
                continue;
        }
        else
        {
            if (curNode.type == 2 and move.id != node_info[curNode.id].roads[curNode.orientation].connected_wp_id) // the exit direction is already determined during previous step(curNode.type ==1)
                continue;
            newNode.i = move.i;
            newNode.j = move.j;
            newNode.id = move.id;
            if (node_info[newNode.id].type == 0)
                newNode.type = 0;
            else
                newNode.type = 1;
            newNode.orientation = (get_direction(curNode, move)); // newNode's entry is determined by the connection with the previous node(=curNode)
        }
        double cost = custom_cost(curNode, newNode);
        newNode.g = curNode.g + cost;
        newNode.label = curNode.label;
        // we have 1) 'constraints' for non-intersection move, 2) 'intersection_intervals' for intersection wait, 3) intersection_constraints for intersection move
        std::vector<std::pair<double, double>> intervals(0);
        auto inter_colls_it = intersection_intervals.find({newNode.id, newNode.type, newNode.orientation});
        if (inter_colls_it != intersection_intervals.end())
        {
            std::pair<double, double> interval = {0, CN_INFINITY};
            for (unsigned int i = 0; i < inter_colls_it->second.size(); i++)
            {
                interval.second = inter_colls_it->second[i].first;
                intervals.push_back(interval);
                interval.first = inter_colls_it->second[i].second;
            }
            interval.second = CN_INFINITY;
            intervals.push_back(interval);
        }
        else
            intervals.push_back({0, CN_INFINITY});
        int id(0);
        for (auto interval : intervals)
        {
            newNode.interval_id = id;
            id++;
            auto it = label_visited.find({newNode.id + newNode.interval_id * map.get_size(), newNode.type * map.get_size() + newNode.orientation, newNode.label});
            if (it != label_visited.end())
                if (it->second.second)
                    continue;
            if (interval.second < newNode.g)
                continue;
            if (interval.first > newNode.g)
                newNode.g = interval.first;
            if (curNode.type == 1 and newNode.type == 2)
            {
                auto cons_it = intersection_constraints.find({curNode.id, curNode.orientation, newNode.orientation});
                if (cons_it != intersection_constraints.end())
                {
                    for (unsigned int i = 0; i < cons_it->second.size(); i++)
                    {
                        if (newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 and newNode.g - cost < cons_it->second[i].t2)
                            newNode.g = cons_it->second[i].t2 + cost;
                    }
                }
            }
            else
            {
                auto cons_it = move_constraints.find({curNode.id, curNode.type, newNode.id, newNode.type});
                if (cons_it != move_constraints.end())
                    for (unsigned int i = 0; i < cons_it->second.size(); i++)
                        if (newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 and newNode.g - cost < cons_it->second[i].t2)
                            newNode.g = cons_it->second[i].t2 + cost;
            }
            newNode.interval = interval;
            if (newNode.g - cost > curNode.interval.second or newNode.g > newNode.interval.second)
                continue;
            if (it != label_visited.end())
            {
                if (it->second.first - CN_EPSILON < newNode.g)
                    continue;
                else
                    it->second.first = newNode.g;
            }
            else
            {
                label_visited.insert({{newNode.id + newNode.interval_id * map.get_size(), newNode.type * map.get_size() + newNode.orientation, newNode.label}, {newNode.g, false}});
            }
            newNode.f = newNode.g + h_values.uvalue()->at({newNode.id, newNode.type * map.get_node_size() + newNode.orientation, goals[newNode.label].id});
            for (std::size_t goal_id = newNode.label; goal_id < goals.size() - 1; ++goal_id)
            {
                newNode.f += h_values.uvalue()->at({goals[goal_id].id, 0, goals[goal_id + 1].id});
            }
            succs.push_back(newNode);
        }
    }
}
void SIPP::replan_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values, std::vector<Node> goals)
{
    Node newNode;
    std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);

    for (auto move : valid_moves)
    {
        if (curNode.type == 1)
        {
            newNode = curNode;
            newNode.type = 2;
            newNode.orientation = get_direction(curNode, move);
            int prev_id = node_info[curNode.id].roads[curNode.orientation].connected_wp_id;
            int next_id = node_info[newNode.id].roads[newNode.orientation].connected_wp_id;
            if (node_info[next_id].type == 0 and next_id != goals[curNode.label].id)
            {
                //"Exception for type0 -> type0"
                continue;
            }
            if (curNode.orientation == newNode.orientation) // prevent only u-turn
            {
                //"Exception for same orientation"
                continue;
            }
            if (curNode.parent == nullptr and agent.initial_id != -1 and newNode.id == agent.initial_id)
            {
                if (agent.initial_orientation != -1 and newNode.orientation != agent.initial_orientation)
                {
                    //"Exception for initial orientaion (fix robot's initial orientation at the beginning)"
                    continue;
                }
            }
        }
        else
        {
            if (curNode.type >= 2)
                if (move.id != node_info[curNode.id].roads[curNode.orientation].connected_wp_id)
                    continue;

            // Before changing obs_type ------------------------------------------------------------
            auto obs_it = obs_info.find({curNode.id, move.id});
            if (obs_it != obs_info.end())
            {
                if (obs_it->second.obs_type == 0 or obs_it->second.obs_type == 2)
                {
                    if (curNode.type == 2)
                    {
                        newNode = curNode;
                        newNode.offset = 0.0;
                        newNode.type = 3;
                    }
                    else if (curNode.type == 3)
                    {
                        newNode = curNode;
                        newNode.offset = 0.0;
                        newNode.type = 5;
                    }
                    else if (curNode.type == 5)
                    {
                        newNode = move;
                        newNode.orientation = get_direction(move, curNode);
                        newNode.type = 1;
                    }
                    else
                    {
                        std::cout << "Invalid node.type in graph search." << std::endl;
                    }
                }
                else if (obs_it->second.obs_type == 1) // the robot has already entered a blocked corridor (when the robot is the first observer of the obstacle)
                {
                    newNode = curNode;
                    newNode.offset = 0.0;
                    if (curNode.type == 2)
                    {
                        newNode.type = 4;
                    }
                    else if (curNode.type == 4)
                    {
                        newNode.type = 1; // orientation is reserved since it u-turn
                    }
                }
            }
            // Before changing obs_type ------------------------------------------------------------

            // After changing obs_type -------------------------------------------------------------
            // auto obs_edge_it = obs_edge_info.find({curNode.id, move.id});
            // if(obs_edge_it != obs_edge_info.end())
            // {
            //     if(obs_edge_it->second.edge_type == 1) // lane 1 is blocked and unavoidable -> local repair
            //     {
            //         if(curNode.type == 2)
            //         {
            //             newNode = curNode;
            //             newNode.offset = 0.0;
            //             newNode.type = 3;
            //         }
            //         else if(curNode.type == 3)
            //         {
            //             newNode = curNode;
            //             newNode.offset = 0.0;
            //             newNode.type = 5;
            //         }
            //         else if(curNode.type == 5)
            //         {
            //             newNode = move;
            //             newNode.orientation = get_direction(curNode, move);
            //             newNode.type = 1;
            //         }
            //         else
            //         {
            //             std::cout<<"Invalid node.type in graph search."<<std::endl;
            //         }
            //     }
            //     else if(obs_edge_it->second.edge_type == 2) // lane 1 is blocked and avoidable -> avoid motion
            //     {
            //         if(curNode.type == 2)
            //         {
            //             newNode = curNode;
            //             newNode.offset = 0.0;
            //             newNode.type = 3;
            //         }
            //         else if(curNode.type == 3)
            //         {
            //             newNode = curNode;
            //             newNode.offset = 0.0;
            //             newNode.type = 5;
            //         }
            //         else if(curNode.type == 5)
            //         {
            //             newNode = move;
            //             newNode.orientation = get_direction(curNode, move);
            //             newNode.type = 1;
            //         }
            //         else
            //         {
            //             std::cout<<"Invalid node.type in graph search."<<std::endl;
            //         }
            //     }
            //     else if(obs_it->second.obs_type == 1)    //the robot has already entered a blocked corridor (when the robot is the first observer of the obstacle)
            //     {
            //         newNode = curNode;
            //         newNode.offset = 0.0;
            //         if(curNode.type == 2)
            //         {
            //             newNode.type = 4;
            //         }
            //         else if(curNode.type == 4)
            //         {
            //             newNode.type = 1;   //orientation is reserved since it u-turn
            //         }
            //     }
            // }
            // After changing obs_type -------------------------------------------------------------
            else
            {
                newNode = move;
                if (node_info[newNode.id].type == 0)
                    newNode.type = 0;
                else
                    newNode.type = 1;

                newNode.orientation = get_direction(move, curNode);
            }
        }
        double cost = custom_cost(curNode, newNode);
        newNode.g = curNode.g + cost;
        newNode.label = curNode.label;

        std::vector<std::pair<double, double>> intervals(0);
        auto inter_colls_it = intersection_intervals.find({newNode.id, newNode.type, newNode.orientation});
        if (inter_colls_it != intersection_intervals.end())
        {
            // std::pair<double, double> interval = {0, CN_INFINITY};
            std::pair<double, double> interval = {start_time, CN_INFINITY};
            for (std::size_t i = 0; i < inter_colls_it->second.size(); ++i)
            {
                interval.second = inter_colls_it->second[i].first;
                intervals.push_back(interval);
                interval.first = inter_colls_it->second[i].second;
            }
            interval.second = CN_INFINITY;
            intervals.push_back(interval);
        }
        else
            intervals.push_back({start_time, CN_INFINITY});
        // intervals.push_back({0, CN_INFINITY});

        int id(0);

        for (auto interval : intervals)
        {
            newNode.interval_id = id;
            id++;
            auto it = label_visited.find({newNode.id + newNode.interval_id * map.get_size(), newNode.type * map.get_size() + newNode.orientation, newNode.label});
            if (it != label_visited.end())
            {
                if (it->second.second)
                {
                    //"Already visited"
                    continue;
                }
            }

            if (interval.second < newNode.g)
            {
                //"A robot can't reach within the safe interval"
                continue;
            }
            if (interval.first > newNode.g)
            {
                //"Update g-value (wait for a while on curNode)"
                newNode.g = interval.first;
            }
            if (curNode.type == 1 and newNode.type == 2)
            {
                auto cons_it = intersection_constraints.find({curNode.id, curNode.orientation, newNode.orientation});
                if (cons_it != intersection_constraints.end())
                {
                    for (std::size_t i = 0; i < cons_it->second.size(); ++i)
                    {
                        if (newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 and newNode.g - cost < cons_it->second[i].t2)
                        {
                            //"Update g-value (wait for a while on curNode)"
                            newNode.g = cons_it->second[i].t2 + cost;
                        }
                    }
                }
            }
            else
            {
                auto cons_it = move_constraints.find({curNode.id, curNode.type, newNode.id, newNode.type});
                if (cons_it != move_constraints.end())
                {
                    for (std::size_t i = 0; i < cons_it->second.size(); ++i)
                    {
                        if (newNode.g - cost + CN_EPSILON > cons_it->second[i].t1 and newNode.g - cost < cons_it->second[i].t2)
                        {
                            //"Update g-value (wait for a while on curNode)"
                            newNode.g = cons_it->second[i].t2 + cost;
                        }
                    }
                }
            }
            newNode.interval = interval;
            if (newNode.g - cost > curNode.interval.second + 0.01 or newNode.g + 0.01 > newNode.interval.second)
            {
                continue;
            }
            if (curNode.parent == nullptr and curNode.g < 0 and newNode.g - cost > curNode.g + CN_EPSILON)
            {
                //"We prevent a robot from waiting in the start position when replanning"
                continue;
            }
            if (it != label_visited.end())
            {
                if (it->second.first - CN_EPSILON < newNode.g)
                {
                    continue;
                }
                else
                    it->second.first = newNode.g;
            }
            else
            {
                label_visited.insert({{newNode.id + newNode.interval_id * map.get_size(), newNode.type * map.get_size() + newNode.orientation, newNode.label}, {newNode.g, false}});
            }
            if (newNode.type > 2)
            {
                int check_next_id = node_info[newNode.id].roads[newNode.orientation].connected_wp_id;
                auto check_obs_it = obs_info.find({newNode.id, check_next_id});
                newNode.f = newNode.g + h_values.uvalue()->at({newNode.id, 2 * map.get_node_size() + newNode.orientation, goals[newNode.label].id});
            }
            else
            {
                newNode.f = newNode.g + h_values.uvalue()->at({newNode.id, newNode.type * map.get_size() + newNode.orientation, goals[newNode.label].id});
            }
            for (int goal_id = newNode.label; goal_id < goals.size() - 1; ++goal_id)
            {
                newNode.f += h_values.uvalue()->at({goals[goal_id].id, 0, goals[goal_id + 1].id});
            }
            succs.push_back(newNode);
        }
    }
}

Node SIPP::find_min()
{
    Node min = *open.begin();
    open.pop_front();
    return min;
}

void SIPP::add_open(Node newNode)
{
    if (open.empty() or open.back().f - CN_EPSILON < newNode.f)
    {
        open.push_back(newNode);
        return;
    }
    for (auto iter = open.begin(); iter != open.end(); ++iter)
    {
        if (iter->f > newNode.f + CN_EPSILON) // if newNode.f has lower f-value
        {
            open.insert(iter, newNode);
            return;
        }
        else if (fabs(iter->f - newNode.f) < CN_EPSILON and newNode.g + CN_EPSILON > iter->g) // if f-values are equal, compare g-values
        {
            open.insert(iter, newNode);
            return;
        }
    }
    open.push_back(newNode);
    return;
}

std::vector<Node> SIPP::reconstruct_path(Node curNode)
{
    path.nodes.clear();
    if (curNode.parent != nullptr)
        do
        {
            path.nodes.insert(path.nodes.begin(), curNode);
            curNode = *curNode.parent;
        } while (curNode.parent != nullptr);
    path.nodes.insert(path.nodes.begin(), curNode);
    for (unsigned int i = 0; i < path.nodes.size(); i++)
    {
        unsigned int j = i + 1;
        if (j == path.nodes.size())
            break;
        if (fabs(path.nodes[j].g - path.nodes[i].g - dist(path.nodes[j], path.nodes[i])) > CN_EPSILON)
        {
            Node add = path.nodes[i];
            add.g = path.nodes[j].g - dist(path.nodes[j], path.nodes[i]);
            path.nodes.emplace(path.nodes.begin() + j, add);
        }
    }
    return path.nodes;
}
std::vector<Node> SIPP::reconstruct_custom_path(Node curNode)
{
    path.nodes.clear();
    if (curNode.parent != nullptr)
        do
        {
            path.nodes.insert(path.nodes.begin(), curNode);
            curNode = *curNode.parent;
        } while (curNode.parent != nullptr);
    path.nodes.insert(path.nodes.begin(), curNode);
    for (unsigned int i = 0; i < path.nodes.size(); i++)
    {
        unsigned int j = i + 1;
        if (j == path.nodes.size())
            break;
        if ((fabs(path.nodes[j].g - path.nodes[i].g - custom_cost(path.nodes[i], path.nodes[j]))) > CN_EPSILON)
        {
            Node add = path.nodes[i];
            add.g = path.nodes[j].g - custom_cost(path.nodes[i], path.nodes[j]);
            path.nodes.emplace(path.nodes.begin() + j, add);
        }
    }
    return path.nodes;
}

void SIPP::add_collision_interval(int id, std::pair<double, double> interval)
{
    std::vector<std::pair<double, double>> intervals(0);
    if (collision_intervals.count(id) == 0)
        collision_intervals.insert({id, {interval}});
    else
        collision_intervals[id].push_back(interval);
    std::sort(collision_intervals[id].begin(), collision_intervals[id].end());
    for (unsigned int i = 0; i + 1 < collision_intervals[id].size(); i++)
        if (collision_intervals[id][i].second + CN_EPSILON > collision_intervals[id][i + 1].first)
        {
            collision_intervals[id][i].second = collision_intervals[id][i + 1].second;
            collision_intervals[id].erase(collision_intervals[id].begin() + i + 1);
            i--;
        }
}
void SIPP::update_collision_interval(const Constraint &cons)
{
    std::vector<std::pair<double, double>> intervals(0);
    std::tuple<int, int, int> key = {cons.id1, cons.type1, cons.orientation1};

    if (intersection_intervals.count(key) == 0)
        intersection_intervals.insert({key, {std::make_pair(cons.t1, cons.t2)}});
    else
        intersection_intervals[key].push_back(std::make_pair(cons.t1, cons.t2));
    std::sort(intersection_intervals[key].begin(), intersection_intervals[key].end());
    for (unsigned int i = 0; i + 1 < intersection_intervals[key].size(); i++)
        if (intersection_intervals[key][i].second + CN_EPSILON > intersection_intervals[key][i + 1].first)
        {
            intersection_intervals[key][i].second = intersection_intervals[key][i + 1].second;
            intersection_intervals[key].erase(intersection_intervals[key].begin() + i + 1);
            i--;
        }
}

void SIPP::add_move_constraint(Move move)
{
    std::vector<Move> m_cons(0);
    if (constraints.count({move.id1, move.id2}) == 0)
        constraints.insert({{move.id1, move.id2}, {move}});
    else
    {
        m_cons = constraints.at({move.id1, move.id2});
        bool inserted(false);
        for (unsigned int i = 0; i < m_cons.size(); i++)
        {
            if (inserted)
                break;
            if (m_cons[i].t1 > move.t1)
            {
                if (m_cons[i].t1 < move.t2 + CN_EPSILON)
                {
                    m_cons[i].t1 = move.t1;
                    if (move.t2 + CN_EPSILON > m_cons[i].t2)
                        m_cons[i].t2 = move.t2;
                    inserted = true;
                    if (i != 0)
                        if (m_cons[i - 1].t2 + CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i - 1].t2 = move.t2;
                            if (m_cons[i - 1].t2 + CN_EPSILON > m_cons[i].t1 and m_cons[i - 1].t2 < m_cons[i].t2 + CN_EPSILON)
                            {
                                m_cons[i - 1].t2 = m_cons[i].t2;
                                m_cons.erase(m_cons.begin() + i);
                            }
                            inserted = true;
                        }
                }
                else
                {
                    if (i != 0)
                        if (m_cons[i - 1].t2 + CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i - 1].t2 = move.t2;
                            inserted = true;
                            break;
                        }
                    m_cons.insert(m_cons.begin() + i, move);
                    inserted = true;
                }
            }
        }
        if (m_cons.back().t2 + CN_EPSILON > move.t1 and m_cons.back().t2 < move.t2 + CN_EPSILON)
            m_cons.back().t2 = move.t2;
        else if (!inserted)
            m_cons.push_back(move);
        constraints.at({move.id1, move.id2}) = m_cons;
    }
}
void SIPP::update_intersection_constraint(const Constraint &cons)
{
    std::vector<Move> m_cons(0);
    std::tuple<int, int, int> key = {cons.id1, cons.orientation1, cons.orientation2};
    Move move(cons);
    if (intersection_constraints.count(key) == 0)
        intersection_constraints.insert({key, {move}});
    else
    {
        m_cons = intersection_constraints.at(key);
        bool inserted(false);
        for (unsigned int i = 0; i < m_cons.size(); i++)
        {
            if (inserted)
                break;
            if (m_cons[i].t1 + CN_EPSILON > move.t1)
            {
                if (m_cons[i].t1 < move.t2 + CN_EPSILON)
                {
                    m_cons[i].t1 = move.t1;
                    if (move.t2 + CN_EPSILON > m_cons[i].t2)
                        m_cons[i].t2 = move.t2;
                    inserted = true;
                    if (i != 0)
                        if (m_cons[i - 1].t2 + CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i - 1].t2 = move.t2;
                            if (m_cons[i - 1].t2 + CN_EPSILON > m_cons[i].t1 and m_cons[i - 1].t2 < m_cons[i].t2 + CN_EPSILON)
                            {
                                m_cons[i - 1].t2 = m_cons[i].t2;
                                m_cons.erase(m_cons.begin() + i);
                            }
                            inserted = true;
                        }
                }
                else
                {
                    if (i != 0)
                        if (m_cons[i - 1].t2 + CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i - 1].t2 = move.t2;
                            inserted = true;
                            break;
                        }
                    m_cons.insert(m_cons.begin() + i, move);
                    inserted = true;
                }
            }
        }
        if (m_cons.back().t2 + CN_EPSILON > move.t1 and m_cons.back().t2 < move.t2 + CN_EPSILON)
            m_cons.back().t2 = move.t2;
        else if (!inserted)
            m_cons.push_back(move);
        intersection_constraints.at(key) = m_cons;
    }
}

void SIPP::update_cross_constraint(const Constraint &cons)
{
    std::vector<Move> m_cons(0);
    std::tuple<int, int, int> key = {cons.id1, cons.type1, cons.id2};
    Move move(cons);
    if (cross_constraints.count(key) == 0)
        cross_constraints.insert({key, {move}});
    else
    {
        m_cons = cross_constraints.at(key);
        bool inserted(false);
        for (unsigned int i = 0; i < m_cons.size(); i++)
        {
            if (inserted)
                break;
            if (m_cons[i].t1 + CN_EPSILON > move.t1)
            {
                if (m_cons[i].t1 < move.t2 + CN_EPSILON)
                {
                    m_cons[i].t1 = move.t1;
                    if (move.t2 + CN_EPSILON > m_cons[i].t2)
                        m_cons[i].t2 = move.t2;
                    inserted = true;
                    if (i != 0)
                        if (m_cons[i - 1].t2 + CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i - 1].t2 = move.t2;
                            if (m_cons[i - 1].t2 + CN_EPSILON > m_cons[i].t1 and m_cons[i - 1].t2 < m_cons[i].t2 + CN_EPSILON)
                            {
                                m_cons[i - 1].t2 = m_cons[i].t2;
                                m_cons.erase(m_cons.begin() + i);
                            }
                            inserted = true;
                        }
                }
                else
                {
                    if (i != 0)
                        if (m_cons[i - 1].t2 + CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i - 1].t2 = move.t2;
                            inserted = true;
                            break;
                        }
                    m_cons.insert(m_cons.begin() + i, move);
                    inserted = true;
                }
            }
        }
        if (m_cons.back().t2 + CN_EPSILON > move.t1 and m_cons.back().t2 < move.t2 + CN_EPSILON)
            m_cons.back().t2 = move.t2;
        else if (!inserted)
            m_cons.push_back(move);
        cross_constraints.at(key) = m_cons;
    }
}

void SIPP::update_move_constraint(const Constraint &cons)
{
    std::vector<Move> m_cons(0);
    std::tuple<int, int, int, int> key = {cons.id1, cons.type1, cons.id2, cons.type2};
    Move move(cons);
    if (move_constraints.count(key) == 0)
        move_constraints.insert({key, {move}});
    else
    {
        m_cons = move_constraints.at(key);
        bool inserted(false);
        for (unsigned int i = 0; i < m_cons.size(); i++)
        {
            if (inserted)
                break;
            if (m_cons[i].t1 + CN_EPSILON > move.t1)
            {
                if (m_cons[i].t1 < move.t2 + CN_EPSILON)
                {
                    m_cons[i].t1 = move.t1;
                    if (move.t2 + CN_EPSILON > m_cons[i].t2)
                        m_cons[i].t2 = move.t2;
                    inserted = true;
                    if (i != 0)
                        if (m_cons[i - 1].t2 + CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i - 1].t2 = move.t2;
                            if (m_cons[i - 1].t2 + CN_EPSILON > m_cons[i].t1 and m_cons[i - 1].t2 < m_cons[i].t2 + CN_EPSILON)
                            {
                                m_cons[i - 1].t2 = m_cons[i].t2;
                                m_cons.erase(m_cons.begin() + i);
                            }
                            inserted = true;
                        }
                }
                else
                {
                    if (i != 0)
                        if (m_cons[i - 1].t2 + CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i - 1].t2 = move.t2;
                            inserted = true;
                            break;
                        }
                    m_cons.insert(m_cons.begin() + i, move);
                    inserted = true;
                }
            }
        }
        if (m_cons.back().t2 + CN_EPSILON > move.t1 and m_cons.back().t2 < move.t2 + CN_EPSILON)
            m_cons.back().t2 = move.t2;
        else if (!inserted)
            m_cons.push_back(move);
        move_constraints.at(key) = m_cons;
    }
}

void SIPP::make_constraints(std::list<Constraint> &cons)
{
    for (auto con : cons)
    {
        if (con.positive == false)
        {
            if (con.id1 == con.id2) // wait consatraint
                add_collision_interval(con.id1, std::make_pair(con.t1, con.t2));
            else
                add_move_constraint(Move(con));
        }
        else
        {
            bool inserted = false;
            for (unsigned int i = 0; i < landmarks.size(); i++)
                if (landmarks[i].t1 > con.t1)
                {
                    landmarks.insert(landmarks.begin() + i, Move(con.t1, con.t2, con.id1, con.id2));
                    inserted = true;
                    break;
                }
            if (!inserted)
                landmarks.push_back(Move(con.t1, con.t2, con.id1, con.id2));
        }
    }
}
void SIPP::update_constraints(std::list<Constraint> &cons)
{
    for (auto con : cons)
    {
        if (con.id1 == con.id2 and con.type1 == con.type2) // intersection wait constraint (same id with different type represents intersection move)
            update_collision_interval(con);
        else if (con.id1 == con.id2 and con.type1 == 1 and con.type2 == 2) // intersection move constraint
            update_intersection_constraint(con);
        else
            update_move_constraint(con);
    }
}

Path SIPP::add_part(Path result, Path part)
{
    part.nodes.erase(part.nodes.begin());
    for (auto n : part.nodes)
        result.nodes.push_back(n);
    return result;
}

std::vector<Path> SIPP::find_partial_path(std::vector<Node> starts, std::vector<Node> goals, const Map &map, Heuristic &h_values, double max_f)
{
    open.clear();
    close.clear();
    path.cost = -1;
    visited.clear();
    std::vector<Path> paths(goals.size());
    int pathFound(0);
    for (auto s : starts)
    {
        s.parent = nullptr;
        open.push_back(s);
        visited.insert({s.id + s.interval_id * map.get_size(), {s.g, false}});
    }
    Node curNode;
    while (!open.empty())
    {
        curNode = find_min();
        auto v = visited.find(curNode.id + curNode.interval_id * map.get_size());
        if (v->second.second)
            continue;
        v->second.second = true;
        auto parent = &close.insert({curNode.id + curNode.interval_id * map.get_size(), curNode}).first->second;
        if (curNode.id == goals[0].id)
        {
            for (unsigned int i = 0; i < goals.size(); i++)
                if (curNode.g - CN_EPSILON < goals[i].interval.second and goals[i].interval.first - CN_EPSILON < curNode.interval.second)
                {
                    paths[i].nodes = reconstruct_path(curNode);
                    if (paths[i].nodes.back().g < goals[i].interval.first)
                    {
                        curNode.g = goals[i].interval.first;
                        paths[i].nodes.push_back(curNode);
                    }
                    paths[i].cost = curNode.g;
                    paths[i].expanded = int(close.size());
                    pathFound++;
                }
            if (pathFound == int(goals.size()))
                return paths;
        }
        std::list<Node> succs;
        succs.clear();
        find_successors(curNode, map, succs, h_values, Node(goals[0].id, 0, 0, goals[0].i, goals[0].j));
        std::list<Node>::iterator it = succs.begin();
        while (it != succs.end())
        {
            if (it->f > max_f)
            {
                it++;
                continue;
            }
            it->parent = parent;
            add_open(*it);
            it++;
        }
    }
    return paths;
}
std::vector<Path> SIPP::update_partial_path(std::vector<Node> starts, std::vector<Node> goals, const Map &map, Heuristic &h_values, double max_f)
{
    open.clear();
    label_close.clear();
    path.cost = -1;
    label_visited.clear(); // START CHANGE custom_visited to label_visited for Multi-Label SIPP
    std::vector<Path> paths(1);
    int pathFound(0);
    for (auto s : starts)
    {
        s.label = 0;
        s.parent = nullptr;
        s.orientation = 0; // default id
        s.type = 0;
        open.push_back(s);
        label_visited.insert({std::make_tuple(s.id + s.interval_id * map.get_size(), s.type * map.get_size() + s.orientation, s.label), {s.g, false}});
    }
    Node curNode;
    while (!open.empty())
    {
        curNode = find_min();
        auto v = label_visited.find(std::make_tuple(curNode.id + curNode.interval_id * map.get_size(), curNode.type * map.get_size() + curNode.orientation, curNode.label));
        if (v->second.second)
            continue;
        v->second.second = true;
        auto parent = &label_close.insert({{curNode.id + curNode.interval_id * map.get_size(), curNode.type * map.get_size() + curNode.orientation, curNode.label}, curNode}).first->second;
        if (curNode.id == goals[curNode.label].id)
        {
            curNode.label++;
            if (curNode.label == goals.size())
            {
                paths[0].nodes = reconstruct_custom_path(curNode);
                if (paths[0].nodes.back().g < goals.back().interval.first)
                {
                    curNode.g = goals.back().interval.first;
                    paths[0].nodes.push_back(curNode);
                }
                paths[0].cost = curNode.g;
                paths[0].expanded = int(label_close.size());
                return paths;
            }
        }
        std::list<Node> succs;
        succs.clear();
        update_successors(curNode, map, succs, h_values, goals);
        std::list<Node>::iterator it = succs.begin();
        while (it != succs.end())
        {
            if (it->f > max_f)
            {
                it++;
                continue;
            }
            it->parent = parent;
            add_open(*it);
            it++;
        }
    }
    return paths;
}
std::vector<Path> SIPP::replan_partial_path(std::vector<Node> starts, std::vector<Node> goals, const Map &map, Heuristic &h_values, double max_f)
{
    open.clear();
    label_close.clear();
    path.cost = -1;
    label_visited.clear();
    std::vector<Path> paths(1);
    start_time = 0.0;
    int pathFound(0);
    int start_interval_id(0);
    for (auto s : starts)
    {
        s.parent = nullptr;
        s.interval_id = start_interval_id;
        if (s.g < start_time)
            start_time = s.g;
        open.push_back(s);
        label_visited.insert({std::make_tuple(s.id + s.interval_id * map.get_size(), s.type * map.get_size() + s.orientation, s.label), {s.g, false}});
        ++start_interval_id;
    }
    Node curNode;
    while (!open.empty())
    {
        curNode = find_min();
        auto v = label_visited.find(std::make_tuple(curNode.id + curNode.interval_id * map.get_size(), curNode.type * map.get_size() + curNode.orientation, curNode.label));
        if (v->second.second)
            continue;
        v->second.second = true;
        auto parent = &label_close.insert({{curNode.id + curNode.interval_id * map.get_size(), curNode.type * map.get_size() + curNode.orientation, curNode.label}, curNode}).first->second;
        if (curNode.id == goals[curNode.label].id)
        {
            curNode.label++;
            if (curNode.label == goals.size())
            {
                paths[0].nodes = reconstruct_custom_path(curNode);
                if (paths[0].nodes.back().g < goals.back().interval.first)
                {
                    curNode.g = goals.back().interval.first;
                    paths[0].nodes.push_back(curNode);
                }
                paths[0].cost = curNode.g;
                paths[0].expanded = int(label_close.size());
                return paths;
            }
        }
        std::list<Node> succs;
        succs.clear();
        replan_successors(curNode, map, succs, h_values, goals);
        std::list<Node>::iterator it = succs.begin();
        while (it != succs.end())
        {
            if (it->f > max_f)
            {
                it++;
                continue;
            }
            it->parent = parent;
            add_open(*it);
            it++;
        }
    }
    std::cout << "replan_partial_path for agent[" << agent.id << "] is early stopped with paths size is " << paths.size() << std::endl;
    return paths;
}

std::vector<Node> SIPP::get_endpoints(int node_id, double node_i, double node_j, double t1, double t2)
{
    std::vector<Node> nodes;
    nodes = {Node(node_id, 0, 0, node_i, node_j, nullptr, t1, t2)};
    if (collision_intervals[node_id].empty())
        return nodes;
    else
        for (unsigned int k = 0; k < collision_intervals[node_id].size(); k++)
        {
            unsigned int i(0);
            while (i < nodes.size())
            {
                Node n = nodes[i];
                auto c = collision_intervals[node_id][k];
                bool changed = false;
                if (c.first - CN_EPSILON < n.interval.first and c.second + CN_EPSILON > n.interval.second)
                {
                    nodes.erase(nodes.begin() + i);
                    changed = true;
                }
                else if (c.first - CN_EPSILON < n.interval.first and c.second > n.interval.first)
                {
                    nodes[i].interval.first = c.second;
                    changed = true;
                }
                else if (c.first - CN_EPSILON > n.interval.first and c.second + CN_EPSILON < n.interval.second)
                {
                    nodes[i].interval.second = c.first;
                    nodes.insert(nodes.begin() + i + 1, Node(node_id, 0, 0, node_i, node_j, nullptr, c.second, n.interval.second));
                    changed = true;
                }
                else if (c.first < n.interval.second and c.second + CN_EPSILON > n.interval.second)
                {
                    nodes[i].interval.second = c.first;
                    changed = true;
                }
                if (changed)
                {
                    i = -1;
                    k = 0;
                }
                i++;
            }
        }
    return nodes;
}
std::vector<Node> SIPP::get_custom_endpoints(Node no)
{
    std::vector<Node> nodes;
    nodes.push_back(no);
    // nodes = {Node(no.id, no.f, no.g, no.i, no.j, nullptr, no.interval.first, no.interval.second)};
    auto colls_it = intersection_intervals.find({no.id, no.type, no.orientation});

    if (colls_it == intersection_intervals.end())
        return nodes;
    else
        for (unsigned int k = 0; k < colls_it->second.size(); k++)
        {
            unsigned int i(0);
            while (i < nodes.size())
            {
                Node n = nodes[i];
                auto c = colls_it->second[k];
                bool changed = false;
                if (c.first - CN_EPSILON < n.interval.first and c.second + CN_EPSILON > n.interval.second)
                {
                    nodes.erase(nodes.begin() + i);
                    changed = true;
                }
                else if (c.first - CN_EPSILON < n.interval.first and c.second > n.interval.first)
                {
                    nodes[i].interval.first = c.second;
                    changed = true;
                }
                else if (c.first - CN_EPSILON > n.interval.first and c.second + CN_EPSILON < n.interval.second)
                {
                    nodes[i].interval.second = c.first;
                    Node temp_node = n;
                    n.interval.first = c.second;
                    nodes.insert(nodes.begin() + i + 1, temp_node);
                    // nodes.insert(nodes.begin() + i + 1, Node(no.id, no.f, no.g, no.i, no.j, nullptr, c.second, n.interval.second));
                    changed = true;
                }
                else if (c.first < n.interval.second and c.second + CN_EPSILON > n.interval.second)
                {
                    nodes[i].interval.second = c.first;
                    changed = true;
                }
                if (changed)
                {
                    i = -1;
                    k = 0;
                }
                i++;
            }
        }
    if (!nodes.empty() and nodes.front().interval.first > no.interval.first)
    {
        Node first_node = no;
        first_node.interval.second = no.interval.first;
        nodes.insert(nodes.begin(), first_node);
    }
    return nodes;
}

double SIPP::check_endpoint(Node start, Node goal)
{
    double cost = sqrt(pow(start.i - goal.i, 2) + pow(start.j - goal.j, 2));
    if (start.g + cost < goal.interval.first)
        start.g = goal.interval.first - cost;
    if (constraints.count({start.id, goal.id}) != 0)
    {
        auto it = constraints.find({start.id, goal.id});
        for (unsigned int i = 0; i < it->second.size(); i++)
            if (start.g + CN_EPSILON > it->second[i].t1 and start.g < it->second[i].t2)
                start.g = it->second[i].t2;
    }
    if (start.g > start.interval.second or start.g + cost > goal.interval.second)
        return CN_INFINITY;
    else
        return start.g + cost;
}

Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values)
{
    this->clear();
    this->agent = agent;
    make_constraints(cons);
    unsigned int loop = CN_LOOP;
    std::vector<Node> starts, goals;
    std::vector<Path> parts, results, new_results;
    Path part, result;
    int expanded(0);
    if (loop > 0)
    {
        for (unsigned int l = 0; l <= 2 * loop - 1; l++)
        {
            if (l == 0)
            {
                starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
                goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY)};
            }
            else
            {
                starts.clear();
                goals.clear();
                for (auto r : results)
                    starts.push_back(r.nodes.back());
                if (l == 2 * loop - 1)
                    goals = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).back()};
                else if (l % 2 == 1)
                    goals = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY)};
                else
                    goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY)};
            }
            if (goals.empty())
                return Path();
            std::vector<Path> subPath = find_partial_path(starts, goals, map, h_values, goals.back().interval.second);
            for (unsigned int i = 0; i < subPath.size(); i++)
                for (unsigned int j = 0; j < subPath[i].nodes.size(); j++)
                    subPath[i].nodes[j].label = l;
            expanded += int(close.size());
            new_results.clear();

            if (l == 0)
            {
                for (unsigned int p = 0; p < subPath.size(); p++)
                {
                    if (subPath[p].nodes.empty())
                        continue;
                    new_results.push_back(subPath[p]);
                }
            }
            else
            {
                for (unsigned int p = 0; p < subPath.size(); p++)
                    for (unsigned int r = 0; r < results.size(); r++)
                    {
                        if (subPath[p].nodes.empty())
                            continue;
                        if (fabs(subPath[p].nodes[0].interval.first - results[r].nodes.back().interval.first) < CN_EPSILON and fabs(subPath[p].nodes[0].interval.second - results[r].nodes.back().interval.second) < CN_EPSILON)
                        {
                            new_results.push_back(results[r]);
                            new_results.back() = add_part(new_results.back(), subPath[p]);
                        }
                    }
            }
            results = new_results;
            if (results.empty())
                return Path();
        }
        result = results[0];
        result.nodes.shrink_to_fit();
        result.cost = result.nodes.back().g;
        result.agentID = agent.id;
        result.expanded = expanded;
        return result;
    }
    if (!landmarks.empty())
    {
        for (unsigned int i = 0; i <= landmarks.size(); i++)
        {
            if (i == 0)
            {
                starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
                goals = get_endpoints(landmarks[i].id1, map.get_i(landmarks[i].id1), map.get_j(landmarks[i].id1), landmarks[i].t1, landmarks[i].t2);
            }
            else
            {
                starts.clear();
                for (auto p : results)
                    starts.push_back(p.nodes.back());
                if (i == landmarks.size())
                    goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY).back()};
                else
                    goals = get_endpoints(landmarks[i].id1, map.get_i(landmarks[i].id1), map.get_j(landmarks[i].id1), landmarks[i].t1, landmarks[i].t2);
            }
            if (goals.empty())
                return Path();
            parts = find_partial_path(starts, goals, map, h_values, goals.back().interval.second);
            expanded += int(close.size());
            new_results.clear();
            if (i == 0)
                for (unsigned int k = 0; k < parts.size(); k++)
                {
                    if (parts[k].nodes.empty())
                        continue;
                    new_results.push_back(parts[k]);
                }
            for (unsigned int k = 0; k < parts.size(); k++)
                for (unsigned int j = 0; j < results.size(); j++)
                {
                    if (parts[k].nodes.empty())
                        continue;
                    if (fabs(parts[k].nodes[0].interval.first - results[j].nodes.back().interval.first) < CN_EPSILON and fabs(parts[k].nodes[0].interval.second - results[j].nodes.back().interval.second) < CN_EPSILON)
                    {
                        new_results.push_back(results[j]);
                        new_results.back() = add_part(new_results.back(), parts[k]);
                    }
                }
            results = new_results;
            if (results.empty())
                return Path();
            if (i < landmarks.size())
            {
                starts.clear();
                for (auto p : results)
                    starts.push_back(p.nodes.back());
                double offset = sqrt(pow(map.get_i(landmarks[i].id1) - map.get_i(landmarks[i].id2), 2) + pow(map.get_j(landmarks[i].id1) - map.get_j(landmarks[i].id2), 2));
                goals = get_endpoints(landmarks[i].id2, map.get_i(landmarks[i].id2), map.get_j(landmarks[i].id2), landmarks[i].t1 + offset, landmarks[i].t2 + offset);
                if (goals.empty())
                    return Path();
                new_results.clear();
                for (unsigned int k = 0; k < goals.size(); k++)
                {
                    double best_g(CN_INFINITY);
                    int best_start_id = -1;
                    for (unsigned int j = 0; j < starts.size(); j++)
                    {
                        double g = check_endpoint(starts[j], goals[k]);
                        if (g < best_g)
                        {
                            best_start_id = j;
                            best_g = g;
                        }
                    }
                    if (best_start_id >= 0)
                    {
                        goals[k].g = best_g;
                        if (collision_intervals[goals[k].id].empty())
                            goals[k].interval.second = CN_INFINITY;
                        else
                        {
                            for (auto c : collision_intervals[goals[k].id])
                                if (goals[k].g < c.first)
                                {
                                    goals[k].interval.second = c.first;
                                    break;
                                }
                        }
                        new_results.push_back(results[best_start_id]);
                        if (goals[k].g - starts[best_start_id].g > offset + CN_EPSILON)
                        {
                            new_results.back().nodes.push_back(new_results.back().nodes.back());
                            new_results.back().nodes.back().g = goals[k].g - offset;
                        }
                        new_results.back().nodes.push_back(goals[k]);
                    }
                }

                results = new_results;
                if (results.empty())
                    return Path();
            }
        }
        result = results[0];
    }
    else
    {
        starts = {get_endpoints(agent.start_id, agent.start_i, agent.start_j, 0, CN_INFINITY).at(0)};
        goals = {get_endpoints(agent.goal_id, agent.goal_i, agent.goal_j, 0, CN_INFINITY).back()};
        parts = find_partial_path(starts, goals, map, h_values);
        expanded = int(close.size());
        if (parts[0].cost < 0)
            return Path();
        result = parts[0];
    }
    result.nodes.shrink_to_fit();
    result.cost = result.nodes.back().g;
    result.agentID = agent.id;
    result.expanded = expanded;
    return result;
}

Path SIPP::update_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values)
{
    this->clear();
    this->obs_info = map.get_obstacle_info();
    this->agent = agent;
    update_constraints(cons);
    std::vector<Node> starts, goals;
    std::vector<Path> paths, results;
    Path result;
    int expanded(0);
    starts.push_back(agent.start_node);
    goals = agent.goal_nodes;
    paths = update_partial_path(starts, goals, map, h_values);
    expanded = int(label_close.size());
    int min = 0;
    for (unsigned int i = 0; i < paths.size(); i++)
    {
        if (paths[i].cost != -1 and paths[i].cost < paths[min].cost)
            min = i;
    }
    result = paths[min];
    result.nodes.shrink_to_fit();
    result.cost = result.nodes.back().g;
    result.agentID = agent.id;
    result.expanded = expanded;
    return result;
}
Path SIPP::replan_initial_path(int agent_id, PDTask &task, const Map &map, Heuristic &h_values, const sPath &prev_path, double current_time)
{
    std::cout << "\nreplan_initial_path for agent[" << agent_id << "]" << std::endl;
    this->obs_info = map.get_obstacle_info();
    int cur_label = 0;
    Path initial_path;
    std::vector<Path> partial;
    agent = task.get_agent(agent_id);
    initial_path.agentID = agent_id;
    initial_path.expanded = 0;

    // Crop remaing path from prev_path
    int start = -1;
    std::cout << "prev_path.nodes.size(): " << prev_path.nodes.size() << std::endl;
    for (unsigned int i = 0; i < prev_path.nodes.size() - 1; i++)
    {
        if (prev_path.nodes[i].g <= current_time and prev_path.nodes[i + 1].g > current_time and !agent.new_goal_assigned)
        {
            start = i;
            break;
        }
    }

    if (start < 0) // already finished
    {
        if (agent.new_goal_assigned)
        {
            std::vector<Node> starts, goals;
            starts.push_back(agent.start_node);
            goals = agent.goal_nodes;
            std::cout << "agent [" << agent_id << "] is start at id: " << agent.start_node.id << ", t: " << agent.start_node.g << std::endl;
            if (goals.empty())
            {
                std::cout << "Agent[" << agent_id << "] has no goal. replan_initial_path-->new_goal_assigned" << std::endl;
                std::cout << "Agent[" << agent_id << "] task list: ";
                for (auto t : agent.task_list)
                {
                    auto [t_id, p_id, d_id, t_status, t_interval] = t;
                    std::cout << "t_id: " << t_id << ", p_id: " << p_id << ", d_id: " << d_id << ", t_status: " << t_status << ", t_interval: [" << t_interval.first << ", " << t_interval.second << "]\n";
                }
                std::cout << std::endl;
            }
            auto paths = replan_partial_path(starts, goals, map, h_values);
            if (!paths[0].nodes.empty())
            {
                initial_path.nodes = paths[0].nodes;
                initial_path.cost = paths[0].nodes.back().g;
                agent.new_goal_assigned = false;
                task.update_agent_info(agent_id, agent);
            }
            else
                std::cout << "we fail to find path (empty path). replan_initial_path-->new_goal_assigned" << std::endl;
            return initial_path;
        }
        auto goal = prev_path.nodes.back();
        Node last(goal.id, 0.0, 0.0, goal.i, goal.j, nullptr, 0, CN_INFINITY);
        // last.label = goal.label;
        // last.type = goal.type;
        // last.orientation = goal.orientation;
        last.label = 0;
        last.type = 0;
        last.orientation = 0;
        agent.start_node = last;
        agent.goal_nodes.clear();
        agent.new_goal_assigned = false;
        task.update_agent_info(agent_id, agent);

        initial_path.nodes.push_back(last);
        initial_path.nodes.push_back(last); // at least two nodes are required to represent empty path
        initial_path.cost = 0;
        return initial_path;
    }
    else
    {
        // initial edge
        auto cur = prev_path.nodes[start];
        auto next = prev_path.nodes[start + 1];

        // update remaining goal_nodes and find removed parts from prev_goals
        std::vector<Node> starts, goals;
        auto prev_goals = agent.goal_nodes; // we have to integrate multi-label concepts
        bool newly_assigned = false;
        for (auto pg : prev_goals)
        {
            double prev_g = -1;
            for (auto prev : prev_path.nodes)
            {
                if (prev.id == pg.id and prev.g >= current_time)
                {
                    prev_g = prev.g;
                    break;
                }
                if (pg.new_assigned)
                {
                    prev_g = 1;
                    newly_assigned = true;
                    break;
                }
            }
            if (prev_g < 0)
            {
                ++cur_label;
                std::cout << "Agent[" << agent_id << "] already passed prev_goal [id, label]->[" << pg.id << "," << pg.label
                          << "], current_time: " << current_time << ", cur.id:" << cur.id << ", cur.label:" << cur.label << std::endl;
                continue;
            }

            Node goal(pg.id, 0.0, 0.0, pg.i, pg.j, nullptr, 0, CN_INFINITY);
            goal.label = pg.label - cur_label;
            goal.type = pg.type;
            goal.orientation = pg.orientation;
            goal.task_id = pg.task_id;
            goal.node_pd_type = pg.node_pd_type;
            goal.new_assigned = false;
            goals.push_back(goal);
        }
        agent.goal_nodes = goals;
        // Node initial_from(cur.id, 0.0, cur.g-current_time, cur.i, cur.j, nullptr, 0, CN_INFINITY);
        Node initial_from(cur.id, 0.0, cur.g - current_time, cur.i, cur.j, nullptr, cur.g - current_time, CN_INFINITY);
        initial_from.label = 0; // replan_path's label starts with label 0
        initial_from.type = cur.type;
        initial_from.orientation = cur.orientation;
        initial_path.nodes.push_back(initial_from);

        initial_path.cost = 0;
        if (prev_path.nodes[start].type == 1 and prev_path.nodes[start + 1].type == 2)
        {
            // assign initial orientation
            agent.initial_id = cur.id;
            agent.initial_orientation = next.orientation;
        }

        agent.start_node = initial_path.nodes.back();
        task.update_agent_info(agent_id, agent);

        // check prev path meets an unavoidable obstacle
        for (std::size_t i = start; i < prev_path.nodes.size() - 1; ++i)
        {
            auto c = prev_path.nodes[i];
            auto n = prev_path.nodes[i + 1];
            auto it = obs_info.find({c.id, n.id});
            if (it != obs_info.end() and it->second.obs_type == 1)
            {
                if (initial_path.nodes.size() > 1)
                    initial_path.nodes.erase(initial_path.nodes.begin() + 1, initial_path.nodes.end());
                auto s = initial_path.nodes.back();
                starts.push_back(s);
                partial = replan_partial_path(starts, goals, map, h_values);
                if (!partial[0].nodes.empty())
                {
                    initial_path = add_part(initial_path, partial[0]);
                    initial_path.cost = initial_path.nodes.back().g;
                }
                else
                    std::cout << "we fail to find path (empty path). replan_initial_path-->found a blocked obstacle" << std::endl;
                return initial_path;
            }
        }
        for (unsigned int i = start; i < prev_path.nodes.size() - 1; i++) // follow previous path until the robot meet unforeseen obstacles.
        {
            auto c = prev_path.nodes[i];
            auto n = prev_path.nodes[i + 1];
            auto it = obs_info.find({c.id, n.id});
            if (it != obs_info.end()) // replanning is required to handle unforeseen obstacles
            {
                if (it->second.obs_type == 1 and i != start)
                {
                    // initial_path.nodes.pop_back();
                    if (initial_path.nodes.size() > 1)
                    {
                        initial_path.nodes.erase(initial_path.nodes.begin() + 1, initial_path.nodes.end());
                    }
                }
                auto s = initial_path.nodes.back();
                // agent.initial_orientation = s.orientation;  //20240412 replanning bug fix
                starts.push_back(s);
                partial = replan_partial_path(starts, goals, map, h_values);
                if (!partial[0].nodes.empty())
                {
                    initial_path = add_part(initial_path, partial[0]);
                    initial_path.cost = initial_path.nodes.back().g;
                }
                else
                    std::cout << "we fail to find path (empty path). replan_initial_path-->found an avoidable obstacle" << std::endl;
                return initial_path;
            }
            // Node temp(n.id, 0.0, n.g-current_time, n.i, n.j, nullptr, 0, CN_INFINITY);
            Node temp(n.id, 0.0, n.g - current_time, n.i, n.j, nullptr, cur.g - current_time, CN_INFINITY);
            // temp.label = n.label - cur.label;
            temp.label = n.label - cur_label; // 240424 fix label bug when the replanning is executed right after robot visit a goal
            temp.type = n.type;
            temp.orientation = n.orientation;
            temp.offset = n.offset;
            initial_path.nodes.push_back(temp);
        }
        initial_path.cost = initial_path.nodes.back().g;
        return initial_path;
    }
}

Path SIPP::replan_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values)
{
    this->clear();
    this->agent = agent;
    update_constraints(cons);

    std::vector<Node> starts, goals;
    std::vector<Path> paths, results;
    Path initial, result;
    int expanded(0);

    // starts.push_back(agent.start_node);
    starts = get_custom_endpoints(agent.start_node);
    goals = agent.goal_nodes;
    if (!agent.goal_nodes.empty() and (starts[0].id != goals.back().id or goals.size() > 1))
    {
        paths = replan_partial_path(starts, goals, map, h_values);
    }
    else if (agent.goal_nodes.empty())
    {
        std::cout << "Agent " << agent.id << " generates dummy path (no goals)" << std::endl;
        Path temp_path;
        temp_path.agentID = agent.id;
        temp_path.cost = 0;
        temp_path.nodes.push_back(agent.start_node);
        temp_path.nodes.push_back(agent.start_node);
        paths.push_back(temp_path);
    }
    else
    {
        std::cout << "Agent " << agent.id << " generates dummy path (complete all the tasks)" << std::endl;
        Path temp_path;
        temp_path.agentID = agent.id;
        temp_path.cost = 0;
        temp_path.nodes.push_back(goals.back());
        temp_path.nodes.push_back(goals.back());
        paths.push_back(temp_path);
    }
    expanded = int(label_close.size());

    int min = 0;
    if (paths[0].nodes.empty())
    {
        std::cout << "we fail to find a path. replan_path-->result" << std::endl;
        return Path();
    }
    for (unsigned int i = 0; i < paths.size(); i++)
    {
        if (paths[i].cost != -1 and paths[i].cost < paths[min].cost)
            min = i;
    }
    result = paths[min];
    result.nodes.shrink_to_fit();
    result.cost = result.nodes.back().g;
    result.agentID = agent.id;
    result.expanded = expanded;
    return result;
}
