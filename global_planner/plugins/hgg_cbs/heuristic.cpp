#include "heuristic.h"

void Heuristic::init(unsigned int size, unsigned int agents)
{
    h_values.clear();
    uh_values.clear();
    h_values.resize(size);
    for (unsigned int i = 0; i < size; i++)
        h_values[i].resize(agents, -1);
}

void Heuristic::count(const Map &map, Agent agent)
{
    Node curNode(agent.goal_id, 0, 0, agent.goal_i, agent.goal_j), newNode;
    open.clear();
    open.insert(curNode);
    auto obs_info = map.get_obstacle_info();
    while (!open.empty())
    {
        curNode = find_min();
        h_values[curNode.id][agent.id] = curNode.g;
        std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
        for (auto move : valid_moves)
        {
            newNode.i = move.i;
            newNode.j = move.j;
            newNode.id = move.id;
            newNode.g = curNode.g + dist(curNode, newNode);
            auto obs_it = obs_info.find({newNode.id, curNode.id});
            if (obs_it != obs_info.end())
            {
                if (obs_it->second.obs_type == 1)
                    newNode.g += CN_INFINITY;
                else
                    newNode.g += CN_LANECHANGE * 2;
            }
            if (h_values[newNode.id][agent.id] < 0)
            {
                auto it = open.get<1>().find(newNode.id);
                if (it != open.get<1>().end())
                {
                    if (it->g > newNode.g)
                        open.get<1>().erase(it);
                    else
                        continue;
                }
                open.insert(newNode);
            }
        }
    }
}

void Heuristic::update(const Map &map, const Config &cfg, const std::vector<Node_info> &info, const Heuristic::INTERMOTION &inter_motion, const Heuristic::INNERMOTION &inner_motion)
{
    int node_size = map.get_node_size(); // In one intersection node, orientation(the number of roads size) is no larger than node_size
    const auto obs_info = map.get_obstacle_info();
    double pointTurnTime = M_PI / cfg.rotVel;
    uh_values.clear();

    auto get_reverse_orientation = [&](Node cur, Node next)
    {
        int ori = -1;
        if (cur.type == 1 && info[next.id].type == 0)
        {
            return 0;
        }
        else if (cur.type == 0 || cur.type == 1) // next node is out of the current intersection
        {
            const auto road_info = info[next.id].roads;
            for (int r = 0; r < road_info.size(); ++r)
            {
                if (road_info[r].connected_wp_id == cur.id)
                {
                    return r;
                }
            }
        }
        else if (cur.type == 2) // next node is in the current intersection
        {
            const auto road_info = info[cur.id].roads;
            for (int r = 0; r < road_info.size(); ++r)
            {
                if (road_info[r].connected_wp_id == next.id)
                {
                    return r;
                }
            }
        }
        if (ori == -1)
        {
            std::cout << "get_reverse_orientation has a negative value. error in Heuristic::update()" << std::endl;
        }
        return ori;
    };

    auto get_reverse_cost = [&](Node cur, Node next)
    {
        if (cur.id != next.id)
            return inter_motion.at({next.id, cur.id}).time_cost;
        else
            return inner_motion.at({next.id, next.orientation, cur.orientation}).time_cost;
    };
    auto equal = [&](Node a, Node b)
    { return (a.id == b.id) && (a.type == b.type) && (a.orientation == b.orientation); };
    auto compare = [&](Node a, Node b)
    { return (a.g > b.g); };

    for (int i = 0; i < node_size; ++i)
    {
        std::priority_queue<Node, std::vector<Node>, decltype(compare)> OPEN(compare);
        if (info[i].type != 0)
            continue;
        Node target(info[i].id, 0, 0);
        target.type = 0;
        target.orientation = 0;

        Node curNode, nextNode;

        OPEN.push(target);
        while (!OPEN.empty())
        {
            curNode = OPEN.top();
            OPEN.pop();
            if (uh_values.find({curNode.id, curNode.type * node_size + curNode.orientation, target.id}) != uh_values.end())
                continue;

            if (curNode.id != target.id && curNode.type == 0) // it starts moving backward direction then execute point turn once
                curNode.g += pointTurnTime;
            uh_values.insert({{curNode.id, curNode.type * node_size + curNode.orientation, target.id}, curNode.g});

            std::vector<Node>
                valid_moves = map.get_valid_moves(curNode.id);
            for (auto move : valid_moves)
            {
                if (curNode.type == 2)
                {
                    nextNode.id = curNode.id;
                    nextNode.type = 1;
                    nextNode.orientation = get_reverse_orientation(curNode, move);
                    if (curNode.orientation == nextNode.orientation)
                        continue;
                }
                else
                {
                    if (curNode.type == 1 && move.id != info[curNode.id].roads[curNode.orientation].connected_wp_id)
                        continue;
                    nextNode.id = move.id;
                    nextNode.type = (info[nextNode.id].type == 0) ? 0 : 2;
                    nextNode.orientation = get_reverse_orientation(curNode, move);
                }
                nextNode.g = curNode.g + get_reverse_cost(curNode, nextNode);

                if (nextNode.type == 2 && curNode.type == 1 && obs_info.find({nextNode.id, curNode.id}) != obs_info.end())
                {
                    if (obs_info.find({nextNode.id, curNode.id})->second.obs_type == 1)
                        nextNode.g += 10000;
                }
                if (uh_values.find({nextNode.id, nextNode.type * node_size + nextNode.orientation, target.id}) == uh_values.end())
                {
                    OPEN.push(nextNode);
                }
            }
        }
    }
}

Node Heuristic::find_min()
{
    Node min = *open.begin();
    open.erase(open.begin());
    return min;
}
