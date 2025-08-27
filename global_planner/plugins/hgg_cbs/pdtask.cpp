#include "pdtask.h"
#include "random"
#include "set"

PDTask::PDTask()
{
    agents.clear();
}

bool PDTask::set_agent(Agent a)
{
    agents.push_back(a);
    station_ids.insert(a.start_id);

    Node initial_start(a.start_id, 0, 0, a.start_i, a.start_j, nullptr, 0, CN_INFINITY);
    initial_start.label = 0;
    initial_start.type = 0;
    initial_start.orientation = 0;
    agents[a.id].start_node = initial_start;

    return true;
}

bool PDTask::assign_additional_tasks(const std::vector<Node_info> &n_info, int seed, int additional_tasks)
{
    srand(seed);
    std::set<int> invalid_set;
    for (auto &a : agents)
    {
        invalid_set.insert(a.start_node.id);
        std::cout << "invalid set new element: " << a.start_node.id << std::endl;
    }
    for (auto &a : agents)
    {
        std::cout << "agent " << a.id << std::endl;
        while (int(a.goal_nodes.size()) < additional_tasks)
        {
            int new_goal_id = rand() % n_info.size();
            std::cout << "initial_new_goal: " << new_goal_id << std::endl;
            while (n_info.at(new_goal_id).type != 0 or invalid_set.find(new_goal_id) != invalid_set.end() or new_goal_id == a.goal_nodes.back().id)
            {
                new_goal_id = rand() % n_info.size();
            }
            std::cout << "selected new goal: " << new_goal_id << std::endl;
            Node new_goal(new_goal_id, 0, 0, 0, 0, nullptr, 0, CN_INFINITY);
            new_goal.label = int(a.goal_nodes.size());
            new_goal.type = 0;
            new_goal.orientation = 0;
            a.goal_nodes.push_back(new_goal);
        }
        Node last_goal(a.start_id, 0, 0, a.start_i, a.start_j, nullptr, 0, CN_INFINITY); // last goal
        last_goal.label = int(a.goal_nodes.size());
        last_goal.type = 0;
        last_goal.orientation = 0;
        a.goal_nodes.push_back(last_goal);

        if (int(a.goal_nodes.size()) < 1 + additional_tasks)
            return false;
    }
    for (auto &a : agents)
    {
        std::cout << "For agent[" << a.id << "]'s additional_task info--> start_id: " << a.start_node.id << ", start_type: " << a.start_node.type << ", and goal_id/type/label: ";
        for (const auto g : a.goal_nodes)
        {
            std::cout << g.id << "/" << g.type << "/" << g.label << " ";
        }
        std::cout << std::endl;
    }
    return true;
}

bool PDTask::online_task_assigner(const std::vector<Node_info> &n_info, std::vector<int> &labels, int capacity, std::vector<double> time_remaining)
{
    // Notice: This function is customized for the "test_map (or example_map)" environment.
    //         Specifically, you must change 'agent_order' (task acceptance priority) to use in other environments

    double min_start_time = CN_INFINITY;
    double max_end_time = 0;

    for (auto da : dead_agents)
    {
        if (agents[da].goal_nodes.empty())
            continue;

        for (auto da_task : agents[da].task_list)
        {
            auto [t_id, p_id, d_id, t_status, t_interval] = da_task;
            if (t_status == 0)
            {
                unassigned_tasks[t_id] = {p_id, d_id, t_interval.first};
            }
        }

        agents[da].task_list.clear();
        agents[da].goal_nodes.clear();
    }
    // DONE(SJ): Task reassignment for dead agents
    // TODO(SJ): Task already picked up by dead agents cannot be reassigned to other agents so the task cannot be completed

    bool new_task_assigned = false;
    for (auto a : agents)
    {
        if (a.goal_nodes.empty())
            continue;

        int current_label = 0;
        if (agents.size() == labels.size())
            current_label = labels[a.id];
        for (std::size_t g = 0; g < a.goal_nodes.size(); ++g)
        {
            if (int(g) < current_label)
                continue;
        }
    }
    if (unassigned_tasks.empty())
    {
        return false;
    }
    // DONE(JW): Task assignment based on agent priority -------------------------------------------
    // for(int a=0; a<agents.size(); ++a) // 로봇 우선순위에 따라 모든 로봇에 대해서
    // {
    //     auto start = agents[a].start_node;
    //     if(agents[a].goal_nodes.empty() and dead_agents.find(a) == dead_agents.end()) // SJ: make dead agent not to be assigned new task
    //     // 로봇이 아직 작업을 할당 받지 않았고 움직일 수 없는 로봇이 아닌 경우
    //     {
    //         double min_cost = CN_INFINITY;
    //         int min_id = -1;
    //         for(auto ut: unassigned_tasks)
    //         {
    //             auto t_id = ut.first;
    //             auto p_id = std::get<0>(ut.second);
    //             double temp_cost = h_val.at({start.id, 0, p_id});
    //             if(temp_cost < min_cost)
    //             {
    //                 min_cost = temp_cost;
    //                 min_id = t_id;
    //             }
    //         }

    //         if(min_id != -1)
    //         {
    //             auto p = std::get<0>(unassigned_tasks[min_id]);
    //             auto d = std::get<1>(unassigned_tasks[min_id]);
    //             auto gen_time = std::get<2>(unassigned_tasks[min_id]);
    //             if(n_info[p].type != 0 or n_info[d].type != 0)
    //             {
    //                 std::cout<<"pickup or delivery node is not an endpoint! you should check this task"<<std::endl;
    //                 return false;
    //             }
    //             Node pickup(p, 0, 0, 0, 0, nullptr, 0, CN_INFINITY);
    //             pickup.label = agents[a].goal_nodes.size();
    //             pickup.type = 0;
    //             pickup.orientation = 0;
    //             pickup.task_id = min_id;
    //             pickup.node_pd_type = 1;
    //             agents[a].goal_nodes.push_back(pickup);

    //             Node delivery(d, 0, 0, 0, 0, nullptr, 0, CN_INFINITY);
    //             delivery.label = agents[a].goal_nodes.size();
    //             delivery.type = 0;
    //             delivery.orientation = 0;
    //             delivery.task_id = min_id;
    //             delivery.node_pd_type = 2;
    //             agents[a].goal_nodes.push_back(delivery);

    //             Node dummy(agents[a].start_node.id, 0, 0, 0, 0, nullptr, 0, CN_INFINITY);
    //             dummy.label = agents[a].goal_nodes.size();
    //             dummy.type = 0;
    //             dummy.orientation = 0;
    //             dummy.node_pd_type = 0;
    //             agents[a].goal_nodes.push_back(dummy);

    //             agents[a].new_goal_assigned = true;
    //             agents[a].task_list.push_back({min_id, p, d, 0, {gen_time, -1}});

    //             unassigned_tasks.erase(min_id);
    //             new_task_assigned = true;
    //             std::cout<<"Online Task Assigner for Agent["<<a<<"] new task[" << min_id << "] is assigned (remainings:"<<unassigned_tasks.size()<<")"<<std::endl;
    //             std::cout<<std::endl;
    //         }
    //     }
    // }
    // DONE(JW): Task assignment based on agent priority -------------------------------------------

    // DONE(SJ): Task assignment based on task priority --------------------------------------------
    int not_assigned = 0;
    while (int(unassigned_tasks.size()) > not_assigned)
    {
        auto ut_itr = unassigned_tasks.begin();
        std::advance(ut_itr, not_assigned);
        auto ut = *ut_itr;
        auto t_id = ut.first;
        auto [p_id, d_id, gen_time] = ut.second;

        if (n_info[p_id].type != 0 or n_info[d_id].type != 0)
        {
            std::cout << "pickup or delivery node is not an endpoint! you should check this task" << std::endl;
            return false;
        }

        int min_id = -1;
        double min_cost = std::numeric_limits<double>::max();
        std::vector<Node> min_goals;

        for (size_t id = 0; id < agents.size(); ++id)
        {
            if (int(agents[id].task_list.size()) < capacity and dead_agents.find(id) == dead_agents.end()) // SJ: make dead agent not to be assigned new task
            {
                auto start = agents[id].start_node;

                Node pickup(p_id, 0, 0, 0, 0, nullptr, 0, CN_INFINITY);
                pickup.type = 0;
                pickup.orientation = 0;
                pickup.task_id = t_id;
                pickup.node_pd_type = 1;
                pickup.new_assigned = true;

                Node delivery(d_id, 0, 0, 0, 0, nullptr, 0, CN_INFINITY);
                delivery.type = 0;
                delivery.orientation = 0;
                delivery.task_id = t_id;
                delivery.node_pd_type = 2;
                delivery.new_assigned = true;

                Node dummy(agents[id].start_id, 0, 0, 0, 0, nullptr, 0, CN_INFINITY);
                dummy.type = 0;
                dummy.orientation = 0;
                dummy.node_pd_type = 0;
                dummy.new_assigned = true;

                auto new_goals = agents[id].goal_nodes;
                double min_temp_cost = std::numeric_limits<double>::max();

                if (!agents[id].task_list.empty() and !new_goals.empty())
                {
                    for (size_t p_pos = 0; p_pos < agents[id].goal_nodes.size(); ++p_pos)
                    {
                        double temp_cost = time_remaining[id];
                        auto temp_goals = agents[id].goal_nodes;
                        temp_goals.insert(temp_goals.begin() + p_pos, pickup);

                        for (size_t g = 0; g < temp_goals.size(); ++g)
                        {
                            if (g < temp_goals.size() - 1)
                            {
                                if (h_val.find({temp_goals[g].id, 0, temp_goals[g + 1].id}) != h_val.end())
                                    temp_cost += h_val.at({temp_goals[g].id, 0, temp_goals[g + 1].id});
                                else // if the heuristic value is not found, then the cost is set to infinity
                                    temp_cost = std::numeric_limits<double>::max();
                            }
                            if (temp_cost > min_temp_cost)
                                break;
                        }

                        temp_cost += h_val.at({temp_goals.back().id, 0, d_id});
                        temp_goals.insert(temp_goals.end() - 1, delivery); // insert before the last dummy node

                        for (size_t g = 0; g < temp_goals.size(); ++g)
                        {
                            temp_goals[g].label = g;
                        }

                        if (temp_cost < min_temp_cost)
                        {
                            min_temp_cost = temp_cost;
                            new_goals = temp_goals;
                        }
                    }
                }
                else if (agents[id].task_list.empty() and !new_goals.empty())
                // if the agent has no task and comes back to the start node
                // we must add only the pickup and delivery nodes at the front of the goal nodes
                {
                    if (h_val.find({agents[id].start_node.id, 0, p_id}) != h_val.end())
                        min_temp_cost = h_val.at({agents[id].start_node.id, 0, p_id});
                    else // if the heuristic value is not found, then the cost is set to infinity
                        min_temp_cost = std::numeric_limits<double>::max();

                    pickup.label = 0;
                    new_goals.insert(new_goals.begin(), pickup);

                    min_temp_cost += h_val.at({p_id, 0, d_id});

                    delivery.label = 1;
                    new_goals.insert(new_goals.begin() + 1, delivery);

                    new_goals.back().label = new_goals.size();
                }
                else
                // first task of the agent
                // if (new_goals.empty()) // only uncomment when task + offline + capa 1
                {
                    min_temp_cost = h_val.at({start.id, 0, p_id});
                    pickup.label = new_goals.size();
                    new_goals.push_back(pickup);

                    min_temp_cost += h_val.at({new_goals.back().id, 0, d_id});

                    delivery.label = new_goals.size();
                    new_goals.push_back(delivery);

                    dummy.label = new_goals.size();
                    new_goals.push_back(dummy);
                }

                if (min_temp_cost < min_cost)
                {
                    min_cost = min_temp_cost;
                    min_id = id;
                    min_goals = new_goals;
                }
            }
        }

        if (min_id != -1) // 할당이 된 경우
        {
            agents[min_id].goal_nodes = min_goals;
            agents[min_id].new_goal_assigned = true;
            agents[min_id].task_list.push_back({t_id, p_id, d_id, 0, {gen_time, -1}});

            unassigned_tasks.erase(t_id);
            new_task_assigned = true;
            std::cout << "Online Task Assigner for Agent[" << min_id << "] with capacity [" << agents[min_id].task_list.size() << "/" << capacity << "] new task[" << ut.first << "] is assigned (remainings:" << unassigned_tasks.size() << ")" << std::endl;
            std::cout << "Agent[" << min_id << "]'s task list: " << std::endl;
            for (const auto &task : agents[min_id].task_list)
            {
                auto [task_id, pickup_id, delivery_id, task_status, task_time] = task;
                std::cout << "task id: " << task_id << ", pickup id: " << pickup_id << ", delivery id: " << delivery_id << ", task status: " << task_status << std::endl;
            }
        }
        else
            not_assigned++;
    }
    // DONE(SJ): Task assignment based on task priority --------------------------------------------

    if (new_task_assigned)
    {
        // for(auto& a: agents)
        // {
        //     std::cout<<"For agent["<<a.id<<"]'s additional_task info--> start_id: "<<a.start_node.id<<", start_type: "<<a.start_node.type<<", and goal_id/type/label: ";
        //     for(const auto g: a.goal_nodes)
        //     {
        //         std::cout<<g.id<<"/"<<g.type<<"/"<<g.label<<" ";
        //     }
        //     std::cout<<std::endl;
        // }
        return true;
    }
    return false;
}

bool PDTask::generate_random_tasks(const std::vector<Node_info> &n_info, int seed, int n_tasks, double current_time)
{
    srand(seed); // default seed is zero (constant), so it generates the same sequence of tasks
    if (get_agents_size() < 1)
    {
        std::cout << "Agent size is 0. Can't generate random tasks" << std::endl;
        return false;
    }

    int new_task_size = 0;
    while (new_task_size < n_tasks)
    {
        int pickup, delivery; // random pickup and delivery node id

        std::set<int> invalid_ids;
        std::set<int> valid_ids;

        for (auto n : n_info)
        {
            if (n.type != 0 or station_ids.find(n.id) != station_ids.end())
                invalid_ids.insert(n.id);
        }

        for (auto n : n_info)
        {
            if (invalid_ids.find(n.id) == invalid_ids.end())
                valid_ids.insert(n.id);
        }

        if (valid_ids.size() < 2)
        {
            std::cout << "valid_ids.size() is less than 2. Can't generate random tasks" << std::endl;
            return false;
        }

        auto pickup_idx = valid_ids.begin();
        std::advance(pickup_idx, rand() % valid_ids.size());
        pickup = *pickup_idx;
        valid_ids.erase(pickup);

        auto delivery_idx = valid_ids.begin();
        std::advance(delivery_idx, rand() % valid_ids.size());
        delivery = *delivery_idx;
        // DONE(SJ): Refactor the code to have no while loop for generating random tasks

        // unassigned_tasks.emplace_back(std::make_tuple(task_id, pickup, delivery));
        unassigned_tasks.insert({task_id, {pickup, delivery, current_time}});
        task_id++;
        new_task_size++;
    }
    if (int(unassigned_tasks.size()) < n_tasks)
        return false;

    return true;
}

void PDTask::make_ids(int width)
{
    for (size_t i = 0; i < agents.size(); i++)
    {
        agents[i].start_id = int(agents[i].start_i) * width + int(agents[i].start_j);
        agents[i].goal_id = int(agents[i].goal_i) * width + int(agents[i].goal_j);
        // std::cout<<agents[i].start_i<<" "<<agents[i].start_j<<"  "<<agents[i].goal_i<<" "<<agents[i].goal_j<<"\n";
    }
}

void PDTask::make_ij(const Map &map)
{
    for (unsigned int i = 0; i < agents.size(); i++)
    {
        gNode start = map.get_gNode(agents[i].start_id), goal = map.get_gNode(agents[i].goal_id);
        agents[i].start_i = start.i;
        agents[i].start_j = start.j;
        agents[i].goal_i = goal.i;
        agents[i].goal_j = goal.j;
    }
}

Agent PDTask::get_agent(int id) const
{
    if (id >= 0 and id < int(agents.size()))
        return agents[id];
    else
        return Agent();
}

void PDTask::update_agent_info(int agent_id, Agent agent)
{
    agents[agent_id] = agent;
}

void PDTask::update_dead_agents(const int id)
{
    dead_agents.insert(id);
}

bool PDTask::check_goals(std::vector<sPath> prev_paths, double current_time, double path_planned_time, double max_delay_time, bool replan_required)
{
    std::vector<double> progressed_time(agents.size(), 0.0);
    for (std::size_t i = 0; i < progressed_time.size(); ++i)
    {
        progressed_time[i] = current_time - path_planned_time - max_delay_time;
    }

    for (auto &agent : agents)
    {
        auto current_time = progressed_time[agent.id];

        if (prev_paths.empty())
        {
            continue;
        }

        for (size_t path_idx = 0; path_idx < prev_paths.size(); ++path_idx)
        {
            auto &prev_path = prev_paths[path_idx];
            auto prev_goals = agent.goal_nodes;

            for (auto pg : prev_goals)
            {
                for (size_t i = 1; i < prev_path.nodes.size(); ++i)
                {
                    auto prev = prev_path.nodes[i];
                    if (prev.id == pg.id and prev.g < current_time) // 이전 경로에 있는 goal node인데 이미 지나간 경우 replan이 필요
                    {
                        std::cout << "Replan required because of goal completion." << std::endl;
                        std::cout << "the goal [" << pg.id << "] is already passed." << std::endl;
                        std::cout << "current_time: " << current_time << " is larger than prev.g: " << prev.g << std::endl;
                        return true;
                    }
                }

                if (pg.new_assigned) // 새로운 goal node라면 replan이 필요
                {
                    std::cout << "Replan required because of new goal assignment." << std::endl;
                    std::cout << "what goal? " << pg.id << std::endl;
                    return true;
                }
            }
        }
    }
    return replan_required;
}

bool PDTask::update_task_list(bool replan_required, double current_time)
{
    for (auto &a : agents)
    {
        std::vector<std::tuple<int, int, int, int, std::pair<double, double>>> updated_task_list;

        for (size_t i = 0; i < a.task_list.size(); ++i)
        {
            auto [t_id, p_id, d_id, t_status, t_interval] = a.task_list[i];
            bool p = false;
            bool d = false;

            for (const auto &goal : a.goal_nodes)
            {
                if (goal.task_id == t_id)
                {
                    if (goal.node_pd_type == 1)
                        p = true;
                    if (goal.node_pd_type == 2)
                        d = true;
                    if (p and d)
                        break;
                }
            }

            if (p and d)
            {
                t_status = 0;
                updated_task_list.emplace_back(t_id, p_id, d_id, t_status, t_interval);
            }
            else if (d and t_status == 0)
            {
                t_status = 1;
                updated_task_list.emplace_back(t_id, p_id, d_id, t_status, t_interval);
                replan_required = true;
                std::cout << "Replan required because of pickup completion." << std::endl;
                std::cout << "what task? " << t_id << " what p? " << p_id << " what d? " << d_id << std::endl;
            }
            else if (!p and !d)
            {
                t_interval = {t_interval.first, current_time};
                completed_tasks.insert({t_id, {a.id, p_id, d_id, t_interval}});
                replan_required = true;
                std::cout << "Replan required because of task completion." << std::endl;
                std::cout << "what task? " << t_id << " what p? " << p_id << " what d? " << d_id << std::endl;
            }
            else
            {
                updated_task_list.emplace_back(t_id, p_id, d_id, t_status, t_interval);
            }
        }
        a.task_list = updated_task_list;
    }
    return replan_required;
}
