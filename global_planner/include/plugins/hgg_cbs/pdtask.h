#ifndef PDTASK_H
#define PDTASK_H

#include "iostream"
#include "string"
#include "algorithm"
#include "sstream"
#include "vector"
#include "yaml-cpp/yaml.h"
#include "structs.h"
#include "const.h"
#include "heuristic.h"
#include "fstream"
#include "map.h"

class PDTask
{
private:
    int task_id;
    std::vector<Agent> agents;
    std::set<int> station_ids;
    std::unordered_map<std::tuple<int, int, int>, double, TupleHash> h_val;
    // std::vector<std::tuple<int, int, int>> unassigned_tasks; //task id, pickup node id, delivery node id
    std::map<int, std::tuple<int, int, double>> unassigned_tasks;                        // task id, pickup node id, delivery node id, generated time
    std::map<int, std::tuple<int, int, int, std::pair<double, double>>> completed_tasks; // task id, agent id, pickup node id, delivery node id, task interval
    std::set<int> dead_agents;                                                           // SJ: agents which cannot move because of obstacles, other agents, communication loss or etc.
public:
    bool set_agent(const Agent a);
    unsigned int get_agents_size() const { return agents.size(); }
    void make_ids(int width);
    void make_ij(const Map &map);
    void get_h_val(const std::unordered_map<std::tuple<int, int, int>, double, TupleHash> &uh_value) { h_val = uh_value; }
    bool assign_additional_tasks(const std::vector<Node_info> &n_info, int seed = 0, int additional_tasks = 1);
    bool online_task_assigner(const std::vector<Node_info> &n_info, std::vector<int> &labels, int capacity = 1, std::vector<double> time_remaining = {});
    bool generate_random_tasks(const std::vector<Node_info> &n_info, int seed = 0, int n_task = -1, double current_time = 0);
    Agent get_agent(int id) const;
    void update_dead_agents(const int id); // SJ: update dead_agents set
    bool check_goals(std::vector<sPath> prev_paths, double current_time, double path_planned_time, double max_delay_time, bool replan_required);
    bool update_task_list(bool replan_required, double current_time);
    void print_task()
    {
        for (auto agent : agents)
            std::cout << "For agent[" << agent.id << "]: " << "start=[" << agent.start_i << ", " << agent.start_j << "] goal=[" << agent.goal_i << ", " << agent.goal_j << "]\n";
    }
    void update_agent_info(int id, Agent info);
    PDTask();
};

#endif // PDTASK_H
