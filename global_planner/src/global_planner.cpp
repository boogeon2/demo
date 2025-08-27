#include "global_planner/global_planner.hpp"

Server::Robot::Robot(
    int new_robot_id,
    geometry_msgs::msg::Pose new_pose)
{
  robot_id = new_robot_id;
  robot_name = "robot_" + std::to_string(new_robot_id);
  pose = Structs::Pose(new_pose);
  is_connected = false;
}

void Server::Robot::update_state(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->pose.position.x > 0.01 or msg->pose.position.y > 0.01)
  {
    last_state_update = multirobot_utils::time::to_seconds(multirobot_utils::time::convert(msg->header.stamp));
    pose = Structs::Pose(msg->pose);
    if (!is_connected)
      is_connected = true;
  }
}

void Server::Robot::update_delayed_time(const std_msgs::msg::Float64::SharedPtr msg)
{
  delayed_time = msg->data;
}

void Server::set_pubs_subs()
{
  node = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  topology_graph_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/topology_graph",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  server_timer = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&Server::server_loop, this));
}

void Server::add_new_robots()
{
  // 현재 토픽 목록을 가져옴
  auto topics = this->get_topic_names_and_types();
  std::regex pose_regex("/robot_([^/]+)/pose");
  for (const auto &topic : topics)
  {
    std::smatch match;
    if (std::regex_match(topic.first, match, pose_regex))
    {
      std::string robot_name = "robot_" + match[1].str();
      bool already_exists = false;
      for (const auto &robot : robot_list)
      {
        if (robot.second.robot_name == robot_name)
        {
          already_exists = true;
          break;
        }
      }
      if (!already_exists)
      {
        bool nonzero_pose = false;
        Structs::Pose pose;

        if (!nonzero_pose)
        {
          {
            auto temp_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic.first, rclcpp::QoS(1).transient_local().durability_volatile(),
                [](geometry_msgs::msg::PoseStamped::UniquePtr) {});
            rclcpp::WaitSet wait_set;
            wait_set.add_subscription(temp_sub);
            auto result = wait_set.wait(std::chrono::milliseconds(10));
            if (result.kind() != rclcpp::WaitResultKind::Timeout)
            {
              geometry_msgs::msg::PoseStamped msg;
              rclcpp::MessageInfo info;
              if (temp_sub->take(msg, info))
              {
                if (std::fabs(msg.pose.position.x) > 0.01 || std::fabs(msg.pose.position.y) > 0.01)
                {
                  nonzero_pose = true;
                  pose = Structs::Pose(msg.pose);
                }
              }
            }
          }
        }

        if (nonzero_pose)
        {
          std::cout << "New robot [" << robot_name << "] detected with pose [" << pose.position.x() << ", " << pose.position.y() << "]" << std::endl;
          int new_robot_id = robot_list.size();

          geometry_msgs::msg::Pose init_pose;
          init_pose.position.x = pose.position.x();
          init_pose.position.y = pose.position.y();
          init_pose.position.z = 0.0;
          init_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), pose.get_yaw()));

          Robot new_robot(new_robot_id, init_pose);
          new_robot.robot_name = robot_name;
          robot_list.insert(std::make_pair(new_robot_id, new_robot));

          new_robot.delayed_time_sub = node->create_subscription<std_msgs::msg::Float64>(
              "/" + robot_name + "/delayed_time", 100, std::bind(&Robot::update_delayed_time, &robot_list.at(new_robot_id), std::placeholders::_1));

          new_robot.pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
              "/" + robot_name + "/pose", 100, std::bind(&Robot::update_state, &robot_list.at(new_robot_id), std::placeholders::_1));

          new_robot.traj_pub = node->create_publisher<nav_msgs::msg::Path>(
              "/" + robot_name + "/server_traj", 100);

          robot_list.at(new_robot_id).delayed_time_sub = new_robot.delayed_time_sub;
          robot_list.at(new_robot_id).pose_sub = new_robot.pose_sub;
          robot_list.at(new_robot_id).traj_pub = new_robot.traj_pub;

          if (tasks.get_agent(new_robot_id).id == -1)
          {
            auto robot = robot_list.find(new_robot_id);
            Agent agent;
            agent.id = robot->first;
            agent.start_id = route_map.get_id(robot->second.pose.position[0] / map_resolution, robot->second.pose.position[1] / map_resolution);
            agent.start_i = route_map.get_i(agent.start_id);
            agent.start_j = route_map.get_j(agent.start_id);
            tasks.set_agent(agent);
            robot->second.is_new_robot = true;
          }
        }
      }
    }
  }
}

void Server::new_trajectory_requests(const Solution &solution, const std::vector<Eigen::Vector3d> &curr_locations, const std::vector<double> &time_offset)
{
  std::vector<nav_msgs::msg::Path> requests;
  multirobot_utils::Time t0 = multirobot_utils::time::convert(node->now());
  path_planned_time = t0;
  auto obs_info = route_map.get_obstacle_info();
  for (std::size_t r_idx = 0; r_idx < solution.paths.size(); r_idx++)
  {
    std::vector<geometry_msgs::msg::PoseStamped> locations;
    geometry_msgs::msg::PoseStamped start_location;
    start_location.header.stamp = multirobot_utils::time::convert(t0);
    start_location.header.frame_id = "map";
    start_location.pose.position.x = curr_locations[r_idx](0);
    start_location.pose.position.y = curr_locations[r_idx](1);
    start_location.pose.position.z = 0.0;
    start_location.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), curr_locations[r_idx](2)));
    locations.push_back(start_location);

    auto curr_path = solution.paths[r_idx].nodes;

    double p_start_from_now = time_offset[r_idx];
    for (std::size_t i = 0; i < curr_path.size() - 1; ++i)
    {
      std::vector<timedPoint> p;
      auto c = curr_path[i];
      auto n = curr_path[i + 1];
      if (c.g < 0 and n.g < 0)
        continue;

      if (c.id == n.id and c.type == n.type)
      {
        if (locations.empty())
          continue;
        geometry_msgs::msg::PoseStamped wait_location = locations.back();
        auto time = t0 + std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(n.g));
        wait_location.header.stamp = multirobot_utils::time::convert(time);
        locations.push_back(wait_location);
        continue;
      }
      else if (c.id == n.id and c.type == 1 and n.type == 2)
      {
        p = route_planner.inner_motion.find({c.id, c.orientation, n.orientation})->second.paths;
      }
      else if (c.id == n.id and c.type == 2 and (n.type == 3 or n.type == 4))
      {
        int next_id = route_planner.node_info[c.id].roads[c.orientation].connected_wp_id;
        p = obs_info.find({c.id, next_id})->second.entry_traj;
      }
      else if (c.id == n.id and c.type == 3 and n.type == 5)
      {
        int next_id = route_planner.node_info[c.id].roads[c.orientation].connected_wp_id;
        p = obs_info.find({c.id, next_id})->second.avoid_traj;
      }
      else if (c.id != n.id and c.type == 5 and n.type == 1)
      {
        int next_id = route_planner.node_info[c.id].roads[c.orientation].connected_wp_id;
        p = obs_info.find({c.id, next_id})->second.exit_traj;
      }
      else if (c.id == n.id and c.type == 4 and n.type == 1)
      {
        int next_id = route_planner.node_info[c.id].roads[c.orientation].connected_wp_id;
        p = obs_info.find({c.id, next_id})->second.uturn_traj;
      }
      else if (c.id != n.id)
      {
        p = route_planner.inter_motion.find({c.id, n.id})->second.paths;
      }

      if (c.type == 0 and c.orientation == 0)
        p.emplace_back(p.back().position(0), p.back().position(1), p.back().t + M_PI / route_planner.config.rotVel);

      for (std::size_t j = p.size()-1; j < p.size(); ++j) //여기 바꿈 bg //waypoint 출력 개수 조절 가능 p.size() --> 2
      {
        if (p[j].t + c.g < p_start_from_now)
          continue;
        double temp_time = p[j].t + c.g;
        if (temp_time < 0)
          continue;
        geometry_msgs::msg::PoseStamped location;
        auto time = t0 + std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(temp_time));
        location.header.stamp = multirobot_utils::time::convert(time);
        location.pose.position.x = p[j].position(0);
        location.pose.position.y = p[j].position(1);
        location.pose.position.z = 0.0;

        if (c.type == 0 and c.orientation == 0)
        {
          location.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), route_planner.node_info[c.id].entry_angle));
          if (j == p.size() - 1)
            location.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), Structs::Pose::get_yaw(locations.back().pose.orientation) + M_PI));
        }
        else
          // location.pose.orientation = Eigen::Vector3d(cos(atan2(p[j].position(1)-p[j-1].position(1), p[j].position(0) - p[j-1].position(0))), sin(atan2(p[j].position(1)-p[j-1].position(1), p[j].position(0) - p[j-1].position(0))), 0.0);
          location.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), atan2(p[j].position(1) - p[j - 1].position(1), p[j].position(0) - p[j - 1].position(0))));
        locations.push_back(location);
      }
    }
    nav_msgs::msg::Path path_request;
    path_request.header.stamp = multirobot_utils::time::convert(t0);
    path_request.header.frame_id = "map";
    path_request.poses.clear();
    for (auto l : locations)
    {
      path_request.poses.push_back(l);
    }
    requests.push_back(path_request);
  }

  for (auto robot : robot_list)
  {
    robot.second.last_goal_location = {requests[robot.second.robot_id].poses.back().pose.position.x, requests[robot.second.robot_id].poses.back().pose.position.y};
    // robot.second.delayed_time = 0.0;
    robot.second.traj_pub->publish(requests[robot.second.robot_id]);
  }
}

void Server::set_params()
{
  declare_parameter("robot_num", robot_num);
  declare_parameter("server_mode", server_mode);
  declare_parameter("cfg_file_name", cfg_file_name);
  declare_parameter("map_file_name", map_file_name);
  declare_parameter("graph_info_file", info_file);
  declare_parameter("guide_path_file", guidepath_file);
  declare_parameter("env_height", env_height);
  declare_parameter("map_resolution", map_resolution);
  declare_parameter("task_seed", task_seed);
  declare_parameter("max_task_count", max_task_count);
  declare_parameter("task_per_period", task_per_period);
  declare_parameter("task_period", task_period);
  declare_parameter("capacity", capacity);
  declare_parameter("agent_size", agent_size);

  get_parameter_or("robot_num", robot_num, 1);
  get_parameter_or("server_mode", server_mode, std::string("hgg_cbs"));
  get_parameter("cfg_file_name", cfg_file_name);
  get_parameter("map_file_name", map_file_name);
  get_parameter("graph_info_file", info_file);
  get_parameter("guide_path_file", guidepath_file);
  get_parameter("env_height", env_height);
  get_parameter("map_resolution", map_resolution);
  get_parameter_or("task_seed", task_seed, 0);
  get_parameter_or("max_task_count", max_task_count, 1);
  get_parameter_or("task_per_period", task_per_period, 1);
  get_parameter_or("task_period", task_period, 1.0);
  get_parameter_or("capacity", capacity, 1);
  get_parameter_or("agent_size", agent_size, 0.5);
}

void Server::set_config()
{
  route_planner.config = Config();
  route_planner.config.getConfig(cfg_file_name.c_str());
}

void Server::set_map()
{
  if (env_height < 0.01)
  {
    std::cout << "Environment height is not set. Please set the environment height." << std::endl;
    std::exit(1);
  }
  route_map = Map(route_planner.config.agent_size);
  route_map.get_map(map_file_name);
  route_planner.map = &route_map;
}

void Server::calc_inner_collision()
{
  const YAML::Node info_config = YAML::LoadFile(info_file);
  route_planner.parse_node_info(info_config);

  const YAML::Node guidepath_config = YAML::LoadFile(guidepath_file);
  route_planner.parse_guide_motion(guidepath_config);

  const YAML::Node inner_motions = guidepath_config["inner_motion_paths"];

  Eigen::Vector3d dummy_velocity(0.0, 0.0, 0.0);
  auto profile_a = multirobot_utils::Profile{
      multirobot_utils::geometry::make_final_convex<multirobot_utils::geometry::Circle>(agent_size),
      multirobot_utils::geometry::make_final_convex<multirobot_utils::geometry::Circle>(agent_size)};

  auto profile_b = multirobot_utils::Profile{
      multirobot_utils::geometry::make_final_convex<multirobot_utils::geometry::Circle>(agent_size),
      multirobot_utils::geometry::make_final_convex<multirobot_utils::geometry::Circle>(agent_size)};

  std::mutex interval_mutex;
  std::atomic<int> checked_motions(0);

  auto worker = [&](size_t start_index, size_t end_index)
  {
    for (size_t i = start_index; i < end_index; ++i)
    {
      const auto &m_a = inner_motions[i];
      int wp_a = m_a["waypoint"].as<int>();
      int start_a = m_a["start_road"].as<int>();
      int end_a = m_a["end_road"].as<int>();

      if (wp_a < int(route_planner.node_info.size()) &&
          route_planner.node_info[wp_a].roads.size() > 15)
      {
        int start_wp_a = route_planner.node_info[wp_a].roads[start_a].connected_wp_id;
        int end_wp_a = route_planner.node_info[wp_a].roads[end_a].connected_wp_id;
        if (route_planner.node_info[start_wp_a].type == 0 &&
            route_planner.node_info[end_wp_a].type == 0)
          continue;
      }
      for (size_t j = 0; j < inner_motions.size(); ++j)
      {
        const auto &m_b = inner_motions[j];
        if (m_a["waypoint"].as<int>() == m_b["waypoint"].as<int>())
        {
          int start_b = m_b["start_road"].as<int>();
          int end_b = m_b["end_road"].as<int>();
          if (wp_a < int(route_planner.node_info.size()) &&
              route_planner.node_info[wp_a].roads.size() > 15)
          {
            int start_wp_b = route_planner.node_info[wp_a].roads[start_b].connected_wp_id;
            int end_wp_b = route_planner.node_info[wp_a].roads[end_b].connected_wp_id;
            if (route_planner.node_info[start_wp_b].type == 0 &&
                route_planner.node_info[end_wp_b].type == 0)
              continue;
          }

          std::tuple<int, int, int, int, int> key1{
              m_a["waypoint"].as<int>(),
              m_a["start_road"].as<int>(),
              m_a["end_road"].as<int>(),
              m_b["start_road"].as<int>(),
              m_b["end_road"].as<int>()};
          std::tuple<int, int, int, int, int> key2{
              m_b["waypoint"].as<int>(),
              m_b["start_road"].as<int>(),
              m_b["end_road"].as<int>(),
              m_a["start_road"].as<int>(),
              m_a["end_road"].as<int>()};

          {
            std::lock_guard<std::mutex> lock(interval_mutex);
            if (route_planner.inner_collision_interval.find(key1) != route_planner.inner_collision_interval.end())
              continue;
            if (route_planner.inner_collision_interval.find(key2) != route_planner.inner_collision_interval.end())
            {
              std::vector<std::pair<double, double>> symmetric_interval;
              for (const auto &interval : route_planner.inner_collision_interval[key2])
              {
                symmetric_interval.emplace_back(-interval.second, -interval.first);
              }
              route_planner.inner_collision_interval[key1] = symmetric_interval;
              continue;
            }
          }

          std::vector<Eigen::Vector3d> position_a, position_b;
          multirobot_utils::Trajectory trajectory_a, trajectory_b;
          auto p_a = m_a["paths"];
          multirobot_utils::Time t_0;
          auto curr_t_a = t_0 + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::duration<double>(p_a[0][2].as<double>()));
          position_a.emplace_back(p_a[0][0].as<double>(), p_a[0][1].as<double>(), 0.0);
          trajectory_a.insert(t_0, position_a.back(), dummy_velocity);
          std::size_t size_a = p_a.size();
          for (std::size_t a = 1; a < size_a; a++)
          {
            curr_t_a = t_0 + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::duration<double>(p_a[a][2].as<double>()));
            position_a.emplace_back(p_a[a][0].as<double>(), p_a[a][1].as<double>(), 0.0);
            trajectory_a.insert(curr_t_a, position_a.back(), dummy_velocity);
          }
          auto p_b = m_b["paths"];
          auto curr_t_b = t_0 + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::duration<double>(p_b[0][2].as<double>()));
          position_b.emplace_back(p_b[0][0].as<double>(), p_b[0][1].as<double>(), 0.0);
          trajectory_b.insert(curr_t_b, position_b.back(), dummy_velocity);
          std::size_t size_b = p_b.size();
          for (std::size_t b = 1; b < size_b; b++)
          {
            curr_t_b = t_0 + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::duration<double>(p_b[b][2].as<double>()));
            position_b.emplace_back(p_b[b][0].as<double>(), p_b[b][1].as<double>(), 0.0);
            trajectory_b.insert(curr_t_b, position_b.back(), dummy_velocity);
          }
          double precision = 0.1;
          double max_offset = std::max(
              trajectory_a.finish_time()->time_since_epoch().count() / 1e9,
              trajectory_b.finish_time()->time_since_epoch().count() / 1e9);
          std::vector<std::pair<double, double>> unsafe_interval;
          std::size_t num_iter = 2 * floor(max_offset / precision);
          bool isSafe = true;
          double t1 = -1e10;
          double t2 = -1e10;
          auto temp_traj_a = trajectory_a;
          temp_traj_a.begin()->adjust_times(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(max_offset)));
          for (std::size_t n_iter = 0; n_iter <= num_iter; n_iter++)
          {
            auto temp_traj_b = trajectory_b;
            temp_traj_b.begin()->adjust_times(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(n_iter * precision)));
            auto c_time = multirobot_utils::DetectConflict::between(profile_a, temp_traj_a, profile_b, temp_traj_b);
            if (c_time.has_value())
              isSafe = false;
            else
              isSafe = true;
            if (!isSafe && t1 == -1e10)
              t1 = -max_offset + n_iter * precision;
            if (isSafe && t1 != -1e10 && t2 == -1e10)
            {
              t2 = -max_offset + (n_iter - 1) * precision;
              unsafe_interval.push_back({t1, t2});
              t1 = -1e10;
              t2 = -1e10;
            }
          }
          if (t1 != -1e10 && t2 == 1e-10 && !isSafe)
            unsafe_interval.push_back({t1, max_offset});
          {
            std::lock_guard<std::mutex> lock(interval_mutex);
            route_planner.inner_collision_interval[key1] = unsafe_interval;
          }
        }
      }
    }
  };

  unsigned int num_threads = std::thread::hardware_concurrency();
  if (num_threads == 0)
    num_threads = 4;
  std::vector<std::thread> threads;
  size_t total = inner_motions.size();
  size_t chunk_size = (total + num_threads - 1) / num_threads;
  for (unsigned int i = 0; i < num_threads; i++)
  {
    size_t start_index = i * chunk_size;
    size_t end_index = std::min(total, (i + 1) * chunk_size);
    threads.emplace_back(worker, start_index, end_index);
  }
  for (auto &t : threads)
    t.join();
}

void Server::set_route_planner()
{
  set_config();
  set_map();
  calc_inner_collision();
}

void Server::init_tasks()
{
  for (auto &robot : robot_list)
  {
    if (tasks.get_agent(robot.first).id != -1)
      continue;
    Agent agent;
    agent.id = robot.first;
    agent.start_id = route_map.get_id(robot.second.pose.position[0] / map_resolution, robot.second.pose.position[1] / map_resolution);
    agent.start_i = route_map.get_i(agent.start_id);
    agent.start_j = route_map.get_j(agent.start_id);
    std::cout << "Agent[" << agent.id << "] is start at id: " << agent.start_id << std::endl;
    std::cout << "i: " << agent.start_i << ", j: " << agent.start_j << std::endl;
    tasks.set_agent(agent);
  }
  if (task_count < max_task_count + 1)
  {
    first_task_gen_time = node->now().nanoseconds() / 1e9;
    tasks.generate_random_tasks(route_planner.node_info, task_seed + task_count, task_per_period, first_task_gen_time);
    task_count++;
  }
}

void Server::init_plan()
{
  init_tasks();
  Solution solution = route_planner.update_solution(route_map, tasks, route_planner.config, capacity);

  std::vector<nav_msgs::msg::Path> requests;
  multirobot_utils::Time t0 = multirobot_utils::time::convert(node->now() + solution.time);
  path_planned_time = t0;
  for (std::size_t r_idx = 0; r_idx < solution.paths.size(); r_idx++)
  {
    if (solution.paths[r_idx].nodes.empty())
      continue;

    std::vector<geometry_msgs::msg::PoseStamped> locations;
    set_start_location(locations, r_idx, t0, solution);

    auto curr_path = solution.paths[r_idx].nodes;
    for (std::size_t i = 0; i < curr_path.size() - 1; i++)
    {
      std::vector<timedPoint> p;
      auto c = curr_path[i];
      auto n = curr_path[i + 1];

      if (c.id == n.id and c.type == n.type)
      {
        if (locations.empty())
          continue;
        geometry_msgs::msg::PoseStamped wait_location;
        wait_location = locations.back();
        auto time = t0 + std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(n.g));
        wait_location.header.stamp = multirobot_utils::time::convert(time);
        locations.push_back(wait_location);
        continue;
      }
      else if (c.id == n.id and c.type == 1 and n.type == 2)
        p = route_planner.inner_motion.find({c.id, c.orientation, n.orientation})->second.paths;
      else if (c.id != n.id)
        p = route_planner.inter_motion.find({c.id, n.id})->second.paths;

      if (c.type == 0 and c.orientation == 0)
        p.emplace_back(p.back().position[0], p.back().position[1], p.back().t + M_PI / route_planner.config.rotVel);
      for (std::size_t j = 1; j < p.size(); j++)
      {
        geometry_msgs::msg::PoseStamped location;
        auto time = t0 + std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(p[j].t + c.g));
        location.header.stamp = multirobot_utils::time::convert(time);
        location.pose.position.x = p[j].position[0];
        location.pose.position.y = p[j].position[1];
        if (c.type == 0 and c.orientation == 0)
        {
          auto yaw = route_planner.node_info[c.id].entry_angle;
          location.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
          if (j == p.size() - 1)
          {
            tf2::Quaternion q;
            tf2::fromMsg(locations.back().pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            location.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw + M_PI));
          }
        }
        else
        {
          auto yaw = atan2(p[j].position[1] - p[j - 1].position[1], p[j].position[0] - p[j - 1].position[0]);
          location.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
        }
        locations.push_back(location);
      }
    }
    nav_msgs::msg::Path path_request;
    path_request.header.stamp = multirobot_utils::time::convert(t0);
    path_request.header.frame_id = "map";
    path_request.poses = locations;
    requests.push_back(path_request);
  }
  // route_planner.generate_random_obstacles(route_map, 10);
  // visualize_obstacles(route_map);
  for (std::size_t r = 0; r < requests.size(); r++)
  {
    robot_list.at(r).traj_pub->publish(requests[r]);
  }
  for (auto &robot : robot_list)
  {
    robot.second.traj = requests[robot.second.robot_id];
  }
  is_initialized = true;
  last_plan_update = node->now().nanoseconds() / 1e9;
}

void Server::set_start_location(std::vector<geometry_msgs::msg::PoseStamped> &locations, int r_idx, multirobot_utils::Time t0, Solution solution)
{
  geometry_msgs::msg::PoseStamped start_location;

  start_location.pose.position.x = tasks.get_agent(r_idx).start_i * map_resolution;
  start_location.pose.position.y = tasks.get_agent(r_idx).start_j * map_resolution;
  start_location.pose.position.z = 0.0;

  auto yaw = route_planner.node_info[tasks.get_agent(r_idx).start_id].entry_angle;
  start_location.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));

  auto start_time = t0 + std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(solution.paths[r_idx].nodes.front().g));
  start_location.header.stamp = multirobot_utils::time::convert(start_time);
  locations.push_back(start_location);
}

void Server::update_graph()
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker node_marker;
  node_marker.header.frame_id = "map";
  node_marker.header.stamp = node->now();
  node_marker.ns = "nodes";
  node_marker.type = visualization_msgs::msg::Marker::SPHERE;
  node_marker.action = visualization_msgs::msg::Marker::ADD;
  node_marker.pose.position.z = 0.3;
  node_marker.pose.orientation.w = 1.0;
  node_marker.scale.x = 1.0;
  node_marker.scale.y = 1.0;
  node_marker.scale.z = 1.0;
  node_marker.color.r = 0.0;
  node_marker.color.g = 1.0;
  node_marker.color.b = 0.0;
  node_marker.color.a = 1.0;
  node_marker.lifetime = rclcpp::Duration::from_seconds(0);

  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = "map";
  edge_marker.header.stamp = node->now();
  edge_marker.ns = "edges";
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.pose.position.z = 0.3;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.2;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 0.8;
  edge_marker.color.b = 0.8;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = rclcpp::Duration::from_seconds(0);

  visualization_msgs::msg::Marker text_marker;
  text_marker.header.frame_id = "map";
  text_marker.header.stamp = node->now();
  text_marker.ns = "node_ids";
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose.position.z = 1.0;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.z = 1.0;
  text_marker.color.r = 0.0;
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.0;
  text_marker.color.a = 1.0;
  text_marker.lifetime = rclcpp::Duration::from_seconds(0);

  for (size_t i = 0; i < route_map.get_node_size(); i++)
  {
    gNode node = route_map.get_gNode(i);

    node_marker.id = i;
    node_marker.pose.position.x = node.i * map_resolution;
    node_marker.pose.position.y = node.j * map_resolution;
    node_marker.pose.position.z = 0.3;
    marker_array.markers.push_back(node_marker);

    text_marker.id = i;
    text_marker.pose.position.x = node.i * map_resolution;
    text_marker.pose.position.y = node.j * map_resolution;
    text_marker.pose.position.z = 1.0;
    text_marker.text = std::to_string(i);
    marker_array.markers.push_back(text_marker);
  }

  int edge_id = 0;
  for (size_t i = 0; i < route_map.get_edge_size(); i++)
  {
    Edge edge = route_map.get_edge(i);

    gNode entry_node = route_map.get_gNode(edge.entry);
    gNode exit_node = route_map.get_gNode(edge.exit);

    geometry_msgs::msg::Point p1;
    p1.x = entry_node.i * map_resolution;
    p1.y = entry_node.j * map_resolution;
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = exit_node.i * map_resolution;
    p2.y = exit_node.j * map_resolution;
    p2.z = 0.0;

    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }

  if (!edge_marker.points.empty())
  {
    edge_marker.id = edge_id;
    marker_array.markers.push_back(edge_marker);
  }

  topology_graph_pub->publish(marker_array);
}

void Server::update_plan()
{
  for (auto &robot : robot_list)
  {
    if (robot.second.is_new_robot)
    {
      replan_required = true;
      robot.second.is_new_robot = false;
      Agent agent = tasks.get_agent(robot.first);
      route_planner.solution.paths.resize(robot_list.size());
      route_planner.solution.paths[robot.first] = route_planner.planner.replan_path(agent, route_map, {}, route_planner.h_values);
    }
  }

  double current_time = node->now().seconds();
  if (route_planner.solution.paths.empty() or (current_time - last_plan_update) < 0.1)
    return;
  double max_delay_time = 0.0;

  std::vector<double> time_progressed;
  std::vector<int> current_labels;
  std::vector<Eigen::Vector3d> curr_robot_locations;
  time_progressed.resize(robot_list.size());
  current_labels.resize(robot_list.size());
  curr_robot_locations.resize(robot_list.size());

  for (const auto &robot : robot_list)
  {
    int idx = robot.first;
    if (idx >= int(robot_list.size()))
      continue;

    double communication_delay = current_time - robot.second.last_state_update;
    curr_robot_locations.at(idx) = {robot.second.pose.position.x(), robot.second.pose.position.y(), Structs::Pose::get_yaw(robot.second.pose.orientation)};

    if (robot.second.traj.poses.empty())
    {
      double dist2goal = std::hypot(robot.second.pose.position.x() - robot.second.last_goal_location.x(), robot.second.pose.position.y() - robot.second.last_goal_location.y());
      if (dist2goal < route_planner.config.agent_size and robot.second.last_state_update + route_planner.config.agent_size / route_planner.config.trVel > route_planner.solution.paths[idx].nodes.back().g)
      {
        double last_time = route_planner.solution.paths[idx].nodes.back().g;
        time_progressed.at(idx) = last_time;
        auto agent = tasks.get_agent(idx);
        if (agent.goal_nodes.size() == 1)
        {
          std::cout << "agent " << idx << " start node id: " << route_planner.solution.paths[idx].nodes.back().id << std::endl;
          agent.start_node.id = route_planner.solution.paths[idx].nodes.back().id;
          agent.start_node.type = route_planner.solution.paths[idx].nodes.back().type;
          agent.start_node.orientation = route_planner.solution.paths[idx].nodes.back().orientation;
          agent.start_node.g = 0;
          agent.task_list.clear();
          tasks.update_agent_info(idx, agent);
        }
      }
      continue;
    }
    else
    {
      // std::cout << "Replan required: " << replan_required << " before checking goal completion." << std::endl;
      replan_required = tasks.check_goals(route_planner.solution.paths, current_time, path_planned_time.time_since_epoch().count() / 1e9, max_delay_time, replan_required);
      replan_required = tasks.update_task_list(replan_required, current_time);
    }

    double duration = current_time - path_planned_time.time_since_epoch().count() / 1e9;
    auto prev_path = route_planner.solution.paths[idx].nodes;
    const Eigen::Vector3d p_next{robot.second.traj.poses.begin()->pose.position.x, robot.second.traj.poses.begin()->pose.position.y, 0.0};
    const Eigen::Vector3d p_curr{robot.second.pose.position.x(), robot.second.pose.position.y(), 0.0};
    double ang = Structs::Pose::get_yaw(robot.second.traj.poses.begin()->pose.orientation) - Structs::Pose::get_yaw(robot.second.pose.orientation);
    while (ang > M_PI)
      ang -= 2 * M_PI;
    while (ang < -M_PI)
      ang += 2 * M_PI;

    double delayed_time = robot.second.delayed_time - communication_delay;

    if (delayed_time > route_planner.config.safe_time - 0.5)
    {
      replan_required = true;
    }
    auto obstacles = route_map.get_obstacles();
    for (auto &obs : obstacles)
    {
      const Eigen::Vector3d oxy{obs.ox, obs.oy, 0.0};
      double dist = (oxy - p_curr).norm();
      if (!obs.registered and dist < 30.0)
      {
        route_planner.add_obstacle(route_map, obs);
        replan_required = true;
      }
    }

    if (delayed_time < 0)
    {
      delayed_time = 0.0;
    }
    if (max_delay_time < delayed_time)
    {
      max_delay_time = delayed_time;
    }
    time_progressed.at(idx) = duration - delayed_time;

    for (const auto n : route_planner.solution.paths[idx].nodes)
    {
      if (n.g < duration - delayed_time)
      {
        continue;
      }
      const auto agent = tasks.get_agent(idx);
      current_labels.at(idx) = n.label;
      break;
    }
  }

  if (robot_list.empty() or time_progressed.empty())
    return;

  if (current_time - first_task_gen_time > (task_count + 1) * task_period and task_count < max_task_count + 1)
  {
    tasks.generate_random_tasks(route_planner.node_info, task_seed + task_count, task_per_period, node->now().nanoseconds() / 1e9);
    task_count++;
  }
  if (current_time - path_planned_time.time_since_epoch().count() / 1e9 > 1.0)
  {
    std::vector<double> time_remaining = {};
    for (std::size_t idx = 0; idx < route_planner.solution.paths.size(); ++idx)
    {
      double end_time = route_planner.solution.paths[idx].nodes.back().g;
      double t_r = end_time - current_time - path_planned_time.time_since_epoch().count() / 1e9;
      if (route_planner.solution.paths[idx].nodes.size() < 3 and
          route_planner.solution.paths[idx].nodes.back().id == route_planner.solution.paths[idx].nodes.front().id)
      {
        t_r = 0.0;
      }
      time_remaining.push_back(std::max(0.0, t_r));
    }
    if (time_remaining.size() < route_planner.solution.paths.size())
    {
      time_remaining = std::vector<double>(route_planner.solution.paths.size(), 0.0);
    }
    bool new_task_assigned = tasks.online_task_assigner(route_planner.node_info, current_labels, capacity, time_remaining);
    if (new_task_assigned)
    {
      replan_required = true;
    }
  }
  if (replan_required and current_time - path_planned_time.time_since_epoch().count() / 1e9 - route_planner.config.safe_time > 0.5)
  {
    auto time_offset = time_progressed;
    if (max_delay_time < route_planner.config.safe_time + 0.5)
    {
      for (std::size_t i = 0; i < time_progressed.size(); ++i)
      {
        time_progressed[i] = current_time - path_planned_time.time_since_epoch().count() / 1e9 - max_delay_time;
        if (time_offset[i] > time_progressed[i])
          time_offset[i] -= time_progressed[i];
        else
          time_offset[i] = 0.0;
      }
    }
    route_planner.replan_solution(route_map, tasks, route_planner.config, route_planner.solution.paths, time_progressed);
    if (route_planner.solution.found)
    {
      new_trajectory_requests(route_planner.solution, curr_robot_locations, time_offset);
      // visualize_obstacles(route_map);
      replan_required = false;
    }
    else
    {
      std::cout << "We failed to find a solution during replanning.\nTo address this situation, you could add some modules such as 'All stop the robot and perform manual coordination' or 'Keep the robots moving and replan in the next timestep'... etc." << std::endl;
    }

    for (auto agent : route_planner.solution.paths)
    {
      // if (agent.nodes.size() < 3)
      // {
      //   std::cout << "tinyRobot" << (agent.agentID+1) << " has no path" << std::endl;
      //   tasks.update_dead_agents(agent.agentID);
      // }
    }
    // TODO(SJ): Adding dead agent to the task manager is done. But the IF condition should be modified. It is a temporary solution.
  }
  last_plan_update = node->now().nanoseconds() / 1e9;
  // visualize_fleet_state(msg);
}

bool Server::is_all_robots_connected()
{
  if (robot_list.empty())
    return false;
  for (const auto &robot : robot_list)
  {
    if (!robot.second.is_connected)
      return false;
  }
  return true;
}

Server::Server()
    : Node("global_planner")
{
  set_params();
  set_pubs_subs();
  set_route_planner();
}

void Server::server_loop()
{
  update_graph();
  add_new_robots();
  if (is_all_robots_connected())
  {
    if (!is_initialized)
      init_plan();
    else
      update_plan();
  }
}
