#pragma once

// C++ Standard Libraries
#include "iostream"
#include "fstream"
#include "string"
#include "vector"
#include "unordered_map"
#include "memory"
#include "random"
#include "queue"
#include "tuple"
#include "thread"
#include "mutex"
#include "regex"

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// External Libraries
#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"

// Internal Libraries
#include "map.h"
#include "cbs.h"
#include "pdtask.h"

// Custom Libraries
#include "multirobot_utils/velocity_profile.hpp"
#include "multirobot_utils/structs.hpp"
#include "multirobot_utils/DetectConflict.hpp"
#include "multirobot_utils/geometry/Circle.hpp"

class Server : public rclcpp::Node
{
public:
  struct Robot
  {
  public:
    // Constructors
    Robot(
        int new_robot_id,
        geometry_msgs::msg::Pose new_pose);

    // ROS2 Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr delayed_time_sub;

    // ROS2 Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub;

    // Logical Variables
    bool is_connected = false;
    bool is_arrived = false;
    bool is_new_robot = false;

    // Character Variables
    std::string robot_name;

    // Numeric Variables
    int robot_id;
    double last_state_update = 0.0;
    double delayed_time = 0.0;

    // ROS2 Variables
    nav_msgs::msg::Path traj;

    // External Library Variables
    Eigen::Vector2d last_goal_location;

    // Custom Variables
    Structs::Pose pose;

    // Functions
    void update_state(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void update_delayed_time(const std_msgs::msg::Float64::SharedPtr msg);
  };

  // ROS2 Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_graph_pub;

  // ROS2 Timer
  rclcpp::TimerBase::SharedPtr server_timer;

  // Logical Variables
  bool is_initialized = false;
  bool replan_required = false;

  // Character Variables
  std::string map_file_name;
  std::string cfg_file_name;
  std::string server_mode;
  std::string info_file;
  std::string guidepath_file;

  // Numeric Variables
  int robot_num;
  int task_seed = 0;
  int max_task_count = 1;
  int task_count = 1;
  int task_per_period = 1;
  int capacity = 1;
  double env_height;
  double map_resolution;
  double agent_size;
  double task_period = 1.0;
  double first_task_gen_time = 0.0;
  double last_plan_update = 0.0;

  // ROS2 Variables
  std::shared_ptr<rclcpp::Node> node;

  // Internal Library Variables
  CBS route_planner;
  Map route_map;
  PDTask tasks;
  multirobot_utils::Time path_planned_time;

  // Custom Variables
  std::unordered_map<int, Robot> robot_list;

  // ROS2 Publisher/Subscriber Functions
  void set_pubs_subs();
  void new_trajectory_requests(const Solution &solution, const std::vector<Eigen::Vector3d> &curr_locations, const std::vector<double> &time_offset);

  // Initialization Functions
  void set_params();
  void set_config();
  void set_map();
  void calc_inner_collision();
  void set_route_planner();
  void init_tasks();
  void init_plan();
  void set_start_location(std::vector<geometry_msgs::msg::PoseStamped> &locations, int r_idx, multirobot_utils::Time t0, Solution solution);

  // Update Functions
  void update_graph();
  void update_plan();
  bool is_all_robots_connected();
  void add_new_robots();

  // Constructors
  Server();

  // Loops
  void server_loop();
};
