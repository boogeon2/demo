#pragma once

// C++ Standard Libraries
#include <memory>
#include <string>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// External Libraries
#include "SDL2/SDL.h"

// Plugin Libraries
#include "keyboard.hpp"
#include "joy.hpp"

class ManualController : public rclcpp::Node
{
public:
  // ROS2 Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  // ROS2 Timer
  rclcpp::TimerBase::SharedPtr controller_timer;

  // Logical Variables
  bool prev_toggle_button_state = false;
  bool prev_help_button_state = false;
  bool joy_move_allowed = false;

  // Numeric Variables
  double lin_vel = 0.0;
  double ang_vel = 0.0;
  double delta_v = 0.1;
  double delta_w = 0.05;
  double joy_scale_linear = 0.5;
  double joy_scale_angular = 0.5;
  double joy_deadzone = 0.05;
  int joy_axis_linear = 1;
  int joy_axis_angular = 0;
  int joy_button_move = 0;
  int joy_button_help = 1;
  int joy_button_toggle = 2;
  int joy_device_index = 2;

  // ROS2 Variables
  geometry_msgs::msg::Twist cmd_vel;

  // External Library Variables
  SDL_Window *window = nullptr;
  SDL_Renderer *renderer = nullptr;
  SDL_Surface *surface = nullptr;
  SDL_Joystick *joystick = nullptr;

  // Plugin objects
  std::unique_ptr<Keyboard> keyboard_plugin;
  std::unique_ptr<Joy> joy_plugin;

  enum InputSource
  {
    KEYBOARD,
    JOYSTICK,
    NONE
  } last_input_source = NONE,
    active_input_source = KEYBOARD;

  // ROS2 Publisher/Subscriber Functions
  void set_pubs_subs();
  void publish_cmd_vel();

  // Initialization Functions
  void set_params();
  void initialize_sdl();
  void initialize_input_devices();

  // Update Functions
  void process_inputs();
  void handle_input_source_toggle(bool joy_toggle_pressed, bool key_x_pressed);
  void print_help();

  // Constructors and Destructor
  ManualController();
  ~ManualController();

  // Loops
  void controller_loop();
};
