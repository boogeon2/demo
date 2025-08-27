/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "multirobot_utils/Time.hpp"

namespace multirobot_utils
{
  namespace time
  {

    //==============================================================================
    double to_seconds(const Duration delta_t)
    {
      using Sec64 = std::chrono::duration<double>;
      return std::chrono::duration_cast<Sec64>(delta_t).count();
    }

    //==============================================================================
    double to_seconds(const Time t)
    {
      using Sec64 = std::chrono::duration<double>;
      return std::chrono::duration_cast<Sec64>(t.time_since_epoch()).count();
    }

    //==============================================================================
    Duration from_seconds(double delta_t)
    {
      using Sec64 = std::chrono::duration<double>;
      return std::chrono::duration_cast<Duration>(Sec64(delta_t));
    }

    //==============================================================================
    Time apply_offset(const Time start_time, const double delta_seconds)
    {
      using Sec64 = std::chrono::duration<double>;
      using NanoInt = std::chrono::steady_clock::duration;
      return start_time + std::chrono::duration_cast<NanoInt>(Sec64(delta_seconds));
    }

    //==============================================================================
    builtin_interfaces::msg::Time convert(Time time)
    {
      return rclcpp::Time(time.time_since_epoch().count(), RCL_ROS_TIME);
    }

    //==============================================================================
    Time convert(builtin_interfaces::msg::Time time)
    {
      return std::chrono::steady_clock::time_point(
          std::chrono::seconds(time.sec) + std::chrono::nanoseconds(time.nanosec));
    }

    //==============================================================================
    rclcpp::Time to_ros2(Time from)
    {
      return rclcpp::Time(from.time_since_epoch().count(), RCL_ROS_TIME);
    }

    //==============================================================================
    Time convert(rclcpp::Time from)
    {
      return std::chrono::steady_clock::time_point(
          std::chrono::nanoseconds(from.nanoseconds()));
    }

    //==============================================================================
    rclcpp::Duration convert(Duration duration)
    {
      return rclcpp::Duration{duration};
    }

    //==============================================================================
    Duration convert(rclcpp::Duration duration)
    {
      return duration.to_chrono<Duration>();
    }

  } // namespace time
} // namespace multirobot_utils
