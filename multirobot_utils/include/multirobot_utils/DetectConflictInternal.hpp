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

#ifndef SRC__MULTIROBOT_UTILS__DETECTCONFLICTINTERNAL_HPP
#define SRC__MULTIROBOT_UTILS__DETECTCONFLICTINTERNAL_HPP

#include "multirobot_utils/DetectConflict.hpp"

#include "geometry/ShapeInternal.hpp"

#include "multirobot_utils/Profile.hpp"
#include "multirobot_utils/Trajectory.hpp"

#include "unordered_map"

namespace multirobot_utils
{

  class DetectConflict::Implementation
  {
  public:
    struct Conflict
    {
      Trajectory::const_iterator a_it;
      Trajectory::const_iterator b_it;
      Time time;
    };

    using Conflicts = std::vector<Conflict>;

    static multirobot_utils::optional<Time> between(
        const Profile &profile_a,
        const Trajectory &trajectory_a,
        const Profile &profile_b,
        const Trajectory &trajectory_b,
        Interpolate interpolation,
        std::vector<Conflict> *output_conflicts = nullptr);

    static std::vector<Time> custom_between( // jw 2022.11.07 declared
        const Profile &profile_a,
        const Trajectory &trajectory_a,
        const Profile &profile_b,
        const Trajectory &trajectory_b,
        Interpolate interpolation,
        std::vector<Conflict> *output_conflicts = nullptr);
  };

  namespace internal
  {

    //==============================================================================
    struct Spacetime
    {
      const Time *lower_time_bound;
      const Time *upper_time_bound;

      Eigen::Isometry2d pose;
      geometry::ConstFinalShapePtr shape;
    };

    //==============================================================================
    bool detect_conflicts(
        const Profile &profile,
        const Trajectory &trajectory,
        const Spacetime &region,
        DetectConflict::Implementation::Conflicts *output_conflicts = nullptr);

  } // namespace internal

} // namespace multirobot_utils

#endif // SRC__MULTIROBOT_UTILS__DETECTCONFLICTINTERNAL_HPP
