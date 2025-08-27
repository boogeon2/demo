/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef MULTIROBOT_UTILS__DETECTCONFLICT_HPP
#define MULTIROBOT_UTILS__DETECTCONFLICT_HPP

#include "multirobot_utils/Trajectory.hpp"
#include "multirobot_utils/Profile.hpp"
#include "exception"

namespace multirobot_utils
{

  //==============================================================================
  class invalid_trajectory_error : public std::exception
  {
  public:
    const char *what() const noexcept override;

    class Implementation;

  private:
    invalid_trajectory_error();
    multirobot_utils::impl_ptr<Implementation> _pimpl;
  };

  //==============================================================================
  class DetectConflict
  {
  public:
    enum class Interpolate : uint16_t
    {
      CubicSpline
    };

    /// Checks if there are any conflicts between the two trajectories.
    ///
    /// \return true if a conflict exists between the trajectories, false
    /// otherwise.
    static multirobot_utils::optional<multirobot_utils::Time> between(
        const Profile &profile_a,
        const Trajectory &trajectory_a,
        const Profile &profile_b,
        const Trajectory &trajectory_b,
        Interpolate interpolation = Interpolate::CubicSpline);

    static std::vector<multirobot_utils::Time> custom_between( // jw 2022.11.07 declared
        const Profile &profile_a,
        const Trajectory &trajectory_a,
        const Profile &profile_b,
        const Trajectory &trajectory_b,
        Interpolate interpolation = Interpolate::CubicSpline);

    class Implementation;
  };

} // namespace multirobot_utils

#endif // MULTIROBOT_UTILS__DETECTCONFLICT_HPP
