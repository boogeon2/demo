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

#ifndef MULTIROBOT_UTILS__GEOMETRY__SPACE_HPP
#define MULTIROBOT_UTILS__GEOMETRY__SPACE_HPP

#include "multirobot_utils/geometry/Shape.hpp"

#include "Eigen/Geometry"

#include "multirobot_utils/impl_ptr.hpp"

namespace multirobot_utils
{
  namespace geometry
  {

    //==============================================================================
    class Space
    {
    public:
      Space(geometry::ConstFinalShapePtr shape, Eigen::Isometry2d tf);

      const geometry::ConstFinalShapePtr &get_shape() const;
      Space &set_shape(geometry::ConstFinalShapePtr shape);

      const Eigen::Isometry2d &get_pose() const;
      Space &set_pose(Eigen::Isometry2d tf);

      class Implementation;

    private:
      multirobot_utils::impl_ptr<Implementation> _pimpl;
    };

    //==============================================================================
    /// Equality operator for Space objects.
    ///
    /// \param[in] lhs
    ///   A const reference to the left-hand-side of the comparison.
    ///
    /// \param[in] rhs
    ///   A const reference to the right-hand-side of the comparison.
    bool operator==(
        const Space &lhs,
        const Space &rhs);

    //==============================================================================
    /// Non-equality operator for Space objects.
    ///
    /// \param[in] lhs
    ///   A const reference to the left-hand-side of the comparison.
    ///
    /// \param[in] rhs
    ///   A const reference to the right-hand-side of the comparison.
    bool operator!=(
        const Space &lhs,
        const Space &rhs);

  } // namespace geometry
} // namespace multirobot_utils

#endif // MULTIROBOT_UTILS__GEOMETRY__SPACE_HPP
